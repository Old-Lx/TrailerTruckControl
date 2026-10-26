"""
arnes.py - Arnés de integración SIN BeamNG (verificación del pipeline del Capítulo 5).

¿Para qué sirve?
    Comprueba, sin abrir BeamNG, que todo el flujo funciona de punta a punta: los tres main() reales
    (forward.py, backward.py, backwards_lqr.py), el TruckTrailer real (read_sensors, calibración,
    tiempo de simulación, avanzar), el fin de ruta (fin_ruta.py) y la telemetría (debugging_sys.py).
    Úsalo después de modificar el código, antes de gastar tiempo en corridas reales.

¿Cómo lo hace?
    Reemplaza BeamNG por un simulador falso: un modelo cinemático tractor-remolque con los parámetros
    identificados (L1 = 2.93 m, L2 = 8.08 m, dp = 5.2 m) y el retardo de la dirección (retardo puro de
    0.32 s + primer orden de 0.34 s). Los controladores no saben que no es BeamNG.

IMPORTANTE: los CSV y las figuras que produce son SINTÉTICOS. Sirven para validar el software,
            NO son resultados de la tesis.

Uso (desde una carpeta aparte, porque escribe los CSV en la carpeta actual):
    python ../arx_truck_ai/arnes.py <forward|backward|lqr> <tipo_ruta 0..6> [CONSTANTE=valor ...]
Los .py del proyecto se buscan en la carpeta de este archivo (o en la variable de entorno CAP5_SRC).
"""
import builtins, collections, importlib.util, math, os, sys, time, types
os.environ["MPLBACKEND"] = "Agg"          # matplotlib sin ventanas (no bloquea la ejecución)
import numpy as np

SRC = os.environ.get("CAP5_SRC", os.path.dirname(os.path.abspath(__file__)))   # carpeta con los .py del proyecto
algoritmo, tipo_ruta = sys.argv[1], int(sys.argv[2])

# =====================================================================================
# 1. MÓDULOS FALSOS: beamngpy y dotenv no se necesitan; se registran módulos vacíos para que
#    los "import" del proyecto no fallen. arx_truck_ai se crea como paquete en memoria.
# =====================================================================================
bp = types.ModuleType('beamngpy'); bs = types.ModuleType('beamngpy.sensors')
bp.BeamNGpy = bp.Vehicle = bp.Scenario = object
for n in ['AdvancedIMU', 'Camera', 'Lidar', 'State', 'Timer']:
    setattr(bs, n, object)
sys.modules['beamngpy'] = bp; sys.modules['beamngpy.sensors'] = bs
de = types.ModuleType('dotenv'); de.load_dotenv = lambda *a, **k: None; sys.modules['dotenv'] = de

pkg = types.ModuleType("arx_truck_ai"); pkg.__path__ = []; sys.modules["arx_truck_ai"] = pkg


def _load(nombre, archivo):
    """Carga un .py REAL del proyecto como submódulo de arx_truck_ai (p. ej. arx_truck_ai.forward)."""
    spec = importlib.util.spec_from_file_location(nombre, os.path.join(SRC, archivo))
    m = importlib.util.module_from_spec(spec); sys.modules[nombre] = m
    setattr(pkg, nombre.split('.')[-1], m)
    spec.loader.exec_module(m)
    return m


# truck_trailer.py real; se fija la ruta pedida y se desactivan las ventanas de OpenCV
tt = _load("arx_truck_ai.truck_trailer", "truck_trailer.py")
tt.TIPO_RUTA = tipo_ruta
tt.cv2.imshow = lambda *a, **k: None
tt.cv2.waitKey = lambda *a, **k: None

# =====================================================================================
# 2. PLANTA CINEMÁTICA (sustituye la física de BeamNG)
# =====================================================================================
DT = 1.0 / tt.FRECUENCIA_SIMULACION   # paso de integración = paso de simulación (1/60 s)
L1, L2, DP = 2.93, 8.08, 5.2          # modelo identificado de la telemetría de BeamNG (script de identificación pendiente): batalla, enganche-eje tráiler, eje-punto medido
DELTA_MAX = 0.55                      # rad de rueda con |steering| = 1
TAU, RETARDO = 0.34, 0.32             # dinámica de la dirección: primer orden + retardo puro [s]
OFF_TRUCK = 0.14                      # state['pos'] del camión, adelante del enganche
LAT_TRAILER = -0.949                  # state['pos'] del tráiler, 0.949 m a la DERECHA de su eje (como en BeamNG)


class Planta:
    """Estado del conjunto: enganche H (= eje trasero del tractor), rumbos th1 (tractor) y th2
    (remolque), velocidad v con signo (< 0 en reversa) y ángulo real de las ruedas delta."""

    def __init__(self, direccion):
        s = -1.0 if direccion == 'bwd' else 1.0           # 'bwd': nariz hacia -x (igual que el escenario real)
        self.th1 = self.th2 = 0.0 if s > 0 else math.pi
        self.H = np.array([-OFF_TRUCK * s, 0.0])           # state['pos'] del camión queda en el origen
        self.v = 0.0; self.delta = 0.0; self.t = 0.0
        self.cola = collections.deque([0.0] * int(round(RETARDO / DT)))   # buffer del retardo puro
        self.cmd = dict(steering=0.0, throttle=0.0, brake=0.0, parkingbrake=0, gear=1)

    def control(self, **kw):
        """Equivalente a vehicle.control(): guarda el último comando (se mantiene hasta el siguiente)."""
        self.cmd.update({k: v for k, v in kw.items() if v is not None})

    def paso(self):
        """Integra un paso de 1/60 s (lo llama bng.control.step)."""
        c = self.cmd
        # Dirección: retardo puro (cola) + primer orden. steering > 0 gira las ruedas a la derecha.
        self.cola.append(float(np.clip(c['steering'], -1, 1)))
        u = self.cola.popleft()
        self.delta += DT / TAU * (-DELTA_MAX * u - self.delta)
        # Longitudinal: marcha (-1 reversa, 0 neutro, otra adelante), acelerador, freno y rodadura.
        sg = -1.0 if c['gear'] == -1 else (0.0 if c['gear'] == 0 else 1.0)
        if c['parkingbrake'] >= 1:
            self.v = 0.0
        else:
            a = 3.0 * c['throttle'] * sg - 0.08 * self.v
            v_new = self.v + a * DT
            fr = (7.0 * c['brake'] + 0.15) * DT            # freno + rodadura: nunca invierten el sentido
            if abs(v_new) <= fr: v_new = 0.0
            else: v_new -= math.copysign(fr, v_new)
            self.v = v_new
        # Cinemática tractor-remolque (enganche sobre el eje trasero del tractor)
        v = self.v
        u1 = np.array([math.cos(self.th1), math.sin(self.th1)])
        self.H = self.H + v * u1 * DT
        self.th2 += v / L2 * math.sin(self.th1 - self.th2) * DT
        self.th1 += v / L1 * math.tan(self.delta) * DT
        self.t += DT

    def estado(self, cual):
        """Devuelve (punto en el eje, adelante, izquierda, dict state) con el mismo formato que
        vehicle.state de BeamNG: pos, dir (nariz), up, vel."""
        th = self.th1 if cual == 'truck' else self.th2
        f = np.array([math.cos(th), math.sin(th), 0.0]); l = np.array([-f[1], f[0], 0.0]); up = np.array([0, 0, 1.0])
        H3 = np.array([self.H[0], self.H[1], 0.0])
        if cual == 'truck':
            eje = H3 + f * OFF_TRUCK; pos = eje.copy()
            vel = self.v * np.array([math.cos(self.th1), math.sin(self.th1), 0.0])
        else:
            eje = H3 - f * (L2 - DP); pos = eje + l * LAT_TRAILER   # punto medido a DP del eje del remolque
            vel = self.v * f   # aproximación (solo se usa la magnitud)
        pos = pos + up * 0.738                                       # altura de spawn en smallgrid
        return eje, f, l, {'pos': pos.tolist(), 'dir': f.tolist(), 'up': up.tolist(), 'vel': vel.tolist()}


# =====================================================================================
# 3. OBJETOS FALSOS CON LA MISMA INTERFAZ QUE BEAMNGPY
# =====================================================================================
class _Sensores:
    """vehicle.sensors: poll() y el sensor 'timer' (tiempo de simulación)."""
    def __init__(self, planta): self.planta = planta
    def poll(self): pass
    def __getitem__(self, k):
        if k == 'timer': return {'time': self.planta.t}
        if k == 'electrics':                   # sensor Electrics: comando y ángulo del volante (450° a tope)
            return {'steering_input': self.planta.cmd['steering'],
                    'steering': -self.planta.delta / DELTA_MAX * 450.0}
        raise KeyError(k)


class Vehiculo:
    """Camión o tráiler: state, control() y get_bbox() (la usa la calibración del punto de control)."""
    def __init__(self, planta, cual, largo, ancho):
        self.planta, self.cual, self.largo, self.ancho = planta, cual, largo, ancho
        self.sensors = _Sensores(planta)
        self.historial = []                    # todos los comandos recibidos (útil para depurar)

    @property
    def state(self): return self.planta.estado(self.cual)[3]

    def control(self, **kw):
        self.historial.append(dict(kw))
        if self.cual == 'truck':               # solo el camión recibe mandos
            self.planta.control(**kw)

    def get_bbox(self):
        """Caja envolvente centrada en el eje del vehículo (8 esquinas, mismo formato que BeamNGpy)."""
        eje, f, l, _ = self.planta.estado(self.cual)
        c = eje + (f * (-self.largo / 2 + 1.0) if self.cual == 'trailer' else 0)
        out = {}
        for nf, sf in (('front', 1), ('rear', -1)):
            for nl, sl in (('left', 1), ('right', -1)):
                for nz, z in (('bottom', 0.0), ('top', 3.8)):
                    P = c + f * sf * self.largo / 2 + l * sl * self.ancho / 2; P = P.copy(); P[2] = z
                    out[f'{nf}_{nz}_{nl}'] = tuple(P.tolist())
        return out


class _Cam:
    """Cámara: imagen negra de 512x512 (la proyección de la ruta se ejecuta, pero no se muestra)."""
    def poll(self): return {'colour': np.zeros((512, 512, 4), np.uint8)}


class _Nada:
    """IMU y lidar: no se usan en el control."""
    def poll(self): return {}


class _Bng:
    """Instancia BeamNGpy falsa: pause/resume/step, set_deterministic y debug.add_polyline."""
    def __init__(self, planta):
        self.planta = planta
        b = self
        self.control = types.SimpleNamespace(pause=lambda: None, resume=lambda: None, step=lambda n=1: b._step(n))
        self.settings = types.SimpleNamespace(set_deterministic=lambda hz: None)
        self.debug = types.SimpleNamespace(add_polyline=lambda *a, **k: None)
    def _step(self, n):                        # bng.control.step(n): avanza n pasos de la planta
        for _ in range(int(n)): self.planta.paso()
    def close(self): pass


ESTADO = {}                                    # guarda la planta para el resumen final


def _bng_open_main(direction):
    """Reemplaza bng_open.main(): arma el TruckTrailer REAL sobre los objetos falsos, con la misma
    posición inicial y la misma ruta que gen_truck_and_trailer()."""
    planta = Planta(direction)
    truck = Vehiculo(planta, 'truck', 6.5, 2.5)
    trailer = Vehiculo(planta, 'trailer', 9.2, 2.6)
    tr = tt.TruckTrailer(truck, trailer, _Bng(planta))
    tr.truck_origin = (0, 0, 0.738)
    tr.route = tt.make_route((20, 0, 0.738), tt.TIPO_RUTA)
    tr.imu_truck = tr.imu_trailer = tr.lidar_front = tr.lidar_rear = _Nada()
    tr.front_cam = tr.reverse_cam = _Cam()
    ESTADO['planta'], ESTADO['tr'] = planta, tr
    return tr, (0, 0, 0.738)


# =====================================================================================
# 4. CARGA DE LOS MÓDULOS REALES Y EJECUCIÓN DEL main() DEL ALGORITMO
# =====================================================================================
bo = types.ModuleType("arx_truck_ai.bng_open"); bo.main = _bng_open_main
sys.modules["arx_truck_ai.bng_open"] = bo; pkg.bng_open = bo
_load("arx_truck_ai.fin_ruta", "fin_ruta.py")
_load("arx_truck_ai.geometria_ruta", "geometria_ruta.py")
_load("arx_truck_ai.debugging_sys", "debugging_sys.py")
nombre_mod, archivo = {"forward": ("forward", "forward.py"), "backward": ("backward", "backward.py"),
                       "lqr": ("backwards_lqr", "backwards_lqr.py"),
                       # experimento de identificación de la planta (src/experimento_identificacion.py)
                       "identificacion": ("experimento_identificacion", os.path.join("..", "experimento_identificacion.py")),
                       }[algoritmo]
mod = _load(f"arx_truck_ai.{nombre_mod}", archivo)

# Ajustes opcionales NOMBRE=valor tras la ruta (p. ej. PREALIMENTACION_CURVATURA=False): se aplican al
# módulo del algoritmo, o a truck_trailer si la constante es de ese módulo. Sirve para comparar variantes.
import ast
for arg in sys.argv[3:]:
    nombre, valor = arg.split('=', 1)
    destino = mod if hasattr(mod, nombre) else tt
    if not hasattr(destino, nombre):
        raise SystemExit(f"arnes: no existe la constante {nombre}")
    try:
        valor = ast.literal_eval(valor)
    except (ValueError, SyntaxError):
        pass                                   # texto sin comillas, p. ej. MODO_INTEGRADOR=corregido
    setattr(destino, nombre, valor)
    print(f"[arnes] {destino.__name__}.{nombre} = {valor!r}")

builtins.input = lambda *a, **k: ""            # el input() final de cada main() no debe bloquear
import matplotlib.pyplot as plt
plt.show = lambda *a, **k: None

t0 = time.time()
mod.main()                                     # escribe telemetria_<alg>_<ruta>.csv/.json y ruta_<ruta>.csv
p = ESTADO['planta']
# Resumen: si termina sin excepciones y con freno de mano + neutro, el flujo completo funciona.
print(f"[{algoritmo} | {tt.NOMBRES_RUTA[tipo_ruta]}] fin sin excepciones | t_sim={p.t:.1f}s | v={p.v:.2f} | "
      f"cmd={ {k: p.cmd[k] for k in ('gear', 'parkingbrake', 'brake')} } | {time.time() - t0:.1f}s de reloj")