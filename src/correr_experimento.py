"""
correr_experimento.py - Lanza una corrida en BeamNG sin editar main.py (para sesiones por lotes).

Toma la configuración de main.py (ruta, modo, frecuencias y opciones por defecto) y aplica encima las opciones
de la línea de comandos, como el arnés: NOMBRE=valor se asigna al módulo del algoritmo, a truck_trailer o a
fin_ruta, el primero que tenga esa constante. La gráfica final se guarda sin abrir ventana y no espera Enter.

Uso (desde la carpeta de la corrida, como main.py):
    python ..\\..\\src\\correr_experimento.py <forward|backward|lqr> <ruta 0..6> [NOMBRE=valor ...]
    python ..\\..\\src\\correr_experimento.py lqr 1 DECIMACION_TELEMETRIA=1
    python ..\\..\\src\\correr_experimento.py forward 3 LEY_DIRECCION=pure_pursuit
    python ..\\..\\src\\correr_experimento.py lqr 5 PREALIMENTACION_CURVATURA=False PROGRAMACION_EN_V=False --config
Con --config solo imprime la configuración resultante y termina (no abre BeamNG).
"""
import ast
import builtins
import os
import sys

os.environ.setdefault('MPLBACKEND', 'Agg')     # plt.show() no bloquea: la gráfica solo se guarda

import main as configuracion                   # aplica los valores de main.py a los módulos
from arx_truck_ai import truck_trailer as tt
from arx_truck_ai import fin_ruta as fr

MODULOS = {'forward': configuracion.fwd, 'backward': configuracion.bwd, 'lqr': configuracion.bwd_lqr}
OPCIONES_REPORTADAS = ['TIPO_RUTA', 'MODO_SINCRONO', 'DECIMACION_CONTROL', 'DECIMACION_TELEMETRIA', 'PASOS_POR_SEGUNDO',
                       'CAMARAS_ACTIVAS', 'ESPERAR_REPOSO', 'MODO_CONTROL_VELOCIDAD', 'ANTIWINDUP_ARRANQUE',
                       'LEY_DIRECCION', 'PREALIMENTACION_CURVATURA', 'PROGRAMACION_EN_V', 'MODO_INTEGRADOR']


def main(argv):
    solo_config = '--config' in argv
    argv = [a for a in argv if a != '--config']
    if len(argv) < 2 or argv[0] not in MODULOS:
        raise SystemExit(__doc__)
    modulo = MODULOS[argv[0]]
    tt.TIPO_RUTA = int(argv[1])
    for arg in argv[2:]:
        nombre, valor = arg.split('=', 1)
        destino = next((m for m in (modulo, tt, fr) if hasattr(m, nombre)), None)
        if destino is None:
            raise SystemExit(f"correr_experimento: no existe la constante {nombre}")
        try:
            valor = ast.literal_eval(valor)
        except (ValueError, SyntaxError):
            pass                                # texto sin comillas, p. ej. LEY_DIRECCION=pure_pursuit
        setattr(destino, nombre, valor)
    print(f"[correr_experimento] {argv[0]} | ruta {tt.TIPO_RUTA} ({tt.NOMBRES_RUTA.get(tt.TIPO_RUTA)})")
    for nombre in OPCIONES_REPORTADAS:
        destino = next((m for m in (modulo, tt, fr) if hasattr(m, nombre)), None)
        if destino is not None:
            print(f"    {destino.__name__.split('.')[-1]}.{nombre} = {getattr(destino, nombre)!r}")
    if solo_config:
        return 0
    builtins.input = lambda *a, **k: ''        # el input() final de cada main() no debe bloquear
    modulo.main()
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
