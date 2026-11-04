% =========================================================================
% ANÁLISIS DE SISTEMA CAMIÓN-TRÁILER (MARCHA ADELANTE Y RETROCESO)
% Script para calcular los modelos de estado, funciones de transferencia,
% análisis de polos, gráficas de incertidumbre paramétrica, regiones de
% diseño para Loop Shaping y validación de modelado.
%
% Las secciones 1-7 usan el MODELO NOMINAL PRELIMINAR de 3 estados (V0 = 2 m/s,
% D0 = 5 m, ruedas sin retardo): es el análisis del Capítulo 3 previo a la
% identificación. La sección 8 compara con el MODELO DE DISEÑO de 5 estados del
% LQR implementado (backwards_lqr.py), con los parámetros identificados en
% BeamNG (04-nov-2026). La sección 9 es el diseño preliminar LQR + observador,
% que NO se implementó. Las figuras se exportan a matlab/figuras (300 dpi) con
% los nombres que usa el libro (figuras/cap3/).
% =========================================================================

clear; clc; close all;
DIR_FIGURAS = fullfile(fileparts(mfilename('fullpath')), 'figuras');
if ~exist(DIR_FIGURAS, 'dir'), mkdir(DIR_FIGURAS); end

%% DEFINICIÓN DE PARÁMETROS NOMINALES
V0 = 2.0; % Magnitud de velocidad nominal [m/s]
D0 = 5.0; % Longitud efectiva desde el enganche al eje del tráiler [m]

disp('======================================================');
disp('   MODELOS NOMINALES Y FUNCIONES DE TRANSFERENCIA     ');
disp('======================================================');

%% MODELO: MARCHA HACIA ADELANTE (FORWARD)
A_fwd = [0, V0, 0; 
         0, 0, V0/D0; 
         0, 0, -V0/D0];
B_fwd = [0; 0; V0];
C = [1, 0, 0];
D_mat = 0;

sys_fwd = ss(A_fwd, B_fwd, C, D_mat);
tf_fwd = tf(sys_fwd);

disp('Modelo Nominal (Marcha hacia adelante) G_fwd(s):');
tf_fwd
disp('Polos (Adelante):');
disp(pole(sys_fwd)); % Mostrará 0, 0, y -0.4

%% MODELO: MARCHA EN RETROCESO (BACKWARD)
A_bwd = [0, -V0, 0; 
         0, 0, -V0/D0; 
         0, 0, V0/D0];
B_bwd = [0; 0; -V0];

sys_bwd = ss(A_bwd, B_bwd, C, D_mat);
tf_bwd = tf(sys_bwd);

disp('Modelo Nominal (Marcha en Retroceso) G_bwd(s):');
tf_bwd
disp('Polos (Retroceso):');
disp(pole(sys_bwd)); % Mostrará 0, 0, y +0.4 (Polo Inestable)

%% VERIFICACIONES ESTRUCTURALES (valores citados en el Capítulo 3)
% Marcha adelante: el doble autovalor en 0 tiene un solo autovector (rango(A) = 2),
% es decir, un bloque de Jordan de orden 2: NO es marginalmente estable.
fprintf('rango(A_fwd) = %d, rango(A_bwd) = %d (3 - rango = autovectores del autovalor 0)\n', ...
        rank(A_fwd), rank(A_bwd));
x_lib = expm(A_fwd * 100) * [0; 0.01; 0];
fprintf('Marcha adelante, x2(0) = 0.01 rad: x1(100 s) = %.2f m (deriva lineal)\n', x_lib(1));
Ck = ctrb(A_bwd, B_bwd);
Ok = obsv(A_bwd, C);
disp('Matriz de controlabilidad (retroceso) [B AB A^2B]:'); disp(Ck);
fprintf('det(Ck) = %.2f, rango(Ck) = %d, rango(Ok) = %d\n', det(Ck), rank(Ck), rank(Ok));
fprintf('Ceros de G_bwd: %d (no es de fase no mínima)\n', numel(tzero(sys_bwd)));

%% ANÁLISIS DE VARIACIÓN DE PARÁMETROS INCIERTOS (RETROCESO)
V_min = 0.9 * V0; V_max = 1.1 * V0;
D_min = 0.9 * D0; D_max = 1.1 * D0;

V_vals = linspace(V_min, V_max, 5);
D_vals = linspace(D_min, D_max, 5);

figure('Name', 'Bode de la familia de plantas', 'NumberTitle', 'off');
hold on;
w = logspace(-2, 2, 500); 

colores = jet(25); 
idx = 1;           

for i = 1:length(V_vals)
    for j = 1:length(D_vals)
        v = V_vals(i);
        d = D_vals(j);
        
        A_unc = [0, -v, 0; 0, 0, -v/d; 0, 0, v/d];
        B_unc = [0; 0; -v];
        sys_unc = ss(A_unc, B_unc, C, D_mat);
        
        old_lines = findobj(gcf, 'Type', 'line');
        bode(sys_unc, w); 
        
        new_lines = setdiff(findobj(gcf, 'Type', 'line'), old_lines);
        set(new_lines, 'Color', colores(idx, :), 'LineStyle', '--');
        
        idx = idx + 1; 
    end
end

% Dibujamos el nominal, buscamos las líneas nuevas y las configuramos
old_lines = findobj(gcf, 'Type', 'line');
bode(sys_bwd, w); 
new_lines = setdiff(findobj(gcf, 'Type', 'line'), old_lines);
set(new_lines, 'Color', 'r', 'LineWidth', 3);

title('Diagrama de Bode: Familia de Plantas (Retroceso) vs. Modelo Nominal');
grid on;
hold off;

%% ERROR RELATIVO POR INCERTIDUMBRE MULTIPLICATIVA
figure('Name', 'Error Relativo (Plantas vs Nominal)', 'NumberTitle', 'off');
hold on;
[mag_nom, ~] = bode(tf_bwd, w);
mag_nom = squeeze(mag_nom);

max_error_rel = zeros(size(w));
idx = 1; 

for i = 1:length(V_vals)
    for j = 1:length(D_vals)
        v = V_vals(i);
        d = D_vals(j);
        
        A_unc = [0, -v, 0; 0, 0, -v/d; 0, 0, v/d];
        B_unc = [0; 0; -v];
        sys_unc = ss(A_unc, B_unc, C, D_mat);
        
        [mag_unc, ~] = bode(sys_unc, w);
        mag_unc = squeeze(mag_unc);
        
        error_relativo = abs(mag_unc - mag_nom) ./ mag_nom;
        
        % Guardamos el identificador (handle) de la última línea dibujada
        h_err = plot(w, error_relativo, 'Color', colores(idx, :), 'LineStyle', '--');
        
        max_error_rel = max(max_error_rel, error_relativo);
        idx = idx + 1;
    end
end

%% FILTROS DE PONDERACIÓN (Wm, Wd, Wn)
s = tf('s');
Wm = 0.16 * (0.1*s + 1)/(0.01*s + 1);

[mag_Wm, ~] = bode(Wm, w);
% Guardamos el identificador de la curva límite
h_wm = plot(w, squeeze(mag_Wm), 'r', 'LineWidth', 3);
set(gca, 'XScale', 'log', 'YScale', 'log');

title('Error Relativo y Cota Superior (W_m)');
ylabel('Magnitud del Error Relativo');
xlabel('Frecuencia (rad/s)');

legend([h_err, h_wm], 'Familia de Errores Inciertos \Delta_{rel}', 'Cota Superior W_m(s)', 'Location', 'NorthWest');

grid on;
hold off;

%% REGIONES DE LOOP SHAPING (BODE DE DISEÑO)
Wd = 0.1 / (s + 1);

figure('Name', 'Regiones de Loop Shaping', 'NumberTitle', 'off');
% Extraemos magnitudes primero para usar plot tradicional y tener handles perfectos
[mag_Wd, ~] = bode(Wd, w);
[mag_Wm_inv, ~] = bode(1/Wm, w);
[mag_bwd, ~] = bode(tf_bwd, w);

% Trazamos guardando sus handles
h_wd = loglog(w, squeeze(mag_Wd), 'b-.', 'LineWidth', 2); hold on;
h_wm_inv = loglog(w, squeeze(mag_Wm_inv), 'r--', 'LineWidth', 2);
h_bwd = loglog(w, squeeze(mag_bwd), 'k', 'LineWidth', 2);

title('Fronteras de Loop Shaping: Desempeño vs Robustez');

% Leyenda 
legend([h_wd, h_wm_inv, h_bwd], ...
       '|W_d(j\omega)| (Lím. Inferior / Rechazo Perturb.)', ...
       '1 / |W_m(j\omega)| (Lím. Superior / Estabilidad Robusta)', ...
       '|G_{bwd}(j\omega)| (Planta Lazo Abierto)', 'Location', 'SouthWest');
grid on;
hold off;

%% RESPUESTA AL ESCALÓN EN LAZO ABIERTO
% Arriba el error lateral (salida): crece en ambos sentidos de marcha, de forma
% polinómica hacia adelante (integradores) y exponencial en retroceso. Abajo la
% articulación: acotada hacia adelante (polo en -V/D), divergente en retroceso.
fig_escalon = figure('Name', 'Respuesta al Escalón (Lazo Abierto)', 'NumberTitle', 'off');
t_sim = 0:0.05:8;
C_theta = [0, 0, 1];
[y_fwd, t_fwd] = step(sys_fwd, t_sim);
[y_bwd, t_bwd] = step(sys_bwd, t_sim);
th_fwd = step(ss(A_fwd, B_fwd, C_theta, 0), t_sim);
th_bwd = step(ss(A_bwd, B_bwd, C_theta, 0), t_sim);

subplot(2, 1, 1);
plot(t_fwd, y_fwd, 'b', 'LineWidth', 2); hold on;
plot(t_bwd, y_bwd, 'r', 'LineWidth', 2);
title('Respuesta al Escalón en Lazo Abierto (curvatura u = 1)');
ylabel('Error Lateral y_2 [m]');
legend('Marcha hacia adelante (crecimiento polinómico)', 'Marcha en retroceso (divergencia exponencial)', ...
       'Location', 'SouthWest');
grid on; hold off;

subplot(2, 1, 2);
plot(t_fwd, th_fwd, 'b', 'LineWidth', 2); hold on;
plot(t_bwd, th_bwd, 'r', 'LineWidth', 2);
xlabel('Tiempo [s]');
ylabel('Articulación \theta [rad]');
legend('Marcha hacia adelante (acotada: polo en -V/D)', 'Marcha en retroceso (polo en +V/D: jackknife)', ...
       'Location', 'SouthWest');
grid on; hold off;

%% VALIDACIÓN DE EQUIVALENCIA: P^a vs P^o
% Forma Canónica de Observabilidad extraída de tu documento
A_o = [0 0 0; 1 0 0; 0 1 0.4];
B_o = [-1.6; 0; 0];
C_o = [0 0 1];
D_o = 0;
sys_Po = ss(A_o, B_o, C_o, D_o);

figure('Name', 'Equivalencia Dinámica P^a vs P^o', 'NumberTitle', 'off');
t_eq = 0:0.05:5;
[y_pa, ~] = step(sys_bwd, t_eq);
[y_po, ~] = step(sys_Po, t_eq);

% Graficamos P^a con línea gruesa y P^o con línea punteada superpuesta
plot(t_eq, y_pa, 'b', 'LineWidth', 6); hold on;
plot(t_eq, y_po, 'r--', 'LineWidth', 2);
title('Validación de Equivalencia: P^a vs P^o (Respuesta al Escalón)');
xlabel('Tiempo [s]');
ylabel('Amplitud [m]');
legend('Modelo Físico Original (P^a)', 'Forma Canónica de Observabilidad (P^o)', 'Location', 'NorthWest');
grid on;
hold off;

%% LUGAR GEOMÉTRICO DE LAS RAÍCES (ROOT LOCUS)
figure('Name', 'Lugar Geométrico de las Raíces (Retroceso)', 'NumberTitle', 'off');

% Generamos el Root Locus de la planta en retroceso (inestable)
rlocus(sys_bwd);
title('Lugar Geométrico de las Raíces - Marcha en Retroceso (G_{bwd})');

% Embellecemos las líneas para que destaquen en el documento LaTeX
h_rlocus = findobj(gca, 'Type', 'line');
set(h_rlocus, 'LineWidth', 2);
grid on;

% Routh-Hurwitz para u = -Kp*x1: s^3 - 0.4 s^2 - 1.6 Kp = 0 nunca es Hurwitz
% (coeficiente de s^2 negativo y de s nulo). Raíces para algunos Kp:
for Kp = [-2, -0.5, 0.1, 1, 10]
    r = roots([1, -0.4, 0, -1.6*Kp]);
    fprintf('Kp = %5.1f: raíces %s\n', Kp, mat2str(round(r.', 3)));
end

%% MODELO DE DISEÑO DE 5 ESTADOS (LQR IMPLEMENTADO, backwards_lqr.py)
% Estados: x1 error lateral del punto medido, x2 error angular, x3 articulación,
% x4 integral de x1, x5 posición real del volante (primer orden de constante tau).
% Entrada: comando de volante normalizado s en [-1, 1]. Mismas ecuaciones que
% _construir_modelo() en backwards_lqr.py.
disp('======================================================');
disp('   MODELO DE DISEÑO DE 5 ESTADOS (RETROCESO, V = 2 m/s)');
disp('======================================================');
PHI_MAX = 0.6;                                  % rad de rueda con |s| = 1
par_ident  = struct('L1', 2.78, 'L2', 6.21, 'DP', 4.69, 'tau', 0.13 + 0.28);  % BeamNG, 04-nov-2026
par_diseno = struct('L1', 3.3,  'L2', 6.5,  'DP', 5.2,  'tau', 0.8);          % diseño (pesimista)
[A5_id, ~]  = modelo_5_estados(V0, par_ident, PHI_MAX);
[A5, B5]    = modelo_5_estados(V0, par_diseno, PHI_MAX);
fprintf('Polos en lazo abierto, parámetros identificados: %s\n', mat2str(round(eig(A5_id).', 3)));
fprintf('Polos en lazo abierto, parámetros de diseño:     %s\n', mat2str(round(eig(A5).', 3)));
% Pesos del LQR en |x3| = 0 (backwards_lqr.py: Q_X1, Q_X2, Q33_BASE, Q_X4, Q_X5 y R_MAX)
Q5 = diag([0.5, 5.0, 0.5, 0.05, 0.5]);
R5 = 15.0;
K5 = lqr(A5, B5, Q5, R5);
fprintf('K (V = 2 m/s, |x3| = 0): %s  (Python: [-0.755 9.288 5.761 -0.058 1.094])\n', mat2str(round(K5, 3)));
polos_lc = eig(A5 - B5*K5);
fprintf('Polos en lazo cerrado del modelo de diseño: %s\n', mat2str(round(polos_lc.', 3)));

fig_polos = figure('Name', 'Polos del modelo de diseño', 'NumberTitle', 'off');
plot(real(eig(A5)), imag(eig(A5)), 'kx', 'MarkerSize', 12, 'LineWidth', 2); hold on;
plot(real(eig(A5_id)), imag(eig(A5_id)), 'bx', 'MarkerSize', 10, 'LineWidth', 1.5);
plot(real(polos_lc), imag(polos_lc), 'ro', 'MarkerSize', 9, 'LineWidth', 2);
xline(0, 'k:');
xlabel('Parte real [1/s]'); ylabel('Parte imaginaria [rad/s]');
title('Modelo de diseño de 5 estados en retroceso (V = 2 m/s)');
legend('Lazo abierto, parámetros de diseño', 'Lazo abierto, parámetros identificados', ...
       'Lazo cerrado con el LQR (diseño)', 'Location', 'SouthWest');
ylim([-0.45, 0.45]);
grid on; hold off;

%% DISEÑO PRELIMINAR DEL CONTROLADOR (LQR + INTEGRAL + OBSERVADOR) - NO IMPLEMENTADO
% Diseño sobre el modelo nominal de 3 estados. No funcionó en BeamNG (no modela el
% retardo de la dirección ni el punto medido desplazado) y fue reemplazado por el
% LQR de 5 estados de la sección anterior. Se conserva como antecedente.
disp('======================================================');
disp('   DISEÑO PRELIMINAR DE ORDEN COMPLETO (RETROCESO)');
disp('======================================================');

% Planteamiento del Sistema Aumentado con Integrador
% Aumentamos el modelo añadiendo un estado extra para el error integral: dot{x}_i = r - y = r - Cx
A_aug = [A_bwd, zeros(3,1); 
         -C,    0];
B_aug = [B_bwd; 0];

% Sintonización LQR (Matrices de Penalización Q y R)
% Ponderamos agresivamente el ángulo theta (estado 3) para evitar el efecto tijera, 
% y el integrador (estado 4) para eliminar el error rápidamente.
Q_lqr = diag([10, 100, 1000, 500]); 
R_lqr = 1;

% Cálculo de Ganancias Óptimas LQR
K_aug = lqr(A_aug, B_aug, Q_lqr, R_lqr);
K_lqr = K_aug(1:3); % Ganancias de los estados físicos
ki = -K_aug(4);     % Ganancia integral (Ajuste de signo por definición dot{x}_i = r - y)

disp('Ganancias de Realimentación de Estados (K):');
disp(K_lqr);
disp('Ganancia Integral (k_i):');
disp(ki);

% Diseño del Observador de Estados (LQR Dual / Filtro de Kalman Óptimo)
% Asumimos que sólo medimos y2. Penalizamos fuertemente el error de estimación.
Q_obs = 1e6 * eye(3); 
R_obs = 1;            
L_obs = lqr(A_bwd', C', Q_obs, R_obs)';

disp('Ganancias del Observador de Estados (L):');
disp(L_obs);

% Construcción del Sistema en Lazo Cerrado Interconectado
% El sistema final es de orden 7x7: [x (3 física); x_hat (3 observador); x_i (1 integrador)]
% u = -K*x_hat + ki*x_i
A_cl = [A_bwd,          -B_bwd*K_lqr,                 B_bwd*ki;
        L_obs*C,         A_bwd - L_obs*C - B_bwd*K_lqr, B_bwd*ki;
       -C,               zeros(1,3),                  0];
B_cl = [zeros(3,1); zeros(3,1); 1]; % La entrada entra por el integrador
C_cl = [C, zeros(1,3), 0];
D_cl = 0;

sys_cl = ss(A_cl, B_cl, C_cl, D_cl);

% Simulación y Gráfica de Validación
figure('Name', 'Respuesta al Escalón (Lazo Cerrado)', 'NumberTitle', 'off');
t_cl = 0:0.05:15;
[y_cl, t_cl] = step(sys_cl, t_cl);

plot(t_cl, y_cl, 'b', 'LineWidth', 2); hold on;
yline(1, 'r--', 'Referencia Deseada', 'LineWidth', 2, 'LabelHorizontalAlignment', 'left');
title('Respuesta al Escalón: LQR + Observador + Acción Integral (Retroceso)');
xlabel('Tiempo [s]');
ylabel('Error Lateral y_2 [m]');
legend('Trayectoria del Tráiler compensado', 'Línea a seguir', 'Location', 'SouthEast');
grid on;
hold off;

% Cálculo e Impresión de Métricas de Desempeño
step_info = stepinfo(sys_cl);
disp('======================================================');
disp('   MÉTRICAS DE DESEMPEÑO EN LAZO CERRADO');
disp('======================================================');
fprintf('Sobrepaso Máximo (Mp): %.2f%%\n', step_info.Overshoot);
fprintf('Tiempo de Establecimiento (ts): %.2f s\n', step_info.SettlingTime);
fprintf('Error en Estado Estacionario (ess): %.4f\n', 1 - y_cl(end));

disp('======================================================');

%% EXPORTACIÓN DE FIGURAS (nombres de figuras/cap3/ del libro)
exportgraphics(findobj('Type', 'figure', 'Name', 'Bode de la familia de plantas'), ...
               fullfile(DIR_FIGURAS, '2_uncertainty_Bode.png'), 'Resolution', 300);
exportgraphics(findobj('Type', 'figure', 'Name', 'Error Relativo (Plantas vs Nominal)'), ...
               fullfile(DIR_FIGURAS, '3_errores_relativos.png'), 'Resolution', 300);
exportgraphics(findobj('Type', 'figure', 'Name', 'Regiones de Loop Shaping'), ...
               fullfile(DIR_FIGURAS, '4_regiones_prohibidas.png'), 'Resolution', 300);
exportgraphics(fig_escalon, fullfile(DIR_FIGURAS, '5_step_response_open_loop.png'), 'Resolution', 300);
exportgraphics(findobj('Type', 'figure', 'Name', 'Equivalencia Dinámica P^a vs P^o'), ...
               fullfile(DIR_FIGURAS, '6_step_pa_vs_po.png'), 'Resolution', 300);
exportgraphics(findobj('Type', 'figure', 'Name', 'Lugar Geométrico de las Raíces (Retroceso)'), ...
               fullfile(DIR_FIGURAS, '7_root_locus.png'), 'Resolution', 300);
exportgraphics(fig_polos, fullfile(DIR_FIGURAS, '12_polos_modelo_diseno.png'), 'Resolution', 300);
disp(['Gráficas generadas y exportadas en ', DIR_FIGURAS]);

%% FUNCIONES LOCALES
function [A, B] = modelo_5_estados(v, p, phi_max)
% Modelo de diseño de 5 estados del LQR en retroceso (backwards_lqr._construir_modelo).
    A = zeros(5);
    A(1, 2) = -v;
    A(1, 3) = v * p.DP / p.L2;
    A(2, 3) = v / p.L2;
    A(3, 3) = v / p.L2;
    A(3, 5) = v * phi_max / p.L1;
    A(4, 1) = 1;
    A(5, 5) = -1 / p.tau;
    B = [0; 0; 0; 0; 1 / p.tau];
end