% =========================================================================
% ANÁLISIS DE SISTEMA CAMIÓN-TRÁILER (MARCHA ADELANTE Y RETROCESO)
% Script para calcular los modelos de estado, funciones de transferencia,
% análisis de polos, gráficas de incertidumbre paramétrica, regiones de 
% diseño para Loop Shaping y validación de modelado.
% =========================================================================

clear; clc; close all;

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
figure('Name', 'Respuesta al Escalón (Lazo Abierto)', 'NumberTitle', 'off');
t_sim = 0:0.05:8; 
[y_fwd, t_fwd] = step(sys_fwd, t_sim);
[y_bwd, t_bwd] = step(sys_bwd, t_sim);

plot(t_fwd, y_fwd, 'b', 'LineWidth', 2); hold on;
plot(t_bwd, y_bwd, 'r', 'LineWidth', 2);
title('Respuesta al Escalón en Lazo Abierto (Entrada u=1)');
xlabel('Tiempo [s]');
ylabel('Error Lateral y_2 [m]');
legend('Marcha hacia adelante (Marginalmente estable)', 'Marcha en retroceso (Inestable - Efecto Tijera)', 'Location', 'NorthWest');
grid on;
hold off;

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

%% DISEÑO DEL CONTROLADOR (LQR + INTEGRAL + OBSERVADOR)
disp('======================================================');
disp('   DISEÑO DE CONTROLADOR DE ORDEN COMPLETO (RETROCESO)');
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
disp('Gráficas generadas exitosamente.');