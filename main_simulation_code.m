clear all; close all; clc;
%% Main Code
% Sample time
Ts = 1e-5;

% Machine parameters
Pn = 746; % Nominal Power
Vn = 220; % Nominal Voltage
fn = 60; % Nominal Frequency
F = 0.001; % Friction factor
J = 0.0017; % Moment of inertia [kg m^2]
p = 2; % Pole pairs
Lm = 0.6941; % Magnetizing inductance [H]
Ls = 0.7185; % Stator inductance [H]
Lr = 0.7185; % Rotor inductance [H]
Rs = 7.5022; % Stator resistance [Ohm]
Rr = 4.8319; % Rotor resistance [Ohm]
sf_nom = 0.78; % Nominal stator flux [Wb]
T_nom = 4.1; % Nominal torque [Nm]
w_nom = (120*60/(2*p))*(2*pi/60); % Nominal Speed [rad/s]


%% ControlParameters
ts = 1e-3;
zeta = 1
wn = log(0.02)/(-zeta*ts)

Pole = -wn*zeta+j*wn*sqrt(1-zeta^2)

%% ControllerCalculation
s = tf('s');
RotorTF = (1/J)/(s  + F/J)
RotorPOLES = pole(RotorTF)

C_s = (s - 2*RotorPOLES)/s
OpenLoopTF = C_s*RotorTF

figure
rlocus(OpenLoopTF)

NumGain = 0;
DenGain = 0;
syms x
for i = 1:length(OpenLoopTF.Numerator{1})
    if OpenLoopTF.Numerator{1}(i) ~= 0
        NumGain = NumGain + OpenLoopTF.Numerator{1}(i)*(x^(length(OpenLoopTF.Numerator{1})-i));
    end
    if OpenLoopTF.Denominator{1}(i) ~= 0
        DenGain = DenGain + OpenLoopTF.Denominator{1}(i)*(x^(length(OpenLoopTF.Numerator{1})-i));
    end
end
disp(NumGain)
disp(DenGain)

Gain = DenGain/NumGain;
Gain = abs(real(double(subs(Gain, Pole))))
C_s = C_s * Gain

OpenLoopTF = RotorTF*C_s

AA = pole(OpenLoopTF)

CL_TF = feedback(RotorTF*C_s,1)
stepinfo(CL_TF)

figure
step(CL_TF)

% Discretization
C_z = c2d(C_s, Ts)

%% ControllerParametersGains
global Gains
Gains = [C_z.Numerator{1}(1), C_z.Numerator{1}(2), -1*C_z.Denominator{1}(2)]

%% PredictiveControlParameters

% Weighting factor for the cost function of PTC
lambda = T_nom/sf_nom;

% DC-link voltage [V]
Vdc = 520;

% Auxiliary constants
ts = Ls/Rs;
tr = Lr/Rr;
sigma = 1-((Lm^2)/(Lr*Ls));
kr = Lm/Lr;
r_sigma = Rs+kr^2*Rr;
t_sigma = sigma*Ls/r_sigma;

% Voltage vectors
v0 = 0;
v1 = 2/3*Vdc;
v2 = 1/3*Vdc + 1j*sqrt(3)/3*Vdc;
v3 = -1/3*Vdc + 1j*sqrt(3)/3*Vdc;
v4 = -2/3*Vdc;
v5 = -1/3*Vdc - 1j*sqrt(3)/3*Vdc;
v6 = 1/3*Vdc - 1j*sqrt(3)/3*Vdc;
v7 = 0;
v = [v0 v1 v2 v3 v4 v5 v6 v7];

% Switching states  
states = [0 0 0;
          1 0 0;
          1 1 0;
          0 1 0;
          0 1 1;
          0 0 1;
          1 0 1;
          1 1 1];

global param
param.Ts = Ts;
param.Rs = Rs;
param.Lr = Lr;
param.Lm = Lm;
param.Ls = Ls;
param.p = p;
param.tr = tr;
param.kr = kr;
param.r_sigma = r_sigma;
param.t_sigma = t_sigma;
param.lambda = lambda;
param.v = v;
param.states = states;
param.sf_nom = sf_nom;
param.T_nom = T_nom;


%% IdealModelTests
% Test: Speed Steps Without Load
sm_time = 12;
SpeedTable = [0, 0;
              2, 0;
              2 + Ts, 0.5*w_nom;
              4, 0.5*w_nom;
              4 + Ts, w_nom;
              6, w_nom;
              6 + Ts, -0.5*w_nom;
              8, -0.5*w_nom;
              8 + Ts,-w_nom;
              10, -w_nom;
              10 + Ts, 0;
              12, 0]
wref = SpeedTable(:,2);
t_w = SpeedTable(:,1);
Tref = [0, 0, 0.5, 0.5, -0.5, -0.5, 0, 0];
t_T = [0, 2, 2+ Ts,  6, 6 + Ts, 10, 10 + Ts 12];

sim('SIMULATIONS/control_simulation_A.slx')
save DATA/SIMULATION_A/TEST_A.mat t w_out w_ref TL T_ref e_PI T_ref

figure(1)
plot(t, w_out, 'b', t, w_ref, 'r', 'LineWidth',2);title('Velocidade do motor (rad/s)');xlabel('Tempo (s)');ylabel('Velocidade (rad/s)'); legend('\omega_{real}','\omega_{ref}');xlim([0 sm_time]);
saveas(gcf, 'IMAGES/SIMULATION_A/RESPOSTA/EPS/A_VELOCIDADES.eps', 'epsc')
saveas(gcf, 'IMAGES/SIMULATION_A/RESPOSTA/PNG/A_VELOCIDADES.png')
saveas(gcf, 'IMAGES/SIMULATION_A/RESPOSTA/FIG/A_VELOCIDADES.fig')

figure(4)
plot(t, TL,'b', t, T_ref, 'r', 'LineWidth', 2); title('Torque'); xlabel('Amostras'); ylabel('Torque (N.m)'); legend('T_{load}','T_{ref}');xlim([0 sm_time]);
saveas(gcf, 'IMAGES/SIMULATION_A/RESPOSTA/EPS/A_CARGA.eps', 'epsc')
saveas(gcf, 'IMAGES/SIMULATION_A/RESPOSTA/PNG/A_CARGA.png')
saveas(gcf, 'IMAGES/SIMULATION_A/RESPOSTA/FIG/A_CARGA.fig')

figure(5)
subplot(2,1,1)
plot(t, e_PI, 'LineWidth', 2); title('Erro controlador PI (rad/s)'); xlabel('Tempo (s)'); ylabel('Erro de Velocidade (rad/s)');xlim([0 sm_time]);
subplot(2,1,2)
plot(t, T_ref, 'LineWidth', 2); title('Torque de referência (N.m)'); xlabel('Tempo (s)'); ylabel('Torque de referência (N.m)');xlim([0 sm_time]);
saveas(gcf, 'IMAGES/SIMULATION_A/RESPOSTA/EPS/A_PI.eps', 'epsc')
saveas(gcf, 'IMAGES/SIMULATION_A/RESPOSTA/PNG/A_PI.png')
saveas(gcf, 'IMAGES/SIMULATION_A/RESPOSTA/FIG/A_PI.fig')

figure(6)
subplot(2,1,1)
plot(t, w_ref, 'LineWidth', 2); title('Velocidade de referência (rad/s)'); xlabel('Tempo (s)'); ylabel('Velocidade de referência (rad/s)');xlim([0 sm_time]);
subplot(2,1,2)
plot(t, TL, 'LineWidth', 2); title('Torque de carga (N.m)'); xlabel('Tempo (s)'); ylabel('Torque de carga (N.m)');xlim([0 sm_time]);
saveas(gcf, 'IMAGES/SIMULATION_A/RESPOSTA/EPS/A_REFERENCIAS.eps', 'epsc')
saveas(gcf, 'IMAGES/SIMULATION_A/RESPOSTA/PNG/A_REFERENCIAS.png')
saveas(gcf, 'IMAGES/SIMULATION_A/RESPOSTA/FIG/A_REFERENCIAS.fig')

% Test: Speed Steps With Load
sm_time = 12;
SpeedTable = [0, 0;
              2, 0;
              2 + Ts, 0.5*w_nom;
              4, 0.5*w_nom;
              4 + Ts, w_nom;
              6, w_nom;
              6 + Ts, -0.5*w_nom;
              8, -0.5*w_nom;
              8 + Ts,-w_nom;
              10, -w_nom;
              10 + Ts, 0;
              12, 0]
wref = SpeedTable(:,2);
t_w = SpeedTable(:,1);
TorqueTable = [0, 0;
               2, 0;
               2 + Ts, 0.9*T_nom;
               5.5, 0.9*T_nom;
               5.5 + Ts, 0;
               6, 0;
               6 + Ts, -0.5;
               6.5, -0.5;
               6.5 + Ts, -0.9*T_nom;
               10, -0.9*T_nom;
               10 + Ts, 0;
               12, 0]
Tref = TorqueTable(:,2);
t_T = TorqueTable(:,1);

sim('SIMULATIONS/control_simulation_A.slx')
save DATA/SIMULATION_A/TEST_B.mat t w_out w_ref TL T_ref e_PI T_ref

figure(1)
plot(t, w_out, 'b', t, w_ref, 'r', 'LineWidth',2);title('Velocidade do motor (rad/s)');xlabel('Tempo (s)');ylabel('Velocidade (rad/s)'); legend('\omega_{real}','\omega_{ref}');xlim([0 sm_time]);
saveas(gcf, 'IMAGES/SIMULATION_A/RESPOSTA/EPS/B_VELOCIDADES.eps', 'epsc')
saveas(gcf, 'IMAGES/SIMULATION_A/RESPOSTA/PNG/B_VELOCIDADES.png')
saveas(gcf, 'IMAGES/SIMULATION_A/RESPOSTA/FIG/B_VELOCIDADES.fig')

figure(4)
plot(t, TL,'b', t, T_ref, 'r', 'LineWidth', 2); title('Torque'); xlabel('Amostras'); ylabel('Torque (N.m)'); legend('T_{load}','T_{ref}');xlim([0 sm_time]);
saveas(gcf, 'IMAGES/SIMULATION_A/RESPOSTA/EPS/B_CARGA.eps', 'epsc')
saveas(gcf, 'IMAGES/SIMULATION_A/RESPOSTA/PNG/B_CARGA.png')
saveas(gcf, 'IMAGES/SIMULATION_A/RESPOSTA/FIG/B_CARGA.fig')

figure(5)
subplot(2,1,1)
plot(t, e_PI, 'LineWidth', 2); title('Erro controlador PI (rad/s)'); xlabel('Tempo (s)'); ylabel('Erro Velocidade (rad/s)');xlim([0 sm_time]);
subplot(2,1,2)
plot(t, T_ref, 'LineWidth', 2); title('Torque de referência (N.m)'); xlabel('Tempo (s)'); ylabel('Torque de referência (N.m)');xlim([0 sm_time]);
saveas(gcf, 'IMAGES/SIMULATION_A/RESPOSTA/EPS/B_PI.eps', 'epsc')
saveas(gcf, 'IMAGES/SIMULATION_A/RESPOSTA/PNG/B_PI.png')
saveas(gcf, 'IMAGES/SIMULATION_A/RESPOSTA/FIG/B_PI.fig')


figure(6)
subplot(2,1,1)
plot(t, w_ref, 'LineWidth', 2); title('Velocidade de referência (rad/s)'); xlabel('Tempo (s)'); ylabel('Velocidade de referência (rad/s)');xlim([0 sm_time]);
subplot(2,1,2)
plot(t, TL, 'LineWidth', 2); title('Torque de carga (N.m)'); xlabel('Tempo (s)'); ylabel('Torque de carga (N.m)');xlim([0 sm_time]);
saveas(gcf, 'IMAGES/SIMULATION_A/RESPOSTA/EPS/B_REFERENCIAS.eps', 'epsc')
saveas(gcf, 'IMAGES/SIMULATION_A/RESPOSTA/PNG/B_REFERENCIAS.png')
saveas(gcf, 'IMAGES/SIMULATION_A/RESPOSTA/FIG/B_REFERENCIAS.fig')



% Test: Constant Speed With Load
sm_time = 12;
SpeedTable = [0, w_nom;
              6, w_nom;
              6 + Ts, -w_nom;
              12, -w_nom]
wref = SpeedTable(:,2);
t_w = SpeedTable(:,1);
TorqueTable = [0, 0.5;
               1, 0.5;
               1 + Ts, 0.9*T_nom;
               5, 0.9*T_nom;
               5 + Ts, 0.5;
               6, 0.5;
               6 + Ts, -0.5;
               7, -0.5;
               7 + Ts, -0.9*T_nom;
               11, -0.9*T_nom;
               11 + Ts, -0.5;
               12, -0.5]
Tref = TorqueTable(:,2);
t_T = TorqueTable(:,1);

sim('SIMULATIONS/control_simulation_A.slx')
save DATA/SIMULATION_A/TEST_C.mat t w_out w_ref TL T_ref e_PI T_ref

figure(1)
plot(t, w_out, 'b', t, w_ref, 'r', 'LineWidth',2);title('Velocidade do motor (rad/s)');xlabel('Tempo (s)');ylabel('Velocidade (rad/s)'); legend('\omega_{real}','\omega_{ref}');xlim([0 sm_time]);
saveas(gcf, 'IMAGES/SIMULATION_A/RESPOSTA/EPS/C_VELOCIDADES.eps', 'epsc')
saveas(gcf, 'IMAGES/SIMULATION_A/RESPOSTA/PNG/C_VELOCIDADES.png')
saveas(gcf, 'IMAGES/SIMULATION_A/RESPOSTA/FIG/C_VELOCIDADES.fig')

figure(4)
plot(t, TL,'b', t, T_ref, 'r', 'LineWidth', 2); title('Torque'); xlabel('Amostras'); ylabel('Torque (N.m)'); legend('T_{load}','T_{ref}');xlim([0 sm_time]);
saveas(gcf, 'IMAGES/SIMULATION_A/RESPOSTA/EPS/C_CARGA.eps', 'epsc')
saveas(gcf, 'IMAGES/SIMULATION_A/RESPOSTA/PNG/C_CARGA.png')
saveas(gcf, 'IMAGES/SIMULATION_A/RESPOSTA/FIG/C_CARGA.fig')

figure(5)
subplot(2,1,1)
plot(t, e_PI, 'LineWidth', 2); title('Erro controlador PI (rad/s)'); xlabel('Tempo (s)'); ylabel('Erro Velocidade (rad/s)');xlim([0 sm_time]);
subplot(2,1,2)
plot(t, T_ref, 'LineWidth', 2); title('Torque de referência (N.m)'); xlabel('Tempo (s)'); ylabel('Torque de referência (N.m)');xlim([0 sm_time]);
saveas(gcf, 'IMAGES/SIMULATION_A/RESPOSTA/EPS/C_PI.eps', 'epsc')
saveas(gcf, 'IMAGES/SIMULATION_A/RESPOSTA/PNG/C_PI.png')
saveas(gcf, 'IMAGES/SIMULATION_A/RESPOSTA/FIG/C_PI.fig')


figure(6)
subplot(2,1,1)
plot(t, w_ref, 'LineWidth', 2); title('Velocidade de referência (rad/s)'); xlabel('Tempo (s)'); ylabel('Velocidade de referência (rad/s)');xlim([0 sm_time]);
subplot(2,1,2)
plot(t, TL, 'LineWidth', 2); title('Torque de carga (N.m)'); xlabel('Tempo (s)'); ylabel('Torque de carga (N.m)');xlim([0 sm_time]);
saveas(gcf, 'IMAGES/SIMULATION_A/RESPOSTA/EPS/C_REFERENCIAS.eps', 'epsc')
saveas(gcf, 'IMAGES/SIMULATION_A/RESPOSTA/PNG/C_REFERENCIAS.png')
saveas(gcf, 'IMAGES/SIMULATION_A/RESPOSTA/FIG/C_REFERENCIAS.fig')

% Test: Slow Speed With Load
sm_time = 12;
SpeedTable = [0, 1;
              6, 1;
              6 + Ts, -1;
              12, -1]
wref = SpeedTable(:,2);
t_w = SpeedTable(:,1);
TorqueTable = [0, 0.5;
               1, 0.5;
               1 + Ts, 0.9*T_nom;
               5, 0.9*T_nom;
               5 + Ts, 0.5;
               6, 0.5;
               6 + Ts, -0.5;
               7, -0.5;
               7 + Ts, -0.9*T_nom;
               11, -0.9*T_nom;
               11 + Ts, -0.5;
               12, -0.5]
Tref = TorqueTable(:,2);
t_T = TorqueTable(:,1);

sim('SIMULATIONS/control_simulation_A.slx')
save DATA/SIMULATION_A/TEST_D.mat t w_out w_ref TL T_ref e_PI T_ref

figure(1)
plot(t, w_out, 'b', t, w_ref, 'r', 'LineWidth',2);title('Velocidade do motor (rad/s)');xlabel('Tempo (s)');ylabel('Velocidade (rad/s)'); legend('\omega_{real}','\omega_{ref}');xlim([0 sm_time]);
saveas(gcf, 'IMAGES/SIMULATION_A/RESPOSTA/EPS/D_VELOCIDADES.eps', 'epsc')
saveas(gcf, 'IMAGES/SIMULATION_A/RESPOSTA/PNG/D_VELOCIDADES.png')
saveas(gcf, 'IMAGES/SIMULATION_A/RESPOSTA/FIG/D_VELOCIDADES.fig')

figure(4)
plot(t, TL,'b', t, T_ref, 'r', 'LineWidth', 2); title('Torque'); xlabel('Amostras'); ylabel('Torque (N.m)'); legend('T_{load}','T_{ref}');xlim([0 sm_time]);
saveas(gcf, 'IMAGES/SIMULATION_A/RESPOSTA/EPS/D_CARGA.eps', 'epsc')
saveas(gcf, 'IMAGES/SIMULATION_A/RESPOSTA/PNG/D_CARGA.png')
saveas(gcf, 'IMAGES/SIMULATION_A/RESPOSTA/FIG/D_CARGA.fig')

figure(5)
subplot(2,1,1)
plot(t, e_PI, 'LineWidth', 2); title('Erro controlador PI (rad/s)'); xlabel('Tempo (s)'); ylabel('Erro Velocidade (rad/s)');xlim([0 sm_time]);
subplot(2,1,2)
plot(t, T_ref, 'LineWidth', 2); title('Torque de referência (N.m)'); xlabel('Tempo (s)'); ylabel('Torque de referência (N.m)');xlim([0 sm_time]);
saveas(gcf, 'IMAGES/SIMULATION_A/RESPOSTA/EPS/D_PI.eps', 'epsc')
saveas(gcf, 'IMAGES/SIMULATION_A/RESPOSTA/PNG/D_PI.png')
saveas(gcf, 'IMAGES/SIMULATION_A/RESPOSTA/FIG/D_PI.fig')


figure(6)
subplot(2,1,1)
plot(t, w_ref, 'LineWidth', 2); title('Velocidade de referência (rad/s)'); xlabel('Tempo (s)'); ylabel('Velocidade de referência (rad/s)');xlim([0 sm_time]);
subplot(2,1,2)
plot(t, TL, 'LineWidth', 2); title('Torque de carga (N.m)'); xlabel('Tempo (s)'); ylabel('Torque de carga (N.m)');xlim([0 sm_time]);
saveas(gcf, 'IMAGES/SIMULATION_A/RESPOSTA/EPS/D_REFERENCIAS.eps', 'epsc')
saveas(gcf, 'IMAGES/SIMULATION_A/RESPOSTA/PNG/D_REFERENCIAS.png')
saveas(gcf, 'IMAGES/SIMULATION_A/RESPOSTA/FIG/D_REFERENCIAS.fig')


% Test: Blocked Rotor With Load
sm_time = 12;
SpeedTable = [0, 0;
              12, 0]
wref = SpeedTable(:,2);
t_w = SpeedTable(:,1);
TorqueTable = [0, 0.5;
               1, 0.5;
               1 + Ts, 0.9*T_nom;
               5, 0.9*T_nom;
               5 + Ts, 0.5;
               6, 0.5;
               6 + Ts, -0.5;
               7, -0.5;
               7 + Ts, -0.9*T_nom;
               11, -0.9*T_nom;
               11 + Ts, -0.5;
               12, -0.5]
Tref = TorqueTable(:,2);
t_T = TorqueTable(:,1);

sim('SIMULATIONS/control_simulation_A.slx')
save DATA/SIMULATION_A/TEST_E.mat t w_out w_ref TL T_ref e_PI T_ref

figure(1)
plot(t, w_out, 'b', t, w_ref, 'r', 'LineWidth',2);title('Velocidade do motor (rad/s)');xlabel('Tempo (s)');ylabel('Velocidade (rad/s)'); legend('\omega_{real}','\omega_{ref}');xlim([0 sm_time]);
saveas(gcf, 'IMAGES/SIMULATION_A/RESPOSTA/EPS/E_VELOCIDADES.eps', 'epsc')
saveas(gcf, 'IMAGES/SIMULATION_A/RESPOSTA/PNG/E_VELOCIDADES.png')
saveas(gcf, 'IMAGES/SIMULATION_A/RESPOSTA/FIG/E_VELOCIDADES.fig')

figure(4)
plot(t, TL,'b', t, T_ref, 'r', 'LineWidth', 2); title('Torque'); xlabel('Amostras'); ylabel('Torque (N.m)'); legend('T_{load}','T_{ref}');xlim([0 sm_time]);
saveas(gcf, 'IMAGES/SIMULATION_A/RESPOSTA/EPS/E_CARGA.eps', 'epsc')
saveas(gcf, 'IMAGES/SIMULATION_A/RESPOSTA/PNG/E_CARGA.png')
saveas(gcf, 'IMAGES/SIMULATION_A/RESPOSTA/FIG/E_CARGA.fig')

figure(5)
subplot(2,1,1)
plot(t, e_PI, 'LineWidth', 2); title('Erro controlador PI (rad/s)'); xlabel('Tempo (s)'); ylabel('Erro Velocidade (rad/s)');xlim([0 sm_time]);
subplot(2,1,2)
plot(t, T_ref, 'LineWidth', 2); title('Torque de referência (N.m)'); xlabel('Tempo (s)'); ylabel('Torque de referência (N.m)');xlim([0 sm_time]);
saveas(gcf, 'IMAGES/SIMULATION_A/RESPOSTA/EPS/E_PI.eps', 'epsc')
saveas(gcf, 'IMAGES/SIMULATION_A/RESPOSTA/PNG/E_PI.png')
saveas(gcf, 'IMAGES/SIMULATION_A/RESPOSTA/FIG/E_PI.fig')


figure(6)
subplot(2,1,1)
plot(t, w_ref, 'LineWidth', 2); title('Velocidade de referência (rad/s)'); xlabel('Tempo (s)'); ylabel('Velocidade de referência (rad/s)');xlim([0 sm_time]);
subplot(2,1,2)
plot(t, TL, 'LineWidth', 2); title('Torque de carga (N.m)'); xlabel('Tempo (s)'); ylabel('Torque de carga (N.m)');xlim([0 sm_time]);
saveas(gcf, 'IMAGES/SIMULATION_A/RESPOSTA/EPS/E_REFERENCIAS.eps', 'epsc')
saveas(gcf, 'IMAGES/SIMULATION_A/RESPOSTA/PNG/E_REFERENCIAS.png')
saveas(gcf, 'IMAGES/SIMULATION_A/RESPOSTA/FIG/E_REFERENCIAS.fig')


%% FullModelTests
% Test: Speed Steps Without Load
sm_time = 12;
SpeedTable = [0, 0;
              2, 0;
              2 + Ts, 0.5*w_nom;
              4, 0.5*w_nom;
              4 + Ts, w_nom;
              6, w_nom;
              6 + Ts, -0.5*w_nom;
              8, -0.5*w_nom;
              8 + Ts,-w_nom;
              10, -w_nom;
              10 + Ts, 0;
              12, 0]
wref = SpeedTable(:,2);
t_w = SpeedTable(:,1);
Tref = [0, 0, 0.5, 0.5, -0.5, -0.5, 0, 0];
t_T = [0, 2, 2+ Ts,  6, 6 + Ts, 10, 10 + Ts 12];

sim('SIMULATIONS/control_simulation_B.slx')
save DATA/SIMULATION_B/TEST_A.mat t w_out w_ref Ia Ib Ic Sfluxd Sfluxq TL Tem T_ref e_PI T_ref

figure(1)
plot(t, w_out, 'b', t, w_ref, 'r', 'LineWidth',2);title('Velocidade do motor (rad/s)');xlabel('Tempo (s)');ylabel('Velocidade (rad/s)'); legend('\omega_{real}','\omega_{ref}');xlim([0 sm_time]);
saveas(gcf, 'IMAGES/SIMULATION_B/RESPOSTA/EPS/A_VELOCIDADES.eps', 'epsc')
saveas(gcf, 'IMAGES/SIMULATION_B/RESPOSTA/PNG/A_VELOCIDADES.png')
saveas(gcf, 'IMAGES/SIMULATION_B/RESPOSTA/FIG/A_VELOCIDADES.fig')

figure(2)
plot(t, Ia, t, Ib, t, Ic, 'LineWidth', 2);title('Correntes do motor I_{abc}');xlabel('Tempo (s)');ylabel('Corrente (A)'); legend('I_a','I_b','I_c');xlim([0 sm_time]);
saveas(gcf, 'IMAGES/SIMULATION_B/RESPOSTA/EPS/A_CORRENTES.eps', 'epsc')
saveas(gcf, 'IMAGES/SIMULATION_B/RESPOSTA/PNG/A_CORRENTES.png')
saveas(gcf, 'IMAGES/SIMULATION_B/RESPOSTA/FIG/A_CORRENTES.fig')

figure(3)
plot(Sfluxd, Sfluxq, 'LineWidth', 2); title('Fluxo do estator (Wb)'); xlabel('Fluxo q (Wb)'); ylabel('Fluxo d (Wb)'); axis([-1.5*sf_nom 1.5*sf_nom -1.5*sf_nom 1.5*sf_nom]);
saveas(gcf, 'IMAGES/SIMULATION_B/RESPOSTA/EPS/A_FLUXOS.eps', 'epsc')
saveas(gcf, 'IMAGES/SIMULATION_B/RESPOSTA/PNG/A_FLUXOS.png')
saveas(gcf, 'IMAGES/SIMULATION_B/RESPOSTA/FIG/A_FLUXOS.fig')

figure(4)
plot(t, TL,'b', t, Tem, 'r', 'LineWidth', 2); title('Torque'); xlabel('Amostras'); ylabel('Torque (N.m)'); legend('T_{load}','T_{em}');xlim([0 sm_time]);
saveas(gcf, 'IMAGES/SIMULATION_B/RESPOSTA/EPS/A_CARGA.eps', 'epsc')
saveas(gcf, 'IMAGES/SIMULATION_B/RESPOSTA/PNG/A_CARGA.png')
saveas(gcf, 'IMAGES/SIMULATION_B/RESPOSTA/FIG/A_CARGA.fig')

figure(5)
subplot(2,1,1)
plot(t, e_PI, 'LineWidth', 2); title('Erro controlador PI (rad/s)'); xlabel('Tempo (s)'); ylabel('Erro Velocidade (rad/s)');xlim([0 sm_time]);
subplot(2,1,2)
plot(t, T_ref, 'LineWidth', 2); title('Torque de referência (N.m)'); xlabel('Tempo (s)'); ylabel('Torque de referência (N.m)');xlim([0 sm_time]);
saveas(gcf, 'IMAGES/SIMULATION_B/RESPOSTA/EPS/A_PI.eps', 'epsc')
saveas(gcf, 'IMAGES/SIMULATION_B/RESPOSTA/PNG/A_PI.png')
saveas(gcf, 'IMAGES/SIMULATION_B/RESPOSTA/FIG/A_PI.fig')

figure(6)
subplot(2,1,1)
plot(t, w_ref, 'LineWidth', 2); title('Velocidade de referência (rad/s)'); xlabel('Tempo (s)'); ylabel('Velocidade de referência (rad/s)');xlim([0 sm_time]);
subplot(2,1,2)
plot(t, TL, 'LineWidth', 2); title('Torque de carga (N.m)'); xlabel('Tempo (s)'); ylabel('Torque de carga (N.m)');xlim([0 sm_time]);
saveas(gcf, 'IMAGES/SIMULATION_B/RESPOSTA/EPS/A_REFERENCIAS.eps', 'epsc')
saveas(gcf, 'IMAGES/SIMULATION_B/RESPOSTA/PNG/A_REFERENCIAS.png')
saveas(gcf, 'IMAGES/SIMULATION_B/RESPOSTA/FIG/A_REFERENCIAS.fig')

% Test: Speed Steps With Load
sm_time = 12;
SpeedTable = [0, 0;
              2, 0;
              2 + Ts, 0.5*w_nom;
              4, 0.5*w_nom;
              4 + Ts, w_nom;
              6, w_nom;
              6 + Ts, -0.5*w_nom;
              8, -0.5*w_nom;
              8 + Ts,-w_nom;
              10, -w_nom;
              10 + Ts, 0;
              12, 0]
wref = SpeedTable(:,2);
t_w = SpeedTable(:,1);
TorqueTable = [0, 0;
               2, 0;
               2 + Ts, 0.9*T_nom;
               5.5, 0.9*T_nom;
               5.5 + Ts, 0;
               6, 0;
               6 + Ts, -0.5;
               6.5, -0.5;
               6.5 + Ts, -0.9*T_nom;
               10, -0.9*T_nom;
               10 + Ts, 0;
               12, 0]
Tref = TorqueTable(:,2);
t_T = TorqueTable(:,1);

sim('SIMULATIONS/control_simulation_B.slx')
save DATA/SIMULATION_B/TEST_B.mat t w_out w_ref Ia Ib Ic Sfluxd Sfluxq TL Tem T_ref e_PI T_ref

figure(1)
plot(t, w_out, 'b', t, w_ref, 'r', 'LineWidth',2);title('Velocidade do motor (rad/s)');xlabel('Tempo (s)');ylabel('Velocidade (rad/s)'); legend('\omega_{real}','\omega_{ref}');xlim([0 sm_time]);
saveas(gcf, 'IMAGES/SIMULATION_B/RESPOSTA/EPS/B_VELOCIDADES.eps', 'epsc')
saveas(gcf, 'IMAGES/SIMULATION_B/RESPOSTA/PNG/B_VELOCIDADES.png')
saveas(gcf, 'IMAGES/SIMULATION_B/RESPOSTA/FIG/B_VELOCIDADES.fig')

figure(2)
plot(t, Ia, t, Ib, t, Ic, 'LineWidth', 2);title('Correntes do motor I_{abc}');xlabel('Tempo (s)');ylabel('Corrente (A)'); legend('I_a','I_b','I_c');xlim([0 sm_time]);
saveas(gcf, 'IMAGES/SIMULATION_B/RESPOSTA/EPS/B_CORRENTES.eps', 'epsc')
saveas(gcf, 'IMAGES/SIMULATION_B/RESPOSTA/PNG/B_CORRENTES.png')
saveas(gcf, 'IMAGES/SIMULATION_B/RESPOSTA/FIG/B_CORRENTES.fig')

figure(3)
plot(Sfluxd, Sfluxq, 'LineWidth', 2); title('Fluxo do estator (Wb)'); xlabel('Fluxo q (Wb)'); ylabel('Fluxo d (Wb)'); axis([-1.5*sf_nom 1.5*sf_nom -1.5*sf_nom 1.5*sf_nom]);
saveas(gcf, 'IMAGES/SIMULATION_B/RESPOSTA/EPS/B_FLUXOS.eps', 'epsc')
saveas(gcf, 'IMAGES/SIMULATION_B/RESPOSTA/PNG/B_FLUXOS.png')
saveas(gcf, 'IMAGES/SIMULATION_B/RESPOSTA/FIG/B_FLUXOS.fig')

figure(4)
plot(t, TL,'b', t, Tem, 'r', 'LineWidth', 2); title('Torque'); xlabel('Amostras'); ylabel('Torque (N.m)'); legend('T_{load}','T_{em}');xlim([0 sm_time]);
saveas(gcf, 'IMAGES/SIMULATION_B/RESPOSTA/EPS/B_CARGA.eps', 'epsc')
saveas(gcf, 'IMAGES/SIMULATION_B/RESPOSTA/PNG/B_CARGA.png')
saveas(gcf, 'IMAGES/SIMULATION_B/RESPOSTA/FIG/B_CARGA.fig')

figure(5)
subplot(2,1,1)
plot(t, e_PI, 'LineWidth', 2); title('Erro controlador PI (rad/s)'); xlabel('Tempo (s)'); ylabel('Erro Velocidade (rad/s)');xlim([0 sm_time]);
subplot(2,1,2)
plot(t, T_ref, 'LineWidth', 2); title('Torque de referência (N.m)'); xlabel('Tempo (s)'); ylabel('Torque de referência (N.m)');xlim([0 sm_time]);
saveas(gcf, 'IMAGES/SIMULATION_B/RESPOSTA/EPS/B_PI.eps', 'epsc')
saveas(gcf, 'IMAGES/SIMULATION_B/RESPOSTA/PNG/B_PI.png')
saveas(gcf, 'IMAGES/SIMULATION_B/RESPOSTA/FIG/B_PI.fig')

figure(6)
subplot(2,1,1)
plot(t, w_ref, 'LineWidth', 2); title('Velocidade de referência (rad/s)'); xlabel('Tempo (s)'); ylabel('Velocidade de referência (rad/s)');xlim([0 sm_time]);
subplot(2,1,2)
plot(t, TL, 'LineWidth', 2); title('Torque de carga (N.m)'); xlabel('Tempo (s)'); ylabel('Torque de carga (N.m)');xlim([0 sm_time]);
saveas(gcf, 'IMAGES/SIMULATION_B/RESPOSTA/EPS/B_REFERENCIAS.eps', 'epsc')
saveas(gcf, 'IMAGES/SIMULATION_B/RESPOSTA/PNG/B_REFERENCIAS.png')
saveas(gcf, 'IMAGES/SIMULATION_B/RESPOSTA/FIG/B_REFERENCIAS.fig')


% Test: Constant Speed With Load
sm_time = 12;
SpeedTable = [0, w_nom;
              6, w_nom;
              6 + Ts, -w_nom;
              12, -w_nom]
wref = SpeedTable(:,2);
t_w = SpeedTable(:,1);
TorqueTable = [0, 0.5;
               1, 0.5;
               1 + Ts, 0.9*T_nom;
               5, 0.9*T_nom;
               5 + Ts, 0.5;
               6, 0.5;
               6 + Ts, -0.5;
               7, -0.5;
               7 + Ts, -0.9*T_nom;
               11, -0.9*T_nom;
               11 + Ts, -0.5;
               12, -0.5]
Tref = TorqueTable(:,2);
t_T = TorqueTable(:,1);

sim('SIMULATIONS/control_simulation_B.slx')
save DATA/SIMULATION_B/TEST_C.mat t w_out w_ref Ia Ib Ic Sfluxd Sfluxq TL Tem T_ref e_PI T_ref

figure(1)
plot(t, w_out, 'b', t, w_ref, 'r', 'LineWidth',2);title('Velocidade do motor (rad/s)');xlabel('Tempo (s)');ylabel('Velocidade (rad/s)'); legend('\omega_{real}','\omega_{ref}');xlim([0 sm_time]);
saveas(gcf, 'IMAGES/SIMULATION_B/RESPOSTA/EPS/C_VELOCIDADES.eps', 'epsc')
saveas(gcf, 'IMAGES/SIMULATION_B/RESPOSTA/PNG/C_VELOCIDADES.png')
saveas(gcf, 'IMAGES/SIMULATION_B/RESPOSTA/FIG/C_VELOCIDADES.fig')

figure(2)
plot(t, Ia, t, Ib, t, Ic, 'LineWidth', 2);title('Correntes do motor I_{abc}');xlabel('Tempo (s)');ylabel('Corrente (A)'); legend('I_a','I_b','I_c');xlim([0 sm_time]);
saveas(gcf, 'IMAGES/SIMULATION_B/RESPOSTA/EPS/C_CORRENTES.eps', 'epsc')
saveas(gcf, 'IMAGES/SIMULATION_B/RESPOSTA/PNG/C_CORRENTES.png')
saveas(gcf, 'IMAGES/SIMULATION_B/RESPOSTA/FIG/C_CORRENTES.fig')

figure(3)
plot(Sfluxd, Sfluxq, 'LineWidth', 2); title('Fluxo do estator (Wb)'); xlabel('Fluxo q (Wb)'); ylabel('Fluxo d (Wb)'); axis([-1.5*sf_nom 1.5*sf_nom -1.5*sf_nom 1.5*sf_nom]);
saveas(gcf, 'IMAGES/SIMULATION_B/RESPOSTA/EPS/C_FLUXOS.eps', 'epsc')
saveas(gcf, 'IMAGES/SIMULATION_B/RESPOSTA/PNG/C_FLUXOS.png')
saveas(gcf, 'IMAGES/SIMULATION_B/RESPOSTA/FIG/C_FLUXOS.fig')

figure(4)
plot(t, TL,'b', t, Tem, 'r', 'LineWidth', 2); title('Torque'); xlabel('Amostras'); ylabel('Torque (N.m)'); legend('T_{load}','T_{em}');xlim([0 sm_time]);
saveas(gcf, 'IMAGES/SIMULATION_B/RESPOSTA/EPS/C_CARGA.eps', 'epsc')
saveas(gcf, 'IMAGES/SIMULATION_B/RESPOSTA/PNG/C_CARGA.png')
saveas(gcf, 'IMAGES/SIMULATION_B/RESPOSTA/FIG/C_CARGA.fig')

figure(5)
subplot(2,1,1)
plot(t, e_PI, 'LineWidth', 2); title('Erro controlador PI (rad/s)'); xlabel('Tempo (s)'); ylabel('Erro Velocidade (rad/s)');xlim([0 sm_time]);
subplot(2,1,2)
plot(t, T_ref, 'LineWidth', 2); title('Torque de referência (N.m)'); xlabel('Tempo (s)'); ylabel('Torque de referência (N.m)');xlim([0 sm_time]);
saveas(gcf, 'IMAGES/SIMULATION_B/RESPOSTA/EPS/C_PI.eps', 'epsc')
saveas(gcf, 'IMAGES/SIMULATION_B/RESPOSTA/PNG/C_PI.png')
saveas(gcf, 'IMAGES/SIMULATION_B/RESPOSTA/FIG/C_PI.fig')

figure(6)
subplot(2,1,1)
plot(t, w_ref, 'LineWidth', 2); title('Velocidade de referência (rad/s)'); xlabel('Tempo (s)'); ylabel('Velocidade de referência (rad/s)');xlim([0 sm_time]);
subplot(2,1,2)
plot(t, TL, 'LineWidth', 2); title('Torque de carga (N.m)'); xlabel('Tempo (s)'); ylabel('Torque de carga (N.m)');xlim([0 sm_time]);
saveas(gcf, 'IMAGES/SIMULATION_B/RESPOSTA/EPS/C_REFERENCIAS.eps', 'epsc')
saveas(gcf, 'IMAGES/SIMULATION_B/RESPOSTA/PNG/C_REFERENCIAS.png')
saveas(gcf, 'IMAGES/SIMULATION_B/RESPOSTA/FIG/C_REFERENCIAS.fig')

% Test: Slow Speed With Load
sm_time = 12;
SpeedTable = [0, 1;
              6, 1;
              6 + Ts, -1;
              12, -1]
wref = SpeedTable(:,2);
t_w = SpeedTable(:,1);
TorqueTable = [0, 0.5;
               1, 0.5;
               1 + Ts, 0.9*T_nom;
               5, 0.9*T_nom;
               5 + Ts, 0.5;
               6, 0.5;
               6 + Ts, -0.5;
               7, -0.5;
               7 + Ts, -0.9*T_nom;
               11, -0.9*T_nom;
               11 + Ts, -0.5;
               12, -0.5]
Tref = TorqueTable(:,2);
t_T = TorqueTable(:,1);

sim('SIMULATIONS/control_simulation_B.slx')
save DATA/SIMULATION_B/TEST_D.mat t w_out w_ref Ia Ib Ic Sfluxd Sfluxq TL Tem T_ref e_PI T_ref

figure(1)
plot(t, w_out, 'b', t, w_ref, 'r', 'LineWidth',2);title('Velocidade do motor (rad/s)');xlabel('Tempo (s)');ylabel('Velocidade (rad/s)'); legend('\omega_{real}','\omega_{ref}');xlim([0 sm_time]);
saveas(gcf, 'IMAGES/SIMULATION_B/RESPOSTA/EPS/D_VELOCIDADES.eps', 'epsc')
saveas(gcf, 'IMAGES/SIMULATION_B/RESPOSTA/PNG/D_VELOCIDADES.png')
saveas(gcf, 'IMAGES/SIMULATION_B/RESPOSTA/FIG/D_VELOCIDADES.fig')

figure(2)
plot(t, Ia, t, Ib, t, Ic, 'LineWidth', 2);title('Correntes do motor I_{abc}');xlabel('Tempo (s)');ylabel('Current (A)'); legend('I_a','I_b','I_c');xlim([0 sm_time]);
saveas(gcf, 'IMAGES/SIMULATION_B/RESPOSTA/EPS/D_CORRENTES.eps', 'epsc')
saveas(gcf, 'IMAGES/SIMULATION_B/RESPOSTA/PNG/D_CORRENTES.png')
saveas(gcf, 'IMAGES/SIMULATION_B/RESPOSTA/FIG/D_CORRENTES.fig')

figure(3)
plot(Sfluxd, Sfluxq, 'LineWidth', 2); title('Fluxo do estator (Wb)'); xlabel('Fluxo q (Wb)'); ylabel('Fluxo d (Wb)'); axis([-1.5*sf_nom 1.5*sf_nom -1.5*sf_nom 1.5*sf_nom]);
saveas(gcf, 'IMAGES/SIMULATION_B/RESPOSTA/EPS/D_FLUXOS.eps', 'epsc')
saveas(gcf, 'IMAGES/SIMULATION_B/RESPOSTA/PNG/D_FLUXOS.png')
saveas(gcf, 'IMAGES/SIMULATION_B/RESPOSTA/FIG/D_FLUXOS.fig')

figure(4)
plot(t, TL,'b', t, Tem, 'r', 'LineWidth', 2); title('Torque'); xlabel('Amostras'); ylabel('Torque (N.m)'); legend('T_{load}','T_{em}');xlim([0 sm_time]);
saveas(gcf, 'IMAGES/SIMULATION_B/RESPOSTA/EPS/D_CARGA.eps', 'epsc')
saveas(gcf, 'IMAGES/SIMULATION_B/RESPOSTA/PNG/D_CARGA.png')
saveas(gcf, 'IMAGES/SIMULATION_B/RESPOSTA/FIG/D_CARGA.fig')

figure(5)
subplot(2,1,1)
plot(t, e_PI, 'LineWidth', 2); title('Erro controlador PI (rad/s)'); xlabel('Tempo (s)'); ylabel('Erro Velocidade (rad/s)');xlim([0 sm_time]);
subplot(2,1,2)
plot(t, T_ref, 'LineWidth', 2); title('Torque de referência (N.m)'); xlabel('Tempo (s)'); ylabel('Torque de referência (N.m)');xlim([0 sm_time]);
saveas(gcf, 'IMAGES/SIMULATION_B/RESPOSTA/EPS/D_PI.eps', 'epsc')
saveas(gcf, 'IMAGES/SIMULATION_B/RESPOSTA/PNG/D_PI.png')
saveas(gcf, 'IMAGES/SIMULATION_B/RESPOSTA/FIG/D_PI.fig')

figure(6)
subplot(2,1,1)
plot(t, w_ref, 'LineWidth', 2); title('Velocidade de referência (rad/s)'); xlabel('Tempo (s)'); ylabel('Velocidade de referência (rad/s)');xlim([0 sm_time]);
subplot(2,1,2)
plot(t, TL, 'LineWidth', 2); title('Torque de carga (N.m)'); xlabel('Tempo (s)'); ylabel('Torque de carga (N.m)');xlim([0 sm_time]);
saveas(gcf, 'IMAGES/SIMULATION_B/RESPOSTA/EPS/D_REFERENCIAS.eps', 'epsc')
saveas(gcf, 'IMAGES/SIMULATION_B/RESPOSTA/PNG/D_REFERENCIAS.png')
saveas(gcf, 'IMAGES/SIMULATION_B/RESPOSTA/FIG/D_REFERENCIAS.fig')

% Test: Blocked Rotor With Load
sm_time = 12;
SpeedTable = [0, 0;
              12, 0]
wref = SpeedTable(:,2);
t_w = SpeedTable(:,1);
TorqueTable = [0, 0.5;
               1, 0.5;
               1 + Ts, 0.9*T_nom;
               5, 0.9*T_nom;
               5 + Ts, 0.5;
               6, 0.5;
               6 + Ts, -0.5;
               7, -0.5;
               7 + Ts, -0.9*T_nom;
               11, -0.9*T_nom;
               11 + Ts, -0.5;
               12, -0.5]
Tref = TorqueTable(:,2);
t_T = TorqueTable(:,1);

sim('SIMULATIONS/control_simulation_B.slx')
save DATA/SIMULATION_B/TEST_E.mat t w_out w_ref Ia Ib Ic Sfluxd Sfluxq TL Tem T_ref e_PI T_ref

figure(1)
plot(t, w_out, 'b', t, w_ref, 'r', 'LineWidth',2);title('Velocidade do motor (rad/s)');xlabel('Tempo (s)');ylabel('Velocidade (rad/s)'); legend('\omega_{real}','\omega_{ref}');xlim([0 sm_time]);
saveas(gcf, 'IMAGES/SIMULATION_B/RESPOSTA/EPS/E_VELOCIDADES.eps', 'epsc')
saveas(gcf, 'IMAGES/SIMULATION_B/RESPOSTA/PNG/E_VELOCIDADES.png')
saveas(gcf, 'IMAGES/SIMULATION_B/RESPOSTA/FIG/E_VELOCIDADES.fig')

figure(2)
plot(t, Ia, t, Ib, t, Ic, 'LineWidth', 2);title('Correntes do motor I_{abc}');xlabel('Tempo (s)');ylabel('Corrente (A)'); legend('I_a','I_b','I_c');xlim([0 sm_time]);
saveas(gcf, 'IMAGES/SIMULATION_B/RESPOSTA/EPS/E_CORRENTES.eps', 'epsc')
saveas(gcf, 'IMAGES/SIMULATION_B/RESPOSTA/PNG/E_CORRENTES.png')
saveas(gcf, 'IMAGES/SIMULATION_B/RESPOSTA/FIG/E_CORRENTES.fig')

figure(3)
plot(Sfluxd, Sfluxq, 'LineWidth', 2); title('Fluxo do estator (Wb)'); xlabel('Fluxo q (Wb)'); ylabel('Fluxo d (Wb)'); axis([-1.5*sf_nom 1.5*sf_nom -1.5*sf_nom 1.5*sf_nom]);
saveas(gcf, 'IMAGES/SIMULATION_B/RESPOSTA/EPS/E_FLUXOS.eps', 'epsc')
saveas(gcf, 'IMAGES/SIMULATION_B/RESPOSTA/PNG/E_FLUXOS.png')
saveas(gcf, 'IMAGES/SIMULATION_B/RESPOSTA/FIG/E_FLUXOS.fig')

figure(4)
plot(t, TL,'b', t, Tem, 'r', 'LineWidth', 2); title('Torque'); xlabel('Amostras'); ylabel('Torque (N.m)'); legend('T_{load}','T_{em}');xlim([0 sm_time]);
saveas(gcf, 'IMAGES/SIMULATION_B/RESPOSTA/EPS/E_CARGA.eps', 'epsc')
saveas(gcf, 'IMAGES/SIMULATION_B/RESPOSTA/PNG/E_CARGA.png')
saveas(gcf, 'IMAGES/SIMULATION_B/RESPOSTA/FIG/E_CARGA.fig')

figure(5)
subplot(2,1,1)
plot(t, e_PI, 'LineWidth', 2); title('Erro controlador PI (rad/s)'); xlabel('Tempo (s)'); ylabel('Erro Velocidade (rad/s)');xlim([0 sm_time]);
subplot(2,1,2)
plot(t, T_ref, 'LineWidth', 2); title('Torque de referência (N.m)'); xlabel('Tempo (s)'); ylabel('Torque de referência (N.m)');xlim([0 sm_time]);
saveas(gcf, 'IMAGES/SIMULATION_B/RESPOSTA/EPS/E_PI.eps', 'epsc')
saveas(gcf, 'IMAGES/SIMULATION_B/RESPOSTA/PNG/E_PI.png')
saveas(gcf, 'IMAGES/SIMULATION_B/RESPOSTA/FIG/E_PI.fig')

figure(6)
subplot(2,1,1)
plot(t, w_ref, 'LineWidth', 2); title('Velocidade de referência (rad/s)'); xlabel('Tempo (s)'); ylabel('Velocidade de referência (rad/s)');xlim([0 sm_time]);
subplot(2,1,2)
plot(t, TL, 'LineWidth', 2); title('Torque de carga (N.m)'); xlabel('Tempo (s)'); ylabel('Torque de carga (N.m)');xlim([0 sm_time]);
saveas(gcf, 'IMAGES/SIMULATION_B/RESPOSTA/EPS/E_REFERENCIAS.eps', 'epsc')
saveas(gcf, 'IMAGES/SIMULATION_B/RESPOSTA/PNG/E_REFERENCIAS.png')
saveas(gcf, 'IMAGES/SIMULATION_B/RESPOSTA/FIG/E_REFERENCIAS.fig')