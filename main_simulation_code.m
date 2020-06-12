clear all; close all; clc;
%% Main Code
global param
Ts = 1e-5;
% PI speed controller parameters
Kp = 70;
Ki = 35;

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


%% Control Parameters
Mp = 3/100;
ts = 0.2;
zeta = sqrt(log(Mp)^2/(log(Mp)^2 + pi^2))
wn = log(0.05)/(-zeta*ts)

Pole = -wn*zeta+j*wn*sqrt(1-zeta^2)

%% Controler - Calculation
s = tf('s');
RotorTF = (1/J)/(s  + F/J)
RotorPOLES = pole(RotorTF)

C_s = (150*s + 75)/s;
% OpenLoopTF = C_s*RotorTF
% 
% NumGain = 0;
% DenGain = 0;
% syms x
% for i = 1:length(OpenLoopTF.Numerator{1})
%     if OpenLoopTF.Numerator{1}(i) ~= 0
%         NumGain = NumGain + OpenLoopTF.Numerator{1}(i)*(x^(length(OpenLoopTF.Numerator{1})-i));
%     end
%     if OpenLoopTF.Denominator{1}(i) ~= 0
%         DenGain = DenGain + OpenLoopTF.Denominator{1}(i)*(x^(length(OpenLoopTF.Numerator{1})-i));
%     end
% end
% disp(NumGain)
% disp(DenGain)
% 
% Gain = DenGain/NumGain;
% Gain = abs(real(double(subs(Gain, Pole))))
% C_s = C_s * Gain
% 
% OpenLoopTF = RotorTF*C_s
% 
% AA = pole(OpenLoopTF)
% 
% CL_TF = feedback(RotorTF*C_s,1)
% stepinfo(CL_TF)
% 
% figure
% step(CL_TF)

% Discretization

% C_z = c2d(C_s,TsController)

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

%% Simulation - A
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

sim('control_simulation.slx')

figure(1)
plot(t, w_out, 'b', t, w_ref, 'r', 'LineWidth',2);title('Speed Real (rad/s)');xlabel('Tempo (s)');ylabel('Velocidade (rad/s)'); legend('\omega_{real}','\omega_{ref}');xlim([0 sm_time]);
saveas(gcf, 'IMAGES/EPS/A_VELOCIDADES.eps')
saveas(gcf, 'IMAGES/PNG/A_VELOCIDADES.png')


figure(2)
plot(t, Ia, t, Ib, t, Ic, 'LineWidth', 2);title('Motor Currents I_{abc}');xlabel('Tempo (s)');ylabel('Current (A)'); legend('I_a','I_b','I_c');xlim([0 sm_time]);
saveas(gcf, 'IMAGES/EPS/A_CORRENTES.eps')
saveas(gcf, 'IMAGES/PNG/A_CORRENTES.png')

figure(3)
plot(Sfluxd, Sfluxq, 'LineWidth', 2); title('Fluxo de estator (Wb)'); xlabel('Fluxo q (Wb)'); ylabel('Fluxo d (Wb)'); axis([-sf_nom sf_nom -sf_nom sf_nom]);
saveas(gcf, 'IMAGES/EPS/A_FLUXOS.eps')
saveas(gcf, 'IMAGES/PNG/A_FLUXOS.png')


figure(4)
plot(t, TL,'b', t, Tem, 'r', 'LineWidth', 2); title('Torque'); xlabel('Amostras'); ylabel('Torque (N.m)'); legend('T_{load}','T_{em}');xlim([0 sm_time]);
saveas(gcf, 'IMAGES/EPS/A_CARGA.eps')
saveas(gcf, 'IMAGES/PNG/A_CARGA.png')


figure(5)
subplot(2,1,1)
plot(t, e_PI, 'LineWidth', 2); title('Erro controlador PI (rad/s)'); xlabel('Tempo (s)'); ylabel('Erro Velocidade (rad/s)');xlim([0 sm_time]);
subplot(2,1,2)
plot(t, T_ref, 'LineWidth', 2); title('Torque de referencia (N.m)'); xlabel('Tempo (s)'); ylabel('Torque de referencia (N.m)');xlim([0 sm_time]);
saveas(gcf, 'IMAGES/EPS/A_PI.eps')
saveas(gcf, 'IMAGES/PNG/A_PI.png')

figure(6)
subplot(2,1,1)
plot(t, w_ref, 'LineWidth', 2); title('Velocidade de referencia (rad/s)'); xlabel('Tempo (s)'); ylabel('Velocidade de referencia (rad/s)');xlim([0 sm_time]);
subplot(2,1,2)
plot(t, TL, 'LineWidth', 2); title('Torque de carga (N.m)'); xlabel('Tempo (s)'); ylabel('Torque de carga (N.m)');xlim([0 sm_time]);
saveas(gcf, 'IMAGES/EPS/A_REFERENCIAS.eps')
saveas(gcf, 'IMAGES/PNG/A_REFERENCIAS.png')

%% Simulation - B
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

sim('control_simulation.slx')

figure(1)
plot(t, w_out, 'b', t, w_ref, 'r', 'LineWidth',2);title('Speed Real (rad/s)');xlabel('Tempo (s)');ylabel('Velocidade (rad/s)'); legend('\omega_{real}','\omega_{ref}');xlim([0 sm_time]);
saveas(gcf, 'IMAGES/EPS/B_VELOCIDADES.eps')
saveas(gcf, 'IMAGES/PNG/B_VELOCIDADES.png')


figure(2)
plot(t, Ia, t, Ib, t, Ic, 'LineWidth', 2);title('Motor Currents I_{abc}');xlabel('Tempo (s)');ylabel('Current (A)'); legend('I_a','I_b','I_c');xlim([0 sm_time]);
saveas(gcf, 'IMAGES/EPS/B_CORRENTES.eps')
saveas(gcf, 'IMAGES/PNG/B_CORRENTES.png')

figure(3)
plot(Sfluxd, Sfluxq, 'LineWidth', 2); title('Fluxo de estator (Wb)'); xlabel('Fluxo q (Wb)'); ylabel('Fluxo d (Wb)'); axis([-sf_nom sf_nom -sf_nom sf_nom]);
saveas(gcf, 'IMAGES/EPS/B_FLUXOS.eps')
saveas(gcf, 'IMAGES/PNG/B_FLUXOS.png')


figure(4)
plot(t, TL,'b', t, Tem, 'r', 'LineWidth', 2); title('Torque'); xlabel('Amostras'); ylabel('Torque (N.m)'); legend('T_{load}','T_{em}');xlim([0 sm_time]);
saveas(gcf, 'IMAGES/EPS/B_CARGA.eps')
saveas(gcf, 'IMAGES/PNG/B_CARGA.png')


figure(5)
subplot(2,1,1)
plot(t, e_PI, 'LineWidth', 2); title('Erro controlador PI (rad/s)'); xlabel('Tempo (s)'); ylabel('Erro Velocidade (rad/s)');xlim([0 sm_time]);
subplot(2,1,2)
plot(t, T_ref, 'LineWidth', 2); title('Torque de referencia (N.m)'); xlabel('Tempo (s)'); ylabel('Torque de referencia (N.m)');xlim([0 sm_time]);
saveas(gcf, 'IMAGES/EPS/B_PI.eps')
saveas(gcf, 'IMAGES/PNG/B_PI.png')

figure(6)
subplot(2,1,1)
plot(t, w_ref, 'LineWidth', 2); title('Velocidade de referencia (rad/s)'); xlabel('Tempo (s)'); ylabel('Velocidade de referencia (rad/s)');xlim([0 sm_time]);
subplot(2,1,2)
plot(t, TL, 'LineWidth', 2); title('Torque de carga (N.m)'); xlabel('Tempo (s)'); ylabel('Torque de carga (N.m)');xlim([0 sm_time]);
saveas(gcf, 'IMAGES/EPS/B_REFERENCIAS.eps')
saveas(gcf, 'IMAGES/PNG/B_REFERENCIAS.png')

%% Simulation - C
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

sim('control_simulation.slx')

figure(1)
plot(t, w_out, 'b', t, w_ref, 'r', 'LineWidth',2);title('Speed Real (rad/s)');xlabel('Tempo (s)');ylabel('Velocidade (rad/s)'); legend('\omega_{real}','\omega_{ref}');xlim([0 sm_time]);
saveas(gcf, 'IMAGES/EPS/C_VELOCIDADES.eps')
saveas(gcf, 'IMAGES/PNG/C_VELOCIDADES.png')


figure(2)
plot(t, Ia, t, Ib, t, Ic, 'LineWidth', 2);title('Motor Currents I_{abc}');xlabel('Tempo (s)');ylabel('Current (A)'); legend('I_a','I_b','I_c');xlim([0 sm_time]);
saveas(gcf, 'IMAGES/EPS/C_CORRENTES.eps')
saveas(gcf, 'IMAGES/PNG/C_CORRENTES.png')

figure(3)
plot(Sfluxd, Sfluxq, 'LineWidth', 2); title('Fluxo de estator (Wb)'); xlabel('Fluxo q (Wb)'); ylabel('Fluxo d (Wb)'); axis([-sf_nom sf_nom -sf_nom sf_nom]);
saveas(gcf, 'IMAGES/EPS/C_FLUXOS.eps')
saveas(gcf, 'IMAGES/PNG/C_FLUXOS.png')


figure(4)
plot(t, TL,'b', t, Tem, 'r', 'LineWidth', 2); title('Torque'); xlabel('Amostras'); ylabel('Torque (N.m)'); legend('T_{load}','T_{em}');xlim([0 sm_time]);
saveas(gcf, 'IMAGES/EPS/C_CARGA.eps')
saveas(gcf, 'IMAGES/PNG/C_CARGA.png')


figure(5)
subplot(2,1,1)
plot(t, e_PI, 'LineWidth', 2); title('Erro controlador PI (rad/s)'); xlabel('Tempo (s)'); ylabel('Erro Velocidade (rad/s)');xlim([0 sm_time]);
subplot(2,1,2)
plot(t, T_ref, 'LineWidth', 2); title('Torque de referencia (N.m)'); xlabel('Tempo (s)'); ylabel('Torque de referencia (N.m)');xlim([0 sm_time]);
saveas(gcf, 'IMAGES/EPS/C_PI.eps')
saveas(gcf, 'IMAGES/PNG/C_PI.png')

figure(6)
subplot(2,1,1)
plot(t, w_ref, 'LineWidth', 2); title('Velocidade de referencia (rad/s)'); xlabel('Tempo (s)'); ylabel('Velocidade de referencia (rad/s)');xlim([0 sm_time]);
subplot(2,1,2)
plot(t, TL, 'LineWidth', 2); title('Torque de carga (N.m)'); xlabel('Tempo (s)'); ylabel('Torque de carga (N.m)');xlim([0 sm_time]);
saveas(gcf, 'IMAGES/EPS/C_REFERENCIAS.eps')
saveas(gcf, 'IMAGES/PNG/C_REFERENCIAS.png')

%% Simulation - D
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

sim('control_simulation.slx')

figure(1)
plot(t, w_out, 'b', t, w_ref, 'r', 'LineWidth',2);title('Speed Real (rad/s)');xlabel('Tempo (s)');ylabel('Velocidade (rad/s)'); legend('\omega_{real}','\omega_{ref}');xlim([0 sm_time]);
saveas(gcf, 'IMAGES/EPS/D_VELOCIDADES.eps')
saveas(gcf, 'IMAGES/PNG/D_VELOCIDADES.png')


figure(2)
plot(t, Ia, t, Ib, t, Ic, 'LineWidth', 2);title('Motor Currents I_{abc}');xlabel('Tempo (s)');ylabel('Current (A)'); legend('I_a','I_b','I_c');xlim([0 sm_time]);
saveas(gcf, 'IMAGES/EPS/D_CORRENTES.eps')
saveas(gcf, 'IMAGES/PNG/D_CORRENTES.png')

figure(3)
plot(Sfluxd, Sfluxq, 'LineWidth', 2); title('Fluxo de estator (Wb)'); xlabel('Fluxo q (Wb)'); ylabel('Fluxo d (Wb)'); axis([-sf_nom sf_nom -sf_nom sf_nom]);
saveas(gcf, 'IMAGES/EPS/D_FLUXOS.eps')
saveas(gcf, 'IMAGES/PNG/D_FLUXOS.png')


figure(4)
plot(t, TL,'b', t, Tem, 'r', 'LineWidth', 2); title('Torque'); xlabel('Amostras'); ylabel('Torque (N.m)'); legend('T_{load}','T_{em}');xlim([0 sm_time]);
saveas(gcf, 'IMAGES/EPS/D_CARGA.eps')
saveas(gcf, 'IMAGES/PNG/D_CARGA.png')


figure(5)
subplot(2,1,1)
plot(t, e_PI, 'LineWidth', 2); title('Erro controlador PI (rad/s)'); xlabel('Tempo (s)'); ylabel('Erro Velocidade (rad/s)');xlim([0 sm_time]);
subplot(2,1,2)
plot(t, T_ref, 'LineWidth', 2); title('Torque de referencia (N.m)'); xlabel('Tempo (s)'); ylabel('Torque de referencia (N.m)');xlim([0 sm_time]);
saveas(gcf, 'IMAGES/EPS/D_PI.eps')
saveas(gcf, 'IMAGES/PNG/D_PI.png')

figure(6)
subplot(2,1,1)
plot(t, w_ref, 'LineWidth', 2); title('Velocidade de referencia (rad/s)'); xlabel('Tempo (s)'); ylabel('Velocidade de referencia (rad/s)');xlim([0 sm_time]);
subplot(2,1,2)
plot(t, TL, 'LineWidth', 2); title('Torque de carga (N.m)'); xlabel('Tempo (s)'); ylabel('Torque de carga (N.m)');xlim([0 sm_time]);
saveas(gcf, 'IMAGES/EPS/D_REFERENCIAS.eps')
saveas(gcf, 'IMAGES/PNG/D_REFERENCIAS.png')

%% Simulation - E
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

sim('control_simulation.slx')

figure(1)
plot(t, w_out, 'b', t, w_ref, 'r', 'LineWidth',2);title('Speed Real (rad/s)');xlabel('Tempo (s)');ylabel('Velocidade (rad/s)'); legend('\omega_{real}','\omega_{ref}');xlim([0 sm_time]);
saveas(gcf, 'IMAGES/EPS/E_VELOCIDADES.eps')
saveas(gcf, 'IMAGES/PNG/E_VELOCIDADES.png')


figure(2)
plot(t, Ia, t, Ib, t, Ic, 'LineWidth', 2);title('Motor Currents I_{abc}');xlabel('Tempo (s)');ylabel('Current (A)'); legend('I_a','I_b','I_c');xlim([0 sm_time]);
saveas(gcf, 'IMAGES/EPS/E_CORRENTES.eps')
saveas(gcf, 'IMAGES/PNG/E_CORRENTES.png')

figure(3)
plot(Sfluxd, Sfluxq, 'LineWidth', 2); title('Fluxo de estator (Wb)'); xlabel('Fluxo q (Wb)'); ylabel('Fluxo d (Wb)'); axis([-sf_nom sf_nom -sf_nom sf_nom]);
saveas(gcf, 'IMAGES/EPS/E_FLUXOS.eps')
saveas(gcf, 'IMAGES/PNG/E_FLUXOS.png')


figure(4)
plot(t, TL,'b', t, Tem, 'r', 'LineWidth', 2); title('Torque'); xlabel('Amostras'); ylabel('Torque (N.m)'); legend('T_{load}','T_{em}');xlim([0 sm_time]);
saveas(gcf, 'IMAGES/EPS/E_CARGA.eps')
saveas(gcf, 'IMAGES/PNG/E_CARGA.png')


figure(5)
subplot(2,1,1)
plot(t, e_PI, 'LineWidth', 2); title('Erro controlador PI (rad/s)'); xlabel('Tempo (s)'); ylabel('Erro Velocidade (rad/s)');xlim([0 sm_time]);
subplot(2,1,2)
plot(t, T_ref, 'LineWidth', 2); title('Torque de referencia (N.m)'); xlabel('Tempo (s)'); ylabel('Torque de referencia (N.m)');xlim([0 sm_time]);
saveas(gcf, 'IMAGES/EPS/E_PI.eps')
saveas(gcf, 'IMAGES/PNG/E_PI.png')

figure(6)
subplot(2,1,1)
plot(t, w_ref, 'LineWidth', 2); title('Velocidade de referencia (rad/s)'); xlabel('Tempo (s)'); ylabel('Velocidade de referencia (rad/s)');xlim([0 sm_time]);
subplot(2,1,2)
plot(t, TL, 'LineWidth', 2); title('Torque de carga (N.m)'); xlabel('Tempo (s)'); ylabel('Torque de carga (N.m)');xlim([0 sm_time]);
saveas(gcf, 'IMAGES/EPS/E_REFERENCIAS.eps')
saveas(gcf, 'IMAGES/PNG/E_REFERENCIAS.png')