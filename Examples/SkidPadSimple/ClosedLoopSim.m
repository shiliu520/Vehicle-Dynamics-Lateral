%% Skid Pad Simple
% Maneuver in circles of a nonlinear simple vehicle with Pacejka tire
% model.
%
% <<SkidPadSimple.gif>>
%
%% Code start
%
clear ; close all ; clc;
addpath('/home/uisee/Desktop/uisee/octave/Vehicle-Dynamics-Lateral');
import VehicleDynamicsLateral.*

%% Model and parameters
% Simulation

T       = 30;                 % Total simulation time [s]
resol   = 300;                % Resolution
TSPAN   = 0:T/resol:T;        % Time span [s]
global_path;

%%
% Vehicle

System = VehicleSimpleNonlinear();

% Defining vehicle parameters
System.mF0      = 700;
System.mR0      = 600;
System.IT       = 10000;
System.lT       = 3.5;
System.nF       = 2;
System.nR       = 2;
System.wT       = 2;
System.muy      = 0.8;
control_method  = 1;
if control_method == 0
    System.deltaf   = @SteerControl;
else
    System.deltaf   = @SteerControl_preview;
end
System.Fxf      = 0;
System.Fxr      = @VelControl; % To keep longitudinal speed.

%%
% Tire (default)
g = 9.81;
FzF = System.mF0 * g;       % Vertical load @ F [N]
FzR = System.mR0 * g;       % Vertical load @ R [N]
camber = 0;
TireModel = TirePacejka();
% TireModel = TireLinear();
% TireModel.k = 2e4;

%% Simulation
System.tire     = TireModel;
simulator       = Simulator(System, TSPAN);
simulator.V0    = 8.333;
if path_selector == 1
    simulator.Y0    = -1.0;
else
    simulator.Y0    = -31.0;
end

simulator.Simulate();

%% Results
%

% Retrieving states
XT      = simulator.XT;
YT      = simulator.YT;
PSI     = simulator.PSI;
VEL     = simulator.VEL;
ALPHAT  = simulator.ALPHAT;
dPSI    = simulator.dPSI;

error_cg = [];
error_pre = [];
for i = 1: length(XT)
    X = XT(i);
    Y = YT(i);
    PSI_ = PSI(i);
    vx = VEL(i) * cos(ALPHAT(i));
    vy = VEL(i) * sin(ALPHAT(i));
    dPSI_ = dPSI(i);
    [kr, err] = error_calc(X, Y, PSI_, vx, vy, dPSI_, s, d, heading_angle, curvature);
    L = 10;
    [e_d, e_psi, kappa] = error_calc_preview(X, Y, PSI_, L, s, d, heading_angle, curvature);
    % error = [error; [err(1), err(3)]];
    error_cg = [error_cg; [err(1), err(3) + atan(vy/vx)]];
    error_pre = [error_pre; [e_d, e_psi]];
end

% 提取横偏和航偏数据
lateral_error_cg = error_cg(:, 1);
heading_error_cg = error_cg(:, 2);
lateral_error_pre = error_pre(:, 1);
heading_error_pre = error_pre(:, 2);
% 创建一个新的图形窗口
figure(1);

% 绘制横偏误差曲线
subplot(2, 1, 1); hold on;
plot(TSPAN, lateral_error_cg, 'b', 'LineWidth', 1.5);
plot(TSPAN, lateral_error_pre, 'k', 'LineWidth', 1.5);
legend('e\_d\_{cg}', 'e\_d\_{pre}');
title('横向偏差');
xlabel('时间步');
ylabel('横向偏差');
grid on;

% 绘制航偏误差曲线
subplot(2, 1, 2); hold on;
plot(TSPAN, heading_error_cg, 'r', 'LineWidth', 1.5);
plot(TSPAN, heading_error_pre, 'k', 'LineWidth', 1.5);
legend('e\_psi\_{cg}', 'e\_psi\_{pre}');
title('航向偏差');
xlabel('时间步');
ylabel('航向偏差');
grid on;

figure(2)
hold on ; grid on ; box on
plot(s, d, 'k');
plot(XT, YT, 'b');
xlabel('X/[m]')
ylabel('Y/[m]')
L = 10.0;
[x_preview, y_preview] = find_preview_point(XT, YT, PSI, L);
plot(x_preview, y_preview, 'r');
legend('ref\_path', 'veh\_path\_cg', 'veh\_path\_pre');


figure(3)
hold on ; grid on ; box on
plot(TSPAN,PSI)
xlabel('time [s]')
ylabel('Yaw angle [rad]')

figure(4)
hold on ; grid on ; box on
plot(TSPAN,VEL)
xlabel('time [s]')
ylabel('Velocity [m/s]')

figure(5)
hold on ; grid on ; box on
plot(TSPAN,ALPHAT)
xlabel('time [s]')
ylabel('Vehicle slip angle [rad]')

figure(6)
hold on ; grid on ; box on
plot(TSPAN,dPSI)
xlabel('time [s]')
ylabel('Yaw rate [rad/s]')

%% Graphics
% Frame and animation

g = Graphics(simulator);
g.TractorColor = 'y';

g.Frame();


% g.Animation();
% g.Animation('html/SkidPadSimple');       % Uncomment to save animation gif