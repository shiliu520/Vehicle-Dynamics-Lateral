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

T       = 10;                 % Total simulation time [s]
resol   = 100;                % Resolution
TSPAN   = 0:T/resol:T;        % Time span [s]

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
System.deltaf   = 20*pi/180 * ones(size(TSPAN));
System.deltaf(1:11) = 0;
System.Fxf      = 0;
System.Fxr      = @VelControl; % To keep longitudinal speed.

%%
% Tire (default)
g = 9.81;
FzF = System.mF0 * g;       % Vertical load @ F [N]
FzR = System.mR0 * g;       % Vertical load @ R [N]
camber = 0;
TireModel = TirePacejka();

%% Simulation
%

System.tire     = TireModel;
simulator       = Simulator(System, TSPAN);
simulator.V0    = 8.333;

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

figure(1)
hold on ; grid on ; box on
plot(TSPAN,XT)
xlabel('time [s]')
ylabel('Distance in the x direction [m]')

figure(2)
hold on ; grid on ; box on
plot(TSPAN,YT)
xlabel('time [s]')
ylabel('Distance in the y direction [m]')

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

angulo = 0:0.01:2*pi;

[R,XC,YC] = circfit(XT(40:end),YT(40:end));

XX = XC + R*cos(angulo);
YY = YC + R*sin(angulo);

hold on
plot(XX,YY,'k', 'linewidth', 2);

% g.Animation();
% g.Animation('html/SkidPadSimple');       % Uncomment to save animation gif

%%
% Maneuver radius

% disp(num2str(R))
fprintf('turning radius[m]: %f\n', R);
fprintf('steady beta nonlinear mode [rad]: %f\n', mean(ALPHAT(end-10:end)));
fprintf('steady yaw rate nonlinear mode [rad/s]: %f\n\n', mean(dPSI(end-10:end)));
ss_simulation;
figure(5);
plot(t,y1(:,1),'r');
legend('nonlinear\_model', 'linear\_model');
figure(6);
plot(t,y1(:,2),'r');
legend('nonlinear\_model', 'linear\_model');

%% See Also
%
% <../../../index.html Home> |
% <../../SkidPadSimple4DOF/html/SkidPadSimple4DOF.html
% SkidPad Simple 4DOF>
%

% params identify
delta_omega = iddata(dPSI, U', T/resol);
np = 2; % ARX模型分母阶数（对应极点）
nz = 1; % ARX模型分子阶数（对应零点）
Options = tfestOptions;                           
Options.Display = 'on';     
Options.SearchOption.MaxIter = 20;
Options.SearchMethod = 'auto';
tf_delta_omega = tfest(delta_omega, np, nz, Options);
disp('辨识后的连续系统传递函数:');
tf_delta_omega
[omega_i, t] = lsim(tf_delta_omega, U, TSPAN);
figure(6);
plot(t, omega_i, 'k');
legend('nonlinear\_model', 'linear\_model', 'linear\_model\_identify');