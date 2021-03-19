%% LQR IMPLEMENTATION FOR 2-WHEELED SEGWAY MODEL
% JEREMY ENGELS FOR UCLA MAE 162D
clc; close all; clear;

%% set sim times
ts = 1; % ms
T = 15000; % ms
N = T/ts;

%% Getting motor and sensor labels
motor_R = wb_robot_get_device('motor_R');
motor_L = wb_robot_get_device('motor_L');

robot = wb_supervisor_node_get_from_def('Robot'); % for ground truth access
 
%% library initialization
t = linspace(1,T,N);
U = zeros(2,N);
E = zeros(4,N);
POSITION = zeros(3,N);
VELOCITY = zeros(3,N);
GYRO = zeros(3,N);
IMU = zeros(3,N);
V = zeros(1,N);

%% sensor initialization
gyro = wb_robot_get_device('gyro');
imu = wb_robot_get_device('imu');
wb_gyro_enable(gyro,ts);
wb_inertial_unit_enable(imu,ts);

%% controller setup

% initial torque inputs
U(:,1) = [0; 0];

% reference velocities
rOmega = 1;
rV = 1;

% controller gain matrix 
% GET THIS MATRIX BY RUNNING lqrcontroller_v2.m
K = [1; 1]*[-349.6918 352.9846 -583.604 -438.7241];
K(2,2) = -K(2,2);

i = 1;
while wb_robot_step(ts) ~= -1
    % send torque info to motors
    wb_motor_set_torque(motor_L, U(2,i)); % rad/s
    wb_motor_set_torque(motor_R, U(1,i)); % rad/s
  
    % get ground truth data
    position = wb_supervisor_node_get_position(robot);
    velocity = wb_supervisor_node_get_velocity(robot);
    POSITION(:,i) = position;
    VELOCITY(:,i) = velocity;
  
    % get sensor data
    GYRO(:,i) = wb_gyro_get_values(gyro);
    IMU(:,i) = wb_inertial_unit_get_roll_pitch_yaw(imu);
  
    % organize data into my variables
    Omega = GYRO(2,i);
    phi = -IMU(1,i);
    phidot = -GYRO(1,i);
  
    % build error vector
    ephi = -phi - .0132;        % might be minus bias
    ephidot = - phidot;
    V(i) = sqrt(VELOCITY(1,i)^2 + VELOCITY(3,i)^2);
    eV = rV - V(i);
    eOmega = rOmega - Omega;
    e = [eV eOmega ephi ephidot]';
    E(:,i) = e;
    
    % run controller
    U(:,i+1) = -K*e;
    
    % end condition
    i = i + 1;
    
    if i*ts >= T
        wb_robot_step(ts) = -1;
    end
    
end

% opens workspace in matlab
desktop;
keyboard;

%% plots
V = sqrt(VELOCITY(1,:).^2 + VELOCITY(3,:).^2);
% figure
% plot(t,-IMU(1,:),t,IMU(3,:))
% legend('phi','theta')
% title('angle')

figure
plot(t,-U(1,:),t,-U(2,:))
legend('right','left')
title('torque')

figure
plot(t,E)
legend('v','Omega','phi','phidot')
title('error')

figure
plot(t,V,t,GYRO(2,:),t,-IMU(1,:),t,-GYRO(1,:))
legend('V','Omega','phi','phidot')
title('outputs')

%%
COLORS = get(gca,'colororder');
plotdefaults(14,1,1.2,'northeast');
Omega = GYRO(2,:);
phi = -IMU(1,:);
phidot = -GYRO(1,:);
tsec = t/1000;
xrange = [0 10];

figure(1)
subplot(1,2,1)
plot(tsec,V,tsec,Omega,tsec,phi,tsec,phidot)
hold on
plot(tsec,ones(1,length(tsec)),'k--','color',COLORS(2,:))
hold off
legend('$V$','$\Omega$','$\phi$','$\dot\phi$')
xlim(xrange)
xlabel('Time, $t$ [s]')
ylabel('State Outputs')

subplot(1,2,2)
plot(tsec,-U(1,:),tsec,-U(2,:))
legend('$\tau_R$','$\tau_L$')
xlabel('Time, $t$ [s]')
ylabel('Torque Commands [N-m]')
xlim(xrange)

figure(1)
tightfig;
saveas(gcf,'Vel_Om_Step_Both.pdf')
