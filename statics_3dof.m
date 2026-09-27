%% 3DOF Robot - Static Torque Calculation

clc;
clear;
close all;

%% Robot dimensions

l1 = 0.30;
l2 = 0.40;
l3 = 0.30;


%% Space screw axes

S1 = [0; 0; 1; 0; 0; 0];

S2 = [1; 0; 0; 0; 0.30; 0];

S3 = [1; 0; 0; 0; 0.30; 0];


%% Home configuration

M = [1 0 0 0.70;
    0 1 0 0;
    0 0 1 0.30;
    0 0 0 1];


%% Robot configuration

theta1 = 30;
theta2 = 20;
theta3 = 10;

theta1 = deg2rad(theta1);
theta2 = deg2rad(theta2);
theta3 = deg2rad(theta3);


%% Payload

mass = 1.0;       % kg

g = 9.81;          % m/s^2

% Downward gravitational force

Fz = -mass * g;


%% Space Jacobian

c1 = cos(theta1);
s1 = sin(theta1);

c2 = cos(theta2);
s2 = sin(theta2);

Js = [0,      c1,              c1;
    0,      s1,              s1;
    1,      0,               0;
    0, -l1*s1,       -l1*s1*c2;
    0,  l1*c1,        l1*c1*c2;
    0,      0,           l1*s2];


%% External wrench

F = [0;
    0;
    0;
    0;
    0;
    Fz];


%% Joint torques

tau = Js' * F;


%% Display results

disp('==============================');
disp('3DOF ROBOT STATIC ANALYSIS');
disp('==============================');

fprintf('\nPayload mass: %.2f kg\n',mass);

fprintf('Gravity force: %.2f N\n',Fz);

fprintf('\nJoint torques:\n');

fprintf('Tau 1 = %.4f Nm\n',tau(1));

fprintf('Tau 2 = %.4f Nm\n',tau(2));

fprintf('Tau 3 = %.4f Nm\n',tau(3));