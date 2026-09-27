%% 3DOF Robot - Velocity Calculation

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


%% Joint positions

theta1 = 30;
theta2 = 20;
theta3 = 10;

% Convert degrees to radians

theta1 = deg2rad(theta1);
theta2 = deg2rad(theta2);
theta3 = deg2rad(theta3);


%% Joint velocities

theta1_dot = 10;
theta2_dot = 5;
theta3_dot = 2;

% Convert degrees per second to radians per second

theta1_dot = deg2rad(theta1_dot);
theta2_dot = deg2rad(theta2_dot);
theta3_dot = deg2rad(theta3_dot);

thetaDot = [theta1_dot;
            theta2_dot;
            theta3_dot];


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


%% End-effector velocity

V = Js * thetaDot;


%% Separate angular and linear velocity

omega = V(1:3);

linearVelocity = V(4:6);


%% Display results

disp('==============================');
disp('3DOF ROBOT VELOCITY');
disp('==============================');

fprintf('\nJoint velocities:\n');

fprintf('Theta1_dot = %.4f rad/s\n', theta1_dot);
fprintf('Theta2_dot = %.4f rad/s\n', theta2_dot);
fprintf('Theta3_dot = %.4f rad/s\n', theta3_dot);

fprintf('\nEnd-effector angular velocity:\n');

fprintf('Wx = %.4f rad/s\n', omega(1));
fprintf('Wy = %.4f rad/s\n', omega(2));
fprintf('Wz = %.4f rad/s\n', omega(3));

fprintf('\nEnd-effector linear velocity:\n');

fprintf('Vx = %.4f m/s\n', linearVelocity(1));
fprintf('Vy = %.4f m/s\n', linearVelocity(2));
fprintf('Vz = %.4f m/s\n', linearVelocity(3));