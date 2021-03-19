clc; clear; close all;

% empirical data matrices
tune_a13 = 1;
tune_b2 = 1;  % tuning parameter because angular velocity still fucky
A = [0 0 .7*tune_a13 0; 0 0 0 0; 0 0 0 1; 0 0 .0502 0];
B = [.0119 .0119; .062*tune_b2 -.062*tune_b2; 0 0; -.0603 -.0603];
C = eye(4);
D = zeros(4,2);

% discretization
ts = .02;
SYSd_emp = c2d(ss(A,B,C,D),ts);

% controller design
r = 5e-7;
R = eye(2)*r;
Q = eye(4);
Q(1,1) = 10;            % v
Q(2,2) = 100;           % omega
Q(3,3) = 1;             % phi
Q(4,4) = 100000;        % phidot

K = lqr(SYSd_emp,C,R);
disp(['K = [1; 1]*[',num2str(K(1,1)),' ',num2str(K(1,2)),' ',...
    num2str(K(1,3)),' ',num2str(K(1,4)),'];']);
% copy that K and put it into segway_lqr_v3.m

