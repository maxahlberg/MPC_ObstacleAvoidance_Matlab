% Exercise 3
yalmip('clear')
clear all
close all
clc

% System

% Model data
A = [2 -1;1 0.2];
B = [1;0];
C = [1,0];
D=0;
sys = ss(A,B,C,D);
nx = 2; % Number of states
nu = 1; % Number of inputs

% MPC data
Q = eye(2);
R = 2;
N = 7;

% Initial state
x0 = [3;1];


u = sdpvar(repmat(nu,1,N),repmat(1,1,N));
x = sdpvar(repmat(nx,1,N+1),repmat(1,1,N+1));

constraints = [];
objective = 0;
for k = 1:N
 
    
 objective = objective + norm(Q*x{k},1) + norm(R*u{k},1);
 constraints = [constraints, x{k+1} == A*x{k}+B*u{k}];
 constraints = [constraints, -1 <= u{k}<= 1, -5<=x{k+1}<=5];
end

optimize(constraints,objective);
value(u{1})
u_p = double(u)
x_p = double(x)
t = 0:0.1:5;
%lsim(sys,u_p,t)