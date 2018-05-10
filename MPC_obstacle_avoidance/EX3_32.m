% Exercise 3
yalmip('clear')
clear all
close all
clc

% System
A = [1.1 2
    0 0.95];
B = [0
    0.0787];
C = [-1 1];
D = 0;
xo = [0.5;-0.5];

%number of states/control inputs
nx = 2;
nu = 1;
% Controllability matrix

C_m = [B A*B];


% Method initialization
N = 10;
Q = C_m'*C_m;   % State Penalty
R = 1;          % Input/control Penalty
Qf = Q;         % Final Penalty

M=5;

% Extras/Dummy
objective =0;
constraints = [];
x_sim = [];
x_sim(:,1)=[0.5,-0.5];

%for i = 1:M % the closed loop
    
    %X = sdpvar(nx,N);
    
    %U = sdpvar(nu,N);
    U = sdpvar(repmat(nu,1,(N)),repmat(1,1,(N)));
    X = sdpvar(repmat(nx,1,N),repmat(1,1,N));  
    
    
    for k =1:(N-1)
        X(:,1) = {xo};
        objective = objective+norm(Q*X{k},1) + norm(R*U{k},1); % Cost
        %objective = objective + norm(Q*X{:,k},1) + norm(R*U{:,k},1);
        %X(:,k+1) = A*X(:,k)+B*U(:,k);
        constraints = [constraints, -1 <= U{k}<= 1 ];
        %constraints = [constraints, X{k+1} == A*X{k} + B*U{k}];
        constraints = [constraints, X{k+1} == A*X{k}+B*U{k}];
    end
    objective = objective + norm(Qf*X{N},1);
    
    options = sdpsettings('solver','quadprog');
    diagnostic = optimize(constraints,objective,options);
    
    x_pred = double(X);
    %u_pred = double(U);

    
%    xo = [];
%end