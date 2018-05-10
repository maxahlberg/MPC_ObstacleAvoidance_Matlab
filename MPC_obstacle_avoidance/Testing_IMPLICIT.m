yalmip('clear')
clear all
clear all 
clc
% Model data
A = [2 -1;1 0.2];
B = [1,0
    0,1];
nx = 2; % Number of states
nu = 2; % Number of inputs

% MPC data
Q = eye(2);
R = 2;
N = 7;


Cont=[];
States = [];
j = 0;
xnew = [3;1];
while j <15
   
 j = j+1;
 
 
u = sdpvar(repmat(nu,1,N),repmat(1,1,N));
x0 = sdpvar(2,1);
%x = sdpvar(2,1);

constraints = [];
objective = 0;
x = x0;
for k = 1:N
 x = A*x + B*u{k};
 objective = objective + norm(Q*x,1) + norm(R*u{k},1);
 constraints = [constraints, -1 <= u{k}<= 1, -5<=x<=5];
 %constraints = [constraints, -1 <= u{k}<= 1, -5<=x<=5, x == A*x+B*u{k}];
end

ops = sdpsettings('solver','fmincon','usex0',1);
controller = optimizer(constraints,objective,ops,x,u{1});

x = xnew
%for i = 1:5
 uk = controller{x};
 Cont = [Cont,uk]
 x = A*x + B*uk;
 States = [States, x];
 
%end
xnew = x;
end




