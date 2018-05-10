% Created by Pedro Lima (pfrdal@kth.se) and Valerio Turri (turri@kth.se)
% EL2700 MPC Course Prof. Mikael Johansson
% Nonlinear MPC for vehicle control and obstacle avoidance
%%

clear; close all;
clc

%% Parameters definition
% model parameters
params.a_max                = 1;                        % acceleration limit
params.beta_max             = pi/4;                     % side slip angle limit
params.beta_dot_max         = (pi/180)*(100);           % side slip angle rate limit
params.l_f                  = 2;                        % distance between center of gravity and front axle
params.l_r                  = 2;                        % distance between center of gravity and rear axle
params.vehicle_width        = 2;                        % vehicle width
params.Ts                   = 0.1;                      % sampling time (both of MPC and simulated vehicle)
params.nstates              = 4;                        % number of states
params.ninputs              = 2;                        % number of inputs

% environment parameters
params.track_end            = 50;                       % end of the track
params.activate_obstacles   = 1;                        % 0 if there are no obstacle, 1 otherwise
params.obstacle_centers     = [10 -2; 20  0; 30 -2; 40 6]; % x and y coordinate of the 4 obstacles
params.obstacle_size        = [2 6];                    % size of the obstacles
params.lane_semiwidth       = 8;                        % semi-width of the lane

rad = [];



% control parameters
params.controller           = 'MPC';                     % 'SF' for state-feedback controller, 'MPC' for MPC controller
% ....... 
% .......
% .......
%EDIT FABIAN
%Q = eye(4);
R = 1;
Q = [1  0 0 0
    0 1 0 0
    0 0 1 0 
    0 0 0 50];
% R = 0.00001;
N = 10;
ref = [60;0;10;0];



% simulation parameters
params.x0                   = 0;                        % initial x coordinate
params.y0                   = 0;                        % initial y coordinate
params.v0                   = 10;                       % initial speed
params.psi0                 = 0;                        % initial heading angle
params.N_max                = 1000;                     % maximum number of simulation steps
%EDIT FABIAN
params.XF                   = 60;                       % terminal position to reach

% Dynamics

params.A = [1 0 params.Ts 0
    0 1 0 params.v0*params.Ts
    0 0 1 0
    0 0 0 1];
% params.B = [ 0
%      params.v0
%      0
%      params.v0/params.l_r ]*params.Ts ;
params.B = [0 0
     0 params.v0*params.Ts
     0 0
    0 (params.v0*params.Ts)/params.l_r] ;


% plotting parameters
params.window_size          = 10;                       % plot window size for the during simulation, centered at the current position (x,y) of the vehicle
params.plot_full            = 0;                        % 0 for plotting only the window size, 1 for plotting from 0 to track_end



%% Simulation environment
% initialization
z                           = zeros(params.N_max,params.nstates);             % z(k,j) denotes state j at step k-1
u                           = zeros(params.N_max,params.ninputs);             % z(k,j) denotes input j at step k-1
z(1,:)                      = [params.x0, params.y0, params.v0, params.psi0]; % definition of the intial state
comp_times                  = zeros(params.N_max);                            % total computation time each sample
solve_times                 = zeros(params.N_max);                            % optimization solver time each sample

% Edits Fabian
                         

i = 0;
while (z(i+1,1)<params.track_end) && (i<params.N_max)
    i = i+1;
    switch params.controller
        case 'SF'                                                   % state-feedbaack controller definition
            K = [0 0 0 0; 0 -0.1 0 -1];
            u(i,:) = K * z(i,:)';
            pause(0.1);
        case 'MPC'                                                  % MPC controller definition
            yalmip('clear')

            U = sdpvar(repmat(params.ninputs,1,N),repmat(1,1,N));
            Z = sdpvar(repmat(params.nstates,1,N),repmat(1,1,N));
            %z0 = sdpvar(1,4);
            %Z = z0;
            
            constraints = [];
            cost = 0;
            

            constraints = [constraints, Z{1,1} ==  z(i,:)'];          
            for k = 1:N-1                       
    
            
    % optimization
    

                cost = cost + (Z{k}-ref)'*Q*(Z{k}-ref) + (U{k})'*R*(U{k});
                %cost = cost + (Z{k}-ref)'*Q*(Z{k}-ref) ; %((U{k+1}-U{k})/params.Ts)'*R*((U{k+1}-U{k})/params.Ts);
                
                
                %input Constraints
                constraints = [constraints, -params.a_max/2  <= U{k}(1,1)<= params.a_max/2];
                constraints = [constraints, -params.beta_max/2  <= U{k}(2,1)<= params.beta_max/2];
                % Dynamic Constraints
                constraints = [constraints, Z{k+1} == params.A*Z{k} + params.B*U{k}];

                % Lane Constraints
                constraints = [constraints, -params.lane_semiwidth <= Z{k}(2,1) <= params.lane_semiwidth];
                
                
                % Linear Obstacle Constraints
                if  z(i,1) <=11  
                    constraints = [constraints, Z{k}(2,1)>= (1/2*Z{k}(1,1)-3.5)];
                               
                elseif 11< z(i,1) && z(i,1) <21
                    constraints = [constraints, Z{k}(2,1)>= (1/2*Z{k}(1,1)-6.5)]; 
                
                elseif 21< z(i,1) && z(i,1) <31
                    constraints = [constraints, Z{k}(2,1)>= (1/2*Z{k}(1,1)-13.5)]; 
%                 elseif (31< z(i,1) && z(i,1) <41)
%                     constraints = [constraints, Z{k}(2,1) <= -(1/3*Z{k}(1,1)+10)];                
                end
                
 
               
            end
            
            
            
            tic()
             %options = sdpsettings('solver','fmincon','usex0',1);
           options = sdpsettings('solver','quadprog');
            diagnostic = optimize(constraints,cost,options);
           
            comp_times(i) = toc();
            solve_times(i) = diagnostic.solvertime;
            
          
             value(U{1})
%             value(Z{1})
             u(i,:)= value(U{1});
    for j = 1:N
      ztemp(j,:) = value(Z{j});
    end
    for j = 1:N-1
                utemp(j,:) = value(U{j});
    end
    %utemp
    %ztemp

    end
    
    % simulate the vehicle
    z(i+1,:) = car_sim_LINEAR(z(i,:),u(i,:),params);
    % plot environment
    plot_environment(z(1:i,:),params);%,ztemp,utemp);
    
end

comp_times = comp_times(comp_times>0);
solve_times = solve_times(solve_times>0);


racetime                    = i*params.Ts;                          % race time
fprintf('Congratulations, you completed the race in %f seconds!\n',racetime);
params.plot_full            = 1;                                    % plot the full trajectory
plot_environment(z(1:i,:),params);
hold on
plot(comp_times)
hold on
plot(solve_times)
min(comp_times)
min(solve_times)