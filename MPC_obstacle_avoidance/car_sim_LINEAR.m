function z_new = car_sim_LINEAR(z_curr,u_curr,params)
%% CAR_SIM simulates a car-like vehicle
%
% z_new = CAR_SIM(z_curr,u_curr,params) computes the new vehicle states
% (x,y,v,\psi) give the current state z_curr and the current input
% (a,\beta). 
% PARAMS is a structure with at least the following parameters:
%    * params.a_max, acceleration limit
%    * params.beta_max, side slip angle limit 
%    * params.beta_dot_max, side slip angle rate limit
%    * params.Ts, sampling time (both of MPC and simulated vehicle)
%
% Created by Pedro Lima (pfrdal@kth.se) and  Valerio Turri (turri@kth.se),
% 23/09/2016 for the EL2700 - MPC Course @ KTH, Sweden

persistent old_beta;                        % old_beta memorizes the previous beta
if isempty(old_beta)
    old_beta = 0; 
end

% imposing the phisical saturation on the inputs
a           = u_curr(1);
beta        = u_curr(1);
a           = min(params.a_max, max(-params.a_max, a));                 % saturation on the acceleration
beta        = min(params.beta_max, max(-params.beta_max, beta));        % saturation on the sideangle
beta        = min(old_beta+params.beta_dot_max*params.Ts, max(old_beta-params.beta_dot_max*params.Ts, beta)); % saturation on the sideangle rate

% vehicle dynamics equations
% z_new(1) = z_curr(1) + (z_curr(3)*cos(z_curr(4)+beta))*params.Ts;
% z_new(2) = z_curr(2) + (z_curr(3)*sin(z_curr(4)+beta))*params.Ts;
% z_new(3) = z_curr(3) + a*params.Ts;
% z_new(4) = z_curr(4) + (z_curr(3)*sin(beta)/params.l_r)*params.Ts;
z_new = params.A*z_curr' + params.B*u_curr';
% saving the current beta 
old_beta = beta;
end