function plot_environment(z,params,varargin)
%% PLOT_ENVIRONMENT Plot autonomous driving environment
% PLOT_ENVIRONMENT(z,params) plots the current vehicle position together with the road
% environment defined in the structure params.
%
% PLOT_ENVIRONMENT(z,params,z_predicted,u_predicted) is the same as PLOT(z,params) but also
% plots the predicted vehicle states and inputs (some in separate plots)
%
% Z is an array with all the vehicle states (x,y,v,\psi) from the beggining until now
% z_predicted is an array with the predicted vehicle states (x,y,v,\psi) by MPC
% u_predicted is an array with the predicted vehicle inputs (a,\beta) by MPC
% PARAMS is a structure with at least the following parameters:
%    * params.l_f, distance between center of gravity and front axle
%    * params.l_r, distance between center of gravity and rear axle
%    * params.vehicle_width, vehicle width
%    * params.track_end, end of the track
%    * params.window_size, plot window size for the during simulation, centered at the current position (x,y) of the vehicle
%    * params.nstates, number of states
%    * params.activate_obstacles, 0 if there are no obstacle, 1 otherwise
%    * params.obstacle_centers, x and y coordinate of the 4 obstacles
%    * params.obstacle_size, size of the obstacles
%    * params.lane_semiwidth, semi-width of the lane
%    * params.plot_full, 0 for plotting only the window size, 1 for plotting from 0 to track_end
%
% Created by Pedro Lima (pfrdal@kth.se) and  Valerio Turri (turri@kth.se),
% 23/09/2016 for the EL2700 - MPC Course @ KTH, Sweden

%% parsing the parameters structure
vehicle_length      = params.l_f + params.l_r;
vehicle_width       = params.vehicle_width;
track_end           = params.track_end;
window_size         = params.window_size;
nstates             = params.nstates;
if nstates < 3
    error('The number of states cannot be less than 3!\n');
end
activate_obstacles  = params.activate_obstacles;
obstacle_centers    = params.obstacle_centers;
obstacle_size       = params.obstacle_size;
lane_semiwidth      = params.lane_semiwidth;
plot_full           = params.plot_full;

%% parsing the arguments
current_pos         = z(end,:);
if length(varargin) == 2
    z_horizon           = varargin{1};
    u_horizon           = varargin{2};
else
    z_horizon = [];
    u_horizon = [];
end
%% car description
car_x = [current_pos(1)-vehicle_length/2 current_pos(1)+vehicle_length/2 current_pos(1)+vehicle_length/2 current_pos(1)-vehicle_length/2];
car_y = [current_pos(2)+vehicle_width/2 current_pos(2)+vehicle_width/2 current_pos(2)-vehicle_width/2 current_pos(2)-vehicle_width/2];

%% window size limits
if plot_full == 1
    windowlimits_x = [0 track_end];
    windowlimits_y = [-lane_semiwidth lane_semiwidth];
elseif plot_full == 0
    if ~isempty(window_size) % if there are window_size limits
        windowlimits_x = [current_pos(1)-window_size current_pos(1)+window_size];
        windowlimits_y = [-lane_semiwidth lane_semiwidth];
    else % if there are no window_size limits then plot everything
        error('The window size cannot be an empty vector if plot_full is 0\n')
    end
else
    error('The flag plot_full must be set to 0 or 1\n')
end
windowlimits = [windowlimits_x,windowlimits_y];


%% define obstacles
if activate_obstacles == 1 % define obstacles
    obs_x = zeros(size(obstacle_centers,1),4); % edges x coordinates
    obs_y = zeros(size(obstacle_centers,1),4); % edges y coordinates
    for i = 1:size(obstacle_centers,1)
        xc = obstacle_centers(i,1); % obstacle center x coordinate
        xsize = obstacle_size(1)/2; % obstacle x size
        obs_x(i,:) = [xc-xsize, xc+xsize, xc+xsize, xc-xsize];
        
        yc = obstacle_centers(i,2); % obstacle center y coordinate
        ysize = obstacle_size(2)/2; % obstacle y size
        obs_y(i,:) = [yc+ysize, yc+ysize, yc-ysize, yc-ysize];
    end
    
elseif activate_obstacles == 0
    % do nothing
else
    error('The flag activate_obstacles must be set to 0 or 1\n')
end


figure(1);
%% plot simulation environment
subplot(4,4,1:12); % plot road and vehicle
hold off; plot(0,0); hold on;
% plot the car (x,y)
car_handle=patch(car_x,car_y,'white','EdgeColor','black');
rotate(car_handle,[0 0 1],rad2deg(current_pos(4)),[current_pos(1),current_pos(2) 0]);
% plot the lane limits
plot(windowlimits_x,[-lane_semiwidth;-lane_semiwidth],'r','linewidth',5)
plot(windowlimits_x,[lane_semiwidth;lane_semiwidth],'r','linewidth',5)
% plot the centerline
plot(windowlimits_x,[0;0],'--k','linewidth',3)
% plot the obstacles
if activate_obstacles == 1
    for i = 1:size(obstacle_centers,1)
        patch(obs_x(i,:),obs_y(i,:),'black','EdgeColor','black');
    end
end
% plot the car path from the beggining of time
plot(z(:,1),z(:,2),'g');
% plot predicted vehicle states (x,y) if given
if ~isempty(z_horizon)
    plot(z_horizon(:,1),z_horizon(:,2),'-om');
end
% plot the "end" of the track (when x==track_end)
plot([track_end;track_end], [-lane_semiwidth;lane_semiwidth], '-r', 'linewidth',20)
axis(windowlimits)
xlabel('x');
ylabel('y');

hold on
a=1; % horizontal radius
b=3; % vertical radius
x0=[10,20,30,40]; % x0,y0 ellipse centre coordinates
y0=[-2 0 -2 6];
t=-pi:0.01:pi;
j =0;
while j <4
    j=j+1;
xled=x0(j)+a*cos(t);
yled=y0(j)+b*sin(t);
plot(xled,yled,'r')
hold on
end
drawnow;

%% plot the heading, velocity and the vehicle inputs (if z/u_horizon vectors are given)
if ~isempty(z_horizon) && ~isempty(u_horizon)
    %% plot vehicle heading
    subplot(4,4,13);
    hold off; plot(0,0); hold on;
    plot(z_horizon(:,1),z_horizon(:,4),'-om')
    ylim([-pi/3 pi/3]);
    xlim([min(z_horizon(:,1)) max(z_horizon(:,1))]);
    xlabel('x');
    ylabel('\psi');
    drawnow;
    
    %% plot vehicle velocity
    subplot(4,4,14);
    hold off; plot(0,0); hold on;
    plot(z_horizon(:,1),z_horizon(:,3),'-om')
    ylim([0 20])
    xlim([min(z_horizon(:,1)) max(z_horizon(:,1))]);
    xlabel('x');
    ylabel('v');
    drawnow;
    
    %% plot vehicle acceleration
    subplot(4,4,15);
    hold off; plot(0,0); hold on;
    plot(z_horizon(1:end-1,1),u_horizon(:,1),'-om')
    ylim([-2 2])
    xlim([min(z_horizon(:,1)) max(z_horizon(:,1))]);
    xlabel('x');
    ylabel('a');
    drawnow;
    
    %% plot vehicle \beta
    subplot(4,4,16);
    hold off; plot(0,0); hold on;
    plot(z_horizon(1:end-1,1),u_horizon(:,2),'-om')
    ylim([-pi/4 pi/4])
    xlim([min(z_horizon(:,1)) max(z_horizon(:,1))]);
    xlabel('x');
    ylabel('\beta');
    drawnow;
    
else
    % do nothing
end
end