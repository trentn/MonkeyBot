%%%%
% Swing Up Phase
% Energy-based Feedback Controller
%%%%
dt = 0.01;

goal_cond = [-pi/4;0;0;0];
init_u = 2;

[energy_control, energy_traj] = swing_up_phase(goal_cond,init_u,dt);

energy_time = 0:dt:dt*length(energy_control);

figure();
plot(energy_traj(1,:),energy_traj(3,:));

%start from end of swing up phase
init_cond = [energy_traj(1,end);
             energy_traj(2,end);
           (-energy_traj(2,end)+pi);
             energy_traj(3,end);
             energy_traj(4,end);
             energy_traj(4,end)];

         
%%%%
% In Swing Phase
% Direct Collocation
%%%%%
naughtPoints = 50;
time_to_swing = 1;
dt = time_to_swing/naughtPoints;


goal_cond = [pi/4;-pi/4;3*pi/4;0;0;0];

%initial guess is 0
u0 = zeros(2,naughtPoints);
x0 = zeros(6,naughtPoints);

%only 4 constraints to start
initial_guess = [u0;x0];
[short_u, short_traj] = in_swing_dc_short(init_cond(1:4), goal_cond(1:4), initial_guess, dt, naughtPoints);

%use a better guess
second_guess = [u0;short_traj(1:3,:);zeros(3,naughtPoints)];
[dc_u, dc_traj] = in_swing_dc(init_cond, goal_cond, second_guess, dt, naughtPoints);

%append state
dc_time = (0:dt:dt*(naughtPoints-1))+energy_time(end);
time = [energy_time dc_time];

theta1 = [[energy_time' energy_traj(1,:)']; [dc_time' dc_traj(1,:)']];
theta2 = [[energy_time' energy_traj(2,:)']; [dc_time' dc_traj(2,:)']];
theta3 = [[energy_time' (-energy_traj(2,:)+pi)']; [dc_time' dc_traj(3,:)']];