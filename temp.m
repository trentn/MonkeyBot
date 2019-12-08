%%%%
% Swing Up Phase
% Energy-based Feedback Controller
%%%%
dt = 0.01;

goal_cond = [-pi/4;0;0;0];
init_u = 2;

[energy_control, energy_traj] = swing_up_phase(goal_cond,init_u,dt);

energy_time = 0:dt:dt*length(energy_control);


load('dc_traj');
naughtPoints = 50;
time_to_swing = 1;
dt = time_to_swing/naughtPoints;
dc_time = (0:dt:dt*(naughtPoints-1))+energy_time(end);

time = [energy_time dc_time];

theta1 = [[energy_time' energy_traj(1,:)']; [dc_time' dc_traj(1,:)']];
theta2 = [[energy_time' energy_traj(2,:)']; [dc_time' dc_traj(2,:)']];
theta3 = [[energy_time' (-energy_traj(2,:)+pi)']; [dc_time' dc_traj(3,:)']];