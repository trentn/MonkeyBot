%%%%
% In Swing Phase
% Direct Collocation
%%%%%
load('short_traj');

naughtPoints = 50;
time_to_swing = 1;
dt = time_to_swing/naughtPoints;

init_cond = [-pi/4;0;pi;0;0;0];
goal_cond = [pi/4;-pi/4;3*pi/4;0;0;0];

%use a better guess
second_guess = [zeros(2,naughtPoints);short_traj(1:3,:);zeros(3,naughtPoints)];
[dc_u, dc_traj] = in_swing_dc(init_cond, goal_cond, second_guess, dt, naughtPoints);