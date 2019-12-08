%%%%
% In Swing Phase
% Direct Collocation
%%%%%
naughtPoints = 50;
time_to_swing = 1;
dt = time_to_swing/naughtPoints;

init_cond = [-pi/4;0;pi;0;0;0];
goal_cond = [pi/4;-pi/4;3*pi/4;0;0;0];

%initial guess is 0
u0 = zeros(2,naughtPoints);
x0 = zeros(6,naughtPoints);

%only 4 constraints to start
initial_guess = [u0;x0];
[short_u, short_traj] = in_swing_dc_short(init_cond(1:4), goal_cond(1:4), initial_guess, dt, naughtPoints);