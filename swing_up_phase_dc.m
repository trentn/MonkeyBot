init_cond = [0;0;0;0];
goal_cond = [-pi/4;0;0;0];

naughtPoints = 200;
time_to_swingup = 60;
dt = time_to_swingup/naughtPoints;

%initial guess is 0
u0 = zeros(1,naughtPoints);
x0 = zeros(4,naughtPoints);

%only 4 constraints to start
initial_guess = [u0;x0];

[swingup_u, swingup_traj] = swing_up_dc(init_cond, goal_cond, initial_guess, dt, naughtPoints);
