function [u, traj] = swing_up_phase(goal_state, init_u, dt)
    %http://underactuated.mit.edu/underactuated.html?chapter=acrobot
    %dt = 0.05;
    k1 = 1;
    k2 = .5;
    k3 = 1;
    %goal_state = [pi/2;0;0;0];
    ed = acrobot_energy(goal_state);

    init_cond = [0;0;0;0];
    %init_u = .5;

    cur_state = init_cond + acrobot_dynamics(init_cond,init_u)*dt;

    control = [init_u];
    state = [init_cond, cur_state];
    while abs(cur_state(1) - goal_state(1)) > 0.01
        q = cur_state(1:2);
        dq = cur_state(3:4);

        delt_e = acrobot_energy(cur_state)-ed;

        ubar = dq(1)*delt_e;
        u = -k1*q(2)-k2*dq(2)-k3*ubar;

        cur_state = cur_state + acrobot_dynamics(cur_state,u)*dt;
        state = [state cur_state];
        control = [control u];
    end

    u = control;
    traj = state;    
end