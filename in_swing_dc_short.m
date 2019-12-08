function [u, traj] = in_swing_dc_short(init_cond, goal_cond, initial_guess, dt, naughtPoints)
    A = [];
    b = [];
    Aeq = [];
    beq = [];

    lb = ones(2,naughtPoints)*-10;
    ub = ones(2,naughtPoints)*10;

    options = optimoptions(@fmincon, ...
                            'TolFun',0.00000001, ...
                            'MaxIter',1000, ...
                            'MaxFunEvals',100000, ...
                            'Display','iter', ...
                            'DiffMinChange',0.001, ...
                            'Algorithm','sqp');

    output = fmincon(@cost,initial_guess,A,b,Aeq,beq,[lb;-Inf(6,naughtPoints)],[ub;Inf(6,naughtPoints)],@nonlcon,options);

    u = output(1:2,:);
    traj = output(3:end,:);

    function J = cost(x)
        R = eye(2);
        J = 0;
        for i = 1:length(x)
            J = J + x(1:2,i)'*R*x(1:2,i);
        end
    end

    %enforce dynamics with non linear constraints
    function [c,ceq] = nonlcon(x)
        %global naughtPoints dt;
        
        c = [];

        %initial constraint
        ceq_init = x(3:6,1)-init_cond;

        %final constraint
        ceq_f = x(3:6,end)-goal_cond;

        ceq = zeros(4,naughtPoints+1);
        ceq(:,1) = ceq_init;
        for i = 1:naughtPoints-1
            uc = (x(1:2,i)+x(1:2,i+1))/2;
            Xc = 1/2*( x(3:end,i) + x(3:end,i+1))+ dt/8*(monkey_bot_dynamics(x(1:2,i),x(3:end,i)) - monkey_bot_dynamics(x(1:2,i+1),x(3:end,i+1))); 
            delt_k = (x(3:end,i)-x(3:end,i+1)) + dt/6*( monkey_bot_dynamics(x(1:2,i),x(3:end,i)) + 4*monkey_bot_dynamics(uc,Xc) + monkey_bot_dynamics(x(1:2,i+1),x(3:end,i+1)));
            ceq(:,i+1)=delt_k(1:4);
        end
        ceq(:,end)=ceq_f;
    end
end