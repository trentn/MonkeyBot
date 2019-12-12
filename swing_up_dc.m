function [u, traj] = swing_up_dc(init_cond, goal_cond, initial_guess, dt, naughtPoints)
    A = [];
    b = [];
    Aeq = [];
    beq = [];

    lb = ones(1,naughtPoints)*-10;
    ub = ones(1,naughtPoints)*10;

    options = optimoptions(@fmincon, ...
                            'TolFun',0.00000001, ...
                            'MaxIter',1000, ...
                            'MaxFunEvals',100000, ...
                            'Display','iter', ...
                            'DiffMinChange',0.001, ...
                            'Algorithm','sqp');

    output = fmincon(@cost,initial_guess,A,b,Aeq,beq,[lb;-Inf(4,naughtPoints)],[ub;Inf(4,naughtPoints)],@nonlcon,options);

    u = output(1,:);
    traj = output(2:end,:);

    function J = cost(x)      
        J = dot(x(1,:),x(1,:));
    end

    %enforce dynamics with non linear constraints
    function [c,ceq] = nonlcon(x)
        %global naughtPoints dt;
        
        c = [];

        %initial constraint
        ceq_init = x(2:end,1)-init_cond;

        %final constraint
        ceq_f = x(2:end,end)-goal_cond;

        ceq = zeros(4,naughtPoints+1);
        ceq(:,1) = ceq_init;
        for i = 1:naughtPoints-1
            uc = (x(1,i)+x(1,i+1))/2;
            Xc = 1/2*( x(2:end,i) + x(2:end,i+1))+ dt/8*(acrobot_dynamics(x(1,i),x(2:end,i)) - acrobot_dynamics(x(1,i+1),x(2:end,i+1))); 
            delt_k = (x(2:end,i)-x(2:end,i+1)) + dt/6*(acrobot_dynamics(x(1,i),x(2:end,i)) + 4*acrobot_dynamics(uc,Xc) + acrobot_dynamics(x(1,i+1),x(2:end,i+1)));
            ceq(:,i+1)=delt_k;
        end
        ceq(:,end)=ceq_f;
    end
end