function dX = monkey_bot_dynamics(u,X)
    m1 = 1;
    m2 = 1;
    m3 = 1;

    l1 = 1;
    lc1 = .5;

    l2 = 1;
    lc2 = .5;
    
    l3 = 1;
    lc3 = .5;

    g = 9.8;

    I1 = (1/3)*m1*l1^2;
    I2 = (1/3)*m2*l2^2;
    I3 = (1/3)*m3*l3^2;

    M = @(q)    [I1+I2+I3+m2*l1^2+m3*l1^2+2*m2*l1*lc2*cos(q(2))+2*m3*l1*lc3*cos(q(3))   I2+m2*l1*lc2*cos(q(2))     I3+m3*l1*lc3*cos(q(3))
                 I2+m2*l1*lc2*cos(q(2))                                                 I2                         0
                 I3+m3*l1*lc3*cos(q(3))                                                 0                          I3];

    C = @(q,dq) [-2*m2*l1*lc2*sin(q(2))*dq(2)-2*m3*l1*lc3*sin(q(3))*dq(3)                 -m2*l1*lc2*sin(q(2))*dq(2)   -m3*l1*lc3*sin(q(3))*dq(3)
                 m2*l1*lc2*sin(q(2))*dq(1)                                                0                            0
                 m3*l1*lc3*sin(q(3))*dq(1)                                                0                            0];

    G = @(q) [-m1*g*lc1*sin(q(1))-m2*g*(l1*sin(q(1))+lc2*sin(q(1)+q(2)))-m3*g*(l1*sin(q(1))+lc3*sin(q(1)+q(3)))
                 -m2*g*lc2*sin(q(1)+q(2))
                 -m3*g*lc3*sin(q(1)+q(3))];

    B = [0 0
         1 0
         0 1];

    q = X(1:3);
    dq = X(4:6);
    
    ddq = M(q)\(G(q) + B*u - C(q,dq)*dq);
    
    dX = [dq;
          ddq];
end