function e = acrobot_energy(X);
    m1 = 1;
    m2 = 1;

    l1 = 1;
    lc1 = .5;

    l2 = 1;
    lc2 = .5;

    g = 9.8;

    I1 = (1/3)*m1*l1^2;
    I2 = (1/3)*m2*l2^2;

    M = @(q) [I1+I2+m2*l1^2+2*m2*l1*lc2*cos(q(2))   I2 + m2*l1*lc2*cos(q(2))
              I2+m2*l1*lc2*cos(q(2))                I2];

    C = @(q,dq) [-2*m2*l1*lc2*sin(q(2))*dq(2)   -m2*l1*lc2*sin(q(2))*dq(2)
                 m2*l1*lc2*sin(q(2))*dq(1)     0];

    G = @(q) [-m1*g*lc1*sin(q(1))-m2*g*(l1*sin(q(1))+lc2*sin(q(1)+q(2)))
                 -m2*g*lc2*sin(q(1)+q(2))];

    B = [0;1];

    q = X(1:2);
    dq = X(3:4);

    ke = .5*dq'*M(q)*dq;
    pe = -m1*g*lc1*cos(q(1))-m2*g*(l1*cos(q(1))+lc2*cos(q(1)+q(2)));

    e = ke+pe;
end