function A = AMatrix()
    clear all

    syms theta phi detheta dphi tau
    syms J M m r l I g
    JMmr = J + (M + m) * (r^2);
    
    Mlr = M*l*r;
    Ml2 = M*(l^2);
    Mgl = M*g*l;
    
    M1 = [JMmr, JMmr + Mlr*cos(theta) ; JMmr + Mlr*cos(theta), JMmr + 2*Mlr*cos(theta) + (I + Ml2)];
    
    C = [0, -detheta * Mlr * sin(theta) ; 0, -detheta * Mlr * sin(theta)];
    
    G = [0; -Mgl * sin(theta)];
    
    M_inverse = inv(M1);
    
    x = [phi; theta; dphi; detheta];
    
    u = [tau; 0];
    
    q_dot = [dphi; detheta];
    
    f = [dphi; detheta; M_inverse * (u - G - (C * q_dot))];
    
    jacobian1 = jacobian(f,x);
    
    subs(jacobian1, [phi; theta; dphi; detheta], [0; 0; 0; 0])

end