function B = BMatrix()

    global J M m r l I 
    

    B = [0;                                                                                                                                   
         0;
         (I + J + M*l^2 + M*r^2 + m*r^2 + 2*M*l*r*cos(0))/(I*J + M^2*l^2*r^2 + J*M*l^2 + I*M*r^2 + I*m*r^2 + M*l^2*m*r^2 - M^2*l^2*r^2*cos(0)^2);
         -(J + M*r^2 + m*r^2 + M*l*r*cos(0))/(I*J + M^2*l^2*r^2 + J*M*l^2 + I*M*r^2 + I*m*r^2 + M*l^2*m*r^2 - M^2*l^2*r^2*cos(0)^2)];
    

end