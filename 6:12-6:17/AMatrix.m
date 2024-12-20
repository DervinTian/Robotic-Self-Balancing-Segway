function A = AMatrix()

    global M g l J r m I 
    
  [0,                                                                                      0, 1, 0]
[0,                                                                                      0, 0, 1]
[0, -(M*g*l*(J + M*r^2 + m*r^2 + M*l*r))/(I*J + J*M*l^2 + I*M*r^2 + I*m*r^2 + M*l^2*m*r^2), 0, 0]
[0,          (M*g*l*(J + M*r^2 + m*r^2))/(I*J + J*M*l^2 + I*M*r^2 + I*m*r^2 + M*l^2*m*r^2), 0, 0]