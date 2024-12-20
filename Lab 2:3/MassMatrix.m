function MM = MassMatrix(q)
    
    global m M J r l I 
    
    phi = q(1);
    theta = q(2);

  
    MM = zeros(2, 2);

    MM(1, 1) = J + ((m + M) * r^2);
    MM(1, 2) = MM(1,1) + (M * l * r * cos(theta));
    MM(2, 1) = MM(1, 2);
    MM(2, 2) = MM(1, 1) + (2 * M * l * r * cos(theta)) + (I + (M * l^2));


end