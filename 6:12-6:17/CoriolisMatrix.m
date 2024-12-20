function C = CoriolisMatrix(qdq)

    global M l r
    
    phi = qdq(1);
    theta = qdq(2);
    dphi = qdq(3);
    detheta = qdq(4);

    C = zeros(2, 2);

    C(1, 1) = 0;
    C(1, 2) = detheta .* M .* l .* r .* sin(theta);
    C(2, 1) = 0;
    C(2, 2) = detheta .* M .* l .* r .* sin(theta);

end