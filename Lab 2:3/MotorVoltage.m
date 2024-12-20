function E = MotorVoltage(taudphi)
    
    global eta N Tm Km Ra 

    tau = taudphi(1);
    dphi = taudphi(2);

    if tau > 0
       i = ((tau / (eta * N)) - Tm) / Km;
    elseif tau < 0
       i = ((tau / (eta * N)) + Tm) / Km;
    elseif tau == 0
        E = 0;
        return
    
    end

    E = (Ra * i) + (Km * N * dphi);

end