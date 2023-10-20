function dy_dt = getParaDerivative(t,y,m,V_w,V_para)
    dh_dt = V_para; %dhdt
    rho = getRho(y(2));
    S = 1.1938*0.1016;
    Cd = getCoeffDrag(pi/2);
    
    num = 0.5*rho*S*Cd*t*(-V_w)*V_w;
    den = m + 0.5*rho*S*Cd*t*(-V_w);
    dx_dt = num/den; %dxdt

    dy_dt = [dx_dt,dh_dt]';
end