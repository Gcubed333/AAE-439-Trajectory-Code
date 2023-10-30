function F = tester(t,y,V_w,m,S)
    Vx = y(1);
    Vy = y(2);
    x = y(3);
    h = y(4);

    rho = getRho(h);
    g = getGravity(h);
    A_para = pi*0.4572^2;
    Cd_para = 2.2;

    theta = atan2(Vy,Vx);

    Cdx = getCoeffDrag(abs(theta));
    Dx = 0.5*rho*S*Cdx*(-1*sign(Vx)*Vx^2 - V_w^2);
    Dy = -0.5*rho*A_para*Cd_para*sign(Vy)*Vy^2;

    dx_dt = Vx;
    dh_dt = Vy;
    
    dVx_dt = (Dx/m);
    dVy_dt = (Dy/m) - g;

    F = [dVx_dt;dVy_dt;dx_dt;dh_dt];

end