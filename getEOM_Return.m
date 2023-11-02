function F = getEOM_Return(t,y,Vw,m)
    Vx = y(1);
    Vy = y(2);
    x = y(3);
    h = y(4);

    rho = getRho(h);
    g = getGravity(h);
    A_para = pi*0.4572^2;
    Cd_para = 2.2;
    S = 0.1016*1.1938;
    
    if Vx >=0 
        V_rel = sqrt((Vx-Vw)^2+Vy^2);
        D = 0.5*rho*A_para*Cd_para*V_rel^2;
        theta = atan2(Vy,Vx-Vw);

        Dx = -D*cos(theta);
        Dy = -D*sin(theta);
    else
        Cd = getCoeffDrag(pi/2);

        Dx = -0.5*rho*S*Cd*(Vx-Vw)^2*sign(Vx-Vw);
        Dy = -0.5*rho*A_para*Cd_para*Vy^2*sign(Vy);
    end

    dx_dt = Vx;
    dh_dt = Vy;
    
    dVx_dt = (Dx/m);
    dVy_dt = (Dy/m) - g;

    F = [dVx_dt;dVy_dt;dx_dt;dh_dt];

end