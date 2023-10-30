function dy_dt = getEOM_Burn_or_Coast(t,y,V_w,S,dt,state)
    V = y(1);
    theta = y(2);
    x = y(3);
    h = y(4);
    psi_o = y(5);
    m = y(6);

    dx_dt = V*cos(theta);
    dh_dt = V*sin(theta);

    rho = getRho(h);
    g = getGravity(h);
    
    T = 0;
    m_dot = 0;
    if state == 1
        [T,m_dot] = getThrust(t,g);
    end

    Cd = getCoeffDrag(0);
    Cd_w = getCoeffDrag(abs(theta));
    Cl = getCoeffLift(0);
    Cl_w = getCoeffLift(abs(theta));

    D = 0.5*rho*S*(V^2*Cd - sign(V_w)*V_w^2*Cd_w);
    L = 0.5*rho*S*(V^2*Cl - sign(V_w)*V_w^2*Cl_w);

    psi = psi_o;
    if t ~= 0
        opts= optimoptions('fsolve','Display','none',...
            'Algorithm','levenberg-marquardt','FunctionTolerance',1e-09);
        psi = fsolve(@(psi) solvePsi(psi,theta,T,L,D,g*m),psi_o,opts);
    end
    dpsi_dt = (psi - psi_o)/dt;

    dv_dt = (T/m)*cos(psi-theta) - (D/m) - g*sin(theta);  

    V_rel = V + V_w*cos(theta);
    dtheta_dt = (T/(m*V_rel))*sin(psi-theta) + L/(m*V_rel) - (g/V_rel)*cos(theta);

    dm_dt = -m_dot;

    dy_dt = [dv_dt; dtheta_dt; dx_dt; dh_dt; dpsi_dt; dm_dt];
    
end