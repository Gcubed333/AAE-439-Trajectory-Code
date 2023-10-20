function dy_dt = getDerivative(t,y,V_w,Isp,m_o,m_dot,S,dt)
    V = y(1);
    theta = y(2);
    x = y(3);
    h = y(4);
    psi_o = y(5);

    dx_dt = V*cos(theta);
    dh_dt = V*sin(theta);

    rho = getRho(h);
    g = getGravity(h);
    m = m_o - m_dot*t;
    Cd = getCoeffDrag(0);
    Cd_w = getCoeffDrag(theta);
    Cl = getCoeffLift(0);
    Cl_w = getCoeffLift(theta);

    T = m_dot*g*Isp;
    D = 0.5*rho*S*(V^2*Cd - sign(V_w)*V_w^2*Cd_w);
    L = 0.5*rho*S*(V^2*Cl - sign(V_w)*V_w^2*Cl_w);

    psi = psi_o;
    if t ~= 0
        opts= optimoptions('fsolve','Display','none',...
            'Algorithm','levenberg-marquardt','FunctionTolerance',1e-05);
        psi = fsolve(@(psi) solvePsi(psi,theta,T,L,D,g*m),psi_o,opts);
    end
    dpsi_dt = (psi - psi_o)/dt;

    dv_dt = (T/m)*cos(psi-theta) - (D/m) - g*sin(theta);
    
    dtheta_dt = 0;
    if V > 1e-10
        dtheta_dt = (T/(m*V))*sin(psi-theta) + L/(m*V) - (g/V)*cos(theta);
    end

    dy_dt = [dv_dt; dtheta_dt; dx_dt; dh_dt; dpsi_dt];
    
end