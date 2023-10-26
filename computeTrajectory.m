function [V,Theta,X,H,Index] = computeTrajectory(psi_o,v_w,dt)
%%
%Start by computing the trajctory caused by the burn
    %define constants
    t_burn = getThrust(-1,0);
    mp = 0.06;
    m_dot = mp/t_burn;
    m_i = 0.9;
    m_o = m_i + mp;
    Isp = 84.28; 
    S = pi*0.0508^2;
    tspan = 0:dt:t_burn;
    Index = [];
    
    %Use numerical integrator to solve for velocity, theta, and position
    opts = odeset('RelTol',1e-4,'AbsTol',1e-4);
    [~,y] = ode45(@(t,y) getEOM_Burn_or_Coast(t,y,v_w,...
        S,dt,1),tspan, [0,psi_o,0,0,psi_o,m_o],opts);
    
    %Add the computed state data to arrays for storage
    v = y(:,1);
    theta = y(:,2);
    x = y(:,3);
    h = y(:,4);
    psi = y(:,5);
    Index = [Index; length(x)];

%%
%Now compute the trajectory after the burn and before the parachute
%deploys
    %define constants
    v_o = v(end);
    theta_o = theta(end);
    x_o = x(end);
    h_o = h(end);
    psi_o = psi(end);
    t_lag = 7;
    tspan = 0:dt:t_lag;
    
    y=0;
    t=0;
    %Use numerical integrator to solve for velocity, theta, and position
    [t,y] = ode45(@(time,y) getEOM_Burn_or_Coast(t,y,v_w,...
                  S,dt,2),tspan, [v_o,theta_o,x_o,h_o,psi_o,m_i],opts);
    
    %Add the computed state data to arrays for storage
    v = [v; y(:,1)];
    theta = [theta; y(:,2)];
    x = [x; y(:,3)];
    h = [h; y(:,4)];
    psi = [psi; y(:,5)];
    Index = [Index; length(x)];

%%
%Finally, compute the return trajectory after the parachute has been
%deployed.
    %define constants
    v_o = v(end);
    theta_o = theta(end);
    x_o = x(end);
    h_o = h(end);
    psi_o = psi(end);
    v_para = -0.5; %found with experimental data
    t_f = -h_o/v_para;
    tspan = 0:dt:t_f;

    y=0;
    t=0;
    %Use numerical integrator to solve for velocity, theta, and position
    [~,y] = ode45(@(t,y) getEOM_Return(t,y,m_i,v_w,v_para),...
        tspan, [x_o,h_o],opts);
    
    %Add the computed state data to arrays for storage
    x = [x; y(:,1)];
    h = [h; y(:,2)];
    
    V = v;
    Theta = theta;
    X = x;
    H = h;
    Index = [Index; length(x)];

end