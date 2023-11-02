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
    %tspan = 0:dt:t_burn;
    tspan = [0,t_burn];
    Index = [];
    
    %Use numerical integrator to solve for velocity, theta, and position
    opts = odeset('RelTol',1e-7,'AbsTol',1e-7);
    [~,y] = ode45(@(t,y) getEOM_Burn_or_Coast(t,y,v_w,...
        S,dt,1),tspan, [0,psi_o,0,0,psi_o,m_o],opts);
    
    %Add the computed state data to arrays for storage
    V = y(:,1);
    Theta = y(:,2);
    X = y(:,3);
    H = y(:,4);
    Psi = y(:,5);
    Index = [Index; length(X)];

%%
%Now compute the trajectory after the burn and before the parachute
%deploys
    %define constants
    t_lag = 4;
    tspan = 0:dt:t_lag;
    

    %Use numerical integrator to solve for velocity, theta, and position
    [~,y] = ode45(@(t,y) getEOM_Burn_or_Coast(t,y,v_w,...
                  S,dt,2),tspan, y(end,:) ,opts);
    
    %Add the computed state data to arrays for storage
    V = [V; y(:,1)];
    Theta = [Theta; y(:,2)];
    X = [X; y(:,3)];
    H = [H; y(:,4)];
    Psi = [Psi; y(:,5)];
    Index = [Index; length(X)];

%%
%Finally, compute the return trajectory after the parachute has been
%deployed.
    %define constants
    Vx_o = V(end)*cos(Theta(end));
    Vy_o = V(end)*sin(Theta(end));
    x_o = X(end);
    h_o = H(end);
    tspan = [0,10000];

    %Use numerical integrator to solve for velocity, theta, and position
    opts = odeset('RelTol',1e-7,'AbsTol',1e-7,'Events',@touchdownEvent);
    [~,y] = ode45(@(t,y) getEOM_Return(t,y,v_w,m_i),...
        tspan, [Vx_o,Vy_o,x_o,h_o],opts);
    
    %Add the computed state data to arrays for storage
    X = [X; y(:,3)];
    H = [H; y(:,4)];
    Index = [Index; length(X)];

end