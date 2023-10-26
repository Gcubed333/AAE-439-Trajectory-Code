function [thrust, m_dot] = getThrust(t,g)
    dt = 1/2000; 
    thrust_data = importdata('Group4.mat');
    shift = mean(thrust_data(7600:end));
    thrust_data = (thrust_data(2905:7600)-shift)*4.4482;

    if t == -1
        thrust = (length(thrust_data)-1)*dt;
        m_dot = thrust;
        return;
    end
    
    time = 0:dt:((length(thrust_data)-1)*dt);
    mp = 0.06;
    
    I = sum(thrust_data*dt);
    Isp = I/(mp*g);
    
    thrust = csaps(time,thrust_data,0.99995,t);
    
    m_dot = thrust/(Isp*g);
end