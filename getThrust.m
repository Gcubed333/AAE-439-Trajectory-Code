function [thrust, m_dot] = getThrust(t,g)
    persistent thrust_fit
    persistent Isp
    persistent time
    
    dt = 1/2000;

    if t <= 0
         
        thrust_data = importdata('Group4.mat');
        shift = mean(thrust_data(7600:end));
        thrust_data = (thrust_data(2960:7600)-shift)*4.4482;
        
        mp = 0.06;
        I = sum(thrust_data*dt); 
        Isp = I/(mp*g);

        time = 0:dt:((length(thrust_data)-1)*dt);
        
        thrust_fit = csaps(time,thrust_data,0.99995,time);
    end
    
    if t == -1
        thrust = (length(thrust_fit)-1)*dt;
        m_dot = thrust;
        return;
    end    
    
    thrust = interp1(time,thrust_fit,t);
    %disp('BREAK')
    %fprintf('The resultant thrust is: %0.3f\n',thrust);
    m_dot = thrust/(Isp*g);
end