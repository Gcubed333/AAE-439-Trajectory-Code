close all;

thrust = importdata('Group4.mat');
dt = 1/2000;

shift = mean(thrust(7600:end));

thrust = thrust(2960:7500) - shift;
thrust = thrust*4.4482;
t = 0:dt:((length(thrust)-1)*dt);

thrust2 = csaps(t,thrust,0.99995,t);

thrust3 = interp1(t,thrust2,0);

plot(t,thrust);
hold on;
plot(t,thrust2);