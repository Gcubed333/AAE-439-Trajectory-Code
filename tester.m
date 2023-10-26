clear; clc; close all;

thrust = importdata('Group4.mat');
dt = 1/2000;

shift = mean(thrust(7600:end));

thrust = thrust(2905:7600) - shift;
t = 0:dt:((length(thrust)-1)*dt);

thrust2 = csaps(t,thrust,0.99995,t);

plot(t,thrust);
hold on;
plot(t,thrust2);