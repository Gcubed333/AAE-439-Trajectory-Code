clear; clc; close all;

v_w = -3; %input('Input the wind velocity in (m/s):\n');
v_para = 0.5;
psi_o = 1.2; %deg2rad(70);
dt = 0.01;

disp('INITIALIZED');

disp('SOLVING FOR A PERFECT RETURN');
opts = optimoptions('lsqnonlin','FunctionTolerance',1e-2);  
psi_o = lsqnonlin(@(psi) perfectReturn(psi, v_w, dt), psi_o,0,pi/2,opts);

disp('COMPUTING FINAL TRAJECTORY');

[V, Theta, X, H, Index] = computeTrajectory(psi_o,v_w,dt);

disp('PLOTTING FINAL TRAJECTORY');
figure();
plot(X(1:Index(1)),H(1:Index(1)));
hold on;
plot(X(Index(1)+1:Index(2)),H(Index(1)+1:Index(2)));
plot(X(Index(2)+1:Index(3)),H(Index(2)+1:Index(3)));
xlabel('Range (m)');
ylabel('Altitude (m)');
legend('Launch Trajectory', 'Parachute Lag Trajectory',...
    'Return Trajectory');

fprintf('Your initial Psi should be %0.3f degrees from horizontal.\n',...
    rad2deg(psi_o));


function X_f = perfectReturn(psi_o,v_w,dt)
    [~, ~, X, ~] = computeTrajectory(psi_o,v_w,dt);

    X_f = X(end);
end
