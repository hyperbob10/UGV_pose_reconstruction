% %% we wanna define a circle of radious r
% %% we define a circle around the beacon in order
% %% to be able to reconstruct the fundamental matrix
% %% without scale factor
% clear all
% close all
% run initializeUnicycleModel.m
% radius = 1;
% x_0 = 0; %x_0 and y_0 defines the center of the circle
% y_0 = 0;
% theta = linspace(0,2*pi, 360);
% coords_x(:,1) = linspace(0,360,360);
% coords_x(:,2) = radius*cos(theta') + x_0;
% coords_y(:,1) = linspace(0,360,360);
% coords_y(:,2) = radius*sin(theta') + y_0;
% theta_des(:,1) = linspace(0,360,360);
% %% finding the heading direction
% %% vector from the center to the given point
% % +pi/2 is due to the starting position
% for i = 1:length(theta)
%     if i <= 90
%         theta_des(i,2) = atan2(coords_y(i,2),coords_x(i,2)) + pi/2;
%     elseif i <= 180
%         theta_des(i,2) = atan2(coords_y(i,2),-coords_x(i,2)) + pi/2;
%     elseif i <= 270
%         theta_des(i,2) = -atan2(-coords_y(i,2),-coords_x(i,2)) + pi/2;
%     else
%         theta_des(i,2) = -atan2(-coords_y(i,2),coords_x(i,2)) + pi/2;
%     end
% end
% 
% v_des = 1;
% w_des = 1;
% 
% %%% Parameters for the controller
% csi = 0.5;  %damping factor
% a = 78;     %natural frequency
% k1 = 2*csi*a;
% k2 = (a^2 - w_des^2)/(v_des);
% k3 = k1; % k3 must be equal to k1
% 
% 
% sim('trajectory_ctrl_unicycle',360); 
% 
% figure;
% subplot(4,1,1)
% plot(coords_x(:,2),coords_y(:,2));
% axis equal
% hold on
% plot(coords_x(1,2),coords_y(1,2), 'r.', 'MarkerSize', 25);
% quiver(coords_x(1,2),0,0,0.75,'color', [1 0 0]);
% title('Desired trajectory to be control')
% legend('desired trajectory', 'starting point','starting heading dir.')
% subplot(4,1,2)
% plot(coords_x(:,1),coords_x(:,2))
% axis([0 360 -4 4])
% legend('x des [m]')
% subplot(4,1,3)
% plot(coords_y(:,1),coords_y(:,2))
% axis([0 360 -4 4])
% legend('y des [m]')
% subplot(4,1,4)
% plot(theta_des(:,2)); %+pi/2 is due to the starting position
% axis([0 360 -4 4])
% legend('theta des [rad]')
% 
% %%plotting the result
% x_out = ans.output.signals(1).values;
% y_out = ans.output.signals(2).values;
% figure;
% plot(x_out, y_out);
% axis equal
% hold on
% plot(coords_x(:,2),coords_y(:,2));
% plot(coords_x(1,2),coords_y(1,2), 'r.', 'MarkerSize', 25);
% quiver(coords_x(1,2),0,0,0.75,'color', [1 0 0]);
% legend('real','des')



%%Simulation with the new trajectory
clearvars
run initializeUnicycleModel.m
r0 = 15;
x0 = 0; %x_0 and y_0 defines the center of the circle
y0 = 0;
theta_samples = 10965;
theta = linspace(0,2*pi, theta_samples);
rect_samples = ceil(theta_samples / (2*pi));
coords_x(:,1) = [zeros(1,rect_samples) linspace(0,360,theta_samples)];
coords_x(:,2) = [linspace(0, r0, rect_samples) r0*cos(theta) + x0];
coords_y(:,1) = [zeros(1,rect_samples) linspace(0,360,theta_samples)];
coords_y(:,2) = [zeros(1,rect_samples) r0*sin(theta) + y0;];
theta_des(:,1) = linspace(0,360,theta_samples);

%% finding the heading direction
%% vector from the center to the given point
% +pi/2 is due to the starting position
% for i = 265:length(theta)
%     if i <= 2750
%         theta_des(i,2) = atan2(coords_y(i,2),coords_x(i,2));
%     elseif i <= 5500
%         theta_des(i,2) = atan2(coords_y(i,2),-coords_x(i,2));
%     elseif i <= 8225
%         theta_des(i,2) = -atan2(-coords_y(i,2),-coords_x(i,2));
%     else
%         theta_des(i,2) = -atan2(-coords_y(i,2),coords_x(i,2));
%     end
% end

%% finding the heading direction
%% vector from the center to the given point
% +pi/2 is due to the starting position
for i = 265:length(theta)
    theta_des(i,2) = 0.0134129*theta_des(i,1) + 1.45453;
end

v_des = 1;
w_des = 5;

%%% Parameters for the controller
csi = 0.5;   %damping factor
a = 35;     %natural frequency
k1 = 2*csi*a;
k2 = (a^2 - w_des^2)/(v_des);
k3 = k1; % k3 must be equal to k1


data = sim('trajectory_ctrl_unicycle',360); 

x_out = data.output.signals(1).values;
y_out = data.output.signals(2).values;
theta_out = data.output.signals(3).values;
e1 = data.error.signals(1).values;
e2 = data.error.signals(2).values;
e3 = data.error.signals(3).values;
time = data.tout;

save("data.mat", "x_out", "y_out", "theta_out");