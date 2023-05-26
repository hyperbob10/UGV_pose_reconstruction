close all
radius = 1;
x_0 = 0; %x_0 and y_0 defines the center of the circle
y_0 = 0;
theta = linspace(0,2*pi, 10965);
coords_x(:,1) = linspace(0,360,10965);
coords_x(:,2) = radius*cos(theta') + x_0;
coords_y(:,1) = linspace(0,360,10965);
coords_y(:,2) = radius*sin(theta') + y_0;
theta_des(:,1) = linspace(0,360,10965);
for i = 1:265
    coords_x(i,2) = i/265;
    coords_y(i,2) = 0;
    theta_des(i,2) = 0;
end
%% finding the heading direction
%% vector from the center to the given point
% +pi/2 is due to the starting position
for i = 265:length(theta)
    theta_des(i,2) = 0.0134129*theta_des(i,1) + 1.45453;
end


figure;
plot(coords_x(:,2),coords_y(:,2),'r')
axis equal
figure;
plot(theta_des(:,1),theta_des(:,2))
figure;
subplot(2,1,1)
plot(coords_x(:,1),coords_x(:,2));
subplot(2,1,2)
plot(coords_y(:,1),coords_y(:,2));