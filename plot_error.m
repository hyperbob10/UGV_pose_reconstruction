clc
clearvars
close all

%% Load data files
files = dir("*trajectories*.mat");
n_files = length(files);
data = cell(n_files,1);
x = zeros(1, n_files);
y = zeros(5, n_files);


%% For each file (each N)
for i = 1:n_files
    name = files(i).name;
    data{i} = load(name);
    x(i) = length(data{i}.T_history_abs);
    %% Substract each measurement from GT
    delta_avg = data{i}.T_history_abs(1:2,:) - data{i}.est_avg(1:2,:);
    delta_ccw = data{i}.T_history_abs(1:2,:) - data{i}.est_ccw(1:2,:);
    delta_cw = data{i}.T_history_abs(1:2,:) - data{i}.est_cw(1:2,:);
    delta_odom = data{i}.T_history_abs(1:2,:) - data{i}.est_odom(1:2,:);
    delta_sym = data{i}.T_history_abs(1:2,:) - data{i}.est_sym(1:2,:);
    norms = zeros(5, x(i));
    %% Calculate distance of each pose
    for j = 1:x(i)
        norms(1, j) = norm(delta_avg(j));
        norms(2, j) = norm(delta_ccw(j));
        norms(3, j) = norm(delta_cw(j));
        norms(4, j) = norm(delta_odom(j));
        norms(5, j) = norm(delta_sym(j));
    end
    err_mean_norms_i = zeros(5, 1);
    %% Average distance of all poses
    for j = 1:5
        err_mean_norms_i(j) = mean(norms(j,:));
    end
    y(:, i) = err_mean_norms_i;
end

%% Plot errors
figure
hold on
grid on
xticks(x)
xlim([min(x) max(x)])
ylim([0 4])
plot(x, y)
legend avg ccw cw odom sym
ylabel 'Average euclidean distance from GT [m]'
xlabel 'Number of shoots'
title 'Evolution mean euclidean distance of trajectories from GT varying N'

