%% script that must be run in order to 
%% be able to work with the simulink model
%% that simulate the control of the unicycle.

%% developed by Riccardo Barbiero

clear all
close all

open trajectory_ctrl_unicycle.slx
run initializeUnicycleModel.m
run def_trajectory.m