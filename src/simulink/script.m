%% hil v1
close all; clear all; clc;



global pck_length;
global pck_ctr;
pck_ctr = 0;

global time_old;
global g_x_old;
global g_y_old;
global g_z_old;

sim('sys_main');

display('simulation done')
pck_ctr

%Time of simulation
%serial.time(length(serial.time))
