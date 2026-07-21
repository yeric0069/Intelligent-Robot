clear all; close all; clc

% modified waypoints
waypoints = [0.2,  3.0,  0;      
            20.0, 3.0,  pi/2;    
            20.0, 22.0, pi/2;      
            13.8,  24.0, pi;
            8.0,  22.0, 3*pi/2;
            11.0,  16.0, 2*pi;    
            13.2, 16.0, 2*pi];     

velocity_profile_generator(waypoints);



