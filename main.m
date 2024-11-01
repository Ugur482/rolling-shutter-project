% MEE349 Mechatronics System Design 1 - Project 1
% Deniz Eren Horoz 
% Uğur Can Kızılcan
% Merve Çabuk
%
% Rolling Shutter Project
%

% Clear previous sessions
clear all;
close all;
clc;


% === Functions === %

% Derivate current over time
% function curr_change = der_curr_time(t, y) 
%     curr_change = (1/L) * (Vin - (I * Res) - (Kb * (derivative)));
% end

% Derivate angular acceleration
% function ang_acc = der_ang_acc() 
%     ang_acc = (1/J) * ((Ki * I) - b * (derivative) - (Tl/r));
% end


% === Constants === %
L    = 0.058;                      % Terminal Inductance (mH)
Vin  = 24;                         % Nominal Voltage (V)
I    = 29.2;                       % Nominal Current (A)
Res  = 0.024;                      % Terminal Resistence (ohm)
Kb   = 47.7;                       % Back EMF Coefficient (V/(rad/s))
J    = 568;                        % Moment of Inertia (g/cm^2)
Ki   = 47.7;                       % Torque Constant (mNm/A)
b    = 0.3405;                     % Friction Coefficient
p    = 2.49;                       % Density of Material (kg/m^2)
l    = 2;                          % Length of Shutter (m)
w    = 0.0089;                     % Thickness of Shutter Panel (m)    
g    = 9.80665;                    % Standart Gravity (m/s^2)
angD = 1200;                       % Total rotation (degrees)
angR = deg2rad(1200);              % Total rotation (rad)
rin  = 0.00175;                    % Radius of inner shutter (m)
rb   = rin + ((w/(2*pi)) * angD);  % Radius of outer shutter (m)
hext = 2.6;                        % Height of Extended Shutter (m)
mext = p * hext * l * w;           % Mass of the Extended Shutter (kg)
Text = mext * g * rb;              % External torque (F)
%Tl  = 
%r   =

% === Variables === %
t = linspace(0, 15, 60);           % Time array (s)
ang = linspace(0, 1200, 60);       % Degree array (degrees)
arr_angvel = diff(ang)- diff(t);   % Angular velocity array (degrees/s)
angvel = arr_angvel(1);            % Angular velocity (degrees/s)
%arr_currch = 
%currch = arr_currch(1);           % Current change (A/s)