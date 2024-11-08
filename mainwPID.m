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

% === Variables === %
tspan   = [0, 15];
y0      = [0, 0];

global I Tl rb ang J
ang     = 0;                              % Rotation Angle (rad)
I       = 29.2;                           % Nominal Current (A)

% Calculating initial rb value
angt    = 21;                             % Estimated Max Rotation Angle (rad)
rin     = 0.00175;                        % Radius of Inner shutter (m)
w       = 0.0089;                         % Thickness of Shutter Panel (m)
rb      = rin + ((w/(2*pi)) * angt);      % Radius of the Shutter (rad)

% Calculating initial Tl value
l       = 2;                              % Length of Shutter (m)
p       = 2.46;                           % Density of Material (kg/m^2)
mb      = p * l * w * (rb^2 - rin^2);     % Mass of Outer Shutter (kg)
Ib      = 0.5 * mb * (rin^2 + rb^2);      % Inertia of Outer Shutter (kg*m^2)
Min     = 5.4;                            % Mass of Inner Shutter (kg)
Iin     = (0.5  * Min) + (rin^2);         % Inertia of Inner Shutter (kg*m^2)
est_aga = 23;                             % Estimated Initial Angular Acceleration (rad/s^2)
Itot    = Iin + Ib;                       % Total Inertia of Shutter (kg*m^2)
Tin     = Itot * est_aga;                 % Torque of Inner Shutter (F*m)
l       = 2;                              % Length of Shutter (m)
hext    = 2.6;                            % Height of Extended Shutter (m)
mext    = p * hext * l * w;               % Mass of the Extended Shutter (kg)
g       = 9.80665;                        % Standart Gravity (m/s^2)
Text    = mext * g * rb;                  % External torque (F * m)
Tl      = Tin + Text;                     % Load Torque (F*m)
Jm      = 5680;                           % Total Inertia of Motor (kg*m^2)
J       = Itot + Jm;                      % Total Inertia of System (kg*m^2)

% PID Variables
global sgn_ctrl hist_curr hist_err err_old counter t_old der
sgn_ctrl    = 0;
hist_curr   = [];
hist_err    = [];
t_old       = -1;
counter     = 0;
err_old     = 0;
der         = 0;

% === Simulation Function === %
function dydt = simulate(t, y)
    global I Tl rb ang J
    global sgn_ctrl hist_curr hist_err err_old counter t_old der

    % Constants
    L    = 0.058;                      % Terminal Inductance (mH)
    Vin  = 24;                         % Nominal Voltage (V)  
    Res  = 0.024;                      % Terminal Resistence (ohm)
    Kb   = 47.7;                       % Back EMF Coefficient (V/(rad/s))
    Ki   = 47.7;                       % Torque Constant (mNm/A)
    b    = 0.3405;                     % Friction Coefficient
    w    = 0.0089;                     % Thickness of Shutter Panel (m)
    l    = 2;                          % Length of Shutter (m)
    p    = 2.46;                       % Density of Material (kg/m^2)
    hext = 2.6;                        % Height of Extended Shutter (m)
    mext = p * hext * l * w;           % Mass of the Extended Shutter (kg)
    g    = 9.80665;                    % Standart Gravity (m/s^2)
    Text = mext * g * rb;              % External torque (F * m)
    Min  = 5.4;                        % Mass of Inner Shutter (kg)
    rin  = 0.00175;                    % Radius of Inner shutter (m)
    
    % PID Values
    gainPr  = 900;
    gainIn  = 15;
    ref     = 0.01;                    % (A)

    % Update Variables
    ang_vel = y(1);
    I = y(2);

    err = ref - I;
    inter = y;
    err_per = err/ref;
    if err_per < 0.01
        inter = 0;
    end

    sgn_ctrl = gainPr * err + gainIn * inter + gainPr * der;
    int_ctrl = err;

    if t > t_old
        if t >= 0
            delta = t - t_old;
            der = (err - err_old) / delta;
        end
        counter = counter + 1;
        hist_curr(counter, 1) = I;
        hist_err(counter, 1) = err;
        t_old = t;
    end
    
    ang_acc = (1/J) * ((Ki * I) - b * (ang_vel) - (Tl/rb));
    curr_change = (1/L) * (Vin - (I * Res) - (Kb * (ang_vel)));

    ang  = ang + ang_vel;
    rb   = rb - ((w/(2*pi)) * ang);
    mb   = p * l * w * (rb^2 - rin^2);
    Ib   = 0.5 * mb * (rin^2 + rb^2);
    Iin  = (0.5  * Min) + (rin^2);
    Itot = Iin + Ib;
    Tin  = Itot * ang_acc;             
    Tl   = Tin + Text;
    
    % Return Variables
    dydt = [ang_acc; curr_change];
end

% Start Simulation
options = odeset('MaxStep', 0.5);
[t, y] = ode45(@simulate, tspan, y0, options);

% Plot Simulation
figure(1)
subplot(2, 2, 1)
plot(t, y(:, 1));
ylabel('Angular Velocity (rad/s)')
subplot(2, 2, 3)
plot(t, y(:, 2));
ylabel('Motor Current (A)')
xlabel('Time (s)')
subplot(2, 2, 2)
plot(hist_curr)
ylabel('Current (A)')
subplot(2, 2, 4)
plot(hist_err)
ylabel('Error (A)')
xlabel('Time (s)')


