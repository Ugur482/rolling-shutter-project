clear all
close all
clc

global Control_Signal t_old Current_History counter

Control_Signal = 0; % t = 0, no control!
Current_History = [];
t_old = -1;
tspan = [0 600]; % (s)
y0 = 0;
counter = 0;

[t x] = ode45(@circuit,tspan,y0);

% figure(1)
% plot(t,x)
% xlabel('Time (s)')
% ylabel('Integral Control')

figure(2)
plot(Current_History)
ylabel('Current (A)')

function dxdt = circuit(t,x)

global Control_Signal t_old Current_History counter

Kp = 100;
Ki = 10;
R = 1000; % (R)
ref = 0.05; % (A)
V = Control_Signal; % (V)
I = V/R; % (A)
err = ref -I; % (A)
Control_Signal = x +Kp*err;

Integral_control = Ki*err;
dxdt = Integral_control;

if t > t_old
    counter = counter +1;
    Current_History(counter,1) = I;
    t_old = t;
end

end