clear all
close all
clc

global Control_Signal ...
    t_old Current_History ...
    counter err_old ...
    derr Err_History

Control_Signal = 0; % t = 0, no control!
Current_History = [];
Err_History = [];
t_old = -1;
tspan = [0 465]; % (s)
y0 = 0;
counter = 0;
err_old = 0;
derr = 0;
[t x] = ode45(@circuit,tspan,y0);

% figure(1)
% plot(t,x)
% xlabel('Time (s)')
% ylabel('Integral Control')

figure(2)
subplot(1,2,1)
plot(Current_History)
ylabel('Current (A)')
subplot(1,2,2)
plot(Err_History)
ylabel('Error (A)')

function dxdt = circuit(t,x)

global Control_Signal ...
    t_old Current_History ...
    counter err_old ...
    derr Err_History

Kp = 900;
Ki = 15;
Kd = 0.0175;
R = 1000; % (Ohms)
ref = 0.01; % (A)
V = Control_Signal; % (V)
I = V/R; % (A)
err = ref -I; % (A)
interr = x;
err_per = err/ref;
if err_per < 0.01
    interr = 0;
end
Control_Signal = Kp*err +Ki*interr +Kd*derr;

Integral_control = err;
dxdt = Integral_control;

if t > t_old
    if t >= 0
        delt = t -t_old;
        derr = (err -err_old)/delt;
    end
    counter = counter +1;
    Current_History(counter,1) = I;
    Err_History(counter,1) = err;
    t_old = t;

end

end