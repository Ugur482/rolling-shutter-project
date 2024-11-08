clear all
close all
clc

global V R L C
V = 5;
R = 1e3;
L = 100e-3;
C = 10e-6;

tspan = [0 1];
y0 = [0 0];

[t,y] = ode45(@example,tspan,y0);

figure(1)
subplot(2,1,1)
plot(t,y(:,1))
subplot(2,1,2)
plot(t,y(:,2))

function dydt = example(t,y)

global V R L C

dIdt = y(1);
I = y(2);

dVdt = 2*pi*V*cos(2*pi*10*t);
    
d2Idt2 = dVdt/L -R/L*dIdt -1/(C*L)*I;

dydt = [d2Idt2; dIdt];

end
