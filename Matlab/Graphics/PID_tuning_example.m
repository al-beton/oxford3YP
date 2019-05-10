%% GRAPHICAL EXAMPLE OF THE PROCESS OF TUNING A PID CONTROLLER %%

clear

% some natural damping for clear results
c = 1;


dt = 0.01;
t = 0;
T = dt * [1:1000];

sp = 1; 

last_x = 0;
x = 0;
X = NaN(1, 1000);
X(1) = x;

Kp = 10;
Ki = 0;
i = 0;
Kd = 0;


for k = 2:1000
    e = sp - x;
    v = (x - last_x) / dt;
    new_p = Kp * e;
    d = -Kd * v
    i = i + Ki * e * dt;
    p = new_p;
    
    a = p + i + d - 1.5 - c * v;
    X(k) = x;
    last_x = x;
    x = x + v * dt + 0.5 * a * dt^2;

end

figure
hold on
xlim([0 10])
ylim([0 2])
plot(T, X)
plot(T, sp * ones(1, 1000), '--')
title("Step response of a damped body affected" + newline + " by gravity and a PID controlled force")
ylabel('Displacement z in meters') 
xlabel('Time t in seconds') 

txt = ['Kp = ' num2str(Kp) ', Ki = ' num2str(Ki) ', Kd = ' num2str(Kd)];
dim = [0.6 .55 .3 .3];
str = 'Straight Line Plot from 1 to 10';
annotation('textbox',dim,'String',txt,'FitBoxToText','on');


