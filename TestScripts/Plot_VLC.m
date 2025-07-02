%% Time Scaling Algorithm
clear all
close all

% Min joint torques
min_tau = [-4800; -5000];
% Max joint torques
max_tau = [6000; 7000];


% Plot VLC
% Inital x2 value
sdot_0 = 0.25;
% Lots of x1 values
s_star = 0:0.001:1;

% Find sdot for each s
% arrayfun() applies fsolve() to each s value iteratively.
sdot_vlc = arrayfun(@(s_star) fsolve(@(sdot) vlcSimulation(min_tau, max_tau, s_star, sdot), sdot_0), s_star);

plot(s_star, sdot_vlc, 'DisplayName', 'VLC')
xlabel('s')
ylabel('$\dot{s}$', 'Interpreter', 'latex')
title('Time-scaling Algoritm')
grid on
hold on

p = polyfit(s_star, sdot_vlc , 10);
% Evaluate the polynomial fit at the s_star values
sdot_fit = polyval(p, s_star);
sdot_fit = sdot_fit - 0.04;
% Plot the polynomial fit
plot(s_star, sdot_fit, 'DisplayName', 'Polynomial Fit');
legend show

