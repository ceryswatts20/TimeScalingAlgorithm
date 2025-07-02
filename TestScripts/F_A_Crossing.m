%% Time Scaling Algorithm
clear all
close all

% Min joint torques
min_tau = [-4800; -5000];
% Max joint torques
max_tau = [6000; 7000];
% List of switch points
S = [];
% Switch counter
i = 0;
% Initial State - (s, sdot) = (0, 0)
s_i = [0 0];
s_lim = [];
% Simulation time
tspan = [0 10];

% Set event conditions
backFunc = @(t, x) backwardsStopEvent(t, x, min_tau, max_tau);
options = odeset('Events', backFunc, 'RelTol', 1e-6, 'AbsTol', 1e-8);
direction = 'backwardMin';
%[Time, Function Output, Points where event occurred, Solutions to tF, ???]
[~, F] = ode45(@(t, x) Simulation(t, x, direction, min_tau, max_tau), tspan, [1 0], options);

forwardFunc = @(t, x) forwardsStopEvent(t, x, min_tau, max_tau);
options2 = odeset('Events', forwardFunc, 'RelTol', 1e-6, 'AbsTol', 1e-8);
direction = 'forwardMax';
[~, A] = ode45(@(t, x) Simulation(t, x, direction, min_tau, max_tau), tspan, s_i, options);

plot(F(:, 1), F(:, 2))
hold on
plot(A(:, 1), A(:, 2))
grid on
legend('F', 'A')
xlabel('s')
ylabel('$\dot{s}$', 'Interpreter', 'latex')

% Plot VLC
% Inital x2 value
sdot_0 = 0.1;
% Lots of x1 values
s_star = 0:0.001:1;

% Find sdot for each s
% arrayfun() applies fsolve() to each s value iteratively.
sdot_vlc = arrayfun(@(s_star) fsolve(@(sdot) vlcSimulation(min_tau, max_tau, s_star, sdot), sdot_0), s_star);

plot(s_star, sdot_vlc, 'DisplayName', 'VLC')
