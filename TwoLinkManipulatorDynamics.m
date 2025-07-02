function [m_s, c_s, g_s] = TwoLinkManipulatorDynamics(s, sdot)
% Generates the parameterised lagrangian dynamics
% m - Masses of links (vector)
% l - Lengths of links (vector)
% q(s) - Parameterised joint angles

% Gravity
g = 9.81;
% m - Masses of links
m = [2; 2.5];
% L - Lengths of links
l = [1; 1.5];
% Path start point
q_start = [0; 0];
% Path end point
q_end = [90; 90];

% Path Parameterisation
% q(s) = q_start + s(q_end - q_start)
q_s = q_start + s*(q_end-q_start);
% New velocity and acceleration vectors:
%   qdot = (q_end - q_start)sdot
%   qddot = (q_end - q_start)sddot
q_s_dot = q_end-q_start;
q_s_ddot = 0;

M11 = m(2)*l(2)^2 + m(2)*(l(1)^2 + 2*l(1)*l(2)*cos(q_s(2)) + l(2)^2);
M12 = m(2)*(l(1)*l(2)*cos(q_s(2)) + l(2)^2);
M21 = M12;
M22 = m(2)*l(2)^2;
% M(q(s))
M_q = [M11 M12; M21 M22];
% c(q(s), q(s)dot*sdot)
c_q = [-m(2)*l(1)*l(2)*sin(q_s(2))*(2*(q_s_dot(1)*sdot)*(q_s_dot(2)*sdot)*(q_s_dot(2)*sdot)^2);
    m(2)*l(1)*l(2)*(q_s_dot(1)*sdot)^2*sin(q_s(2))];
% g(q(s))
g_q = [(m(1)+m(2))*l(1)*g*cos(q_s(1) + m(2)*g*l(2)*cos(q_s(1)+q_s(2)));
    m(2)*g*l(2)*cos(q_s(1)+q_s(2))];


% M(s) = M(q(s))*q(s)dot
m_s = M_q*q_s_dot;
% c(s)sdot^2 = M(q(s))*q(s)ddot*sdot^2 + c(q(s), q(s)dot*sdot)
c_s = M_q*q_s_ddot*sdot^2 + c_q;
% g(s) = g(q(s))
g_s = g_q;

end