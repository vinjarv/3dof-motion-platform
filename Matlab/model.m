clear; clc
format shortg % Less exponents in output

% SS model of the ball in one axis
A = [0 1; 0 0];
B = [0; 7];
C = [1 0];
D = 0;
ball = ss(A, B, C, D);

% Check controllability and observability, needs to be same rank as SS
% model
disp("Controllability matrix rank:")
disp(rank(ctrb(A,B)))
disp("Observability matrix rank:")
disp(rank(obsv(A,C)))

% % Starting from Bryson's rule:
% Q = [1/(0.300)^2 0; 0 1/(0.100)^2]; % max diameter, max speed
% R = 1/(12.5*pi/180)^2; % max tilt
% 
% % Calculate LQR
% [K_opt, S, P_ctrl] = lqr(A, B, Q, R);

% Manual pole placement
PO = 0.5;
Ts = 2;

zeta = abs(log(PO/100))/sqrt(pi^2 + log(PO/100)^2)
wn = -log(0.02)/(zeta*Ts)
P_ctrl = roots([1 2*zeta*wn wn^2]);
K_opt = place(A, B, P_ctrl);

% Place observer poles 15x further out
P_obsv = 15*(P_ctrl);
L = place(A', C', P_obsv)';

Acl = A-B*K_opt;
Bcl = B;
Ccl = [1 0; 0 180/pi];
Dcl = D;
sys_cl = ss(Acl, Bcl, Ccl, Dcl);

% figure(1)
% x0 = [-0.17 0];
% initial(sys_cl, x0)

% Observer-controller SS model
Aoc = A-L*C-B*K_opt;
Boc = L;
Coc = K_opt;
Doc = 0;
sys_oc = ss(Aoc, Boc, Coc, Doc);

% Observer-controller TF
[num_oc, den_oc] = ss2tf(Aoc, Boc, Coc, Doc);
tf_oc = tf(num_oc, den_oc);

% Discretize TF
fs = 30;
tf_oc_d = c2d(tf_oc, 1/fs, "tustin");

% Check step response of observer-controller
% sys_tot = feedback(tf_oc*ball, 1);
% figure(2)
% step(0.1*sys_tot)
% Run sim
sim("state_regulator")

format long
disp("Controller filter coeffs. output:")
fprintf("B:\n")
fprintf("%.10f,", tf_oc_d.Numerator{1})
fprintf("\nA:\n")
fprintf("%.10f,", tf_oc_d.Denominator{1})