%==========================================================================
% Symbolic Linearization and Discretization of the ABA-Based Dynamic Model 
% for the Skid-Steer Mobile Manipulator﻿
%==========================================================================
% This script:
%   1) Loads symbolic joint/base efforts (tau1, flin1, tau3-5) generated
%      by InvDym_MM.m using Featherstone's ABA with a floating base.
%   2) Constructs the inertia matrix M(q), Coriolis/centrifugal vector
%      C(q,qd) and gravity vector G(q) symbolically.
%   3) Linearizes and discretizes the resulting state-space model around a
%      static equilibrium point, preserving the original pipeline.
%==========================================================================

close all; clear; clc;

%% ------------------------------------------------------------------------
%  LOAD SYMBOLIC EFFORTS FROM ABA MODEL
% -------------------------------------------------------------------------
% inv_dyn.mat must contain:
%   tau1   : base yaw torque (about base z-axis)
%   flin1  : base longitudinal force (along base x-axis)
%   tau3-5 : arm joint torques

load('inv_dyn.mat','tau1','flin1','tau3','tau4','tau5');

%% ------------------------------------------------------------------------
%  DEFINE SYMBOLIC ACCELERATIONS AND PARAMETERS FOR M, C, G CONSTRUCTION
% -------------------------------------------------------------------------

syms phipp vbp th3pp th4pp th5pp real
syms g real

% The generalized accelerations are ordered as:
%   qpp = [phipp; vbp; th3pp; th4pp; th5pp]

%% ------------------------------------------------------------------------
%  INERTIA MATRIX M(q) VIA COEFFICIENT EXTRACTION
% -------------------------------------------------------------------------
% Each row Mi* is obtained by extracting the coefficients of 
% phipp, vbp, th3pp, th4pp, th5pp from the corresponding effort equation:

% Row associated with tau1 (base yaw torque)
M11 = coeffs(tau1,  phipp); M11 = M11(end);
M12 = coeffs(tau1,  vbp);   M12 = M12(end);
M13 = coeffs(tau1,  th3pp); M13 = M13(end);
M14 = coeffs(tau1,  th4pp); M14 = M14(end);
M15 = coeffs(tau1,  th5pp); M15 = M15(end);

% Row associated with flin1 (base longitudinal force)
M21 = coeffs(flin1, phipp); M21 = M21(end);
M22 = coeffs(flin1, vbp);   M22 = M22(end);
M23 = coeffs(flin1, th3pp); M23 = M23(end);
M24 = coeffs(flin1, th4pp); M24 = M24(end);
M25 = coeffs(flin1, th5pp); M25 = M25(end);

% Row associated with tau3
M31 = coeffs(tau3,  phipp); M31 = M31(end);
M32 = coeffs(tau3,  vbp);   M32 = M32(end);
M33 = coeffs(tau3,  th3pp); M33 = M33(end);
M34 = coeffs(tau3,  th4pp); M34 = M34(end);
M35 = coeffs(tau3,  th5pp); M35 = M35(end);

% Row associated with tau4
M41 = coeffs(tau4,  phipp); M41 = M41(end);
M42 = coeffs(tau4,  vbp);   M42 = M42(end);
M43 = coeffs(tau4,  th3pp); M43 = M43(end);
M44 = coeffs(tau4,  th4pp); M44 = M44(end);
M45 = coeffs(tau4,  th5pp); M45 = M45(end);

% Row associated with tau5
M51 = coeffs(tau5,  phipp); M51 = M51(end);
M52 = coeffs(tau5,  vbp);   M52 = M52(end);
M53 = coeffs(tau5,  th3pp); M53 = M53(end);
M54 = coeffs(tau5,  th4pp); M54 = M54(end);
M55 = coeffs(tau5,  th5pp); M55 = M55(end);

M = [M11 M12 M13 M14 M15;
     M21 M22 M23 M24 M25;
     M31 M32 M33 M34 M35;
     M41 M42 M43 M44 M45;
     M51 M52 M53 M54 M55];

%% ------------------------------------------------------------------------
%  GRAVITY VECTOR G(q)
% -------------------------------------------------------------------------

G1 = coeffs(tau1,  g); G1 = G1(end);
G2 = coeffs(flin1, g); G2 = G2(end);
G3 = coeffs(tau3,  g); G3 = G3(end);
G4 = coeffs(tau4,  g); G4 = G4(end);
G5 = coeffs(tau5,  g); G5 = G5(end);

G = [G1; G2; G3; G4; G5];

%% ------------------------------------------------------------------------
%  CORIOLIS/CENTRIFUGAL VECTOR C(q,qd)
% -------------------------------------------------------------------------
% C is obtained as the residual of each effort equation after subtracting
% the inertial and gravity contributions:

C1 = simplify(tau1  - (M11*phipp + M12*vbp + M13*th3pp + M14*th4pp + M15*th5pp) - G1*g);
C2 = simplify(flin1 - (M21*phipp + M22*vbp + M23*th3pp + M24*th4pp + M25*th5pp) - G2*g);
C3 = simplify(tau3  - (M31*phipp + M32*vbp + M33*th3pp + M34*th4pp + M35*th5pp) - G3*g);
C4 = simplify(tau4  - (M41*phipp + M42*vbp + M43*th3pp + M44*th4pp + M45*th5pp) - G4*g);
C5 = simplify(tau5  - (M51*phipp + M52*vbp + M53*th3pp + M54*th4pp + M55*th5pp) - G5*g);

C = [C1; C2; C3; C4; C5];

%% ------------------------------------------------------------------------
%  SAVE DYNAMIC MATRICES FOR FURTHER USE
% -------------------------------------------------------------------------

save('ABA_MCG.mat','M','C','G');

%% ------------------------------------------------------------------------
% LINEARIZATION/DISCRETIZATION
% -------------------------------------------------------------------------

% Direct dynamics: qpp = inv(M) * (taus - C - G*g)

syms t1 f1 t3 t4 t5 real
taus = [t1; f1; t3; t4; t5];

aux_der = taus - C - G*g;
aux_der = simplify(aux_der);
aux_der = vpa(aux_der, 5);

invM = inv(M);
invM = simplify(invM);
invM = vpa(invM, 5);

qpp = invM * aux_der;
qpp = vpa(qpp, 5);

qpp1 = simplify(qpp(1));   % phipp
qpp2 = simplify(qpp(2));   % vbp
qpp3 = simplify(qpp(3));   % th3pp
qpp4 = simplify(qpp(4));   % th4pp
qpp5 = simplify(qpp(5));   % th5pp

qpp1 = vpa(qpp1, 5);
qpp2 = vpa(qpp2, 5);
qpp3 = vpa(qpp3, 5);
qpp4 = vpa(qpp4, 5);
qpp5 = vpa(qpp5, 5);

% State, input and nonlinear dynamics f(x,u)

syms phi phip int_vb vb th3 th4 th5 th3p th4p th5p phipp vbp th3pp th4pp th5pp real

fxu = [ phip;
        qpp1;
        vb;
        qpp2;
        th3p;
        th4p;
        th5p;
        qpp3;
        qpp4;
        qpp5 ];

x = [phi; phip; int_vb; vb; th3; th4; th5; th3p; th4p; th5p];
u = [t1; f1; t3; t4; t5];

%% ------------------------------------------------------------------------
%  EQUILIBRIUM CONDITIONS
% -------------------------------------------------------------------------

% Equilibrium configuration (static operating point)
phi = 0;
th3 = 0;
th4 = 2*pi - pi/12;
th5 = 0;

% Zero velocities and accelerations at equilibrium
phip  = 0;
phipp = 0;
vb    = 0;
vbp   = 0;
th3p  = 0;
th4p  = 0;
th5p  = 0;
th3pp = 0;
th4pp = 0;
th5pp = 0;

%% ------------------------------------------------------------------------
%  MODEL PARAMETERS
% -------------------------------------------------------------------------

% Base parameters
mb = 12;       % Base mass without the arm
Jb = 0.5;      % Base yaw inertia
b1 = 1;        % Rotational friction coefficient
b2 = 0.07;     % Linear friction coefficient

% Arm parameters
m3 = 2.867;
m4 = 0.633;
m5 = 0.79;
r3 = 0.025;    % Link radius (Katana reference)
L3 = 0.06;
L4 = 0.19;
L5 = 0.139;
b3 = 0.42;
b4 = 0.42;
b5 = 0.42;
g  = 9.8062;

%% ------------------------------------------------------------------------
%  EFFORTS AT EQUILIBRIUM (FROM ABA-BASED MODEL)
% -------------------------------------------------------------------------
% tau1, flin1, tau3, tau4, tau5 must be loaded or defined symbolically
% before this section (e.g., from ABA_tau.mat).

t1 = eval(tau1);
f1 = eval(flin1);
t3 = eval(tau3);
t4 = eval(tau4);
t5 = eval(tau5);

%% ------------------------------------------------------------------------
%  LINEAR MODEL MATRICES A, B, C, D
% -------------------------------------------------------------------------

Nfxu = length(fxu);
Nx   = length(x);
Nu   = length(u);

% State Jacobian A = ∂f/∂x
A = sym(zeros(Nx, Nx));
for i = 1:Nx
    for j = 1:Nx
        A(i,j) = diff(fxu(i), x(j));
    end
end

% Input Jacobian B = ∂f/∂u
B = sym(zeros(Nx, Nu));
for i = 1:Nx
    for j = 1:Nu
        B(i,j) = diff(fxu(i), u(j));
    end
end

% Output matrix C
C = eye(Nfxu);

% Remove velocity components from the output (position-only outputs)
C(2,2)   = 0;
C(4,4)   = 0;
C(8,8)   = 0;
C(9,9)   = 0;
C(10,10) = 0;

% Direct feedthrough matrix D
D = zeros(Nfxu, size(B,2));

%% ------------------------------------------------------------------------
%  NUMERICAL EVALUATION AND DISCRETIZATION
% -------------------------------------------------------------------------

An = eval(A);
Bn = eval(B);

Ts = 0.01;                  % Sampling period (s)
sysc = ss(An, Bn, C, D);    % Continuous-time state-space model
tfc  = tf(sysc);            % Continuous-time transfer function

sysd = c2d(sysc, Ts, 'zoh');% Discrete-time state-space model (ZOH)
tfd  = tf(sysd);            % Discrete-time transfer function

% Controllability and observability analysis
Co   = ctrb(sysd);                          % Controllability matrix
unco = length(sysd.A) - rank(Co);          % Number of uncontrollable states

Ob   = obsv(sysd);                         % Observability matrix
unob = length(sysd.A) - rank(Ob);          % Number of unobservable states

%% ------------------------------------------------------------------------
%  ROOT-LOCUS-BASED GAIN RANGE EXPLORATION (DISCRETE PD CONTROLLER)
% -------------------------------------------------------------------------

z = tf('z');
alpha_ctrl = 0.1;
ctld = 1 + alpha_ctrl * ((z - 1)/z);   % Discrete-time PD-like controller

ft1 = series(ctld, tfd(1,1));
figure;
subplot(2,2,1);
rlocus(ft1);
ax = gca;
ax.XAxis.FontSize = 15;
ax.YAxis.FontSize = 15;
title('\gamma = 0.1','Interpreter','latex','FontSize',20);

alpha_ctrl = 0.5;
ctld = 1 + alpha_ctrl * ((z - 1)/z);
ft1  = series(ctld, tfd(1,1));
subplot(2,2,2);
rlocus(ft1);
ax = gca;
ax.XAxis.FontSize = 15;
ax.YAxis.FontSize = 15;
title('\gamma = 0.5','Interpreter','latex','FontSize',20);

alpha_ctrl = 1.0;
ctld = 1 + alpha_ctrl * ((z - 1)/z);
ft1  = series(ctld, tfd(1,1));
subplot(2,2,3);
rlocus(ft1);
ax = gca;
ax.XAxis.FontSize = 15;
ax.YAxis.FontSize = 15;
title('\gamma = 1.0','Interpreter','latex','FontSize',20);

alpha_ctrl = 2.0;
ctld = 1 + alpha_ctrl * ((z - 1)/z);
ft1  = series(ctld, tfd(1,1));
subplot(2,2,4);
rlocus(ft1);
ax = gca;
ax.XAxis.FontSize = 15;
ax.YAxis.FontSize = 15;
title('\gamma = 2.0','Interpreter','latex','FontSize',20);

% Additional closed-loop transfer functions for different input-output
% channels (can be inspected with rltool if desired)

ft2  = series(ctld, tfd(5,1));
ft3  = series(ctld, tfd(3,2));
ft4  = series(ctld, tfd(1,3));
ft5  = series(ctld, tfd(5,3));
ft6  = series(ctld, tfd(3,4));
ft7  = series(ctld, tfd(6,4));
ft8  = series(ctld, tfd(7,4));
ft9  = series(ctld, tfd(3,5));
ft10 = series(ctld, tfd(6,5));
ft11 = series(ctld, tfd(7,5));

disp('Symbolic linearization, discretization and preliminary control analysis completed.');

