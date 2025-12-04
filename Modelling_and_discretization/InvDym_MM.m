function Inv_Dyn_MM()
% Inv_Dyn_MM obtain symbolic base and arm efforts using ABA (IDfb_MM)
%
% This script builds the dynamic model of the skid-steer mobile manipulator
% using Featherstone's ABA with a floating base (Spatial_v2 + floatbase),
% and computes symbolic expressions for:
%
%   tau1   : base yaw torque (about base z-axis)
%   flin1  : base longitudinal force (along base x-axis)
%   tau3-5 : joint torques of the 3-DOF arm
%
% The result is stored in 'inv_dyn.mat' and later used by the
% linearization/discretization script to construct M(q), C(q,qd), G(q).

    %% 1) Build the ABA model
    MM_sym_ini;   % defines "model" and applies floatbase(model)

    %% 2) Symbolic variables consistent with the linearization script
    syms phi phip phipp real
    syms int_vb vb vbp real
    syms th3 th4 th5 real
    syms th3p th4p th5p real
    syms th3pp th4pp th5pp real
    syms g real

    %% 3) Floating-base state xfb (13x1)
    % xfb = [q0; q1; q2; q3; px; py; pz; wx; wy; wz; vx; vy; vz]
    q0 = cos(phi/2);
    q1 = sym(0);
    q2 = sym(0);
    q3 = sin(phi/2);

    px = int_vb;
    py = sym(0);
    pz = sym(0);

    wx = sym(0);
    wy = sym(0);
    wz = phip;

    vx = vb;
    vy = sym(0);
    vz = sym(0);

    xfb = [ q0;
            q1;
            q2;
            q3;
            px;
            py;
            pz;
            wx;
            wy;
            wz;
            vx;
            vy ];

    %% 4) Joint coordinates for wheels + arm (symbolic)
    % First 4 entries: generic wheel angles; last 3: arm joints th3, th4, th5.
    syms qw2 qw3 qw4 qw5 real
    syms qw2d qw3d qw4d qw5d real
    syms qw2pp qw3pp qw4pp qw5pp real

    q   = [qw2;
           qw3;
           qw4;
           qw5;
           th3;
           th4;
           th5];

    qd  = [qw2d;
           qw3d;
           qw4d;
           qw5d;
           th3p;
           th4p;
           th5p];

    qdd = [qw2pp;
           qw3pp;
           qw4pp;
           qw5pp;
           th3pp;
           th4pp;
           th5pp];

    %% 5) Call symbolic-friendly IDfb_MM (extended to return Wb)
    % IDfb_MM must have signature:
    %   [xdfb, tau_all, a_all, Wb] = IDfb_MM(model, xfb, q, qd, qdd, f_ext)
    %
    % Here we assume no external forces (f_ext omitted).

    [xdfb, tau_all, a_all, Wb] = IDfb_MM(model, xfb, q, qd, qdd);

    %% 6) Extract arm joint torques and base wrench
    tau3 = tau_all(end-2);  % arm joint torque 3
    tau4 = tau_all(end-1);  % arm joint torque 4
    tau5 = tau_all(end);    % arm joint torque 5

    % Spatial wrench on the base: Wb = [n; f] in base coordinates
    n_base = Wb(1:3);
    f_base = Wb(4:6);

    % Projection to base yaw torque and longitudinal force:
    tau1  = n_base(3);   % yaw torque about base z-axis
    flin1 = f_base(1);   % longitudinal force along base x-axis

    % Optional symbolic simplification (lightweight)
    if isa(tau1,  'sym'), tau1  = simplify(tau1);  end
    if isa(flin1, 'sym'), flin1 = simplify(flin1); end
    if isa(tau3,  'sym'), tau3  = simplify(tau3);  end
    if isa(tau4,  'sym'), tau4  = simplify(tau4);  end
    if isa(tau5,  'sym'), tau5  = simplify(tau5);  end

    %% 7) Save symbolic efforts for later use in linearization
    save('inv_dyn.mat','tau1','flin1','tau3','tau4','tau5');

    disp('Inv_Dyn_MM: symbolic efforts (tau1, flin1, tau3-5) saved in inv_dyn.mat');
end
