function [xdfb, tau, a, Wb] = IDfb_MM(model, xfb, q, qd, qdd, f_ext)
% IDfb4  Floating-base inverse dynamics with symbolic simplification
%
%   [xdfb, tau, a, Wb] = IDfb4(model, xfb, q, qd, qdd, f_ext)
%
% This function is a symbolic-friendly variant of Featherstone's
% floating-base inverse dynamics algorithm (IDfb). In addition to the
% joint effort vector tau and the floating-base state derivative xdfb,
% it returns:
%   - a  : cell array of spatial accelerations for all bodies
%   - Wb : net spatial wrench acting on the floating base (body 6),
%          expressed in base coordinates
%
% The implementation follows the structure of IDfb, with additional
% simplify() calls to improve the readability of symbolic expressions.

a_grav = get_gravity(model);

qn = xfb(1:4);				% unit quaternion fixed-->f.b.
r = xfb(5:7);				% position of f.b. origin
Xup{6} = plux( rq(qn), r );		% xform fixed --> f.b. coords
Xup{6} = simplify(Xup{6});

vfb = xfb(8:end);
v{6} = Xup{6} * vfb;			% f.b. vel in f.b. coords
v{6} = simplify(v{6});

a{6} = zeros(6,1);

IC{6} = model.I{6};
IC{6} = simplify(IC{6});
pC{6} = model.I{6}*a{6} + crf(v{6})*model.I{6}*v{6};
pC{6} = simplify(pC{6});
disp('Paso 1 Listo');

for i = 7:model.NB
  [ XJ, S{i} ] = jcalc( model.jtype{i}, q(i-6) );
  vJ = S{i}*qd(i-6);
  Xup{i} = XJ * model.Xtree{i};
  Xup{i} = simplify(Xup{i});
  v{i} = Xup{i}*v{model.parent(i)} + vJ;
  v{i} = simplify(v{i});
  a{i} = Xup{i}*a{model.parent(i)} + S{i}*qdd(i-6) + crm(v{i})*vJ;
  a{i} = simplify(a{i});
  IC{i} = model.I{i};
  IC{i} = simplify(IC{i});
  pC{i} = IC{i}*a{i} + crf(v{i})*IC{i}*v{i};
  pC{i} = simplify(pC{i});
  i
end
disp('Paso 2 Listo');

if nargin == 6 && length(f_ext) > 0
  prnt = model.parent(6:end) - 5;
  pC(6:end) = apply_external_forces(prnt, Xup(6:end), pC(6:end), f_ext(6:end));
end

for i = model.NB:-1:7
  IC{model.parent(i)} = IC{model.parent(i)} + Xup{i}'*IC{i}*Xup{i};
  %IC{model.parent(i)} = simplify(IC{model.parent(i)});
  pC{model.parent(i)} = pC{model.parent(i)} + Xup{i}'*pC{i};
  %pC{model.parent(i)} = simplify(pC{model.parent(i)});
  i
end
disp('Paso 3 Listo');

disp('Iniciando paso 4');
IC{6} = simplify(IC{6});
pC{6} = simplify(pC{6});

% Floating-base acceleration (in base coordinates)
a{6} = - IC{6} \\ pC{6};

% Net spatial wrench acting on the floating base (body 6), in base coords
% Wb = [n; f], where:
%   n : 3x1 moment vector about the base origin
%   f : 3x1 force vector at the base origin
Wb = IC{6} * a{6} + pC{6};

% Optional symbolic simplification
if isa(Wb, 'sym')
    Wb = simplify(Wb);
end

% without gravity
disp('Paso 4 Listo');

for i = 7:model.NB
  a{i} = Xup{i} * a{model.parent(i)};
  tau(i-6,1) = S{i}'*(IC{i}*a{i} + pC{i});
end
disp('Paso 5 Listo');                                        

qnd = rqd( vfb(1:3), qn );		% derivative of qn
rd = Vpt( vfb, r );			% lin vel of flt base origin
afb = Xup{6} \ a{6} + a_grav;		% f.b. accn in fixed-base coords
%save('newdata.mat','a','tau','qnd','rd','afb');
xdfb = [ qnd; rd; afb ];
