function F=cal_cindir(q1,d1,q2,q3,q4)
%Definición de parámetros
L3=0.06;
L4=0.19;
L5=0.139;
%Cinemática Directa (simbólica)
% Fx=d1*cos(q1) + L4*cos(q1 + q2)*cos(q3) + L5*cos(q1 + q2)*cos(q3 + q4)-x;
% Fy=d1*sin(q1) + L5*cos(q3 + q4)*sin(q1 + q2) + L4*sin(q1 + q2)*cos(q3)-y;
% Fz=L3 + L5*sin(q3 + q4) + L4*sin(q3)-z;
Fx=d1*cos(q1) + L4*cos(q1 + q2)*cos(q3) + L5*cos(q1 + q2)*cos(q3 + q4);
Fy=d1*sin(q1) + L5*cos(q3 + q4)*sin(q1 + q2) + L4*sin(q1 + q2)*cos(q3);
Fz=L3 + L5*sin(q3 + q4) + L4*sin(q3);
F=[Fx;Fy;Fz];
end