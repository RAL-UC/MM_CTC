function [ o ] = ObjectiveFunction_error_pos (x)

fprintf('\nTesting combination: Kp1: %.3f Kp2: %.3f Kp3: %.3f Kp4: %.3f Kp5: %.3f',x(1),x(2),x(3),x(4),x(5));
fprintf('\nTesting combination: Kv1: %.3f Kv2: %.3f Kv3: %.3f Kv4: %.3f Kv5: %.3f\n',x(6),x(7),x(8),x(9),x(10));

%Reset noise in sensors
rng('shuffle')
rr1=abs(rand);
rr1=rr1*100;
assignin('base','r1',rr1);

rng('shuffle')
rr2=abs(rand);
rr2=rr2*100;
assignin('base','r2',rr2);

rng('shuffle')
rr3=abs(rand);
rr3=rr3*100;
assignin('base','r3',rr3);

rng('shuffle')
rr4=abs(rand);
rr4=rr4*100;
assignin('base','r4',rr4);

rng('shuffle')
rr5=abs(rand);
rr5=rr5*100;
assignin('base','r5',rr5);

%Reset noise in actuators
rng('shuffle')
tt1=abs(rand);
tt1=tt1*100;
assignin('base','t1',tt1);

rng('shuffle')
tt2=abs(rand);
tt2=tt2*100;
assignin('base','t2',tt2);

rng('shuffle')
tt3=abs(rand);
tt3=tt3*100;
assignin('base','t3',tt3);

rng('shuffle')
tt4=abs(rand);
tt4=tt4*100;
assignin('base','t4',tt4);

rng('shuffle')
tt5=abs(rand);
tt5=tt5*100;
assignin('base','t5',tt5);

assignin('base','Kp1',x(1));
assignin('base','Kp2',x(2));
assignin('base','Kp3',x(3));
assignin('base','Kp4',x(4));
assignin('base','Kp5',x(5));
assignin('base','Kv1',x(6));
assignin('base','Kv2',x(7));
assignin('base','Kv3',x(8));
assignin('base','Kv4',x(9));
assignin('base','Kv5',x(10));

modelName='CPC_discreto_ruido_vel_sin.slx';
%modelName='PD_discreto_ruido_vel_sin.slx';
simout = sim(modelName,'FastRestart','on');

N=length(simout.xd(:,end));
xref=simout.xd(:,2);
yref=simout.xd(:,3);
zref=simout.xd(:,4);
xmed=simout.xa(:,2);
ymed=simout.xa(:,3);
zmed=simout.xa(:,4);
errorx=xref-xmed;
errory=yref-ymed;
errorz=zref-zmed;
enq1=simout.energias(:,2);
enq2=simout.energias(:,3);
enq3=simout.energias(:,4);
enq4=simout.energias(:,5);
enq5=simout.energias(:,6);
sumtr=0;
sumqs=0;
for i=1:N
    error_x=errorx(i,1);
    error_y=errory(i,1);
    error_z=errorz(i,1);
    sumtr=sumtr+error_x*error_x+error_y*error_y+error_z*error_z;
    q1=enq1(i,1);
    q2=enq2(i,1);
    q3=enq3(i,1);
    q4=enq4(i,1);
    q5=enq5(i,1);
    sumqs=sumqs+q1*q1+q2*q2+q3*q3+q4*q4+q5*q5;
end
Jtray=sqrt((1/N)*sumtr);
Jener=sqrt((1/N)*sumqs);
alfa=10;
beta=0.01;
J=alfa*Jtray+beta*Jener;

fprintf('Jtray = %.15f\n',Jtray);
fprintf('Jener = %.15f\n',Jener);
fprintf('Error función J1: %.15f\n\n',J);

%pause;
o=J;
end