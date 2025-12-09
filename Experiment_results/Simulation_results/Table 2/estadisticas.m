function estadisticas()
clc, close all
warning off

load('pd.mat')
%load('cpc.mat')
Kp1=Swarm_save{1,end}.GBEST.X(1);
Kp2=Swarm_save{1,end}.GBEST.X(2);
Kp3=Swarm_save{1,end}.GBEST.X(3);
Kp4=Swarm_save{1,end}.GBEST.X(4);
Kp5=Swarm_save{1,end}.GBEST.X(5);
Kv1=Swarm_save{1,end}.GBEST.X(6);
Kv2=Swarm_save{1,end}.GBEST.X(7);
Kv3=Swarm_save{1,end}.GBEST.X(8);
Kv4=Swarm_save{1,end}.GBEST.X(9);
Kv5=Swarm_save{1,end}.GBEST.X(10);
assignin('base','Kp1',Kp1);
assignin('base','Kp2',Kp2);
assignin('base','Kp3',Kp3);
assignin('base','Kp4',Kp4);
assignin('base','Kp5',Kp5);
assignin('base','Kv1',Kv1);
assignin('base','Kv2',Kv2);
assignin('base','Kv3',Kv3);
assignin('base','Kv4',Kv4);
assignin('base','Kv5',Kv5);

Nrep=10;
cont_exp=4;
if cont_exp==1
    stoptime=119.99;        %cuadrado_03_xy 1200 muestras
elseif cont_exp==2
    stoptime=399.99;        %cuadrado_yz_1mm_b 4000 muestras
elseif cont_exp==3
    stoptime=200;        %helice_xy 2001 muestras
elseif cont_exp==4
    stoptime=200;        %helice_yz 2001 muestras
end
for j=1:Nrep
    
    clear xd xa energias
    fprintf('Comenzando interación %d\n',j);
    
    %Reinicio ruido en los sensores
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
    
    %Reinicio ruido en los actuadores
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
    
    modelName='PD_discreto_ruido_vel.slx';
    %modelName='CPC_discreto_ruido_vel.slx';
    simout = sim(modelName,'StopTime',num2str(stoptime),'FastRestart','on');
    
    N=length(simout.xd(:,end));
    t=simout.xd(:,1);
    xd=simout.xd(:,2:4);
    xa=simout.xa(:,2:4);
    xdd(:,:,j)=xd;
    xaa(:,:,j)=xa;
    xref=xd(:,1);
    yref=xd(:,2);
    zref=xd(:,3);
    xmed=xa(:,1);
    ymed=xa(:,2);
    zmed=xa(:,3);
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
    Jtray(j)=sqrt((1/N)*sumtr);
    Jener(j)=sqrt((1/N)*sumqs);
    errloopclo(j)=norm(xd(end,1:3)-xa(end,1:3));
%     fprintf('Jtray = %.15f\n',Jtray(j));
%     fprintf('Jener = %.15f\n',Jener(j));
%     fprintf('Error loop closure = %.15f\n\n',errloopclo(j));
end
if cont_exp==1
    exp='cuadrado_xy';        %cuadrado_03_xy 1200 muestras
elseif cont_exp==2
    exp='cuadrado_yz';        %cuadrado_yz_1mm_b 4000 muestras
elseif cont_exp==3
    exp='hélice_xy';        %helice_xy 2001 muestras
elseif cont_exp==4
    exp='hélice_yz';        %helice_yz 2001 muestras
end
fprintf("\nDatos calculados de experimento: %s\n\n",exp);
Jtraymean=mean(Jtray);
Jenermean=mean(Jener);
errclorms=mean(errloopclo);
fprintf("Jtray rms para experimento: %.16f\n",Jtraymean);
fprintf("Jener rms para experimento: %.16f\n",Jenermean);
fprintf("Error de cierre rms para experimento: %.16f\n",errclorms);
sumJtray=0;
for i=1:Nrep
    sumJtray=sumJtray+(Jtray(i)-Jtraymean)^2;
end
varJtray=(1/Nrep)*sumJtray;
fprintf("var J cpc lab: %.16f\n",varJtray);
CIJtray=1.96*sqrt(varJtray)/(sqrt(Nrep));
fprintf("CI J tray: %.16f\n",CIJtray);
end