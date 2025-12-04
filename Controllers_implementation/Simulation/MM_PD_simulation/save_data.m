function save_data()
clear all, clc, close all
warning off
load('pd.mat')
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
Nrep=7;
for j=1:Nrep
    
    assignin('base','num_exp',j);
    clear xd xa energias q1 q2 q3 q4 q5
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
    simout = sim(modelName,'FastRestart','on');
    
    t=simout.q1(:,1);
    enq1=simout.energias(:,2);
    enq2=simout.energias(:,3);
    enq3=simout.energias(:,4);
    enq4=simout.energias(:,5);
    enq5=simout.energias(:,6);
    q1ref=simout.q1(:,2);
    q2ref=simout.q2(:,2);
    q3ref=simout.q3(:,2);
    q4ref=simout.q4(:,2);
    q5ref=simout.q5(:,2);    
    q1med=simout.q1(:,3);
    q2med=simout.q2(:,3);
    q3med=simout.q3(:,3);
    q4med=simout.q4(:,3);
    q5med=simout.q5(:,3);    
    numexp=strcat('exp',num2str(j));
    cpc_data.(numexp)(1,:)=q1ref;
    cpc_data.(numexp)(2,:)=q2ref;
    cpc_data.(numexp)(3,:)=q3ref;
    cpc_data.(numexp)(4,:)=q4ref;
    cpc_data.(numexp)(5,:)=q5ref;
    cpc_data.(numexp)(6,:)=q1med;
    cpc_data.(numexp)(7,:)=q2med;
    cpc_data.(numexp)(8,:)=q3med;
    cpc_data.(numexp)(9,:)=q4med;
    cpc_data.(numexp)(10,:)=q5med;
    cpc_data.(numexp)(11,:)=enq1;
    cpc_data.(numexp)(12,:)=enq2;
    cpc_data.(numexp)(13,:)=enq3;
    cpc_data.(numexp)(14,:)=enq4;
    cpc_data.(numexp)(15,:)=enq5;
    cpc_data.(numexp)(16,:)=t;
end
%save('pd_sim_lab.mat','cpc_data');
disp('Proceso terminado');
end