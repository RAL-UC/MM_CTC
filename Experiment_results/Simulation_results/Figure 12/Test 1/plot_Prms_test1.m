function plot_Prms_test1()
close all, clear all, clc
load('ene_cpc_lab.mat')
load('ene_pd_lab.mat')
load('cpc_simlab_int.mat');
encpcsim=enesimcpc_int;
clear enesimcpc_int
load('pd_simlab_int.mat');
enpdsim=enesimcpc_int;
disp('Data read energy Ok');
close all
N=3176;
for i=1:7
    sum_en1=0;
    sum_en2=0;
    sum_en3=0;
    sum_en4=0;
    tot_en_cpc_sim=0;
    tot_en_pd_sim=0;
    tot_en_cpc_lab=0;    
    tot_en_pd_lab=0;    
    for z=1:N
        sum_en1=(cpc_data(1,z,i)^2+cpc_data(2,z,i)^2+...
            cpc_data(3,z,i)^2+cpc_data(4,z,i)^2+cpc_data(5,z,i)^2);
        tot_en_cpc_lab=tot_en_cpc_lab+sum_en1;
        sum_en3=(pd_data(1,z,i)^2+pd_data(2,z,i)^2+...
            pd_data(3,z,i)^2+pd_data(4,z,i)^2+pd_data(5,z,i)^2);
        tot_en_pd_lab=tot_en_pd_lab+sum_en3;
        if z<=3176
            sum_en2=(encpcsim(1,z,i)^2+encpcsim(2,z,i)^2+...
                encpcsim(3,z,i)^2+encpcsim(4,z,i)^2+encpcsim(5,z,i)^2);
            tot_en_cpc_sim=tot_en_cpc_sim+sum_en2;
            sum_en4=(enpdsim(1,z,i)^2+enpdsim(2,z,i)^2+...
                enpdsim(3,z,i)^2+enpdsim(4,z,i)^2+enpdsim(5,z,i)^2);
            tot_en_pd_sim=tot_en_pd_sim+sum_en4;
            en_inst_cpc_sim(i,z)=sqrt(sum_en2);
            en_inst_pd_sim(i,z)=sqrt(sum_en4);
        else
            en_inst_cpc_sim(i,z)=en_inst_cpc_sim(i,z-1);
            en_inst_pd_sim(i,z)=en_inst_pd_sim(i,z-1);
        end
        en_inst_cpc_lab(i,z)=sqrt(sum_en1);
        en_inst_pd_lab(i,z)=sqrt(sum_en3);
    end
    sumtot1(i)=tot_en_cpc_sim;
    sumtot2(i)=tot_en_pd_sim;
    sumtot3(i)=tot_en_cpc_lab;
    sumtot4(i)=tot_en_pd_lab;    
end
mean_en_cpc_sim=mean(en_inst_cpc_sim);
mean_en_pd_sim=mean(en_inst_pd_sim);
mean_en_cpc_lab=mean(en_inst_cpc_lab);
mean_en_pd_lab=mean(en_inst_pd_lab);
prms_cpc_lab=sqrt((1/N)*mean(sumtot3));
prms_cpc_sim=sqrt((1/N)*mean(sumtot1));
prms_pd_lab=sqrt((1/N)*mean(sumtot4));
prms_pd_sim=sqrt((1/N)*mean(sumtot2));
fprintf("Pot rms cpc lab: %.16f\n",prms_cpc_lab);
fprintf("Pot rms cpc sim: %.16f\n",prms_cpc_sim);
fprintf("Pot rms pd lab: %.16f\n",prms_pd_lab);
fprintf("Pot rms pd sim: %.16f\n",prms_pd_sim);
time_final=cpc_data(6,:,1);
%Orden CTC_sim PD_sim CTC_lab PD_lab
y1MeanTot=mean_en_cpc_sim;
y2MeanTot=mean_en_pd_sim;
y3MeanTot=mean_en_cpc_lab;
y4MeanTot=mean_en_pd_lab;
h=figure('WindowState','maximized');
plot(time_final,y1MeanTot,'LineWidth',2.0)
hold on
plot(time_final,y2MeanTot,'LineWidth',2.0)
plot(time_final,y3MeanTot,'LineWidth',2.0)
plot(time_final,y4MeanTot,'LineWidth',2.0)
grid on
a=get(gca,'Children');
lgd=legend({'CTC_{Sim}','PD_{Sim}','CTC_{Lab}','PD_{Lab}'},'Location','northeast','NumColumns',4);
lgd.FontSize = 26;
ax=gca;
ax.XAxis.FontSize = 30;
ax.YAxis.FontSize = 30;
xlabel("Tiempo (s)",'FontSize',36);
ylabel("$P_{rms}$ (J/s)",'FontSize',42,'Interpreter','latex');
title("$P_{rms}$ utilizada para trayectoria de laboratorio",'FontWeight','normal','FontSize',44,'Interpreter','latex')
%ax.Title.FontName='Times';
%savefig(h,'pot_rms.fig');
%exportgraphics(ax,'pot_rms.pdf','ContentType','vector');
disp('Energy calculated Ok');