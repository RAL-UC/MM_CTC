function plot_xee_test1()
clear all, clc
load('cpcdata_lab.mat')
%load('cpcdata.mat')
cpcmed_int=med_int;
cpcref_int=ref_int;
clear med_int ref_int
load('pddata_lab.mat')
%load('pddata.mat')
pdmed_int=med_int;
pdref_int=ref_int;
tiempo=ref_int(6,:,1);
clear med_int ref_int
load('cpc_simlab_int.mat');
cpcref_sim=refsimcpc_int;
cpcmed_sim=medsimcpc_int;
clear med_int ref_int
load('pd_simlab_int.mat');
pdref_sim=refsimcpc_int;
pdmed_sim=medsimcpc_int;
disp('Data read Ok');
for i=1:7
    %CPC lab:
    clear errx erry errz
    qcpcref_lab=cpcref_int(1:5,:,i);          %Calculo el error de los q_j
    qcpcmed_lab=cpcmed_int(1:5,:,i);          %Calculo el error de los q_j
    xee_cpcref_lab(:,:,i)=cal_cindir2(qcpcref_lab(1,:),qcpcref_lab(2,:),qcpcref_lab(3,:),qcpcref_lab(4,:),qcpcref_lab(5,:));
    xee_cpcmed_lab(:,:,i)=cal_cindir2(qcpcmed_lab(1,:),qcpcmed_lab(2,:),qcpcmed_lab(3,:),qcpcmed_lab(4,:),qcpcmed_lab(5,:));
    errx=xee_cpcref_lab(1,:,i)-xee_cpcmed_lab(1,:,i);
    erry=xee_cpcref_lab(2,:,i)-xee_cpcmed_lab(2,:,i);
    errz=xee_cpcref_lab(3,:,i)-xee_cpcmed_lab(3,:,i);
    nerrcpc_lab(i,:)=sqrt(errx.^2+erry.^2+errz.^2);
    %CPC Sim
    clear errx erry errz
    qcpcref_sim=cpcref_sim(1:5,:,i);          %Calculo el error de los q_j
    qcpcmed_sim=cpcmed_sim(1:5,:,i);          %Calculo el error de los q_j
    xee_cpcref_sim(:,:,i)=cal_cindir2(qcpcref_sim(1,:),qcpcref_sim(2,:),qcpcref_sim(3,:),qcpcref_sim(4,:),qcpcref_sim(5,:));
    xee_cpcmed_sim(:,:,i)=cal_cindir2(qcpcmed_sim(1,:),qcpcmed_sim(2,:),qcpcmed_sim(3,:),qcpcmed_sim(4,:),qcpcmed_sim(5,:));
    errx=xee_cpcref_sim(1,:,i)-xee_cpcmed_sim(1,:,i);
    erry=xee_cpcref_sim(2,:,i)-xee_cpcmed_sim(2,:,i);
    errz=xee_cpcref_sim(3,:,i)-xee_cpcmed_sim(3,:,i);
    nerrcpc_sim(i,:)=sqrt(errx.^2+erry.^2+errz.^2);
    %PD Lab
    clear errx erry errz
    qpdref_lab=pdref_int(1:5,:,i);          %Calculo el error de los q_j
    qpdmed_lab=pdmed_int(1:5,:,i);          %Calculo el error de los q_j
    xee_pdref_lab(:,:,i)=cal_cindir2(qpdref_lab(1,:),qpdref_lab(2,:),qpdref_lab(3,:),qpdref_lab(4,:),qpdref_lab(5,:));
    xee_pdmed_lab(:,:,i)=cal_cindir2(qpdmed_lab(1,:),qpdmed_lab(2,:),qpdmed_lab(3,:),qpdmed_lab(4,:),qpdmed_lab(5,:));
    errx=xee_pdref_lab(1,:,i)-xee_pdmed_lab(1,:,i);
    erry=xee_pdref_lab(2,:,i)-xee_pdmed_lab(2,:,i);
    errz=xee_pdref_lab(3,:,i)-xee_pdmed_lab(3,:,i);
    nerrpd_lab(i,:)=sqrt(errx.^2+erry.^2+errz.^2);
    %PD Sim
    clear errx erry errz
    qpdref_sim=pdref_sim(1:5,:,i);          %Calculo el error de los q_j
    qpdmed_sim=pdmed_sim(1:5,:,i);          %Calculo el error de los q_j
    xee_pdref_sim(:,:,i)=cal_cindir2(qpdref_sim(1,:),qpdref_sim(2,:),qpdref_sim(3,:),qpdref_sim(4,:),qpdref_sim(5,:));
    xee_pdmed_sim(:,:,i)=cal_cindir2(qpdmed_sim(1,:),qpdmed_sim(2,:),qpdmed_sim(3,:),qpdmed_sim(4,:),qpdmed_sim(5,:));
    errx=xee_pdref_sim(1,:,i)-xee_pdmed_sim(1,:,i);
    erry=xee_pdref_sim(2,:,i)-xee_pdmed_sim(2,:,i);
    errz=xee_pdref_sim(3,:,i)-xee_pdmed_sim(3,:,i);
    nerrpd_sim(i,:)=sqrt(errx.^2+erry.^2+errz.^2);
end
close all
x = tiempo;                                                 % Tiempo: Create Independent Variable
N=length(pdmed_sim(1,:,1));
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% for j=1:7
%     for i=1:N
%         nerrcpc_lab(j,i)=nerrcpc_lab(j,i)+rand*0.0001;
%     end
% end
y1=nerrcpc_sim;
y2=nerrpd_sim;
y3=nerrcpc_lab;
y4=nerrpd_lab;
y1Mean = mean(y1);                                    % Mean Of All Experiments At Each Value Of ‘x’
y2Mean = mean(y2);                                    % Mean Of All Experiments At Each Value Of ‘x’
y3Mean = mean(y3);                                    % Mean Of All Experiments At Each Value Of ‘x’
y4Mean = mean(y4);                                    % Mean Of All Experiments At Each Value Of ‘x’
y1SEM = std(y1)/sqrt(N);                              % Compute ‘Standard Error Of The Mean’ Of All Experiments At Each Value Of ‘x’
y2SEM = std(y2)/sqrt(N);                              % Compute ‘Standard Error Of The Mean’ Of All Experiments At Each Value Of ‘x’
y3SEM = std(y3)/sqrt(N);                              % Compute ‘Standard Error Of The Mean’ Of All Experiments At Each Value Of ‘x’
y4SEM = std(y4)/sqrt(N);                              % Compute ‘Standard Error Of The Mean’ Of All Experiments At Each Value Of ‘x’
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
CI95 = tinv([0.025 0.975], N-1);                    % Calculate 95% Probability Intervals Of t-Distribution
y1CI95 = bsxfun(@times, y1SEM, CI95(:));              % Calculate 95% Confidence Intervals Of All Experiments At Each Value Of ‘x’
y2CI95 = bsxfun(@times, y2SEM, CI95(:));              % Calculate 95% Confidence Intervals Of All Experiments At Each Value Of ‘x’
y3CI95 = bsxfun(@times, y3SEM, CI95(:));              % Calculate 95% Confidence Intervals Of All Experiments At Each Value Of ‘x’
y4CI95 = bsxfun(@times, y4SEM, CI95(:));              % Calculate 95% Confidence Intervals Of All Experiments At Each Value Of ‘x’
h=figure('WindowState','maximized');

str1 = 	'#0072BD';
color1 = sscanf(str1(2:end),'%2x%2x%2x',[1 3])/255;
str2 = 	'#D95319';
color2 = sscanf(str2(2:end),'%2x%2x%2x',[1 3])/255;
str3 = 	'#EDB120';
color3 = sscanf(str3(2:end),'%2x%2x%2x',[1 3])/255;
str4 = 	'#7E2F8E';
color4 = sscanf(str4(2:end),'%2x%2x%2x',[1 3])/255;

[l,p]=boundedline(x, y1Mean, y1CI95(2,:),x, y2Mean, y2CI95(2,:),...
    x, y3Mean, y3CI95(2,:),x, y4Mean, y4CI95(2,:),...
    'Color',[color1;color2;color3;color4],'LineWidth',5.0,'transparency', 0.1);

ho=outlinebounds(l,p);
set(ho,{'LineWidth'},{0.1;0.1;0.1;0.1},'Color',[0.80,0.80,0.80]);
grid
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
a=get(gca,'Children');
lgd=legend([a(8);a(7);a(6);a(5)],{'CTC_{Sim}','PD_{Sim}','CTC_{Lab}','PD_{Lab}'},'Location','northeast','NumColumns',4);
lgd.FontSize = 26;
ax=gca;
ax.XAxis.FontSize = 30;
ax.YAxis.FontSize = 30;
xlabel("Tiempo (s)",'FontSize',40);
ylabel("$e_{rms}$ (m)",'FontSize',42,'Interpreter','latex');
title("$e_{rms}$ del extremo efector del MM",'FontWeight','normal','FontSize',44,'Interpreter','latex')
% savefig(h,'error_rms.fig');
%exportgraphics(ax,'error_rms.pdf','ContentType','vector');
% disp('Norma del Extremo Efector Calculada');
% exportgraphics(gcf,'error_rms.pdf','ContentType','vector');