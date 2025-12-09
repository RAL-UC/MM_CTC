function plot_Prms_test2()
clear all, clc
%CPC Lab
load('CPC_zigzag.mat');
cpc_pot_lab=Jenekrms;
clear Jenekrms
%PD Lab
load('PD_zigzag.mat');
pd_pot_lab=Jenekrms;
clear Jenekrms

tiempo=q_ref(6,:,1);
disp('Data read Ok');
for i=1:10
    %CPC lab:
    %Cargado en inicio
    %CPC Sim
    cpc_pot_sim(i,:)=(cpc_pot_lab(i,:)/10)*rand*0.1;
    %PD Lab
    %Cargado en inicio    
    %PD Sim
    pd_pot_sim(i,:)=(pd_pot_lab(i,:)/10)*rand*0.1;
end
close all
x = tiempo;                                                 % Tiempo: Create Independent Variable
N=length(cpc_pot_lab(1,:));

y1=cpc_pot_sim;
y2=pd_pot_sim;
y3=cpc_pot_lab;
y4=pd_pot_lab;
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