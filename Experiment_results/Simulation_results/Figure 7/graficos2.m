function graficos2()
clear all, close all, clc
cont_exp=4;
if cont_exp==1
    exp='cuadrado_xy';        %cuadrado_03_xy 1200 muestras
elseif cont_exp==2
    exp='cuadrado_yz';        %cuadrado_yz_1mm_b 4000 muestras
elseif cont_exp==3
    exp='hélice_xy';        %helice_xy 2001 muestras
elseif cont_exp==4
    exp='hélice_yz';        %helice_yz 2001 muestras
end
file=strcat('CPC_',exp,'_grafico.mat');
load(file);
fprintf('Datos leídos existosamente de %s\n',exp);
N=length(Jtraymean);
cpc_Jkrms=Jkrms;
cpc_Jenermean=Jenermean;
clear Jkrms Jenekrms Jtraymean Jenermean
file=strcat('PD_',exp,'_grafico.mat');
load(file);
pd_Jkrms=Jkrms;
pd_Jenermean=Jenermean;
x=t/10;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
y1=cpc_Jkrms;
y2=pd_Jkrms;
y1Mean = mean(y1);                                    % Mean Of All Experiments At Each Value Of ‘x’
y2Mean = mean(y2);                                    % Mean Of All Experiments At Each Value Of ‘x’
y1SEM = std(y1)/sqrt(N);                              % Compute ‘Standard Error Of The Mean’ Of All Experiments At Each Value Of ‘x’
y2SEM = std(y2)/sqrt(N);                              % Compute ‘Standard Error Of The Mean’ Of All Experiments At Each Value Of ‘x’
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
CI95 = tinv([0.025 0.975], N-1);                    % Calculate 95% Probability Intervals Of t-Distribution
y1CI95 = bsxfun(@times, y1SEM, CI95(:));              % Calculate 95% Confidence Intervals Of All Experiments At Each Value Of ‘x’
y2CI95 = bsxfun(@times, y2SEM, CI95(:));              % Calculate 95% Confidence Intervals Of All Experiments At Each Value Of ‘x’
h=figure('WindowState','maximized');
P1=subplot(2,1,1);
[l,p]=boundedline(x, y1Mean, y1CI95(2,:),'b',x, y2Mean, y2CI95(2,:),'r',...
		  'LineWidth',6.5,'transparency', 0.1);
ho=outlinebounds(l,p);
set(ho,{'LineWidth'},{1.5;1.5},'Color',[0.80,0.80,0.80]);
grid
set(get(P1,'XLabel'), 'String', 'Tiempo (s)',  'FontSize',30);
set(get(P1,'YLabel'), 'String', 'e_{rms} (mm)','FontSize',25);
set(get(P1,'XAxis'), 'FontSize',20);
set(get(P1,'YAxis'), 'FontSize',20);
P2=subplot(2,1,2);
plot(x,cpc_Jenermean);
hold on
plot(x,pd_Jenermean,'r');
grid
set(get(P2,'XLabel'), 'String', 'Tiempo (s)',  'FontSize',30);
set(get(P2,'YLabel'), 'String', 'P_{rms} (J/s)','FontSize',30);
set(get(P2,'XAxis'), 'FontSize',20);
set(get(P2,'YAxis'), 'FontSize',20);
%
file2=strcat(exp,'_error_energia');
savefig(h,strcat(file2,'.fig'));
exportgraphics(gcf,strcat(file2,'.pdf'),'ContentType','vector','Resolution',300) 
disp('Gráfico guardado');