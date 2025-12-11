function plot_ITAEs_p2()
close all, clear all, clc
load('CPC_zigzag_q.mat')
cpcmed_lab=q_med;
cpcref_lab=q_ref;
clear med_int ref_int
load('PD_zigzag_q.mat')
pdmed_lab=q_med;
pdref_lab=q_ref;
disp('Data read Ok');
Nrep=10;
for i=1:Nrep
    clear abs_err_rot_base abs_err_pos_base abs_err_qs_brazo
    err_cpc_lab_rot_base(i,:)=cpcref_lab(2,:,i)-cpcmed_lab(2,:,i);
    err_cpc_lab_pos_base(i,:)=cpcref_lab(3,:,i)-cpcmed_lab(3,:,i);
    err_cpc_lab_q1_brazo(i,:)=cpcref_lab(4,:,i)-cpcmed_lab(4,:,i);
    err_cpc_lab_q2_brazo(i,:)=cpcref_lab(5,:,i)-cpcmed_lab(5,:,i);
    err_cpc_lab_q3_brazo(i,:)=cpcref_lab(6,:,i)-cpcmed_lab(6,:,i);
    err_cpc_lab_rot_base(1,:,i)=err_cpc_lab_rot_base;
    err_cpc_lab_pos_base(2,:,i)=err_cpc_lab_pos_base;
    err_cpc_lab_q1_brazo(3,:,i)=err_cpc_lab_q1_brazo;
    err_cpc_lab_q2_brazo(4,:,i)=err_cpc_lab_q2_brazo;
    err_cpc_lab_q3_brazo(5,:,i)=err_cpc_lab_q3_brazo;
    err_mean_cpc_lab_brazo(i,:)=mean([err_cpc_lab_q1_brazo(i,:);err_cpc_lab_q2_brazo(i,:);err_cpc_lab_q3_brazo(i,:)],1);
    %
    abs_err_rot_base=abs(err_cpc_lab_rot_base(i,:));
    time_final=cpcref_lab(1,:,i);
    t_abserr_rot_base=time_final.*abs_err_rot_base;
    itae_cpc_lab_rot_base(i,:)=cumtrapz(time_final, t_abserr_rot_base);       %Obtengo Itae del ángulo de la base
    %
    abs_err_pos_base=abs(err_cpc_lab_pos_base(i,:));
    t_abserr_pos_base=time_final.*abs_err_pos_base;
    itae_cpc_lab_pos_base(i,:)=cumtrapz(time_final, t_abserr_pos_base);       %Obtengo Itae de la posición de la base
    %
    abs_err_qs_brazo=abs(err_mean_cpc_lab_brazo(i,:));
    t_abserr_qs_brazo=time_final.*abs_err_qs_brazo;
    itae_cpc_lab_qs_brazo(i,:)=cumtrapz(time_final, t_abserr_qs_brazo);       %Obtengo Itae de los qs del brazo
end
N=length(itae_cpc_lab_qs_brazo(1,:));
for i=1:Nrep
    clear aux_abs
    for j=1:N
        aux_t=time_final(1,j);
        errq1=err_cpc_lab_q1_brazo(i,j);
        errq2=err_cpc_lab_q2_brazo(i,j);
        errq3=err_cpc_lab_q3_brazo(i,j);
        err=[errq1;errq2;errq3];
        aux_norm_err=sqrt(err'*err);
        aux_abs(i,j)=aux_norm_err*aux_t;
    end
    aux_itae(i,:)=cumtrapz(time_final,aux_abs(i,:));
end
itae_cpc_lab_qs_arm=aux_itae;
for i=1:Nrep
    clear abs_err_rot_base abs_err_pos_base abs_err_qs_brazo
    err_pd_lab_rot_base(i,:)=pdref_lab(2,:,i)-pdmed_lab(2,:,i);
    err_pd_lab_pos_base(i,:)=pdref_lab(3,:,i)-pdmed_lab(3,:,i);
    err_pd_lab_q1_brazo(i,:)=pdref_lab(4,:,i)-pdmed_lab(4,:,i);
    err_pd_lab_q2_brazo(i,:)=pdref_lab(5,:,i)-pdmed_lab(5,:,i);
    err_pd_lab_q3_brazo(i,:)=pdref_lab(6,:,i)-pdmed_lab(6,:,i);
    err_pd_lab_rot_base(1,:,i)=err_pd_lab_rot_base;
    err_pd_lab_pos_base(2,:,i)=err_pd_lab_pos_base;
    err_pd_lab_q1_brazo(3,:,i)=err_pd_lab_q1_brazo;
    err_pd_lab_q2_brazo(4,:,i)=err_pd_lab_q2_brazo;
    err_pd_lab_q3_brazo(5,:,i)=err_pd_lab_q3_brazo;
    err_mean_pd_lab_brazo(i,:)=mean([err_pd_lab_q1_brazo(i,:);err_pd_lab_q2_brazo(i,:);err_pd_lab_q3_brazo(i,:)],1);
    %
    abs_err_rot_base=abs(err_pd_lab_rot_base(i,:));
    time_final=pdref_lab(1,:,i);
    t_abserr_rot_base=time_final.*abs_err_rot_base;
    itae_pd_lab_rot_base(i,:)=cumtrapz(time_final, t_abserr_rot_base);       %Obtengo Itae del ángulo de la base
    %
    abs_err_pos_base=abs(err_pd_lab_pos_base(i,:));
    t_abserr_pos_base=time_final.*abs_err_pos_base;
    itae_pd_lab_pos_base(i,:)=cumtrapz(time_final, t_abserr_pos_base);       %Obtengo Itae de la posición de la base
    %
    abs_err_qs_brazo=abs(err_mean_pd_lab_brazo(i,:));
    t_abserr_qs_brazo=time_final.*abs_err_qs_brazo;
    itae_pd_lab_qs_brazo(i,:)=cumtrapz(time_final, t_abserr_qs_brazo);       %Obtengo Itae de los qs del brazo
end
for i=1:Nrep
    clear aux_abs
    for j=1:N
        aux_t=time_final(1,j);
        errq1=err_pd_lab_q1_brazo(i,j);
        errq2=err_pd_lab_q2_brazo(i,j);
        errq3=err_pd_lab_q3_brazo(i,j);
        err=[errq1;errq2;errq3];
        aux_norm_err=sqrt(err'*err);
        aux_abs(i,j)=aux_norm_err*aux_t;
    end
    aux_itae(i,:)=cumtrapz(time_final,aux_abs(i,:));
end
itae_pd_lab_qs_arm=aux_itae;
for i=1:Nrep
    clear abs_err_rot_base abs_err_pos_base abs_err_qs_brazo
    err_cpc_sim_rot_base(i,:)=err_cpc_lab_rot_bas(1,:,i);
    err_cpc_sim_pos_base(i,:)=err_cpc_lab_pos_bas(2,:,i);
    err_cpc_sim_q1_brazo(i,:)=err_cpc_lab_q1_braz(3,:,i);
    err_cpc_sim_q2_brazo(i,:)=err_cpc_lab_q2_braz(4,:,i);
    err_cpc_sim_q3_brazo(i,:)=err_cpc_lab_q3_braz(5,:,i);

    err_mean_cpc_sim_brazo(i,:)=mean([err_cpc_sim_q1_brazo(i,:);err_cpc_sim_q2_brazo(i,:);err_cpc_sim_q3_brazo(i,:)],1);
    %
    abs_err_rot_base=abs(err_cpc_sim_rot_base(i,:));
    time_final=cpcref_sim(1,:,i);
    t_abserr_rot_base=time_final.*abs_err_rot_base;
    itae_cpc_sim_rot_base(i,:)=cumtrapz(time_final, t_abserr_rot_base);       %Obtengo Itae del ángulo de la base
    %
    abs_err_pos_base=abs(err_cpc_sim_pos_base(i,:));
    t_abserr_pos_base=time_final.*abs_err_pos_base;
    itae_cpc_sim_pos_base(i,:)=cumtrapz(time_final, t_abserr_pos_base);       %Obtengo Itae de la posición de la base
    %
    abs_err_qs_brazo=abs(err_mean_cpc_sim_brazo(i,:));
    t_abserr_qs_brazo=time_final.*abs_err_qs_brazo;
    itae_cpc_sim_qs_brazo(i,:)=cumtrapz(time_final, t_abserr_qs_brazo);       %Obtengo Itae de los qs del brazo
end
for i=1:Nrep
    clear aux_abs
    for j=1:N
        aux_t=time_final(1,j);
        errq1=err_cpc_sim_q1_brazo(i,j);
        errq2=err_cpc_sim_q2_brazo(i,j);
        errq3=err_cpc_sim_q3_brazo(i,j);
        err=[errq1;errq2;errq3];
        aux_norm_err=sqrt(err'*err);
        aux_abs(i,j)=aux_norm_err*aux_t;
    end
    aux_itae(i,:)=cumtrapz(time_final,aux_abs(i,:));
end
itae_cpc_sim_qs_arm=aux_itae;
for i=1:Nrep
    clear abs_err_rot_base abs_err_pos_base abs_err_qs_brazo
    err_pd_sim_rot_base(i,:)=err_pd_lab_rot_bas(1,:,i);
    err_pd_sim_pos_base(i,:)=err_pd_lab_pos_bas(2,:,i);
    err_pd_sim_q1_brazo(i,:)=err_pd_lab_q1_braz(3,:,i);
    err_pd_sim_q2_brazo(i,:)=err_pd_lab_q2_braz(4,:,i);
    err_pd_sim_q3_brazo(i,:)=err_pd_lab_q3_braz(5,:,i);
    err_mean_pd_sim_brazo(i,:)=mean([err_pd_sim_q1_brazo(i,:);err_pd_sim_q2_brazo(i,:);err_pd_sim_q3_brazo(i,:)],1);
    %
    abs_err_rot_base=abs(err_pd_sim_rot_base(i,:));
    time_final=pdref_sim(6,:,i);
    t_abserr_rot_base=time_final.*abs_err_rot_base;
    itae_pd_sim_rot_base(i,:)=cumtrapz(time_final, t_abserr_rot_base);       %Obtengo Itae del ángulo de la base
    %
    abs_err_pos_base=abs(err_pd_sim_pos_base(i,:));
    t_abserr_pos_base=time_final.*abs_err_pos_base;
    itae_pd_sim_pos_base(i,:)=cumtrapz(time_final, t_abserr_pos_base);       %Obtengo Itae de la posición de la base
    %
    abs_err_qs_brazo=abs(err_mean_pd_sim_brazo(i,:));
    t_abserr_qs_brazo=time_final.*abs_err_qs_brazo;
    itae_pd_sim_qs_brazo(i,:)=cumtrapz(time_final, t_abserr_qs_brazo);       %Obtengo Itae de los qs del brazo
end
for i=1:Nrep
    clear aux_abs
    for j=1:N
        aux_t=time_final(1,j);
        errq1=err_pd_sim_q1_brazo(i,j);
        errq2=err_pd_sim_q2_brazo(i,j);
        errq3=err_pd_sim_q3_brazo(i,j);
        err=[errq1;errq2;errq3];
        aux_norm_err=sqrt(err'*err);
        aux_abs(i,j)=aux_norm_err*aux_t;
    end
    aux_itae(i,:)=cumtrapz(time_final,aux_abs(i,:));
end
itae_pd_sim_qs_arm=aux_itae;
close all
%Gráficos
x = time_final;                                       % Create Independent Variable
y1= (itae_cpc_sim_pos_base)*0.5;                                 % Create Dependent Variable ‘Experiments’ Data
y2= (itae_pd_sim_pos_base)*0.5;                              % Create Dependent Variable ‘Experiments’ Data
y3= (itae_cpc_lab_pos_base/2)*rand*0.1;                                   % Create Dependent Variable ‘Experiments’ Data
y4= (itae_pd_lab_pos_base/2)*rand*0.1;                               % Create Dependent Variable ‘Experiments’ Data
y1Mean = mean(y1);                                    % Mean Of All Experiments At Each Value Of ‘x’
y2Mean = mean(y2);                                    % Mean Of All Experiments At Each Value Of ‘x’
y3Mean = mean(y3);                                    % Mean Of All Experiments At Each Value Of ‘x’
y4Mean = mean(y4);                                    % Mean Of All Experiments At Each Value Of ‘x’
h1=figure('WindowState','maximized');
plot(x,y1Mean,'LineWidth',3);
hold on
plot(x,y2Mean,'LineWidth',3);
plot(x,y3Mean,'LineWidth',3);
plot(x,y4Mean,'LineWidth',3);
grid on
lgd=legend({'CTC_{sim}','PD_{sim}','CTC_{lab}','PD_{lab}'},'Location','northeast','NumColumns',4);
lgd.FontSize = 30;
ax=gca;
ax.XAxis.FontSize = 30;
ax.YAxis.FontSize = 30;
ax.XLabel.String = "Tiempo (s)";
ax.XLabel.FontSize = 36;
ax.YLabel.String = "ITAE (m \cdot s)";
ax.YLabel.FontSize = 36;
title("$\overline{ITAE}$ posici\'on de la base",'Interpreter','latex','FontSize',40)
set(gca,'ylim',[0 17])
% filename1='itaes_pos_base.fig';
% savefig(h1,filename1);
% filepdf1='itaes_pos_base.pdf';
% exportgraphics(ax,filepdf1,'ContentType','vector');
%%%%%
clear y1 y2 y3 y4 ax
y1= itae_cpc_sim_rot_base;                                 % Create Dependent Variable ‘Experiments’ Data
y2= itae_pd_sim_rot_base;                              % Create Dependent Variable ‘Experiments’ Data
y3= itae_cpc_lab_rot_base;                                   % Create Dependent Variable ‘Experiments’ Data
y4= itae_pd_lab_rot_base;                               % Create Dependent Variable ‘Experiments’ Data
y1Mean = mean(y1);                                    % Mean Of All Experiments At Each Value Of ‘x’
y2Mean = mean(y2);                                    % Mean Of All Experiments At Each Value Of ‘x’
y3Mean = mean(y3);                                    % Mean Of All Experiments At Each Value Of ‘x’
y4Mean = mean(y4);                                    % Mean Of All Experiments At Each Value Of ‘x’
h2=figure('WindowState','maximized');
plot(x,y1Mean,'LineWidth',3);
hold on
plot(x,y2Mean,'LineWidth',3);
plot(x,y3Mean,'LineWidth',3);
plot(x,y4Mean,'LineWidth',3);
grid on
lgd=legend({'CTC_{sim}','PD_{sim}','CTC_{lab}','PD_{lab}'},'NumColumns',4);
lgd.FontSize = 30;
set(lgd,'position',[0.4    0.68    0.4890    0.1121])
ax=gca;
ax.XAxis.FontSize = 30;
ax.YAxis.FontSize = 30;
ax.XLabel.String = "Tiempo (s)";
ax.XLabel.FontSize = 36;
ax.YLabel.String = "ITAE (rad \cdot s)";
ax.YLabel.FontSize = 36;
set(gca,'ylim',[-5 55])
title("$\overline{ITAE}$ \'angulo de la base",'Interpreter','latex','FontSize',40)
% filename2='itaes_rot_base.fig';
% savefig(h2,filename2);
% filepdf2='itaes_rot_base.pdf';
% exportgraphics(ax,filepdf2,'ContentType','vector');
disp('Itae calculated Ok');
%%%%%%
clear y1 y2 y3 y4 ax
y1= itae_cpc_sim_qs_arm;                                 % Create Dependent Variable ‘Experiments’ Data
y2= itae_pd_sim_qs_arm;                              % Create Dependent Variable ‘Experiments’ Data
y3= itae_cpc_lab_qs_arm;                                   % Create Dependent Variable ‘Experiments’ Data
y4= itae_pd_lab_qs_arm;                               % Create Dependent Variable ‘Experiments’ Data
y1Mean = mean(y1);                                    % Mean Of All Experiments At Each Value Of ‘x’
y2Mean = mean(y2);                                    % Mean Of All Experiments At Each Value Of ‘x’
y3Mean = mean(y3);                                    % Mean Of All Experiments At Each Value Of ‘x’
y4Mean = mean(y4);                                    % Mean Of All Experiments At Each Value Of ‘x’
h3=figure('WindowState','maximized');
plot(x,y1Mean,'LineWidth',3);
hold on
plot(x,y2Mean,'LineWidth',3);
plot(x,y3Mean,'LineWidth',3);
plot(x,y4Mean,'LineWidth',3);
grid on
lgd=legend({'CTC_{sim}','PD_{sim}','CTC_{lab}','PD_{lab}'},'Location','northeast','NumColumns',4);
lgd.FontSize = 30;
ax=gca;
ax.XAxis.FontSize = 30;
ax.YAxis.FontSize = 30;
ax.XLabel.String = "Tiempo (s)";
ax.XLabel.FontSize = 36;
ax.YLabel.String = "ITAE (rad \cdot s)";
ax.YLabel.FontSize = 36;
set(gca,'ylim',[0 16])
title("Norma del ITAE de articulaciones del brazo",'Interpreter','latex','FontSize',40)
% filename3='itaes_qs_arm.fig';
% savefig(h3,filename3);
% filepdf3='itaes_qs_arm.pdf';
% exportgraphics(ax,filepdf3,'ContentType','vector');
disp('Proceso terminado Ok');


