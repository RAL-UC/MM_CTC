function calcula_datos_lab_2()
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
cpcpot_sim=enesimcpc_int;
clear refsimcpc_int medsimcpc_int enesimcpc_int
load('pd_simlab_int.mat');
pdref_sim=refsimcpc_int;
pdmed_sim=medsimcpc_int;
pdpot_sim=enesimcpc_int;
load('ene_cpc_lab.mat');
cpcpot_lab=cpc_data;
load('ene_pd_lab.mat');
pdpot_lab=pd_data;
disp('Data read Ok');
N=3176;
%N=3187;
for i=1:7
    sum_pot_cpc_lab=0;
    sum_pot_cpc_sim=0;
    sum_pot_pd_lab=0;
    sum_pot_pd_sim=0;
    sum_err_cpc_lab=0;
    sum_err_cpc_sim=0;
    sum_err_pd_lab=0;
    sum_err_pd_sim=0;
    for h=1:N
        %CPC data lab
        pos_ref_cpc_lab=cal_cindir(deg2rad(cpcref_int(1,h,i)),cpcref_int(2,h,i),...
                        cpcref_int(3,h,i),cpcref_int(4,h,i),cpcref_int(5,h,i));
        pos_med_cpc_lab=cal_cindir(deg2rad(cpcmed_int(1,h,i)),cpcmed_int(2,h,i),...
                        cpcmed_int(3,h,i),cpcmed_int(4,h,i),cpcmed_int(5,h,i));        
        xcpc_err_lab = pos_ref_cpc_lab(1)-pos_med_cpc_lab(1);
        ycpc_err_lab = pos_ref_cpc_lab(2)-pos_med_cpc_lab(2);
        zcpc_err_lab = pos_ref_cpc_lab(3)-pos_med_cpc_lab(3);
        sum_err_cpc_lab=sum_err_cpc_lab+(xcpc_err_lab^2+ycpc_err_lab^2+zcpc_err_lab^2);
        %pd data lab
        pos_ref_pd_lab=cal_cindir(deg2rad(pdref_int(1,h,i)),pdref_int(2,h,i),...
                       pdref_int(3,h,i),pdref_int(4,h,i),pdref_int(5,h,i));
        pos_med_pd_lab=cal_cindir(deg2rad(pdmed_int(1,h,i)),pdmed_int(2,h,i),...
                       pdmed_int(3,h,i),pdmed_int(4,h,i),pdmed_int(5,h,i));        
        xpd_err_lab = pos_ref_pd_lab(1)-pos_med_pd_lab(1);
        ypd_err_lab = pos_ref_pd_lab(2)-pos_med_pd_lab(2);
        zpd_err_lab = pos_ref_pd_lab(3)-pos_med_pd_lab(3);
        sum_err_pd_lab=sum_err_pd_lab+(xpd_err_lab^2+ypd_err_lab^2+zpd_err_lab^2);
        %CPC data sim
        pos_ref_cpc_sim=cal_cindir(cpcref_sim(1,h,i),cpcref_sim(2,h,i),...
                        cpcref_sim(3,h,i),cpcref_sim(4,h,i),cpcref_sim(5,h,i));
        pos_med_cpc_sim=cal_cindir(cpcmed_sim(1,h,i),cpcmed_sim(2,h,i),...
                        cpcmed_sim(3,h,i),cpcmed_sim(4,h,i),cpcmed_sim(5,h,i));        
        xcpc_err_sim = pos_ref_cpc_sim(1)-pos_med_cpc_sim(1);
        ycpc_err_sim = pos_ref_cpc_sim(2)-pos_med_cpc_sim(2);
        zcpc_err_sim = pos_ref_cpc_sim(3)-pos_med_cpc_sim(3);
        sum_err_cpc_sim=sum_err_cpc_sim+(xcpc_err_sim^2+ycpc_err_sim^2+zcpc_err_sim^2);
        %pd data sim
        pos_ref_pd_sim=cal_cindir(pdref_sim(1,h,i),pdref_sim(2,h,i),...
                        pdref_sim(3,h,i),pdref_sim(4,h,i),pdref_sim(5,h,i));
        pos_med_pd_sim=cal_cindir(pdmed_sim(1,h,i),pdmed_sim(2,h,i),...
                        pdmed_sim(3,h,i),pdmed_sim(4,h,i),pdmed_sim(5,h,i));        
        xpd_err_sim = pos_ref_pd_sim(1)-pos_med_pd_sim(1);
        ypd_err_sim = pos_ref_pd_sim(2)-pos_med_pd_sim(2);
        zpd_err_sim = pos_ref_pd_sim(3)-pos_med_pd_sim(3);
        sum_err_pd_sim=sum_err_pd_sim+(xpd_err_sim^2+ypd_err_sim^2+zpd_err_sim^2);
        %Potencias
        sum_pot_cpc_lab=sum_pot_cpc_lab+cpcpot_lab(1,h,i)^2+cpcpot_lab(2,h,i)^2+...
                        cpcpot_lab(3,h,i)^2+cpcpot_lab(4,h,i)^2+cpcpot_lab(5,h,i)^2;
        sum_pot_cpc_sim=sum_pot_cpc_sim+cpcpot_sim(1,h,i)^2+cpcpot_sim(2,h,i)^2+...
                        cpcpot_sim(3,h,i)^2+cpcpot_sim(4,h,i)^2+cpcpot_sim(5,h,i)^2;
        sum_pot_pd_lab=sum_pot_pd_lab+pdpot_lab(1,h,i)^2+pdpot_lab(2,h,i)^2+...
                        pdpot_lab(3,h,i)^2+pdpot_lab(4,h,i)^2+pdpot_lab(5,h,i)^2;
        sum_pot_pd_sim=sum_pot_pd_sim+pdpot_sim(1,h,i)^2+pdpot_sim(2,h,i)^2+...
                        pdpot_sim(3,h,i)^2+pdpot_sim(4,h,i)^2+pdpot_sim(5,h,i)^2;   
    end
    ene_cpc_lab(i)=sqrt((1/N)*sum_pot_cpc_lab);
    ene_cpc_sim(i)=sqrt((1/N)*sum_pot_cpc_sim);  
    ene_pd_lab(i)=sqrt((1/N)*sum_pot_pd_lab);
    ene_pd_sim(i)=sqrt((1/N)*sum_pot_pd_sim); 
    J1(i)=sqrt((1/N)*sum_err_cpc_lab);
    J2(i)=sqrt((1/N)*sum_err_cpc_sim);
    J3(i)=sqrt((1/N)*sum_err_pd_lab);    
    J4(i)=sqrt((1/N)*sum_err_pd_sim);  
    err_cierr_cpc_lab(i)=norm(pos_ref_cpc_lab-pos_med_cpc_lab);
    err_cierr_cpc_sim(i)=norm(pos_ref_cpc_sim-pos_med_cpc_sim);
    err_cierr_pd_lab(i)=norm(pos_ref_pd_lab-pos_med_pd_lab);
    err_cierr_pd_sim(i)=norm(pos_ref_pd_sim-pos_med_pd_sim);
end
close all
pot_cpc_lab=mean(ene_cpc_lab);
pot_cpc_sim=mean(ene_cpc_sim);
pot_pd_lab=mean(ene_pd_lab);
pot_pd_sim=mean(ene_pd_sim);
fprintf('\nDatos de Potencia: \n\n');
fprintf('Potencia CPC lab: %.16f\n',pot_cpc_lab);
fprintf('Potencia CPC sim: %.16f\n',pot_cpc_sim);
fprintf('Potencia PD lab: %.16f\n',pot_pd_lab);
fprintf('Potencia PD sim: %.16f\n',pot_pd_sim);
%%%
J1mean=mean(J1);
J2mean=mean(J2);
J3mean=mean(J3);
J4mean=mean(J4);
fprintf("J para cpc lab: %.16f\n",J1mean);
fprintf("J para cpc sim: %.16f\n",J2mean);
fprintf("J para pd lab: %.16f\n",J3mean);
fprintf("J para pd sim: %.16f\n",J4mean);
sumJ1=0;
sumJ2=0;
sumJ3=0;
sumJ4=0;
for i=1:7
    sumJ1=sumJ1+(J1(i)-J1mean)^2;
    sumJ2=sumJ2+(J2(i)-J2mean)^2;
    sumJ3=sumJ3+(J3(i)-J3mean)^2;
    sumJ4=sumJ4+(J4(i)-J4mean)^2;
end
varJ1=(1/i)*sumJ1;
varJ2=(1/i)*sumJ2;
varJ3=(1/i)*sumJ3;
varJ4=(1/i)*sumJ4;
fprintf("var J cpc lab: %.16f\n",varJ1);
fprintf("var J cpc sim: %.16f\n",varJ2);
fprintf("var J pd lab: %.16f\n",varJ3);
fprintf("var J pd sim: %.16f\n",varJ4);
CIJ1=1.96*sqrt(varJ1)/(sqrt(i));
CIJ2=1.96*sqrt(varJ2)/(sqrt(i));
CIJ3=1.96*sqrt(varJ3)/(sqrt(i));
CIJ4=1.96*sqrt(varJ4)/(sqrt(i));
fprintf("CI J cpc lab: %.16f\n",CIJ1);
fprintf("CI J cpc sim: %.16f\n",CIJ2);
fprintf("CI J pd lab: %.16f\n",CIJ3);
fprintf("CI J pd sim: %.16f\n",CIJ4);
err_cierre_cpc_lab=mean(err_cierr_cpc_lab);
err_cierre_cpc_sim=mean(err_cierr_cpc_sim);
err_cierre_pd_lab=mean(err_cierr_pd_lab);
err_cierre_pd_sim=mean(err_cierr_pd_sim);
fprintf("err cierre cpc lab: %.16f\n",err_cierre_cpc_lab);
fprintf("err cierre cpc sim: %.16f\n",err_cierre_cpc_sim);
fprintf("err cierre pd lab: %.16f\n",err_cierre_pd_lab);
fprintf("err cierre pd sim: %.16f\n",err_cierre_pd_sim);
end
