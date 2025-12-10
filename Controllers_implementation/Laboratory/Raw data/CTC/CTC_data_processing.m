function CTC_data_processing()
clear all, clc
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
time_final=0:(18.565429/2586.5):22.87;
NN=length(time_final);
ngdl=5;
numexp=7;
ref_int=zeros(ngdl+1,NN,numexp);	%Fila extra para el vector de tiempo interpolado
med_int=zeros(ngdl+1,NN,numexp);	%Fila extra para el vector de tiempo interpolado
for i=1:7
   ref_int(6,:,i)=time_final;
   med_int(6,:,i)=time_final;
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
for i=1:7
    num_exp = strcat('exp',num2str(i));
    file_thbase  = strcat('itae_base_th_',num2str(i),'.txt');
    file_posbase = strcat('itae_base_',num2str(i),'.txt');
    thbase.(num_exp)  = readmatrix(file_thbase,'Delimiter',{'\t',' '});
    posbase.(num_exp) = readmatrix(file_posbase,'Delimiter',{'\t',' '});
    N_thbase(i)=length(thbase.(num_exp));
    N_posbase(i)=length(posbase.(num_exp));
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %PARA TH DE LA BASE
    clear time_aux
    %Obtener tiempo máximo utilizado para interpolar ángulo de la base
    timemax_baseth(i)=thbase.(num_exp)(end,3);
    %Creación de vector auxiliar de tiempo
    idx_timemax_baseth(i)=min(find(time_final>=timemax_baseth(i)));	%índice de posicionamiento en vector de tiempo total
    time_aux=time_final(1,1:idx_timemax_baseth(i));
    %Interpolación vectores de referencia y medición del ángulo de la base
    refth.(num_exp)=thbase.(num_exp)(:,5);
    medth.(num_exp)=thbase.(num_exp)(:,6);
    time_refth.(num_exp)=thbase.(num_exp)(:,3);
    inter_refth.(num_exp)=interp1(time_refth.(num_exp),refth.(num_exp),time_aux,'pchip');
    inter_medth.(num_exp)=interp1(time_refth.(num_exp),medth.(num_exp),time_aux,'pchip'); 
    %Guardo interpolación en vector de referencia
    ref_int(1,1:length(inter_refth.(num_exp)),i)=inter_refth.(num_exp);
    med_int(1,1:length(inter_medth.(num_exp)),i)=inter_medth.(num_exp);   
%     %rescato último valor del vector de referencia y medición, y relleno el vector ref_int con esos valores
%     ref_int(1,(length(inter_refth.(num_exp))+1):end,i)=refth.(num_exp)(end);    %Primer gdl ref(th base)
%     %med_int(1,(length(inter_medth.(num_exp))+1):end,i)=medth.(num_exp)(end);    %Primer gdl med(th base)
%     med_int(1,(length(inter_medth.(num_exp))+1):end,i)=refth.(num_exp)(end)+rand;    %Primer gdl med(th base)
%     %med_int(1,(length(inter_medth.(num_exp))+1):end,i)=refth.(num_exp)(end);    %Primer gdl med(th base)
    ref_int(1,(length(inter_refth.(num_exp))+1):end,i)=ref_int(1,length(inter_refth.(num_exp)),i);
    med_int(1,(length(inter_medth.(num_exp))+1):end,i)=med_int(1,length(inter_refth.(num_exp)),i);
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %PARA POS DE LA BASE
    clear time_aux
    %Obtener tiempo máximo utilizado para interpolar posición de la base
    timemax_basepos(i)=posbase.(num_exp)(end,3);    
    %Creación de vector auxiliar de tiempo
    idx_timemax_basepos(i)=min(find(time_final>=timemax_basepos(i)));	%índice de posicionamiento en vector de tiempo total
    time_aux=time_final(1,1:idx_timemax_basepos(i));
    %Interpolación vectores de referencia y medición del ángulo de la base
    refpos.(num_exp)=posbase.(num_exp)(:,4);
    medpos.(num_exp)=posbase.(num_exp)(:,5);
    time_refpos.(num_exp)=posbase.(num_exp)(:,3);
    inter_refpos.(num_exp)=interp1(time_refpos.(num_exp),refpos.(num_exp),time_aux,'pchip');
    inter_medpos.(num_exp)=interp1(time_refpos.(num_exp),medpos.(num_exp),time_aux,'pchip'); 
    %Guardo interpolación en vector de referencia
    ref_int(2,1:length(inter_refpos.(num_exp)),i)=inter_refpos.(num_exp);
    med_int(2,1:length(inter_medpos.(num_exp)),i)=inter_medpos.(num_exp);   
    %rescato último valor del vector de referencia y medición, y relleno el vector ref_int con esos valores
%   ref_int(2,(length(inter_refpos.(num_exp))+1):end,i)=refpos.(num_exp)(end);    %Segundo gdl ref (pos base)
%   med_int(2,(length(inter_medpos.(num_exp))+1):end,i)=medpos.(num_exp)(end);    %Segundo gdl med (pos base)
    %med_int(2,(length(inter_medpos.(num_exp))+1):end,i)=refpos.(num_exp)(end);    %Segundo gdl med (pos base)
    ref_int(1,(length(inter_refth.(num_exp))+1):end,i)=ref_int(1,length(inter_refth.(num_exp)),i);
    med_int(1,(length(inter_medth.(num_exp))+1):end,i)=med_int(1,length(inter_refth.(num_exp)),i);
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%PARA EL BRAZO
%Orden dentro del fichero
%datos_th_N (Brazo):
%th0_ref,th0_ac,th0_err,th0_itae,tac_interp
%%%
epcm1=51200;
epcm2=94976;
epcm3=47488;
epcm=[epcm1; epcm2; epcm3];
cont=1;
for i=0:2
    for j=1:7
        num_art = num2str(i);
        file_arm= strcat('datos_th',num_art,'_',num2str(j),'.txt');
        num_pos = strcat('art',num_art,'_',num2str(j));
        posth.(num_pos)=readmatrix(file_arm,'Delimiter',{'\t',' '});
        aux_med=posth.(num_pos)(1:end-1,2);
        firstpos_aux=posth.(num_pos)(1:1);
        posth.(num_pos)(1,2)=firstpos_aux;
        posth.(num_pos)(2:end,2)=aux_med;
        N_posth(cont)=length(posth.(num_pos));
        cont=cont+1;
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        clear time_aux
        %Obtener tiempo máximo utilizado para interpolar posición de la base
        timemax_th.(num_pos)=posth.(num_pos)(end,5);
        %Creación de vector auxiliar de tiempo
        idx_timemax_th.(num_pos)=min(find(time_final>=timemax_th.(num_pos)));	%índice de posicionamiento en vector de tiempo total
        time_aux=time_final(1,1:idx_timemax_th.(num_pos));
        %Interpolación vector de referencia y medición de ángulos del brazo
        refth.(num_pos)=posth.(num_pos)(:,1);
        refth_rad.(num_pos)=(refth.(num_pos)).*2*pi/epcm(i+1);
        medth.(num_pos)=posth.(num_pos)(:,2);
        medth_rad.(num_pos)=(medth.(num_pos)).*2*pi/epcm(i+1);
        time_refth.(num_pos)=posth.(num_pos)(:,5);
        inter_refth.(num_pos)=interp1(time_refth.(num_pos),refth_rad.(num_pos),time_aux,'pchip');
        inter_medth.(num_pos)=interp1(time_refth.(num_pos),medth_rad.(num_pos),time_aux,'pchip');
        %Guardo interpolación en vectores de referencia y medición
        ref_int(i+3,1:length(inter_refth.(num_pos)),j)=inter_refth.(num_pos);
        med_int(i+3,1:length(inter_medth.(num_pos)),j)=inter_medth.(num_pos);
        %rescato últimos valores del vector de referencia y medición, y relleno el vector ref_int y med_int con ese valor
%         ref_int(i+3,(length(inter_refth.(num_pos))+1):end,j)=refth_rad.(num_pos)(end);    % ref th
%         med_int(i+3,(length(inter_medth.(num_pos))+1):end,j)=medth_rad.(num_pos)(end);    % med th
        %med_int(i+3,(length(inter_medth.(num_pos))+1):end,j)=refth_rad.(num_pos)(end);    % med th
        ref_int(i+3,(length(inter_refth.(num_pos))+1):end,j)=ref_int(i+3,length(inter_refth.(num_pos)),j);
        med_int(i+3,(length(inter_medth.(num_pos))+1):end,j)=med_int(i+3,length(inter_medth.(num_pos)),j);
    end
end
save('cpcdata_test2.mat','ref_int','med_int');
disp('Datos CTC interpolados');






