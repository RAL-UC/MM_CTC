#include "base_control.h"

//Ecuación de la dinámica inversa = M*qpp+C*qp=B*tau
double MM11,MM12,MM21,MM22;	//Matriz de Inercia
double CC11,CC12,CC21,CC22;	//Matriz de Coriolis
double BB11,BB12,BB21,BB22; //Matriz que multiplica al torque

double tau1_base,tau2_base;			//Vector de toques calculados
double largo,ancho,Rr,Rl,mb,Jb,ul,ua;
//double th1dd,th2dd,th3dd;		//Solo para pruebas

mat MM(2,2);
mat CC(2,2);
mat BB(2,2);
mat qpm(2,1);
mat qpp_base(2,1);
mat tau_control_base(2,1);
mat tau_cpc_base(2,1);
mat tau_num_base(2,1);
mat qp_ac_base(2,1);
mat qp_error_base(2,1);
mat q_error_base(2,1);
//mat q_error_an_base(2,1);
mat qpp_ref_base(2,1);

mat qm(2,1);
mat qppd(2,1);
mat qerr(2,1);
mat qperr(2,1);
mat qp_base(2,1);


mat Binv(2,2);
mat Minv(2,2);

//std::vector<double> acel_ref;

void base_control::inicializa(){

	//en mts
	largo=0.508;
	ancho=0.497;
	Rr=0.222/2;	//Radio rueda derecha
	Rl=0.222/2;	//Radio rueda izquierda

	//En kg
	//mb=21;	//Incluye masa de la base + masa del brazo + masa del notebook
	mb=12;		//Para pruebas con el simulador

	//Inercia
	//Jb=(1/12)*mb*((largo*largo)+(ancho*ancho));
	Jb=0.0833*mb*((largo*largo)+(ancho*ancho));

	//Coeficientes de roce, calculados con pruebas del laboratorio, incluyen masa del brazo y del notebook
	ul=0.019;
	ua=0.076;

}

mat base_control::matriz_inercia(){
	
	MM11=Jb;
	MM12=0;
	MM21=0;
	MM22=mb;

	MM << MM11 << MM12 << endr
	   << MM21 << MM22 << endr;

	return MM;
}

mat base_control::matriz_coriolis(){

	CC11=ua;
	CC12=0;
	CC21=0;
	CC22=ul;
	
	CC << CC11 << CC12 << endr
	   << CC21 << CC22 << endr;

	return CC;
}

mat base_control::matriz_actuador(){

	BB11=ancho/(2*Rr);
	BB12=-ancho/(2*Rl);
	BB21=1/(2*Rr);
	BB22=1/(2*Rl);
	
	BB << BB11 << BB12 << endr
	   << BB21 << BB22 << endr;

	return BB;
}


mat base_control::control_cpc(std::vector<double> qm,std::vector<double> qpm,std::vector<double> qppd,std::vector<double> qerr,std::vector<double> qperr,mat kp_base,mat kv_base){

	//Valores de theta medidos
	double pos=qm.at(0);
	double ang=qm.at(1);

	//Valores de velocidad medidos
	double v=qpm.at(0);
	double w=qpm.at(1);

	//Aceleración de referencia
	double al=qppd.at(0);
	double aw=qppd.at(1);

	//Error en posición
	double err_pos=qerr.at(0);
	double err_ang=qerr.at(1);

	//Error en velocidad
	double err_v=qperr.at(0);
	double err_w=qperr.at(1);

	base_control::matriz_inercia();
	base_control::matriz_coriolis();
	base_control::matriz_actuador();

	//Vector columna de error en posición lineal y angular
	q_error_base << err_pos << endr
			     << err_ang << endr;

	//Vector columna de velocidad medida
	qp_ac_base << v << endr
	 	       << w << endr;

	//Vector columna de error en velocidad
	qp_error_base << err_v << endr
	 		      << err_w << endr;

	//Vector columna de aceleración de referencia
	qpp_ref_base << al << endr
			     << aw << endr;

	Binv=inv(BB);

	mat prueba1(2,1);
	prueba1=qpp_ref_base+kv_base*qp_error_base+kp_base*q_error_base;
	//prueba1=trans(prueba1);

	mat prueba2(2,1);
	prueba2=Binv*(MM*prueba1+CC*qp_ac_base);

	tau_cpc_base=prueba2;

	//tau_cpc_base=(MM*(qpp_ref_base+kv_base*qp_error_base+kp_base*q_error_base)+CC*qp_ac_base)*Binv;

	return tau_cpc_base;
}

mat base_control::din_dir(std::vector<double> tau_base,std::vector<double> qpm){

	//Valores de velocidad medidos
	double v=qpm.at(0);
	double w=qpm.at(1);

	//Valores de los torques deseados
	double tau1_base=tau_base.at(0);
	double tau2_base=tau_base.at(1);

	qp_base << v << endr
	        << w << endr;

	tau_control_base << tau1_base << endr
	                 << tau2_base << endr;

	tau_num_base=BB*tau_control_base-CC*qp_base;
	//tau_num_base=trans(tau_num_base);
	//MM.print("Matriz M:");
	Minv=inv(MM);
	//qpp_base=tau_num_base*Minv;
	qpp_base=Minv*tau_num_base;
	return qpp_base;
}
