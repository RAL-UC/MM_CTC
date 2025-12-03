#include "base_control.h"

//Ecuación de la dinámica inversa = M*qpp+C*qp=B*tau
double M11,M12,M21,M22;	//Matriz de Inercia
double C11,C12,C21,C22;	//Matriz de Coriolis
double B11,B12,B21,B22; //Matriz que multiplica al torque

double tau1,tau2;			//Vector de toques calculados
double largo,ancho,Rr,Rl,mb,Jb,ul,ua;
//double th1dd,th2dd,th3dd;		//Solo para pruebas

mat M(2,2);
mat C(2,2);
mat B(2,2);
mat qpm(2,1);
mat qpp(2,1);
mat tau_control(2,1);
mat tau_cpc(2,1);
mat tau_num(2,1);
mat qp_ac(2,1);
mat qp_error(2,1);
mat q_error(2,1);
mat q_error_an(2,1);
mat qpp_ref(2,1);

mat qm(2,1);
mat qppd(2,1);
mat qerr(2,1);
mat qperr(2,1);
mat qp(2,1);


mat Binv(2,2);
mat Minv(2,2);

std::vector<double> acel_ref;

void base_control::inicializa(){

	//en mts
	largo=0.508;
	ancho=0.497;
	Rr=0.222;	//Radio rueda derecha
	Rl=0.222;	//Radio rueda izquierda

	//En kg
	//mb=21;	//Incluye masa de la base + masa del brazo + masa del notebook
	mb=12;		//Para pruebas con el simulador

	//Inercia
	Jb=(1/12)*mb*((largo*largo)+(ancho*ancho));

	//Coeficientes de roce, calculados con pruebas del laboratorio, incluyen masa del brazo y del notebook
	ul=0.019;
	ua=0.076;

}

mat base_control::matriz_inercia(){
	
	M11=Jb;
	M12=0;
	M21=0;
	M22=mb;

	M << M11 << M12 << endr
	  << M21 << M22 << endr;

	return M;
}

mat base_control::matriz_coriolis(){

	C11=ua;
	C12=0;
	C21=0;
	C22=ul;
	
	C << C11 << C12 << endr
	  << C21 << C22 << endr;

	return C;
}

mat base_control::matriz_actuador(){

	B11=ancho/(2*Rr);
	B12=-ancho/(2*Rl);
	B21=1/(2*Rr);
	B22=1/(2*Rl);
	
	B << B11 << B12 << endr
	  << B21 << B22 << endr;

	return B;
}


mat base_control::control_cpc(std::vector<double> qm,std::vector<double> qpm,std::vector<double> qppd,std::vector<double> qerr,std::vector<double> qperr,mat kp,mat kv){

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
	q_error << err_pos << endr
			<< err_ang << endr;

	//Vector columna de velocidad medida
	qp_ac << v << endr
	 	  << w << endr;

	//Vector columna de error en velocidad
	qp_error << err_v << endr
	 		 << err_w << endr;

	//Vector columna de aceleración de referencia
	qpp_ref << al << endr
			<< aw << endr;

	Binv=inv(B);
	tau_cpc=(M*(qpp_ref+kv*qp_error+kp*q_error)+C*qp_ac)*Binv;

	return tau_cpc;
}

mat base_control::din_dir(std::vector<double> tau,std::vector<double> qpm){

	//Valores de velocidad medidos
	double v=qpm.at(0);
	double w=qpm.at(1);

	//Valores de los torques deseados
	double tau1=tau.at(0);
	double tau2=tau.at(1);
	double tau3=tau.at(2);

	qp << v << endr
	   << w << endr;

	tau_control << tau1 << endr
	            << tau2 << endr;

	tau_num=tau_control-C*qp;
	tau_num=trans(tau_num);
	Minv=inv(M);
	qpp=tau_num*Minv;
	return qpp;
}
