#include "robot_control.h"

double M11,M12,M13,M21,M22,M23,M31,M32,M33;	//Matriz de Inercia
double C11,C12,C13,C21,C22,C23,C31,C32,C33;	//Matriz de Coriolis
double G1,G2,G3;							//Matriz de fuerza gravitatoria
double tau1,tau2,tau3;						//Vector de toques calculados
double a1,a2,a3,m1,m2,m3,a1_2,a2_2,a3_2,b1,b2,b3;	//Parámetros del robot
//double theta1,theta2,theta3,th1d,th2d,th3d;
double th1dd,th2dd,th3dd;		//Solo para pruebas
double S1,S2,S3,C1,C2,C3,S1_2,S2_2,S3_2,C1_2,C2_2,C3_2;
double g;

mat M(3,3);
mat C(3,3);
mat G(3,1);
mat qp(3,1);
mat qpp(3,1);
mat tau_control(3,1);
mat tau_cpc(3,1);
mat tau_num(3,1);
mat qp_ac(3,1);
mat qp_error(3,1);
mat q_error(3,1);
mat q_error_an(3,1);
mat qpp_ref(3,1);
mat pid(3,1);

std::vector<double> acel_ref;

void control::inicializa(){

	//en mts
	a1=0.06;
	a2=0.190;
	a3=0.139;

	//En kg
	m1=2.867;
	m2=0.633;	
	m3=0.79;

	//
	g=9.8;
	b1=0.12;
	b2=0.12;
	b3=0.12;

	a1_2=a1*a1;
	a2_2=a2*a2;
	a3_2=a3*a3;

}

mat control::matriz_inercia(double theta1,double theta2,double theta3){
	
	C1=cos(theta1);
	C2=cos(theta2);
	C3=cos(theta3);
	S1=sin(theta1);
	S2=sin(theta2);
	S3=sin(theta3);

	S1_2=(sin(theta1))*(sin(theta1));
	S2_2=(sin(theta2))*(sin(theta2));
	S3_2=(sin(theta3))*(sin(theta3));
	C1_2=(cos(theta1))*(cos(theta1));
	C2_2=(cos(theta2))*(cos(theta2));
	C3_2=(cos(theta3))*(cos(theta3));

	M11=m1*a1_2+0.33333333*a2_2*m2*C2_2-S2*S3*a3*m3*a1+C3*a3*m3*a2*C2_2+C2*C3*a3*m3*a1+C2*a2*m2*a1+0.66666667*a3_2*m3*C3_2*C2_2+0.33333333*m3*a3_2-0.66666667*S2*S3*a3_2*m3*C3*C2-S2*S3*a3*m3*a2*C2+a1_2*m2+a1_2*m3+2*a1*m3*a2*C2-0.33333333*a3_2*m3*C2_2-0.33333333*a3_2*m3*C3_2+a2_2*m3*C2_2;
	M12=0;
	M13=0;
	M21=0;
	M22=0.33333333*m2*a2_2+a3*m3*C3*a2+0.33333333*m3*a3_2+a2_2*m3;
	M23=0.33333333*m3*a3_2+0.5*a3*m3*C3*a2;
	M31=0;
	M32=0.33333333*m3*a3_2+0.5*a3*m3*C3*a2;
	M33=0.33333333*m3*a3_2;

	M << M11 << M12 << M13 << endr
	  << M21 << M22 << M23 << endr
	  << M31 << M32 << M33 << endr;

	return M;
}

mat control::matriz_coriolis(double th1d,double th2d,double th3d){

	C11=b1-0.66666667*C2*S2*m2*a2_2*th2d-a1*m2*th2d*a2*S2-2*a1*m3*th2d*a2*S2-1.33333333*C2*a3_2*m3*C3_2*S2*th2d-1.33333333*C2*a3_2*m3*C3_2*S2*th3d-1.33333333*C3*a3_2*m3*S3*C2_2*th2d-1.33333333*C3*a3_2*m3*S3*C2_2*th3d-2*C2*C3*a3*m3*S2*th2d*a2-a1*m3*a3*C3*th2d*S2-a1*m3*a3*C3*th3d*S2+0.66666667*S3*a3_2*m3*C3*th3d+S3*a3*m3*th2d*a2+0.66666667*S2*a3_2*m3*C2*th2d+0.66666667*S2*a3_2*m3*C2*th3d-2*C2*a2_2*m3*S2*th2d+0.66666667*C3*a3_2*m3*S3*th2d-a1*m3*a3*S3*th2d*C2-a1*m3*a3*S3*th3d*C2-S3*a2*m3*a3*C2_2*th3d-2*S3*a3*m3*th2d*a2*C2_2-C2*a2*m3*a3*S2*th3d*C3;
	C12=-0.66666667*C2*S2*th1d*m2*a2_2-a1*m2*th1d*a2*S2-2*a1*m3*th1d*a2*S2-1.33333333*C2*a3_2*m3*C3_2*S2*th1d-1.33333333*C3*a3_2*m3*S3*C2_2*th1d-2*C2*C3*a3*m3*S2*th1d*a2-a1*m3*a3*C3*th1d*S2+S3*a3*m3*th1d*a2+0.66666667*S2*a3_2*m3*C2*th1d-2*C2*a2_2*m3*S2*th1d+0.66666667*C3*a3_2*m3*S3*th1d-a1*m3*a3*S3*th1d*C2-2*S3*a3*m3*th1d*a2*C2_2;
	C13=-1.33333333*C2*a3_2*m3*C3_2*S2*th1d-1.33333333*C3*a3_2*m3*S3*C2_2*th1d-a1*m3*a3*C3*th1d*S2+0.66666667*C3*a3_2*m3*S3*th1d+0.66666667*S2*a3_2*m3*C2*th1d-a1*m3*a3*S3*th1d*C2-S3*a3*m3*th1d*a2*C2_2-C2*C3*a3*m3*S2*th1d*a2;
	C21=0;
	C22=-S3*a2*m3*a3*th3d+b2;
	C23=-S3*a3*m3*th2d*a2;
	C31=0;
	C32=0;
	C33=b3;

	C << C11 << C12 << C13 << endr
	  << C21 << C22 << C23 << endr
	  << C31 << C32 << C33 << endr;

	return C;
}

mat control::matriz_gravedad(){
	G1=0;
	G2=-0.5*a3*m3*S3*S2+0.5*a3*m3*C3*C2+0.5*a2*m2*C2+a2*m3*C2;
	G3=-0.5*a3*m3*S3*S2+0.5*a3*m3*C3*C2;

	G << G1 << endr
	  << G2 << endr
	  << G3 << endr;

	return G;
}


mat control::control_cpc(std::vector<int> th,std::vector<int> thd,std::vector<double> acel_ref,std::vector<int> th_error,std::vector<double> thd_error,mat kp,mat kv){

	//Valores de theta medidos
	double theta1=th.at(0);
	double theta2=th.at(1);
	double theta3=th.at(2);

	//Valores de velocidad medidos
	double th1d=thd.at(0);
	double th2d=thd.at(1);
	double th3d=thd.at(2);

	//Aceleración de referencia
	double acel1_ref=acel_ref.at(0);
	double acel2_ref=acel_ref.at(1);
	double acel3_ref=acel_ref.at(2);

	//Error en posición
	double th1_error=th_error.at(0);
	double th2_error=th_error.at(1);
	double th3_error=th_error.at(2);

	//Error en velocidad
	double thd1_error=thd_error.at(0);
	double thd2_error=thd_error.at(1);
	double thd3_error=thd_error.at(2);

	control::matriz_inercia(theta1,theta2,theta3);
	control::matriz_coriolis(th1d,th2d,th3d);
	control::matriz_gravedad();

	//Vector columna de velocidad actual
	qp_ac << th1d << endr
	 	  << th2d << endr
		  << th3d << endr;

	//Vector columna de error en posición
	q_error << th1_error << endr
			<< th2_error << endr
			<< th3_error << endr;

	//Vector columna de error en velocidad
	qp_error << thd1_error << endr
	 		 << thd2_error << endr
			 << thd3_error << endr;

	//Vector columna de aceleración de referencia
	qpp_ref << acel1_ref << endr
			<< acel2_ref << endr
			<< acel3_ref << endr;

	tau_cpc=M*(qpp_ref+kv*qp_error+kp*q_error)+C*qp_ac+G;

	return tau_cpc;
}

mat control::control_pid(std::vector<int> th_error,std::vector<int> th_error_an,std::vector<double> thd_error,mat kp,mat kv,mat ki,double dt){

	//Error de posición actual
	double th1_error=th_error.at(0);
	double th2_error=th_error.at(1);
	double th3_error=th_error.at(2);

	//Error de posición anterior
	double th1_error_an=th_error_an.at(0);
	double th2_error_an=th_error_an.at(1);
	double th3_error_an=th_error_an.at(2);

	//Error en velocidad
	double thd1_error=thd_error.at(0);
	double thd2_error=thd_error.at(1);
	double thd3_error=thd_error.at(2);

	//Vector columna de error de posición actual
	q_error << th1_error << endr
			<< th2_error << endr
			<< th3_error << endr;

	//Vector columna de error de posición actual
	q_error_an	<< th1_error_an << endr
				<< th2_error_an << endr
				<< th3_error_an << endr;

	//Vector columna de error en velocidad
	qp_error << thd1_error << endr
	 		 << thd2_error << endr
			 << thd3_error << endr;

	pid=kp*q_error+kv*qp_error+ki*(q_error_an*dt);

	return pid;
}

mat control::din_dir(std::vector<double> tau,std::vector<int> thd){

	//Valores de velocidad medidos
	double th1d=thd.at(0);
	double th2d=thd.at(1);
	double th3d=thd.at(2);

	double tau1=tau.at(0);
	double tau2=tau.at(1);
	double tau3=tau.at(2);

	qp << th1d << endr
	   << th2d << endr
	   << th3d << endr;

	tau_control << tau1 << endr
	            << tau2 << endr
	            << tau3 << endr;

	tau_num=tau_control-C*qp-G;
	tau_num=trans(tau_num);
	M=inv(M);
	qpp=tau_num*M;
	return qpp;
}
