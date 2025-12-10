#include "robot_control.h"

double M11,M12,M13,M21,M22,M23,M31,M32,M33;	//Matriz de Inercia
double C11,C12,C13,C21,C22,C23,C31,C32,C33;	//Matriz de Coriolis
double G1,G2,G3;							//Matriz de fuerza gravitatoria
double tau1,tau2,tau3;						//Vector de toques calculados
double d1,a2,a3,m1,m2,m3,a1_2,a2_2,a3_2,b1,b2,b3;	//Parámetros del robot
//double theta1,theta2,theta3,th1d,th2d,th3d;
double th1dd,th2dd,th3dd;		//Solo para pruebas
double S1,S2,S3,C1,C2,C3,S1_2,S2_2,S3_2,C1_2,C2_2,C3_2;
double g;

mat M(3,3);
mat C(3,3);
mat Cq(3,1);
mat G(3,1);
mat F(3,1);
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
	d1=0.06;
	a2=0.190;
	//a3=0.139;
	//a3=0.139+0.185+0.130; //Incluyendo el gripper
	a3=0.327;

	//En kg
	m1=2.867;
	m2=0.633;	
	////m3=0.79;
	m3=1.3;

	//
	g=9.8;
	b1=0.42;
	b2=0.42;
	b3=0.42;

}

mat control::matriz_inercia(double th1,double th2,double th3){

	M11 = -pow(cos(th2), 0.2e1) * a3 * a3 * m3 / 0.3e1 - pow(cos(th3), 0.2e1) * a3 * a3 * m3 / 0.3e1 + a2 * a2 * m3 * pow(cos(th2), 0.2e1) - 0.2e1 / 0.3e1 * sin(th2) * sin(th3) * a3 * a3 * m3 * cos(th2) * cos(th3) - sin(th2) * sin(th3) * a3 * m3 * cos(th2) * a2 + cos(th3) * a3 * m3 * pow(cos(th2), 0.2e1) * a2 + 0.2e1 / 0.3e1 * a3 * a3 * m3 * pow(cos(th2), 0.2e1) * pow(cos(th3), 0.2e1) + m3 * a3 * a3 / 0.3e1 + a2 * a2 * m2 * pow(cos(th2), 0.2e1) / 0.3e1;
	M12 = 0;
	M13 = 0;
	M21 = 0;
	M22 = a2 * a2 * m3 + a3 * m3 * cos(th3) * a2 + m3 * a3 * a3 / 0.3e1 + m2 * a2 * a2 / 0.3e1;
	M23 = a3 * m3 * cos(th3) * a2 / 0.2e1 + m3 * a3 * a3 / 0.3e1;
	M31 = 0;
	M32 = a3 * m3 * cos(th3) * a2 / 0.2e1 + m3 * a3 * a3 / 0.3e1;
	M33 = m3 * a3 * a3 / 0.3e1;

	M << M11 << M12 << M13 << endr
	  << M21 << M22 << M23 << endr
	  << M31 << M32 << M33 << endr;

	return M;
}

mat control::matriz_coriolis(double th1,double th2,double th3,double th1d,double th2d,double th3d){

	Cq[0]=-0.2e1 * sin(th3) * a2 * m3 * pow(cos(th2), 0.2e1) * a3 * th1d * th2d - sin(th3) * a2 * m3 * pow(cos(th2), 0.2e1) * a3 * th1d * th3d + 0.2e1 / 0.3e1 * cos(th3) * a3 * a3 * m3 * sin(th3) * th1d * th2d + 0.2e1 / 0.3e1 * cos(th3) * a3 * a3 * m3 * sin(th3) * th1d * th3d - 0.2e1 * cos(th2) * a2 * a2 * m3 * sin(th2) * th1d * th2d + 0.2e1 / 0.3e1 * cos(th2) * a3 * a3 * m3 * sin(th2) * th1d * th2d + 0.2e1 / 0.3e1 * cos(th2) * a3 * a3 * m3 * sin(th2) * th1d * th3d + a2 * m3 * sin(th3) * a3 * th1d * th2d - 0.2e1 / 0.3e1 * cos(th2) * sin(th2) * th1d * m2 * a2 * a2 * th2d - cos(th2) * a2 * m3 * sin(th2) * a3 * th1d * th3d * cos(th3) - 0.2e1 * cos(th2) * cos(th3) * a3 * m3 * sin(th2) * th1d * th2d * a2 - 0.4e1 / 0.3e1 * cos(th2) * a3 * a3 * m3 * sin(th2) * pow(cos(th3), 0.2e1) * th1d * th2d - 0.4e1 / 0.3e1 * cos(th2) * a3 * a3 * m3 * sin(th2) * pow(cos(th3), 0.2e1) * th1d * th3d - 0.4e1 / 0.3e1 * cos(th3) * a3 * a3 * m3 * pow(cos(th2), 0.2e1) * sin(th3) * th1d * th2d - 0.4e1 / 0.3e1 * cos(th3) * a3 * a3 * m3 * pow(cos(th2), 0.2e1) * sin(th3) * th1d * th3d;
	Cq[1]=a3 * m3 * pow(cos(th2), 0.2e1) * sin(th3) * a2 * th1d * th1d - sin(th3) * a2 * m3 * a3 * th2d * th3d + 0.2e1 / 0.3e1 * a3 * a3 * m3 * pow(cos(th2), 0.2e1) * cos(th3) * sin(th3) * th1d * th1d + 0.2e1 / 0.3e1 * a3 * a3 * m3 * cos(th2) * sin(th2) * pow(cos(th3), 0.2e1) * th1d * th1d - a3 * a3 * m3 * cos(th2) * sin(th2) * th1d * th1d / 0.3e1 - a3 * a3 * m3 * cos(th3) * sin(th3) * th1d * th1d / 0.3e1 + a2 * a2 * m3 * cos(th2) * sin(th2) * th1d * th1d - sin(th3) * a2 * m3 * a3 * th3d * th3d / 0.2e1 - a3 * m3 * sin(th3) * a2 * th1d * th1d / 0.2e1 + sin(th2) * th1d * th1d * m2 * a2 * a2 * cos(th2) / 0.3e1 + a3 * m3 * cos(th2) * sin(th2) * cos(th3) * a2 * th1d * th1d;
	Cq[2]=0.2e1 / 0.3e1 * a3 * a3 * m3 * pow(cos(th2), 0.2e1) * cos(th3) * sin(th3) * th1d * th1d + 0.2e1 / 0.3e1 * a3 * a3 * m3 * cos(th2) * sin(th2) * pow(cos(th3), 0.2e1) * th1d * th1d - a3 * a3 * m3 * cos(th2) * sin(th2) * th1d * th1d / 0.3e1 - a3 * a3 * m3 * cos(th3) * sin(th3) * th1d * th1d / 0.3e1 + a3 * m3 * pow(cos(th2), 0.2e1) * sin(th3) * a2 * th1d * th1d / 0.2e1 + a3 * m3 * sin(th3) * a2 * th2d * th2d / 0.2e1 + a3 * m3 * cos(th2) * sin(th2) * cos(th3) * a2 * th1d * th1d / 0.2e1;

	return Cq;
}

mat control::matriz_gravedad(double th1,double th2,double th3){
	
	G[0] = 0;
	G[1] = a2 * m2 * cos(th2) * g / 0.2e1 + a2 * m3 * cos(th2) * g - a3 * m3 * sin(th2) * sin(th3) * g / 0.2e1 + a3 * m3 * cos(th2) * cos(th3) * g / 0.2e1;
	G[2] = -a3 * m3 * sin(th2) * sin(th3) * g / 0.2e1 + a3 * m3 * cos(th2) * cos(th3) * g / 0.2e1;

	return G;
}

mat control::matriz_roce(double th1d,double th2d,double th3d){

	F[0] = th1d*b1;
	F[1] = th2d*b2;
	F[2] = th3d*b3;

	return F;
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
	control::matriz_coriolis(theta1,theta2,theta3,th1d,th2d,th3d);
	control::matriz_gravedad(theta1,theta2,theta3);
	control::matriz_roce(th1d,th2d,th3d);

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

	tau_cpc=M*(qpp_ref+kv*qp_error+kp*q_error)+Cq+G+F;

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

	tau_num=tau_control-Cq-G-F;
	//tau_num=trans(tau_num);
	M=inv(M);
	qpp=M*tau_num;

	return qpp;
}
