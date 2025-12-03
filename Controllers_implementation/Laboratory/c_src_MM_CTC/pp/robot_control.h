#ifndef ROBOT_CONTROL_H // header guards
#define ROBOT_CONTROL_H

#define ARMA_USE_LAPACK
#define ARMA_USE_BLAS
#include <armadillo>

using namespace arma;

class control{
public:
	void inicializa();
	mat matriz_inercia(double theta1,double theta2,double theta3);
	mat matriz_coriolis(double th1d,double th2d,double th3d);
	mat matriz_gravedad();
	mat control_cpc(std::vector<int> th,std::vector<int> thd,std::vector<double> acel_ref,std::vector<int> th_error,std::vector<double> thd_error,mat kp,mat kv);
	mat control_pid(std::vector<int> th_error,std::vector<int> th_error_an,std::vector<double> thd_error,mat kp,mat kv,mat ki,double dt);
	mat din_dir(std::vector<double> tau,std::vector<int> thd);
};
#endif