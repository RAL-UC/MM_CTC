#ifndef ROBOT_CONTROL_H // header guards
#define ROBOT_CONTROL_H

#define ARMA_USE_LAPACK
#define ARMA_USE_BLAS
#include <armadillo>

using namespace arma;

class base_control{
public:
	void inicializa();
	mat matriz_inercia();
	mat matriz_coriolis();
	mat matriz_actuador();
	mat base_control::control_cpc(std::vector<double> qm,std::vector<double> qpm,std::vector<double> qppd,std::vector<double> qerr,std::vector<double> qperr,mat kp,mat kv);
	mat base_control::din_dir(std::vector<double> tau,std::vector<double> qpm);
};
#endif