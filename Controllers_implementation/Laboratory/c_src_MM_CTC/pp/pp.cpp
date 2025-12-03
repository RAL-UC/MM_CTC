//Librerías para uso de la cámara
#include "cv.h"
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <time.h>

//DEBE IR EN ESTE ORDEN PARA QUE NO EXISTA ERRORES DE DUPLICACIÓN, 1° BASE Y 2° BRAZO
//Librerías para uso de la base
#include "Aria.h"
#include "math.h"

//Librerías para uso del brazo
#include "kniBase.h"
#include <MathHelperFunctions.h>
#include <pthread.h>

//Librería para cálculo del control
#include "robot_control.h"		
#include "base_control.h"	

//Librería para uso de Matrices
#define ARMA_USE_LAPACK
#define ARMA_USE_BLAS
#include <armadillo>

//Namespaces
using namespace arma;
using namespace cv;
using namespace KNI_MHF;
using namespace std;

//#define PI 3.14159265

cv::Mat world;
cv::Mat depthMap;
cv::Mat show;
cv::Mat bgrImage;
cv::Point coordenadas(0,0);
float dist,x,y,z;

//Thread structs:
pthread_mutex_t mutex;

struct TPoint {
	double X, Y, Z;
	double phi, theta, psi;
};

struct TCurrentMot {
	int idx;
	bool running;
	bool dir;
};

struct Tpos{
	static std::vector<int> x,y,z,u,v,w;
	static const int xArr[], yArr[], zArr[], uArr[], vArr[], wArr[];
};
//Katana obj.
std::auto_ptr<CLMBase> katana;
//std::vector<int> Foo::vec(array, array + sizeof(array)/sizeof(*array));
//positionen, hard-coded. Use values from file instead
const int Tpos::xArr[] = {30206, -23393, -3066, 14454, 30000, 30000};
const int Tpos::yArr[] = {24327, -7837, -16796, 5803, 30290, 31000};
const int Tpos::zArr[] = {24327, -7837, -16796, 5802, 30290, 10924};
const int Tpos::uArr[] = {5333, -13791, -9985, 11449, 30996, 12063};
const int Tpos::vArr[] = {-3799, -5703, -11676, 8210, 30995, 12063};
const int Tpos::wArr[] = {-3799, -5703, -11676, 8210, 30995, 30992};
std::vector<int> Tpos::x(xArr, xArr + sizeof(xArr)/sizeof(*xArr));
std::vector<int> Tpos::y(yArr, yArr + sizeof(yArr)/sizeof(*yArr));
std::vector<int> Tpos::z(zArr, zArr + sizeof(zArr)/sizeof(*zArr));
std::vector<int> Tpos::u(uArr, uArr + sizeof(uArr)/sizeof(*uArr));
std::vector<int> Tpos::v(vArr, vArr + sizeof(vArr)/sizeof(*vArr));
std::vector<int> Tpos::w(wArr, wArr + sizeof(wArr)/sizeof(*wArr));
std::vector<TPoint> points(0);
void StartPointlistMovement();
void StartProgram(int index);
pthread_t tid;
void* RunProgram(void*);
pid_t threadPid;
int retVal = 0;
bool progRunning = false;
const double PI = 3.14159265358979323846;

void StartProgram(int index){
	//Q&D test to launch program:
	//std::system("/home/katprog katana6M180.cfg 1");
	progRunning = true;
	pthread_create(&tid, NULL, RunProgram, (void*)&retVal);//(&tid, NULL, start_func, arg);
	pthread_detach(tid);
}
//////////////////////////////////////////////////////////////////////////////////
void* RunProgram(void*){
	//katana->calibrate();
	std::cout << "\nProgram running...type any key to stop after the next cycle\n";
	while(progRunning){
		if(progRunning) katana->moveRobotToEnc(Tpos::x, true);
		if(progRunning) katana->moveRobotToEnc(Tpos::y, true);
		if(progRunning) katana->moveRobotToEnc(Tpos::z, true);
		if(progRunning) katana->moveRobotToEnc(Tpos::u, true);
		if(progRunning) katana->moveRobotToEnc(Tpos::v, true);
		if(progRunning) katana->moveRobotToEnc(Tpos::w, true);
	}
	pthread_exit((void*) &retVal);
	return ((void*) &retVal);
}
//*****************************************
//FIN RUTINAS PARA INICIALIZACIÓN DEL BRAZO
//*****************************************

//****************************************
//RUTINAS PARA INICIALIZACIÓN DE LA BASE
//****************************************
class ConnHandler
{
public:
	// Constructor
	ConnHandler(ArRobot *robot);
	// Destructor, its just empty
	~ConnHandler(void) {}
	// to be called if the connection was made
	void connected(void);
	// to call if the connection failed
	void connFail(void);
	// to be called if the connection was lost
	void disconnected(void);
protected:
	// robot pointer
	ArRobot *myRobot;
	// the functor callbacks
	ArFunctorC<ConnHandler> myConnectedCB;
	ArFunctorC<ConnHandler> myConnFailCB;
	ArFunctorC<ConnHandler> myDisconnectedCB;
};

ConnHandler::ConnHandler(ArRobot *robot) :
myConnectedCB(this, &ConnHandler::connected),  
myConnFailCB(this, &ConnHandler::connFail),
myDisconnectedCB(this, &ConnHandler::disconnected)

{
	myRobot = robot;
	myRobot->addConnectCB(&myConnectedCB, ArListPos::FIRST);
	myRobot->addFailedConnectCB(&myConnFailCB, ArListPos::FIRST);
	myRobot->addDisconnectNormallyCB(&myDisconnectedCB, ArListPos::FIRST);
	myRobot->addDisconnectOnErrorCB(&myDisconnectedCB, ArListPos::FIRST);
}

// just exit if the connection failed
void ConnHandler::connFail(void)
{
	printf("directMotionDemo connection handler: Failed to connect.\n");
	myRobot->stopRunning();
	Aria::exit(1);
	return;
}

// turn on motors, and off sonar, and off amigobot sounds, when connected
void ConnHandler::connected(void)
{
	printf("directMotionDemo connection handler: Connected\n");
	myRobot->comInt(ArCommands::SONAR, 0);
	myRobot->comInt(ArCommands::ENABLE, 1);
	//myRobot->comInt(ArCommands::SOUNDTOG, 0);
}

// lost connection, so just exit
void ConnHandler::disconnected(void)
{
	printf("directMotionDemo connection handler: Lost connection, exiting program.\n");
	Aria::exit(0);
}
//****************************************************
//FIN RUTINAS DE INICIALIZACIÓN DE LA BASE
//****************************************************

float profundidad(int x, int y)
{
	float x2,y2,z2;
	coordenadas=Point(x,y);
	Vec3f s = world.at<Vec3f>(coordenadas.y, coordenadas.x);
	x2 = s[0];
	y2 = s[1];
	z2 = s[2];		//Profundidad, OJO que cuando el objeto está muy cerca la cámara no puede ver el objeto, min=0,60 mts. aprox.
	return(z2);
}

IplImage* captura_rojo(IplImage* img)
{
	IplImage* imgHSV = cvCreateImage(cvSize(img->width,img->height), 8, 3);
	IplImage* imgThreshed = cvCreateImage(cvSize(img->width,img->height), 8, 1);

	cvCvtColor(img, imgHSV, CV_BGR2HSV);
	cvInRangeS(imgHSV, cvScalar(0, 50, 50), cvScalar(15, 255, 255), imgThreshed);	//HSV ROJO

	cvReleaseImage(&imgHSV);
	return imgThreshed;
}

IplImage* captura_azul(IplImage* img)
{
	IplImage* imgHSV = cvCreateImage(cvSize(img->width,img->height), 8, 3);
	IplImage* imgThreshed = cvCreateImage(cvSize(img->width,img->height), 8, 1);

	cvCvtColor(img, imgHSV, CV_BGR2HSV);
	//cvInRangeS(imgHSV, cvScalar(0, 50, 50), cvScalar(15, 255, 255), imgThreshed);	//HSV ROJO
	cvInRangeS(imgHSV, cvScalar(110,150,100), cvScalar(120,255,150), imgThreshed);	//HSV AZUL

	cvReleaseImage(&imgHSV);
	return imgThreshed;
}

void onMouse( int event, int x, int y, int flags, void* )
{
	if( event == CV_EVENT_LBUTTONUP){
		coordenadas=Point(x,y);
		Vec3f s = world.at<Vec3f>(coordenadas.y, coordenadas.x);
		float xp = s[0];
		float yp = s[1];
		float zp = s[2];
		printf("x=%f y=%f z=%f\n",xp,yp,zp);
	}
}
//***************************************************
//FIN RUTINAS PARA CAPTURA DEL COLOR
//**************************************************

//***********************************
//INICIO RUTINAS PARA OBTENER TIEMPO
//**********************************
void delay(double secs)
{
	clock_t t0, t1;

	t0=clock();
	t1=t0;
	while ((double) (t1-t0) / (double) CLOCKS_PER_SEC < secs)
	{
		t1=clock();
	}
} // delay()

// Returns current value of the high resolution performance counter
double timer_hr_count(void)
{
	LARGE_INTEGER lpPerformanceCount;

	QueryPerformanceCounter(&lpPerformanceCount);

	return (double) lpPerformanceCount.QuadPart;
}

// Returns high resolution performance counter frequency in counts per second
double timer_hr_freq(void)
{
	LARGE_INTEGER lpFrequency;

	QueryPerformanceFrequency(&lpFrequency);

	return (double) lpFrequency.QuadPart;
}

// Returns elapsed time in seconds.  The input must be the initial count value
// of the high resolution performance counter as returned by function timer_hr_count().
double timer_hr_elapsed(double initial_count)
{
	LARGE_INTEGER lpPerformanceCount;

	QueryPerformanceCounter(&lpPerformanceCount);

	return ((double) lpPerformanceCount.QuadPart - initial_count)/timer_hr_freq();
}
//***************************************
//FIN RUTINAS DE OBTENCIÓN DE TIEMPO
//***************************************
double t_angulo(double tth){
	//Se restringe el ángulo solo entre +180° a -180°
	if (tth >= 360)
		tth = tth - 360.0 * (tth / 360);

	if (tth < -360)
		tth = tth + 360.0 * (tth / -360);

	if (tth <= -180)
		tth = + 180.0 + (tth + 180.0);

	if (tth > 180)
		tth = - 180.0 + (tth - 180.0);

	return(tth);
}

int integral_simpson(double dt,int f_ant,int f_act){
	int integral=(dt/6)*(f_ant+4*((f_ant+f_act)/2)+f_act);
	return integral;
}


int main(int argc, char* argv[]){

	//Variables para el par calculado
	double kp1_base,kp2_base,kv1_base,kv2_base;
	double v_ref_ant,vth_ref_ant,acel_ref_ac,acelth_ref_ac;

	double ab_lin_control_ac,vb_lin_control_ac,ab_lin_control_ant,pb_lin_control_ac,vb_lin_control_ant;
	double ab_th_control_ac,vb_th_control_ac,ab_th_control_ant,pb_th_control_ac,vb_th_control_ant;

	std::vector<double> qm(2, 0); //vector de velocidad actual en encoders/seg
	std::vector<double> qpm(2, 0); //vector de velocidad actual en encoders/seg
	std::vector<double> qppd(2, 0); //vector de velocidad actual en encoders/seg

	std::vector<double> qerr(2, 0); //vector de velocidad actual en encoders/seg
	std::vector<double> qperr(2, 0); //vector de velocidad actual en encoders/seg
	std::vector<double> taud(2, 0); //vector de velocidad actual en encoders/seg

	arma::mat qpp_base_control(2,1);
	arma::mat tau_cpc(2,1);
	arma::mat q_base_control(2,1);

	arma::mat kp_base(2,2);
	arma::mat kv_base(2,2);

	//Variables para la cámara
	float th_ref_cam,div,dist;
	float delta;
	double vel_lin;
	double vel;
	int sw5=1;
	float z=0;
	//Fin de variables de la cámara

	//VARIABLES DEL BRAZO
	double acel0_control_ac,acel0_control_an,vel0_control_an,vel0_control;
	double acel1_control_an,acel2_control_an,vel1_control_an,vel2_control_an;
	double acel1_control_ac,acel2_control_ac,vel1_control,vel2_control;
	int pos0_control,pos1_control,pos2_control;

	//double vel0_control_an,vel1_control_an,vel2_control_an;
	int pos0_control_an,pos1_control_an,pos2_control_an;

	int i,sw0,sw1,sw2;

	//Parámetros del control par calculado y PID
	double kp0,kp1,kp2,kv0,kv1,kv2,ki0,ki1,ki2;

	//Parámetros de posición, velocidad y aceleración
	//double x_ref,y_ref,z_ref,x_tar,y_tar,z_tar;	
	int th0_an,th1_an,th2_an,th0_ref,th1_ref,th2_ref,th0vec,th1vec,th2vec,th0,th1,th2,th0_tar,th1_tar,th2_tar;
	double vel0_ref,vel1_ref,vel2_ref,vel0_an,vel1_an,vel2_an,vel0_ac,vel1_ac,vel2_ac;
	double acel0_ref,acel1_ref,acel2_ref,acel0_control,acel1_control,acel2_control;

	int th0_ac,th1_ac,th2_ac;

	//Parámetros de tiempo
	double t_ac,t_an,dt,t0_interp,tac_interp;	//variables para captura de tiempo

	double th0_itae,th1_itae,th2_itae;

	//Variables para cálculo de error
	int th0_err,th1_err,th2_err,vel0_err,vel1_err,vel2_err;
	double rad0_err,rad1_err,rad2_err;

	int th0_med_an,th1_med_an,th2_med_an;

	//Parámetros de delta aceptados
	int d_th0,d_th1,d_th2;

	int err_pos0_control,err_pos1_control,err_pos2_control;

	std::vector<int> current_encoders;//encoders actuales
	std::vector<int> reference_encoders (6,0);//encoders actuales
	std::vector<int> target_encoders(6, 0); //encoders posición deseada 

	std::vector<double> acel_ref(3, 0); //vector de errores en velocidad
	std::vector<int> error_pos(3, 0); //vector de errores en posición en encoders
	std::vector<int> error_pos_an(3, 0); //vector de errores en posición en encoders
	std::vector<double> error_vel(3, 0); //vector de errores en velocidad
	std::vector<int> current_angles(3, 0); //vector de encoders actuales
	std::vector<int> current_vel(3, 0); //vector de velocidad actual en encoders/seg

	arma::mat acel_control(3,1);
	arma::mat tau_control(3,1);
	arma::mat pos_control(3,1);

	arma::mat kp(3,3);
	arma::mat kv(3,3);
	arma::mat ki(3,3);

	std::vector<double> tau_control2(3, 0); //vector de errores en velocidad
	std::vector<double> acel_control2(3, 0); //vector de errores en velocidad

	int th0_err_final,th1_err_final,th2_err_final;
	int th0_ref_mov,th1_ref_mov,th2_ref_mov;

	double lam0,lam1,lam2;
	double tf0,tf1,tf2,tt0,tt1,tt2;
	//FIN VARIABLES DEL BRAZO

	//VARIABLES DE LA BASE
	double x_med_ac,y_med_ac,pos_med_ac,th_med_ac,vel_med_ac;
	double x_ref_ant,y_ref_ant,th_ref_ant,th_refxy_ac,th_medxy_ant,x_ref_ac,y_ref_ac,th_ref_ac,v_ref_ac;
	double x_med_ant,y_med_ant,th_med_ant,vel_med_ant;
	double err_x,err_y,err_th,err_vel,err_pos;
	double kp_th,kp_vel,kv_vel,kv_th,PD_vel,PD_th;
	double dif_an;
	double err_vel_th,vth_ref_ac,vth_med_ant,vth_med_ac;

	double pos_ref_ac,pos_ref_ant;
	double th_tar;
	double xbase_vec,ybase_vec,thbase_vec;
	double err_pos_final,err_th_final,err_x_final,err_y_final,posbase_vec;
	double max_vel_tra,max_vel_rot;
	double lamx_tra,lamy_tra,lam_rot,lampos_tra;
	int ii=1;
	float cth,sth;
	double itae_pos_base,itae_x_base,itae_y_base,itae_th_base;

	double x_tar,y_tar;
	double *vec_pos_ref;
	//Fin declaración de variables de la base

	//Otras variables de la base
	double max_vel_tra_x,max_vel_tra_y,lam_x,lam_y;
	int jjj=0;
	int sw_pos=0;
	int sw_th=0;
	int sw_thxy=0;
	int sw_x=0;
	int sw_y=0;

	if (argc != 5) {
		std::cout << "---------------------------------\n";
		std::cout << "usage: control CONFIGFILE IP_ADDR\n";
		std::cout << "---------------------------------\n";
		return 0;
	}

	control robot_control;
	robot_control.inicializa();

	//Creo objeto llamado base del tipo base_control.
	base_control base;
	base.inicializa();

	/*
	//INICIALIZACIÓN DEL BRAZO
	std::auto_ptr<CCdlSocket> device;
	std::auto_ptr<CCplSerialCRC> protocol;

	try {

	int port = 5566;
	device.reset(new CCdlSocket(argv[4], port));

	std::cout << "-------------------------------------------\n";
	std::cout << "success:  port " << port << " open\n";
	std::cout << "-------------------------------------------\n";

	protocol.reset(new CCplSerialCRC());
	std::cout << "-------------------------------------------\n";
	std::cout << "success: protocol class instantiated\n";
	std::cout << "-------------------------------------------\n";
	protocol->init(device.get());
	std::cout << "-------------------------------------------\n";
	std::cout << "success: communication with Katana initialized\n";
	std::cout << "-------------------------------------------\n";

	katana.reset(new CLMBase());
	katana->create(argv[3], protocol.get());

	//} catch(Exception &e) {
	} catch(DLLDIR::Exception &e) {
	std::cout << "ERROR: " << e.message() << std::endl;
	return -1;
	}

	std::cout << "success: katana initialized\n";

	std::cout << "Calibrando... espere a terminar con la calibración\n";
	katana->calibrate();

	//FIN INICIALIZACIÓN DEL BRAZO
	*/

	//INICIALIZACIÓN DE LA BASE
	Aria::init();

	ArArgumentParser argParser(&argc, argv);
	argParser.loadDefaultArguments();

	ArRobot robot;
	ArRobotConnector con(&argParser, &robot);

	// the connection handler from above
	ConnHandler ch(&robot);

	if(!Aria::parseArgs())
	{
		Aria::logOptions();
		Aria::exit(1);
		return 1;
	}

	if(!con.connectRobot())
	{
		ArLog::log(ArLog::Normal, "directMotionExample: Could not connect to the robot. Exiting.");

		if(argParser.checkHelpAndWarnUnparsed()) 
		{
			Aria::logOptions();
		}
		Aria::exit(1);
		return 1;
	}

	ArLog::log(ArLog::Normal, "directMotionExample: Connected.");

	if(!Aria::parseArgs() || !argParser.checkHelpAndWarnUnparsed())
	{
		Aria::logOptions();
		Aria::exit(1);
	}

	// Start the robot task loop running in a new background thread. The 'true' argument means if it loses
	// connection the task loop stops and the thread exits.
	robot.runAsync(true);
	//FIN INICIALIZACIÓN DE LA BASE

	/*
	//BRAZO
	//Obtención de los parámetros de los motores 0 al 2 (encoderOffset,encodersPerCycle,angleOffset,angleRange,rotationDirection,angleStop)
	const TMotInit* mot_data_0=katana->GetBase()->GetMOT()->arr[0].GetInitialParameters();
	const TMotInit* mot_data_1=katana->GetBase()->GetMOT()->arr[1].GetInitialParameters();
	const TMotInit* mot_data_2=katana->GetBase()->GetMOT()->arr[2].GetInitialParameters();
	*/

	//Para par calculado;
	kp0=0.86;
	//kp0=2; //mod
	kp1=0.25;
	//kp1=0.5; //mod
	kp2=1;
	kv0=1.855;
	//kv0=3;//mod
	kv1=1;
	//kv1=2; //mod
	kv2=2;
	ki0=0;
	ki1=0;
	ki2=0;

	/*
	//Para PID;
	kp0=8.6;
	kp1=2.5;
	kp2=10;
	kv0=18.55;
	kv1=10;
	kv2=20;
	ki0=0;
	ki1=0;
	ki2=0;
	*/

	printf("\nLos valores ingresado para las ganancias son:\n");
	printf("kp0=%f, kv0=%f\nkp1=%f, kv1=%f\nkp2=%f, kv2=%f\n",kp0,kv0,kp1,kv1,kp2,kv2);

	kp << kp0 << 0 << 0 << endr
		<< 0 << kp1 << 0 << endr
		<< 0 << 0 << kp2 << endr;

	kv << kv0 << 0 << 0 << endr
		<< 0 << kv1 << 0 << endr
		<< 0 << 0 << kv2 << endr;


	ki << ki0 << 0 << 0 << endr
		<< 0 << ki1 << 0 << endr
		<< 0 << 0 << ki2 << endr;

	kp.print("Matriz Kp:");
	kv.print("Matriz Kv:");
	ki.print("Matriz Ki:");


	//***************************************************************

	//VALORES INICIALES PARA LAS VARIABLES DEL BRAZO
	/*
	th0_an=katana->getMotorEncoders(0);
	th1_an=katana->getMotorEncoders(1);
	th2_an=katana->getMotorEncoders(2);
	*/

	/*
	int vel0_max=katana->getMotorVelocityLimit(0);
	int vel1_max=katana->getMotorVelocityLimit(1);
	int vel2_max=katana->getMotorVelocityLimit(2);
	printf("vel0_max=%d vel1_max=%d vel2_max=%d\n",vel0_max,vel1_max,vel2_max);
	*/

	//Para pruebas sin brazo
	th0_an=0;
	th1_an=0;
	th2_an=0;
	//

	th0_ac=th0_an;
	th1_ac=th1_an;
	th2_ac=th2_an;
	th0_med_an=th0_ac;
	th1_med_an=th1_ac;
	th2_med_an=th2_ac;

	vel0_an=0;
	vel1_an=0;
	vel2_an=0;

	acel0_control_an=0;
	acel1_control_an=0;
	acel2_control_an=0;
	vel0_control_an=0;
	vel1_control_an=1;
	vel2_control_an=2;
	pos1_control_an=0;
	vel2_control_an=0;
	pos0_control_an=0;
	pos1_control_an=0;
	pos2_control_an=0;

	i=0;
	sw0=0;
	sw1=0;
	sw2=0;

	vel0_ac=0;
	vel1_ac=0;
	vel2_ac=0;

	error_pos_an.at(0)=0;
	error_pos_an.at(1)=0;
	error_pos_an.at(2)=0;

	th0_itae=0;
	th1_itae=0;
	th2_itae=0;

	vel0_ref=0;
	vel1_ref=0;
	vel2_ref=0;
	//FIN INICIALIZACIÓN VARIABLES DEL BRAZO

	//Para el par calculado
	ab_lin_control_ac=0;
	vb_lin_control_ac=0;
	vb_lin_control_ant=0;
	ab_lin_control_ant=0;
	pb_lin_control_ac=0;

	ab_th_control_ac=0;
	vb_th_control_ac=0;
	ab_th_control_ant=0;
	pb_th_control_ac=0;
	vb_th_control_ant=0;

	/*
	//Aceptable
	kp1_base=0.757;
	kp2_base=0.49;
	kv1_base=1.74;
	kv2_base=1.4;
	*/

	kp1_base=100;
	kp2_base=100;
	kv1_base=0;
	kv2_base=0;

	/*
	//Aceptable 2
	kp1_base=4900;
	kp2_base=90000;
	kv1_base=14;
	kv2_base=203;
	*/

	/*
	//Aceptable 1
	kp1_base=49000;
	kp2_base=7570;
	kv1_base=1400;
	kv2_base=174;
	*/

	printf("\nLos valores ingresado para las ganancias de la base son:\n");
	printf("kp1=%f, kv1=%f\nkp2=%f, kv2=%f\n",kp1_base,kv1_base,kp2_base,kv2_base);

	kp_base << kp1_base << 0 << endr
		    << 0 << kp2_base << endr;

	kv_base << kv1_base << 0 << endr
		    << 0 << kv2_base << endr;

	kp_base.print("Matriz Kp:");
	kv_base.print("Matriz Kv:");

	v_ref_ant=0;
	vth_ref_ant=0;
	acel_ref_ac=0;
	//Fin par calculado

	//VALORES INICIALES PARA LAS VARIABLES DE LA BASE
	x_ref_ant=0;
	y_ref_ant=0;
	x_ref_ac=0;
	y_ref_ac=0;
	x_med_ac=0;
	y_med_ac=0;
	pos_med_ac=0;
	th_med_ac=0;
	vel_med_ac=0;
	pos_ref_ant=0;
	t_an=0;
	vel_med_ant=0;
	vth_med_ac=0;
	th_ref_ant=0;
	th_medxy_ant=0;
	th_refxy_ac=0;
	itae_pos_base=0;
	itae_x_base=0;
	itae_y_base=0;
	itae_th_base=0;
	//FIN INICIALIZACIÓN VARIABLES DE LA BASE

	//Obtengo los valores medidos de la base
	robot.lock();
	x_med_ac=robot.getX();
	y_med_ac=robot.getY();
	th_med_ac=robot.getTh();	//En grados
	robot.unlock();
	x_med_ant=x_med_ac;
	y_med_ant=y_med_ac;
	th_med_ant=th_med_ac;
	//Fin obtener valores medidos

	//Parámetros de delta de posición en encoders
	d_th0=100;
	d_th1=700;
	d_th2=400;

	int j=0;

	//Habro archivos para crear txt
	FILE *pf0;
	pf0=fopen("datos_th0.txt","w");

	FILE *pf1;
	pf1=fopen("datos_th1.txt","w");

	FILE *pf2;
	pf2=fopen("datos_th2.txt","w");

	//Habro archivo para crear txt
	FILE *itae_base;
	itae_base=fopen("itae_base.txt","w");

	//OJO CON LA DIRECCIÓN DEL MOVIMIENTO
	/*th0_err_final=std::abs(th0_tar-th0_ac);
	th1_err_final=std::abs(th1_tar-th1_ac);
	th2_err_final=std::abs(th2_tar-th2_ac);
	*/


	/*
	//Confuguración de la cámara
	VideoCapture capture;
	capture.open(CV_CAP_OPENNI);

	if( !capture.isOpened() ){
	cout << "Can not open a capture object." << endl;
	return -1;
	}
	*/

	static int posX = 0;
	static int posY = 0;

	int lastX = 0;
	int lastY = 0;
	int lastZ = 0;

	posX=0;
	posY=0;
	th_ref_cam=0;
	dist=0;

	//Fin configuración y variables de la cámara
	int ok_brazo=0;
	int ok_base=0;
	int zzz=0;
	int yyy=0;
	int www=0;

	/*
	//Obtención de la distancia
	while (1){
		if( !capture.grab() ){
			cout << "Can not grab images." << endl;
			return -1;
		}
		else{

			capture.set( CV_CAP_PROP_OPENNI_REGISTRATION , 0);
			capture.retrieve(world, CV_CAP_OPENNI_POINT_CLOUD_MAP);
			if( capture.retrieve( depthMap, CV_CAP_OPENNI_DEPTH_MAP ) ) 
				depthMap.convertTo( show, CV_8UC1, 0.05f);

			capture.retrieve( bgrImage, CV_CAP_OPENNI_BGR_IMAGE );
			//imshow( "bgr", bgrImage );

			//Variables de la cámara
			IplImage test = bgrImage;
			IplImage* redOnly = captura_rojo(&test);
			//IplImage* redOnly = captura_azul(&test);

			//Filtro para color rojo
			cvErode(redOnly,redOnly,NULL,1);
			cvMorphologyEx(redOnly,redOnly,redOnly,NULL,CV_MOP_CLOSE,5); 
			cvSmooth(redOnly,redOnly,CV_MEDIAN,5,5);

			//Filtro para color azul
			//cvSmooth(redOnly,redOnly,CV_MEDIAN,5,5);
			//cvMorphologyEx(redOnly,redOnly,redOnly,NULL,CV_MOP_CLOSE,27); 
			//cvDilate(redOnly,redOnly,NULL,7);

			//Cálculo del punto medio
			CvMoments *moments = (CvMoments*)malloc(sizeof(CvMoments));
			cvMoments(redOnly, moments);
			double moment10 = cvGetSpatialMoment(moments, 1, 0);
			double moment01 = cvGetSpatialMoment(moments, 0, 1);
			double area = cvGetCentralMoment(moments, 0, 0);

			posX = moment10/area;
			posY = moment01/area;

			//Posiciones del objeto en la cámara
			if ((posX>0)&&(posY>0)){
				z=profundidad(posX,posY);
				//Muestro la imagen con filtro
				//cvNamedWindow( "Con filtro", CV_WINDOW_AUTOSIZE );
				//cvShowImage("Con filtro", redOnly);

				//printf("posiciones x=%d y=%d z=%f\n",posX,posY,z);
				//printf("pixeles x=%d e y=%d\",)
				lastX = posX;
				lastY = posY;
				lastZ = z;

				if (zzz==0){
					th_ref_cam=-0.090625*posX+29;
					dist=z*1000;
					if (dist!=0){
						zzz=1;
						break;
					}
				}
			}
		}
	}
	*/

	dist=1000;
	th_ref_cam=10;

	/*
	//Solo para ajustar offset en pruebas de laboratorio
	if (th_ref_cam>0)
		th_ref_cam=th_ref_cam-3;

	if (th_ref_cam<0)
		th_ref_cam=th_ref_cam+3;

	*/

	printf("dist=%f th_ref_cam=%f posx=%d posy=%d\n",dist,th_ref_cam,posX,posY);
	system("pause");

	//Obtengo el tiempo para realizar la interpolación lineal
	t_an=timer_hr_count();
	t0_interp=t_an;

	while(1){

		t_ac=timer_hr_count();	
		dt=timer_hr_elapsed(t_an);

		if (t_ac!=t_an){

			//Guardo el tiempo y obtengo tiempo transcurrido desde el inicio del proceso
			tac_interp=timer_hr_elapsed(t0_interp);
			t_an=tac_interp-dt;

			printf("t_an=%g t_ac=%g dt=%g\n",t_an,tac_interp,dt);

			t_an=t_ac;

			/*
			if( !capture.grab() ){
			cout << "Can not grab images." << endl;
			return -1;
			}
			else{

			capture.set( CV_CAP_PROP_OPENNI_REGISTRATION , 0);
			capture.retrieve(world, CV_CAP_OPENNI_POINT_CLOUD_MAP);
			if( capture.retrieve( depthMap, CV_CAP_OPENNI_DEPTH_MAP ) ) 
			depthMap.convertTo( show, CV_8UC1, 0.05f);

			capture.retrieve( bgrImage, CV_CAP_OPENNI_BGR_IMAGE );
			//imshow( "bgr", bgrImage );

			//Variables de la cámara
			IplImage test = bgrImage;
			IplImage* redOnly = captura_rojo(&test);
			//IplImage* redOnly = captura_azul(&test);

			//Filtro para color rojo
			cvErode(redOnly,redOnly,NULL,1);
			cvMorphologyEx(redOnly,redOnly,redOnly,NULL,CV_MOP_CLOSE,5); 
			cvSmooth(redOnly,redOnly,CV_MEDIAN,5,5);

			//Filtro para color azul
			//cvSmooth(redOnly,redOnly,CV_MEDIAN,5,5);
			//cvMorphologyEx(redOnly,redOnly,redOnly,NULL,CV_MOP_CLOSE,27); 
			//cvDilate(redOnly,redOnly,NULL,7);

			//Cálculo del punto medio
			CvMoments *moments = (CvMoments*)malloc(sizeof(CvMoments));
			cvMoments(redOnly, moments);
			double moment10 = cvGetSpatialMoment(moments, 1, 0);
			double moment01 = cvGetSpatialMoment(moments, 0, 1);
			double area = cvGetCentralMoment(moments, 0, 0);

			posX = moment10/area;
			posY = moment01/area;

			//Posiciones del objeto en la cámara
			if ((posX>0)&&(posY>0)){
			z=profundidad(posX,posY);
			//Muestro la imagen con filtro
			//cvNamedWindow( "Con filtro", CV_WINDOW_AUTOSIZE );
			//cvShowImage("Con filtro", redOnly);

			//printf("posiciones x=%d y=%d z=%f\n",posX,posY,z);
			//printf("pixeles x=%d e y=%d\",)
			lastX = posX;
			lastY = posY;
			lastZ = z;

			if ((yyy==1)&&(www==0)){
			th0_tar=((posX*7.18)-599)+825;
			th1_tar=(posY*10,65)-4230;
			th2_tar=-17540;
			www=1;
			printf("th0_tar=%d th1_tar=%d th2_tar=%d\n",th0_tar,th1_tar,th2_tar);

			}

			}
			}
			*/

			//Para pruebas sin cámara
			if ((yyy==1)&&(www==0)){
				th0_tar=1893;
				th1_tar=(posY*12.0792)-2867-1900; //con 1600 offset respecto al centro de la cámara 
				th2_tar=-15217;
				www=1;
				printf("th0_tar=%d th1_tar=%d th2_tar=%d\n",th0_tar,th1_tar,th2_tar);
			}

			/*
			//****************************************
			//RUTINA PARA CONTROL DEL BRAZO
			//****************************************

			if (www==1){
			//Obtengo encoders de la posición actual 
			th0=katana->getMotorEncoders(0);
			th1=katana->getMotorEncoders(1);
			th2=katana->getMotorEncoders(2);

			//Vector para obtener interpolación lineal (en encoders)
			th0vec=th0_tar-th0;
			th1vec=th1_tar-th1;
			th2vec=th2_tar-th2;

			th0_err_final=std::abs(th0_tar-th0_ac);
			th1_err_final=std::abs(th1_tar-th1_ac);
			th2_err_final=std::abs(th2_tar-th2_ac);

			//Obtengo thetas de referencia usando ecuaciones paramétricas
			//Interpolación lineal para obtener ruta de referencia
			//Se realiza el cálculo siempre y cuando no se haya llegado al target

			if (sw0==0){
			lam0=(vel0_max*100/th0_err_final);
			th0_ref=th0_ac+(lam0/10000)*th0vec*tac_interp;
			}
			if (sw1==0){
			lam1=(vel1_max*100/th1_err_final);
			th1_ref=th1_ac+(lam1/10000)*th1vec*tac_interp;
			}
			if (sw2==0){
			lam2=(vel2_max*100/th2_err_final);
			th2_ref=th2_ac+(lam2/10000)*th2vec*tac_interp;
			}

			//2° método: interpolación según libro (no funciona)
			//th0_ref=(th0_ac-th0_med_an)*(((tac_interp-(t_an/2))-t_an)/dt)+th0_med_an;
			//th1_ref=(th1_ac-th1_med_an)*(((tac_interp-(t_an/2))-t_an)/dt)+th1_med_an;
			//th2_ref=(th2_ac-th2_med_an)*(((tac_interp-(t_an/2))-t_an)/dt)+th2_med_an;

			///Saturo posiciones
			if (th0_ref>31000)
			th0_ref=31000;
			else if (th0_ref<-18000)
			th0_ref=-18000;

			if (th1_ref>5935)
			th1_ref=5935;
			else if (th1_ref<-31000)
			th1_ref=-31000;

			if (th2_ref>1977)
			th2_ref=1977;
			else if (th2_ref<-31000)
			th2_ref=-31000;
			printf("th0_ref=%d th1_ref=%d th2_ref=%d\n",th0_ref,th1_ref,th2_ref);

			//Obtengo velocidad deseada y aceleración deseada (en encoders/seg y encoders/seg^2)
			if (th0_ref==th0_an){
			vel0_ref=0;
			acel0_ref=0;
			}
			else{
			vel0_ref=(th0_ref-th0_an)/dt;
			acel0_ref=(vel0_ref-vel0_an)/dt;
			}
			if (th1_ref==th1_an){
			vel1_ref=0;
			acel1_ref=0;
			}
			else{
			vel1_ref=(th1_ref-th1_an)/dt;
			acel1_ref=(vel1_ref-vel1_an)/dt;
			}
			if (th2_ref!=th2_an){
			vel2_ref=0;
			acel2_ref=0;
			}
			else{
			vel2_ref=(th2_ref-th2_an)/dt;
			acel2_ref=(vel2_ref-vel2_an)/dt;
			}

			acel_ref.at(0)=acel0_ref;
			acel_ref.at(1)=acel1_ref;
			acel_ref.at(2)=acel2_ref;

			//Guardo valores de posición y velocidad de referencia
			th0_an=th0_ref;
			th1_an=th1_ref;
			th2_an=th2_ref;
			vel0_an=vel0_ref;
			vel1_an=vel1_ref;
			vel2_an=vel2_ref;

			//Obtengo encoders de la posición actual
			th0_ac=katana->getMotorEncoders(0);
			th1_ac=katana->getMotorEncoders(1);
			th2_ac=katana->getMotorEncoders(2);

			//En realidad las unidades de current_angles son encoders
			current_angles.at(0)=th0_ac;
			current_angles.at(1)=th1_ac;
			current_angles.at(2)=th2_ac;

			//Obtengo velocidades actuales en encoders/seg
			vel0_ac=(th0_ac-th0_med_an)/dt;
			vel1_ac=(th1_ac-th1_med_an)/dt;
			vel2_ac=(th2_ac-th2_med_an)/dt;

			//Guardo valores de posición y velocidad actual medidas
			th0_med_an=th0_ac;
			th1_med_an=th1_ac;
			th2_med_an=th2_ac;
			current_vel.at(0)=vel0_ac;
			current_vel.at(1)=vel1_ac;
			current_vel.at(2)=vel2_ac;

			//Obtengo el error de posición con respecto a la referencia (en encoders) y los guardo en el vector err_pos
			th0_err=th0_ref-th0_ac;
			th1_err=th1_ref-th1_ac;
			th2_err=th2_ref-th2_ac;
			error_pos.at(0)=th0_err;
			error_pos.at(1)=th1_err;
			error_pos.at(2)=th2_err;
			printf("th0_err=%d th1_err=%d th2_err=%d\n",th0_err,th1_err,th2_err);

			//Obtengo el error en velocidad (encoders/s) y los guardo
			vel0_err=vel0_ref-vel0_ac;
			vel1_err=vel1_ref-vel1_ac;
			vel2_err=vel2_ref-vel2_ac;
			error_vel.at(0)=vel0_err;
			error_vel.at(1)=vel1_err;
			error_vel.at(2)=vel2_err;

			//Calculo el torque con control par calculado
			tau_control=robot_control.control_cpc(current_angles,current_vel,acel_ref,error_pos,error_vel,kp,kv);
			tau_control2.at(0)=tau_control.at(0,0);
			tau_control2.at(1)=tau_control.at(1,0);
			tau_control2.at(2)=tau_control.at(2,0);

			//Se realiza el cálculo de la aceleración deseada en base a la dinámica directa del robot
			acel_control=robot_control.din_dir(tau_control2,current_vel);

			acel0_control=acel_control.at(0,0);
			acel1_control=acel_control.at(1,0);
			acel2_control=acel_control.at(2,0);

			acel0_control_ac=acel0_control;
			acel1_control_ac=acel1_control;
			acel2_control_ac=acel2_control;

			//Método del rectángulo para integración
			vel0_control=dt*(acel0_control_an);
			pos0_control=dt*(vel0_control_an);
			acel0_control_an=acel0_control_ac;

			vel1_control=dt*(acel1_control_an);
			pos1_control=dt*(vel1_control_an);
			acel1_control_an=acel1_control_ac;

			vel2_control=dt*(acel2_control_an);
			pos2_control=dt*(vel2_control_an);
			acel2_control_an=acel2_control_ac;

			acel0_control_ac=acel0_control;
			acel1_control_ac=acel1_control;
			acel2_control_ac=acel2_control;

			//Integración usando método de Simpson
			//vel0_control=integral_simpson(dt,acel0_control_an,acel0_control_ac);
			//pos0_control=integral_simpson(dt,vel0_control_an,vel0_control);

			//vel1_control=integral_simpson(dt,acel1_control_an,acel1_control_ac);
			//pos1_control=integral_simpson(dt,vel1_control_an,vel1_control);

			//vel2_control=integral_simpson(dt,acel2_control_an,acel2_control_ac);
			//pos2_control=integral_simpson(dt,vel2_control_an,vel2_control);
			//Fin Control Par Calculado

			//Control PID
			//pos_control=robot_control.control_pid(error_pos,error_pos_an,error_vel,kp,kv,ki,dt);
			//pos0_control=pos_control.at(0,0);
			//pos1_control=pos_control.at(1,0);
			//pos2_control=pos_control.at(2,0);
			//Fin Control PID

			if (pos0_control>31000)
			pos0_control=31000;
			else if (pos0_control<-18000)
			pos0_control=-18000;

			if (pos1_control>5935)
			pos1_control=5935;
			else if (pos1_control<-31000)
			pos1_control=-31000;

			if (pos2_control>1977)
			pos2_control=1977;
			else if (pos2_control<-31000)
			pos2_control=-31000;

			//Guardo valores de posición, velocidad y aceleración
			pos0_control_an=pos0_control;
			pos1_control_an=pos1_control;
			pos2_control_an=pos2_control;

			//Solo para el Control Par Calculado
			vel0_control_an=vel0_control;
			vel1_control_an=vel1_control;
			vel2_control_an=vel2_control;
			acel0_control_an=acel0_control_ac;
			acel1_control_an=acel1_control_ac;
			acel2_control_an=acel2_control_ac;

			th0_err_final=(th0_tar-th0_ac);
			th1_err_final=(th1_tar-th1_ac);
			th2_err_final=(th2_tar-th2_ac);

			if ((std::abs(th0_err_final)<d_th0)&&(sw0==0)){					//Solo para depuración para probar primero con el primer grado de libertad.
			printf("Punto 0 logrado\n");
			katana->freezeMotor(0);
			i=i+1;
			sw0=1;
			}
			else{
			if (sw0==0){
			katana->mov(0,pos0_control);
			}
			else
			printf("Punto 0 ya alcanzado\n");
			}

			if ((std::abs(th1_err_final)<d_th1)&&(sw1==0)){					//Solo para depuración para probar primero con el primer grado de libertad.
			printf("Punto 1 logrado\n");
			katana->freezeMotor(1);
			i=i+1;
			sw1=1;
			}
			else{
			if (sw1==0){
			katana->mov(1,pos1_control);
			}
			else
			printf("Punto 1 ya alcanzado\n");
			}

			if ((std::abs(th2_err_final)<d_th2)&&(sw2==0)){					//Solo para depuración para probar primero con el primer grado de libertad.
			printf("Punto 2 logrado\n");
			katana->freezeMotor(2);
			i=i+1;
			sw2=1;
			}
			else{
			if (sw2==0){
			katana->mov(2,pos2_control);
			}
			else
			printf("Punto 2 ya alcanzado\n");
			}

			th0_ac=katana->getMotorEncoders(0);
			th1_ac=katana->getMotorEncoders(1);
			th2_ac=katana->getMotorEncoders(2);
			printf("th0_ac=%d th1_ac=%d th2_ac=%d\n",th0_ac,th1_ac,th2_ac);

			//Se calcula el error respecto a la posición donde se quiere llegar
			th0_err_final=(th0_tar-th0_ac);
			th1_err_final=(th1_tar-th1_ac);
			th2_err_final=(th2_tar-th2_ac);
			th0_err=th0_ref-th0_ac;
			th1_err=th1_ref-th1_ac;
			th2_err=th2_ref-th2_ac;

			//Cálculo itae						
			th0_itae=th0_itae+(tac_interp*std::abs(error_pos_an.at(0)))*dt;
			th1_itae=th1_itae+(tac_interp*std::abs(error_pos_an.at(1)))*dt;
			th2_itae=th2_itae+(tac_interp*std::abs(error_pos_an.at(2)))*dt;

			error_pos_an.at(0)=th0_err;
			error_pos_an.at(1)=th1_err;
			error_pos_an.at(2)=th2_err;

			fprintf(pf0,"%d	%d	%d	%f\n",th0_ref,th0_ac,th0_err,th0_itae);
			fprintf(pf1,"%d	%d	%d	%f\n",th1_ref,th1_ac,th1_err,th1_itae);
			fprintf(pf2,"%d	%d	%d	%f\n",th2_ref,th2_ac,th2_err,th2_itae);

			//dist=4000;
			//th_ref_cam=30;
			}
			*/

			//if (dist!=0){

			if (dist!=0){
			//if (th_ref_cam!=0){
				x_tar=dist*cos(th_ref_cam*PI/180);
				y_tar=dist*sin(th_ref_cam*PI/180);
				printf("dist=%g x_tar=%g y_tar=%g\n",dist,x_tar,y_tar);
				//system("pause");

				if ((y_tar==0)&&(x_tar>0))
					th_tar=0;
				else if ((y_tar==0)&&(x_tar<0))
					th_tar=-180;
				else if ((y_tar>0)&&(x_tar==0))
					th_tar=90;
				else if ((y_tar<0)&&(x_tar==0))
					th_tar=-90;
				else{
					//th_tar=(atan2(y_tar,x_tar))*180/PI;	//En grados
					th_tar=th_ref_cam;		//En grados
				}

				//dist=sqrt((pow(x_tar,2))+(pow(y_tar,2)));
				if (x_tar<0)			
					dist=-dist;

				max_vel_rot=50;
				max_vel_tra=0.25;
				max_vel_tra_x=max_vel_tra*cos(th_tar*PI/180);
				max_vel_tra_y=max_vel_tra*sin(th_tar*PI/180);

				//Obtengo los valores medidos
				//Posiciones actuales
				robot.lock();
				x_med_ac=robot.getX();
				y_med_ac=robot.getY();
				th_med_ac=robot.getTh();	//En grados
				robot.unlock();

				pos_med_ac=sqrt((pow(x_med_ac,2))+(pow(y_med_ac,2)));
				if (x_med_ac<0)			
					pos_med_ac=-pos_med_ac;

				//pos_med_ac=x_med_ac/cos(th_med_ac*PI/180);	//Transformo de grados a radianes
				//Fin obtener valores medidos

				//if (jjj==0){
				xbase_vec=x_tar-x_med_ac;
				ybase_vec=y_tar-y_med_ac;
				thbase_vec=th_tar-th_med_ac;
				//thbase_vec=err_th_final;
				//posbase_vec=err_pos_final;
				jjj=1;
				//}

				err_x_final=x_tar-x_med_ac;
				err_y_final=y_tar-y_med_ac;
				err_th_final=th_tar-th_med_ac;	//?????
				err_pos_final=dist-pos_med_ac;

				if (err_th_final==0)
					lam_rot=0;
				else
					lam_rot=(max_vel_rot/(ArMath::fabs(err_th_final)));

				if (sw_thxy==0)
					th_refxy_ac=th_medxy_ant+(lam_rot/100)*thbase_vec*tac_interp;	//En grados, cambiar th_medxy_ant por th_refxy_ant

				if (th_tar>0){
					if (th_refxy_ac>th_tar){
						th_refxy_ac=th_tar;
						sw_thxy=1;
					}
				}

				if (th_tar<0){
					if (th_refxy_ac<th_tar){
						th_refxy_ac=th_tar;
						sw_thxy=1;
					}
				}

				//th_refxy_ac=t_angulo(th_refxy_ac);	//Normalizo a +180° -180°
				//printf("th_refxy_ac=%f",th_refxy_ac);

				if (err_x_final==0)
					lam_x=0;
				else
					lam_x=ArMath::fabs(max_vel_tra_x/err_x_final);

				if (err_y_final==0)
					lam_y=0;
				else
					lam_y=ArMath::fabs(max_vel_tra_y/err_y_final);

				if (sw_x==0)
					x_ref_ac=(x_ref_ant+(lam_x/2)*xbase_vec*tac_interp)*3.0;

				if (sw_y==0)
					y_ref_ac=(y_ref_ant+(lam_y/2)*ybase_vec*tac_interp)*3.0;

				//Saturo referencias
				if (x_tar==0)
					x_ref_ac=0;
				else if ((x_tar>0)&&(x_ref_ac>x_tar)){
					x_ref_ac=x_tar;
					sw_x=1;
				}
				else if ((x_tar<0)&&(x_ref_ac<x_tar)){
					x_ref_ac=x_tar;
					sw_x=1;
				}

				if (y_tar==0)
					y_ref_ac=0;
				if ((y_tar>0)&&(y_ref_ac>y_tar)){
					y_ref_ac=y_tar;
					sw_y=1;
				}
				else if ((y_tar<0)&&(y_ref_ac<y_tar)){
					y_ref_ac=y_tar;
					sw_y=1;
				}

				pos_ref_ac=sqrt((pow(x_ref_ac,2))+(pow(y_ref_ac,2)));
				if (x_ref_ac<0)			
					pos_ref_ac=-pos_ref_ac;

				err_x=x_ref_ac-x_med_ac;
				err_y=y_ref_ac-y_med_ac;

				//th_ref_ac=atan2(err_y,err_x);
				th_ref_ac=atan2(err_y_final,err_x_final);
				th_ref_ac=th_ref_ac*180/PI;  //Transformo el ángulo de radianes a grados
				th_ref_ac=t_angulo(th_ref_ac);	//Normalizo a +180° -180°

				if (pos_ref_ac==pos_ref_ant)
					v_ref_ac=0;
				else
					v_ref_ac=(pos_ref_ac-pos_ref_ant)/dt;

				if (th_ref_ac==th_ref_ant)
					vth_ref_ac=0;
				else
					vth_ref_ac=(th_ref_ac-th_ref_ant)/dt;

				if (v_ref_ac==v_ref_ant)
					acel_ref_ac=0;
				else
					acel_ref_ac=(v_ref_ac-v_ref_ant)/dt;

				if (vth_ref_ac==vth_ref_ant)
					acelth_ref_ac=0;
				else
					acelth_ref_ac=(vth_ref_ac-vth_ref_ant)/dt;

				//Obtengo los errores de posición y velocidad respecto a la referencia, antes de aplicar el control
				err_x=x_ref_ac-x_med_ac;
				err_y=y_ref_ac-y_med_ac;

				err_th=th_ref_ac-th_med_ac;
				err_th=t_angulo(err_th);	//Normalizo a +180° -180°

				err_vel=v_ref_ac-vel_med_ac;
				err_vel_th=vth_ref_ac-vth_med_ac;
				err_pos=pos_ref_ac-pos_med_ac;
				err_pos_final=dist-pos_med_ac;

				//Para el par calculado
				qm.at(0)=pos_med_ac/1000;	//Traspaso a mts
				qm.at(1)=th_med_ac;

				qpm.at(0)=vel_med_ac/1000;	//Tra
				qpm.at(1)=vth_med_ac;

				qerr.at(0)=err_pos/1000;
				qerr.at(1)=err_th;

				qperr.at(0)=err_vel/1000;
				qperr.at(1)=err_vel_th;

				qppd.at(0)=acel_ref_ac/1000;
				qppd.at(1)=acelth_ref_ac;

				//Control Par calculado					
				tau_cpc=base.control_cpc(qm,qpm,qppd,qerr,qperr,kp_base,kv_base);
				taud.at(0)=tau_cpc.at(0,0);
				taud.at(1)=tau_cpc.at(1,0);
				qpp_base_control=base.din_dir(taud,qpm);
				ab_lin_control_ac=qpp_base_control.at(0,0);
				ab_lin_control_ac=ab_lin_control_ac*1000;	//Traspaso a mm

				ab_th_control_ac=qpp_base_control.at(1,0);
				//Método del rectángulo para integración
				vb_lin_control_ac=dt*(ab_lin_control_ant);
				pb_lin_control_ac=dt*(vb_lin_control_ant);

				vb_th_control_ac=dt*(ab_th_control_ant);
				pb_th_control_ac=dt*(vb_th_control_ant);

				//if (sw_th==1)
				//	system("pause");

				/*
				if ((pb_lin_control_ac>dist)&&(dist>0))
					pb_lin_control_ac=dist;
				else if ((pb_lin_control_ac<dist)&&(dist<0))
					pb_lin_control_ac=-dist;
				//else if ((pb_lin_control_ac<-pos_ref_ac)&&(pos_ref_ac>0))
				//	pb_lin_control_ac=-dist;
				
				
				if ((th_ref_ac>0)&&(pb_th_control_ac>180))
					pb_th_control_ac=th_ref_ac;
				else if ((th_ref_ac<0)&&(pb_th_control_ac<180))				
					pb_th_control_ac=-th_ref_ac;

				*/
				

				//pb_th_control_ac=t_angulo(pb_th_control_ac);	//Normalizo a +180° -180°

				printf("pl_control=%f pth_control=%f\n",pb_lin_control_ac,pb_th_control_ac);

				//Guardo valores actuales
				ab_lin_control_ant=ab_lin_control_ac;
				vb_lin_control_ant=vb_lin_control_ac;
				ab_th_control_ant=ab_th_control_ac;
				vb_th_control_ant=vb_th_control_ac;
				//Fin control par calculado

				if  (((ArMath::fabs(err_th))>0.5)&&(sw_th==0)){
					kp_th=1;
					kv_th=0;
					//PD_th=kp_th*err_th;	//PD
					//Aplico el control para el ángulo
					robot.lock();
					//robot.setDeltaHeading(PD_th);
					//robot.setDeltaHeading(pb_th_control_ac);
					//robot.setRotVel(vb_th_control_ac);
					robot.setRotAccel(ab_th_control_ac);
					robot.unlock();
				}
				else{
					sw_th=1;
				}

				if  (((ArMath::fabs(err_pos_final))>40)&&(sw_pos==0)){
				//if ((((ArMath::fabs(err_pos_final))-20)>20)&&(sw_pos==0)){
					kp_vel=0.1;
					kv_vel=0;

					if (x_tar<0)
						kp_vel=-kp_vel;

					//PD_vel=kp_vel*err_pos;
					//Aplico el control para la posición
					robot.lock();
					//robot.move(PD_vel);
					//robot.move(pb_lin_control_ac);		
					robot.move(vb_lin_control_ac);
					robot.unlock();
				}
				else{
					sw_pos=1;
				}

				//Guardo valores de posición, ángulo y velocidad del ángulo, antes de aplicar el control
				pos_ref_ant=pos_ref_ac;
				th_medxy_ant=th_refxy_ac; //Ya está en grados
				x_ref_ant=x_ref_ac;
				y_ref_ant=y_ref_ac;
				v_ref_ant=v_ref_ac;
				vth_ref_ant=vth_ref_ac;

				//Obtengo los valores medidos y los errores respecto a la referencia, después de aplicar el control
				robot.lock();
				x_med_ac=robot.getX();
				y_med_ac=robot.getY();
				th_med_ac=robot.getTh();	//En grados
				robot.unlock();

				pos_med_ac=sqrt((pow(x_med_ac,2))+(pow(y_med_ac,2)));
				if (x_med_ac<0)			
					pos_med_ac=-pos_med_ac;

				//Obtengo ITAE antes de volver a medir el error
				itae_pos_base=itae_pos_base+(tac_interp*ArMath::fabs(err_pos))*dt;
				itae_x_base=itae_x_base+(tac_interp*ArMath::fabs(err_x))*dt;
				itae_y_base=itae_y_base+(tac_interp*ArMath::fabs(err_y))*dt;
				itae_th_base=itae_th_base+(tac_interp*ArMath::fabs(err_th))*dt;

				fprintf(itae_base,"%f	%f	%f	%f	",pos_ref_ac,pos_med_ac,err_pos,itae_pos_base);
				fprintf(itae_base,"%f	%f	%f	%f	",x_ref_ac,x_med_ac,err_x,itae_x_base);
				fprintf(itae_base,"%f	%f	%f	%f	",y_ref_ac,y_med_ac,err_y,itae_y_base);
				fprintf(itae_base,"%f	%f	%f	%f\n",th_ref_ac,th_med_ac,err_th,itae_th_base);

				//Obtengo errores después de aplicar el control
				err_x=x_ref_ac-x_med_ac;
				err_y=y_ref_ac-y_med_ac;
				err_th=th_ref_ac-th_med_ac;
				err_pos=pos_ref_ac-pos_med_ac;

				printf("\n");
				printf("Ref. x_ref=%g, y_ref=%g, th_ref=%g th_refxy=%g pos_ref=%f\n",x_ref_ac,y_ref_ac,th_ref_ac,th_refxy_ac,pos_ref_ac);
				printf("Medidos x=%g y=%g th=%g pos=%f\n",x_med_ac,y_med_ac,th_med_ac,pos_med_ac);
				printf("Errores pos=%g x=%g y=%g th=%g vel=%g\n",err_pos,err_x,err_y,err_th,err_vel);
				printf("err_pos_final=%f\n",err_pos_final);

				if ((sw_th==1)&&(sw_pos==1)){
					robot.stop();
					printf("x_tar=%g y_tar=%g th_tar=%g pos_tar=%g\n",x_tar,y_tar,th_tar,dist);
					printf("llego...\n");
					robot.move(800);
					fclose(itae_base);
					system("pause");
					Aria::exit(0);
					return 0;
				}
			}

			/*
			if ((sw_th==1)&&(sw_pos==1)){
			//if (sw_th==1){
			robot.stop();
			printf("x_tar=%g y_tar=%g th_tar=%g pos_tar=%g\n",x_tar,y_tar,th_tar,dist);
			printf("llego...\n");
			//robot.move(700);
			fclose(itae_base);
			ok_base=1;
			//system("pause");
			//Aria::exit(0);
			//return 0;
			}
			}

			if ((i==3)&&(sw0==1)&&(sw1==1)&&(sw2==1)){

			ok_brazo=1;

			}
			//FIN RUTINA PARA CONTROL DEL BRAZO
			if ((ok_base==1)&&(ok_brazo==1)){
			katana->openGripper();
			katana->mov(3,5000,true);
			//katana->closeGripper(true);
			katana->mov(5,18670,true);
			katana->mov(1,-6000);
			printf("OK todos los puntos\n");
			printf("error final th0=%d th1=%d th2=%d\n",th0_err_final,th1_err_final,th2_err_final);
			fclose(pf0);
			fclose(pf1);
			fclose(pf2);
			system("pause");
			Aria::exit(0);
			pthread_exit((void*) &retVal);
			return 0;
			}
			*/
		}
	}
}


