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
//#include "robot_control.h"		


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

#define PI 3.14159265

cv::Mat world;
cv::Mat depthMap;
cv::Mat show;
cv::Mat bgrImage;
cv::Point coordenadas(0,0);
float dist,x,y,z;

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

double *ruta_ref(double x_tar, double y_tar, double x_ref_ant, double y_ref_ant, double t_ac, double dt,double vel_med_ant){
	double x_ref_ac,y_ref_ac,dx,dy,dpos,vel,th;
	double *ruta;
	int tamano = 4; // asignamos el tamaño del vector
	ruta = (double*) malloc ( sizeof(int) * tamano); // se crea el arreglo

	//	x_ref_ac=x_ref_ant+x_tar*t_ac;
	//	y_ref_ac=y_ref_ant+y_tar*t_ac;
	x_ref_ac=x_ref_ant+10;
	y_ref_ac=y_ref_ant+10;


	if (x_ref_ac>x_tar)
		x_ref_ac=x_tar;
	if (y_ref_ac>y_tar)
		y_ref_ac=y_tar;

	dx=x_ref_ac-x_ref_ant;
	dy=y_ref_ac-y_ref_ant;

	dpos=sqrt((pow(dx,2))+(pow(dy,2)));
	vel=dpos/dt;
	th=atan2(y_ref_ac,x_ref_ac)*180/PI;

	if ((x_ref_ac!=0)&&(y_ref_ac!=0)){
		ruta[0]=x_ref_ac;
		ruta[1]=y_ref_ac;
		ruta[2]=th;
		ruta[3]=vel;
	}
	else{
		ruta[0]=0;
		ruta[1]=0;
		ruta[2]=0;
		ruta[3]=0;
	}
	return ruta;
}

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

//Rutina para obtener distancia hacia el objeto
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

int integral_simpson(double dt,int f_ant,int f_act){
	int integral=(dt/6)*(f_ant+4*((f_ant+f_act)/2)+f_act);
	return integral;
}

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
int main(int argc, char **argv) 
{

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

	//Declaración de variables
	double x_med_ac,y_med_ac,pos_med_ac,th_med_ac,vel_med_ac;
	double x_ref_ant,y_ref_ant,th_ref_ant,th_refxy_ac,th_medxy_ant,x_ref_ac,y_ref_ac,th_ref_ac,v_ref_ac;
	double x_med_ant,y_med_ant,th_med_ant,vel_med_ant;
	double err_x,err_y,err_th,err_vel,err_pos;
	double kp_th,kp_vel,kv_vel,kv_th,PD_vel,PD_th;
	double dif_an;
	double err_vel_th,vth_ref_ac,vth_med_ac;

	//Parámetros de tiempo
	double t_ac,t_an,dt,t0_interp,tac_interp;	//variables para captura de tiempo

	double x_tar,y_tar;

	double *vec_pos_ref;
	//Fin declaración de variables

	double pos_ref_ac,pos_ref_ant;
	double th_tar;
	double xbase_vec,ybase_vec,thbase_vec;
	double err_pos_final,err_th_final,err_x_final,err_y_final,posbase_vec;
	double max_vel_tra,max_vel_rot;
	double lamx_tra,lamy_tra,lam_rot,lampos_tra;
	int ii=1;
	float cth,sth;
	double itae_pos_base,itae_x_base,itae_y_base,itae_th_base;

	//Creo objeto llamado base del tipo base_control.
	base_control base;
	base.inicializa();

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

	ArTime start;
	start.setToNow();

	//Inicialización de variables
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

	//Inicialización de variables para el par calculado
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

	//Para el par calculado
	kp1_base=0.49;
	kp2_base=19600;
	kv1_base=1.4;
	kv2_base=280;

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

	double max_vel_tra_x,max_vel_tra_y,lam_x,lam_y;

	itae_pos_base=0;
	itae_x_base=0;
	itae_y_base=0;
	itae_th_base=0;

	//Habro archivo para crear txt
	FILE *itae_base;
	itae_base=fopen("itae_base.txt","w");

	//Obtengo los valores medidos
	//Posiciones actuales
	robot.lock();
	x_med_ac=robot.getX();
	y_med_ac=robot.getY();
	th_med_ac=robot.getTh();	//En grados
	robot.unlock();
	x_med_ant=x_med_ac;
	y_med_ant=y_med_ac;
	th_med_ant=th_med_ac;
	//Fin obtener valores medidos

	int jjj=0;
	int sw_pos=0;
	int sw_th=0;
	int sw_thxy=0;
	int sw_x=0;
	int sw_y=0;

	/*
	//Inicialización de la cámara y variables asociadas
	VideoCapture capture;
	capture.open(CV_CAP_OPENNI);

	if( !capture.isOpened() ){
	cout << "Can not open a capture object." << endl;
	return -1;
	}

	static int posX = 0;
	static int posY = 0;

	int lastX = 0;
	int lastY = 0;
	int lastZ = 0;

	int sw_cam=0;
	//Fin configuración y variables de la cámara
	*/

	dist=0;

	t_an=timer_hr_count();
	t0_interp=t_an;

	while(1){

		t_ac=timer_hr_count();	
		dt=timer_hr_elapsed(t_an);

		if (t_ac!=t_an){

			//Guardo el tiempo y obtengo tiempo transcurrido desde el inicio del proceso
			tac_interp=timer_hr_elapsed(t0_interp);
			t_an=tac_interp-dt;

			/*
			if( !capture.grab() ){
			cout << "Can not grab images." << endl;
			return -1;
			}else{

			//Resolución de 640x480
			capture.set( CV_CAP_PROP_OPENNI_REGISTRATION , 0);
			capture.retrieve(world, CV_CAP_OPENNI_POINT_CLOUD_MAP);
			if( capture.retrieve( depthMap, CV_CAP_OPENNI_DEPTH_MAP ) ) 
			depthMap.convertTo( show, CV_8UC1, 0.05f);

			capture.retrieve( bgrImage, CV_CAP_OPENNI_BGR_IMAGE );
			//cv::imshow("bgr", bgrImage );

			IplImage test = bgrImage;	//Creo una imagen del tipo IplImage
			//Segmento el color
			IplImage* redOnly = captura_rojo(&test);


			//Filtro para color rojo
			cvErode(redOnly,redOnly,NULL,1);
			cvMorphologyEx(redOnly,redOnly,redOnly,NULL,CV_MOP_CLOSE,5); 
			cvSmooth(redOnly,redOnly,CV_MEDIAN,5,5);


			//Azul
			//cvSmooth(redOnly,redOnly,CV_MEDIAN,5,5);
			//cvMorphologyEx(redOnly,redOnly,redOnly,NULL,CV_MOP_CLOSE,27); 
			//cvDilate(redOnly,redOnly,NULL,7);

			//cvNamedWindow( "Con filtro", CV_WINDOW_AUTOSIZE );
			//cvShowImage("Con filtro", redOnly);

			//Cálculo del punto medio
			CvMoments *moments = (CvMoments*)malloc(sizeof(CvMoments));
			cvMoments(redOnly, moments);
			double moment10 = cvGetSpatialMoment(moments, 1, 0);
			double moment01 = cvGetSpatialMoment(moments, 0, 1);
			double area = cvGetCentralMoment(moments, 0, 0);

			posX = moment10/area;
			posY = moment01/area;

			//printf("posiciones x=%d y=%d z=%f\n",posX,posY,z);

			//Posiciones del objeto en la cámara
			if ((posX>0)&&(posY>0)){

			z=profundidad(posX,posY);

			lastX = posX;
			lastY = posY;
			lastZ = z;

			th_ref_cam=-0.090625*posX+29;
			dist=z*1000;

			//printf("Desplazamiento requerido z=%f th=%f\n",dist,th_ref_cam);

			}
			}
			*/

			/*
			//Para pruebas sin cámara
			posX=285;
			posY=274;
			th0_tar=(posX*8.184375)-1485;
			th1_tar=((posY-210)*15.567742)-5244;
			th2_tar=-18464;
			*/
			//Fin captura y segmentación del color

			dist=4000;
			th_ref_cam=120;

			if (dist!=0){
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
					x_ref_ac=(x_ref_ant+(lam_x/2)*xbase_vec*tac_interp)*1.5;

				if (sw_y==0)
					y_ref_ac=(y_ref_ant+(lam_y/2)*ybase_vec*tac_interp)*1.5;

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
				qm.at(0)=pos_med_ac;
				qm.at(1)=th_med_ac;

				qpm.at(0)=vel_med_ac;
				qpm.at(1)=vth_med_ac;

				qerr.at(0)=err_pos;
				qerr.at(1)=err_th;

				qperr.at(0)=err_vel;
				qperr.at(1)=err_vel_th;

				qppd.at(0)=acel_ref_ac;
				qppd.at(1)=acelth_ref_ac;

				//Control Par calculado					
				tau_cpc=base.control_cpc(qm,qpm,qppd,qerr,qperr,kp_base,kv_base);
				taud.at(0)=tau_cpc.at(0,0);
				taud.at(1)=tau_cpc.at(1,0);
				qpp_base_control=base.din_dir(taud,qpm);
				ab_lin_control_ac=qpp_base_control.at(0,0);
				ab_th_control_ac=qpp_base_control.at(1,0);

				//Método del rectángulo para integración
				vb_lin_control_ac=dt*(ab_lin_control_ant);
				pb_lin_control_ac=dt*(vb_lin_control_ant);

				vb_th_control_ac=dt*(ab_th_control_ant);
				pb_th_control_ac=dt*(vb_th_control_ant);

				//Guardo valores actuales
				ab_lin_control_ant=ab_lin_control_ac;
				vb_lin_control_ant=vb_lin_control_ac;
				ab_th_control_ant=ab_th_control_ac;
				vb_th_control_ant=vb_th_control_ac;

				//Fin control par calculado


				if  (((ArMath::fabs(err_th))>1)&&(sw_th==0)){
					kp_th=1;
					kv_th=0;
					//PD_th=kp_th*err_th;	//PD
					//Aplico el control para el ángulo
					robot.lock();
					//robot.setDeltaHeading(PD_th);
					robot.setDeltaHeading(pb_th_control_ac);
					robot.unlock();
				}
				else{
					sw_th=1;
				}

				//if  (((ArMath::fabs(err_pos_final))>20)&&(sw_pos==0)){
				if ((((ArMath::fabs(err_pos_final))-1000)>20)&&(sw_pos==0)){
					kp_vel=0.1;
					kv_vel=0;

					if (x_tar<0)
						kp_vel=-kp_vel;

					//PD_vel=kp_vel*err_pos;
					//Aplico el control para la posición
					robot.lock();
					//robot.move(PD_vel);
					robot.move(pb_lin_control_ac);					
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
		}
	}
}

/*
int *devuelve_arreglo ()
{
int *arreglo; // declaramos en apuntador
int tamano = 4; // asignamos el tamaño del vector
arreglo = (int*) malloc ( sizeof(int) * tamano); // se crea el arreglo
arreglo[0] = 1; // asignamos valores
arreglo[1] = 2; // asignamos valores
arreglo[2] = 3; // asignamos valores
arreglo[3] = 4; // asignamos valores
return arreglo;
}
*/