#include <iostream>
#include <memory>
#include <vector>
#include<conio.h>		//Para capturar teclas con kbhit

//
#include <cv.h>
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
//#include "opencv2/imgproc/imgproc.hpp"
#include <time.h>

//#include <iostream>

using namespace cv;
using namespace std;


Mat world;
Mat depthMap;
Mat show;
Mat bgrImage;
Point coordenadas(0,0);

//Variables del teclado
#define KEY_UP 72
#define KEY_LEFT 75
#define KEY_RIGHT 77
#define KEY_DOWN 80
#define KEY_ESC 27

//VARIABLES BÁSICAS DEL BRAZO

#define  M_PI      3.141592653
//------------------------------------------------------------------------//

#ifdef WIN32
#include <conio.h>		//for keyboard strokes

// windows/linux workaround:
void init_keyboard() {}
void close_keyboard() {}

#else //LINUX
#include "keyboard.h"		//for keyboard strokes

#endif


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
	//cvInRangeS(imgHSV, cvScalar(110,150,100), cvScalar(120,255,150), imgThreshed);	//HSV AZUL

	cvReleaseImage(&imgHSV);
	return imgThreshed;
}

int main(int argc, char **argv) 
{
	//Definición de variables

	float th_ref_cam,div,dist;
	float delta;
	double vel_lin;
	double vel;
	int sw5=1;
	float z=0;
	//Fin definición de variables

	//Inicialización de la cámara
	VideoCapture capture;
	capture.open(CV_CAP_OPENNI);

	if( !capture.isOpened() ){
		cout << "Can not open a capture object." << endl;
		return -1;
	}
	//Fin inicialización de la cámara

	namedWindow( "bgr", 1 );
	while(1){
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
			cv::imshow("bgr", bgrImage );
			//imshow( "bgr", bgrImage );

			int c = waitKey(1);   // Tiempo de espera de 1 ms para capturar la tecla
			if (c == 27){ // Si presionamos Esc o se apretó el mouse termina.
				system("pause");
				return 0;
			}

			//Variables de la cámara
			static int posX = 0;
			static int posY = 0;

			int lastX = 0;
			int lastY = 0;
			int lastZ = 0;

			IplImage test = bgrImage;	//Creo una imagen del tipo IplImage

			//Segmento el color
			IplImage* redOnly = captura_rojo(&test);

			//Filtro para color rojo
			cvErode(redOnly,redOnly,NULL,1);
			cvMorphologyEx(redOnly,redOnly,redOnly,NULL,CV_MOP_CLOSE,5); 
			cvSmooth(redOnly,redOnly,CV_MEDIAN,5,5);

			/*
			//Azul
			cvSmooth(redOnly,redOnly,CV_MEDIAN,5,5);
			cvMorphologyEx(redOnly,redOnly,redOnly,NULL,CV_MOP_CLOSE,27); 
			cvDilate(redOnly,redOnly,NULL,7);
			*/

			//cvErode(redOnly,redOnly,NULL,1);
			//cvMorphologyEx(redOnly,redOnly,redOnly,NULL,CV_MOP_OPEN,7); 
			//cvMorphologyEx(redOnly,redOnly,redOnly,NULL,CV_MOP_GRADIENT,3);  //OK
			//cvSmooth(redOnly,redOnly,CV_GAUSSIAN,5,5);
			//cvMorphologyEx(redOnly,redOnly,redOnly,NULL,CV_MOP_CLOSE,7); 
			//cvMorphologyEx(redOnly,redOnly,redOnly,NULL,CV_MOP_TOPHAT,11); 
			//cvMorphologyEx(redOnly,redOnly,redOnly,NULL,CV_MOP_BLACKHAT,11); 
			//cvMorphologyEx(redOnly,redOnly,redOnly,NULL,CV_MOP_TOPHAT,11); 
			//cvSmooth(redOnly,redOnly,CV_MEDIAN,5,5);

			cvNamedWindow( "Con filtro", CV_WINDOW_AUTOSIZE );
			cvShowImage("Con filtro", redOnly);

			//Cálculo del punto medio
			CvMoments *moments = (CvMoments*)malloc(sizeof(CvMoments));
      		cvMoments(redOnly, moments);
			double moment10 = cvGetSpatialMoment(moments, 1, 0);
			double moment01 = cvGetSpatialMoment(moments, 0, 1);
			double area = cvGetCentralMoment(moments, 0, 0);

			posX = moment10/area;
			posY = moment01/area;

			printf("posiciones x=%d y=%d z=%f\n",posX,posY,z);

			//Posiciones del objeto en la cámara
			if ((posX>0)&&(posY>0)){
				
				z=profundidad(posX,posY);

				//cvCircle(redOnly,cvPoint(int(posX),int(posY)), 10, CV_RGB(200,50,50),3);
				//Muestro la imagen con filtro
				//cvNamedWindow( "Con filtro", CV_WINDOW_AUTOSIZE );
				//cvShowImage("Con filtro", redOnly);

				lastX = posX;
				lastY = posY;
				lastZ = z;

				th_ref_cam=-0.090625*posX+29;
				dist=z*1000;

				printf("Desplazamiento requerido z=%f th=%f\n",dist,th_ref_cam);

			}
		}
	}
}	


