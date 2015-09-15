/*****************************************************************************************
Copyright 2011 Rafael Muñoz Salinas. All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are
permitted provided that the following conditions are met:

   1. Redistributions of source code must retain the above copyright notice, this list of
	  conditions and the following disclaimer.

   2. Redistributions in binary form must reproduce the above copyright notice, this list
	  of conditions and the following disclaimer in the documentation and/or other materials
	  provided with the distribution.

THIS SOFTWARE IS PROVIDED BY Rafael Muñoz Salinas ''AS IS'' AND ANY EXPRESS OR IMPLIED
WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND
FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL Rafael Muñoz Salinas OR
CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

The views and conclusions contained in the software and documentation are those of the
authors and should not be interpreted as representing official policies, either expressed
or implied, of Rafael Muñoz Salinas.
********************************************************************************************/
#include <iostream>
#include <fstream>
#include <sstream>
#include "aruco.h"
#include "cvdrawingutils.h"
#include <opencv2/highgui/highgui.hpp>


#include <thread>
#include <opencv2/opencv.hpp>
#include <cassert>
#include <cmath>
//#include "OctoArcasController.h"
//#include <Task.h>
#include <queue>
#include <time.h>
#include <string>
#include "comunication.h"
/*
//#include <arpa/inet.h>
//#include <inet.h>
#include <Windows.h>
//#include <stdio.h>
#include <WinSock.h>
//#include <Winsock2.h>
//#include <unistd.h>
#include <io.h>
#include <stdlib.h>

#include "comunication.h"

#define BUFLEN 512
#define PORT 44000

#define IP "10.0.0.211"
typedef struct  msgSend
	
	{
	  unsigned int camera;
	  double posX;
	  double posY;
	  double posZ;
	  double x;
	  double y;
	  double z;
	  double w;
	  unsigned int markerId;

};
struct sockaddr_in serv_addr;
int sockfd, i, slen = sizeof(serv_addr);
*/
#define MAX_DATE 12


using namespace std;
using namespace cv;
using namespace aruco;


string get_date(void);
string TheInputVideo;
string TheIntrinsicFile;
float TheMarkerSize=-1;
int ThePyrDownLevel;
MarkerDetector MDetector;
VideoCapture TheVideoCapturer;
vector<Marker> TheMarkers;
Mat TheInputImage, TheInputImage1, TheInputImageCopy;
CameraParameters TheCameraParameters;
void cvTackBarEvents(int pos,void*);
void ObtainBarCenter(int indice, Point2f diff,Point2f middlePoint);
Mat Stabilize();
bool readCameraParameters(string TheIntrinsicFile,CameraParameters &CP,Size size);
pair<double,double> AvrgTime(0,0) ;//determines the average time required for detection
double ThresParam1,ThresParam2;
int iThresParam1,iThresParam2;
int waitTime=0;
vector<Point2f>centers[2];

/************************************
 *
 *
 *
 *
 ************************************/
 /*void setupUDP() {
	
	 WSADATA wsaData;

	 int iResult;

	 // Initialize Winsock
	 iResult = WSAStartup(MAKEWORD(2, 2), &wsaData);
	 if (iResult != 0) {
		 printf("WSAStartup failed: %d\n", iResult);
		 exit;
	 }
	 
	 
	 if ((sockfd = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP)) == -1)
		std::cout << "Error creating socket descriptor" << std::endl;

	//bzero(&serv_addr, sizeof(serv_addr));
	memset(&serv_addr, 0,sizeof(serv_addr));
	serv_addr.sin_family = AF_INET;
	serv_addr.sin_port = htons(PORT);
	//if (inet_aton(IP, &serv_addr.sin_addr) == 0) {
	if (serv_addr.sin_addr.s_addr = inet_addr(IP) == 0) {
		std::cout << "Failed decode of host" << std::endl;
	}
}
void closeUDP() {
	closesocket(sockfd);
}*/
 /*****************************************/
bool readArguments ( int argc,char **argv )
{
   /* if (argc<2) {
		cerr<<"Invalid number of arguments"<<endl;
		cerr<<"Usage: (in.jpg|in.avi|live[:idx_cam=0]) [intrinsics.yml] [size]"<<endl;
		return false;
	}*/
	TheInputVideo = "live";//argv[1];
	//if (argc >= 3)
		TheIntrinsicFile = "cameraocto.xml";//argv[2];
	//if (argc >= 4)
		TheMarkerSize = atof("0.05");//argv[3]);

	//if (argc==3)
		cerr<<"NOTE: You need makersize to see 3d info!!!!"<<endl;
	return true;
}

int findParam ( std::string param,int argc, char *argv[] )
{
	for ( int i=0; i<argc; i++ )
		if ( string ( argv[i] ) ==param ) return i;

	return -1;

}
/************************************
 *
 *
 *
 *
 ************************************/
int main(int argc,char **argv)
{
	//Medir duración de procesamiento de una imagen
	int tiempo;
	clock_t inicio, parada;
	setupUDP();

	try
	{
		if (readArguments (argc,argv)==false) {
			return 0;
		}
		//parse arguments
		
		//read from camera or from  file
		if (TheInputVideo.find("live")!=string::npos) {
	  int vIdx=0;
	  //check if the :idx is here
	  char cad[100];
	  if (TheInputVideo.find(":")!=string::npos) {
		  std::replace(TheInputVideo.begin(),TheInputVideo.end(),':',' ');
		 sscanf(TheInputVideo.c_str(),"%s %d",cad,&vIdx);
	  }
		cout<<"Opening camera index "<<vIdx<<endl;
			TheVideoCapturer.open(vIdx);
			waitTime=10;
		}
		else  
		if (TheInputVideo.find(".avi") != string::npos){
				TheVideoCapturer.open(TheInputVideo);
		//check video is open
				if (!TheVideoCapturer.isOpened()) {
					cerr << "Could not open video" << endl;
					return -1;
				}
		}
		else TheInputImage=cv::imread(TheInputVideo);
				
		//read first image to get the dimensions
				TheVideoCapturer >> TheInputImage;
				//Stabilize().copyTo(TheInputImage);
		//read camera parameters if passed
		if (TheIntrinsicFile!="") {
			TheCameraParameters.readFromXMLFile(TheIntrinsicFile);
			TheCameraParameters.resize(TheInputImage.size());
		}
		//Configure other parameters
		if (ThePyrDownLevel>0)
			MDetector.pyrDown(ThePyrDownLevel);


		//Create gui

		//cv::namedWindow("thres",1);
		cv::namedWindow("in",1);
		MDetector.getThresholdParams( ThresParam1,ThresParam2);       
		MDetector.setCornerRefinementMethod(MarkerDetector::SUBPIX);
		iThresParam1=ThresParam1;
		iThresParam2=ThresParam2;
		cv::createTrackbar("ThresParam1", "in",&iThresParam1, 13, cvTackBarEvents);
		cv::createTrackbar("ThresParam2", "in",&iThresParam2, 13, cvTackBarEvents);

		char key=0;
		int index=0, imgcounter=0;
		
		 int positive=0, negative=0, total=0;
		 int posporcentage=0, negporcentage=0;
		do
		{
			//Inicio contador tiempo de proceso.
			inicio = clock();
			TheVideoCapturer.retrieve(TheInputImage);
			//Stabilize().copyTo(TheInputImage);
			
			//copy image

			index++; //number of images captured
			double tick = (double)getTickCount();//for checking the speed
			//Detection of markers in the image passed
			//MDetector.setDesiredSpeed(4);
			MDetector.detect(TheInputImage, TheMarkers, TheCameraParameters, TheMarkerSize);
			//cout << "Size of Markers " << TheMarkers.size() << endl;
			//chekc the speed by calculating the mean speed of all iterations
			AvrgTime.first += ((double)getTickCount() - tick) / getTickFrequency();
			AvrgTime.second++;
			//cout<<"\rTime detection="<<1000*AvrgTime.first/AvrgTime.second<<" milliseconds nmarkers="<<TheMarkers.size()<< std::flush;


			if (TheMarkers.size()==0){
					   negative++;
					   total++;
			}else if (TheMarkers.size()>0){
					   positive++;
					   total++;
			}

			posporcentage= (positive*100/total);
			negporcentage= (negative*100/total);
		    //cout<<endl;
			//cout<<"Positive Detection:"<<posporcentage<<"%"<<endl;
			//cout<<"Negative Detection:"<<negporcentage<<"%"<<endl;
			//cout<<"total:"<<total<<endl;

			//print marker info and draw the markers in image

			TheInputImage.copyTo(TheInputImageCopy);
			float Tcx, Tcy;
			Point2f centers, middlePoint,diff;

			if (TheMarkers.size() > 1){

				Point2f point1 = TheMarkers[0].getCenter();
				Point2f point2 = TheMarkers[1].getCenter();
				Point2f middlePoint1, center;
				middlePoint.x = (point1.x + point2.x) / 2;
				middlePoint.y = (point1.y + point2.y) / 2;

				cv::circle(TheInputImageCopy, middlePoint, 10, Scalar(255, 0, 0, 255), 1, CV_AA);
				//cv::line(TheInputImageCopy, point1, point2, Scalar(255, 0, 0, 255), 1, CV_AA);

				centers.x = abs(point1.x - point2.x);
				centers.y = abs(point1.y - point2.y);
				Tcx = point1.x;
				Tcy = point1.y;

				
				diff.y = (TheMarkers[0][1].y - TheMarkers[0][2].y);
				diff.x = (TheMarkers[0][0].y - TheMarkers[0][1].y);
			
				
				
			}
			else
			if (TheMarkers.size() != 0)  {
			
				diff.y = (TheMarkers[0][1].y - TheMarkers[0][2].y);
				diff.x = (TheMarkers[0][0].y - TheMarkers[0][1].y);
			}

			//cout << "size of candidates" << MDetector.getCandidates().size()<< endl;

			//print other rectangles that contains no valid markers
			/*for (unsigned int i=0;i<MDetector.getCandidates().size();i++) {
				aruco::Marker m( MDetector.getCandidates()[i],999);
				m.draw(TheInputImageCopy,cv::Scalar(255,0,0));
			}*/
			
			int j = 0;
			
			
			//draw a 3d cube in each marker if there is 3d info
			if (  TheCameraParameters.isValid())
				for (unsigned int i=0;i<TheMarkers.size();i++) {

					
					CvDrawingUtils::draw3dCube(TheInputImageCopy,TheMarkers[i],TheCameraParameters);
					
					CvDrawingUtils::draw3dAxis(TheInputImageCopy,TheMarkers[i],TheCameraParameters);
					if (i < TheMarkers.size())
						j = i+1;
				}
				if (TheMarkers.size() != 0){
					ObtainBarCenter(j,diff, middlePoint);
				}
				
				
			//DONE! Easy, right?
			//show input with augmented information and  the thresholded image
			cv::imshow("in",TheInputImageCopy);
			
			//Almacenamiento de imágenes data sheet

			std::stringstream ss, ss1;
					
			ss << "C:/programming/image/image_" << get_date()<< imgcounter << time << ".jpg";
			ss1 << "C:/programming/imagesin/image_" << imgcounter << time << ".jpg";
			
			cv::imwrite(ss.str(),TheInputImageCopy);
			cv::imwrite(ss1.str(), TheInputImage);
		
			//std::cout << "storing img in: " << ss.str() << std::endl;
			//cout << "Num of img:" << imgcounter++;
			imgcounter++;


		   //cv::imshow("thres",MDetector.getThresholdedImage());
			TheMarkers.clear();
			
			
			key=cv::waitKey(waitTime);//wait for key to be pressed
			
			//Calculo del tiempo de proceso
			parada = clock();
			tiempo = 1000L * (parada - inicio) / CLK_TCK;
			cout << tiempo << endl;
			
		}while(key!=27 && TheVideoCapturer.grab());
		
	} catch (std::exception &ex)

	{
		cout<<"Exception :"<<ex.what()<<endl;
	}
	
}	
/************************************
 
 
 
 
 ************************************/
std::string get_date(void)
{
	time_t now;
	char the_date[MAX_DATE];

	the_date[0] = '\0';

	now = time(NULL);

	if (now != -1)
	{
		strftime(the_date, MAX_DATE, "%d_%m_%Y", gmtime(&now));
	}

	return std::string(the_date);
}




void cvTackBarEvents(int pos,void*)
{
	if (iThresParam1<3) iThresParam1=3;
	if (iThresParam1%2!=1) iThresParam1++;
	if (ThresParam2<1) ThresParam2=1;
	ThresParam1=iThresParam1;
	ThresParam2=iThresParam2;
	MDetector.setThresholdParams(ThresParam1,ThresParam2);
//recompute
	MDetector.detect(TheInputImage,TheMarkers,TheCameraParameters);
	TheInputImage.copyTo(TheInputImageCopy);
	
	for (unsigned int i=0;i<TheMarkers.size();i++)	TheMarkers[i].draw(TheInputImageCopy,Scalar(0,0,255),1);
	//print other rectangles that contains no valid markers
	for (unsigned int i=0;i<MDetector.getCandidates().size();i++) {
		aruco::Marker m( MDetector.getCandidates()[i],999);
		m.draw(TheInputImageCopy,cv::Scalar(255,0,0));
	}
	
//draw a 3d cube in each marker if there is 3d info
	if (TheCameraParameters.isValid())
		for (unsigned int i=0;i<TheMarkers.size();i++)
			CvDrawingUtils::draw3dCube(TheInputImageCopy,TheMarkers[i],TheCameraParameters);

	cv::imshow("in",TheInputImageCopy);
	cv::imshow("thres",MDetector.getThresholdedImage());

}

void ObtainBarCenter(int indice,Point2f diff,Point2f middlePoint){
	
		
	int j = indice;
	TheMarkers.push_back(TheMarkers[0]);

	if (TheMarkers.size() == 2){

		const float PI = 3.1415926535897932384626433832795;
		double halfSize = TheMarkerSize / 2.;

		cv::Mat D3Points(4, 3, CV_32FC1);

		D3Points.at<float>(1, 0) = -halfSize;
		D3Points.at<float>(1, 1) = halfSize;
		D3Points.at<float>(1, 2) = 0;
		D3Points.at<float>(2, 0) = halfSize;
		D3Points.at<float>(2, 1) = halfSize;
		D3Points.at<float>(2, 2) = 0;
		D3Points.at<float>(3, 0) = halfSize;
		D3Points.at<float>(3, 1) = -halfSize;
		D3Points.at<float>(3, 2) = 0;
		D3Points.at<float>(0, 0) = -halfSize;
		D3Points.at<float>(0, 1) = -halfSize;
		D3Points.at<float>(0, 2) = 0;

		float anglex = cos((TheMarkers[j].Rvec.ptr<float>(0)[0]));
		float angley = sin((TheMarkers[j].Rvec.ptr<float>(0)[0]));
		float distz = TheMarkers[j].Tvec.ptr<float>(0)[2];

		if (TheMarkers[0].id == 735){

			if (abs(anglex) < abs(angley)){

				if (abs(TheMarkers[j].Rvec.ptr<float>(0)[0]) > 1 && abs(TheMarkers[j].Rvec.ptr<float>(0)[1])<3 && TheMarkers[j].Rvec.ptr<float>(0)[1] > 0){
					anglex = (280 * (-1 * anglex));
					distz = 60 * (1 - distz);
					angley = 0;
				}
				else{

					anglex = (280 * (anglex));
					distz = 60 * (1 * distz);
					angley = 0;
				}

				TheMarkers[j][0].x = TheMarkers[j][0].x - (anglex)+(distz);
				TheMarkers[j][0].y = TheMarkers[j][0].y + (angley);
				TheMarkers[j][1].x = TheMarkers[j][1].x - (anglex)+(distz);
				TheMarkers[j][1].y = TheMarkers[j][1].y + (angley);
				TheMarkers[j][2].x = TheMarkers[j][2].x - (anglex)+(distz);
				TheMarkers[j][2].y = TheMarkers[j][2].y + (angley);
				TheMarkers[j][3].x = TheMarkers[j][3].x - (anglex)+(distz);
				TheMarkers[j][3].y = TheMarkers[j][3].y + (angley);

			}
			else{

				if (abs(TheMarkers[j].Rvec.ptr<float>(0)[0]) > 1 && abs(TheMarkers[j].Rvec.ptr<float>(0)[1]) < 3 && TheMarkers[j].Rvec.ptr<float>(0)[0] < 0){
					anglex = (100 * (anglex));
					distz = 80 * (-1 * distz);
					angley = 0;
				}
				else{

					anglex = (100 * (anglex));
					distz = 80 * (1 * distz);
					angley = 0;
				}

				TheMarkers[j][0].x = TheMarkers[j][0].x;
				TheMarkers[j][0].y = TheMarkers[j][0].y + (anglex)+(distz);
				TheMarkers[j][1].x = TheMarkers[j][1].x;
				TheMarkers[j][1].y = TheMarkers[j][1].y + (anglex)+(distz);
				TheMarkers[j][2].x = TheMarkers[j][2].x;
				TheMarkers[j][2].y = TheMarkers[j][2].y + (anglex)+(distz);
				TheMarkers[j][3].x = TheMarkers[j][3].x;
				TheMarkers[j][3].y = TheMarkers[j][3].y + (anglex)+(distz);
			}

		}
		else if (TheMarkers[0].id == 839){

			if (abs(anglex) < abs(angley)){

				if (abs(TheMarkers[j].Rvec.ptr<float>(0)[0]) > 1 && abs(TheMarkers[j].Rvec.ptr<float>(0)[1])<3 && TheMarkers[j].Rvec.ptr<float>(0)[0] > 0){
					anglex = (280 * (-1 * anglex));
					distz = 60 * (1 - distz);
					angley = 0;
				}
				else{

					anglex = (280 * (anglex));
					distz = 60 * (1 * distz);
					angley = 0;
				}

				TheMarkers[j][0].x = TheMarkers[j][0].x + (anglex)-(distz);
				TheMarkers[j][0].y = TheMarkers[j][0].y + (angley);
				TheMarkers[j][1].x = TheMarkers[j][1].x + (anglex)-(distz);
				TheMarkers[j][1].y = TheMarkers[j][1].y + (angley);
				TheMarkers[j][2].x = TheMarkers[j][2].x + (anglex)-(distz);
				TheMarkers[j][2].y = TheMarkers[j][2].y + (angley);
				TheMarkers[j][3].x = TheMarkers[j][3].x + (anglex)-(distz);
				TheMarkers[j][3].y = TheMarkers[j][3].y + (angley);

			}
			else{

				if (abs(TheMarkers[j].Rvec.ptr<float>(0)[0]) > 1 && abs(TheMarkers[j].Rvec.ptr<float>(0)[1]) < 3 && TheMarkers[j].Rvec.ptr<float>(0)[0] < 0){
					anglex = (60 * (-1 * anglex));
					distz = 300 * (distz);
					angley = 0;
				}
				else{

					anglex = (60 * (1 - anglex));
					distz = 300 * (-1 * distz);
					angley = 0;
				}

				TheMarkers[j][0].x = TheMarkers[j][0].x;
				TheMarkers[j][0].y = TheMarkers[j][0].y - (anglex)-(distz);
				TheMarkers[j][1].x = TheMarkers[j][1].x;
				TheMarkers[j][1].y = TheMarkers[j][1].y - (anglex)-(distz);
				TheMarkers[j][2].x = TheMarkers[j][2].x;
				TheMarkers[j][2].y = TheMarkers[j][2].y - (anglex)-(distz);
				TheMarkers[j][3].x = TheMarkers[j][3].x;
				TheMarkers[j][3].y = TheMarkers[j][3].y - (anglex)-(distz);

			}

		}



		TheMarkers[j].id = 0;


		cv::Mat D2points(4, 2, CV_32FC1);;


		for (int c = 0; c < 4; c++)
		{
			D2points.at<float>(c, 0) = ((TheMarkers[j])[c].x);
			D2points.at<float>(c, 1) = ((TheMarkers[j])[c].y);

		}

		cv::Mat r1, t1;
		cv::solvePnP(D3Points, D2points, TheCameraParameters.CameraMatrix, TheCameraParameters.Distorsion, r1, t1);
		r1.convertTo(TheMarkers[j].Rvec, CV_32F);
		t1.convertTo(TheMarkers[j].Tvec, CV_32F);
		//aruco::Marker rotateXaxis(TheMarkers[j].Rvec);

		cv::Rodrigues(r1, TheMarkers[j].Rvec);
		TheMarkers[j].Rvec = TheMarkers[0].Rvec;
		std::vector<cv::Point2f> projectedPoints;
		cv::projectPoints(D3Points, TheMarkers[j].Rvec, TheMarkers[j].Tvec, TheCameraParameters.CameraMatrix, TheCameraParameters.Distorsion, projectedPoints);
		//std::cout << "rvec: " << TheMarkers[j].Rvec << std::endl;
		//std::cout << "tvec: " << TheMarkers[j].Tvec << std::endl;

		/*	for (unsigned int i = 0; i < projectedPoints.size(); ++i)
		{
		//std::cout  << " Projected to " << projectedPoints[i] << std::endl;
		}*/
	}


	if (TheMarkers.size() ==3){
		//TheMarkers.push_back(TheMarkers[0]);
		TheMarkers[j].id = 0;
		TheMarkers[j][0].x = middlePoint.x + diff.x;
		TheMarkers[j][0].y = middlePoint.y + diff.x;
		TheMarkers[j][1].x = middlePoint.x - diff.x;
		TheMarkers[j][1].y = middlePoint.y + diff.x;
		TheMarkers[j][2].x = middlePoint.x - diff.x;
		TheMarkers[j][2].y = middlePoint.y - diff.x;
		TheMarkers[j][3].x = middlePoint.x + diff.x;
		TheMarkers[j][3].y = middlePoint.y - diff.x;

		TheMarkers[j].Tvec = (TheMarkers[0].Tvec + TheMarkers[1].Tvec) / 2;
	}
	if (TheMarkers.size() != 0){
			//CvDrawingUtils::draw3dAxis(TheInputImageCopy, TheMarkers[j], TheCameraParameters);
		//cv::circle(TheInputImageCopy, center, 10, Scalar(255, 0, 0, 255), 1, CV_AA);
			// Para mandar mensajes de control escribe
		double correc=0;
		/*if (TheMarkers[j].Tvec.ptr<float>(0)[2] < 0.7){
			correc = (0.1*TheMarkers[j].Tvec.ptr<float>(0)[2]);
		}
		else{
			correc = (0.11*TheMarkers[j].Tvec.ptr<float>(0)[2]);
		}*/
			unsigned int camera = 1;
			double posX = TheMarkers[j].Tvec.ptr<float>(0)[0];
			double posY = TheMarkers[j].Tvec.ptr<float>(0)[1];
			double posZ = TheMarkers[j].Tvec.ptr<float>(0)[2]+correc;
			cout<<endl;
			cout << "POSX:" << posX <<"m"<< endl;
			cout << "POSY:" << posY << endl;
			cout << "POSZ:" << posZ << endl;
			
				
			//Change Rotation angles from Euler to Quaternions
			
			
			double phi = TheMarkers[j].Rvec.ptr<float>(0)[0];
			double theta = TheMarkers[j].Rvec.ptr<float>(0)[1];
			double psi = TheMarkers[j].Rvec.ptr<float>(0)[2];
			/*
			cout << "Phi:" << phi << endl;
			cout << "theta:" << theta << endl;
			cout << "psi:" << psi << endl;
			*/
			double x = -sin(phi / 2)*sin(theta / 2)*sin(psi / 2) + cos(phi / 2)*cos(theta / 2)*cos(psi / 2);;
			double y = sin(phi / 2)*cos(theta / 2)*cos(psi / 2) + cos(phi / 2)*sin(theta/ 2)*sin(psi / 2);;
			double z = -sin(phi / 2)*cos(theta / 2)*sin(psi / 2) + cos(phi / 2)*sin(theta / 2)*cos(psi / 2);;
			double w = sin(phi / 2)*sin(theta / 2)*cos(psi / 2) + cos(phi / 2)*cos(theta / 2)*sin(psi / 2);;
			
			cout << "x:" << x << endl;
			cout << "y:" << y << endl;
			cout << "z:" << z << endl;
			cout << "w:" << w << endl;

			unsigned int markerId = TheMarkers[j].id;
			
			cout << "markerId:" << markerId << endl;
			
			//Cominucación socket UDP
			comunication(camera,posX,posY, posZ, x, y, z, w, markerId);
			/*
			bool condition = true;
			msgSend msg;
			
			msg.camera = camera;
			msg.posX = posX;
			msg.posY = posY;
			msg.posZ = posZ;
			msg.x= x;
			msg.y = y;
			msg.z = z;
			msg.w = w;
			msg.markerId =markerId;
			
			float itime = 0.5;

			char *raw = new char[sizeof(msg)];
			memcpy(raw, &msg, sizeof(msg));

	//for(;;){
		int len = 0;
		if ((len = sendto(sockfd, raw, sizeof(msgSend), 0, (struct sockaddr*) &serv_addr, slen)) == -1){
			std::cout << "Fallo de envio" << std::endl;	
			closeUDP();
			
			

			
		}else{
			std::cout << "Sended " << len << " bytes" << std::endl;
			Sleep(itime);
		}*/
	//}
			
			//unsigned int heartBeat=10;
			std::stringstream ss;
			ss << camera << ","  << posX << "," << posY << "," << posZ << "," << x << "," << y << "," << z <<","<< w <<","<<markerId ;
			//Message commandMessage(0, MessageType::eMove, 0, ss.str());
			//queueMessage(commandMessage);

			//Write position in a .mat file

			Point3d Pos;
			Pos.x = posX;
			Pos.y = posY;
			Pos.z = posZ;

			//write txt file to matlab

			fstream f2;
			f2.open("positions.txt", fstream::out | ofstream::app);
			f2 << Pos * 100 << endl;
			f2.close();
			
		}
}


// This video stablisation smooths the global trajectory using a sliding average window

const int SMOOTHING_RADIUS = 30; // In frames. The larger the more stable the video, but less reactive to sudden panning
const int HORIZONTAL_BORDER_CROP = 5; // In pixels. Crops the border to reduce the black borders from stabilisation being too noticeable.

// 1. Get previous to current frame transformation (dx, dy, da) for all frames
// 2. Accumulate the transformations to get the image trajectory
// 3. Smooth out the trajectory using an averaging window
// 4. Generate new set of previous to current transform, such that the trajectory ends up being the same as the smoothed trajectory
// 5. Apply the new transformation to the video



struct TransformParam
{
	TransformParam() {}
	TransformParam(double _dx, double _dy, double _da) {
		dx = _dx;
		dy = _dy;
		da = _da;
	}

	double dx;
	double dy;
	double da; // angle
};

struct Trajectory
{
	Trajectory() {}
	Trajectory(double _x, double _y, double _a) {
		x = _x;
		y = _y;
		a = _a;
	}
	// "+"
	friend Trajectory operator+(const Trajectory &c1, const Trajectory  &c2){
		return Trajectory(c1.x + c2.x, c1.y + c2.y, c1.a + c2.a);
	}
	//"-"
	friend Trajectory operator-(const Trajectory &c1, const Trajectory  &c2){
		return Trajectory(c1.x - c2.x, c1.y - c2.y, c1.a - c2.a);
	}
	//"*"
	friend Trajectory operator*(const Trajectory &c1, const Trajectory  &c2){
		return Trajectory(c1.x*c2.x, c1.y*c2.y, c1.a*c2.a);
	}
	//"/"
	friend Trajectory operator/(const Trajectory &c1, const Trajectory  &c2){
		return Trajectory(c1.x / c2.x, c1.y / c2.y, c1.a / c2.a);
	}
	//"="
	Trajectory operator =(const Trajectory &rx){
		x = rx.x;
		y = rx.y;
		a = rx.a;
		return Trajectory(x, y, a);
	}

	double x;
	double y;
	double a; // angle
};

int k = 1;
Mat Stabilize(){

	
	
		// For further analysis
		ofstream out_transform("prev_to_cur_transformation.txt");
		ofstream out_trajectory("trajectory.txt");
		ofstream out_smoothed_trajectory("smoothed_trajectory.txt");
		ofstream out_new_transform("new_prev_to_cur_transformation.txt");

		//VideoCapture cap(0);
		//assert(cap.isOpened());

		Mat cur, cur_grey;
		Mat prev, prev_grey;
		
		//TheInputImage.copyTo(cap);//get the first frame.ch
		TheVideoCapturer>>(prev);
		cvtColor(prev, prev_grey, COLOR_BGR2GRAY);

		// Step 1 - Get previous to current frame transformation (dx, dy, da) for all frames
		vector <TransformParam> prev_to_cur_transform; // previous to current
		// Accumulated frame to frame transform
		double a = 0;
		double x = 0;
		double y = 0;
		// Step 2 - Accumulate the transformations to get the image trajectory
		vector <Trajectory> trajectory; // trajectory at all frames
		//
		// Step 3 - Smooth out the trajectory using an averaging window
		vector <Trajectory> smoothed_trajectory; // trajectory at all frames
		Trajectory X;//posteriori state estimate
		Trajectory	X_;//priori estimate
		Trajectory P;// posteriori estimate error covariance
		Trajectory P_;// priori estimate error covariance
		Trajectory K;//gain
		Trajectory	z;//actual measurement
		double pstd = 4e-3;//can be changed
		double cstd = 0.25;//can be changed
		Trajectory Q(pstd, pstd, pstd);// process noise covariance
		Trajectory R(cstd, cstd, cstd);// measurement noise covariance 
		// Step 4 - Generate new set of previous to current transform, such that the trajectory ends up being the same as the smoothed trajectory
		vector <TransformParam> new_prev_to_cur_transform;
		//
		// Step 5 - Apply the new transformation to the video
		//cap.set(CV_CAP_PROP_POS_FRAMES, 0);
		Mat T(2, 3, CV_64F);

		int vert_border = HORIZONTAL_BORDER_CROP * prev.rows / prev.cols; // get the aspect ratio correct
		VideoWriter outputVideo;
		outputVideo.open("compare.avi", CV_FOURCC('X', 'V', 'I', 'D'), 24, cvSize(cur.rows, cur.cols * 2 + 10), true);
		//
		//int k = 1;
		//int max_frames = cap.get(CV_CAP_PROP_FRAME_COUNT);
		int max_frames = TheVideoCapturer.get(CV_CAP_PROP_FRAME_COUNT);
		Mat last_T;
		Mat prev_grey_, cur_grey_;
		Mat cur2;
		int key = 0;
		//while (true) {

			TheVideoCapturer >> cur;
			//if (cur.data == NULL) {
				//break;
			//}

			cvtColor(cur, cur_grey, COLOR_BGR2GRAY);

			// vector from prev to cur
			vector <Point2f> prev_corner, cur_corner;
			vector <Point2f> prev_corner2, cur_corner2;
			vector <uchar> status;
			vector <float> err;

			goodFeaturesToTrack(prev_grey, prev_corner, 200, 0.01, 30);
			calcOpticalFlowPyrLK(prev_grey, cur_grey, prev_corner, cur_corner, status, err);

			// weed out bad matches
			for (size_t i = 0; i < status.size(); i++) {
				if (status[i]) {
					prev_corner2.push_back(prev_corner[i]);
					cur_corner2.push_back(cur_corner[i]);
				}
			}

			// translation + rotation only
			 T = estimateRigidTransform(prev_corner2, cur_corner2, false); // false = rigid transform, no scaling/shearing

			// in rare cases no transform is found. We'll just use the last known good transform.
			if (T.data == NULL) {
				last_T.copyTo(T);
			}

			T.copyTo(last_T);

			// decompose T
			double dx = T.at<double>(0, 2);
			double dy = T.at<double>(1, 2);
			double da = atan2(T.at<double>(1, 0), T.at<double>(0, 0));
			//
			//prev_to_cur_transform.push_back(TransformParam(dx, dy, da));

			out_transform << k << " " << dx << " " << dy << " " << da << endl;
			//
			// Accumulated frame to frame transform
			x += dx;
			y += dy;
			a += da;
			//trajectory.push_back(Trajectory(x,y,a));
			//
			out_trajectory << k << " " << x << " " << y << " " << a << endl;
			//
			z = Trajectory(x, y, a);
			//
			if (k == 1){
				// intial guesses
				X = Trajectory(0, 0, 0); //Initial estimate,  set 0
				P = Trajectory(1, 1, 1); //set error variance,set 1
			}
			else
			{
				//time update£¨prediction£©
				X_ = X; //X_(k) = X(k-1);
				P_ = P + Q; //P_(k) = P(k-1)+Q;
				// measurement update£¨correction£©
				K = P_ / (P_ + R); //gain;K(k) = P_(k)/( P_(k)+R );
				X = X_ + K*(z - X_); //z-X_ is residual,X(k) = X_(k)+K(k)*(z(k)-X_(k)); 
				P = (Trajectory(1, 1, 1) - K)*P_; //P(k) = (1-K(k))*P_(k);
			}
			//smoothed_trajectory.push_back(X);
			out_smoothed_trajectory << k << " " << X.x << " " << X.y << " " << X.a << endl;
			//-
			// target - current
			double diff_x = X.x - x;//
			double diff_y = X.y - y;
			double diff_a = X.a - a;

			dx = dx + diff_x;
			dy = dy + diff_y;
			da = da + diff_a;

			//new_prev_to_cur_transform.push_back(TransformParam(dx, dy, da));
			//
			out_new_transform << k << " " << dx << " " << dy << " " << da << endl;
			//
			T.at<double>(0, 0) = cos(da);
			T.at<double>(0, 1) = -sin(da);
			T.at<double>(1, 0) = sin(da);
			T.at<double>(1, 1) = cos(da);

			T.at<double>(0, 2) = dx;
			T.at<double>(1, 2) = dy;



			warpAffine(prev, cur2, T, cur.size());

			cur2 = cur2(Range(vert_border, cur2.rows - vert_border), Range(HORIZONTAL_BORDER_CROP, cur2.cols - HORIZONTAL_BORDER_CROP));

			// Resize cur2 back to cur size, for better side by side comparison
			resize(cur2, cur2, cur.size());

			// Now draw the original and stablised side by side for coolness
			Mat canvas = Mat::zeros(cur.rows, cur.cols * 2 + 10, cur.type());

			prev.copyTo(canvas(Range::all(), Range(0, cur2.cols)));
			cur2.copyTo(canvas(Range::all(), Range(cur2.cols + 10, cur2.cols * 2 + 10)));

			// If too big to fit on the screen, then scale it down by 2, hopefully it'll fit :)
			if (canvas.cols > 1920) {
				resize(canvas, canvas, Size(canvas.cols / 2, canvas.rows / 2));
			}
			//outputVideo<<canvas;
			Mat final = Mat::zeros(cur.rows, cur.cols * 2 + 10, cur.type());
			//simshow("before and after", cur2);

			waitKey(10);
			//
			prev = cur.clone();//cur.copyTo(prev);
			cur_grey.copyTo(prev_grey);

			//cout << "Frame: " << k << "/" << max_frames << " - good optical flow: " << prev_corner2.size() << endl;
			k++;

		//}
		return (cur2);

}
