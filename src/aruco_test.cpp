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
#include <opencv2\highgui\highgui.hpp>
#include "drone/Ardrone.h"
#include "core/time/TimeTools.h"
using namespace cv;
using namespace aruco;
using namespace ardronepp;

string TheInputVideo;
string TheIntrinsicFile;
float TheMarkerSize=-1;
int ThePyrDownLevel;
MarkerDetector MDetector;
VideoCapture TheVideoCapturer;
Ardrone drone;
vector<Marker> TheMarkers;
Mat TheInputImage,TheInputImageCopy;
CameraParameters TheCameraParameters;
void cvTackBarEvents(int pos,void*);
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
bool readArguments ( int argc,char **argv )
{
    if (argc<2) {
        cerr<<"Invalid number of arguments"<<endl;
        cerr<<"Usage: (in.jpg|in.avi|live[:idx_cam=0]) [intrinsics.yml] [size]"<<endl;
        return false;
    }
    TheInputVideo=argv[1];
     if (argc>=3)
         TheIntrinsicFile=argv[2];
    if (argc>=4)
        TheMarkerSize=atof(argv[3]);

    if (argc==3)
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
		//if (TheInputVideo.find(".avi") != string::npos){
				TheVideoCapturer.open(TheInputVideo);
        //check video is open
				if (!TheVideoCapturer.isOpened()) {
					cerr << "Could not open video" << endl;
					return -1;
				//}
        }
		//else TheInputImage=cv::imread(TheInputVideo);
				
        //read first image to get the dimensions
        TheVideoCapturer>>TheInputImage;

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
        int index=0;
        //capture until press ESC or until the end of the video
		do
		{
			TheVideoCapturer.retrieve(TheInputImage);
			//copy image

			index++; //number of images captured
			double tick = (double)getTickCount();//for checking the speed
			//Detection of markers in the image passed
			MDetector.detect(TheInputImage, TheMarkers, TheCameraParameters, TheMarkerSize);
			//cout << "Size of Markers " << TheMarkers.size() << endl;
			//chekc the speed by calculating the mean speed of all iterations
			AvrgTime.first += ((double)getTickCount() - tick) / getTickFrequency();
			AvrgTime.second++;
			//cout<<"\rTime detection="<<1000*AvrgTime.first/AvrgTime.second<<" milliseconds nmarkers="<<TheMarkers.size()<< std::flush;

			//print marker info and draw the markers in image

			TheInputImage.copyTo(TheInputImageCopy);
			float Tcx,Tcy;
			if (TheMarkers.size() > 1){
			
				Point2f point1 = TheMarkers[0].getCenter();
				Point2f point2 = TheMarkers[1].getCenter();
				Point2f middlePoint;
				middlePoint.x = (point1.x+ point2.x) / 2;
				middlePoint.y = (point1.y + point2.y) / 2;
				cv::circle(TheInputImageCopy, middlePoint, 10, Scalar(255, 0, 0, 255), 1, CV_AA);
				cv::line(TheInputImageCopy, point1, point2, Scalar(255, 0, 0, 255), 1, CV_AA);
				float Centerx = abs(point1.x-point2.x);
				float Centery = abs(point1.y - point2.y);
				Tcx = Centerx;
				Tcy = Centery;
				
				cout << endl;
				
			}
			cout << "Tcx:" << Tcx << endl;
			if (TheMarkers.size()!=0)      
				cout<<endl;


			//cout << "size of candidates" << MDetector.getCandidates().size()<< endl;

            //print other rectangles that contains no valid markers
            /*for (unsigned int i=0;i<MDetector.getCandidates().size();i++) {
                aruco::Marker m( MDetector.getCandidates()[i],999);
                m.draw(TheInputImageCopy,cv::Scalar(255,0,0));
            }*/
			
			int j=0;
			float Tx,Ty,Tz;

            //draw a 3d cube in each marker if there is 3d info
            if (  TheCameraParameters.isValid())
                for (unsigned int i=0;i<TheMarkers.size();i++) {



                    CvDrawingUtils::draw3dCube(TheInputImageCopy,TheMarkers[i],TheCameraParameters);
					
                    CvDrawingUtils::draw3dAxis(TheInputImageCopy,TheMarkers[i],TheCameraParameters);
					if (i < TheMarkers.size())
						j = i+1;
                }

				if (TheMarkers.size() != 0){
					TheMarkers.push_back(TheMarkers[0]);
					TheMarkers[j].id = 0;
					TheMarkers[j][0].x = TheMarkers[j][0].x + Tcx;
					TheMarkers[j][0].y = Tcy;

					float Rangosupx = 0.1, Rangoinfx = -0.1;
					float Rangosupy = 0.1, Rangoinfy = -0.1;
					float Rangosupz = 0.4, Rangoinfz = 0.1;

					Tx = TheMarkers[j].Tvec.at<float>(0, 0);
					Ty = TheMarkers[j].Tvec.at<float>(1, 0);
					Tz = TheMarkers[j].Tvec.at<float>(2, 0);
					
					double t0 = STime::get()->getTime();

					if (Tx<Rangosupx)
						while (STime::get()->getTime() - t0 < 1){
							drone.translate(0.0f, 0.2f);
					}
					else if (Tx>Rangoinfx)
						while (STime::get()->getTime() - t0 < 1){
							drone.translate(0.0f, -0.2f);
					}
					if (Tz<Rangosupz) 
					while (STime::get()->getTime() - t0 < 1){
						drone.translate(-0.1f, 0.0f);
					}
					else if (Tz>Rangoinfz)
					while (STime::get()->getTime() - t0 < 1){
						drone.translate(0.1f, 0.0f);
					}
					//cout << "Traslacionx:" <<  << endl;
					//cout << to << endl;
					CvDrawingUtils::draw3dAxis(TheInputImageCopy, TheMarkers[j], TheCameraParameters);
					
					
				}


            //DONE! Easy, right?
            //show input with augmented information and  the thresholded image
            cv::imshow("in",TheInputImageCopy);
			
           // cv::imshow("thres",MDetector.getThresholdedImage());

            key=cv::waitKey(waitTime);//wait for key to be pressed
			TheMarkers.clear();
			
        }while(key!=27 && TheVideoCapturer.grab());
		
    } catch (std::exception &ex)

    {
        cout<<"Exception :"<<ex.what()<<endl;
    }
	
}
/************************************
 
 
 
 
 ************************************/

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
            //CvDrawingUtils::draw3dCube(TheInputImageCopy,TheMarkers[i],TheCameraParameters);

    cv::imshow("in",TheInputImageCopy);
    cv::imshow("thres",MDetector.getThresholdedImage());

}

