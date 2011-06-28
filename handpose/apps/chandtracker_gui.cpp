#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "chandtracker.h"

CHandTracker gTstTracker;

void setStdH(int pStdH,void* pStdColor){gTstTracker.setStdColor(*(reinterpret_cast<cv::Scalar_<int>*>(pStdColor)));}
void setMeanH(int pMeanH,void* pMeanColor){gTstTracker.setMeanColor(*(reinterpret_cast<cv::Scalar_<int>*>(pMeanColor)));}

int main(int argc,char* argv[])
{
	cv::Scalar_<int> lMeanColor(0,128,128);
	cv::Scalar_<int> lStdColor(250,228,228);
	cv::Mat lTstIm;
	cv::namedWindow("image");
	cv::namedWindow("mask");
	cv::createTrackbar("stdH", "image",&(lStdColor.val[0]),256,&setStdH,reinterpret_cast<void*>(&lStdColor));
	cv::createTrackbar("stdS", "image",&(lStdColor.val[1]),256,&setStdH,reinterpret_cast<void*>(&lStdColor));
	cv::createTrackbar("stdV", "image",&(lStdColor.val[2]),256,&setStdH,reinterpret_cast<void*>(&lStdColor));
	cv::createTrackbar("meanH", "image",&(lMeanColor.val[0]),256,&setMeanH,reinterpret_cast<void*>(&lMeanColor));
	cv::createTrackbar("meanS", "image",&(lMeanColor.val[1]),256,&setMeanH,reinterpret_cast<void*>(&lMeanColor));
	cv::createTrackbar("meanV", "image",&(lMeanColor.val[2]),256,&setMeanH,reinterpret_cast<void*>(&lMeanColor));
	
	cv::VideoCapture lCap;
	
	if(argc==1)
	{
		lCap.open(0);
		if(!lCap.isOpened())
		{
			std::cerr<<"error opening device 0"<<std::endl;
			exit(-1);
		}
	}
	
	for(int i=1;;++i)
	{
		if(argc==1)
			lCap >> lTstIm;
		else
		{
// 			gTstTracker.setMeanColor(cv::Scalar_<int>(-5,74,74));
// 			gTstTracker.setStdColor(cv::Scalar_<int>(30,74,74));
			lTstIm=cv::imread(argv[i]);
		}
		std::pair<cv::Mat,cv::Mat> lTstMaskCrop;
		cv::Rect lResBox = gTstTracker.getHand(lTstIm,lTstMaskCrop);
		cv::rectangle(lTstIm,lResBox.tl(),lResBox.br(),cv::Scalar(255,255,255));
		cv::imshow("image",lTstIm);
		cv::imshow("mask",lTstMaskCrop.second);
		cv::waitKey(5);
	}
}