#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <fstream>
#include <iterator>

#include "feature.h"
#include "chandtracker.h"
#define private public /** <@attention I want to test private classes **/
#include "hog.h"
#undef private
#include "constants.h"

int main(int argc,char* argv[])
{
	std::cout << "starting" << std::endl;
	const int lSz(80);
	cv::Mat lMask(lSz,lSz,CV_8UC1,cv::Scalar(1));
	cv::Mat lIm(lSz,lSz,CV_8UC3,cv::Scalar(10,133,129));
	cv::Mat lRGB(lSz,lSz,CV_8UC3);
	cv::Mat lRGB32F(lSz,lSz,CV_32FC3);
	std::vector<cv::Mat> lHSVPlanes;
	cv::split(lIm,lHSVPlanes);
// 	lHSVPlanes[2].at<uchar>(9,9)+=cv::saturate_cast<uchar>(20);
	for(int y=40;y<59;y++)
	{
		for(int x=60;x<79;x++)
		{
			lHSVPlanes[2].at<uchar>(y,x)+=cv::saturate_cast<uchar>(x-40);
		}
	}
	cv::merge(lHSVPlanes,lIm);
	cv::cvtColor(lIm,lRGB,CV_HSV2RGB);
	lRGB.convertTo(lRGB32F,CV_32FC3);
	std::pair<cv::Mat,cv::Mat> lTstMaskCrop=std::make_pair(lRGB32F,lMask);
	std::cout << "computing" << std::endl;
	Hog<float> lTstHog;
	std::vector<float> lTstFloat = lTstHog.doCompute(lTstMaskCrop);
	for(int i=0;i<8;++i)
		std::cout << lTstFloat[i] << " ";
	std::cout << std::endl;
	for(int i=8;i<16;++i)
		std::cout << lTstFloat[i] << " ";
	std::cout << std::endl;
	for(int i=64;i<72;++i)
		std::cout << lTstFloat[i] << " ";
	std::cout << std::endl;
	for(int i=72;i<80;++i)
		std::cout << lTstFloat[i] << " ";
	std::cout << std::endl;
	std::cout << "done" << std::endl;
	
	std::ofstream lS("/home/javier/lTst.txt",std::iostream::out);
	std::copy(lTstFloat.begin(),lTstFloat.end(),std::ostream_iterator<float>(lS," "));
	lS.close();
	
	cv::imshow("image",lTstMaskCrop.first/250); // weird because it's 32FC3
	cv::imshow("mask",lTstMaskCrop.second);
	cv::Mat lTstDraw(cv::Size(200,200),CV_8UC1,cv::Scalar(255));
	lTstHog.draw(lTstDraw);
	cv::imshow("feature",lTstDraw);
	cv::waitKey();
	cv::imwrite("image.png",lTstMaskCrop.first);
	cv::imwrite("mask.png",lTstMaskCrop.second);
	cv::imwrite("hog.png",lTstDraw);
}