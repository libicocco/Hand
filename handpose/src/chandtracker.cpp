#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <algorithm>
#include <iostream>

#include "chandtracker.h"


bool TrackCMP(tIntTrack &p1,tIntTrack &p2)
{
	cv::Point br1 = p1.second.getReal().br();
	cv::Point br2 = p2.second.getReal().br();
	return (br1.y-br1.x)>(br2.y-br2.x);
}

void CHandTracker::colorDetector(const cv::Mat &pIm, cv::Mat &pMask)
{
	//cv::Mat lHSV(pIm.size(),CV_32FC3);
	cv::Mat lHSV(pIm.size(),CV_8UC3);
	
	cv::cvtColor(pIm,lHSV,CV_BGR2HSV);
	
	cv::Scalar_<int> lMeanColor180(mMeanColor);
	lMeanColor180.val[0]+=180;
	std::vector<cv::Mat> lPlanes,lPlanes180;
	cv::split(cv::abs(lHSV-mMeanColor),lPlanes);
	cv::split(cv::abs(lHSV-lMeanColor180),lPlanes180);
	pMask=((lPlanes[0]<mStdColor.val[0])|(lPlanes180[0]<mStdColor.val[0]))&
	      ((lPlanes[1]<mStdColor.val[1])|(lPlanes180[1]<mStdColor.val[1]))&
	      ((lPlanes[2]<mStdColor.val[2])|(lPlanes180[2]<mStdColor.val[2]));
	cv::morphologyEx(pMask,pMask,cv::MORPH_CLOSE,cv::getStructuringElement(cv::MORPH_RECT,cv::Size(5,5)));
}

cv::Rect CHandTracker::getHand( const cv::Mat& pIm, tMatMat& pCropMask)
{
	mFrame++;
	cv::Mat lMask(pIm.size(),CV_8UC1);
	colorDetector(pIm,lMask);
	
	// find contours; it give us also the connected components
	std::vector<std::vector<cv::Point> > lContours;
	{
		cv::Mat lTmp = lMask.clone();
		cv::findContours(lTmp, lContours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
	}
	std::vector<cv::Rect> lBoundBoxes;
	for(auto it=lContours.rbegin();it!=lContours.rend();++it)
		lBoundBoxes.push_back(cv::boundingRect(cv::Mat(*it)));
	
	// remove if the rectangle area is too small
	lBoundBoxes.erase( std::remove_if( lBoundBoxes.begin(),lBoundBoxes.end(),
																		 [](const cv::Rect &lR){return lR.width*lR.height<gMinSize;}),
										 lBoundBoxes.end());
	
	mTrackedSet = mTracker.Identify(lBoundBoxes);
	mTrackedHand = mTrackedSet.find(mLeftBottomTag);
	if(mFrame%gResetTrack==1 || mTrackedHand==mTrackedSet.end())
	{
 		mLeftBottomTag = min_element(mTrackedSet.begin(),mTrackedSet.end(),&TrackCMP)->first;
		mTrackedHand = mTrackedSet.find(mLeftBottomTag);
	}
	if (mTrackedHand == mTrackedSet.end())
	{
		std::cout << "No hand found" << std::endl;
		throw NOHANDERROR;
	}
	cv::Rect lHandBox = (mTrackedHand->second).getReal();
	//pIm(lHandBox).copyTo(pCropMask.first);
	pIm(lHandBox).convertTo(pCropMask.first,CV_32FC3); // we'll need a gray level 32F, so we better convert to 32F now
	lMask(lHandBox).copyTo(pCropMask.second);
	
	// All values are clamped to 255 or 0
	pCropMask.second = (pCropMask.second>0);
	//cv::compare( pCropMask.second, 0, pCropMask.second, CV_CMP_GT);
	
	return lHandBox;
}
