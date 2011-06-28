/***********************************************
This file is part of the Hand project (https://github.com/libicocco/Hand).

    Copyright(c) 2007-2011 Javier Romero
    * jrgn AT kth DOT se

    Hand is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    Hand is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with Hand.  If not, see <http://www.gnu.org/licenses/>.

**********************************************/
#define BOOST_TEST_DYN_LINK
#define BOOST_TEST_MODULE HOG
#include <boost/test/unit_test.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <fstream>

#include "feature.h"
#include "chandtracker.h"
#define private public /** <@attention I want to test private classes **/
#include "hog.h"
#undef private 
#include "constants.h"

static const cv::Scalar gSkinColor(20,50,110);

BOOST_AUTO_TEST_CASE(gradientTest)
{
	int lNRows=3;
	cv::Size lSz(lNRows,lNRows);
	cv::Mat lTstIm(lSz,CV_32FC1,cv::Scalar(0));
	cv::Mat lTstMask(lSz,CV_8UC1,cv::Scalar(255));
	cv::Mat lTstOri(lSz,CV_32FC1,cv::Scalar(255));
	cv::Mat lExpOri(lSz,CV_32FC1,cv::Scalar(90));
	cv::Mat lCmp(lSz,CV_8UC1,cv::Scalar(1));
	
	for(int y=0;y<lTstIm.rows;y++)
		for(int x=0;x<lTstIm.cols;x++)
			lTstIm.at<float>(y,x)=static_cast<float>(7.0*y);
		
	Hog<float> lTstHog;
	lTstHog.calcGradient(lTstIm,lTstMask,lTstOri);
	cv::compare(lTstOri,lExpOri,lCmp,CV_CMP_NE);
	cv::Scalar lDifference = cv::sum(lCmp);
	
	BOOST_TEST_MESSAGE("checking vertical gradient");
	BOOST_CHECK(lDifference == cv::Scalar(0));
	
	cv::Mat lExpMask(lSz,CV_8UC1,cv::Scalar(255));
	cv::compare(lExpMask,lTstMask,lCmp,CV_CMP_NE);
	lDifference = cv::sum(lCmp);
	
	BOOST_TEST_MESSAGE("checking correct mask (1)");
	BOOST_CHECK(lDifference == cv::Scalar(0));
	
	// the minimum gradient is related to the threshold in the call 
	// myCalcMotionGradient(pGray, lOriOKmask, pOri, 1, 10000, 3 );
	for(int y=0;y<lTstIm.rows;y++)
		for(int x=0;x<lTstIm.cols;x++)
			lTstIm.at<float>(y,x)=static_cast<float>(0.49*y); 
	
	lTstHog.calcGradient(lTstIm,lTstMask,lTstOri);
	lExpMask = cv::Scalar(0); 
	cv::compare(lExpMask,lTstMask,lCmp,CV_CMP_NE);
	lDifference = cv::sum(lCmp);
	
	BOOST_TEST_MESSAGE("checking correct mask (0)");
	BOOST_CHECK(lDifference == cv::Scalar(0));
	
	for(int y=0;y<lTstIm.rows;y++)
		for(int x=0;x<lTstIm.cols;x++)
			lTstIm.at<float>(y,x) = 5*x;
		
	lExpOri=cv::Scalar(0);
	lTstHog.calcGradient(lTstIm,lTstMask,lTstOri);
	cv::compare(lTstOri,lExpOri,lCmp,CV_CMP_NE);
	lDifference = cv::sum(lCmp);
	
// 	cv::MatIterator_<float> it = lTstOri.begin<float>(),
// 	it_end = lTstOri.end<float>();
// 	for(; it != it_end; ++it)
// 		BOOST_TEST_MESSAGE(*it << " ");
	
	BOOST_TEST_MESSAGE("checking horizontal gradient");
	BOOST_CHECK(lDifference == cv::Scalar(0));
	
	for(int y=0;y<lTstIm.rows;y++)
		for(int x=0;x<lTstIm.cols;x++)
			lTstIm.at<float>(y,x) = 2*(x+y);
		
	cv::Rect lTstRect=cv::Rect(1,1,lNRows-2,lNRows-2);
	lExpOri=cv::Scalar(45);
	lTstHog.calcGradient(lTstIm,lTstMask,lTstOri);
	cv::compare(lTstOri,lExpOri,lCmp,CV_CMP_NE);
	// don't take into account boundaries
	lDifference = cv::sum(abs(lTstOri(lTstRect)-lExpOri(lTstRect)));
	double lDiff = lDifference(0)/((lNRows-2)*(lNRows-2));
	
	
	BOOST_TEST_MESSAGE("checking 45-diagonal gradient");
	BOOST_CHECK(lDiff<0.25); // the result is 44.7623
	
	for(int y=0;y<lTstIm.rows;y++)
		for(int x=0;x<lTstIm.cols;x++)
			lTstIm.at<float>(y,x) = 2*(-x+y);
		
	lExpOri=cv::Scalar(135);
	lTstHog.calcGradient(lTstIm,lTstMask,lTstOri);
	cv::compare(lTstOri,lExpOri,lCmp,CV_CMP_NE);
	// don't take into account boundaries
	lDifference = cv::sum(abs(lTstOri(lTstRect)-lExpOri(lTstRect)));
	lDiff = lDifference(0)/((lNRows-2)*(lNRows-2));
	cv::Mat lExpOriCrop=lExpOri(lTstRect);
	cv::MatIterator_<float> it = lExpOriCrop.begin<float>(),it_end =lExpOriCrop.end<float>();
	
	BOOST_TEST_MESSAGE("checking 135-diagonal gradient");
	BOOST_CHECK(lDiff<0.25);
}

BOOST_AUTO_TEST_CASE(histTest)
{
	Hog<float> lTstHog(4,8,false,false);// we try not-normalized holes here
	int lNRows=50;
	double lAngle=10.0;
	cv::Size lSz(lNRows,lNRows);
	cv::Mat lTstOri(lSz,CV_32FC1,cv::Scalar(lAngle));
	cv::Mat lTstMask(lSz,CV_8UC1,cv::Scalar(255));
	const int lNcuts = pow(2.0,lTstHog.mNLevels-1);
	
	cv::MatND *lHists = new cv::MatND[lTstHog.mNHists];
	int lSizes[1]={2*lTstHog.mNBins};
	cv::MatND lExpHist(1,lSizes,CV_32FC1,cv::Scalar(0));
	lExpHist.at<float>(static_cast<int>(floor(lAngle*lTstHog.mNBins/180.0)))=1;
	cv::Scalar lDifference(0);
	
	lTstHog.calcCellsHist(lTstOri, lTstMask,
												lTstHog.mMultilevel?lTstHog.mNLevels:1, lHists);
	
	BOOST_TEST_MESSAGE("checking constant histogram");
	BOOST_CHECK(lDifference == cv::Scalar(0));
	
	lDifference = cv::Scalar(0);
	//lTstMask(cv::Rect(0,0,1,1))=cv::Scalar(0);
	lTstMask(cv::Rect(0,0,lNRows/lNcuts,lNRows/lNcuts))=cv::Scalar(0);
	cv::MatND lExp0Hist(1,lSizes,CV_32FC1,cv::Scalar(0));
	lTstHog.calcCellsHist(lTstOri, lTstMask,
												lTstHog.mMultilevel?lTstHog.mNLevels:1, lHists);
	
	for(int c=0;c<lTstHog.mNHists;++c)
			lDifference += cv::sum(abs(lHists[c]-(c?lExpHist:lExp0Hist)));
	
	BOOST_TEST_MESSAGE("checking masked histogram");
	BOOST_CHECK(lDifference == cv::Scalar(0));
	
	Hog<float> lTstHogNormHole(4,8,false,true);
	lDifference = cv::Scalar(0);
	lTstMask = cv::Scalar(255);
	lTstMask(cv::Rect(0,0,lNRows/lNcuts,lNRows/(2*lNcuts)))=cv::Scalar(0);
	cv::MatND lExp05Hist(1,lSizes,CV_32FC1,cv::Scalar(0));
	lExp05Hist.at<float>(0)=0.5;
	
	lTstHogNormHole.calcCellsHist(lTstOri, lTstMask,
												lTstHogNormHole.mMultilevel?lTstHogNormHole.mNLevels:1, lHists);
	
	for(int c=0;c<lTstHog.mNHists;++c)
	{
		lDifference += cv::sum(abs(lHists[c]-(c?lExpHist:lExp05Hist)));
	}
	
	BOOST_TEST_MESSAGE("checking normalized holes histogram");
	BOOST_CHECK(lDifference == cv::Scalar(0));
	
	// FIXME: multilevel hists UNTESTED!!!!!
// 	double lAngle2=30.0;
// 	
// 	Hog<float> lTstHogMulti(4,8,true,false);
// 	const int lNcutsMulti = pow(2.0,lTstHogMulti.mNLevels-1);
// 	unsigned lRealBinsNumber=2*lTstHogMulti.mNBins;
// 	lDifference = cv::Scalar(0);
// 	lTstMask = cv::Scalar(255);
// 	lTstOri(cv::Rect(0,0,lNRows,lNRows/4))=cv::Scalar(lAngle2);
// 	cv::MatND *lHistsMulti = new cv::MatND[lTstHogMulti.mNHists];
// 	
// 	int lSizesMulti[1]={lRealBinsNumber};
// 	cv::MatND lExp2Hist(1,lSizesMulti,CV_32FC1,cv::Scalar(0));
// 	lExp2Hist.at<float>(1)=1;
// 	cv::MatND lExp3Hist(1,lSizesMulti,CV_32FC1,cv::Scalar(0));
// 	lExp3Hist.at<float>(0)=0.5;
// 	lExp3Hist.at<float>(1)=0.5;
// 	cv::MatND lExp4Hist(1,lSizesMulti,CV_32FC1,cv::Scalar(0));
// 	lExp4Hist.at<float>(0)=0.75;
// 	lExp4Hist.at<float>(1)=0.25;
// 	
// 	lTstHogNormHole.calcCellsHist(lTstOri, lTstMask,
// 												lTstHogMulti.mMultilevel?lTstHogMulti.mNLevels:1, lHistsMulti);
// 	
// 	for(int cy=0;cy<lNcutsMulti;++cy)
// 	{
// 		for(int cx=0;cx<lNcutsMulti;++cx)
// 		{
// 			std::cout << cy << " " << cx << std::endl;
//  			cv::MatIterator_<float> it = lHistsMulti[cy*lNcutsMulti+cx].begin<float>();
//  			cv::MatIterator_<float> it_end = lHistsMulti[cy*lNcutsMulti+cx].end<float>();
//  			for(; it != it_end; ++it)
//  				std::cout << *it << " ";
//  			std::cout << std::endl;
// 		}
// 	}
// 	
// 	for(int cy=0;cy<lNcutsMulti;++cy)
// 	{
// 		for(int cx=0;cx<lNcutsMulti;++cx)
// 		{
// 			if(cx<2)
// 				lDifference += cv::sum(abs(lHistsMulti[cy*lNcutsMulti+cx]-lExp2Hist));
// 			else
// 				lDifference += cv::sum(abs(lHistsMulti[cy*lNcutsMulti+cx]-lExpHist));
// 		}
// 	}
// 	std::cout << lDifference[0] << std::endl;
// 	for(int cy=0;cy<4;++cy)
// 		for(int cx=0;cx<4;++cx)
// 			lDifference += cv::sum(abs(lHistsMulti[64+cy*4+cx]-(cx?lExpHist:lExp2Hist)));
// 	std::cout << lDifference[0] << std::endl;
// 	for(int cy=0;cy<2;++cy)
// 		for(int cx=0;cx<2;++cx)
// 			lDifference += cv::sum(abs(lHistsMulti[80+cy*2+cx]-(cx?lExpHist:lExp3Hist)));
// 	std::cout << lDifference[0] << std::endl;
// 		
// 	lDifference += cv::sum(abs(lHistsMulti[84]-lExp4Hist));
// 	std::cout << lDifference[0] << std::endl;
// 	
// 	
// 	BOOST_TEST_MESSAGE("checking multilevel histogram");
// 	BOOST_CHECK(lDifference == cv::Scalar(0));
	
	delete []lHists;
// 	delete []lHistsMulti;
}

BOOST_AUTO_TEST_CASE(completeTest)
{
	const int lSz(80);
	Hog<float> lTstHog;
	const int lNcuts = pow(2.0,lTstHog.mNLevels-1);
	const int lCutWidth=lSz/lNcuts;
	const int lBinX(6),lBinY(4); 
	
	cv::Mat lMask(lSz,lSz,CV_8UC1,cv::Scalar(255));
	cv::Mat lIm(lSz,lSz,CV_8UC3,cv::Scalar(10,133,129));
	cv::Mat lRGB(lSz,lSz,CV_8UC3);
	cv::Mat lRGB32F(lSz,lSz,CV_32FC3);
	std::vector<cv::Mat> lHSVPlanes;
	cv::split(lIm,lHSVPlanes);
	
	// bins in xumn 4-5 and ys 6-7 will have a gradient in x
	for(int y=lBinY*lCutWidth;y<(lBinY+1)*lCutWidth;y++)
		for(int x=lBinX*lCutWidth;x<(lBinX+1)*lCutWidth;x++)
			lHSVPlanes[2].at<uchar>(y,x)+=cv::saturate_cast<uchar>(x-(lBinX*lCutWidth));
		
	// doCompute (or compute from features) need an RGB 32FC3 image
	cv::merge(lHSVPlanes,lIm);
	cv::cvtColor(lIm,lRGB,CV_HSV2RGB);
	lRGB.convertTo(lRGB32F,CV_32FC3);
	std::pair<cv::Mat,cv::Mat> lTstMaskCrop=std::make_pair(lRGB32F,lMask);
	
	double lDifference=0;
	// I'm calling doCompute (private), but using compute from features.h should be the same
	std::vector<float> lTstFloat = lTstHog.doCompute(lTstMaskCrop);
	// hog stored xumnwise
	// We should allow for some margin (0.8-0.15) because around the pattern
	// the gradients are not horizontal
	for(int x=0;x<lNcuts;++x)
		for(int y=0;y<lNcuts;++y)
			if((x==lBinX)&&(y==lBinY))
				lDifference+=(lTstFloat[(x*lNcuts+y)*lTstHog.mNBins]<0.6);// the first bin,hztal gradient
			else
				for(int b=0;b<8;++b)
					lDifference+=(lTstFloat[(x*lNcuts+y)*lTstHog.mNBins+b]>0.15);
					
	BOOST_CHECK(fabs(lDifference)<0.1);
	
//  	cv::Mat lTstDraw(cv::Size(200,200),CV_8UC1,cv::Scalar(255));
//  	lTstHog.draw(lTstDraw);
//  	cv::imshow("feature",lTstDraw);
//  	cv::waitKey();
}