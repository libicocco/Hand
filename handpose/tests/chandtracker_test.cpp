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
#define BOOST_TEST_MODULE CHANDTRACKER
#include <boost/test/unit_test.hpp>
#include "chandtracker.h"

static const cv::Scalar gSkinColor(20,50,110);

BOOST_AUTO_TEST_CASE(singleTracking)
{
	cv::Mat lTstIm(cv::Size(100,100),CV_8UC3,cv::Scalar(0,0,0));
	cv::Rect lTstBox(20,30,40,50); 
	lTstIm(lTstBox)=gSkinColor;
	CHandTracker lTstTracker;
	std::pair<cv::Mat,cv::Mat> lTstMaskCrop;
	cv::Rect lResBox = lTstTracker.getHand(lTstIm,lTstMaskCrop);
	//BOOST_TEST_MESSAGE("in rectangle: " << lTstBox.tl() << "," << lTstBox.br());
	//BOOST_TEST_MESSAGE("out rectangle: " << lResBox.tl() << "," << lResBox.br());
	BOOST_CHECK(lResBox == lTstBox);
	lTstBox+=cv::Point(10,15);
	lTstIm=cv::Scalar(0,0,0);
	lTstIm(lTstBox)=gSkinColor;
	lResBox = lTstTracker.getHand(lTstIm,lTstMaskCrop);
	BOOST_TEST_MESSAGE("checking rectangle");
	BOOST_CHECK(lResBox == lTstBox);
	//lTstMaskCrop.first(cv::Rect(1,2,3,4))=cv::Scalar(0,0,0); // checking that it actually fails
	//lTstMaskCrop.second(cv::Rect(1,2,3,4))=cv::Scalar(0);    // checking that it actually fails 
	std::vector<cv::Mat> lResPlanes,lExpPlanes;
	cv::split(lTstMaskCrop.first,lResPlanes);
	cv::Mat lExpIm(cv::Size(lTstBox.width,lTstBox.height),CV_32FC3,gSkinColor);
	cv::split(lExpIm,lExpPlanes);
	cv::Mat lCmp(cv::Size(lTstBox.width,lTstBox.height),CV_8UC1,cv::Scalar(255));
	cv::Scalar lDifference;
	for(int i=0;i<3;++i)
	{
		cv::compare(lExpPlanes[i],lResPlanes[i],lCmp,CV_CMP_NE);
		lDifference += cv::sum(lCmp);
	}
	BOOST_TEST_MESSAGE("checking output image matrix");
	BOOST_CHECK(lDifference==cv::Scalar(0));
	cv::Mat lResMask(cv::Size(lTstBox.width,lTstBox.height),CV_8UC1,cv::Scalar(255));
	cv::compare(lResMask,lTstMaskCrop.second,lCmp,CV_CMP_NE);
	lDifference = cv::sum(lCmp);
	BOOST_TEST_MESSAGE("checking mask image matrix");
	BOOST_CHECK(lDifference==cv::Scalar(0));
}

BOOST_AUTO_TEST_CASE(doubleTracking)
{
	cv::Mat lTstIm(cv::Size(100,100),CV_8UC3,cv::Scalar(0,0,0));
	cv::Rect lTstBoxBL(5,45,40,40), lTstBoxTR(55,5,40,40); 
	lTstIm(lTstBoxBL)=gSkinColor;
	lTstIm(lTstBoxTR)=gSkinColor;
	CHandTracker lTstTracker;
	std::pair<cv::Mat,cv::Mat> lTstMaskCrop;
	cv::Rect lResBox = lTstTracker.getHand(lTstIm,lTstMaskCrop);
	BOOST_TEST_MESSAGE("bl rectangle: " << lTstBoxBL.tl() << "," << lTstBoxBL.br() 
	<< "; tr rectangle: " << lTstBoxTR.tl() << "," << lTstBoxTR.br()
	<< "; res rectangle: " << lResBox.tl() << "," << lResBox.br());
	BOOST_CHECK(lResBox == lTstBoxBL);
	lTstBoxBL+=cv::Point(0,-25);
	lTstBoxTR+=cv::Point(0,25);
	lTstIm=cv::Scalar(0,0,0);
	lTstIm(lTstBoxBL)=gSkinColor;
	lTstIm(lTstBoxTR)=gSkinColor;
	lResBox = lTstTracker.getHand(lTstIm,lTstMaskCrop);
	BOOST_TEST_MESSAGE("bl rectangle: " << lTstBoxBL.tl() << "," << lTstBoxBL.br() 
	<< "; tr rectangle: " << lTstBoxTR.tl() << "," << lTstBoxTR.br()
	<< "; res rectangle: " << lResBox.tl() << "," << lResBox.br());
	BOOST_CHECK(lResBox == lTstBoxBL);
	lTstBoxBL+=cv::Point(0,-19);
	lTstBoxTR+=cv::Point(0,19);
	lTstIm=cv::Scalar(0,0,0);
	lTstIm(lTstBoxBL)=gSkinColor;
	lTstIm(lTstBoxTR)=gSkinColor;
	lResBox = lTstTracker.getHand(lTstIm,lTstMaskCrop);
	BOOST_TEST_MESSAGE("bl rectangle: " << lTstBoxBL.tl() << "," << lTstBoxBL.br() 
	<< "; tr rectangle: " << lTstBoxTR.tl() << "," << lTstBoxTR.br()
	<< "; res rectangle: " << lResBox.tl() << "," << lResBox.br());
	BOOST_CHECK(lResBox == lTstBoxBL);
	lTstBoxBL+=cv::Point(25,0);
	lTstBoxTR+=cv::Point(-25,0);
	lTstIm=cv::Scalar(0,0,0);
	lTstIm(lTstBoxBL)=gSkinColor;
	lTstIm(lTstBoxTR)=gSkinColor;
	lResBox = lTstTracker.getHand(lTstIm,lTstMaskCrop);
	BOOST_TEST_MESSAGE("bl rectangle: " << lTstBoxBL.tl() << "," << lTstBoxBL.br() 
	<< "; tr rectangle: " << lTstBoxTR.tl() << "," << lTstBoxTR.br()
	<< "; res rectangle: " << lResBox.tl() << "," << lResBox.br());
	BOOST_CHECK(lResBox == lTstBoxBL);
}