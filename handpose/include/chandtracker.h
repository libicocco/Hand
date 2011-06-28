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
#ifndef _CHANDTRACKER_H_
#define _CHANDTRACKER_H_
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "ctracker.h"
#include "ctrackable.h"
#include "typeDefinitions.h"

typedef std::pair<const unsigned int, CTrackable<tRect,tPoint> > tIntTrack;

static const int gMinSize(1000); //FIXME: I cannot use it in a lambda funtion if it's a member
// static const int gResetTrack(1);
static const int gResetTrack(20);

/**
 * @brief Class that tracks skin color blobs
 **/
class CHandTracker
{
	friend std::ostream& operator << 
	( std::ostream& output, const CHandTracker& pH)
	{output << pH.mTracker;return output;}
	
public:
	CHandTracker(cv::Scalar_<int> pMeanColor=cv::Scalar_<int>(10,133,129),
							 cv::Scalar_<int> pStdColor=cv::Scalar_<int>(10,123,127)):
	mTracker(),mLeftBottomTag(0),mFrame(0),mTrackedSet(),
	mTrackedHand(NULL),mMeanColor(pMeanColor),mStdColor(pStdColor){};
	
	/**
	 * @brief Track skin blobs and returns their masks and cropped image 
	 *
	 * @param pIm Input image
	 * @param pCropMask Output pair of cropped image and cropped mask
	 * @return :Rect Bounding box of the hand in the input image
	 **/
	cv::Rect getHand( const cv::Mat& pIm, tMatMat& pCropMask);
	void setMeanColor(const cv::Scalar &pMeanColor){mMeanColor=pMeanColor;};
	void setStdColor(const cv::Scalar &pStdColor){mStdColor=pStdColor;};
private:
	/**
	 * @brief Color thresholding in HSV
	 *
	 * @param pIm Input image
	 * @param pMask Output binary mask, where 1 means color detected
	 * @return void
	 **/
	void colorDetector(const cv::Mat &pIm, cv::Mat &pMask);
	
	/**
	 * @brief Generic tracker, used for the skin blobs in our case
	 **/
	CTracker<tRect,tPoint> mTracker;
	/**
	 * @brief Identifier of tracked blob, in our case the left bottom blob (right hand)
	 **/
	unsigned int mLeftBottomTag;
	/**
	 * @brief Frame number
	 **/
	unsigned int mFrame;
	/**
	 * @brief Set of identifiers and their blobs
	 **/
	std::map<unsigned int,CTrackable<tRect,tPoint> > mTrackedSet;
	/**
	 * @brief Pointer to the tracked hand
	 **/
	std::map<unsigned int,CTrackable<tRect,tPoint> >::iterator mTrackedHand;
	/**
	 * @brief Mean of expected color to be tracked
	 **/
	cv::Scalar_<int> mMeanColor;
	/**
	 * @brief Allowed range for the color to be tracked
	 **/
	cv::Scalar_<int> mStdColor;
};
#endif
