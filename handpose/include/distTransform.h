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
/*
 * =====================================================================================
 * 
 *       Filename:  silhouette.h
 * 
 *    Description:  DistTransform as a bitmap
 * 
 *        Version:  1.0
 *        Created:  06/02/10 12:29:52 CEST
 *       Revision:  none
 *       Compiler:  gcc
 * 
 *         Author:  Javier Romero (jrgn), jrgn@kth.se
 *        Company:  CAS/CSC KTH
 * 
 * =====================================================================================
 */

#ifndef DISTTRANSFORMH
#define DISTTRANSFORMH

//#include <opencv/cv.h>
//#include <opencv/highgui.h>
#include "opencv2/core/core.hpp"
#include <vector>
#include <iostream>
#include "constants.h"
#include "feature.h"

#include <algorithm>
#include <stdint.h>
#include <math.h>
#include <vector>
#include <stdlib.h>
#include <string>
#include <iomanip>
#include <iostream>
#include <opencv2/highgui/highgui_c.h>
#include <opencv2/imgproc/imgproc.hpp>
//#include <opencv/highgui.h>
#include <cmath>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/progress.hpp>
#include <boost/format.hpp>
#include <buola/image.h>
#include <buola/image/image_algo.h>
#include <buola/cv/opencv.h>
#include "utils.h"
#include "bb.h"
#include "distTransform.h"
/** 
 * @brief Class for computing hogs, given different types of inputs
 */
template<typename tType>
class DistTransform: public Feature<tType>
{
    private: 
        cv::Mat mSil;
    private:
        const std::vector<tType>& doCompute()
        {
            CvSize lSz = cvGetSize(this->mGray);
//            IplImage *lEdge = cvCreateImage( lSz, IPL_DEPTH_8U , 1 );
            cv::Mat lEdgeMat;
//            IplImage *lDistT = cvCreateImage( lSz, IPL_DEPTH_32F , 1 );
//            cvCanny( this->mGray, lEdge, 100, 200);
            cv::Canny( this->mGray, lEdgeMat, 100, 200);
//            cv::Mat lEdgeMat = cv::Mat(lEdge);
            cv::threshold(lEdgeMat,lEdgeMat,2,255,cv::THRESH_BINARY_INV);
//            cv::imwrite("edge.png",lEdgeMat);
            cv::Mat lDistTMat;// = cv::Mat(lDistT);
            //cv::distanceTransform( lEdgeMat, lDistTMat, CV_DIST_L2, 3);
            //cv::GaussianBlur(lEdge,lDistTMat,cvSize(5,5),5);
            cv::distanceTransform( lEdgeMat, lDistTMat, CV_DIST_L2, 3);

            // the distance should be normalized depending on the image size,
            // to be able to compare images with different sizes
            // It's not perfect size we don't know if the distance which represent
            // the pixel is vcal or htal; therefore we do the average
            lDistTMat = lDistTMat/(sqrt(lDistTMat.rows*lDistTMat.cols));
            lDistTMat.convertTo(lDistTMat,IPL_DEPTH_8U,255.0);
            cv::resize(lDistTMat,mSil,cvSize(NCELLS,NCELLS));

            this->mFeat.clear();// isn't it faster to overwrite?
            for(int i = 0; i < mSil.rows; i++)
            {
                for(int j = 0; j < mSil.cols; j++)
                {
                    tType lij = (tType)mSil.at<unsigned char>(i,j);
                    this->mFeat.push_back(lij);
//                                std::cout << lij << " \t";
                }
            }
//            std::cout << std::endl;
            //    char lPath[50];
            //    sprintf(lPath,"out/graybefore%04u.jpg",gFRAME);
            //    cv::imwrite(lPath,mSil);
            return this->mFeat;
        }
    public:
        DistTransform():Feature<tType>(),mSil(NCELLS,NCELLS,IPL_DEPTH_32F){}
        void draw(boost::gil::gray8_image_t &pFeatIm,char* pFeatPath=NULL) const
        {
            //    pFeatIm.recreate(NCELLS*50,NCELLS*50,boost::gil::gray32f_pixel_t(0.0),NCELLS*50);
            //    IplImage* pDist_ipl = buola::cvi::ipl_wrap(view(pFeatIm));
            //    cvResize(mSil,pDist_ipl);
            pFeatIm.recreate(NCELLS,NCELLS,boost::gil::gray8_pixel_t(255),NCELLS);
            boost::gil::copy_pixels(buola::cvi::gil_wrap<boost::gil::gray8_view_t>(mSil),view(pFeatIm));
        }
};

#endif
