/*
 * =====================================================================================
 * 
 *       Filename:  silhouette.h
 * 
 *    Description:  Edge as a bitmap
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

#ifndef EDGEH
#define EDGEH

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
#include <cmath>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/progress.hpp>
#include <boost/format.hpp>
#include <buola/image.h>
#include <buola/image/image_algo.h>
#include <buola/cv/opencv.h>
#include "utils.h"
#include "bb.h"
/** 
 * @brief Class for computing hogs, given different types of inputs
 */

bool comppoint (cv::Point p1,cv::Point p2) { return (p1.y<p2.y || (p1.y==p2.y && p1.x<p2.x)); }
bool comppair2 (std::pair<int,float> p1,std::pair<int,float> p2) { return (p1.second<p2.second || (p1.second==p2.second && p1.first<p2.first)); } // FIXME: if i define comppair, it's redefined; if not it's missing
typedef std::pair<unsigned short,unsigned short> tPairUShort;
class Edge: public Feature<tPairUShort>
{
    public:
        void draw() const
        {
            for(unsigned short i=0;i<NCELLS;i++)
            {
                for(unsigned short j=0;j<NCELLS;j++)
                {   

                    tPairUShort lPair = std::make_pair(j,i);
                    if(std::binary_search(mFeat.begin(),mFeat.end(),lPair,comppair2))
                        std::cout  << "X";
                    else
                        std::cout << ".";
                }
                std::cout << std::endl;
            }
        }
    private: 
        cv::Mat mEdge;
        // this feature should be printed preceeded by the number of points
        std::ostream& printVector(std::ostream& pOutput) const
        {

            pOutput << setiosflags(std::ios::fixed) << std::setprecision(4) << 2*mFeat.size() << " \t";
            for(int i=0;i<mFeat.size();++i)
            {
                //pOutput << setiosflags(std::ios::fixed) << std::setprecision(4) << mFeat[i].first << " \t"<< mFeat[i].second << " \t";
                printValue(mFeat[i],pOutput);
            }
            draw();
            return pOutput;
        }
    private:
        const std::vector<tPairUShort>& doCompute()
        {
            mFeat.clear();
            cv::Size lSz = cvGetSize(this->mGray);
//            std::cout << "=============== Size = " << lSz.width << "," << lSz.height << " ==============" << std::endl;
            cv::Mat mEdge;
            cv::Canny( this->mGray, mEdge, 100, 200);

            std::vector<std::vector<cv::Point> > lContours;
            std::vector<cv::Point> lAllContours;
            cv::Mat lContoursBuf = mEdge.clone();
            cv::findContours(lContoursBuf, lContours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
            for(int i=0;i<lContours.size();++i)
                lAllContours.insert(lAllContours.end(),lContours[i].begin(), lContours[i].end());
//            std::cout << "-------------------FIRST----------------" << std::endl;
//            for(int i=0;i<lAllContours.size();i++)
//                std::cout << "(" << lAllContours[i].x << "," << lAllContours[i].y << ") \t";
//            std::cout << std::endl;
//            std::cout << "-------------------SORTED----------------" << std::endl;
//            std::sort(lAllContours.begin(),lAllContours.end(),comppoint);
//            for(int i=0;i<lAllContours.size();i++)
//                std::cout << "(" << lAllContours[i].x << "," << lAllContours[i].y << ") \t";
//            std::cout << std::endl;
//            std::cout << "-------------------CROPPED----------------" << std::endl;
//            std::vector<cv::Point>::iterator lIt = std::unique(lAllContours.begin(),lAllContours.end());
//            lAllContours.resize(lIt-lAllContours.begin());
//            for(int i=0;i<lAllContours.size();i++)
//                std::cout << "(" << lAllContours[i].x << "," << lAllContours[i].y << ") \t";
//            std::cout << std::endl;
            for(int i=0;i<lAllContours.size();++i)
            {
                tPairUShort lPair = std::make_pair( lAllContours[i].x*NCELLS/lSz.width,lAllContours[i].y*NCELLS/lSz.height);
//                std::cout << "(" << lAllContours[i].x << "," << lAllContours[i].y << ")->(" <<
//                    lPair.first << "," << lPair.second <<") \t";
                mFeat.push_back(lPair);
            }
            std::sort(mFeat.begin(),mFeat.end(),comppair2);
            std::vector<tPairUShort>::iterator lIt = std::unique(mFeat.begin(),mFeat.end());
            mFeat.resize(lIt-mFeat.begin());

            return this->mFeat;
        }
    public:
        Edge():Feature<tPairUShort>(){}
        void draw(boost::gil::gray8_image_t &pFeatIm,char* pFeatPath=NULL) const
        {
            cv::Size lSz = mEdge.size();
            pFeatIm.recreate(lSz.width,lSz.height,boost::gil::gray8_pixel_t(255),lSz.width);
            boost::gil::copy_pixels(buola::cvi::gil_wrap<boost::gil::gray8_view_t>(mEdge),view(pFeatIm));
        }
};
#endif
