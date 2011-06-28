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
#ifndef HOGH
#define HOGH

#include <vector>
#include <iostream>
#include <math.h>
#include <stdlib.h>
#include <string>
#include "opencv2/core/core.hpp"
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/video/tracking.hpp>
#include "constants.h"
#include "feature.h"
#include "utils.h"

template<typename tType>
void drawhog(const unsigned pNLevels,const unsigned pNBins,const std::vector<tType> &pFeat,cv::Mat &pFeatIm)
{
  const int lNcuts = pow(2.0,pNLevels-1);
  cv::Size lCellSz(pFeatIm.size().width/lNcuts,pFeatIm.size().height/lNcuts);
  for(int lCol=0;lCol<lNcuts;++lCol)
  {
    for(int lRow=0;lRow<lNcuts;++lRow)
    {
      static const double lLineSize(lCellSz.width*0.9);
      cv::Point lCenter(lCellSz.width*(lCol+0.5),lCellSz.height*(lRow+0.5));
      typename std::vector<tType>::const_iterator lBeginBin = pFeat.begin()+(lCol*lNcuts+lRow)*pNBins;
      typename std::vector<tType>::const_iterator lEndBin = lBeginBin+pNBins;
      typename std::vector<tType>::const_iterator lMaxBin = std::max_element(lBeginBin,lEndBin);
      double lAngle = (*lMaxBin>0.0001)?((lMaxBin-lBeginBin)*M_PI/pNBins + M_PI/2):-1;
      double lHalfLength = (*lMaxBin)*lLineSize*(0.5);
      //for(auto it=lBeginBin;it!=lEndBin;++it)
      // std::cout << *it << " ";
      //std::cout << ": " << lHalfLength << std::endl;
      cv::Point lV(lHalfLength*cos(lAngle),lHalfLength*sin(lAngle));
      if(pFeatIm.channels()==3)
        cv::line(pFeatIm,lCenter-lV,lCenter+lV,cv::Scalar(0,0,0));
      else
        cv::line(pFeatIm,lCenter-lV,lCenter+lV,cv::Scalar(0));
    }
  }
}

/** 
 * @brief Class for computing hogs, given different types of inputs
 */
template<typename tType>
class Hog: public Feature<tType>
{
private:
  const int mNLevels,mNBins,mNHists;
  const bool mMultilevel,mNormalizeHoles;
  std::ostream& printVector(std::ostream& pOutput) const
  {
    for(int i=0;i<mNHists*mNBins;++i)
      pOutput << setiosflags(std::ios::fixed) << std::setw(3) << std::setprecision(3)<< this->mFeat[i] << " ";
    return pOutput;
  }
  /**
   * @brief Computes the gradient orientation (0-180) of a masked image
   *
   * @param lGray Input image (grey level)
   * @param lMask lMask will be updated with low gradients from lOriOKmask
   * @param lOri Output gradient orientation in degrees
   * @return void
   **/
  void calcGradient(const cv::Mat &pGray,cv::Mat &pMask,cv::Mat &pOri)
  {
    // oriOKmask contain which pixels orientation were properly extracted
    cv::Mat lOriOKmask(pGray.size(),CV_8UC1);
    cv::calcMotionGradient(pGray, lOriOKmask, pOri, 1, 10000, 3 );
    //cvCmpS( lOriOKmask, 0, lOriOKmask, CV_CMP_GT);
    lOriOKmask = lOriOKmask>0;
    //cvAnd(mask,oriOKmask,mask);
    pMask = pMask & lOriOKmask;
  }
  
  /**
   * @brief Computes the histogram given a couple of images. It can compute it with different depths 
   * (nlevels) and multilevel or not (all levels or just the last one)
   *
   * @param pOri Input image with the gradient image
   * @param pMask Mask; anything black is not considered
   * @param pNlevels One level means just one histogram, two levels means 4 quarters, etc.
   * @param pHists Output histograms
   * @return void
   **/
  void calcCellsHist(const cv::Mat &pOri, const cv::Mat &pMask, const int pNlevels, cv::MatND *pHists)
  {
    cv::Size lSrcSize = pOri.size();
    const int lNcuts = pow(2.0,mNLevels-1);
    cv::Size lCellSz(lSrcSize.width/lNcuts,lSrcSize.height/lNcuts);
    
    for (int w=0;w<lNcuts;w++)
    {
      for (int h=0;h<lNcuts;h++)
      {
        cv::Rect lCell(w*lCellSz.width,h*lCellSz.height,
                       w==lNcuts-1?lSrcSize.width-w*lCellSz.width:lCellSz.width,
                       h==lNcuts-1?lSrcSize.height-h*lCellSz.height:lCellSz.height);
        
        int lChannels[] = {0};
        //int lBins[] = {mNBins};
        //float lRanges[] = {0.0,180.01};
        int lBins[] = {2*mNBins};
        float lRanges[] = {0.0,360.01};
        const float* lRRanges[]={lRanges};
        //cv::calcHist( &(pOri(lCell)), 1, lChannels, pMask(lCell), lHists[w*lNcuts+h], 1, lBins, lRRanges, true, false); // warning!
        cv::Mat lOriCell(pOri(lCell));
        cv::calcHist( &lOriCell, 1, lChannels, pMask(lCell), pHists[w*lNcuts+h], 1, lBins, lRRanges, true, false); 
        
        double lSum = (cv::sum(pHists[w*lNcuts+h]))[0];
        if(mNormalizeHoles)
        {
          // the idea is to reject bins with not so many good gradients
          // the normalization factor is the ratio good gradients/total gradients, corrected with the 255 from the mask 1s
          double lNorm=static_cast<double>(cv::sum(pMask(lCell))[0])/(255*static_cast<double>(lCell.area()));
          pHists[w*lNcuts+h]*=lSum?lNorm/lSum:0;
        }
        else
          pHists[w*lNcuts+h]*=lSum?1.0/lSum:0;
      }
    }
    if(pNlevels>1)
    {
      // pointers to beginnning of previous and actual level hists
      cv::MatND *lPrevLv=&(pHists[0]);
      cv::MatND *lThisLv=&(pHists[int(pow(4.0,pNlevels-1))]);
      // for each of the remaining levels to compute
      for(int lv=pNlevels-2;lv>=0;--lv)
      {
        // read and write pointers
        cv::MatND *lRead=lPrevLv;
        cv::MatND *lStore=lThisLv;
        // number of cells to jump in the array to get the cell to your side
        int lStride=(int)pow(2.0,lv);
        // for each cell to compute in this level
        for(int c=0;c<(int)pow(4.0,lv);++c)
        {
          // 					std::cout << "(" << lv << "," << c << "):" << lStore << std::endl;
          // write the average of 4cells group
          *lStore = 0.25*(*lRead+(*(lRead+1))+(*(lRead+lStride))+(*(lRead+lStride+1)));
          // advance write pointer
          lStore++;
          if((lRead+lStride+2)==lThisLv) // redundant with loop, but good for checking
						break;
          // advance 2 or stride+2 the read pointer, depending on the column where it lies
            lRead+=2;
            if(((lRead-lPrevLv)/lStride)%2)
              lRead+=lStride;
        }
        // update beginning of previous and actual levels
        lPrevLv=lThisLv;
        lThisLv=lStore;
      }
    }
  }
  
  /**
   * @brief Interface for computing the feature
   *
   * @param pIm Input image CV_32FC3
   * @param pMask pIm(x,y) should be disregarded if pMask(x,y)=0 CV_8UC1
   * @return :vector< tType >& Output vector of numbers (feature)
   **/
  const std::vector<tType>& doCompute(std::pair<cv::Mat,cv::Mat> &pImMask)
  {
    cv::Mat lGray(pImMask.first.size(),CV_32FC1),lOri(pImMask.first.size(),CV_32FC1);
    cv::cvtColor(pImMask.first, lGray, CV_RGB2GRAY);
    calcGradient(lGray,pImMask.second,lOri);
    cv::MatND *lHists = new cv::MatND[mNHists];
    calcCellsHist(lOri, pImMask.second, mMultilevel?mNLevels:1, lHists);
    
    this->mFeat.clear();
    for( int h = 0; h < mNHists; h++ )
      for( int o = 0; o < mNBins; o++ )
        this->mFeat.push_back((tType)lHists[h].at<float>(o)+(tType)lHists[h].at<float>(o+mNBins));
      
      delete []lHists;
    return this->mFeat;
  }
  
public:
  Hog(const int pNLevels=4,
      const int pNBins=8,
      const int pMultilevel=false,
      const bool pNormalizeHoles=true):
      Feature<tType>(),
      mNLevels(pNLevels),mNBins(pNBins),
      mNHists(pMultilevel?(((1<<(2*pNLevels))-1)/3):(1<<(2*(pNLevels-1)))),
      mMultilevel(pMultilevel),mNormalizeHoles(pNormalizeHoles){}
      ~Hog(){}
      unsigned getFeatSize() const{return mNHists*mNBins;}
      /**
       * @brief Generates an graphical representation pFeatIm of the feature
       *
       * @param pFeatIm Output image with the graphical representation (needs to be allocated)
       * @return void
       **/
      void draw(cv::Mat &pFeatIm) const
      {
        drawhog<tType>(mNLevels,mNBins,this->mFeat,pFeatIm);
      }
};

#endif
