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
#ifndef CTRACKERH
#define CTRACKERH

#include <vector>
#include <map>
#include <opencv2/core/core.hpp>
#include <boost/numeric/ublas/matrix.hpp>
#include <boost/numeric/ublas/io.hpp>
#include <iostream>
#include "constants.h"
#include "ctrackable.h"
namespace ublas  = boost::numeric::ublas;

/** 
 * @brief Function that compairs the second element of a pair
 * 
 * @param first First pair
 * @param second Second pair
 * 
 * @return True if the first pair is less than the second
 */
  template <typename anyFlag, typename anyNum>
bool compPairLessFirst(std::pair<anyFlag,anyNum> first, std::pair<anyFlag,anyNum> second)
{
  return (first.second < second.second);
}

/** 
 * @brief Returns the distance from two points
 * 
 * @param p1 First point
 * @param p2 Second point
 * 
 * @return  Distance
 */
template <typename CAnyPoint> double dist(CAnyPoint &p1, const CAnyPoint &p2)
{
  return (double) p1.GetDist(p2);
}

/** Computes the distance between two rectangles.
 * If they intersect, 0<=d<1.
 * @brief Computes the distance between two rectangles.
 * 
 * @param r1 First rectangle
 * @param r2 Second rectangle
 * 
 * @return Distance 
 */
template<>
inline double dist (cv::Rect &r1, const cv::Rect &r2)
{
  double lDist;
	double lIntersection = (r1&r2).area();
	if(lIntersection>0.01)
    lDist = 1 - lIntersection*lIntersection/(r1.height*r1.width*r2.height*r2.width);
  else
  {
		cv::Point c1(r1.x+r1.width/2, r1.y+r1.height/2);
		cv::Point c2(r2.x+r2.width/2, r2.y+r2.height/2);
    //double d1 = cv::norm(r1.size());///2.0;
    //double d2 = cv::norm(r1.size());///2.0;
    double d1 = sqrt(r1.width*r1.width+r1.height*r1.height)/2;
    double d2 = sqrt(r2.width*r2.width+r2.height*r2.height)/2;
    double a1 = r1.area();
    double a2 = r2.area();
    double sizeRatio = std::max(a1/a2,a2/a1);
    lDist = (norm(c1-c2)/(d1+d2))*sqrt(sizeRatio);
  }
  return lDist;
}

/** 
 * @brief Class that tracks any CShape (for example, rectangles).
 */
template <typename CShape, typename CAnyPoint>
class CTracker
{
    /** 
     * @brief Printer function: prints the id and the center of each Shape
     */
  friend std::ostream& operator << ( std::ostream& output, const CTracker& pT)
  {
    typename std::map<unsigned int,CTrackable<CShape,CAnyPoint> >::const_iterator itr;
    for (itr=(pT.mTrackedSet).begin();itr!=(pT.mTrackedSet).end();++itr)
      output << itr->first << ":" << itr->second << " \t";
    return output;
  }

  public:
  CTracker():maxtag(0),mTrackedSet(){;}
  ~CTracker(){;}

  /** 
   * @brief Given a vector of shapes, it relates them to the current set mTrackedSet
   * 
   * @param pActual Vector to be identified
   * 
   * @return Map of identifiers and CTrackable objects
   */
  std::map<unsigned int,CTrackable<CShape,CAnyPoint> > Identify(std::vector<CShape> &pActual)
  {
    if(mTrackedSet.empty()) // No old hypothesis: all new blobs are new hypothesis
    {
      maxtag = pActual.size();
      CTrackable<CShape,CAnyPoint>  lTmp;
      for(unsigned int i=0;i<maxtag;i++)
      {
        lTmp = CTrackable<CShape,CAnyPoint> (pActual[i],pActual[i],CAnyPoint(0,0),1);
        mTrackedSet.insert(std::pair<unsigned int,CTrackable<CShape,CAnyPoint> >(i+1,lTmp));
      }
    }
    else
    {
      if(pActual.size()!=0)
        Associate(pActual);
      Predict();
      typename std::map<unsigned int,CTrackable<CShape,CAnyPoint> >::iterator lItrT;
      for(lItrT=mTrackedSet.begin();lItrT!=mTrackedSet.end();++lItrT)
        (lItrT->second).incAge();
    }
    return mTrackedSet;
  }

  const std::map<unsigned int,CTrackable<CShape,CAnyPoint> > getSet() const{return mTrackedSet;}

  private:

  /** 
   * @brief Detects copies in a vector
   * 
   * @param pV Vector to be checked
   * 
   * @return True if all the elements are unique
   */
  template <typename CAny>
    bool one2one(std::vector<CAny> pV) // this is a copy, isn't it?
    {
      typename std::vector<CAny>::iterator itr;
      sort(pV.begin(),pV.end());
      itr = adjacent_find(pV.begin(),pV.end());
      return (itr==pV.end());
    }

  /** Simple function that reassigns the correspondances that are not unique.
   * The assignments not unique and not optimal are changed in a fast way
   * Can be improved!!
   * @brief Makes the correspondances between frames one to one
   * 
   * @param pDist Distance Matrix
   * @param pBestPred Prediction vector
   * @param pBestPos Position vector
   */
  void SolveConflicts(const ublas::matrix<double> &pDist, std::vector<unsigned int> &pBestPred, const std::vector<unsigned int> &pBestPos)
  {
    if(one2one(pBestPos))
    {
      pBestPred.assign(pBestPred.size(),0);
      for(unsigned int i=0;i<pBestPos.size();++i)
      {
        unsigned int bp = pBestPos[i];
        if(bp==0)
          continue;
        pBestPred[pBestPos[i]-1]=i+1;
      }
      return;
    }
    else
    {
      for(unsigned int i=0;i<pBestPred.size();++i)
      {
        if(pBestPred[i]==0)
          continue;//new
        else if(std::count(pBestPred.begin(),pBestPred.end(),pBestPred[i])==1)
          continue;//unique
        else if(pBestPos[pBestPred[i]-1]==(i+1))
          continue;//not unique, but the best
        else
        {
          std::vector<std::pair<unsigned int,float> > lTmpDistV;
          unsigned int j;
          for(j=0;j<pBestPos.size();++j)
            lTmpDistV.push_back(std::pair<unsigned int,float>(j+1,pDist(i,j)));
          sort(lTmpDistV.begin(),lTmpDistV.end(),compPairLessFirst<unsigned int,float>);
          // the best are given in order: subsubsuboptimal!
          for(j=0;j<pBestPos.size();++j)
          {
            unsigned int tag = lTmpDistV[j].first;
            if(std::count(pBestPred.begin(),pBestPred.end(),tag)==0)
            {
              pBestPred[i] = tag;
              break;
            }
          }
          if(j==pBestPos.size()) // if loop finished: reset to new!
            pBestPred[i] = 0;
        }
      }
    }
  }

  /** This function creates a distance matrix, assign the correspondances and 
   * solve the conflicts
   * @brief Associates a vector of shapes with the shapes from previous frame
   * 
   * @param pActual Observed shapes in current frame
   */
  void Associate(std::vector<CShape> &pActual)
  {
    ublas::matrix<double> lDist(pActual.size(),mTrackedSet.size());
    std::vector<unsigned int> lBestPred(lDist.size1(),0);
    std::vector<unsigned int> lBestPos(lDist.size2(),0);
    typename std::map<unsigned int,CTrackable<CShape,CAnyPoint> >::iterator lItrT;
    typename std::vector<CShape>::iterator lItrA;
    unsigned int pred,pos;
    for(lItrA=pActual.begin(),pos=0;lItrA!=pActual.end();++lItrA,++pos)
      for(lItrT=mTrackedSet.begin(),pred=0;lItrT!=mTrackedSet.end();++lItrT,++pred)
      {
        double tmpdist = dist(*lItrA,(lItrT->second).getPred());
        lDist(pos,pred) = tmpdist; 
        if((lBestPos[pred]==0 && (lDist(pos,pred)<MAXDISTTRACKER))||
            ((lBestPos[pred])!=0 && (lDist(pos,pred)< lDist(lBestPos[pred]-1,pred))))
          lBestPos[pred]=pos+1;
        if((lBestPred[pos]==0 && lDist(pos,pred)<MAXDISTTRACKER) || 
            (lBestPred[pos]!=0 && lDist(pos,pred)<lDist(pos,lBestPred[pos]-1)))
          lBestPred[pos]=pred+1;
      }
    if(!one2one(lBestPred))
      SolveConflicts(lDist,lBestPred,lBestPos);
    for (pos = 0;pos<pActual.size();++pos)
    {
      if (lBestPred[pos] != 0)
      {
        lItrT = mTrackedSet.begin();
        advance(lItrT,lBestPred[pos]-1);
        (lItrT->second).setReal(pActual[pos]);
        (lItrT->second).setNew();
      }
        /*
      else
      {
        maxtag++;
        CTrackable<CShape,CAnyPoint> lTmp = CTrackable<CShape,CAnyPoint>(*lItrA,*lItrA,cv::point(0,0));
        mTrackedSet.insert(std::pair<unsigned int,CTrackable<CShape,CAnyPoint> >(maxtag,lTmp));
      }
        */
    }
    for (pos = 0;pos<pActual.size();++pos)
    {
      if (lBestPred[pos] == 0)
      {
        maxtag++;
        CTrackable<CShape,CAnyPoint> lTmp(pActual[pos],pActual[pos],CAnyPoint(0,0));
        mTrackedSet.insert(std::pair<unsigned int,CTrackable<CShape,CAnyPoint> >(maxtag,lTmp));
      }
    }
  }
  /** 
   * @brief Predict positions for next frame
   */
  void Predict()
  {
    typename std::map<unsigned int,CTrackable<CShape,CAnyPoint> >::iterator itrT;
    CAnyPoint lV;
    for(itrT=mTrackedSet.begin();itrT!=mTrackedSet.end();)
    {
      if((itrT->second).isDead())
      {
        mTrackedSet.erase(itrT++);
        continue;
        //if(itrT==mTrackedSet.end())
          //break;// if i don't do this it freezes!!!
      }
      if((itrT->second).isNew())
      {
        (itrT->second).updateVel();
        (itrT->second).updatePred();
      }
      itrT++;
      //else
        //  (itrT->second).incAge();
    }
  }
  private:
  unsigned int maxtag;/**< Maximum identifier given*/
  typename std::map<unsigned int,CTrackable<CShape,CAnyPoint> > mTrackedSet;
};

#endif
