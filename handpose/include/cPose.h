#ifndef __CPOSE_H
#define __CPOSE_H

#include <vector>
#include <string>
#include <iostream>
#include <math.h>
#include <boost/filesystem.hpp>
#include <boost/accumulators/numeric/functional/vector.hpp>
#include <boost/accumulators/statistics/stats.hpp>
#include <boost/accumulators/framework/features.hpp>
#include <boost/accumulators/statistics/variance.hpp>
#include <boost/accumulators/accumulators.hpp>
#include <boost/accumulators/statistics/min.hpp>
#include <Eigen/Dense>

#include "utils.h"
#include "constants.h"
#include "cDB.h"
#include "cDBelement.h"
#include "typeDefinitions.h"

namespace fsystem=boost::filesystem;

class CPose
{
  friend std::ostream& operator << ( std::ostream& pOutput, const CPose& pPose)
  {
    pOutput << pPose.mIndex << ": ";
    return pPose.showWeights(pOutput);
    //return pPose.showPose(pOutput);
  }
  
public:
  /**
   * @brief CPose ctor; extracts information from database and acc distances
   *
   * @param pIndexDist Pair of pose index and distance to real Feature
   * @param pAcc_FeatD Accumulator that holds information about distance to real feature
   * @param pWPose Previous pose (empty if first frame)
   * @param pAcc_PoseD Accumulator that holds information about distance to previous pose
   * @param pDB Database object used for querying the pose index
   **/
  CPose(const std::pair<unsigned,double> &pIndexDist,tAcc_MinVar &pAcc_FeatD,
        tPoseV &pWPose,tAcc_MinVar &pAcc_PoseD,CDB &pDB):
        mIndex(pIndexDist.first),mFeatD(pIndexDist.second),mFeatW(UNDEFWEIGHT),mPoseW(UNDEFWEIGHT)
        {
          CDBelement lElem=pDB.query(pIndexDist.first);
          mPose=lElem.getPose();
          mImagePath=lElem.getImagePath();
          mNextIndices=lElem.getNextIndicesV();
          pAcc_FeatD(pIndexDist.second);
          if(pWPose!=tPoseV::Zero())//checking sum was too dangerous; however this fails if pose(0,0..0) is given
		{
      tPoseV lTmpV=mPose-pWPose;
      mPoseD=lTmpV.norm();
      // 			mPoseD=lTmpV.dot(lTmpV);
      pAcc_PoseD(mPoseD);
    }
    else
      mPoseD=UNDEFDIST;
        }
        
        std::ostream& showWeights(std::ostream& pOutput) const
        {
          pOutput<<"("<<mImagePath.filename()<<","<<mPoseD<<","<<
          mPoseW<<","<<mFeatD<<","<<mFeatW<<","<<mWeight<<")";
          return pOutput;
        }
        
        std::ostream& showPose(std::ostream& pOutput) const
        {
          pOutput<<" \t: ("<<mPose<<")"<<std::endl;
          return pOutput;
        }
        
        void setPoseW(double pPoseW,tAcc_Sum &pSumPoseW)
        {pSumPoseW(pPoseW);mPoseW=pPoseW;}
        
        /**
         * @brief Set the gaussian weight corresponding to the pose, and accumulate it
         *
         * @param pMinPoseD Minimum distance, used to center the gaussian weighting
         * @param pVarPoseD Variance of distance, sets the spread of the gaussian weighting
         * @param pSumPoseW Accumulator used to register the sum of all weights
         * @return void
         **/
        // 	void setPoseW(double pMinPoseD,double pVarPoseD,tAcc_Sum &pSumPoseW)
        // 	{
          // 		if(mPoseD!=UNDEFDIST)
          // 		{
            // 			double lPoseD2Min = fabs(mPoseD-pMinPoseD);
            // 			mPoseW=pVarPoseD?exp(-(lPoseD2Min*lPoseD2Min)/(2*pVarPoseD)):1;
            // 		}
            // 		else
            // 		{
              // 			mPoseW = 0;
              // 		}
              // 		pSumPoseW(mPoseW);
              // 	}
              
              /**
               * @brief Set the gaussian weight corresponding to the Feature, and accumulate it
               *
               * @param pMinPoseD Minimum distance, used to center the gaussian weighting
               * @param pVarPoseD Variance of distance, sets the spread of the gaussian weighting
               * @param pSumPoseW Accumulator used to register the sum of all weights
               * @return void
               **/
              void setFeatW(double pMinFeatD,double pVarFeatD,tAcc_Sum &pSumFeatW)
              {
                double lFeatD2Min = fabs(mFeatD-pMinFeatD);
                mFeatW=pVarFeatD?exp(-(lFeatD2Min*lFeatD2Min)/(2*pVarFeatD)):1;
                pSumFeatW(mFeatW);
              }
              
              /**
               * @brief Set the gaussian weight corresponding to pose and feature, and accumulate them
               *
               * @param pMinFeatD Minimum distance to real feature, used to center the gaussian weighting
               * @param pVarFeatD Variance of distance to real feature, sets the spread of the gaussian weighting
               * @param pMinPoseD Minimum distance to previous pose, used to center the gaussian weighting
               * @param pVarPoseD Variance of distance to previous pose, sets the spread of the gaussian weighting
               * @param pSumFeatW Accumulator used to register the sum of all feature weights
               * @param pSumPoseW Accumulator used to register the sum of all pose weights
               * @return void
               **/
              // 	void setWeights(double pMinFeatD,double pVarFeatD,double pMinPoseD,
              // 									double pVarPoseD,tAcc_Sum &pSumFeatW,tAcc_Sum &pSumPoseW)
              // 	{
                // 		setFeatW(pMinFeatD,pVarFeatD,pSumFeatW);
                // 		setPoseW(pMinPoseD,pVarPoseD,pSumPoseW);
                // 	}
                void NormalizeWeight(double pNormConstant){mWeight/=pNormConstant;}
                void NormalizeWeights(double pSumFeatW,double pSumPoseW,double pPoseW=0.5)
                {
                  mFeatW=(abs(pSumFeatW)<0.01)?mFeatW:mFeatW/pSumFeatW;
                  mPoseW=(abs(pSumPoseW)<0.01)?mPoseW:mPoseW/pSumPoseW;
                  mWeight=(1-pPoseW)*mFeatW+pPoseW*mPoseW;
                }
                double getWeight() const{return mWeight;}
                const tPoseV& getPose() const{return mPose;}
                const fsystem::path& getImagePath() const{return mImagePath;}
                unsigned getIndex() const{return mIndex;}
private:
  unsigned mIndex;/**< DB index of pose. @attention Poses should be sorted when added to the db!*/
  double mFeatD;/**< HOG distance between the pose HOG and the HOG of the real image*/
  double mPoseD;/**< Pose distance between the pose HOG and the previous estimated Pose*/
  double mFeatW;/**< Weight related with hog_distance and the statistics of hog_distances*/
  double mPoseW;/**< Weight related with pose_distance and the statistics of pose_distances*/
  double mWeight;/**< Total weight. It's basically how good is this estimatation for the current frame */
  tPoseV mPose;/**< Joint angles for this pose (including orientation)*/
  std::vector<unsigned int> mNextIndices;/**< Indexes which are probable to follow this pose. Extracted from the database*/
  fsystem::path mImagePath;/**< Path to the database image*/
};
#endif