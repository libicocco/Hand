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
#ifndef __CTEMPORALF_H
#define __CTEMPORALF_H

#include <iostream>
#include <boost/ptr_container/ptr_vector.hpp>
#include <Eigen/Dense>
#include "cPose.h"
#include "cDB.h"
#include "typeDefinitions.h"
typedef boost::ptr_vector<CPose> tPoseVP;

/** 
 * @brief Class that votes the elements in a list of poses based on the
 * previous frames. Poselist, Poselist_multi and Poselist_PF inherit from it
 */
class CTemporalFilter
{
  friend std::ostream& operator << ( std::ostream& pOut, const CTemporalFilter& pTempF);
public:
  CTemporalFilter(const fsystem::path &pDBPath,bool pTemporalSmoothing=true):
      mDB(pDBPath),mPoseW(0.5),mWPose(tPoseV::Zero()),mUseTemporalLikelihood(pTemporalSmoothing){}
  virtual ~CTemporalFilter(){};
  /** 
   * @brief Main function: it initializes a set of poses based on the NN
   * results (indexes and distances), and weights them based on the similarity
   * to the observed HOG and a set of previous best poses (with parzen windows)
   * 
   * @param lNNDist List of pairs with NN indexes and distances to the current observed HOG
   * 
   * @return Estimated joint vector
   */
  const tPoseV& interpolatePoses(const tPairV &lNNDist);
  tPoseV getWPose() const {return mWPose;}
  const tPoseVP& getPoselist() const{return mPoseList;}
  const CPose & getBestPose() const{return mPoseList.front();}
  
protected:
  /**
   * @brief Normalizes the feature and pose weights for every pose in mPoseList according to the sums passed as parameters
   *
   * @param lTotalFeatW Sum of all the feature weights
   * @param lTotalPoseW Sum of all the pose weights
   * @param pPoseWeight Sets the balance between pose and feature importance Defaults to 0.5.
   * @return void
   **/
  void NormalizeWeights(double lTotalFeatW,double lTotalPoseW, double pPoseWeight=0.5)
  {
    for (auto itrpl=mPoseList.begin();itrpl!=mPoseList.end();++itrpl)
      itrpl->NormalizeWeights(lTotalFeatW,lTotalPoseW,pPoseWeight);
  }
  /**
   * @brief Virtual function to be defined in children classes: sets the weights of the poses
   * based on previous frame pose and current feature
   *
   * @return void
   **/
  virtual void setWeights()=0;
  /**
   * @brief Virtual function to be defined in children classes: saves the poses and weights
   * for the next frame. The saving format depends on which method is used in setWeightk
   *
   * @return void
   **/
  virtual void saveBestPoses()=0;
  /** 
   * @brief Reset mPoseList, creating new poses and putting them in the list.
   * It's used just by Poselist and Poselist_multi, since Poselist_PF doesn't have pNNlist.
   * 
   * @param pNNlist List of pairs with NN indexes and distances to the current observed HOG
   * 
   * @return The size of the input list (same as the mPoseList)
   */
  int reset(const tPairV& pNNlist);
  static bool cmpW(const CPose& pFirst,const CPose& pSecond)
  {return (pFirst.getWeight()>pSecond.getWeight());}
  /** 
   * @brief Computes Weighted Pose, and sort the mPoseList.
   *  It can alse reject outliers, but the results doesn't improve.
   */
  void computeWeightedPose();
  /** 
   * @brief Based on the Mahalanobis distance, rejects poses to far from the 
   * predicted one.
   * 
   * @param pNNN How many neighbors should be left in the list
   */
  void reduceNNN(size_t pNNN);
protected:
  CDB mDB; /**< Database holding the data about generated poses*/
  tAcc_MinVar mAccFeatD; /**< Accumulator for Features: holds the min and variance*/
  tAcc_MinVar mAccPoseD; /**< Accumulator for Pose: holds the min and variance*/
  unsigned int mGraspClass; /**< Grasp class majority for the best poses*/
  boost::ptr_vector <CPose> mPoseList; /**< List with all the treated poses (NN or particles)*/
  double mPoseW; /**< Weight with which the pose is mixed with the HOG*/
  tPoseV mWPose; /**< Best pose vector from last frame*/
  bool mUseTemporalLikelihood; /**< bool to disactivate the use of temporal smoothing*/
};
#endif
