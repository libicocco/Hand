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
// FIXME:to check or review:
// - the bandwidth of KDE is set to avg(var(Poses))/nPoses; is there a better way?
#include <iostream>
#include <fstream>
#include <cstdlib> 
#include <boost/accumulators/numeric/functional/vector.hpp>
#include <boost/accumulators/statistics/stats.hpp>
#include <boost/accumulators/framework/features.hpp>
#include <boost/accumulators/statistics/variance.hpp>
#include <boost/accumulators/accumulators.hpp>
#include <boost/accumulators/statistics/min.hpp>
#include "figtree.h"
#include "utils.h"
#include "constants.h"
#include "cPoselistMulti.h"
#include "typeDefinitions.h"

namespace acc=boost::accumulators;

void CPoselistMulti::saveBestPoses()
{
  if(mNFinalNeighbors!=mPoseList.size()) // if different size, resize!
    {
      if(mNFinalNeighbors!=0)
      {
        delete [] mBestOri;
        delete [] mBestJoints;
        delete [] mBestWeights;
      }
      mNFinalNeighbors = mPoseList.size();
      mBestOri = new double[mNFinalNeighbors*(NORI)];
      mBestJoints = new double[mNFinalNeighbors*(mDimMetric)];
      mBestWeights = new double[mNFinalNeighbors];
    }
    
    double *lBestOri = mBestOri; // local pointers created for being moved!
    double *lBestJoints = mBestJoints; // local pointers created for being moved!
    double *lBestWeights = mBestWeights;
    boost::ptr_vector<CPose>::iterator itr;
    for (itr=mPoseList.begin();itr!=mPoseList.end();++itr)
    {
      tPoseV lPose(itr->getPose());
      unsigned lMetricIndex = itr->getIndex()/NVIEWS;
      for(int i=0;i<NORI;++i)
        lBestOri[i]=lPose[i];
      for(int i=0;i<mDimMetric;++i)
        if(mMetric.empty())
          lBestJoints[i]=lPose[i+NORI];
        else
          lBestJoints[i]=mMetric[lMetricIndex*mDimMetric+i];
      lBestOri+=(NORI);
      lBestJoints+=(mDimMetric);
      *lBestWeights=itr->getWeight();
      lBestWeights++;
    }
    // normalize mBestWeights
    double totalWeight=0;
    for(int i=0;i<mNFinalNeighbors;i++)
    {
      mBestWeights[i] = (mBestWeights[i]>0.25)?0.25:mBestWeights[i]; // clamp big val
      totalWeight+=mBestWeights[i];
    }
    for(int i=0;i<mNFinalNeighbors;i++)
      mBestWeights[i]/=totalWeight;
}

void CPoselistMulti::setWeights()
{
  tAcc_Sum lSumFeatW,lSumPoseW;
  boost::ptr_vector<CPose>::iterator itr;
  double lMinFeatD = acc::extract::min(mAccFeatD);
  double lVarFeatD = acc::variance(mAccFeatD);
  if(mNFinalNeighbors!=0 && mUseTemporalLikelihood)
  {
    double *lOriWeights = new double[mPoseList.size()];
    double *lOri = new double[mPoseList.size()*(NORI)];
    double *lJointWeights = new double[mPoseList.size()];
    double *lJoints = new double[mPoseList.size()*(mDimMetric)];
    double *lTmpOri = lOri; /**< lOri holds a copy of the oris, since figtree needs them in array mode*/
    double *lTmpJoints = lJoints; /**< lJoints hold a copy of the joints, since figtree needs them in array mode*/
    std::vector<tAcc_Var> lPosesVar(mDimMetric+NORI);
    for (itr=mPoseList.begin();itr!=mPoseList.end();++itr)
    {
      tPoseV lPose(itr->getPose());
      unsigned lMetricIndex = itr->getIndex()/NVIEWS;
      for(int i=0;i<NORI;++i)
      {
        lTmpOri[i]=lPose[i];
        lPosesVar[i](lTmpOri[i]);
      }
      lTmpOri+=NORI;
      for(int i=0;i<mDimMetric;++i)
      {
        if(mMetric.empty())
          lTmpJoints[i]=lPose[NORI+i];
        else
          lTmpJoints[i]=mMetric[lMetricIndex*mDimMetric+i];
        lPosesVar[i](lTmpJoints[NORI+i]);
      }
    }
    
    double lVarPoseOri = 0;
    double lVarPoseJoints = 0;
    for(int i=0;i<NORI;++i)
      lVarPoseOri+=acc::variance(lPosesVar[i]);
    lVarPoseOri/=NORI;
    for(int i=NORI;i<NORI+mDimMetric;++i)
      lVarPoseJoints+=acc::variance(lPosesVar[i]);
    lVarPoseJoints/=mDimMetric;
    
    /** sqrt(2*varPoseDist) would put the bandwidth of the whole distribution
     *  to each kernel! It has to be lower... NPARTICLES lower?
     * Another issue is that it's probably better to set the bandwidth related to 
     * the var of the final neighbors, and not all the neighbors: all the non-final 
     * neighbors contribute to the variance (a lot) and are not related to the final 
     * distribution
     */
    double lBWOri = sqrt(2*lVarPoseOri)/mNFinalNeighbors; // FIXME
    double lBWJoints = sqrt(2*lVarPoseJoints)/mNFinalNeighbors; // FIXME
    
    figtree( NORI, mNFinalNeighbors, mPoseList.size(), 1, mBestOri, lBWOri, 
             mBestWeights, lOri, 1e-2, lOriWeights, FIGTREE_EVAL_IFGT_TREE, FIGTREE_PARAM_NON_UNIFORM, 1 );
    figtree( mDimMetric, mNFinalNeighbors, mPoseList.size(), 1, mBestJoints, lBWJoints, 
             mBestWeights, lJoints, 1e-2, lJointWeights, FIGTREE_EVAL_IFGT_TREE, FIGTREE_PARAM_NON_UNIFORM, 1 );
    
    // summing both weights into lJointWeights, just to avoid allocating a new array
    for(int i=0;i<mNFinalNeighbors;++i)
      lJointWeights[i] += lOriWeights[i];
    
    int i=0;
    for (itr=mPoseList.begin();itr!=mPoseList.end();++itr,i++)
    {
      mMaxWeight(lJointWeights[i]);
      itr->setFeatW(lMinFeatD,lVarFeatD,lSumFeatW);
      itr->setPoseW(lJointWeights[i],lSumPoseW);
    }
    
    delete[] lOriWeights;
    delete[] lOri;
    delete[] lJointWeights;
    delete[] lJoints;
  }
  else
  {
    for (itr=mPoseList.begin();itr!=mPoseList.end();++itr)
    {
      itr->setFeatW(lMinFeatD,lVarFeatD,lSumFeatW);
      itr->setPoseW(0,lSumPoseW);
    }
  }
  NormalizeWeights(acc::sum(lSumFeatW),acc::sum(lSumPoseW));
}