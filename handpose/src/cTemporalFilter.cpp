#include <iostream>
#include <boost/timer.hpp>
#include <boost/ptr_container/ptr_vector.hpp>
#include <boost/accumulators/statistics/stats.hpp>
#include <boost/accumulators/numeric/functional/vector.hpp>
#include <boost/accumulators/accumulators.hpp>
#include <boost/accumulators/framework/features.hpp>
#include <boost/accumulators/statistics/weighted_sum.hpp>
#include <boost/accumulators/statistics/weighted_mean.hpp>
#include <boost/accumulators/statistics/weighted_variance.hpp>
#include <boost/accumulators/statistics/min.hpp>
#include <Eigen/Dense>
#include "cPose.h"
#include "cDB.h"
#include "cTemporalFilter.h"
#include "constants.h"

std::ostream& operator << ( std::ostream& pOut, const CTemporalFilter& pTempF)
{
  const tPoseVP& lPlist = pTempF.mPoseList; 
  for(auto it=lPlist.begin();it!=lPlist.end();++it)
    pOut<<*it<<"\n";
  pOut << std::endl;
  return pOut;
}

const tPoseV& CTemporalFilter::interpolatePoses(const tPairV &lNNDist)
{
//   std::cout << lNNDist.size() << std::endl;
//   std::cout << mWPose << std::endl << std::endl;
  if(lNNDist.empty())
  {
    mWPose.setZero();
    throw EMPTYPOSELISTERROR;
  }
  reset(lNNDist);
  setWeights();
  computeWeightedPose();
  mPoseList.sort(cmpW);
  if(mPoseList.size()>MAXINFLUENCENN)
    reduceNNN(MAXINFLUENCENN);
  saveBestPoses();
  if(mPoseList.empty())
  {
    mWPose.setZero();
    throw EMPTYPOSELISTERROR;
  }
  mPoseW=(mPoseList.front().getWeight() < 0.05)?0.3:0.7;
  mWPose =  mPoseList.front().getPose(); // CONSIDERING BEST POSE AS WPOSE!!!
  return mWPose;
}

int CTemporalFilter::reset(const tPairV& pNNlist)
{
  mAccFeatD = tAcc_MinVar(); // clean acc
  mAccPoseD = tAcc_MinVar(); // clean acc
  mPoseList.clear(); // I assume this deletes the poses.
  tPairV::const_iterator itrpair;
  for(itrpair=pNNlist.begin();itrpair!=pNNlist.end();++itrpair)
    mPoseList.push_back(new CPose(*itrpair,mAccFeatD,mWPose,mAccPoseD,mDB));
  return pNNlist.size();
}

void CTemporalFilter::computeWeightedPose()
{
  mWPose.setZero();
  for(auto itr=mPoseList.begin();itr!=mPoseList.end();++itr)
    mWPose+=(itr->getPose())*(itr->getWeight());
  mWPose/=mPoseList.size();
  //mPoseList.sort(cmpW);
}

void CTemporalFilter::reduceNNN(size_t pNNN)
{
  if(pNNN>mPoseList.size())
    return;
  //mPoseList.sort(cmpW);
  auto itr=mPoseList.begin();
  advance(itr,pNNN);
  mPoseList.erase(itr,mPoseList.end());
  
  mWPose.setZero();
  for(auto itr=mPoseList.begin();itr!=mPoseList.end();++itr)
    mWPose+=itr->getPose()*itr->getWeight();
  mWPose/=mPoseList.size();
}