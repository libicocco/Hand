#ifndef PROCESSHOGH
#define PROCESSHOGH
#include <vector>
#include <time.h>
#include <boost/progress.hpp>
#include <iostream>

#include <boost/accumulators/numeric/functional/vector.hpp>
#include <boost/accumulators/statistics/stats.hpp>
#include <boost/accumulators/framework/features.hpp>
#include <boost/accumulators/statistics/variance.hpp>
#include <boost/accumulators/accumulators.hpp>

#include "nn.h"
// #include "poselist_pf.h"
#include "cTemporalFilter.h"
#include "constants.h"
#include "typeDefinitions.h"

namespace acc=boost::accumulators;

/**
 * @brief Common function for processFeat for poselist and poselist_pf: accumulate the error in pErrorAcc
 *
 * @param pExpectedPose Ground truth pose
 * @param pWPose Pose computed by the system
 * @param pErrorAcc Accumulator for the error in pose and orientation
 * @return void
 **/
void accumulateError(const tPoseV &pExpectedPose,const tPoseV &pWPose,std::vector<tAcc_MeanVar> &pErrorAccV, tAcc_MeanVar &pErrorAcc)
{
  if(pExpectedPose!=tPoseV::Zero())
  {
    float lSumSqErr = 0;
    for(int i=0;i<NORI+NJOINTS;++i)
      lSumSqErr += (180/M_PI)*(asin(pExpectedPose[i])-asin(pWPose[i]))*(180/M_PI)*(asin(pExpectedPose[i])-asin(pWPose[i]));
    pErrorAcc(sqrt(lSumSqErr));
    
    //tPoseV lErr=pExpectedPose-pWPose; //FIXME:there should be some weigths applied to different elements
    //for(int i=0;i<lErr.size();++i)
    //  pErrorAccV[i](fabs(lErr[i]));
    for(int i=0;i<pWPose.size();++i)
    {
      float lA0 = (180/M_PI)*asin(pExpectedPose[i]);
      float lA1 = (180/M_PI)*asin(pWPose[i]);
      pErrorAccV[i](fabs(fmod(lA0-lA1,180)));
    }
  }
}

/** 
 * @brief Class that keeps the common things for ProcessFeat for PF and non-PF
 */
class CGenericProcessFeat
{
public:
  CGenericProcessFeat():mErrorAccV(NJOINTS+NORI),mErrorAcc(){}
  const float getMeanError()const{return acc::extract::mean(mErrorAcc);}
  const float getVarError()const{return acc::extract::variance(mErrorAcc);}
  const tPoseV getMeanErrorV()const
  {
    tPoseV lMean;
    for(int i=0;i<mErrorAccV.size();++i)
      lMean[i]=acc::extract::mean(mErrorAccV[i]);
    return lMean;
  }
  const tPoseV getVarErrorV()const
  {
    tPoseV lVar;
    for(int i=0;i<mErrorAccV.size();++i)
      lVar[i]=acc::extract::variance(mErrorAccV[i]);
    return lVar;
  }
protected:
  std::vector<tAcc_MeanVar> mErrorAccV; /**< Vector accumulator for ground truth data */
  tAcc_MeanVar mErrorAcc; /**< accumulator for ground truth data */
};

/** 
 * @brief Class that conglomerates the dynamics and observation of the system.
 * For poselist and poselist_multi, the dynamics are reprensented by continuity enforcement over the 
 * observation driven sieve, done by NN based on hog
 */
template <typename PL,typename tType>
class ProcessFeat:public CGenericProcessFeat
{
public:
  /** 
   * @brief Constructor for poselist_multi and poselist
   * 
   * @param pNearn Nearest Neighbor object (necessary for the first sieve)
   * @param pPoselist): Poselist to process with the NN from pNearn
   */
  ProcessFeat(nn<tType>* pNearn, PL* pPoselist):
  mPoselist(pPoselist),mNearn(pNearn){}
  const PL* getPoselist()const{return mPoselist;}
  /** 
   * @brief Updates the poselist based on the current feature: get the NN and vote them in interpolatePoses
   * 
   * @param pVFeat Current feature vector
   * @param pFrame Frame number
   * @param pExpectedPose Expected pose (if ground truth available; if not NULL)
   * 
   * @return Grasp class
   */
  void UpdatePoselist(const std::vector<tType> &pVFeat,const tPoseV &pExpectedPose)
  {
    //std::copy(pVFeat.begin(),pVFeat.end(),std::ostream_iterator<float>(std::cout," ")); td::cout << std::endl;
    // get nearest neighbors lLNearn based on feature pVHog
    tPairV lLNearn =  mNearn->computeNN(pVFeat);
    //std::cout << "nn results: " << lLNearn.front().first << ": " << lLNearn.front().second << std::endl;
    // get weighted posed lWPose based on lLNearn nearest neighbors
    tPoseV lWPose = mPoselist->interpolatePoses(lLNearn);
    std::cout << *mPoselist << std::endl;
    // if there's an expected pose pExpectedPose (there's ground truth)
    // compute the error and accumulate it in mErrorAcc
    accumulateError(pExpectedPose,lWPose,mErrorAccV,mErrorAcc);
  }
  const nn<tType>* getApproxNN()const{return mNearn;}
protected:
  PL* mPoselist; /**< Poselist that keeps the posible poses, their weights, etc*/
  nn<tType>* mNearn;    /**< Nearest neighbor object; computes nn based on the feat*/
};

/** 
 * @brief Class that conglomerates the dynamics and observation of the system, especialization for poselist_pf.
 * For poselist_pf, the dynamics are represented by a field in the database, and the candidates are
 * weighted based on the hog distance to the observation.
 */
// template<typename tType>
// class ProcessFeat<Poselist_PF,tType>: public CGenericProcessFeat
// {
  // public:
  // 	/** 
  // 	 * @brief Constructor for Poselist_PF
  // 	 * 
  // 	 * @param pPoselist Poselist (particle list) to process
  // 	 */
  // 	ProcessFeat(Poselist_PF* pPoselist):
  // 	mPoselist(pPoselist),mErrorAccV(){}
  // 	const PL* getPoselist()const{return mPoselist;}
  // 	/** 
  // 	 * @brief Updates the poselist based on the current hog: propagates the particles and weights them
  // 	 * 
  // 	 * @param pVFeat Current feature vector
  // 	 * @param pFrame Current frame
  // 	 * @param pExpectedPose Expected pose (if ground truth available; if not NULL)
  // 	 * 
  // 	 * @return Grasp class
  // 	 */
  // 	void UpdatePoselist(const std::vector<tType> &pVFeat,const tVectorD *pExpectedPose)
  // 	{
    // 		tPoseV lWPose = mPoselist->interpolatePoses(pVFeat);
    // 		accumulateError(pExpectedPose,lWPose,mErrorAccV);
    // 	}
    // protected:
    //	PL* mPoselist; /**< Poselist that keeps the posible poses, their weights, etc*/
    // };
    #endif