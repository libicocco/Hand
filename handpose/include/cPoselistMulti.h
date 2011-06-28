#ifndef __CPOSELISTMULTI_H
#define __CPOSELISTMULTI_H

#include "cTemporalFilter.h"
#include "typeDefinitions.h"

/** 
 * @brief Given a set of pose indexes and distances, it extracts the grasp class and 
 * estimates the joint pose vector. The continuos enforcement is done based on a 
 * set of poses; the weighting is done through parzen windows
 */
class CPoselistMulti: public CTemporalFilter
{
public:
  /** 
   * @brief Constructor
   */
  CPoselistMulti(const fsystem::path &pDBPath,bool pTemporalSmoothing=true,unsigned pDimMetric=0,const std::string &pMetricPath=""):
          CTemporalFilter(pDBPath,pTemporalSmoothing),mNFinalNeighbors(0)
          {
            mDimMetric = pDimMetric;
            if(!pMetricPath.empty())
            {
              std::ifstream lMetricS(pMetricPath);
              std::copy(std::istream_iterator<float>(lMetricS),std::istream_iterator<float>(),std::back_inserter(mMetric));
            }
            else
              mDimMetric = NJOINTS;
          }
  //const double* getBestPoses(){return mBestPoses;}
  const double* getBestWeights(){return mBestWeights;}
  unsigned int getNFinalNeighbors(){return mNFinalNeighbors;}
private:
  /** 
   * @brief Saves the joints from the best poses from current frame,
   *  in a single dimension array, to be used for weighting the next frame NN
   */
  void saveBestPoses();
  /** 
   * @brief Sets the poses weights, based on parzen windows (for poseW) generated from the 
   * best poses in the previous frame( if no previous pose available, poseW=0)
   */
  void setWeights();
private:
  double* mBestOri; /**< Array with had orientation wrt cam of best poses from previous frame*/
  double* mBestJoints; /**< Array with joint values of best poses from previous frame*/
  double* mBestWeights; /**< Array with the weights of best poses from previous frame*/
  unsigned int mNFinalNeighbors; /**< Number of poses taken into account from previous frame*/
  tAcc_Max mMaxWeight;
  unsigned mDimMetric;
  std::vector<float> mMetric;
};
#endif