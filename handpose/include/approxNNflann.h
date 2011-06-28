/*
 * =====================================================================================
 * 
 *       Filename:  approxNNflann.h
 * 
 *    Description:  wrapper to use flann as my ANN scheme
 * 
 *        Version:  1.0
 *        Created:  05/27/10 18:11:02 CEST
 *       Revision:  none
 *       Compiler:  gcc
 * 
 *         Author:  Javier Romero (jrgn), jrgn@kth.se
 *        Company:  CAS/CSC KTH
 * 
 * =====================================================================================
 */

#ifndef APPROXNNFLANN
#define APPROXNNFLANN

//#include <unistd.h>
#include <boost/timer.hpp>
#include <boost/format.hpp>
#include <boost/progress.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/flann/flann.hpp>
#include <vector>
#include <iostream>
#include <fstream>
#include "nn.h"
#include "handclass_config.h"


/** 
 * @brief Interface to flann library (http://people.cs.ubc.ca/~mariusm/index.php/FLANN/FLANN)
 */
template<typename tType>
class approxNNflann: public nn<tType>
{
private:
	int mK;
	bool mInitialized;
	const char* mDataPath;
	const char* mIndexPath;
	float *mData;
	const float mPrecision;
	const float mBuildWeight;
	
	cv::flann::Index *mIndex;
public:
	approxNNflann(const int pK,const int pNPoints=106920,const int pDimPoints=512,const char *pDataPath=FLANNBINPATH,
								const char *pIndexPath=FLANNINDEXPATH,const float pPrecision=0.9,const float pBuildWeight=0.1):
								mK(pK),mInitialized(false),mDataPath(pDataPath),mIndexPath(pIndexPath),mPrecision(pPrecision),mBuildWeight(pBuildWeight)
								{this->nn_set_values(pNPoints,pDimPoints,mDataPath);initialize();} /**< @todo data.bin and data.idx should be configured with cmake*/
	~approxNNflann(){delete mIndex;delete mData;}
	
	bool initialize()
	{
		#ifndef NDEBUG
		std::cout << "LOADING DATA..." << std::endl;
		boost::timer t;
		t.restart();
		#endif
		mData = new tType[this->nPoints*this->pointsDimension];
		std::ifstream lS(mDataPath,std::ifstream::in|std::ifstream::binary);
		if(lS.fail())
		{
			std::cout << "PROBLEMS OPENING DATA FILE " << mDataPath << " for aNN" << std::endl;
			return 0;
		}
		lS.read((char*)mData,this->nPoints*this->pointsDimension*sizeof(tType));
		cv::Mat lMat = cv::Mat(this->nPoints,this->pointsDimension,CV_32FC1,mData);
		#ifndef NDEBUG
		std::cout << boost::format("LOAD TIME: %1%s.") % t.elapsed() << std::endl;
		#endif
		
		#ifndef NDEBUG
		std::cout << "LOADING INDEX..." << std::endl;
		t.restart();
		#endif
		if(access(mIndexPath,F_OK)==0)
			mIndex = new cv::flann::Index(lMat,cv::flann::SavedIndexParams(mIndexPath)); 
		else
		{
			mIndex = new cv::flann::Index(lMat,cv::flann::AutotunedIndexParams(mPrecision,mBuildWeight,0,0.1)); 
			//mIndex = new cv::flann::Index(lMat,cv::flann::KDTreeIndexParams(mNTrees)); 
			mIndex->save(mIndexPath);
		}
		#ifndef NDEBUG
		std::cout << boost::format("LOAD TIME: %1%s.") % t.elapsed() << std::endl;
		#endif
		return true;
	}
	const tPairV& computeNN(const std::vector<tType>& pFeat)
	{
		#ifndef NDEBUG
		boost::timer t;
		t.restart();
		#endif
		this->nndist.clear();
		std::vector<int> lIndices(mK);
		std::vector<float> lDists(mK);
		{
			mIndex->knnSearch(pFeat,lIndices,lDists,mK,cv::flann::SearchParams()); // internally converts vectors to mat
			for (int i=0;i<mK;i++)
			{
				tUnsDouble nn(lIndices[i],sqrt(lDists[i]));
				this->nndist.push_back(nn);
			}
		}
		#ifndef NDEBUG
		std::cout << boost::format("QUERY TIME: %1%s.") % t.elapsed() << std::endl;
		#endif
		return this->nndist;
	}
};
#endif