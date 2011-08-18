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

#ifndef APPROXNNFLANN
#define APPROXNNFLANN

//#include <unistd.h>
#include <chrono>
#include <opencv2/core/core.hpp>
#include <opencv2/flann/flann.hpp>
#include <vector>
#include <iostream>
#include <fstream>
#include "nn.h"


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
  approxNNflann(const int pK,const int pNPoints,const int pDimPoints,const char *pDataPath,const char *pIndexPath,
                const float pPrecision=0.9,const float pBuildWeight=0.1):
								mK(pK),mInitialized(false),mDataPath(pDataPath),mIndexPath(pIndexPath),mPrecision(pPrecision),mBuildWeight(pBuildWeight)
								{this->nn_set_values(pNPoints,pDimPoints,mDataPath);initialize();} /**< @todo data.bin and data.idx should be configured with cmake*/
	~approxNNflann(){delete mIndex;delete mData;}
	
	bool initialize()
	{
		#ifndef NDEBUG
    namespace sc = std::chrono;
		std::cout << "LOADING DATA..." << std::endl;
    auto lSinceEpoch = sc::system_clock::now().time_since_epoch();
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
		std::cout << "LOAD TIME: " << 
      sc::duration_cast<sc::microseconds>(sc::system_clock::now().time_since_epoch() - lSinceEpoch).count() << " us"  << std::endl;
		#endif
		
		#ifndef NDEBUG
		std::cout << "LOADING INDEX..." << std::endl;
    lSinceEpoch = sc::system_clock::now().time_since_epoch();
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
		std::cout << "LOAD TIME: " << 
      sc::duration_cast<sc::microseconds>(lSinceEpoch).count() << " us"  << std::endl;
		#endif
		return true;
	}
	const tPairV& computeNN(const std::vector<tType>& pFeat)
	{
		#ifndef NDEBUG
    namespace sc = std::chrono;
    auto lSinceEpoch = sc::system_clock::now().time_since_epoch();
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
		std::cout << "QUERY TIME: " << 
      sc::duration_cast<sc::microseconds>(lSinceEpoch).count() << " us"  << std::endl;
		#endif
		return this->nndist;
	}
};
#endif
