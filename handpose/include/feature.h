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

#ifndef FEATUREH
#define FEATUREH

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <vector>
#include <iostream>
#include <iomanip>
#include "constants.h"

template<typename tType1,typename tType2> std::ostream& printValue(std::pair<tType1,tType2> pValue,std::ostream& pOutput)
{pOutput << setiosflags(std::ios::fixed) << std::setw(3) << std::setprecision(3) << pValue.first << " "<< pValue.second << " \t";return pOutput;}
template<typename tType> std::ostream& printValue(tType pValue,std::ostream& pOutput)
{pOutput << setiosflags(std::ios::fixed) << std::setw(4) << std::setprecision(4)<< pValue << "\t";return pOutput;}

/**
 * @brief Class for computing any feature, given different types of inputs
 * HOG and others should inherit from them 
 */
template<typename tType>
class Feature
{
	friend inline std::ostream& operator << (std::ostream& pOutput, const Feature<tType>& pFeat) { pFeat.printVector(pOutput);return pOutput;}
private:
	// the print should be late-binded for edges, but << cannot (it's not a member)
	// and the templates printValue cannot be virtual
	virtual std::ostream& printVector(std::ostream& pOutput) const
	{
		for(int i=0;i<NCELLS;++i)
		{
			for(int j=0;j<NCELLS;++j)
				printValue(mFeat[i*NCELLS+j],pOutput);
			pOutput << std::endl;
		}
		pOutput << std::endl;
		return pOutput;
	}
public:
	Feature():mFrame(0){}
	virtual ~Feature(){};
	/**
	 * @brief Public function for computing the feature: acts as an interface to the 
	 * function which actually computes it, doCompute
	 *
	 * @param pIm Image from which features should be extracted
	 * @param pMask if pMask(x,y)==0, pIm(x,y) should be ignored in the computation
	 * @param pInvProb if low, 1 out of pInvProb pixels from the mask will be removed
	 * @return :vector< tType >&
	 **/
	virtual const std::vector<tType>& compute(std::pair<cv::Mat,cv::Mat> &pImMask, const unsigned int pInvProb)
	{
		if(pInvProb!=INVPROBDEFAULT)
			worsenMask(pImMask.second, pInvProb);
		
		//cvAnd(mGray,mMask,mGray);
			return doCompute(pImMask);
	}
	/**
	 * @brief Introduces error in the hand segmentation, returning a worsen mask
	 *
	 * @param pMask In/out mask to be worsen
	 * @param invprob 1 out of invprob pixels will be deleted from the mask 
	 * @return void
	 **/
	void worsenMask(cv::Mat &pMask, const unsigned int invprob)
	{
		//std::cout << "Mask is being worsened" << std::endl;
		cv::MatIterator_<uchar> it = pMask.begin<uchar>(),
		it_end = pMask.end<uchar>();
		for(; it != it_end; ++it)
		{
			if(rand()%invprob==1)
				*it=(uchar)0;
		}
		cv::erode(pMask,pMask,cv::Mat());
		cv::erode(pMask,pMask,cv::Mat());
		cv::erode(pMask,pMask,cv::Mat());
		cv::dilate(pMask,pMask,cv::Mat());
		cv::dilate(pMask,pMask,cv::Mat());
		cv::dilate(pMask,pMask,cv::Mat());
	}
	
	/**
	 * @brief Draws the feature on image pFeatIm, and saves it in pFeatPath
	 *
	 * @param pFeatIm Image where the feature should be drawn
	 * @param pFeatPath ... Defaults to 0. Path where the feature can be optionally saved
	 * @return void
	 **/
	virtual void draw(cv::Mat &pFeatIm) const=0;
	int getFrame(){return mFrame;}
	std::vector<tType> getFeat() const{return mFeat;}
	virtual unsigned getFeatSize() const=0;
protected:
	/**
	 * @brief Frame number
	 **/
	int mFrame;
	/**
	 * @brief Vector of features to be computed
	 **/
	std::vector<tType> mFeat;
private:
	/**
	 * @brief Internal function that actually computes the features vector
	 * To be implemented by the children classes
	 *
	 * @param pIm Image from which features should be computed
	 * @param pMask Mask(x,y)=0 means pIm(x,y) should be ignored
	 * @return :vector< tType >& Vector of computed features
	 **/
	virtual const std::vector<tType>& doCompute(std::pair<cv::Mat,cv::Mat> &pImMask)=0;
};
#endif
