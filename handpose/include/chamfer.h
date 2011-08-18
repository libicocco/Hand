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

#ifndef CHAMFERNN
#define CHAMFERNN

#include <vector>
#include <algorithm>
#include <vector>
#include <fstream>
#include <boost/format.hpp>
#include <boost/progress.hpp>

#include "constants.h"
#include "nn.h"
typedef std::pair<unsigned short,unsigned short> tPairUShort;
typedef float tBinType; 
class chamferNN: public nn<tPairUShort>
{
    private:
        int mK;
        bool mInitialized;
        const char* mDataPath;
        tBinType *mData;
        unsigned short mCols;
        std::vector<std::pair<int,float> >mNNdist_all;
    public:
        // pIndexPath is included in order to have a common signature with indexed approxNN like flann
        chamferNN(int pK,int pNPoints,int pDimPoints,const char *pDataPath,const char *pIndexPath="ignored"):mK(pK),mInitialized(false),mDataPath(pDataPath){this->nn_set_values(pNPoints,pDimPoints,mDataPath);initialize();} /**< @todo data.bin and data.idx should be configured with cmake*/
        ~chamferNN(){delete []mData;}

        bool initialize()
        {
#ifndef NDEBUG
            std::cout << "LOADING DATA..." << std::endl;
            boost::timer t;
            t.restart();
#endif
            mCols = (unsigned short)sqrt(this->pointsDimension);
            mData = new tBinType[this->nPoints*this->pointsDimension];
            std::ifstream lS(mDataPath,std::ifstream::in|std::ifstream::binary);
            if(lS.fail())
            {
                std::cout << "PROBLEMS OPENING DATA FILE " << mDataPath << " for aNN" << std::endl;
                return 0;
            }
            lS.read((char*)mData,this->nPoints*this->pointsDimension*sizeof(tBinType));

#ifndef NDEBUG
            std::cout << boost::format("LOAD TIME: %1%s.") % t.elapsed() << std::endl;
#endif

#ifndef NDEBUG
            std::cout << "LOADING INDEX..." << std::endl;
            t.restart();
#endif
            mNNdist_all.reserve(this->nPoints);
            mNNdist_all.resize(this->nPoints);

#ifndef NDEBUG
            std::cout << boost::format("LOAD TIME: %1%s.") % t.elapsed() << std::endl;
#endif
            //        std::cout << "INDEX COULDN'T BE LOADED" << std::endl;
            //        return 0;
            return true;
        }// i had to do it in the constructor
        const std::vector<std::pair<int,float> >& computeNN(const std::vector<tPairUShort>& pFeat)
        {
#ifndef NDEBUG
            boost::timer t;
            t.restart();
#endif
//            for(int k=0;k<this->pointsDimension;k++)
//                std::cout << mData[k] << " \t";
//            std::cout << std::endl;
            for(int i=0;i<this->nPoints;i++)
            {
                tBinType *lP1=mData+this->pointsDimension*i;

                float lDist = 0;
                for(int j=0;j<pFeat.size();j++)
                {
                    // the feature advances in x first, then i!
                    unsigned short lShift = pFeat[j].first+pFeat[j].second*mCols;
                    //unsigned short lShift = pFeat[j].first*mCols+pFeat[j].second;
//                    if(i==0)
//                    {
//                        std::cout << "mCols=" << mCols << std::endl;
//                        std::cout << pFeat[j].first << "," << pFeat[j].second << std::endl;
//                        std::cout << "lShift=" << lShift << std::endl;
//                        std::cout << "lP1[" << lShift << "]=" << lP1[lShift] << std::endl;
//                    }
                    if(lShift>=this->pointsDimension)
                    {
                        std::cerr << "Edge point (" << i << "," << j << ")=" << lShift << 
                            " > " << this->pointsDimension << " is out of feature; probably the feature wasn't resized properly" << std::endl;
                        exit(-1);
                    }
                    lDist += lP1[lShift];
                }

                std::pair<int,float> lPair = std::pair<int,float>(i,lDist);
                //std::cout << lPair.first << "," << lPair.second << std::endl;
                mNNdist_all[i]= lPair;
                //lR[i]=std::pair<int,float>(i,mDataData[i]-2*(lResult[0]+lResult[1]+lResult[2]+lResult[3]));
            }
            std::sort(mNNdist_all.begin(),mNNdist_all.end(),comppair);
            this->nndist.reserve(mK);
            this->nndist.resize(mK);
            std::copy(mNNdist_all.begin(),mNNdist_all.begin()+mK,this->nndist.begin());
            //    std::vector<std::pair<int,float> >::iterator lIt;
            //    for(lIt=this->nndist.begin();lIt!=this->nndist.end();++lIt)
            //        std::cout << "( " << lIt->first << "," << lIt->second << ")\t";
            //    std::cout << std::endl;
            return this->nndist;
        }
};

#endif
