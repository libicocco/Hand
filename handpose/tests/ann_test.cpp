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
#define BOOST_TEST_DYN_LINK
#define BOOST_TEST_MODULE ANN
#include <boost/test/unit_test.hpp>
#include <iostream>
#include <fstream>
#include <stdlib.h>
#include <string>
#include <algorithm>

#include "nn.h"
//#include "approxNN.h"
#include "approxNNflann.h"
#include "approxNNlshkit.h"
#include "exactNN.h"
#include "typeDefinitions.h"

static const int gNFeatures=10000;
static const int gDim=100;
static const std::string gPathData="/tmp/data.bin";
static const std::string gPathDataLSH="/tmp/dataLSH.bin";

BOOST_AUTO_TEST_CASE(approximatedNearestNeighbors)
{
	srand(40000);
	float *lData=new float[gDim*gNFeatures];
	for(int i=0;i<gDim*gNFeatures;i++)
		lData[i]=static_cast<float>(rand());
	
	std::ofstream lO(gPathData,std::ofstream::out|std::ofstream::binary);
	lO.write(reinterpret_cast<char*>(lData),gNFeatures*gDim*sizeof(float));//FIXME: why reiterpret?
	lO.close();
	
	unsigned lSize(sizeof(float)),lRows(gNFeatures),lCols(gDim);
	std::ofstream lOLSH(gPathDataLSH,std::ofstream::out|std::fstream::binary);
	lOLSH.seekp(sizeof(unsigned)*3,std::ios::beg);
	lOLSH.write(reinterpret_cast<char*>(lData),gNFeatures*gDim*sizeof(float));
	lOLSH.seekp(0,std::ios::beg);
	lOLSH.write((const char*)&lSize,sizeof(unsigned)); 
	lOLSH.write((const char*)&lRows,sizeof(unsigned)); 
	lOLSH.write((const char*)&lCols,sizeof(unsigned)); 
	lOLSH.close();
	
	nn<float>* lNN;
	//     approxNNflann lFlann(1,106920,1024,"logDistT.bin","logDistTFlann.index");
	exactNN<float> lExactNN(5,gNFeatures,gDim,gPathData.c_str());
	lNN = static_cast<nn<float>*>(&lExactNN);
	
	std::vector<float> lFeat(gDim);
	int lTestFeat=78;
	std::copy(lData+lTestFeat*100,lData+(lTestFeat+1)*100,lFeat.begin());
	tPairV lNNlistExact = lNN->computeNN(lFeat);
	for(auto it=lNNlistExact.begin();it!=lNNlistExact.end();++it)
		std::cout << it->first << "," << it->second << std::endl;
	
	BOOST_TEST_MESSAGE("checking exact nearest neighbors");
	BOOST_CHECK(lTestFeat==lNNlistExact.front().first);
	
	approxNNflann<float> lApproxNNflann(5,gNFeatures,gDim,gPathData.c_str(),"/tmp/index.flann",0.1,0.999);
	lNN=static_cast<nn<float>*>(&lApproxNNflann);
	tPairV lNNlist= lNN->computeNN(lFeat);
	auto itExact=lNNlistExact.begin();
	int lDifference=0;
	for(auto it=lNNlist.begin();it!=lNNlist.end() && itExact!=lNNlistExact.end();++it,++itExact)
	{
		lDifference+=fabs(it->first-itExact->first);
		std::cout << it->first << "," << it->second << std::endl;
	}
	
	BOOST_TEST_MESSAGE("checking flann nearest neighbors");
	BOOST_CHECK(lDifference==0);
	
	std::cout << "creating lshkit" << std::endl;
	approxNNlshkit<float> lApproxNNlshkit(5,gNFeatures,gDim,gPathDataLSH.c_str(),"/tmp/index.lshkit",50,100);
	std::cout << "finished lshkit" << std::endl;
	lNN=static_cast<nn<float>*>(&lApproxNNlshkit);
	lNNlist.clear();
	lNNlist=lNN->computeNN(lFeat);
	
	itExact=lNNlistExact.begin();
	lDifference=0;
	for(auto it=lNNlist.begin();it!=lNNlist.end() && itExact!=lNNlistExact.end();++it,++itExact)
	{
		lDifference+=fabs(it->first-itExact->first);
		std::cout << it->first << "," << it->second << std::endl;
	}
	
	BOOST_TEST_MESSAGE("checking lshkit nearest neighbors");
	BOOST_CHECK(lDifference==0);
}
