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
#define BOOST_TEST_MODULE CPOSELISTMULTI
#include <boost/test/unit_test.hpp>
#include <iostream>
#include <stdlib.h>
#include "utils.h"

#include "cDB.h"
#include "cDB.h"

#define protected public /** <@attention I want to test protected classes **/
#define private public /** <@attention I want to test protected classes **/
#include "cPoselistMulti.h"
#undef protected 
#undef private 

typedef std::pair<unsigned,double> tUnsDbl;

// reset: cleans and fills accumulators and pose list 
// this test checks private and protected routines and data, so it doesn't show how to use the routing
BOOST_AUTO_TEST_CASE(reset)
{
// 	tPriorityPathQ lPaths;
// 	find_file("/home/javier/handDB_5stages","\\d{3}_\\d{4}\\.png",lPaths);
	srand(19819);
	CPoselistMulti lTstList("/home/javier/hand/scene/hands.db");;
	tPairV lNNN;
	unsigned lListLength=5;
	for(int i=0;i<lListLength;++i)
		lNNN.push_back(std::pair<unsigned,double>(rand()%(NPOSES*NREALGRASPS),static_cast<double>(rand())/32000.0));
	lTstList.reset(lNNN);
	double lDifference=0;
	lDifference+=(acc::variance(lTstList.mAccFeatD)==0); // variance shouldn't be zero unless all poses are the same
	lDifference+=(acc::variance(lTstList.mAccPoseD)!=0); // in the first iteration, all pose distances should be 0
	const tPoseVP lTstPL=lTstList.getPoselist();
	for(int i=0;i<lTstPL.size();++i)
	{
// 		lDifference+=(lPaths[lNNN[i].first]!=lTstPL[i].getImagePath()); // the path extracted corresponds to the one in disc
		lDifference+=(lNNN[i].second-lTstPL[i].mFeatD); // the feature distance got recorded well
	}
	lDifference+=(lListLength-lTstPL.size()); // all the poses are there, and not more (it removed previous ones)
	
	BOOST_TEST_MESSAGE("checking reset results, first frame");
	BOOST_CHECK(fabs(lDifference)<0.01);
	
	lNNN.clear();
	for(int i=0;i<lListLength;++i)
		lNNN.push_back(std::pair<unsigned,double>(rand()%(NPOSES*NREALGRASPS),static_cast<double>(rand())/32000.0));
	lTstList.reset(lNNN);
	
	lDifference=0;
	lDifference+=(acc::variance(lTstList.mAccFeatD)==0); // variance shouldn't be zero unless all poses are the same
	lDifference+=(acc::variance(lTstList.mAccPoseD)!=0); // the variance should still be zero since pWPose haven't changed inside reset
	const tPoseVP lTstPL2=lTstList.getPoselist();
	for(int i=0;i<lTstPL2.size();++i)
	{
// 		lDifference+=(lPaths[lNNN[i].first]!=lTstPL2[i].getImagePath()); // the path extracted corresponds to the one in disc
		lDifference+=(lNNN[i].second-lTstPL2[i].mFeatD); // the feature distance got recorded well
	}
	lDifference+=(lListLength-lTstPL2.size()); // all the poses are there, and not more (it removed previous ones)
	
	BOOST_TEST_MESSAGE("checking reset results, second frame");
	BOOST_CHECK(fabs(lDifference)<0.01);
}

// reduceNNN: sorts the list, reduces its size and computes the new weighted pose mWPose
BOOST_AUTO_TEST_CASE(reduceNNN)
{
	srand(12314);
	CPoselistMulti lTstList("/home/javier/hand/scene/hands.db");;
	tPairV lNNN;
	unsigned lListLength=5;
	unsigned lReducedLength=2;
	double lDifference=0;
	for(int i=0;i<lListLength;++i)
		lNNN.push_back(std::pair<unsigned,double>(rand()%(NPOSES*NREALGRASPS),static_cast<double>(lListLength-i)));
	lTstList.reset(lNNN); // I need to create the poses in the poselist first
	lTstList.setWeights();// I need to set weights in order to check the sorting inside reduceNNN
	tPoseV lWPose(tPoseV::Zero());
	// best poses are the last ones since they have the lowest distance
	for(int i=0;i<lReducedLength;++i)
	{
		CPose lTmp=lTstList.getPoselist()[lListLength-i-1];
		lWPose+=lTmp.getWeight()*lTmp.getPose();
	}
	lWPose/=lReducedLength;
	
	lTstList.reduceNNN(lReducedLength);
	// checking that the weighted pose is only created by the best lReducedLength poses
	lDifference+=(lWPose-lTstList.getWPose()).norm();
	
	const tPoseVP lTstPL=lTstList.getPoselist();
	lDifference+=(lReducedLength-lTstPL.size());
	BOOST_TEST_MESSAGE("checking reduceNNN");
	BOOST_CHECK(fabs(lDifference)<0.01);
}

// computeWeightedPose: computes weighted average of the poselist
BOOST_AUTO_TEST_CASE(computeWeightedPose)
{
	double lDifference(0);
	// don't know how to test this without doing the same as it's done in the routine
	BOOST_TEST_MESSAGE("checking computeWeightedPose: testing is trivial");
	BOOST_CHECK(fabs(lDifference)<0.01);
}

// saveBestPoses: save the poses and weights to be ready for next frame KDE evaluation
BOOST_AUTO_TEST_CASE(saveBestPoses)
{
	double lDifference(0);
	BOOST_TEST_MESSAGE("TODO: checking saveBestPoses");
	BOOST_CHECK(fabs(lDifference)<0.01);
}
// setWeights(medium-size function): if empty poselist, sets the poseW to zero;else, performs KDE weighting based on previous poselist. Then it normalizes 

BOOST_AUTO_TEST_CASE(setWeights)
{
	double lDifference(0);
	BOOST_TEST_MESSAGE("TODO: checking setWeights");
	BOOST_CHECK(fabs(lDifference)<0.01);
}

// interpolatePoses: big function, given a list of pair does all the stuff
BOOST_AUTO_TEST_CASE(interpolatePoses)
{
	srand(19819);
	CPoselistMulti lTstList("/home/javier/hand/scene/hands.db");;
	tPairV lNNN;
	for(int i=0;i<5;++i)
		lNNN.push_back(std::pair<unsigned,double>(rand()%(NPOSES*NREALGRASPS),static_cast<double>(rand())/32000.0));
	lTstList.interpolatePoses(lNNN);
	std::cout << lTstList << std::endl;
// 	const double *lBestPoses=lTstList.getBestPoses(); //deallocated when the next frame comes
// 	const double *lBestWeights=lTstList.getBestWeights(); //deallocated when the next frame comes
// 	unsigned lNFinalNeighbors=lTstList.getNFinalNeighbors();
// 	for(int i=0;i<lNFinalNeighbors;++i)
// 	{
// 		for(int j=0;j<NJOINTS+NORI;++j)
// 			std::cout << lBestPoses[i*(NJOINTS+NORI)+j] << " ";
// 		std::cout << ": " << lBestWeights[i] << std::endl;
// 	}
	
// 	lNNN.clear();
// 	for(int i=0;i<5;++i)
// 		lNNN.push_back(std::pair<unsigned,double>(rand()%(NPOSES*NREALGRASPS),static_cast<double>(rand())/32000.0));
	lTstList.interpolatePoses(lNNN);
	std::cout << lTstList << std::endl;
	
	unsigned lDifference=0;
	BOOST_TEST_MESSAGE("checking pose class");
	BOOST_CHECK(lDifference<0.01);
}