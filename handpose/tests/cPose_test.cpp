#define BOOST_TEST_DYN_LINK
#define BOOST_TEST_MODULE CPOSE
#include <boost/test/unit_test.hpp>
#include <iostream>

#include "cPose.h"
#include "cDB.h"

typedef std::pair<unsigned,double> tUnsDbl;

BOOST_AUTO_TEST_CASE(pose)
{
	CDB lTstDB("/home/javier/handDB_5stages/handsLechuck.db");
	tUnsDbl lTstIndexDist(17,1.9);
	tAcc_MinVar lAcc_FeatD,lAcc_PoseD;
	tPoseV lTstPose;
	lTstPose << 0.282341,0.0334796,-0.95873,-0.616635,0.771911,-0.15464,0.734877,0.634848,0.238587,
									-0.002793,0.001047,-0.059306,0.083678,0.018150,0.042573,0.000000,0.000000,-0.041876,
									0.051290,0.015009,0.000000,0.000000,-0.076719,0.066274,-0.055822,0.000000,0.000000,
									-0.081939,0.128796,0.541327,0.178373,-0.384070,0.138136,-0.055124;
	CPose lPose(lTstIndexDist,lAcc_FeatD,lTstPose,lAcc_PoseD,lTstDB);
	std::cout << lPose << std::endl;
	
	unsigned lDifference=0;
	BOOST_TEST_MESSAGE("checking pose class");
	BOOST_CHECK(lDifference<0.01);
}
