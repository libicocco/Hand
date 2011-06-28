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
