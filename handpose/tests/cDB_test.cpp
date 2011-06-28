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
#define BOOST_TEST_MODULE CDB
#include <boost/test/unit_test.hpp>
#include <boost/format.hpp>
#include <iostream>
#include <fstream>
#include <stdlib.h>
#include <string>
#include <algorithm>

#include "cDB.h"
#include "cDBelement.h"
static const int gPosesPerGrasp(3240);
BOOST_AUTO_TEST_CASE(database)
{
  unsigned lIndex(9);
  std::string lExpImagePath(boost::str(boost::format("/usr/local/src/handDB_5stages/%1$03d/%1$03d_%2$04d.info")
  %((lIndex/gPosesPerGrasp)+1)%(lIndex%gPosesPerGrasp)));
  const std::vector<double> lExpPose={0.282341,0.0334796,-0.95873,-3.46945e-18,0.999391,0.0348995,
  0.959314,-0.00985357,0.282169,-0.002793,0.001047,-0.059306,0.083678,0.018150,0.042573,
  0.000000,0.000000,-0.041876,0.051290,0.015009,0.000000,0.000000,-0.076719,0.066274,-0.055822,
  0.000000,0.000000,-0.081939,0.128796,0.541327,0.178373,-0.384070,0.138136,-0.055124}; 
//   const int lExpGraspPhase(2);
//   const int lExpGraspClass(1);
  // 	const std::vector<double> lExpParts={0.600095,0.729976,-0.069424,0.410096,0.709988,-0.070371,0.355878,
  // 	0.652628,0.040399,0.322968,0.603962,0.147864,0.274466,0.536379,0.229210,0.118351,0.767635,0.097443,
  // 	-0.029824,0.737564,0.127674,-0.129163,0.681579,0.136557,0.085278,0.762768,0.001097,-0.069967,0.712080,
  // 	0.024114,-0.145676,0.617683,0.040230,0.072643,0.738440,-0.097129,-0.052462,0.683732,-0.067872,-0.112411,
  // 	0.595396,-0.045074,0.103221,0.686044,-0.179710,0.002285,0.656752,-0.160826,-0.055695,0.598878,-0.145196};
  const std::vector<double> lExpNextIndices={9,657,1305,1953,2601};
  
  CDB lTstDB("/usr/local/src/handDB_5stages/hands.db");
  std::cout << "created db instance" << std::endl;
  CDBelement lElem=lTstDB.query(lIndex);
  
  tPoseV lPose(lElem.getPose());
  std::vector<unsigned> lNextIndices(lElem.getNextIndicesV());
  std::string lImagePath(lElem.getImagePath());

//    //unsigned lGraspPhase(lElem.getGraspPhase());
//    //unsigned lGraspClass(lElem.getGraspCode());
//   
//    std::cout << "Index: " << lIndex << std::endl;
//    std::cout << "Pose: ";
//    std::cout << lPose << std::endl;
//    std::cout << std::endl << "Next Indices: ";
//    std::copy( lNextIndices.begin(), lNextIndices.end(), std::ostream_iterator<double>(std::cout," "));
//    std::cout << std::endl << "Image Path: " << lImagePath << std::endl;;
//    //std::cout << "Grasp Phase: " << lGraspPhase << std::endl;
//    //std::cout << "Grasp Class: " << lGraspClass << std::endl;
//    exit(1);
  
  double lDifference(0); //FIXME: check differences in some way
  //if(lExpImagePath==lImagePath && lExpGraspPhase==lGraspPhase && lExpGraspClass==lGraspClass)
//   if(lExpGraspPhase==lGraspPhase && lExpGraspClass==lGraspClass)
//   {
    for(int i=0;i<34;++i)
      lDifference+=fabs(lExpPose[i]-lPose[i]);
  std::cout << lDifference << std::endl;
//    for(int i=0;i<5;++i)
//      lDifference+=fabs(lExpNextIndices[i]-lNextIndices[i]);
//   }
//   else
//     lDifference=1;
  
  BOOST_TEST_MESSAGE("checking database");
  BOOST_CHECK(fabs(lDifference)<0.01);
  std::cout << lDifference << std::endl;
  
  fsystem::path lTstDBPath("/tmp/hands.db");
  CDB lTstDBNew(lTstDBPath,"/usr/local/src/handDB_5stages","001_00\\d{2}\\.info");
  //CDB lTstDBNew(lTstDBPath,"/usr/local/src/handDB_5stages","\\d{3}_\\d{4}\\.info");
  CDBelement lElemNew=lTstDBNew.query(lIndex);
  
  lPose=lElemNew.getPose();
  lNextIndices=lElemNew.getNextIndicesV();
  lImagePath=lElemNew.getImagePath();
//   lGraspPhase=lElemNew.getGraspPhase();
//   lGraspClass=lElemNew.getGraspCode();
  
//    std::cout << "Index: " << lIndex << std::endl;
//    std::cout << "Pose: ";
//    std::cout << lPose << std::endl;
//    std::cout << std::endl << "Next Indices: ";
//    std::copy( lNextIndices.begin(), lNextIndices.end(), std::ostream_iterator<double>(std::cout," "));
//    std::cout << std::endl << "Image Path: " << lImagePath << std::endl;;
//    std::cout << "Grasp Phase: " << lGraspPhase << std::endl;
//    std::cout << "Grasp Class: " << lGraspClass << std::endl;
  
  lDifference=0; //FIXME: check differences in some way
//   if(lExpGraspPhase==lGraspPhase && lExpGraspClass==lGraspClass)
//   {
    for(int i=0;i<34;++i)
      lDifference+=fabs(lExpPose[i]-lPose[i]);
  std::cout << lDifference << std::endl;
    for(int i=0;i<5;++i)
      lDifference+=fabs(lExpNextIndices[i]-lNextIndices[i]);
  std::cout << lDifference << std::endl;
//   }
//   else
//     lDifference=1;
  
  BOOST_TEST_MESSAGE("checking new database");
  BOOST_CHECK(fabs(lDifference)<0.01);
  //fsystem::remove(lTstDBPath);
}
