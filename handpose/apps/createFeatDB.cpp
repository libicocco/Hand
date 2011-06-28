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
#include <iostream>
#include <fstream>
#include "cDB.h"
#include "cDBelement.h"
#include "hog.h"
#include "chandtracker.h"

int main(int argc,char *argv[])
{
  if(argc!=3)
  {
    std::cerr << "command numfeats dbpath" << std::endl;
    exit(-1);
  }
  unsigned lNFeats = atoi(argv[1]);
  
  CDB lTstDB("/usr/local/src/handDB_5stages/hands.db");
  std::ofstream lS(argv[2],std::iostream::out|std::iostream::binary);
  //std::ofstream lStext("test.txt",std::iostream::out|std::iostream::binary);
  static const unsigned lFeatDim = 512;
  float *lFeatArray = new float[lFeatDim];
  for(int i=0;i<lNFeats;++i)
  {
    CDBelement lElem=lTstDB.query(i);
    cv::Mat lIm = cv::imread(lElem.getImagePath());
    std::cout << lElem.getImagePath() << std::endl;
    tMatMat lImMask;
    CHandTracker lHT;
    lHT.getHand(lIm,lImMask);
    
    Hog<float> lHog;
    Feature<float> *lFeat = &lHog;
    lFeat->compute(lImMask,999999999999);
    std::vector<float> lFeatV = lFeat->getFeat();
    //std::copy(lFeatV.begin(),lFeatV.end(),std::ostream_iterator<float>(lStext," "));
    std::copy(lFeatV.begin(),lFeatV.end(),lFeatArray);
    lS.write((char*)lFeatArray,lFeatDim*sizeof(float));
  }
  delete[] lFeatArray;
  lS.close();
  //lStext.close();
}