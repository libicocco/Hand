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
#include "exactNN.h"

int main(int argc,char *argv[])
{
  if(argc!=2)
  {
    std::cerr << "command index" << std::endl;
    exit(-1);
  }
  unsigned lIndex = atoi(argv[1]);
  
  CDB lTstDB("/usr/local/src/handDB_5stages/hands.db");
  CDBelement lElem=lTstDB.query(lIndex);
  
  cv::Mat lIm = cv::imread(lElem.getImagePath());
  std::cout << lElem.getImagePath() << std::endl;
  tMatMat lImMask;
  CHandTracker lHT;
  lHT.getHand(lIm,lImMask);
  
  Hog<float> lHog;
  Feature<float> *lFeat = &lHog;
  lFeat->compute(lImMask,INVPROBDEFAULT);
  //std::cout << *lFeat << std::endl;
  //std::cout << "---------------------------" << std::endl;
  
  static const unsigned lFeatDim = 512;
  static const unsigned lNPoints = 106920;
  float *lData = new float[lNPoints*lFeatDim];
  //std::string mDataPath = "/usr/local/src/handDB_5stages/log_4_8_5stages.HOGnew.bin";
  std::string lDataPath = "/usr/local/src/handDB_5stages/hog_refactoring2.bin";
  std::ifstream lS(lDataPath.c_str(),std::ifstream::in|std::ifstream::binary);
  if(lS.fail())
  {
    std::cout << "PROBLEMS OPENING DATA FILE " << lDataPath << std::endl;
    return 0;
  }
  lS.read((char*)lData,lFeatDim*lNPoints*sizeof(float));
  std::vector<float> lDataV;
  std::copy(&lData[lIndex*lFeatDim],&lData[(lIndex+1)*lFeatDim],std::back_inserter(lDataV));
  //for(int i=0;i<lFeatDim;++i)
  //  std::cout << setiosflags(std::ios::fixed) << std::setw(3) << std::setprecision(3) << lDataV[i] << " ";
  //std::cout << std::endl;
  
  exactNN<float> lNN(20,lNPoints,lFeatDim,lDataPath.c_str());
  nn<float> *lNNptr=&lNN;
  //std::copy(lDataV.begin(),lDataV.end(),std::ostream_iterator<float>(std::cout," "));
  std::vector<std::pair <unsigned int, double > > lRes = lNNptr->computeNN(lDataV);
  //std::cout << "nn(20," << lNPoints << "," << lFeatDim << "," << lDataPath << " ):" << lRes.front().first << ":" << lRes.front().second << std::endl;
  std::cout << "nn : " << lRes.front().first << ":" << lRes.front().second << std::endl;
  
  cv::Mat lHogIm0(200,200,CV_8UC1,cv::Scalar(255));
  cv::Mat lHogIm1(200,200,CV_8UC1,cv::Scalar(255));
  lFeat->draw(lHogIm0);
  cv::imshow("computed hog",lHogIm0);
  drawhog<float>(4,8,lDataV,lHogIm1);
  cv::imshow("hog db",lHogIm1);
  cv::imshow("im",lIm);
  cv::waitKey();
}
