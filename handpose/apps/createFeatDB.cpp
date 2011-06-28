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