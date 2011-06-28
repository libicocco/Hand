#include <iostream>
#include <fstream>
#include <stdlib.h>
#include <string>
#include <algorithm>
#include <iterator>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "cDB.h"
#include "nn.h"
#include "approxNNflann.h"
#include "approxNNlshkit.h"
#include "exactNN.h"
#include "typeDefinitions.h"

enum lTypes{FEAT,JOINTS,PARTS,NTYPES};
static const int gDims[3]={512,34,30};
static const int gDimsPadded[3]={512,36,32};
static const int gNFeatures=150000;
static const int gNNN=50;
static const int gRows=100;

int main(int argc,char *argv[])
{
  CDB lDB("/home/javier/hand/scene/hands.db");
  float *lData[NTYPES];
  for(int i=0;i<NTYPES;++i)
    lData[i]=new float[gNFeatures*gDimsPadded[i]];
  float *lDataP[NTYPES]={lData[FEAT],lData[JOINTS],lData[PARTS]};
  
  for(int i=0;i<gNFeatures;++i)
  {
    CDBelement lElem=lDB.query(i);
    
    std::vector<float> lFeatV=lElem.getFeature();
    for(int j=0;j<gDims[FEAT];++j)
      *(lDataP[FEAT]++)=lFeatV[j];
    
    tPoseV lJointsV=lElem.getPose();
    for(int j=0;j<gDims[JOINTS];++j)
      *(lDataP[JOINTS]++)=lJointsV[j];
    *(lDataP[JOINTS]++)=0.0;*(lDataP[JOINTS]++)=0.0;
    
    tPartsV lPartsV=lElem.getPartsLocationV();
    for(int j=0;j<gDims[PARTS];++j)
      *(lDataP[PARTS]++)=lPartsV[j];
    *(lDataP[PARTS]++)=0.0;*(lDataP[PARTS]++)=0.0;
  }
  
  
  std::cout << "Enter index to test" << std::endl;
  int lTestIndex=23239;
  exactNN<float>* lNN[NTYPES];
  for(int i=0;i<NTYPES;++i)
    lNN[i]=new exactNN<float>(gNNN,gNFeatures,gDimsPadded[i],lData[i]);
  while(std::cin >> lTestIndex)
  {
    cv::Mat lIm(gRows*NTYPES,gRows*gNNN,CV_8UC3);
    for(int i=0;i<NTYPES;++i)
    {
      std::vector<float> lTest(gDimsPadded[i]);
      std::copy(lData[i]+lTestIndex*gDimsPadded[i],lData[i]+(lTestIndex+1)*gDimsPadded[i],lTest.begin());
      tPairV lNNlistExact= lNN[i]->computeNN(lTest);
      for(int j=0;j<lNNlistExact.size();++j)
      {
        CDBelement lElem=lDB.query(lNNlistExact[j].first);
        cv::Mat lMat=cv::imread(lElem.getImagePath());
        cv::Rect lBB(gRows*j,gRows*i,gRows,gRows);
        cv::Mat lResult=lIm(lBB);
        cv::resize(lMat,lResult,lBB.size());
        std::cout << lNNlistExact[j].first << "," << lNNlistExact[j].second << std::endl;
      }
    }
    cv::imshow("best neighbors",lIm);
    cv::waitKey();
    
    //    int lConflictingIndex=822;
    //   std::copy(lData[PARTS]+gDimsPadded[PARTS]*lConflictingIndex,lData[PARTS]+gDimsPadded[PARTS]*(lConflictingIndex+1),std::ostream_iterator<float>(std::cout," "));
    //   std::cout << std::endl;
    //   lConflictingIndex=154;
    //   std::copy(lData[PARTS]+gDimsPadded[PARTS]*lConflictingIndex,lData[PARTS]+gDimsPadded[PARTS]*(lConflictingIndex+1),std::ostream_iterator<float>(std::cout," "));
    //   std::cout << std::endl;
      std::cout << "Enter index to test" << std::endl;
  }
  
  for(int i=0;i<NTYPES;++i)
  {
    delete lNN[i];
    delete []lData[i];
  }
}