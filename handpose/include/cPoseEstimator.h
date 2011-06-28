#ifndef __CPOSEESTIMATOR_H_
#define __CPOSEESTIMATOR_H_

#include <boost/filesystem.hpp>
#include <queue>

#include "chandtracker.h"
#include "feature.h"
#include "hog.h"
#include "processFeat.h"
#include "approxNNflann.h"
#include "cTemporalFilter.h"
#include "cPoselistMulti.h"

#include "typeDefinitions.h"

#ifdef CAM_ACCESS
#include <unistd.h>
#include <oldrobot/ccorbacamclient.h>
#endif

namespace fsystem=boost::filesystem;

enum tReturnCode{OK,notReady,EndOfStream};

struct CGroundTruth
{
  std::queue<tPoseV> mInfoPosesQ;
  tPoseV mInputVar;
  operator bool(){return mInfoPosesQ.empty();}
  void getNextExpectedPose(tPoseV &pExpectedPose){pExpectedPose=mInfoPosesQ.front();mInfoPosesQ.pop();}
  // FIXME:this should be moved out, it's only adding mInputVar
  tPoseV init(tPriorityPathQ &pInfoPathQ)
  {
    std::vector<tAcc_Var> lPosesVar(NJOINTS+NORI);
    // parse each info file
    while(!pInfoPathQ.empty())
    {              
      fsystem::path lInfoPath=pInfoPathQ.top();
      pInfoPathQ.pop();
      static const unsigned lBufferSz=1024;
      char *lLine=new char[lBufferSz];
      std::fstream lFS(lInfoPath.string(),std::fstream::in|std::fstream::out);
      lFS.getline(lLine,lBufferSz);//comment
      tPoseV lPoseV;
      for(unsigned i=0;i<NORI;++i)
      {
        lFS>>lPoseV[i];
        lPosesVar[i](lPoseV[i]);
      }
      lFS.getline(lLine,lBufferSz);//end ori line
      lFS.getline(lLine,lBufferSz);//comment
      delete []lLine;
      for(unsigned i=NORI;i<NORI+NJOINTS;++i)
      {
        lFS>>lPoseV[i];
        lPosesVar[i](lPoseV[i]);
      }
      mInfoPosesQ.push(lPoseV);
    }
    for(int i=0;i<NJOINTS+NORI;++i)
      mInputVar[i]=acc::variance(lPosesVar[i]);
    return mInputVar;
  }
};

/* for dealing homogeneusly with cameras and files */
struct CCapture
{
  tPriorityPathQ mImPathPQ;
  bool mOffline;
  CGroundTruth mGroundTruth;
  tPoseV mInputVar;
  #ifdef CAM_ACCESS
  CCorbaCamClient mCam;
  buola::img::CImage_rgb8 mBuolaIm;
  #endif
  const tPoseV& getInputVar() const{return mInputVar;}
  tReturnCode getNextFrame(cv::Mat &pIm,tPoseV &pExpectedPose)
  {
    if(mOffline)
    { 
      if(!mImPathPQ.empty())
      {
        pIm=cv::imread(mImPathPQ.top().string());
        // 				std::cout << mImPathPQ.top() << std::endl;
        mImPathPQ.pop();
        mGroundTruth.getNextExpectedPose(pExpectedPose);
        return tReturnCode::OK;
      }
      else
        return tReturnCode::EndOfStream;
    }
    else
    {
      #ifdef CAM_ACCESS
      if(mCam.IsNextFrameReady(CCamClient::CAM_ALL))
      {
        mCam.CopyCurrentImage(mBuolaIm,CCamClient::CAM_WLEFT);
        // is there any way of assigning the data without constructing a new cv::Mat?
        pIm=cv::Mat(buola::cvi::ipl_wrap(view(mFLImage)),false);
        return tReturnCode::OK;
      }
      else
        return tReturnCode::notReady;
      #else
        std::cerr << "Recompile with CAM_ACCESS ON with ccmake" << std::endl;
        exit(-1);
        #endif
    }
  }
  CCapture(const fsystem::path &pImPath,std::string pPattern)
  {
    mOffline=find_file(pImPath,pPattern,mImPathPQ);
    // use cam if no jpg image found in pImPath
    if(!mOffline)
    {
      #ifdef CAM_ACCESS
      mCam.Init();
      #else
      std::cerr << "Recompile with CAM_ACCESS ON with ccmake" << std::endl;
      exit(-1);
      #endif
    }
    else // if there are images in pImPath, check if there are infos
		{
      tPriorityPathQ lInfoPathQ;
      if(find_file(pImPath,".*.info",lInfoPathQ))
        mInputVar=mGroundTruth.init(lInfoPathQ);
    }
  }
};

template <typename PL,typename tType>
class CPoseEstimator
{
public:
  
  CPoseEstimator(Feature<tType>* pFeat,ProcessFeat<PL,tType>* pProcFeat,
                 const fsystem::path &pImPath,const std::string &pImPattern,bool pGUI=false):
                 mWorsen(INVPROBDEFAULT),mCapture(pImPath,pImPattern),mFeat(pFeat),
                 mTracker(),mProcFeat(pProcFeat),mOfflineImages(false),mGT(false),mGUI(pGUI),
                 mDelay(0),mResult(480,640+320,CV_8UC3),mResultName("result")
                 {
                   if(mGUI)
                   {
                     cv::namedWindow(mResultName,CV_WINDOW_FREERATIO | CV_WINDOW_AUTOSIZE);
                     cv::createTrackbar("delay",mResultName,&mDelay,1000);
                   }
                 }
                 
                 /** Check if there is a new frame, process it and create images for showing. */
                 bool  DoProcessing()
                 {
                   cv::Mat lIm;
                   tPoseV lExpectedPose(tPoseV::Zero());
                   tReturnCode lRet=mCapture.getNextFrame(lIm,lExpectedPose);
                   // 		std::cout << "new frame captured" << std::endl;
                   if(lRet==tReturnCode::notReady)
                     return true;
                   else if(lRet==tReturnCode::EndOfStream)
                   {
                     //const tPoseV lInputVar=mCapture.getInputVar();
                     //tPoseV lMeanErrorV = mProcFeat->getMeanErrorV();
                     //tPoseV lVarErrorV = mProcFeat->getVarErrorV();
                     //assert(lMeanErrorV.size()==NORI+NJOINTS && lVarErrorV.size()==NORI+NJOINTS && lInputVar.size()==NORI+NJOINTS);
                     //double lWeightedMeanError = 0;
                     //for(int i=0;i<NORI+NJOINTS;i++)
                     //  lWeightedMeanError+=(fabs(lInputVar[i])<0.001)?lMeanErrorV[i]:lMeanErrorV[i]/sqrt(lInputVar[i]);
                     //lWeightedMeanError/=(NORI+NJOINTS);
                     //double lMeanError=lMeanErrorV.sum()/(NORI+NJOINTS);
                     //double lVarError=lVarErrorV.sum()/(NORI+NJOINTS);
                     //std::cout << "(mean,var,weighted mean) = " << lMeanError << " \t" <<  lVarError << " \t" <<  lWeightedMeanError << std::endl;
                     double lMeanError=mProcFeat->getMeanError()/(NORI+NJOINTS);
                     double lVarError=mProcFeat->getVarError()/(NORI+NJOINTS);
                     std::cout << "(mean,var) = " << lMeanError << " \t" <<  lVarError << std::endl;
                     exit(1);
                   }
                   else
                   {
                     std::pair<cv::Mat,cv::Mat> lImMaskCrop;
                     cv::Rect lResBox=mTracker.getHand(lIm,lImMaskCrop);
                     std::vector<tType> lVHog=mFeat->compute(lImMaskCrop,mWorsen);
                     mProcFeat->UpdatePoselist(lVHog,lExpectedPose);
                     //std::cout << "error so far: " << (mProcFeat->getMeanErrorV()).sum()/(NORI+NJOINTS) << std::endl;
                     tPoseV lWPose = mProcFeat->getPoselist()->getWPose();
                     
                     std::ofstream outfile;
                     outfile.open("logWPose", std::ios::app);
                     outfile << lWPose << std::endl;
                     outfile.close();
                     
                     // FIXME:display stuff
                     if(mGUI)
                     {
                       // construct mResult image
                       buildResultIm(lIm,lImMaskCrop.first);
                       // display it
                       DoExpose();
                     }
                     return true;
                   }
                 }
                 
                 /** If there are images, it shows them */
                 void DoExpose()
                 {
                   cv::imshow(mResultName,mResult);
                   cv::waitKey(mDelay);
                 }
                 
                 /** Run for non gui interfaces */
                 void Run(){while(DoProcessing()){}}
                 
private:
  void buildResultIm(const cv::Mat &pIm, const cv::Mat &pImCrop)
  {
    const int lPosesPerRow(4);
    const cv::Size lPoseSz(cv::Size(640/lPosesPerRow,480/lPosesPerRow));
    // 		const boost::ptr_vector<CPose> lPoselist(mProcFeat->getPoselist().getPoselist());
    const CPoselistMulti *lPLM(mProcFeat->getPoselist());
    const boost::ptr_vector<CPose> lPoselist(lPLM->getPoselist());
    for(int i=0;i<lPosesPerRow;++i)
    {
      for(int j=0;j<lPosesPerRow;++j)
      {
        if(i*lPosesPerRow+j<lPoselist.size())
        {
          cv::Rect lR(cv::Point(i*lPoseSz.width,j*lPoseSz.height),lPoseSz);
          cv::Mat lPoseIm(cv::imread(lPoselist[i*lPosesPerRow+j].getImagePath().string()));
          cv::Mat lResultPose=mResult(lR);
          cv::resize(lPoseIm,lResultPose,lR.size());
        }
      }
    }
    
    cv::Rect lImRect(cv::Point(640,0),cv::Size(320,240));
    cv::Mat lResultIm=mResult(lImRect);
    cv::resize(pIm,lResultIm,lImRect.size());
    cv::Rect lImCropRect(cv::Point(640,240),cv::Size(320,240));
    
    cv::Mat lResultImCrop=mResult(lImCropRect);
    //cv::Mat lImCrop8UC3(pImCrop.size(),CV_8UC3);
    //pImCrop.convertTo(lImCrop8UC3,CV_8UC3);
    //cv::resize(lImCrop8UC3,lResultImCrop,lImCropRect.size()); // could need converting 32FC3 to 8UC3
    lResultImCrop = cv::Scalar(255,255,255);
    mFeat->draw(lResultImCrop);
  }
  
  
protected:
  
  unsigned int mWorsen;
  CCapture mCapture;
  Feature<tType>* mFeat;                   /**< Feature instance*/
  CHandTracker mTracker;      /**< Tracker instance*/
  ProcessFeat<PL,tType>* mProcFeat;/**< ProcessHog instance (includes poselist and nn)*/
  //	cv::Mat mInImage;                 /**< Input Image*/
  //	std::list<fsystem::path> mImPaths;              /**< Image Paths (offline)*/
  std::list<tPoseV> mGTPoses;  /**< Image Paths (offline)*/
  bool mOfflineImages,mGT;    /**< Flags for offline, image ready, step-by-step and ground truth available*/
  const bool mGUI;
  int mDelay;
  cv::Mat mResult;
  const std::string mResultName;
};
#endif