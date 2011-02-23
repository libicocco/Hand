// I have to include hog first
// because X defines collide with them
#include "hog.h" // from handclass

#include <stdlib.h>
#include <cstdlib>

#include <time.h>
#include <sstream>
#include <iomanip>
#include <iterator>
#include <algorithm>
#include <vector>
#include <fstream>

#include <buola/scene.h>
#include <buola/scene/csceneview.h>
#include <buola/scene/ccamera.h>
#include <buola/scene/cperspectivecamera.h>
#include <buola/scene/cscene.h>
#include <buola/scene/cmesh.h>
#include <buola/scene/cskeleton.h>
#include <buola/scene/cbone.h>
#include <buola/scene/cgeode.h>
#include <buola/image/format.h>
#include <buola/image/io_pgm.h>
#include <buola/scene/crttransform.h>
#include <buola/scene/cimagerenderer.h>
#include <buola/cv/opencv.h>

#include <buola/app/cprogramoption.h>

#include <buola/geometry/clookatmatrix.h>
#include <buola/geometry/c3dvector.h>
#include <buola/geometry/cperspectiveprojectionmatrix.h>

#include "csavebutton.h"
#include "CHandSkeleton.h"
#include "loadPose.h"

static const buola::C3DVector lHandZero(0.31,-0.57,0.02);
static const buola::C3DVector lObjZero(0,0,0);
static const double gCamDistance(0.3);
static const unsigned gNumViews(1000);

using namespace buola;

int main(int pNArg,char **pArgs)
{
  const std::vector<std::string> &lPosePathV=CProgramOption::GetArgs();
  
  srand(time(NULL));
  buola_init(pNArg,pArgs);
  
  std::string lObjectPath("/home/javier/tmp/objAfterFeedback/objs/adductedThumb_onlyObject.obj");
  std::string lPosePath;
  
  double *lCam2PalmRArray=new double[16];
  CHandSkeleton lSkeleton("/home/javier/scene/rHandP3.obj","/home/javier/scene/hand_texture.ppm");
  
  scene::PRTTransform lHandTransf=new scene::CRTTransform;
  scene::PRTTransform lObjTransf=new scene::CRTTransform;
  lHandTransf->SetTranslation(lHandZero);
  lObjTransf->SetTranslation(lObjZero);
  
  std::pair<C3DVector,C3DVector> *lXYZ=new std::pair<C3DVector,C3DVector>[gNumViews];
  for(int i=0;i<gNumViews;)
  {
    lXYZ[i].first.Set((2*static_cast<float>(rand())/static_cast<float>(RAND_MAX))-1.0,
                      (2*static_cast<float>(rand())/static_cast<float>(RAND_MAX))-1.0,
                      (2*static_cast<float>(rand())/static_cast<float>(RAND_MAX))-1.0);
    if(lXYZ[i].first.Modulus2()<=1 && lXYZ[i].first.Modulus2()!=0)
    {
      // strictly speaking lRand should be non-zero and not parallel to lXYZ[i].first
      C3DVector lRand((2*static_cast<float>(rand())/static_cast<float>(RAND_MAX))-1.0,
                      (2*static_cast<float>(rand())/static_cast<float>(RAND_MAX))-1.0,
                      (2*static_cast<float>(rand())/static_cast<float>(RAND_MAX))-1.0);
//       lXYZ[i].first=normalize(lXYZ[i].first);
      lXYZ[i].first=lXYZ[i].first*(gCamDistance/lXYZ[i].first.Modulus());
      lXYZ[i].second=cross_product(lXYZ[i].first,lRand);
      lXYZ[i].second=normalize(lXYZ[i].second);
      ++i;
    }
  }
  
//   float lYPR[gNumViews*3]={0,0,0,0,0,M_PI/2,0,M_PI/2,0,0,M_PI/2,M_PI/2,M_PI/2,0,0,M_PI/2,0,M_PI/2,M_PI/2,M_PI/2,0,M_PI/2,M_PI/2,M_PI/2};
  
//   float *lYPR=new float[gNumViews*3];
//   static const float gMaxRadious=static_cast<float>(RAND_MAX/2);
//   for(int i=0;i<gNumViews;)
//   {
//     float x=static_cast<float>(rand())-gMaxRadious;
//     float y=static_cast<float>(rand())-gMaxRadious;
//     float z=static_cast<float>(rand())-gMaxRadious;
//     if(x*x+y*y+z*z <= gMaxRadious*gMaxRadious && (x!=0 && y!=0 && z!=0))
//     {
//       lYPR[i*3]=atan2(y,x); // yaw
//       lYPR[i*3+1]=atan2(z,sqrt(x*x+y*y)); //pitch
//       lYPR[i*3+2]=(static_cast<float>(rand())-gMaxRadious)*(2*M_PI/static_cast<float>(RAND_MAX)); // roll
//       ++i;
//     }
//   }
  
  try
  {
    std::ofstream lHOGFS;
    if(gHOGPathOption.IsSet())
      lHOGFS.open(gHOGPathOption.GetValue().c_str());
    
    for(int p=0;p<lPosePathV.size();++p)
    {
      loadPose(lPosePathV[p],lSkeleton,lHandTransf,lObjTransf,lObjectPath,lCam2PalmRArray);
      scene::PGeode lGeode=buola::scene::CGeode::Import(lObjectPath.c_str(),0.1);
      
      scene::PPerspectiveCamera lCamera=new scene::CPerspectiveCamera;
      lCamera->SetClipping(0.01,200);
      CSaveButton lButton(lSkeleton,lHandTransf,lObjTransf,lCamera,lObjectPath,NULL);
      
      scene::PScene lScene=new scene::CScene;
      
      lScene->GetWorld()->AddChild(lHandTransf);
      lHandTransf->AddChild(lSkeleton.GetSkeleton()->GetRoot()->GetTransform());
      lScene->AddObject(lSkeleton.GetSkeleton());
      
      lGeode->AttachTo(lObjTransf);
      lScene->GetWorld()->AddChild(lObjTransf);
      
      scene::CImageRenderer lRenderer;
      lRenderer.SetScene(lScene);
      lRenderer.SetClearColor(buola::CColor(0,0,0));
      buola::img::CImage_rgb8 lImage({400,400});
      
      Hog<float> lHog;
      for(int i=0;i<gNumViews;++i)
      {
        //std::cout << i << std::endl;
//         lCamera->LookAt(C3DVector(0,0,0),C3DRotation(lYPR[i*3],lYPR[i*3+1],lYPR[i*3+2]),0.25);
        lCamera->LookAt(C3DVector(0,0,0),lXYZ[i].first,lXYZ[i].second);
        lRenderer.SetCamera(lCamera);
        lRenderer.GetImage(mview(lImage));
        std::stringstream lPath;
        lPath << std::setfill('0');
        lPath << "out/test" << std::setw(3) << p << "_" << std::setw(3) << i << ".pgm";
//         lPath << "out/" << std::setw(3) << p << "_" << std::setw(3) << int(rad2deg(lYPR[i*3])) << "_" << std::setw(3) << int(rad2deg(lYPR[i*3+1])) << "_" << std::setw(3) << int(rad2deg(lYPR[i*3+2])) << ".pgm";
        buola::save_pgm(view(lImage),lPath.str());
        lPath.seekp(static_cast<long>(lPath.tellp())-4);
        lPath << ".txt";
        lButton.saveURL(lPath.str());
        
        if(gHOGPathOption.IsSet())
        {
          cv::Mat lImageCV=cv::Mat(buola::cvi::ipl_wrap(view(lImage)),false);
          cv::Mat lGrayIm(lImageCV.size(),CV_8UC1);
          cv::Mat lImageCV32F(lImageCV.size(),CV_32FC3);
          lImageCV.convertTo(lImageCV32F,CV_32FC3);
          cv::cvtColor(lImageCV,lGrayIm,CV_BGR2GRAY);
          cv::threshold(lGrayIm,lGrayIm,1,255,cv::THRESH_BINARY);
          
          std::vector<cv::Point> lAllContours;
          {
            std::vector<std::vector<cv::Point> > lContours;
            cv::Mat lTmp = lGrayIm.clone();
            cv::findContours(lTmp, lContours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
            for(int i=0;i<lContours.size();++i)
              lAllContours.insert(lAllContours.end(),lContours[i].begin(),lContours[i].end());
          }
          
          cv::Rect lBBox=cv::boundingRect(cv::Mat(lAllContours));
          
          std::pair<cv::Mat,cv::Mat> lTstMaskCrop=std::make_pair(lImageCV32F(lBBox),lGrayIm(lBBox));
          std::vector<float> lTstFloat=lHog.compute(lTstMaskCrop,99999999);
          std::copy(lTstFloat.begin(),lTstFloat.end(),std::ostream_iterator<float>(lHOGFS," "));
          lHOGFS << std::endl;
          
//           cv::Mat lTstDraw(cv::Size(400,400),CV_8UC1,cv::Scalar(255));
//           lHog.draw(lTstDraw);
//           cv::imwrite("hog.png",lTstDraw);
//           cv::imwrite("mask.png",lGrayIm(lBBox));
        }
      }
      lObjTransf->RemoveObject(lGeode);
    }
    if(gHOGPathOption.IsSet())
      lHOGFS.close();
    //delete []lYPR;
  }
  catch(std::exception &pE)
  {
    msg_info() << "caught exception " << pE.what() << "\n";
  }
  
  return buola_finish();
}
