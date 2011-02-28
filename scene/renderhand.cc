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

#include "cDB.h"
#include "cDBelement.h"

static const unsigned gNFrames(5);
static const buola::C3DVector lHandZero(0.31,-0.57,0.02);
static const buola::C3DVector lObjZero(0,0,0);
static const double gCamDistance(0.3);
static const unsigned gNumViews(1000);

using namespace buola;

int main(int pNArg,char **pArgs)
{
//   const std::vector<std::string> &lPosePathV=CProgramOption::GetArgs();
  
  srand(time(NULL));
  buola_init(pNArg,pArgs);
  
  std::string lObjectPath("/home/javier/tmp/objAfterFeedback/objs/adductedThumb_onlyObject.obj");
  std::string lPosePath;
  
  double *lCam2PalmRArray=new double[9];
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
  
  try
  {
    std::ofstream lHOGFS;
    CDB *lDB=(gDBPathOption.IsSet())?new CDB(gDBPathOption.GetValue()):NULL;
    CDB lDBtaxonomy("taxonomy.db");
//     CDB lDBtaxonomy("hands.db");
    std::vector<float> lFeature;
    if(gHOGPathOption.IsSet())
      lHOGFS.open(gHOGPathOption.GetValue().c_str());
    
    CDBelement lDBelemRest=lDBtaxonomy.query(0);
    tFullPoseV lRestPose;
    lDBelemRest.getFullPose(lRestPose);
    Hog<float> lHog;
    unsigned lFeatSize=lHog.getFeatSize();
    float *lFeatureA=new float[lFeatSize*31*gNFrames*gNumViews];
    
//     for(int p=0;p<lPosePathV.size();++p)
    for(int p=0;p<31;++p)
    {
      CDBelement lDBelem=lDBtaxonomy.query(p+1); // skipping rest pose
      //       std::cout << lDBelem << std::endl;
      //       loadPose(lPosePathV[p],lSkeleton,lHandTransf,lObjTransf,lObjectPath,lCam2PalmRArray);
      //       CDBelement lDBelem(lPosePathV[p],p);
      //       lDBtaxonomy.insertElement(lDBelem);
      
      scene::PPerspectiveCamera lCamera=new scene::CPerspectiveCamera;
      lCamera->SetClipping(0.01,200);
//       CSaveButton lButton(lSkeleton,lHandTransf,lObjTransf,lCamera,lObjectPath,NULL);
      
      scene::PScene lScene=new scene::CScene;
      
      scene::CImageRenderer lRenderer;
      lRenderer.SetClearColor(buola::CColor(0,0,0));
      buola::img::CImage_rgb8 lImage({400,400});
      
      tFullPoseV lFullPose;
      lDBelem.getFullPose(lFullPose);
      std::stringstream lPoseFolder;
      lPoseFolder << std::setfill('0');
      lPoseFolder << "out/" << std::setw(3) << p;
      fsystem::path lFolderPath(fsystem::initial_path());
      lFolderPath/=lPoseFolder.str();
      fsystem::create_directory(lFolderPath);
      for(int f=0;f<gNFrames;++f)
      {
        tFullPoseV lFullPoseInterp=((gNFrames-(f+1))*lRestPose+(f+1)*lFullPose)/gNFrames;
        lDBelem.setFullPose(lFullPoseInterp);
        loadPose(lDBelem,lSkeleton,lHandTransf,lObjTransf,lObjectPath,lCam2PalmRArray);
        
        lScene->GetWorld()->AddChild(lHandTransf);
        lHandTransf->AddChild(lSkeleton.GetSkeleton()->GetRoot()->GetTransform());
        lScene->AddObject(lSkeleton.GetSkeleton());
        
        scene::PGeode lGeode=buola::scene::CGeode::Import(lObjectPath.c_str(),0.1); // why if I put this outside the loop it does weird things?
        lGeode->AttachTo(lObjTransf);
        lScene->GetWorld()->AddChild(lObjTransf);
        
        lRenderer.SetScene(lScene);
      
        for(int i=0;i<gNumViews;++i)
        {
          //std::cout << i << std::endl;
          //         lCamera->LookAt(C3DVector(0,0,0),C3DRotation(lYPR[i*3],lYPR[i*3+1],lYPR[i*3+2]),0.25);
          
          C3DVector lAt(0,0,0);
          C3DVector lFrom(lXYZ[i].first);
          C3DVector lUp(lXYZ[i].second);
          lCamera->LookAt(lAt,lFrom,lUp);
          
          lRenderer.SetCamera(lCamera);
          lRenderer.GetImage(mview(lImage));
          std::stringstream lPoseName;
          lPoseName << std::setfill('0');
          lPoseName << std::setw(3) << f << "_" << std::setw(3) << i << ".pgm";
          fsystem::path lPosePath(lFolderPath);
          lPosePath/=lPoseName.str();
          //         lPath << "out/" << std::setw(3) << p << "_" << std::setw(3) << int(rad2deg(lYPR[i*3])) << "_" << std::setw(3) << int(rad2deg(lYPR[i*3+1])) << "_" << std::setw(3) << int(rad2deg(lYPR[i*3+2])) << ".pgm";
          buola::save_pgm(view(lImage),lPosePath.string());
//           lPath.seekp(static_cast<long>(lPath.tellp())-4);
//           lPath << ".txt";
//           lButton.saveURL(lPath.str());
          
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
            lFeature=lHog.compute(lTstMaskCrop,99999999);
            std::copy(lFeature.begin(),lFeature.end(),&(lFeatureA[(p*(gNumViews*gNFrames)+f*gNumViews+i)*lFeatSize]));
//             std::copy(lFeature.begin(),lFeature.end(),std::ostream_iterator<float>(lHOGFS," "));
//             lHOGFS << std::endl;
          }
          if(gDBPathOption.IsSet())
          {
//             for(int e=0;e<9;++e)
//               std::cout << lCam2PalmRArray[e] << " ";
//             std::cout << std::endl;
            lDBelem.setOri(getCam2PalmR(lSkeleton,lCamera));
            
            lDBelem.setPartsLocation(partsLocation2String(lSkeleton,lCamera));
            lDBelem.setCamAtFromUp(lAt,lFrom,lUp);
            lDBelem.setImagePath(lPosePath.string());
            lDBelem.setIndex(p*(gNumViews*gNFrames)+f*gNumViews+i);
            if(gHOGPathOption.IsSet())
              lDBelem.setFeature(lFeature);
            lDB->insertElement(lDBelem);
          }
        }
      lObjTransf->RemoveObject(lGeode);
      }
    }
    if(gHOGPathOption.IsSet())
    {
      lHOGFS.write(reinterpret_cast<char*>(lFeatureA),lFeatSize*31*gNFrames*gNumViews*sizeof(float));
      lHOGFS.close();
    }
    if(gDBPathOption.IsSet())
    {
      lDB->finalizeStatement();
      //      delete lDB;
    }
    delete []lFeatureA;
  }
  catch(std::exception &pE)
  {
    msg_info() << "caught exception " << pE.what() << "\n";
  }
  
  return buola_finish();
}
