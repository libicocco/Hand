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
// I have to include hog first
// because X defines collide with them
#include <buola/image/algorithm/detail/opencv.h>
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
#include <buola/image/io.h>
#include <buola/scene/crttransform.h>
#include <buola/scene/cimagerenderer.h>

#include <buola/app/ccmdline.h>

#include <buola/geometry/clookatmatrix.h>
#include <buola/geometry/c3dvector.h>
#include <buola/geometry/cperspectiveprojectionmatrix.h>

#include "csavebutton.h"
#include "CHandSkeleton.h"
#include "loadPose.h"

#include "cDB.h"
#include "cDBelement.h"

#include "handclass_config.h"

static const unsigned gNFrames(5);
static const buola::C3DVector lHandZero(0.31,-0.57,0.02);
static const buola::C3DVector lObjZero(0,0,0);
static const double gCamDistance(0.3);
static const unsigned gNumViews(10);

using namespace buola;

int main(int pNArg,char **pArgs)
{
//   const std::vector<std::string> &lPosePathV=cmd_line()GetArgs();
  
  srand(time(NULL));
  buola_init(pNArg,pArgs);
  
  fsystem::path lObjectPathFS(SCENEPATH);
  lObjectPathFS/="objects/adductedThumb_onlyObject.obj";
  std::string lObjectPath = lObjectPathFS.string();

  fsystem::path lObjPathFS(SCENEPATH);
  lObjPathFS/="rHandP3.obj";
  fsystem::path lTexturePathFS(SCENEPATH);
  lTexturePathFS/="hand_texture.ppm";
  
  double *lCam2PalmRArray=new double[9];
  CHandSkeleton lSkeleton(lObjPathFS.string().c_str(),lTexturePathFS.string().c_str());
  
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
    CDB *lDB=(cmd_line().IsSet(gDBPathOption))?new CDB(cmd_line().GetValue(gDBPathOption)):NULL;
    fsystem::path lBasicRenderDbFS(SCENEPATH);
    lBasicRenderDbFS/="taxonomy.db";
    CDB lDBtaxonomy(lBasicRenderDbFS);
    std::vector<float> lFeature;
    if(cmd_line().IsSet(gHOGPathOption))
      lHOGFS.open(cmd_line().GetValue(gHOGPathOption).c_str());
    
    tFullPoseV lRestPose = tFullPoseV::Zero();
    Hog<float> lHog;
    unsigned lFeatSize=lHog.getFeatSize();
    float *lFeatureA=new float[lFeatSize*31*gNFrames*gNumViews];
    
//     for(int p=0;p<lPosePathV.size();++p)
    for(int p=0;p<31;++p)
    {
      CDBelement lDBelem=lDBtaxonomy.query(p);
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
          lRenderer.GetImage(lImage);
          std::stringstream lPoseName;
          lPoseName << std::setfill('0');
          lPoseName << std::setw(3) << f << "_" << std::setw(3) << i << ".pgm";
          fsystem::path lPosePath(lFolderPath);
          lPosePath/=lPoseName.str();
          //         lPath << "out/" << std::setw(3) << p << "_" << std::setw(3) << int(rad2deg(lYPR[i*3])) << "_" << std::setw(3) << int(rad2deg(lYPR[i*3+1])) << "_" << std::setw(3) << int(rad2deg(lYPR[i*3+2])) << ".pgm";
          save(lImage,lPosePath.string());
//           lPath.seekp(static_cast<long>(lPath.tellp())-4);
//           lPath << ".txt";
//           lButton.saveURL(lPath.str());
          
          if(cmd_line().IsSet(gHOGPathOption))
          {
            cv::Mat lImageCV=cv::Mat(buola::img::ipl_wrap(lImage),false);
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
          if(cmd_line().IsSet(gDBPathOption))
          {
//             for(int e=0;e<9;++e)
//               std::cout << lCam2PalmRArray[e] << " ";
//             std::cout << std::endl;
            lDBelem.setOri(getCam2PalmR(lSkeleton,lCamera));
            
            lDBelem.setPartsLocation(partsLocation2String(lSkeleton,lCamera));
            lDBelem.setCamAtFromUp(lAt,lFrom,lUp);
            lDBelem.setImagePath(lPosePath.string());
            lDBelem.setIndex(p*(gNumViews*gNFrames)+f*gNumViews+i);
            if(cmd_line().IsSet(gHOGPathOption))
              lDBelem.setFeature(lFeature);
            lDB->insertElement(lDBelem);
          }
        }
      lObjTransf->RemoveObject(lGeode);
      }
    }
    if(cmd_line().IsSet(gHOGPathOption))
    {
      lHOGFS.write(reinterpret_cast<char*>(lFeatureA),lFeatSize*31*gNFrames*gNumViews*sizeof(float));
      lHOGFS.close();
    }
    if(cmd_line().IsSet(gDBPathOption))
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
