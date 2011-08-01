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

static const buola::C3DVector lHandZero(0.31,-0.57,0.02);
static const buola::C3DVector lHandRest(0.31,-0.5,0.02);
static const buola::C3DVector lObjZero(0,0,0);
static const double gCamDistance(0.3);
static const unsigned gNumGrasps(31);

using namespace buola;
static buola::CCmdLineOption<std::string> gHOGPathOption("hogpath",'h',L"Path to hog file to be saved","scene/hog.bin");
static buola::CCmdLineOption<std::string> gDBPathOption("db",'d',L"Path to db file to be saved","scene/hands.db");
static buola::CCmdLineOption<unsigned> gNumViewsOption("nv",'v',L"Number of views to be render of each grasp step",10);
static buola::CCmdLineOption<unsigned> gNStepsOption("ns",'s',L"Number of steps rendered for each grasp",5);

int main(int pNArg,char **pArgs)
{
  // engine for generating real numbers between -1 and 1
  std::mt19937 lEngine(time(0));
  std::uniform_real_distribution<> lDist(-1,1);

  buola_init(pNArg,pArgs);
  static const unsigned gNumViews(cmd_line().GetValue(gNumViewsOption));
  static const unsigned gNSteps(cmd_line().GetValue(gNStepsOption));
  
  fsystem::path lObjectPathFS(SCENEPATH);
  lObjectPathFS/="objects/adductedThumb_onlyObject.obj";
  std::string lObjectPath = lObjectPathFS.string();

  fsystem::path lHandObjPathFS(SCENEPATH);
  lHandObjPathFS/="rHandP3.obj";
  fsystem::path lTexturePathFS(SCENEPATH);
  lTexturePathFS/="hand_texture.ppm";
  
  // rotation matrix of the camera wrt the palm
  // not used since we obtain the hand and the camera orientations
  double *lCam2PalmRArray=new double[9];

  // Hand skeleton with the hand mesh and texture
  CHandSkeleton lSkeleton(lHandObjPathFS.string().c_str(),lTexturePathFS.string().c_str());
  
  scene::PRTTransform lHandTransf=new scene::CRTTransform;
  scene::PRTTransform lObjTransf=new scene::CRTTransform;
  lHandTransf->SetTranslation(lHandZero);
  lObjTransf->SetTranslation(lObjZero);
  
  // generate a vector of random camera origins(first) on the unit sphere,
  // with random camera orientation (second)
  std::pair<C3DVector,C3DVector> *lXYZ=new std::pair<C3DVector,C3DVector>[gNumViews];
  for(int i=0;i<gNumViews;)
  {
    // generate a 3D point in a cube [-1,1]
    lXYZ[i].first.Set(lDist(lEngine),lDist(lEngine),lDist(lEngine));
    // if it inside the sphere centered in the origin with radious 1 and not in the origin
    if(lXYZ[i].first.Modulus2()<=1 && lXYZ[i].first.Modulus2()!=0)
    {
      // strictly speaking lRand should be non-zero and not parallel to lXYZ[i].first
      C3DVector lRand(lDist(lEngine),lDist(lEngine),lDist(lEngine));
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

    // previously generated database with the basic poses
    fsystem::path lBasicRenderDbFS(SCENEPATH);
    lBasicRenderDbFS/="taxonomy.db";
    CDB lDBtaxonomy(lBasicRenderDbFS);

    std::vector<float> lFeature;
    if(cmd_line().IsSet(gHOGPathOption))
      lHOGFS.open(cmd_line().GetValue(gHOGPathOption).c_str());
    
    // we consider the rest position as all joints being 0
    tFullPoseV lRestPose = tFullPoseV::Zero();
    Hog<float> lHog;
    unsigned lFeatSize=lHog.getFeatSize();
    float *lFeatureA=new float[lFeatSize*gNumGrasps*gNSteps*gNumViews];
    
    for(int p=0;p<gNumGrasps;++p)
    {
      // get the basic pose
      CDBelement lDBelem=lDBtaxonomy.query(p);
      
      scene::PPerspectiveCamera lCamera=new scene::CPerspectiveCamera;
      lCamera->SetClipping(0.01,200);
      
      scene::PScene lScene=new scene::CScene;
      
      scene::CImageRenderer lRenderer;
      lRenderer.SetClearColor(buola::CColor(0,0,0));
      buola::img::CImage_rgb8 lImage({400,400});
      
      tFullPoseV lFullPose;
      lDBelem.getFullPose(lFullPose);

      // create the output folder for the images
      std::stringstream lPoseFolder;
      lPoseFolder << std::setfill('0');
      lPoseFolder << "out/" << std::setw(3) << p;
      fsystem::path lFolderPath(fsystem::initial_path());
      lFolderPath/=lPoseFolder.str();
      fsystem::create_directory(lFolderPath);

      for(int f=0;f<gNSteps;++f)
      {
        // interpolate between the rest position (Zero) and the basic pose
        tFullPoseV lFullPoseInterp=((gNSteps-(f+1))*lRestPose+(f+1)*lFullPose)/gNSteps;
        lDBelem.setFullPose(lFullPoseInterp);


        // it's probably better to put as much information inside lDBelem
        // before passing it to loadPose (line 255)



        loadPose(lDBelem,lSkeleton,lHandTransf,lObjTransf,lObjectPath,lCam2PalmRArray);

        // interpolate rest translation and basic translation
        lHandTransf->SetTranslation(((gNSteps-(f+1))*lHandRest+(f+1)*lHandTransf->GetTranslation())/gNSteps);

        // why is this changing the result??
        //std::ostringstream lHandPosSS;
        //C3DVector lHT =lHandTransf->GetTranslation(); 
        //lHandPosSS << lHT.x << " " << lHT.y << " " << lHT.z;
        //lDBelem.setHandPos(lHandPosSS.str());
        
        lScene->GetWorld()->AddChild(lHandTransf);
        lHandTransf->AddChild(lSkeleton.GetSkeleton()->GetRoot()->GetTransform());
        lScene->AddObject(lSkeleton.GetSkeleton());
        
        scene::PGeode lGeode=buola::scene::CGeode::Import(lObjectPath.c_str(),0.1); // why if I put this outside the loop it does weird things?
        lGeode->AttachTo(lObjTransf);
        lScene->GetWorld()->AddChild(lObjTransf);
        
        lRenderer.SetScene(lScene);
      
        for(int i=0;i<gNumViews;++i)
        {
          
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
          save(lImage,lPosePath.string());
          
          // if required, compute and save the hog
          if(cmd_line().IsSet(gHOGPathOption))
          {
            cv::Mat lImageCV=cv::Mat(buola::img::ipl_wrap(lImage),false);
            cv::Mat lGrayIm(lImageCV.size(),CV_8UC1);
            cv::Mat lImageCV32F(lImageCV.size(),CV_32FC3);
            lImageCV.convertTo(lImageCV32F,CV_32FC3);
            cv::cvtColor(lImageCV,lGrayIm,CV_BGR2GRAY);
            cv::threshold(lGrayIm,lGrayIm,1,255,cv::THRESH_BINARY);
            
            // compute contours to localize the hand
            std::vector<cv::Point> lAllContours;
            {
              std::vector<std::vector<cv::Point> > lContours;
              cv::Mat lTmp = lGrayIm.clone();
              cv::findContours(lTmp, lContours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
              for(int i=0;i<lContours.size();++i)
                lAllContours.insert(lAllContours.end(),lContours[i].begin(),lContours[i].end());
            }

            // exit if there's no hand pixel; it would fail with a weird error when computing the hog
            if(lAllContours.empty())
            {
              std::cerr << "There is no hand in the figure" << std::endl << 
                "Run it again (to randomize the view point) or change the rendering parameters" << std::endl;
              exit(1);
            }
            
            cv::Rect lBBox=cv::boundingRect(cv::Mat(lAllContours));
            
            std::pair<cv::Mat,cv::Mat> lTstMaskCrop=std::make_pair(lImageCV32F(lBBox),lGrayIm(lBBox));
            lFeature=lHog.compute(lTstMaskCrop,99999999);
            std::copy(lFeature.begin(),lFeature.end(),&(lFeatureA[(p*(gNumViews*gNSteps)+f*gNumViews+i)*lFeatSize]));
          }

          // if required, save the pose parameters in a sqlite3 database
          if(cmd_line().IsSet(gDBPathOption))
          {
            lDBelem.setOri(getCam2PalmR(lSkeleton,lCamera));
            
            lDBelem.setPartsLocation(partsLocation2String(lSkeleton,lCamera));
            lDBelem.setCamAtFromUp(lAt.x,lAt.y,lAt.z,lFrom.x,lFrom.y,lFrom.z,lUp.x,lUp.y,lUp.z);
            lDBelem.setImagePath(lPosePath.string());
            lDBelem.setIndex(p*(gNumViews*gNSteps)+f*gNumViews+i);
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
      lHOGFS.write(reinterpret_cast<char*>(lFeatureA),lFeatSize*gNumGrasps*gNSteps*gNumViews*sizeof(float));
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
