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
#ifndef __HANDRENDERER_H
#define __HANDRENDERER_H

#include <buola/image/algorithm/detail/opencv.h>
#include "hog.h" // from handclass

#include <stdlib.h>

#include <sstream>
#include <vector>

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

#include <buola/geometry/clookatmatrix.h>
#include <buola/geometry/c3dvector.h>
#include <buola/geometry/cperspectiveprojectionmatrix.h>

#include "CHandSkeleton.h"

#include "cDBelement.h"

using namespace buola;
void setTransf(const std::string &pPos,const std::string &pOri,scene::PRTTransform &pTransf)
{
  std::stringstream lVectorSS(pOri);
  buola::CQuaternion lQ;
  lVectorSS >> lQ;
  lVectorSS.clear();
  lVectorSS.str(pPos);
  buola::C3DVector lT;
  lVectorSS >> lT;
  pTransf->SetRotation(lQ);
  pTransf->SetTranslation(lT);
}

class HandRenderer
{
  public:
    HandRenderer(const char* pHandObjPath,const char* pHandTexturePath):
      mCamera(new scene::CPerspectiveCamera),
      mScene(new scene::CScene),
      mRenderer(),
      mImage({400,400}),
      mHandTransf(new scene::CRTTransform),
      mObjTransf(new scene::CRTTransform),
      mSkeleton(pHandObjPath,pHandTexturePath),
      mHog(),
      mFeature(),
      mObjPath("")
    {
      mCamera->SetClipping(0.01,200);
      mRenderer.SetClearColor(buola::CColor(255,0,0));
    }


    void render(const CDBelement &pDBelem)
    {
      // set joints in the skeleton
      tFullPoseV lFullPose;
      pDBelem.getFullPose(lFullPose);
      for(int i=0;i<17;++i)
        for(int a=0;a<3;++a)
          mSkeleton[i]->SetJointValue(gJointTypes[a],lFullPose[i*3+a]);

      // place hand in scene according to mHandTransf
      setTransf(pDBelem.getHandPos(),pDBelem.getHandOri(),mHandTransf);
      mScene->GetWorld()->AddChild(mHandTransf);
      mHandTransf->AddChild(mSkeleton.GetSkeleton()->GetRoot()->GetTransform());
      mScene->AddObject(mSkeleton.GetSkeleton());

      mObjPath=pDBelem.getObjPath();
      scene::PGeode lGeode;
      if(!mObjPath.empty())
      {
        // place object in scene according to mObjTransf
        setTransf(pDBelem.getObjPos(),pDBelem.getObjOri(),mObjTransf);
        lGeode=buola::scene::CGeode::Import(mObjPath.c_str(),0.1);
        lGeode->AttachTo(mObjTransf);
        mScene->GetWorld()->AddChild(mObjTransf);
      }

      mRenderer.SetScene(mScene);

      // set the camera
      C3DVector lAt,lFrom,lUp;
      std::istringstream lVectorsSS(pDBelem.getCamAt());
      lVectorsSS >> lAt.x >> lAt.y >> lAt.z;
      lVectorsSS.clear();
      lVectorsSS.str(pDBelem.getCamFrom());
      lVectorsSS >> lFrom.x >> lFrom.y >> lFrom.z;
      lVectorsSS.clear();
      lVectorsSS.str(pDBelem.getCamUp());
      lVectorsSS >> lUp.x >> lUp.y >> lUp.z;
      mCamera->LookAt(lAt,lFrom,lUp);
      mRenderer.SetCamera(mCamera);

      // render to image
      mRenderer.GetImage(mImage);
      save(mImage,pDBelem.getImagePath());

      computeHog();
      //pDBelem.setFeature(mFeature);

      if(!mObjPath.empty())
        mObjTransf->RemoveObject(lGeode);
    }

  private:
    void computeHog()
    {
      cv::Mat lImageCV=cv::Mat(buola::img::ipl_wrap(mImage),false);
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
      mFeature=mHog.compute(lTstMaskCrop,99999999);
    }

    scene::PPerspectiveCamera mCamera;
    scene::PScene mScene;
    scene::CImageRenderer mRenderer;
    buola::img::CImage_rgb8 mImage;
    scene::PRTTransform mHandTransf;
    scene::PRTTransform mObjTransf;
    CHandSkeleton mSkeleton;
    Hog<float> mHog;
    std::vector<float> mFeature;
    std::string mObjPath;
};

#endif // __HANDRENDERER_H
