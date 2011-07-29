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
#include "loadPose.h" // conflicts with X

#include <buola/scene.h>
#include <buola/scene/csceneview.h>
#include <buola/scene/ccamera.h>
#include <buola/app/capp.h>
#include <buola/scene/cperspectivecamera.h>
#include <buola/scene/cscene.h>
#include <buola/scene/cmesh.h>
#include <buola/scene/cskeleton.h>
#include <buola/scene/cbone.h>
#include <buola/scene/cgeode.h>
#include <buola/image/format.h>
#include <buola/scene/crttransform.h>

#include <buola/widgets/cbutton.h>
#include <buola/app/ccmdline.h>

#include <buola/geometry/clookatmatrix.h>
#include <buola/geometry/c3dvector.h>
#include <buola/geometry/cperspectiveprojectionmatrix.h>

#include "handclass_config.h"

#include <sstream>
#include <iterator>
#include <algorithm>
#include <vector>
#include <fstream>

#include "csavebutton.h"
#include "cjointslider.h"
#include "cposslider.h"
#include "ccamslider.h"
#include "CHandSkeleton.h"

static const buola::C3DVector lHandZero(0.31,-0.57,0.02);
static const buola::C3DVector lObjZero(0,0,0);
static const unsigned gSliderHeight(12);
static const unsigned gSliderWidth(150);
static const unsigned gSliderOffsetX(150);

using namespace buola;

void addCamSlider(int y,scene::CSceneView *pScene,std::vector<CCamSlider*> &pRots,
        C3DRotation &pRot,double buola::C3DRotation::*pRotVal,
        const std::string &pCaption,scene::PPerspectiveCamera &pCam)
{
    gui::CLabelBox *lLabel=new gui::CLabelBox;
    // dynamic cast unnecesary
	lLabel->Create(dynamic_cast<gui::CWindow*>(pScene),CPoint(0,y),CSize(gSliderWidth,gSliderHeight));
	lLabel->SetStyle(LBL_TOP);
    // L"..." creates them wide directly 
	lLabel->SetCaption(decode(pCaption));
	lLabel->Show();

    gui::CSliderBox *lBox=new gui::CSliderBox;
	lBox->Create(dynamic_cast<gui::CWindow*>(pScene),CPoint(gSliderOffsetX,y),CSize(gSliderWidth,gSliderHeight));
	lBox->SetStyle(gui::CSliderBox::STYLE_HORIZONTAL);
	lBox->SetRange(-100,100);
	lBox->SetValue(0);
	lBox->Show();

    CCamSlider *lSlider=new CCamSlider(lBox,pRot,pRotVal,pCam,pScene);
	pRots.push_back(lSlider);

	lBox->eChanged.Connect(&CCamSlider::OnSlider,lSlider);
}

void addXYZSlider(int y,scene::CSceneView *pScene,std::vector<CPosSlider*> &pPoses,C3DVector &pV,
        double buola::C3DVector::*pPosVal,const std::string &pCaption,boost::intrusive_ptr<buola::scene::CRTTransform> pTransf)
{
    gui::CLabelBox *lLabel=new gui::CLabelBox;
	lLabel->Create(dynamic_cast<gui::CWindow*>(pScene),CPoint(0,y),CSize(gSliderWidth,gSliderHeight));
	lLabel->SetStyle(LBL_TOP);
	lLabel->SetCaption(decode(pCaption));
	lLabel->Show();

    gui::CSliderBox *lBox=new gui::CSliderBox;
	lBox->Create(dynamic_cast<gui::CWindow*>(pScene),CPoint(gSliderOffsetX,y),CSize(gSliderWidth,gSliderHeight));
	lBox->SetStyle(gui::CSliderBox::STYLE_HORIZONTAL);
	lBox->SetRange(-gPosSliderRange,gPosSliderRange);
	lBox->SetValue(0);
	lBox->Show();

    CPosSlider *lSlider=new CPosSlider(lBox,pV,pPosVal,pTransf,pScene);
	pPoses.push_back(lSlider);

	lBox->eChanged.Connect(&CPosSlider::OnSlider,lSlider);
}

void addSlider(int y,const scene::EJointType &pType,scene::PBone &pBone,
        scene::CSceneView *pScene, std::vector<CJointSlider*> &pSliders)
{
  gui::CLabelBox *lLabel=new gui::CLabelBox;
  lLabel->Create(dynamic_cast<gui::CWindow*>(pScene),CPoint(0,y),CSize(gSliderWidth,gSliderHeight));
  lLabel->SetStyle(LBL_TOP);
  lLabel->SetCaption(decode(pBone->GetName()+gJointTypeNames[pType]));
  lLabel->Show();
  
  gui::CSliderBox *lBox=new gui::CSliderBox;
  lBox->Create(dynamic_cast<gui::CWindow*>(pScene),CPoint(gSliderOffsetX,y),CSize(gSliderWidth,gSliderHeight));
  lBox->SetStyle(gui::CSliderBox::STYLE_HORIZONTAL);
  lBox->SetRange(-180,180);
  lBox->SetValue(pBone->GetJointValue(pType)*180/M_PI);
  lBox->Show();
  
  CJointSlider *lSlider=new CJointSlider(lLabel,lBox,pType,pBone,pScene);
  pSliders.push_back(lSlider);
  lSlider->OnSlider();
  
  lBox->eChanged.Connect(&CJointSlider::OnSlider,lSlider);
}

int main(int pNArg,char **pArgs)
{
    buola_init(pNArg,pArgs);
    
    fsystem::path lObjectPathFS(SCENEPATH);
    lObjectPathFS/="objects/adductedThumb_onlyObject.obj";
    std::string lObjectPath = lObjectPathFS.string();
    
    fsystem::path lObjPathFS(SCENEPATH);
    lObjPathFS/="rHandP3.obj";
    fsystem::path lTexturePathFS(SCENEPATH);
    lTexturePathFS/="hand_texture.ppm";
    
    double *lCam2PalmRArray=new double[16];
    CHandSkeleton lSkeleton(lObjPathFS.string().c_str(),lTexturePathFS.string().c_str());

    scene::PRTTransform lHandTransf=new scene::CRTTransform;
    scene::PRTTransform lObjTransf=new scene::CRTTransform;
    std::vector<scene::PRTTransform> lTransformations;
    lTransformations.push_back(lHandTransf);
    lTransformations.push_back(lObjTransf);
    lHandTransf->SetTranslation(lHandZero);
    lObjTransf->SetTranslation(lObjZero);
    
    // if they specify an object .obj
    if(cmd_line().IsSet(gObjectPathOption))
      lObjectPath=cmd_line().GetValue(gObjectPathOption);
    // if they specify a pose file
    if(cmd_line().IsSet(gPosePathOption))
      loadPose(cmd_line().GetValue(gPosePathOption),lSkeleton,lHandTransf,lObjTransf,lObjectPath,lCam2PalmRArray);
    else
      setPose(lSkeleton);
    
    scene::PGeode lGeode=buola::scene::CGeode::Import(lObjectPath.c_str(),0.1);
    std::cout << pArgs[2] << std::endl;
    std::string lObjectObjPath(pArgs[2]);
    
    try
    {
        scene::CSceneView lView;
        lView.Create(NULL,CSize(1000,1000));
        CApp lApp;

        scene::PPerspectiveCamera lCamera=new scene::CPerspectiveCamera;
        lCamera->LookAt(C3DVector(0,0,0),C3DRotation(0,0,0),gCamDistance);
        lCamera->SetClipping(0.01,200);

        scene::PScene lScene=new scene::CScene;
        std::vector<CJointSlider*> lSliders;


        std::string lSliderNames[2]={"hand","object"};
        std::string lXYZ[3]={"X","Y","Z"};
        
        std::vector<CPosSlider*> lPosesV;
        std::vector<buola::C3DVector> lPoses3DV(2);
        
        std::vector<CCamSlider*> lRotsV;
        buola::C3DRotation lRot;

        lScene->GetWorld()->AddChild(lHandTransf);
        lHandTransf->AddChild(lSkeleton.GetSkeleton()->GetRoot()->GetTransform());
        lScene->AddObject(lSkeleton.GetSkeleton());

        lGeode->AttachTo(lObjTransf);
        lScene->GetWorld()->AddChild(lObjTransf);

        addCamSlider((0*3+0)*gSliderHeight,&lView,lRotsV,lRot,&buola::C3DRotation::a,"cam yaw",lCamera);
        addCamSlider((0*3+1)*gSliderHeight,&lView,lRotsV,lRot,&buola::C3DRotation::b,"cam pitch",lCamera);
        addCamSlider((0*3+2)*gSliderHeight,&lView,lRotsV,lRot,&buola::C3DRotation::c,"cam roll",lCamera);
        
        for(int i=0;i<2;++i)
        {
                addXYZSlider(((i+1)*3+0)*gSliderHeight,&lView,lPosesV,lPoses3DV[i],&buola::C3DVector::x,lSliderNames[i]+lXYZ[0],lTransformations[i]);
                addXYZSlider(((i+1)*3+1)*gSliderHeight,&lView,lPosesV,lPoses3DV[i],&buola::C3DVector::y,lSliderNames[i]+lXYZ[1],lTransformations[i]);
                addXYZSlider(((i+1)*3+2)*gSliderHeight,&lView,lPosesV,lPoses3DV[i],&buola::C3DVector::z,lSliderNames[i]+lXYZ[2],lTransformations[i]);
        }

        for(int i=0;i<17;i++)
          for(int j=0;j<3;j++)
            addSlider(((i+3)*3+j)*gSliderHeight,gJointTypes[j],lSkeleton[i],&lView,lSliders);
        
        lView.SetCamera(lCamera);
        lView.SetScene(lScene);

        CSaveButton lButton(lSkeleton,lHandTransf,lObjTransf,lCamera,lObjectObjPath,&lView);

        lView.Show();
        lApp.Run();
    }
    catch(std::exception &pE)
    {
        msg_info() << "caught exception " << pE.what() << "\n";
    }

    return buola_finish();
}
