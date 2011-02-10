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
#include <buola/image/io_pgm.h>
#include <buola/scene/crttransform.h>

#include <buola/widgets/cbutton.h>
#include <buola/app/cprogramoption.h>

#include <buola/geometry/clookatmatrix.h>
#include <buola/geometry/c3dvector.h>
#include <buola/geometry/cperspectiveprojectionmatrix.h>

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
static const unsigned gBufSize(1024);
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

	lBox->eChanged.connect(boost::bind(&CCamSlider::OnSlider,lSlider));
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

	lBox->eChanged.connect(boost::bind(&CPosSlider::OnSlider,lSlider));
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
  
  lBox->eChanged.connect(boost::bind(&CJointSlider::OnSlider,lSlider));
}

static buola::CTypedProgramOption<std::string> gObjectPathOption("objpath",'o',L"Path to object .obj file");
static buola::CTypedProgramOption<std::string> gPosePathOption("posepath",'p',L"Path to pose file");

int main(int pNArg,char **pArgs)
{
    buola_init(pNArg,pArgs);
    
    std::string lObjectPath("/home/javier/tmp/objAfterFeedback/objs/adductedThumb_onlyObject.obj");
    std::string lPosePath;
    
    double *lCam2PalmRArray=new double[16];
    CHandSkeleton lSkeleton("/home/javier/scene/rHandP3.obj","/home/javier/scene/hand_texture.ppm");

    scene::PRTTransform lHandTransf=new scene::CRTTransform;
    scene::PRTTransform lObjTransf=new scene::CRTTransform;
    std::vector<scene::PRTTransform> lTransformations;
    lTransformations.push_back(lHandTransf);
    lTransformations.push_back(lObjTransf);
    lHandTransf->SetTranslation(lHandZero);
    lObjTransf->SetTranslation(lObjZero);
    
    // if they specify an object .obj
    if(gObjectPathOption.IsSet())
    {
      lObjectPath=gObjectPathOption.GetValue();
      std::cout << "object path set to " << lObjectPath << std::endl;
    }
    // if they specify a pose file
    if(gPosePathOption.IsSet())
    {
      std::ifstream lFS(gPosePathOption.GetValue().c_str());
      char lLine[gBufSize];
      lFS.getline(lLine,gBufSize); // comment
      lFS.getline(lLine,gBufSize); // cam
      std::stringstream lOriSS(lLine);
      std::copy(std::istream_iterator<double>(lOriSS),
                std::istream_iterator<double>(),
                lCam2PalmRArray);
      
      lFS.getline(lLine,gBufSize); // comment
      lFS.getline(lLine,gBufSize); // joints
      std::stringstream lJointsSS(lLine);
      double *lJointValues=new double[51];
      std::copy(std::istream_iterator<double>(lJointsSS),
                std::istream_iterator<double>(),
                lJointValues);
      for(int i=0;i<17;++i)
        for(int a=0;a<3;++a)
          lSkeleton[i]->SetJointValue(gJointTypes[a],lJointValues[i*3+a]);
      std::copy(lJointValues,lJointValues+51,std::ostream_iterator<double>(std::cout," "));
      std::cout << std::endl;
      delete []lJointValues;
        
      lFS.getline(lLine,gBufSize); // comment
      lFS.getline(lLine,gBufSize); // positions, not required for loading
      lFS.getline(lLine,gBufSize); // comment
      lFS.getline(lLine,gBufSize); // hand orientation
      buola::CQuaternion lHandQ;
      std::stringstream lHandQSS(lLine);
      lHandQSS >> lHandQ;
      lFS.getline(lLine,gBufSize); // comment
      lFS.getline(lLine,gBufSize); // hand translation
      buola::C3DVector lHandT;
      std::stringstream lHandTSS(lLine);
      lHandTSS >> lHandT;
      lHandTransf->SetRotation(lHandQ);
      lHandTransf->SetTranslation(lHandT);
      
      lFS.getline(lLine,gBufSize); // comment
      lFS.getline(lLine,gBufSize); // obj orientation
      buola::CQuaternion lObjQ;
      std::stringstream lObjQSS(lLine);
      lObjQSS >> lObjQ;
      lFS.getline(lLine,gBufSize); // comment
      lFS.getline(lLine,gBufSize); // obj translation
      buola::C3DVector lObjT;
      std::stringstream lObjTSS(lLine);
      lObjTSS >> lObjT;
      lObjTransf->SetRotation(lObjQ);
      lObjTransf->SetTranslation(lObjT);
      
      lFS.getline(lLine,gBufSize); // comment
      lFS.getline(lLine,gBufSize); // obj Transform
      lObjectPath = std::string(lLine);
    }
    else
    {
      const std::vector<std::string> &lJointsV=CProgramOption::GetArgs();
      // if the array doesn't have all the 9+5*(3+2) values, set the joints to zero
      if(lJointsV.size()!=34)
      {
        std::cerr << "Joint vector passed has wrong size " << lJointsV.size() << ". Setting to 0" << std::endl;
        for(int i=0;i<17;++i)
          for(int a=0;a<3;++a)
            lSkeleton[i]->SetJointValue(gJointTypes[a],0);
      }
      else // if it has the required values, disregard the orientation and set the joints to the passed values
      {
        // first two joints always have the same values
        double lArmWristJoints[6]={0.000,0.000,0.000,
                                   buola::deg2rad(2.100),buola::deg2rad(7.102),buola::deg2rad(-3.408)};
        unsigned lFingerEquivalence[5]={4,0,1,2,3};
        for(int a=0;a<2;a++)
          for(int j=0;j<3;j++)
            lSkeleton[a]->SetJointValue(gJointTypes[j],lArmWristJoints[a*3+j]);
          
        // for the rest of the joints, each finger has 3+1+1 values.
        for(int f=0;f<5;++f)
          for(int a=0;a<3;++a)
            for(int j=0;j<3;++j)
            {
              if(a!=0 && j!=0)
                lSkeleton[f*3+a+2]->SetJointValue(gJointTypes[j],0.0);
              else if(a==0)
                lSkeleton[f*3+a+2]->SetJointValue(gJointTypes[j],asin(stod(lJointsV[lFingerEquivalence[f]*5+j+9])));
              else if(a==1)
                lSkeleton[f*3+a+2]->SetJointValue(gJointTypes[j],asin(stod(lJointsV[lFingerEquivalence[f]*5+3+9])));
              else if(a==2)
                lSkeleton[f*3+a+2]->SetJointValue(gJointTypes[j],asin(stod(lJointsV[lFingerEquivalence[f]*5+4+9])));
            }
            
      }
    }
    
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
