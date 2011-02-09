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

#include <buola/geometry/clookatmatrix.h>
#include <buola/geometry/c3dvector.h>
#include <buola/geometry/cperspectiveprojectionmatrix.h>

#include <iterator>
#include <vector>

#include "csavebutton.h"
#include "cjointslider.h"
#include "cposslider.h"
#include "ccamslider.h"
#include "CHandSkeleton.h"

static const buola::C3DVector lHandZero(0.31,-0.57,0.02);
static const buola::C3DVector lObjZero(0,0,0);

using namespace buola;

static const scene::EJointType gJointTypes[3] = {buola::scene::JOINTTYPE_BEND,buola::scene::JOINTTYPE_SIDE,buola::scene::JOINTTYPE_TWIST};
static const std::string gJointTypeNames[3] = {"bend","side","twist"};

void addCamSlider(int y,scene::CSceneView *pScene,std::vector<CCamSlider*> &pRots,
        C3DRotation &pRot,double buola::C3DRotation::*pRotVal,
        const std::string &pCaption,scene::PPerspectiveCamera &pCam)
{
    gui::CLabelBox *lLabel=new gui::CLabelBox;
    // dynamic cast unnecesary
	lLabel->Create(dynamic_cast<gui::CWindow*>(pScene),CPoint(0,y),CSize(150,10));
	lLabel->SetStyle(LBL_TOP);
    // L"..." creates them wide directly 
	lLabel->SetCaption(decode(pCaption));
	lLabel->Show();

    gui::CSliderBox *lBox=new gui::CSliderBox;
	lBox->Create(dynamic_cast<gui::CWindow*>(pScene),CPoint(20,y+10),CSize(150,10));
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
	lLabel->Create(dynamic_cast<gui::CWindow*>(pScene),CPoint(0,y),CSize(150,10));
	lLabel->SetStyle(LBL_TOP);
	lLabel->SetCaption(decode(pCaption));
	lLabel->Show();

    gui::CSliderBox *lBox=new gui::CSliderBox;
	lBox->Create(dynamic_cast<gui::CWindow*>(pScene),CPoint(20,y+10),CSize(150,10));
	lBox->SetStyle(gui::CSliderBox::STYLE_HORIZONTAL);
	lBox->SetRange(-100,100);
	lBox->SetValue(0);
	lBox->Show();

    CPosSlider *lSlider=new CPosSlider(lBox,pV,pPosVal,pTransf,pScene);
	pPoses.push_back(lSlider);

	lBox->eChanged.connect(boost::bind(&CPosSlider::OnSlider,lSlider));
}

void addSlider(int y,const scene::EJointType &pType,scene::PBone &pBone,
        scene::CSceneView *pScene, std::vector<CJointSlider*> &pSliders,double lDefaultValue)
{
    gui::CLabelBox *lLabel=new gui::CLabelBox;
	lLabel->Create(dynamic_cast<gui::CWindow*>(pScene),CPoint(0,y),CSize(150,10));
	lLabel->SetStyle(LBL_TOP);
	lLabel->SetCaption(decode(pBone->GetName()+gJointTypeNames[pType]));
	lLabel->Show();

    gui::CSliderBox *lBox=new gui::CSliderBox;
	lBox->Create(dynamic_cast<gui::CWindow*>(pScene),CPoint(20,y+10),CSize(150,10));
	lBox->SetStyle(gui::CSliderBox::STYLE_HORIZONTAL);
	lBox->SetRange(-180,180);
	lBox->SetValue(lDefaultValue);
	lBox->Show();

	CJointSlider *lSlider=new CJointSlider(lLabel,lBox,pType,pBone,pScene);
	pSliders.push_back(lSlider);
    lSlider->OnSlider();

	lBox->eChanged.connect(boost::bind(&CJointSlider::OnSlider,lSlider));
}


int main(int pNArg,char **pArgs)
{
    buola_init(pNArg,pArgs);

    std::vector<double> lJointsV;
    double lCam2PalmRArray[9];
    if(pNArg<3)
    {
        std::cerr << "command -- objectObjPath [ joint0 joint1 .. joint 33]" << std::cout;
        exit(-1);
    }
    //"/home/javier/tmp/objAfterFeedback/objs/adductedThumb_onlyObject.obj"
    scene::PGeode lGeode=scene::CGeode::Import(pArgs[2],0.1);
    std::string lObjectObjPath(pArgs[2]);
    if(pNArg>3)
    {
        for(int i=3;i<pNArg;++i)
            lJointsV.push_back(atof(pArgs[i]));
        for(int i=0;i<9;i++)
            lCam2PalmRArray[i] = lJointsV[i];
    }
    double *lJointValues=(lJointsV.size()==34)?new double[51]{
        0.000,0.000,0.000,   // arm, to keep or modify based on Rot matrix (0-8)
        2.100,7.102,-3.408,  // wrist, to keep
        asin(lJointsV[29])*180.0/M_PI,asin(lJointsV[30])*180.0/M_PI,asin(lJointsV[31])*180.0/M_PI,
        asin(lJointsV[32])*180.0/M_PI,0.0,0.0,
        asin(lJointsV[33])*180.0/M_PI,0.0,0.0,
        asin(lJointsV[9])*180.0/M_PI,asin(lJointsV[10])*180.0/M_PI,asin(lJointsV[11])*180.0/M_PI,
        asin(lJointsV[12])*180.0/M_PI,0.0,0.0,
        asin(lJointsV[13])*180.0/M_PI,0.0,0.0,
        asin(lJointsV[14])*180.0/M_PI,asin(lJointsV[15])*180.0/M_PI,asin(lJointsV[16])*180.0/M_PI,
        asin(lJointsV[17])*180.0/M_PI,0.0,0.0,
        asin(lJointsV[18])*180.0/M_PI,0.0,0.0,
        asin(lJointsV[19])*180.0/M_PI,asin(lJointsV[20])*180.0/M_PI,asin(lJointsV[21])*180.0/M_PI,
        asin(lJointsV[22])*180.0/M_PI,0.0,0.0,
        asin(lJointsV[23])*180.0/M_PI,0.0,0.0,
        asin(lJointsV[24])*180.0/M_PI,asin(lJointsV[25])*180.0/M_PI,asin(lJointsV[26])*180.0/M_PI,
        asin(lJointsV[27])*180.0/M_PI,0.0,0.0,
            asin(lJointsV[28])*180.0/M_PI,0.0,0.0
    }:new double[51]{0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
    C3DMatrix cam2PalmR=(lJointsV.size()==34)?C3DMatrix(n9Array,lCam2PalmRArray):C3DMatrix();
    C3DMatrix palm2CamR = inverse(cam2PalmR);

    try
    {
        scene::CSceneView lView;
        lView.Create(NULL,CSize(800,600));
        CApp lApp;

        scene::PPerspectiveCamera lCamera=new scene::CPerspectiveCamera;
        lCamera->LookAt(C3DVector(0,0,0),C3DRotation(0,0,0),0.5);
        lCamera->SetClipping(0.01,200);

        scene::PScene lScene=new scene::CScene;
        std::vector<CJointSlider*> lSliders;

        CHandSkeleton lSkeleton("/home/javier/scene/rHandP3.obj","/home/javier/scene/hand_texture.ppm");

        std::string lSliderNames[2]={"hand","object"};
        std::string lXYZ[3]={"X","Y","Z"};
        
        std::vector<CPosSlider*> lPosesV;
        std::vector<scene::PRTTransform> lTransformations;
        scene::PRTTransform lHandTransf=new scene::CRTTransform;
        scene::PRTTransform lObjTransf=new scene::CRTTransform;
        lTransformations.push_back(lHandTransf);
        lTransformations.push_back(lObjTransf);
        std::vector<buola::C3DVector> lPoses3DV(2);
        
        std::vector<CCamSlider*> lRotsV;
        buola::C3DRotation lRot;

        lHandTransf->SetTranslation(lHandZero);
        lScene->GetWorld()->AddChild(lHandTransf);
        lHandTransf->AddChild(lSkeleton.GetSkeleton()->GetRoot()->GetTransform());
        lScene->AddObject(lSkeleton.GetSkeleton());

        lObjTransf->SetTranslation(lObjZero);
        lGeode->AttachTo(lObjTransf);
        lScene->GetWorld()->AddChild(lObjTransf);

        addCamSlider((0*3+0)*20,&lView,lRotsV,lRot,&buola::C3DRotation::a,"cam yaw",lCamera);
        addCamSlider((0*3+1)*20,&lView,lRotsV,lRot,&buola::C3DRotation::b,"cam pitch",lCamera);
        addCamSlider((0*3+2)*20,&lView,lRotsV,lRot,&buola::C3DRotation::c,"cam roll",lCamera);
        
        for(int i=0;i<2;++i)
        {
                addXYZSlider(((i+1)*3+0)*20,&lView,lPosesV,lPoses3DV[i],&buola::C3DVector::x,lSliderNames[i]+lXYZ[0],lTransformations[i]);
                addXYZSlider(((i+1)*3+1)*20,&lView,lPosesV,lPoses3DV[i],&buola::C3DVector::y,lSliderNames[i]+lXYZ[1],lTransformations[i]);
                addXYZSlider(((i+1)*3+2)*20,&lView,lPosesV,lPoses3DV[i],&buola::C3DVector::z,lSliderNames[i]+lXYZ[2],lTransformations[i]);
        }

        for(int i=0;i<17;i++)
            for(int j=0;j<3;j++)
                addSlider(((i+3)*3+j)*20,gJointTypes[j],lSkeleton[i],&lView,lSliders,lJointValues[i*3+j]);
        
        lView.SetCamera(lCamera);
        lView.SetScene(lScene);

        CSaveButton(lJointValues,lTransformations,lCamera,lObjectObjPath,&lView);

        lView.Show();

        lApp.Run();
    }
    catch(std::exception &pE)
    {
        msg_info() << "caught exception " << pE.what() << "\n";
    }

    return buola_finish();
}
