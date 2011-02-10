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

using namespace buola;

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
        scene::CSceneView *pScene, std::vector<CJointSlider*> &pSliders)
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

    if(gObjectPathOption.IsSet())
    {
      lObjectPath=gObjectPathOption.GetValue();
      std::cout << "object path set to " << lObjectPath << std::endl;
    }
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
      std::cout << "Parsing joint values" << std::endl;
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
      lFS.getline(lLine,gBufSize); // positions, not required
      lFS.getline(lLine,gBufSize); // comment
      lFS.getline(lLine,gBufSize); // hand Transform
      double *lHT=new double[16];
      std::stringstream lHTSS(lLine);
      std::copy(std::istream_iterator<double>(lHTSS),
                std::istream_iterator<double>(),
                lHT);
      lFS.getline(lLine,gBufSize); // comment
      lFS.getline(lLine,gBufSize); // obj Transform
      double *lOT=new double[16];
      std::stringstream lOTSS(lLine);
      std::copy(std::istream_iterator<double>(lOTSS),
                std::istream_iterator<double>(),
                lOT);
      lFS.getline(lLine,gBufSize); // comment
      lFS.getline(lLine,gBufSize); // obj Transform
      lObjectPath = std::string(lLine);
//       std::copy(lC,lC+9,std::ostream_iterator<double>(std::cout," "));
//       std::cout << std::endl;
//       std::copy(lHT,lHT+16,std::ostream_iterator<double>(std::cout," "));
//       std::cout << std::endl;
//       std::copy(lOT,lOT+16,std::ostream_iterator<double>(std::cout," "));
//       std::cout << std::endl << lOS << std::endl;
    }
    else
    {
      const std::vector<std::string> &lJointsV=CProgramOption::GetArgs();
      if(lJointsV.size()!=34)
      {
        std::cerr << "Joint vector passed has wrong size " << lJointsV.size() << ". Setting to 0" << std::endl;
        for(int i=0;i<17;++i)
          for(int a=0;a<3;++a)
            lSkeleton[i]->SetJointValue(gJointTypes[a],0);
        for(int i=0;i<16;++i)
          if(i%4!=0)
            lCam2PalmRArray[i]=0;
          else
            lCam2PalmRArray[i]=1;
      }
      else
      {
        double lArmWristJoints[6]={0.000,0.000,0.000,2.100,7.102,-3.408};//fixed 
        unsigned lFingerEquivalence[5]={4,0,1,2,3};
        
        for(int a=0;a<2;a++)
          for(int j=0;j<3;j++)
            lSkeleton[a]->SetJointValue(gJointTypes[j],lArmWristJoints[a*3+j]);
        for(int f=0;f<5;++f)
          for(int a=0;a<3;++a)
            for(int j=0;j<3;++j)
            {
              if(a!=0 && j!=0)
                lSkeleton[f*3+a+2]->SetJointValue(gJointTypes[j],0.0);
              else if(a==0)
                lSkeleton[f*3+a+2]->SetJointValue(gJointTypes[j],asin(stod(lJointsV[lFingerEquivalence[f]*5+j+9]))*180.0/M_PI);
              else if(a==1)
                lSkeleton[f*3+a+2]->SetJointValue(gJointTypes[j],asin(stod(lJointsV[lFingerEquivalence[f]*5+3+9]))*180.0/M_PI);
              else if(a==2)
                lSkeleton[f*3+a+2]->SetJointValue(gJointTypes[j],asin(stod(lJointsV[lFingerEquivalence[f]*5+4+9]))*180.0/M_PI);
            }
            
        for(int i=0;i<16;++i)
          lCam2PalmRArray[i]=0;
        for(int i=0;i<9;++i)
          lCam2PalmRArray[i+i/3]=stod(lJointsV[i]);
        lCam2PalmRArray[15]=1;
        
        for(int i=0;i<17;++i)
        {
          for(int a=0;a<3;++a)
            std::cout << lSkeleton[i]->GetJointValue(gJointTypes[a]) << " ";
          std::cout << std::endl;
        }
      }
    }
    
    scene::PGeode lGeode=buola::scene::CGeode::Import(lObjectPath.c_str(),0.1);
    std::cout << pArgs[2] << std::endl;
    std::string lObjectObjPath(pArgs[2]);
    
    //C3DMatrix cam2PalmR=C3DMatrix(n9Array,lCam2PalmRArray);
    C3DMatrix cam2PalmR=C3DMatrix(lCam2PalmRArray[0],lCam2PalmRArray[1],lCam2PalmRArray[2],lCam2PalmRArray[3],
                                  lCam2PalmRArray[4],lCam2PalmRArray[5],lCam2PalmRArray[6],lCam2PalmRArray[7],
                                  lCam2PalmRArray[8],lCam2PalmRArray[9],lCam2PalmRArray[10],lCam2PalmRArray[11],
                                  lCam2PalmRArray[12],lCam2PalmRArray[13],lCam2PalmRArray[14],lCam2PalmRArray[15]);
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

        if(gPosePathOption.IsSet())
        {
          std::cout << "TODO: set lHandTransf and lScene based on the file" << std::endl;
        }
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
          {
//             lSkeleton[i]->SetJointValue(gJointTypes[j],lJointValues[i*3+j]);
            addSlider(((i+3)*3+j)*20,gJointTypes[j],lSkeleton[i],&lView,lSliders);
          }
        
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
