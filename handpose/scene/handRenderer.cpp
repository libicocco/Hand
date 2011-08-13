#include "handRenderer.h"
HandRenderer::HandRenderer(const char* pHandObjPath,const char* pHandTexturePath):
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
  mRenderer.SetClearColor(buola::CColor(0,0,0));
}

void HandRenderer::render(const CDBelement &pDBelem)
{
  // set joints in the skeleton
  tFullPoseV lFullPose;
  pDBelem.getFullPose(lFullPose);
  for(int i=0;i<17;++i)
    for(int a=0;a<3;++a)
      mSkeleton[i]->SetJointValue(gJointTypes[a],lFullPose[i*3+a]);

  // place hand in scene according to mHandTransf
  std::string lHandPos = pDBelem.getHandPos();
  if(!lHandPos.compare("(0,0,0)"))
    setTransf("(0.31,-0.57,0.02)","(1 0 0 0)",mHandTransf);
  else
    setTransf(lHandPos,pDBelem.getHandOri(),mHandTransf);
  mScene->GetWorld()->AddChild(mHandTransf);
  mHandTransf->AddChild(mSkeleton.GetSkeleton()->GetRoot()->GetTransform());
  mScene->AddObject(mSkeleton.GetSkeleton());

  mObjPath=pDBelem.getObjPath();
  scene::PGeode lGeode;
  if(!mObjPath.empty())
  {
    // place object in scene according to mObjTransf
    std::string lObjPos = pDBelem.getObjPos();
    if(!lObjPos.compare("(0,0,0)"))
      setTransf("(0,0,0)","(1 0 0 0)",mObjTransf);
    else
      setTransf(lObjPos,pDBelem.getObjOri(),mObjTransf);

    lGeode=buola::scene::CGeode::Import(mObjPath.c_str(),0.1);
    lGeode->AttachTo(mObjTransf);
    mScene->GetWorld()->AddChild(mObjTransf);
  }

  mRenderer.SetScene(mScene);

  // set the camera
  std::string lCamUp = pDBelem.getCamUp();
  if(!lCamUp.compare("(0,0,0)"))
  {
    // cam defined by relative orientation wrt palm
    double lCam2PalmRArray[9];
    double lJoints[51];
    pDBelem.getOriJoints(lCam2PalmRArray,lJoints);
    setCamera(lCam2PalmRArray);
  }
  else
  {
    // cam defined by at-from-up
    C3DVector lAt,lFrom,lUp;
    std::istringstream lVectorsSS(pDBelem.getCamAt());
    lVectorsSS >> lAt;
    lVectorsSS.clear();
    lVectorsSS.str(pDBelem.getCamFrom());
    lVectorsSS >> lFrom;
    lVectorsSS.clear();
    lVectorsSS.str(lCamUp);
    lVectorsSS >> lUp;
    mCamera->LookAt(lAt,lFrom,lUp);
  }
  mRenderer.SetCamera(mCamera);

  // render to image
  mRenderer.GetImage(mImage);
  save(mImage,pDBelem.getImagePath());

  if(!mObjPath.empty())
    mObjTransf->RemoveObject(lGeode);
}

void HandRenderer::saveInfo(CDBelement &pDBelem,std::ofstream *pHogOFS)
{
  computeHog();
  pDBelem.setFeature(mFeature);
  if(pHogOFS!=NULL)
    pHogOFS->write(reinterpret_cast<char*>(mFeature.data()),mFeature.size()*sizeof(float));

  pDBelem.setPartsLocation(partsLocation2String(mSkeleton,mCamera));

  buola::C3DVector lAt,lFrom,lUp;
  lAt = mCamera->GetAt();
  lFrom = mCamera->GetFrom();
  lUp = mCamera->GetUp();
  pDBelem.setCamAtFromUp(lAt.x,lAt.y,lAt.z,lFrom.x,lFrom.y,lFrom.z,lUp.x,lUp.y,lUp.z);

  // either saving the atfromup or the cam2palm is redundant
  // but different cases needed one or another
  pDBelem.setOri(getCam2PalmR());
}
void HandRenderer::setCamera(const double *pCam2PalmRArray)
{
  buola::CQuaternion lHandQ(mSkeleton[BONE_FOREARM]->GetTransform()->GetWorldTransform());
  buola::CQuaternion lCam2Palm(pCam2PalmRArray);
  buola::C3DRotation lCamRotation(lCam2Palm*conj(lHandQ));
  buola::CQuaternion lOri(lCamRotation);
  C3DVector lAt(0,0,0);

  buola::C3DVector lUp=lOri*buola::C3DVector(0,1,0);
  buola::C3DVector lFrom=lAt-gCamDistance*(lOri*buola::C3DVector(0,0,-1));

  mCamera->LookAt(lAt,lFrom,lUp);

}
void HandRenderer::computeHog()
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

const std::string HandRenderer::getCam2PalmR()
{
  std::ostringstream lS;
  buola::CQuaternion lHandQuat(mSkeleton[BONE_FOREARM]->GetTransform()->GetWorldTransform());
  buola::CQuaternion lCamQuat(mCamera->GetLookAtMatrix());
  buola::CQuaternion lHandCamQuat=conj(lCamQuat)*lHandQuat;
  buola::C3DMatrix lHandCamMat(buola::nRotate,lHandCamQuat);
  for(int i=0;i<3;++i)
    for(int j=0;j<3;++j)
      lS << lHandCamMat(i,j) << " ";
  return lS.str();
}
