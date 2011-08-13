#include <sstream>
#include "csavebutton.h"

const std::string getCam2PalmR(const CHandSkeleton &pSkeleton,const buola::scene::PPerspectiveCamera &pCamera)
{
  std::ostringstream lS;
  buola::CQuaternion lHandQuat(pSkeleton[BONE_FOREARM]->GetTransform()->GetWorldTransform());
  buola::CQuaternion lCamQuat(pCamera->GetLookAtMatrix());
  buola::CQuaternion lHandCamQuat=conj(lCamQuat)*lHandQuat;
  //std::cout << pCamera->GetLookAtMatrix() << std::endl;
  //std::cout << lHandQuat << "|" << lCamQuat << "|" << lHandCamQuat << std::endl;
  buola::C3DMatrix lHandCamMat(buola::nRotate,lHandCamQuat);
  for(int i=0;i<3;++i)
    for(int j=0;j<3;++j)
      lS << lHandCamMat(i,j) << " ";
  return lS.str();
}

const std::string partsLocation2String(const CHandSkeleton &pSkeleton,const buola::scene::PPerspectiveCamera &pCamera)
{
  std::ostringstream lS;
  buola::C3DVector lForearm=pSkeleton[BONE_FOREARM]->GetTransformedEndPoint();
  std::vector<int> lPosesToSave {BONE_THUMB1,BONE_INDEX1,BONE_MIDDLE1,BONE_RING1,BONE_PINKY1,
                                 BONE_THUMB3,BONE_INDEX3,BONE_MIDDLE3,BONE_RING3,BONE_PINKY3};
  buola::CQuaternion lCamQuat(pCamera->GetLookAtMatrix());
  for(int i=0;i<lPosesToSave.size();++i)
  {
    buola::C3DVector lPose=lCamQuat*(pSkeleton[lPosesToSave[i]]->GetTransformedEndPoint()-lForearm);
    lS << lPose.x << " " << lPose.y << " " << lPose.z << " ";
  }
  return lS.str();
}

CSaveButton::CSaveButton(const CHandSkeleton &pSkeleton,
    const buola::scene::PRTTransform &pHandTransformation,
    const buola::scene::PRTTransform &pObjTransformation,
    const buola::scene::PPerspectiveCamera &pCamera,
    const std::string &pObjectObjPath,
    buola::scene::CSceneView *pScene):
  mSkeleton(pSkeleton),
  mHandTransformation(pHandTransformation),
  mObjTransformation(pObjTransformation),
  mCamera(pCamera),
  mObjectObjPath(pObjectObjPath),
  mScene(pScene)
{
  if(pScene!=NULL)
  {
    mButton.Create(dynamic_cast<buola::gui::CWindow*>(pScene),buola::CPoint(300,10),buola::CSize(50,20));
    //mButton.CreateAndSet(dynamic_cast<buola::gui::CWindow*>(pScene),buola::CPoint(300,300),buola::CSize(100,100),NULL);
    mButton.SetCaption(L"save");
    mButton.ePressed.Connect(&CSaveButton::OnPressed,this);
    mButton.Show();
  }
}

void CSaveButton::OnPressed()
{
  if(mURL.IsEmpty())
  {
    using namespace buola;
    CFileDialog lDialog;

    lDialog.SetStyle(FDS_SAVE);
    lDialog.SetCurDir("/home/javier");
    lDialog.Create(NULL);
    lDialog.StayOnTopOf(mScene);
    lDialog.DoModal();

    if(lDialog.mResult==buola::gui::DIALOG_OK)
      mURL=lDialog.mFile;
    else
      return ;
  }

  std::ofstream lFS(mURL.GetFullPath().c_str());
  lFS << "# hand orientation with respect to the camera" << std::endl;
  lFS << getCam2PalmR(mSkeleton,mCamera) << std::endl;
  lFS << "# hand joints (51)" << std::endl;
  for(int j=0;j<17;++j)
    for(int a=0;a<3;++a)
      lFS << mSkeleton[j]->GetJointValue(gJointTypes[a]) << " ";
  lFS << std::endl;
  lFS << "# fingerbases and tips positions" << std::endl;
  lFS << partsLocation2String(mSkeleton,mCamera) << std::endl;
  lFS << " # hand orientation" << std::endl;
  buola::CQuaternion lHandQ = mHandTransformation->GetRotation();
  //             lFS << lHandQ.w << " " << lHandQ.x << " " << lHandQ.y << " " << lHandQ.z << std::endl;
  lFS << lHandQ << std::endl;
  lFS << " # hand position" << std::endl;
  buola::C3DVector lHandT = mHandTransformation->GetTranslation();
  //lFS << lHandT.x << " " << lHandT.y << " " << lHandT.z << std::endl;
  lFS << lHandT << std::endl;
  lFS << " # obj orientation" << std::endl;
  buola::CQuaternion lObjQ = mObjTransformation->GetRotation();
  //lFS << lObjQ.w << " " << lObjQ.x << " " << lObjQ.y << " " << lObjQ.z << std::endl;
  lFS << lObjQ << std::endl;
  lFS << " # obj position" << std::endl;
  buola::C3DVector lObjT = mObjTransformation->GetTranslation();
  //lFS << lObjT.x << " " << lObjT.y << " " << lObjT.z << std::endl;
  lFS << lObjT << std::endl;
  lFS << "# object path" << std::endl;
  lFS << mObjectObjPath << std::endl;
  // FIXME: write something for the camera that can be read afterwards
  lFS << "# camera" << std::endl;
  lFS << "0 0 0 0 -1 0 0 0 1" << std::endl;
  lFS << "# next indices (dummy for now)" << std::endl;
  lFS << "1 2 3 4 5" << std::endl;
  lFS.close();
}
