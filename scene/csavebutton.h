#ifndef _CSAVEBUTTON_H_
#define _CSAVEBUTTON_H_

#include <buola/widgets/cbutton.h>
#include <buola/fsgui.h>
#include <buola/fsgui/cfiledialog.h>
#include <buola/gui/cwindow.h>
#include <fstream>
#include <iterator>
#include <algorithm>
#include "CHandSkeleton.h"

class CSaveButton
{
    private:
        buola::gui::CButton  mButton;
        const CHandSkeleton mSkeleton;
        const buola::scene::PRTTransform &mHandTransformation;
        const buola::scene::PRTTransform &mObjTransformation;
        const buola::scene::PPerspectiveCamera &mCamera;
        const std::string &mObjectObjPath;
        buola::scene::CSceneView *mScene;
        buola::CURL mURL;
    public:
        CSaveButton(const CHandSkeleton &pSkeleton,
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
        void setURL(const buola::CURL &pURL){mURL=pURL;}
        inline void OnPressed()
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

            buola::CQuaternion lHandQuat(mSkeleton[BONE_FOREARM]->GetTransform()->GetWorldTransform());
            buola::CQuaternion lCamQuat(mCamera->GetLookAtMatrix());
            // order still unclear
            buola::CQuaternion lHandCamQuat=conj(lCamQuat)*lHandQuat;
            buola::C3DMatrix lHandCamMat(buola::nRotate,lHandCamQuat);
            std::ofstream lFS(mURL.GetFullPath().c_str());
            lFS << "# hand orientation with respect to the camera" << std::endl;
            for(int i=0;i<3;++i)
                for(int j=0;j<3;++j)
                    lFS << lHandCamMat(i,j) << " ";
            lFS << std::endl;
            lFS << "# hand joints (51)" << std::endl;
            for(int j=0;j<17;++j)
                for(int a=0;a<3;++a)
                    lFS << mSkeleton[j]->GetJointValue(gJointTypes[a]) << " ";
            lFS << std::endl;
            lFS << "# hand wrist and fingertip positions" << std::endl;
            std::vector<int> lPosesToSave {BONE_FOREARM,BONE_THUMB3,BONE_INDEX3,BONE_MIDDLE3,BONE_RING3,BONE_PINKY3};
            for(int i=0;i<lPosesToSave.size();++i)
            {
              buola::C3DVector lPose=mSkeleton[lPosesToSave[i]]->GetTransformedEndPoint();
              lFS << lPose.x << " " << lPose.y << " " << lPose.z << " ";
            }
            lFS << std::endl;
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
//             lFS << "# hand transform" << std::endl;
//             buola::C3DMatrix lHandMatrix = mHandTransformation->GetWorldTransform();
//             for(int i=0;i<16;++i)
//               lFS << lHandMatrix[i] << " ";
//             lFS << std::endl;
//             lFS << "# object transform" << std::endl;
//             buola::C3DMatrix lObjMatrix = mObjTransformation->GetWorldTransform();
//             for(int i=0;i<16;++i)
//               lFS << lObjMatrix[i] << " ";
//             lFS << std::endl;
            lFS << "# object path" << std::endl;
            lFS << mObjectObjPath << std::endl;
            lFS.close();
        }
};

#endif


