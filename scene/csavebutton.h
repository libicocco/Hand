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
        const buola::scene::PPerspectiveCamera &mCamera;
        const std::string &mObjectObjPath;
        buola::scene::CSceneView *mScene;
        buola::CURL mURL;
    public:
        CSaveButton(const CHandSkeleton &pSkeleton,
                    const buola::scene::PRTTransform &pHandTransformation,
                    const buola::scene::PPerspectiveCamera &pCamera,
                    const std::string &pObjectObjPath,
                    buola::scene::CSceneView *pScene):
            mSkeleton(pSkeleton),
            mHandTransformation(pHandTransformation),
            mCamera(pCamera),
            mObjectObjPath(pObjectObjPath),
            mScene(pScene)
        {
            mButton.Create(dynamic_cast<buola::gui::CWindow*>(pScene),buola::CPoint(200,10),buola::CSize(50,20));
            //mButton.CreateAndSet(dynamic_cast<buola::gui::CWindow*>(pScene),buola::CPoint(300,300),buola::CSize(100,100),NULL);
            mButton.SetCaption(L"save");
            mButton.ePressed.connect(MEMBER_THIS(OnPressed));
            mButton.Show();
        }
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
                {
                    std::cout << "Dialog OK!" << std::endl;
                    mURL=lDialog.mFile;
                    std::cout << "Dialog done!" << std::endl;
                }
                else
                {
                    std::cout << "Dialog not OK!" << std::endl;
                    return ;
                }
            }

            buola::CQuaternion lHandQuat(mHandTransformation->GetWorldTransform());
            buola::CQuaternion lCamQuat(mCamera->GetLookAtMatrix());
            // order still unclear
            buola::CQuaternion lHandCamQuat=conj(lCamQuat)*lHandQuat;
            buola::C3DMatrix lHandCamMat(buola::nRotate,lHandCamQuat);
            //std::cout << lHandCamQuat << std::endl;
            //std::cout << lHandCamMat<< std::endl;
            std::ofstream lFS("/tmp/pose.txt");
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
            lFS.close();
        }
};

#endif


