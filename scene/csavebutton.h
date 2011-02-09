#ifndef _CSAVEBUTTON_H_
#define _CSAVEBUTTON_H_

#include <buola/widgets/cbutton.h>
#include <buola/gui/cwindow.h>
#include <fstream>

class CSaveButton
{
    private:
        buola::gui::CButton  mButton;
        const double *mJointValues;
        const std::vector<buola::scene::PRTTransform> &mTransformations;
        const buola::scene::PPerspectiveCamera &mCamera;
        const std::string &mObjectObjPath;
    public:
        CSaveButton(const double *pJointValues,
                    const std::vector<buola::scene::PRTTransform> &pTransformations,
                    const buola::scene::PPerspectiveCamera &pCamera,
                    const std::string &pObjectObjPath,
                    buola::scene::CSceneView *pScene):
            mJointValues(pJointValues),
            mTransformations(pTransformations),
            mCamera(pCamera),
            mObjectObjPath(pObjectObjPath)
        {
            mButton.Create(dynamic_cast<buola::gui::CWindow*>(pScene),buola::CPoint(200,10),buola::CSize(50,20));
            //mButton.CreateAndSet(dynamic_cast<buola::gui::CWindow*>(pScene),buola::CPoint(300,300),buola::CSize(100,100),NULL);
            mButton.SetCaption(L"save");
            mButton.ePressed.connect(MEMBER_THIS(OnPressed));
            mButton.Show();
        }
        inline void OnPressed()
        {
            std::cout << "button pressed" << std::endl;
            std::ofstream lFS(mObjectObjPath.c_str());
        }
};

#endif


