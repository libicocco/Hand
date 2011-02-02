#ifndef _CJOINTSLIDER_H_
#define _CJOINTSLIDER_H_

//#include <cwindow.h>
#include <buola/widgets/csliderbox.h>
#include <buola/widgets/clabelbox.h>
#include <buola/gui/cwindow.h>

class CJointSlider
{
    private:
        buola::gui::CLabelBox   *mLabelBox;
        buola::gui::CSliderBox  *mSliderBox;
        buola::scene::EJointType  mType;
        buola::scene::PBone       mBone;
        buola::scene::CSceneView *mView;

    public:
        CJointSlider(buola::gui::CLabelBox *pLabelBox,
                buola::gui::CSliderBox *pSliderBox,
                buola::scene::EJointType pType,
                buola::scene::PBone pBone,
                buola::scene::CSceneView *pView):
            mLabelBox(pLabelBox),
            mSliderBox(pSliderBox),
            mType(pType),
            mBone(pBone),
            mView(pView){}
        inline void OnSlider()
        {
            mBone->SetJointValue(mType,buola::deg2rad(mSliderBox->GetValue()));
            //std::cout << mBone->GetName() << "_" << mType << " = " << mSliderBox->GetValue() << "," << mBone->GetTransformedEndPoint().x << " " << mBone->GetTransformedEndPoint().y << " " << mBone->GetTransformedEndPoint().z << " " << std::endl;
            mBone->GetTransform()->Update();
            mView->Refresh();
        }
};

#endif
