#ifndef _CCAMSLIDER_H_
#define _CCAMSLIDER_H_

#include <buola/widgets/csliderbox.h>
#include <buola/widgets/clabelbox.h>
#include <buola/gui/cwindow.h>
#include <buola/geometry/c3dvector.h>

class CCamSlider
{
    private:
        buola::gui::CSliderBox  *mSliderBox;
        buola::C3DRotation &mRot;
        double buola::C3DRotation::*mRotVal;
        buola::scene::PPerspectiveCamera mCam;
        buola::scene::CSceneView *mView;

    public:
        CCamSlider(buola::gui::CSliderBox *pSliderBox,
                buola::C3DRotation &pRot,
                double buola::C3DRotation::*pRotVal,
                buola::scene::PPerspectiveCamera pCam,
                buola::scene::CSceneView *pView):
            mSliderBox(pSliderBox),
            mRot(pRot),
            mRotVal(pRotVal),
            mCam(pCam),
            mView(pView){}
        inline void OnSlider()
        {
            mRot.*mRotVal=mSliderBox->GetValue();
            buola::C3DRotation lV(mRot*M_PI/100.0);
            std::cout << mRot << std::endl;
            mCam->LookAt(buola::C3DVector(0,0,0),lV,0.5);
        }
};

#endif

