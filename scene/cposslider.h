#ifndef _CPOSSLIDER_H_
#define _CPOSSLIDER_H_

#include <buola/widgets/csliderbox.h>
#include <buola/widgets/clabelbox.h>
#include <buola/gui/cwindow.h>
#include <buola/geometry/c3dvector.h>

class CPosSlider
{
    private:
        buola::gui::CSliderBox  *mSliderBox;
        buola::C3DVector &mPos;
        double buola::C3DVector::*mPosVal;
        boost::intrusive_ptr<buola::scene::CRTTransform> mTransf;
        buola::C3DVector mPosZero;
        buola::scene::CSceneView *mView;

    public:
        CPosSlider(buola::gui::CSliderBox *pSliderBox,
                buola::C3DVector &pPos,
                double buola::C3DVector::*pPosVal,
                boost::intrusive_ptr<buola::scene::CRTTransform> pTransf,
                buola::scene::CSceneView *pView):
            mSliderBox(pSliderBox),
            mPos(pPos),
            mPosVal(pPosVal),
            mTransf(pTransf),
            mPosZero(pTransf->GetTranslation()),
            mView(pView){}
        inline void OnSlider()
        {
            mPos.*mPosVal=mSliderBox->GetValue();
            buola::C3DVector lV(mPos/100+mPosZero);
            std::cout << lV << std::endl;
            mTransf->SetTranslation(lV);
            mView->Refresh();
        }
};

#endif
