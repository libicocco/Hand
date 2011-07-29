/***********************************************
This file is part of the Hand project (https://github.com/libicocco/Hand).

    Copyright(c) 2007-2011 Javier Romero
    * jrgn AT kth DOT se

    Hand is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    Hand is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with Hand.  If not, see <http://www.gnu.org/licenses/>.

**********************************************/
#ifndef _CPOSSLIDER_H_
#define _CPOSSLIDER_H_

#include <buola/widgets/csliderbox.h>
#include <buola/widgets/clabelbox.h>
#include <buola/gui/cwindow.h>
#include <buola/geometry/c3dvector.h>

static const unsigned gPosSliderRange=10000;

class CPosSlider
{
    private:
        buola::gui::CSliderBox  *mSliderBox;
        buola::C3DVector &mPos;
        double buola::C3DVector::*mPosVal;
        buola::intrusive_ptr<buola::scene::CRTTransform> mTransf;
        buola::C3DVector mPosZero;
        buola::scene::CSceneView *mView;

    public:
        CPosSlider(buola::gui::CSliderBox *pSliderBox,
                buola::C3DVector &pPos,
                double buola::C3DVector::*pPosVal,
                buola::intrusive_ptr<buola::scene::CRTTransform> pTransf,
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
            buola::C3DVector lV(mPos/gPosSliderRange+mPosZero);
            std::cout << lV << std::endl;
            mTransf->SetTranslation(lV);
            mView->Refresh();
        }
};

#endif
