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
#ifndef _CCAMSLIDER_H_
#define _CCAMSLIDER_H_

#include <buola/widgets/csliderbox.h>
#include <buola/widgets/clabelbox.h>
#include <buola/gui/cwindow.h>
#include <buola/geometry/c3dvector.h>

static const double gCamDistance(0.3);
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
            mCam->LookAt(buola::C3DVector(0,0,0),lV,gCamDistance);
        }
};

#endif

