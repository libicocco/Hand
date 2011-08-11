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
            //std::cout << mBone->GetName() << "_" << mType << " = " << 
            //  mSliderBox->GetValue() << "," << mBone->GetTransformedEndPoint().x <<
            //  " " << mBone->GetTransformedEndPoint().y << " " << 
            //  mBone->GetTransformedEndPoint().z << " " << std::endl;
            mBone->GetTransform()->Update();
            mView->Refresh();
        }
};

#endif
