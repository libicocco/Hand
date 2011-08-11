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

const std::string getCam2PalmR(const CHandSkeleton &pSkeleton,const buola::scene::PPerspectiveCamera &pCamera);

const std::string partsLocation2String(const CHandSkeleton &pSkeleton,const buola::scene::PPerspectiveCamera &pCamera);

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
        buola::scene::CSceneView *pScene);

    void saveURL(const buola::CURL &pURL){mURL=pURL;OnPressed();}
    inline void OnPressed();
};

#endif


