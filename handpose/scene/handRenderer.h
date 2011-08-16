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
#ifndef __HANDRENDERER_H
#define __HANDRENDERER_H

#include <buola/image/algorithm/detail/opencv.h>
#include "hog.h" // from handclass

#include <stdlib.h>

#include <sstream>
#include <vector>

#include <buola/scene.h>
#include <buola/scene/csceneview.h>
#include <buola/scene/ccamera.h>
#include <buola/scene/cperspectivecamera.h>
#include <buola/scene/cscene.h>
#include <buola/scene/cmesh.h>
#include <buola/scene/cskeleton.h>
#include <buola/scene/cbone.h>
#include <buola/scene/cgeode.h>
#include <buola/image/format.h>
#include <buola/image/io.h>
#include <buola/scene/crttransform.h>
#include <buola/scene/cimagerenderer.h>

#include <buola/geometry/clookatmatrix.h>
#include <buola/geometry/c3dvector.h>
#include <buola/geometry/cperspectiveprojectionmatrix.h>

#include <iterator>

#include "CHandSkeleton.h"

#include "cDBelement.h"
#include "loadPose.h"
#include "ccamslider.h"
#include "csavebutton.h"

using namespace buola;


class HandRenderer
{
  public:
    HandRenderer(const char* pHandObjPath,const char* pHandTexturePath);

    // give a CDBelement, render the image in the path set inside it
    void render(const CDBelement &pDBelem);

    // save information about the rendering in the db element
    void saveInfo(CDBelement &pDBelem,std::ofstream *pHogOFS=NULL);

  private:

    void setCamera(const double *pCam2PalmRArray);

    void computeHog();

    const std::string getCam2PalmR();

    scene::PPerspectiveCamera mCamera;
    scene::PScene mScene;
    scene::CImageRenderer mRenderer;
    buola::img::CImage_rgb8 mImage;
    scene::PRTTransform mHandTransf;
    scene::PRTTransform mObjTransf;
    CHandSkeleton mSkeleton;
    Hog<float> mHog;
    std::vector<float> mFeature;
    std::string mObjPath;
    buola::CQuaternion mHandQ;
};

#endif // __HANDRENDERER_H
