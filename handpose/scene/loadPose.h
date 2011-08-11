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
#ifndef __LOADPOSE_H
#define __LOADPOSE_H

#include "cDBelement.h" // conflicts with X

#include <stdlib.h>

#include <sstream>
#include <iomanip>
#include <iterator>
#include <algorithm>
#include <vector>
#include <fstream>

#include <buola/scene.h>
#include <buola/scene/csceneview.h>
#include <buola/scene/ccamera.h>
#include <buola/app/capp.h>
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

#include <buola/widgets/cbutton.h>
#include <buola/app/ccmdline.h>

#include <buola/geometry/clookatmatrix.h>
#include <buola/geometry/c3dvector.h>
#include <buola/geometry/cperspectiveprojectionmatrix.h>

#include "CHandSkeleton.h"
#include "handclass_config.h"

using namespace buola;

static const unsigned gBufSize(1024);

void setTransf(const std::string &pPos,const std::string &pOri,scene::PRTTransform &pTransf);

void setPose(CHandSkeleton &pSkeleton);

void loadPose(const fsystem::path &pPosePath,CHandSkeleton &pSkeleton,scene::PRTTransform &pHandTransf,
              scene::PRTTransform &pObjTransf,std::string &pObjectPath,double *pCam2PalmRArray);

#endif
