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

using namespace buola;

static const unsigned gBufSize(1024);

void setPose(CHandSkeleton &pSkeleton)
{
    const std::vector<std::string> &lJointsV=cmd_line().GetArgs();
    // if the array doesn't have all the 9+5*(3+2) values, set the joints to zero
    if(lJointsV.size()!=34)
    {
      std::cerr << "Joint vector passed has wrong size " << lJointsV.size() << ". Setting to 0" << std::endl;
      for(int i=0;i<17;++i)
        for(int a=0;a<3;++a)
          pSkeleton[i]->SetJointValue(gJointTypes[a],0);
    }
    else // if it has the required values, disregard the orientation and set the joints to the passed values
      {
        // first two joints always have the same values
        double lArmWristJoints[6]={0.000,0.000,0.000,
        buola::deg2rad(2.100),buola::deg2rad(7.102),buola::deg2rad(-3.408)};
        unsigned lFingerEquivalence[5]={4,0,1,2,3};
        for(int a=0;a<2;a++)
          for(int j=0;j<3;j++)
            pSkeleton[a]->SetJointValue(gJointTypes[j],lArmWristJoints[a*3+j]);
          
          // for the rest of the joints, each finger has 3+1+1 values.
            for(int f=0;f<5;++f)
              for(int a=0;a<3;++a)
                for(int j=0;j<3;++j)
                {
                  if(a!=0 && j!=0)
                    pSkeleton[f*3+a+2]->SetJointValue(gJointTypes[j],0.0);
                  else if(a==0)
                    pSkeleton[f*3+a+2]->SetJointValue(gJointTypes[j],asin(stod(lJointsV[lFingerEquivalence[f]*5+j+9])));
                  else if(a==1)
                    pSkeleton[f*3+a+2]->SetJointValue(gJointTypes[j],asin(stod(lJointsV[lFingerEquivalence[f]*5+3+9])));
                  else if(a==2)
                    pSkeleton[f*3+a+2]->SetJointValue(gJointTypes[j],asin(stod(lJointsV[lFingerEquivalence[f]*5+4+9])));
                }
      }
}

void loadPose(const CDBelement &pElem,CHandSkeleton &pSkeleton,scene::PRTTransform &pHandTransf,
              scene::PRTTransform &pObjTransf,std::string &pObjectPath,double *pCam2PalmRArray)
{
  std::stringstream lOriSS(pElem.getOriJoints());
  for(int i=0;i<9;++i)
    lOriSS >> pCam2PalmRArray[i];
  
  double *lJointValues=new double[51];
  for(int i=0;i<51;++i)
    lOriSS >> lJointValues[i];
  for(int i=0;i<17;++i)
    for(int a=0;a<3;++a)
      pSkeleton[i]->SetJointValue(gJointTypes[a],lJointValues[i*3+a]);
  delete []lJointValues;
  
  std::stringstream lHandOri(pElem.getHandOri());
  buola::CQuaternion lHandQ;
  lHandOri >> lHandQ;
  std::stringstream lHandPos(pElem.getHandPos());
  buola::C3DVector lHandT;
  lHandPos >> lHandT;
  pHandTransf->SetRotation(lHandQ);
  pHandTransf->SetTranslation(lHandT);
  
  std::stringstream lObjOri(pElem.getObjOri());
  buola::CQuaternion lObjQ;
  lObjOri >> lObjQ;
  std::stringstream lObjPos(pElem.getObjPos());
  buola::C3DVector lObjT;
  lObjPos >> lObjT;
  pObjTransf->SetRotation(lObjQ);
  pObjTransf->SetTranslation(lObjT);
  pObjectPath=pElem.getObjPath();
}

void loadPose(const std::string &pPosePath,CHandSkeleton &pSkeleton,scene::PRTTransform &pHandTransf,
              scene::PRTTransform &pObjTransf,std::string &pObjectPath,double *pCam2PalmRArray)
{
  std::ifstream lFS(pPosePath.c_str());
  char lLine[gBufSize];
  lFS.getline(lLine,gBufSize); // comment
  lFS.getline(lLine,gBufSize); // cam
  std::stringstream lOriSS(lLine);
  std::copy(std::istream_iterator<double>(lOriSS),
            std::istream_iterator<double>(),
            pCam2PalmRArray);
  
  lFS.getline(lLine,gBufSize); // comment
  lFS.getline(lLine,gBufSize); // joints
  std::stringstream lJointsSS(lLine);
  double *lJointValues=new double[51];
  std::copy(std::istream_iterator<double>(lJointsSS),
            std::istream_iterator<double>(),
            lJointValues);
  for(int i=0;i<17;++i)
    for(int a=0;a<3;++a)
      pSkeleton[i]->SetJointValue(gJointTypes[a],lJointValues[i*3+a]);
  delete []lJointValues;
  
  lFS.getline(lLine,gBufSize); // comment
  lFS.getline(lLine,gBufSize); // positions, not required for loading
  lFS.getline(lLine,gBufSize); // comment
  lFS.getline(lLine,gBufSize); // hand orientation
  buola::CQuaternion lHandQ;
  std::stringstream lHandQSS(lLine);
  lHandQSS >> lHandQ;
  lFS.getline(lLine,gBufSize); // comment
  lFS.getline(lLine,gBufSize); // hand translation
  buola::C3DVector lHandT;
  std::stringstream lHandTSS(lLine);
  lHandTSS >> lHandT;
  pHandTransf->SetRotation(lHandQ);
  pHandTransf->SetTranslation(lHandT);
  
  lFS.getline(lLine,gBufSize); // comment
  lFS.getline(lLine,gBufSize); // obj orientation
  buola::CQuaternion lObjQ;
  std::stringstream lObjQSS(lLine);
  lObjQSS >> lObjQ;
  lFS.getline(lLine,gBufSize); // comment
  lFS.getline(lLine,gBufSize); // obj translation
  buola::C3DVector lObjT;
  std::stringstream lObjTSS(lLine);
  lObjTSS >> lObjT;
  pObjTransf->SetRotation(lObjQ);
  pObjTransf->SetTranslation(lObjT);
  
  lFS.getline(lLine,gBufSize); // comment
  lFS.getline(lLine,gBufSize); // obj Transform
  pObjectPath = std::string(lLine);
}

#endif
