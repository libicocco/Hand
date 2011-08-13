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
// I have to include hog first
// because X defines collide with them
#include <buola/image/algorithm/detail/opencv.h>
#include "hog.h" // from handclass

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

#include <buola/app/ccmdline.h>

#include <buola/geometry/clookatmatrix.h>
#include <buola/geometry/c3dvector.h>
#include <buola/geometry/cperspectiveprojectionmatrix.h>

#include "csavebutton.h"
#include "CHandSkeleton.h"
#include "loadPose.h"

#include "cDB.h"
#include "cDBelement.h"
#include "handRenderer.h"

#include "handclass_config.h"

static const buola::C3DVector lHandZero(0.31,-0.57,0.02);
static const buola::C3DVector lHandRest(0.31,-0.5,0.02);
static const buola::C3DVector lObjZero(0,0,0);
static const unsigned gNumGrasps(31);
static const tFullPoseV lRestPose = tFullPoseV::Zero();

using namespace buola;
static buola::CCmdLineOption<std::string> gOutPathOption("outpath",'f',L"Path to save the db and images","/tmp/out/");
static buola::CCmdLineOption<unsigned> gNumViewsOption("nv",'v',L"Number of views to be render of each grasp step",10);
static buola::CCmdLineOption<unsigned> gNStepsOption("ns",'s',L"Number of steps rendered for each grasp",5);

void getRandomViews(const unsigned gNumViews,std::pair<C3DVector,C3DVector> *pFromUp)
{
  // engine for generating real numbers between -1 and 1
  std::mt19937 lEngine(time(0));
  std::uniform_real_distribution<> lDist(-1,1);

  // generate a vector of random camera origins(first) on the unit sphere,
  // with random camera orientation (second)
  for(int i=0;i<gNumViews;)
  {
    // generate a 3D point in a cube [-1,1]
    pFromUp[i].first.Set(lDist(lEngine),lDist(lEngine),lDist(lEngine));
    // if it inside the sphere centered in the origin with radious 1 and not in the origin
    if(pFromUp[i].first.Modulus2()<=1 && pFromUp[i].first.Modulus2()!=0)
    {
      // strictly speaking lRand should be non-zero and not parallel to pFromUp[i].first
      C3DVector lRand(lDist(lEngine),lDist(lEngine),lDist(lEngine));
      //       pFromUp[i].first=normalize(pFromUp[i].first);
      pFromUp[i].first=pFromUp[i].first*(gCamDistance/pFromUp[i].first.Modulus());
      pFromUp[i].second=cross_product(pFromUp[i].first,lRand);
      pFromUp[i].second=normalize(pFromUp[i].second);
      ++i;
    }
  }
}

int main(int pNArg,char **pArgs)
{
  buola_init(pNArg,pArgs);
  static const unsigned gNumViews(cmd_line().GetValue(gNumViewsOption));
  static const unsigned gNSteps(cmd_line().GetValue(gNStepsOption));

  fsystem::path lObjectPathFS(SCENEPATH);
  lObjectPathFS/="objects/adductedThumb_onlyObject.obj";
  std::string lObjectPath = lObjectPathFS.string();

  fsystem::path lHandObjPathFS(SCENEPATH);
  lHandObjPathFS/="rHandP3.obj";
  fsystem::path lTexturePathFS(SCENEPATH);
  lTexturePathFS/="hand_texture.ppm";

  std::pair<C3DVector,C3DVector> *lXYZ=new std::pair<C3DVector,C3DVector>[gNumViews];
  getRandomViews(gNumViews,lXYZ);

  try
  {
    std::cout << lHandObjPathFS.string() << "," << lObjectPathFS.string() << std::endl;
    HandRenderer lHR(lHandObjPathFS.string().c_str(),lTexturePathFS.string().c_str());

    fsystem::path lDBPath(cmd_line().GetValue(gOutPathOption));
    fsystem::create_directory(lDBPath);
    lDBPath/="hands.db";
    CDB *lDB=new CDB(lDBPath.string());

    fsystem::path lHogPath(cmd_line().GetValue(gOutPathOption));
    lHogPath/="hog.bin";
    std::ofstream lHogOFS;
    lHogOFS.open(lHogPath.string().c_str());

    // previously generated database with the basic poses
    fsystem::path lBasicRenderDbFS(SCENEPATH);
    lBasicRenderDbFS/="taxonomy.db";
    CDB lDBtaxonomy(lBasicRenderDbFS);


    // each grasp has the same basic pose coming from lDBtaxonomy
    for(int p=0;p<gNumGrasps;++p)
    {
      // get the basic pose
      CDBelement lGraspElem =lDBtaxonomy.query(p);

      scene::PRTTransform lOrigHandTransf=new scene::CRTTransform;
      setTransf(lGraspElem.getHandPos(),lGraspElem.getHandOri(),lOrigHandTransf);

      fsystem::path lGraspOutPath(cmd_line().GetValue(gOutPathOption));
      std::ostringstream lGraspFolderOSS;
      lGraspFolderOSS << std::setfill('0') << std::setw(3) << p;
      lGraspOutPath/=lGraspFolderOSS.str();
      fsystem::create_directory(lGraspOutPath.string());

      // each step has the same actual interpolated pose
      for(int f=0;f<gNSteps;++f)
      {
        CDBelement lTmpElem = lGraspElem;

        tFullPoseV lFullPose;
        lTmpElem.getFullPose(lFullPose);

        tFullPoseV lFullPoseInterp=((gNSteps-(f+1))*lRestPose+(f+1)*lFullPose)/gNSteps;
        lTmpElem.setFullPose(lFullPoseInterp);
        
        // interpolate rest translation and basic translation
        scene::PRTTransform lHandTransf=new scene::CRTTransform;
        lHandTransf->SetTranslation(((gNSteps-(f+1))*lHandRest+(f+1)*lOrigHandTransf->GetTranslation())/gNSteps);
        std::ostringstream lHandPosSS;
        lHandPosSS << lHandTransf->GetTranslation();
        lTmpElem.setHandPos(lHandPosSS.str());

        // each viewpoint defines completely the element
        for(int i=0;i<gNumViews;++i)
        {
          lTmpElem.setCamAtFromUp(0,0,0,
              lXYZ[i].first.x,lXYZ[i].first.y,lXYZ[i].first.z,
              lXYZ[i].second.x,lXYZ[i].second.y,lXYZ[i].second.z);

          std::ostringstream lPoseName;
          lPoseName << std::setfill('0');
          lPoseName << std::setw(3) << f << "_" << std::setw(3) << i << ".pgm";
          fsystem::path lPosePath(lGraspOutPath);
          lPosePath/=lPoseName.str();
          lTmpElem.setImagePath(lPosePath.string());
          lTmpElem.setIndex(p*gNSteps*gNumViews+f*gNumViews+i);
          lHR.render(lTmpElem);
          lHR.saveInfo(lTmpElem,&lHogOFS);
          lDB->insertElement(lTmpElem);
        }
      }
    }
    //lDB->finalizeStatement(); // done anyway when CDB is deleted
    lHogOFS.close();

  }
  catch(std::exception &pE)
  {
    msg_info() << "caught exception " << pE.what() << "\n";
  }

  return buola_finish();
}
