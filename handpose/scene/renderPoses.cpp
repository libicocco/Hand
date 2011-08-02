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

#include <buola/app/ccmdline.h>

#include "handRenderer.h"
#include "cDB.h"
#include "cDBelement.h"
#include "handclass_config.h"

static buola::CCmdLineOption<std::string> gOutFolderPathOption("outfolder",'o',L"Path to where the images and db will be saved","/tmp/images");

int main(int argc,char* argv[])
{
  buola_init(argc,argv);

  fsystem::path lHandObjPath(SCENEPATH);
  lHandObjPath/="rHandP3.obj";
  fsystem::path lHandTexturePath(SCENEPATH);
  lHandTexturePath/="hand_texture.ppm";
  fsystem::path lOutFolderPath(buola::cmd_line().GetValue(gOutFolderPathOption));
  fsystem::create_directory(lOutFolderPath);
  //fsystem::path lDBPath(lOutFolderPath);
  //lDBPath/="poses.db";

  HandRenderer lHRender(lHandObjPath.string().c_str(),lHandTexturePath.string().c_str());
  
  // Given a db, rerender it; not interesting
  fsystem::path lDBPath(SCENEPATH);
  lDBPath/="hands.db";
  CDB lDB(lDBPath);

  for(int p=0;p<100;++p)
  {
    CDBelement lDBelem = lDB.query(p);
    lHRender.render(lDBelem);
  }

  /*
  // create a cDBelement from input
  std::istream_iterator<double> lIt(std::cin);
  unsigned lIndex=0
  while(true)
  {
    fsystem::path lImPath(lOutFolderPath);

    std::stringstream lImFilename;
    lImFilename << std::setfill('0');
    lImFilename << std::setw(3) << lIndex;
    lImPath/=lImFilename.str();

    std::ostringstream lJointsSS,lCamFrom,lCamUp;
    for(int i=0;i<51;++i)
      lJointsSS << *(lIt++);
    lIndex++;
  }
  CDBelement("1 0 0 0 1 0 0 0 1",lJointsSS.str(),"",lIndex,
             lImPath.str(),pHandOri,pHandPos,pObjOri,
             pObjPos,pObjPath, pCamAt, pCamFrom,
             pCamUp,pNextIndices,pFeature):
  // render needs FullPose (last 51) ,HandPos,HandOri,ObjPath,ObjPos,ObjOri,Cam{At,From,Up},imagePath
  */

  return 1;
}
