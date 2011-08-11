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
static buola::CCmdLineOption<unsigned> gNPosesOption("nposes",'n',L"Number of poses to render");
static buola::CCmdLineOption<bool> gFullPoseOption("fullpose",'f',L"True if the pose is provided as 60 parameters, instead of 34",false);

// for old format, get the asin and put the thumb first (with bend negated)
void pose2fullpose(const tPoseV &pPose,tFullOriPoseV &pFullPose)
{
  pFullPose[0]=pPose[0];pFullPose[1]=pPose[1];pFullPose[2]=pPose[2];
  pFullPose[3]=pPose[3];pFullPose[4]=pPose[4];pFullPose[5]=pPose[5];
  pFullPose[6]=pPose[6];pFullPose[7]=pPose[7];pFullPose[8]=pPose[8];

  pFullPose[9]=0;pFullPose[10]=0;pFullPose[11]=0;
  pFullPose[12]=0;pFullPose[13]=0;pFullPose[14]=0;

  pFullPose[15]=pPose[9];pFullPose[16]=pPose[10];pFullPose[17]=pPose[11];
  pFullPose[18]=pPose[12];pFullPose[19]=0;pFullPose[20]=0;
  pFullPose[21]=pPose[13];pFullPose[22]=0;pFullPose[23]=0;

  pFullPose[24]=pPose[14];pFullPose[25]=pPose[15];pFullPose[26]=pPose[16];
  pFullPose[27]=pPose[17];pFullPose[28]=0;pFullPose[29]=0;
  pFullPose[30]=pPose[18];pFullPose[31]=0;pFullPose[32]=0;

  pFullPose[33]=pPose[19];pFullPose[34]=pPose[20];pFullPose[35]=pPose[21];
  pFullPose[36]=pPose[22];pFullPose[37]=0;pFullPose[38]=0;
  pFullPose[39]=pPose[23];pFullPose[40]=0;pFullPose[41]=0;

  pFullPose[42]=pPose[24];pFullPose[43]=pPose[25];pFullPose[44]=pPose[26];
  pFullPose[45]=pPose[27];pFullPose[46]=0;pFullPose[47]=0;
  pFullPose[48]=pPose[28];pFullPose[49]=0;pFullPose[50]=0;

  pFullPose[51]=pPose[29];pFullPose[52]=pPose[30];pFullPose[53]=pPose[31];
  pFullPose[54]=pPose[32];pFullPose[55]=0;pFullPose[56]=0;
  pFullPose[57]=pPose[33];pFullPose[58]=0;pFullPose[59]=0;
}

int main(int argc,char* argv[])
{
  buola_init(argc,argv);

  fsystem::path lHandObjPath(SCENEPATH);
  lHandObjPath/="rHandP3.obj";
  fsystem::path lHandTexturePath(SCENEPATH);
  lHandTexturePath/="hand_texture.ppm";
  fsystem::path lOutFolderPath(buola::cmd_line().GetValue(gOutFolderPathOption));
  const unsigned gNPoses(buola::cmd_line().GetValue(gNPosesOption));
  const bool gFullPose(buola::cmd_line().GetValue(gFullPoseOption));
  fsystem::create_directory(lOutFolderPath);

  HandRenderer lHRender(lHandObjPath.string().c_str(),lHandTexturePath.string().c_str());
  
  // Given a db, rerender it; not interesting
  fsystem::path lDBPath(lOutFolderPath);
  lDBPath/="hands.db";
  CDB lDB(lDBPath);

  /*
  for(int p=0;p<100;++p)
  {
    std::cout << p << std::endl;
    CDBelement lDBelem = lDB.query(p);
    lHRender.render(lDBelem);
  }
  */

  // create a cDBelement from input
  std::istream_iterator<double> lIt(std::cin);
  tPoseV lJoints;
  tFullOriPoseV lFJoints;
  fsystem::path lImPath(lOutFolderPath);
  for(unsigned i=0;i<gNPoses;++i)
  {
    lImPath = fsystem::path(lOutFolderPath);

    std::stringstream lImFilename;
    lImFilename << std::setfill('0');
    lImFilename << std::setw(3) << i << ".pgm";
    lImPath/=lImFilename.str();

    if(gFullPose)
    {
      for(int i=0;i<60;++i)
        lFJoints[i] = *(lIt++);
    }
    else
    {
      for(int i=0;i<34;++i)
        lJoints[i] = *(lIt++);
      pose2fullpose(lJoints,lFJoints);
    }

    CDBelement lDBelem(lFJoints,i,lImPath.string(),"");
    lHRender.render(lDBelem);
    lHRender.saveInfo(lDBelem);
    lDB.insertElement(lDBelem);
  }
  lDB.finalizeStatement();
  return 1;
}
