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

#include "cDBelement.h"

CDBelement::CDBelement(const CDBregisterQuery &pRegister):
  mOriJoints(pRegister.getText(ORIJOINTS)),
  mPartsLocation(pRegister.getText(PARTSLOCATION)),
  mIndex(pRegister.getIndex()),
  mImagePath(pRegister.getText(IMAGEPATH)),
  mHandOri(pRegister.getText(HANDORI)),
  mHandPos(pRegister.getText(HANDPOS)),
  mObjOri(pRegister.getText(OBJORI)),
  mObjPos(pRegister.getText(OBJPOS)),
  mObjPath(pRegister.getText(OBJPATH)),
  mCamAt(pRegister.getText(CAMAT)),
  mCamFrom(pRegister.getText(CAMFROM)),
  mCamUp(pRegister.getText(CAMUP)),
  mNextIndices(pRegister.getText(NEXTINDICES))
{
  mFeatureStr = pRegister.getText(FEATURE);
  std::istringstream lFeatureISS(mFeatureStr);
  std::copy(std::istream_iterator<float>(lFeatureISS),
      std::istream_iterator<float>(),
      std::back_inserter(mFeature));
  /*
     const float *lFeatureA=reinterpret_cast<const float*>(pRegister.getBlob(FEATURE)); // gets deallocated by sqlite3
     std::copy(lFeatureA,lFeatureA+(pRegister.getSize(FEATURE)/sizeof(float)),std::back_inserter(mFeature));
     */
}
CDBelement::CDBelement(const std::string &pOri,const std::string &pJoints,const std::string &pPartLocations,const unsigned &pIndex,
    const std::string &pImagePath,const std::string &pHandOri, const std::string &pHandPos,const std::string &pObjOri,
    const std::string &pObjPos,const std::string &pObjPath, const std::string &pCamAt, const std::string &pCamFrom,
    const std::string &pCamUp,const std::string &pNextIndices,const std::vector<float> &pFeature):
  mOriJoints(pOri+pJoints),mPartsLocation(pPartLocations),mIndex(pIndex),mImagePath(pImagePath),mHandOri(pHandOri),
  mHandPos(pHandPos),mObjOri(pObjOri),mObjPos(pObjPos),mObjPath(pObjPath),mCamAt(pCamAt),mCamFrom(pCamFrom),mCamUp(pCamUp),
  mNextIndices(pNextIndices),mFeature(pFeature){}

  CDBelement::CDBelement(const tFullOriPoseV &pJoints,const unsigned &pIndex,
      const std::string &pImagePath,const std::string &pObjPath):
    mPartsLocation(""),mIndex(pIndex),mImagePath(pImagePath),mHandOri("(1 0 0 0)"),
    mHandPos("(0.31,-0.57,0.02)"),mObjOri("(1 0 0 0)"),mObjPos("(0,0,0)"),
    mObjPath(pObjPath),mCamAt("(0,0,0)"),mCamFrom("(0,0,0)"),mCamUp("(0,0,0)"),
    mNextIndices("")
{
  std::ostringstream lOSS;
  for(int i=0;i<60;++i)
    lOSS << pJoints[i] << " ";

  mOriJoints = lOSS.str();
}

CDBelement::CDBelement(const fsystem::path &pInfoPath, const unsigned &pIndex):
  mIndex(pIndex),mFeature(std::vector<float>(0))
  {
    fsystem::path lTmpPath(pInfoPath);
    mImagePath=(lTmpPath.replace_extension(".png").string());
    std::string lStrTmp,lOri,lJoints;
    std::ifstream lInfoS(pInfoPath.string().c_str(),std::ifstream::in);
    std::getline(lInfoS,lStrTmp); // comment
    std::getline(lInfoS,lOri);
    std::getline(lInfoS,lStrTmp); // comment
    std::getline(lInfoS,lJoints);
    mOriJoints=lOri+lJoints;
    std::getline(lInfoS,lStrTmp); // comment
    std::getline(lInfoS,mPartsLocation);
    std::getline(lInfoS,lStrTmp); // comment
    std::getline(lInfoS,mHandOri);
    std::getline(lInfoS,lStrTmp); // comment
    std::getline(lInfoS,mHandPos);
    std::getline(lInfoS,lStrTmp); // comment
    std::getline(lInfoS,mObjOri);
    std::getline(lInfoS,lStrTmp); // comment
    std::getline(lInfoS,mObjPos);
    std::getline(lInfoS,lStrTmp); // comment

    std::getline(lInfoS,mObjPath); // relative path, we should build the full path
    fsystem::path lObjPath = pInfoPath;
    lObjPath.remove_filename();
    lObjPath/=mObjPath;
    mObjPath = lObjPath.string();

    mCamAt="0 0 0";
    mCamFrom="0 -1 0";
    mCamUp="0 0 1";
    mNextIndices="1 2 3 4 5";
//     std::getline(lInfoS,lStrTmp); // comment
//     std::getline(lInfoS,mCamAtFromUp);
//     std::getline(lInfoS,lStrTmp); // comment
//     std::getline(lInfoS,mNextIndices);
//     std::getline(lInfoS,lStrTmp); // comment
//     std::string lFeatureStr;
//     std::getline(lInfoS,lFeatureStr);
//     std::istringstream lFeatureSS(lFeatureStr);
//     std::copy(std::istream_iterator<float>(lFeatureSS),std::istream_iterator<float>(),std::back_inserter<float>(mFeature));
    lInfoS.close(); // the rest is useless right now
  }

void CDBelement::insert(const CDBregisterInsert &lRegister) const
{
  lRegister.bindInt(mIndex,INDEX+1);
  lRegister.bindText(mOriJoints,ORIJOINTS+1);
  lRegister.bindText(mPartsLocation,PARTSLOCATION+1);
  lRegister.bindText(mImagePath,IMAGEPATH+1);
  lRegister.bindText(mHandOri,HANDORI+1);
  lRegister.bindText(mHandPos,HANDPOS+1);
  lRegister.bindText(mObjOri,OBJORI+1);
  lRegister.bindText(mObjPos,OBJPOS+1);
  lRegister.bindText(mObjPath,OBJPATH+1);
  lRegister.bindText(mCamAt,CAMAT+1);
  lRegister.bindText(mCamFrom,CAMFROM+1);
  lRegister.bindText(mCamUp,CAMUP+1);
  lRegister.bindText(mNextIndices,NEXTINDICES+1);
  /*
     float *lFeatureA=new float[mFeature.size()]; // how will this behave with 0-sized vectors?
     std::copy(mFeature.begin(),mFeature.end(),lFeatureA);
     lRegister.bindBlob(reinterpret_cast<const void*>(lFeatureA),mFeature.size()*sizeof(float),FEATURE+1);
     */
  lRegister.bindText(mFeatureStr,FEATURE+1);
  lRegister.stepReset();
  //delete []lFeatureA;
}

std::vector<unsigned> CDBelement::getNextIndicesV() const
{
  std::vector<unsigned> lNextIndices;
  std::istringstream lNextIndicesS(mNextIndices); 
  std::copy(std::istream_iterator<unsigned>(lNextIndicesS),
      std::istream_iterator<unsigned>(),
      back_inserter(lNextIndices));
  return lNextIndices;
}

tPartsV CDBelement::getPartsLocationV() const
{
  tPartsV lParts;
  double lTmpD;
  std::istringstream lPartsS(mPartsLocation); 
  for(int i=0;i<NPARTS*3;++i)
  {
    lPartsS>>lTmpD;
    lParts[i]=lTmpD;
  }
  return lParts;
}

tPoseV CDBelement::getPose() const
{
  tPoseV lPose;
  double lTmpD;
  std::istringstream lOriS(mOriJoints); 
  for(int i=0;i<NORI;++i)
  {
    lOriS>>lTmpD;
    lPose[i]=lTmpD;
  }
  //     lOriS.seekg(lOriS.tellg()+static_cast<long>(2*3)); //DOESN'T WORK!
  for(int i=0;i<2*3;++i)
    lOriS>>lTmpD;
  // j={0,1} are forearm and hand
  for(int f=0;f<5;++f)
  {
    for(int jthumb=0;jthumb<3;++jthumb)
    {
      lOriS>>lTmpD;
      lPose[NORI+f*5+jthumb]=lTmpD;
    }
    for(int j=0;j<2;++j)
    {
      lOriS>>lTmpD;
      lPose[NORI+f*5+3+j]=lTmpD;
      //         lOriS.seekg(lOriS.tellg()+static_cast<long>(2)); //DOESN'T WORK!
      lOriS>>lTmpD;lOriS>>lTmpD; // skip side-twist
    }
  }
  return lPose;
}

void CDBelement::setOri(const std::string &pOri)
{
  std::ostringstream lNewSS;
  std::istringstream lOldSS(mOriJoints);
  std::istringstream lOriSS(pOri);
  double lTmp;
  for(int i=0;i<NORI;++i)
  {
    lOriSS >> lTmp;
    lNewSS << lTmp << " ";
    lOldSS >> lTmp; // just advancing the pointer
  }
  for(int i=0;i<NFULLJOINTS;++i)
  {
    lOldSS >> lTmp;
    lNewSS << lTmp << " ";
  }
  mOriJoints=lNewSS.str();
}

void CDBelement::setFullPose(tFullPoseV &pFullPoseV)
{
  std::ostringstream lNewSS;
  std::istringstream lOldSS(mOriJoints);
  double lTmp;
  for(int i=0;i<NORI;++i)
  {
    lOldSS >> lTmp;
    lNewSS << lTmp << " ";
  }
  for(int i=0;i<NFULLJOINTS;++i)
    lNewSS << pFullPoseV[i] << " ";
  mOriJoints=lNewSS.str();
}

void CDBelement::getFullPose(tFullPoseV &pFullPoseV) const
{
  double lTmp;
  std::istringstream lRestSS(mOriJoints);
  for(int i=0;i<NORI;++i)
    lRestSS >> lTmp;
  for(int i=0;i<NFULLJOINTS;++i)
    lRestSS >> pFullPoseV[i];
}

void CDBelement::getOriJoints(double *pCam2PalmRArray,double *pJoints) const
{
  std::stringstream lOriSS(mOriJoints);
  for(int i=0;i<9;++i)
    lOriSS >> pCam2PalmRArray[i];
  for(int i=0;i<51;++i)
    lOriSS >> pJoints[i];
}

void CDBelement::setCamAtFromUp(double pAtx,double pAty,double pAtz,
    double pFromx,double pFromy,double pFromz,
    double pUpx,double pUpy,double pUpz)
{
  std::ostringstream lAtS;
  lAtS << "(" << pAtx << "," << pAty << "," << pAtz << ")";
  mCamAt=lAtS.str();
  std::ostringstream lFromS;
  lFromS << "(" << pFromx << "," << pFromy << "," << pFromz << ")";
  mCamFrom=lFromS.str();
  std::ostringstream lUpS;
  lUpS << "(" << pUpx << "," << pUpy << "," << pUpz << ")";
  mCamUp=lUpS.str();
}
void CDBelement::setFeature(const std::vector<float> &pFeature)
{
  mFeature=pFeature;
  std::ostringstream lFeatureOSS;
  lFeatureOSS.precision(2);
  std::copy(mFeature.begin(),mFeature.end(),std::ostream_iterator<float>(lFeatureOSS," "));
  mFeatureStr = lFeatureOSS.str();
}
