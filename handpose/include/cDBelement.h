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
#ifndef __CDBELEMENT_H
#define __CDBELEMENT_H

#include <string>
#include <sstream>
#include <iterator>
#include <fstream>
#include "typeDefinitions.h"
#include "cDBregisterQuery.h"
#include "cDBregisterInsert.h"
#include "boost/filesystem.hpp"
#include <buola/utility/usignal.h>
#include <buola/geometry.h>

#define OLDDB

#ifdef OLDDB
static char sPragmas[]=
"PRAGMA synchronous=0;\n"
"PRAGMA count_changed=0;\n"
"PRAGMA cache_size=5000;\n"
"PRAGMA auto_vacuum=0;\n"
"PRAGMA temp_store=MEMORY;\n"
"PRAGMA legacy_file_format=OFF;\n"
"PRAGMA encoding=\"UTF-8\";";
static char sGetInfo[]="SELECT * FROM hands WHERE image_id=?";
static char sCreate[]="create table hands (image_id integer primary key,orientation text, joints text, grasp_phase integer ,grasp_type integer ,grasp_illumination integer,parts text,image_path text, next_indexes text)";
static char sWrite[]="insert into hands values (?,?,?,?,?,?,?,?,?)";
static const int NJOINTS          = 25; // 28-3; first 3 joints were removed since  they are redundant
static const int NORI             = 9;
static const int NPARTS           = 17; // 17*3 fingertips and fingerbases
typedef Eigen::Matrix<double,1,NJOINTS+NORI> tPoseV;
typedef Eigen::Matrix<double,1,NPARTS*3> tPartsV;


class CDBelement
{
public:
  CDBelement(const CDBregisterQuery &pRegister):
    mOri(pRegister.getText(1)),
    mJoints(pRegister.getText(2)),
    mPartsLocation(pRegister.getText(6)),
    mNextIndices(pRegister.getText(8)),
    mGraspPhase(pRegister.getInt(3)),
    mGraspCode(pRegister.getInt(4)),
    mImagePath(pRegister.getText(7)){}
  CDBelement(const std::string &pOri,const std::string &pJoints,const std::string &pPartLocations,
             const std::string &pNextIndices,const unsigned &pIndex,const unsigned pGraspPhase,
             const unsigned pGraspCode,const std::string &pImagePath):
             mOri(pOri),mJoints(pJoints),mPartsLocation(pPartLocations),
             mNextIndices(pNextIndices),mIndex(pIndex),mGraspPhase(pGraspPhase),
             mGraspCode(pGraspCode),mImagePath(pImagePath){}
  CDBelement(const fsystem::path &pInfoPath, const unsigned &pIndex):
  mIndex(pIndex)
  {
    fsystem::path lTmpPath(pInfoPath);
    mImagePath=(lTmpPath.replace_extension(".png").string());
    std::string lStrTmp;
    std::ifstream lInfoS(pInfoPath.string().c_str(),std::ifstream::in);
    std::getline(lInfoS,lStrTmp); // comment
    std::getline(lInfoS,mOri);
    std::getline(lInfoS,lStrTmp); // comment
    std::getline(lInfoS,mJoints);
    std::getline(lInfoS,lStrTmp); // comment
    lInfoS >> mGraspPhase;
    std::getline(lInfoS,lStrTmp); // rest of line grasp phase
    std::getline(lInfoS,lStrTmp); // comment
    lInfoS >> mGraspCode;
    std::getline(lInfoS,lStrTmp); // rest of line grasp code
    std::getline(lInfoS,lStrTmp); // comment
    std::getline(lInfoS,mPartsLocation);
    std::getline(lInfoS,lStrTmp); // comment
    std::getline(lInfoS,mNextIndices);
    lInfoS.close();
  }
  void insert(const CDBregisterInsert &lRegister) const
  {
    lRegister.bindInt(mIndex,1);
    lRegister.bindText(mOri,2);
    lRegister.bindText(mJoints,3);
    lRegister.bindInt(mGraspPhase,4);
    lRegister.bindInt(mGraspCode,5);
    lRegister.bindInt(0,6); // Pointless right now
    lRegister.bindText(mPartsLocation,7);
    lRegister.bindText(mImagePath,8);
    lRegister.bindText(mNextIndices,9);
    lRegister.stepReset();
  }
  const std::string& getOri() const{return mOri;}
  const std::string& getJoints() const{return mJoints;}
  tPartsV getPartsLocationV() const{
    tPartsV lParts;
    double lTmpD;
    std::istringstream lPartsS(mPartsLocation); 
    for(int i=0;i<NPARTS;++i)
    {
      lPartsS>>lTmpD;
      lParts[i]=lTmpD;
    }
    return lParts;
  }
  tPoseV getPose() const{
    tPoseV lPose;
    double lTmpD;
    std::istringstream lOriS(mOri); 
    for(int i=0;i<NORI;++i)
    {
      lOriS>>lTmpD;
      lPose[i]=lTmpD;
    }
    std::istringstream lJointsS(mJoints); 
    for(int i=NORI;i<NORI+NJOINTS;++i)
    {
      lJointsS>>lTmpD;
      lPose[i]=lTmpD;
    }
    return lPose;
  }
  const std::string& getPartsLocation() const{return mPartsLocation;}
  const std::string& getNextIndices() const{return mNextIndices;}
  std::vector<unsigned> getNextIndicesV()
  {
    std::vector<unsigned> lNextIndices;
    std::istringstream lNextIndicesS(mNextIndices); 
    std::copy(std::istream_iterator<double>(lNextIndicesS),
              std::istream_iterator<double>(),
              std::back_inserter(lNextIndices));
    return lNextIndices;
  }
  const unsigned& getIndex() const{return mIndex;}
  const unsigned& getGraspPhase() const{return mGraspPhase;}
  const unsigned& getGraspCode() const{return mGraspCode;}
  const std::string& getImagePath() const{return mImagePath;}
private:
  std::string mOri;
  std::string mJoints;
  std::string mPartsLocation;
  std::string mNextIndices;
  unsigned mIndex;
  unsigned mGraspPhase;
  unsigned mGraspCode;
  std::string mImagePath;
};

#else
enum EENTRIES
{
  INDEX,
  ORIJOINTS,
  PARTSLOCATION,
  IMAGEPATH,
  HANDORI,
  HANDPOS,
  OBJORI,
  OBJPOS,
  OBJPATH,
  CAMAT,
  CAMFROM,
  CAMUP,
  NEXTINDICES,
  FEATURE
};
static char sPragmas[]=
"PRAGMA synchronous=0;\n"
"PRAGMA count_changed=0;\n"
"PRAGMA cache_size=5000;\n"
"PRAGMA auto_vacuum=0;\n"
"PRAGMA temp_store=MEMORY;\n"
"PRAGMA legacy_file_format=OFF;\n"
"PRAGMA encoding=\"UTF-8\";";
static char sGetInfo[]="SELECT * FROM hands WHERE image_id=?";
static char sCreate[]="create table hands (image_id integer primary key,orijoints text,parts text,"
                      "image_path text,hand_ori text,hand_pos text,obj_ori text,obj_pos text,"
                      "obj_path text,camAt text,camFrom text,camUp text,nextIndices text, feature blob)";
static char sWrite[]="insert into hands values (?,?,?,?,?,?,?,?,?,?,?,?,?,?)";

static const int NJOINTS          = 25; // 28-3; first 3 joints were removed since  they are redundant
static const int NORI             = 9;
static const int NPARTS           = 10;
static const int NFULLJOINTS      = ((3*5)+2)*3; // 17*3 fingertips and fingerbases
typedef Eigen::Matrix<double,1,NJOINTS+NORI> tPoseV;
typedef Eigen::Matrix<double,1,NFULLJOINTS> tFullPoseV;
typedef Eigen::Matrix<double,1,NPARTS*3> tPartsV;


class CDBelement
{
  friend std::ostream& operator << ( std::ostream& pOutput, const CDBelement& pElem)
  {
    pOutput << pElem.mOriJoints << std::endl;
    pOutput << pElem.mPartsLocation << std::endl;
    pOutput << pElem.mIndex << std::endl;
    pOutput << pElem.mImagePath << std::endl;
    pOutput << pElem.mHandOri << std::endl;
    pOutput << pElem.mHandPos << std::endl;
    pOutput << pElem.mObjOri << std::endl;
    pOutput << pElem.mObjPos << std::endl;
    pOutput << pElem.mObjPath << std::endl;
    pOutput << pElem.mCamAt<< std::endl;
    pOutput << pElem.mCamFrom<< std::endl;
    pOutput << pElem.mCamUp<< std::endl;
    pOutput << pElem.mNextIndices << std::endl;
    std::copy(pElem.mFeature.begin(),pElem.mFeature.end(),std::ostream_iterator<float>(pOutput," "));
    pOutput << std::endl;
    return pOutput;
  }
public:
  CDBelement(const CDBregisterQuery &pRegister):
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
      const float *lFeatureA=reinterpret_cast<const float*>(pRegister.getBlob(FEATURE)); // gets deallocated by sqlite3
      std::copy(lFeatureA,lFeatureA+(pRegister.getSize(FEATURE)/sizeof(float)),std::back_inserter(mFeature));
    }
    
  CDBelement(const std::string &pOri,const std::string &pJoints,const std::string &pPartLocations,const unsigned &pIndex,
             const std::string &pImagePath,const std::string &pHandOri, const std::string &pHandPos,const std::string &pObjOri,
             const std::string &pObjPos,const std::string &pObjPath, const std::string &pCamAt, const std::string &pCamFrom,
             const std::string &pCamUp,const std::string &pNextIndices,const std::vector<float> &pFeature):
             mOriJoints(pOri+pJoints),mPartsLocation(pPartLocations),mIndex(pIndex),mImagePath(pImagePath),mHandOri(pHandOri),
             mHandPos(pHandPos),mObjOri(pObjOri),mObjPos(pObjPos),mObjPath(pObjPath),mCamAt(pCamAt),mCamFrom(pCamFrom),mCamUp(pCamUp),
             mNextIndices(pNextIndices),mFeature(pFeature){}
  CDBelement(const fsystem::path &pInfoPath, const unsigned &pIndex):
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
    std::getline(lInfoS,mObjPath);
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
  void insert(const CDBregisterInsert &lRegister) const
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
    float *lFeatureA=new float[mFeature.size()]; // how will this behave with 0-sized vectors?
    std::copy(mFeature.begin(),mFeature.end(),lFeatureA);
    lRegister.bindBlob(reinterpret_cast<const void*>(lFeatureA),mFeature.size()*sizeof(float),FEATURE+1);
    lRegister.stepReset();
    delete []lFeatureA;
  }
  std::vector<unsigned> getNextIndicesV() const{
    std::vector<unsigned> lNextIndices;
    std::istringstream lNextIndicesS(mNextIndices); 
    std::copy(std::istream_iterator<unsigned>(lNextIndicesS),
              std::istream_iterator<unsigned>(),
              back_inserter(lNextIndices));
    return lNextIndices;
  }
  tPartsV getPartsLocationV() const{
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
  tPoseV getPose() const{
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
  void setOri(const std::string &pOri){
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
  void setFullPose(tFullPoseV &pFullPoseV){
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
  void getFullPose(tFullPoseV &pFullPoseV) const{
    double lTmp;
    std::istringstream lRestSS(mOriJoints);
    for(int i=0;i<NORI;++i)
      lRestSS >> lTmp;
    for(int i=0;i<NFULLJOINTS;++i)
      lRestSS >> pFullPoseV[i];
  }
  void setPartsLocation(const std::string &pPartsLocation){mPartsLocation=pPartsLocation;}
  const std::string& getPartsLocation() const{return mPartsLocation;}
  const unsigned& getIndex() const{return mIndex;}
  const std::string& getImagePath() const{return mImagePath;}
  const std::string& getOriJoints() const{return mOriJoints;}
  const std::string& getHandOri() const{return mHandOri;}
  const std::string& getHandPos() const{return mHandPos;}
  const std::string& getObjOri() const{return mObjOri;}
  const std::string& getObjPos() const{return mObjPos;}
  const std::string& getObjPath() const{return mObjPath;}
  const std::string& getCamAt() const{return mCamAt;}
  const std::string& getCamFrom() const{return mCamFrom;}
  const std::string& getCamUp() const{return mCamUp;}
  void setCamAtFromUp(const buola::C3DVector& pAt,const buola::C3DVector& pFrom,const buola::C3DVector& pUp)
  {
    std::ostringstream lAtS;
    lAtS << pAt;
    mCamAt=lAtS.str();
    std::ostringstream lFromS;
    lFromS << pFrom;
    mCamFrom=lFromS.str();
    std::ostringstream lUpS;
    lUpS << pUp;
    mCamUp=lUpS.str();
  }
  const std::vector<float>& getFeature() const{return mFeature;}
  void setFeature(const std::vector<float> &pFeature){mFeature=pFeature;}
  void setIndex(const unsigned pIndex){mIndex=pIndex;}
  void setImagePath(const std::string pImagePath){mImagePath=pImagePath;}
private:
  std::string mOriJoints;
  std::string mPartsLocation;
  unsigned mIndex;
  std::string mImagePath;
  std::string mHandOri;
  std::string mHandPos;
  std::string mObjOri;
  std::string mObjPos;
  std::string mObjPath;
  std::string mCamAt;
  std::string mCamFrom;
  std::string mCamUp;
  std::string mNextIndices;
  std::vector<float> mFeature;
};

#endif // OLDDB

#endif // __CDBELEMENT_H