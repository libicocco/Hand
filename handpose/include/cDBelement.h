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
#include <array>
#include <fstream>
#include "typeDefinitions.h"
#include "cDBregisterQuery.h"
#include "cDBregisterInsert.h"
#include "boost/filesystem.hpp"

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

static const int NJOINTS          = 25; // 28-3; first 3 joints were removed since  they are redundant
static const int NORI             = 9;
static const int NPARTS           = 10;
static const int NFULLJOINTS      = ((3*5)+2)*3; // 17*3 fingertips and fingerbases
typedef Eigen::Matrix<double,1,NJOINTS+NORI> tPoseV;
typedef Eigen::Matrix<double,1,NFULLJOINTS> tFullPoseV;
typedef Eigen::Matrix<double,1,NFULLJOINTS+NORI> tFullOriPoseV;
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
  CDBelement(const CDBregisterQuery &pRegister);
    
  CDBelement(const std::string &pOri,const std::string &pJoints,const std::string &pPartLocations,const unsigned &pIndex,
             const std::string &pImagePath,const std::string &pHandOri, const std::string &pHandPos,const std::string &pObjOri,
             const std::string &pObjPos,const std::string &pObjPath, const std::string &pCamAt, const std::string &pCamFrom,
             const std::string &pCamUp,const std::string &pNextIndices,const std::vector<float> &pFeature);

  // constructor for easy pose rendering
  CDBelement(const tFullOriPoseV &pJoints,const unsigned &pIndex,
      const std::string &pImagePath,const std::string &pObjPath);

  CDBelement(const fsystem::path &pInfoPath, const unsigned &pIndex);

  void insert(const CDBregisterInsert &lRegister) const;

  std::vector<unsigned> getNextIndicesV() const;

  tPartsV getPartsLocationV() const;

  tPoseV getPose() const;

  void setOri(const std::string &pOri);

  void setFullPose(tFullPoseV &pFullPoseV);

  void getFullPose(tFullPoseV &pFullPoseV) const;

  void getOriJoints(double *pCam2PalmRArray,double *pJoints) const;
  void setFeature(const std::vector<float> &pFeature);
  void setCamAtFromUp(double pAtx,double pAty,double pAtz,
                      double pFromx,double pFromy,double pFromz,
                      double pUpx,double pUpy,double pUpz);

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
  const std::vector<float>& getFeature() const{return mFeature;}

  void setIndex(const unsigned pIndex){mIndex=pIndex;}
  void setImagePath(const std::string pImagePath){mImagePath=pImagePath;}
  void setHandPos(const std::string pHandPos){mHandPos=pHandPos;}
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
  std::string mFeatureStr;
};

#endif // __CDBELEMENT_H
