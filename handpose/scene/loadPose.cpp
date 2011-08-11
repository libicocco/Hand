#include "loadPose.h"

void setTransf(const std::string &pPos,const std::string &pOri,scene::PRTTransform &pTransf)
{
  std::stringstream lVectorSS(pOri);
  buola::CQuaternion lQ;
  lVectorSS >> lQ;
  lVectorSS.clear();
  lVectorSS.str(pPos);
  buola::C3DVector lT;
  lVectorSS >> lT;
  pTransf->SetRotation(lQ);
  pTransf->SetTranslation(lT);
}

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
            pSkeleton[f*3+a+2]->SetJointValue(gJointTypes[j],
                asin(stod(lJointsV[lFingerEquivalence[f]*5+j+9])));
          else if(a==1)
            pSkeleton[f*3+a+2]->SetJointValue(gJointTypes[j],
                asin(stod(lJointsV[lFingerEquivalence[f]*5+3+9])));
          else if(a==2)
            pSkeleton[f*3+a+2]->SetJointValue(gJointTypes[j],
                asin(stod(lJointsV[lFingerEquivalence[f]*5+4+9])));
        }
  }
}

void loadPose(const fsystem::path &pPosePath,
    CHandSkeleton &pSkeleton,
    scene::PRTTransform &pHandTransf,
    scene::PRTTransform &pObjTransf,
    std::string &pObjectPath,
    double *pCam2PalmRArray)
{
  std::ifstream lFS(pPosePath.string().c_str());
  char lLine[gBufSize];
  char lLine2[gBufSize];
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
  lFS.getline(lLine2,gBufSize); // comment
  lFS.getline(lLine2,gBufSize); // hand translation
  setTransf(lLine2,lLine,pHandTransf);

  lFS.getline(lLine,gBufSize); // comment
  lFS.getline(lLine,gBufSize); // obj orientation
  lFS.getline(lLine2,gBufSize); // comment
  lFS.getline(lLine2,gBufSize); // obj translation
  setTransf(lLine2,lLine,pObjTransf);

  lFS.getline(lLine,gBufSize); // comment
  lFS.getline(lLine,gBufSize); // obj Transform
  pObjectPath = std::string(lLine);
  fsystem::path lObjectPath(pPosePath);
  lObjectPath.remove_filename();
  lObjectPath/=pObjectPath;
  pObjectPath=lObjectPath.string();
}
