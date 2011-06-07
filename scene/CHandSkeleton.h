#ifndef _CHANDSKELETON_H__
#define _CHANDSKELETON_H__
#include <buola/scene.h>
#include <buola/scene/csceneview.h>
#include <buola/scene/ccamera.h>
#include <buola/scene/cperspectivecamera.h>
#include <buola/scene/cscene.h>
#include <buola/scene/cmesh.h>
#include <buola/scene/cskeleton.h>
#include <buola/scene/cbone.h>
#include <buola/scene/cgeode.h>
#include <buola/scene/crttransform.h>

#include <buola/geometry/clookatmatrix.h>
#include <buola/geometry/c3dvector.h>
#include <buola/geometry/cperspectiveprojectionmatrix.h>

#include <boost/array.hpp>
enum EBone
{
    BONE_FOREARM,
    BONE_HAND,
    BONE_THUMB1,
    BONE_THUMB2,
    BONE_THUMB3,
    BONE_INDEX1,
    BONE_INDEX2,
    BONE_INDEX3,
    BONE_MIDDLE1,
    BONE_MIDDLE2,
    BONE_MIDDLE3,
    BONE_RING1,
    BONE_RING2,
    BONE_RING3,
    BONE_PINKY1,
    BONE_PINKY2,
    BONE_PINKY3,
    BONE_COUNT
};

static const buola::scene::EJointType gJointTypes[3] = {buola::scene::JOINTTYPE_BEND,buola::scene::JOINTTYPE_SIDE,buola::scene::JOINTTYPE_TWIST};
static const std::string gJointTypeNames[3] = {"bend","side","twist"};

class CHandSkeleton
{
    public:
        CHandSkeleton(const char* pObjPath,const char* pTexturePath):
            mSkeleton(new buola::scene::CSkeleton),
            mMesh(buola::scene::CMesh::Import(pObjPath))
    {
        using namespace buola;
        using namespace buola::scene;

        mBones[BONE_FOREARM]=new CBone("rForeArm",C3DVector(-0.276,0.58,-0.026),C3DVector(-0.295,0.578,-0.026),deg2rad(C3DRotation(0,0,0)));
        mBones[BONE_FOREARM]->AddJoint(JOINTTYPE_BEND,C3DVector(0,1,0),C3DVector(1,0,0));
        mBones[BONE_FOREARM]->AddJoint(JOINTTYPE_SIDE,C3DVector(0,0,1),C3DVector(1,0,0));
        mBones[BONE_FOREARM]->AddJoint(JOINTTYPE_TWIST,C3DVector(1,0,0),C3DVector(0,1,0));
        mBones[BONE_HAND]=new CBone("rHand",C3DVector(-0.295,0.578,-0.026),C3DVector(-0.302,0.572,-0.016),
                deg2rad(C3DRotation(0,0,0)));
        mBones[BONE_HAND]->AddJoint(JOINTTYPE_BEND,C3DVector(0,0,1),C3DVector(1,0,0),deg2rad(-45),deg2rad(-80.82),deg2rad(-228.28),deg2rad(54.42));
        mBones[BONE_HAND]->AddJoint(JOINTTYPE_SIDE,C3DVector(0,1,0),C3DVector(1,0,0));
        mBones[BONE_HAND]->AddJoint(JOINTTYPE_TWIST,C3DVector(1,0,0),C3DVector(0,1,0));
        mBones[BONE_THUMB1]=new CBone("rThumb1",C3DVector(-0.302,0.572,-0.016),C3DVector(-0.312,0.567,-0.011),
                deg2rad(C3DRotation(26.4735,24.0263,5.68781)));
        mBones[BONE_THUMB1]->AddJoint(JOINTTYPE_SIDE,C3DVector(0,0,1),C3DVector(1,0,0));
        mBones[BONE_THUMB1]->AddJoint(JOINTTYPE_BEND,C3DVector(0,1,0),C3DVector(1,0,0),deg2rad(-86),deg2rad(-151),deg2rad(-241.4),deg2rad(54.25),
                C3DMatrix(0.0224738,0,0,-0.322685,
                    0,0.0218958,0,0.558036,
                    0,0,0.0169274,-0.0102757,
                    0,0,0,1),
                C3DMatrix(-0.0193263,-0.00475875,-0.00196137,-0.322151,
                    -0.00130642,-0.00283651,0.0197546,0.569583,
                    -0.00497855,0.0192174,0.00243011,-0.00492872,
                    0,0,0,1));
        mBones[BONE_THUMB1]->AddJoint(JOINTTYPE_TWIST,C3DVector(1,0,0),C3DVector(0,1,0));
        mBones[BONE_THUMB2]=new CBone("rThumb2",C3DVector(-0.312,0.567,-0.011),C3DVector(-0.321596,0.561863,-0.0104257),
                deg2rad(C3DRotation(23.35,0,0)));
        mBones[BONE_THUMB2]->AddJoint(JOINTTYPE_SIDE,C3DVector(0,0,1),C3DVector(1,0,0));
        mBones[BONE_THUMB2]->AddJoint(JOINTTYPE_BEND,C3DVector(0,1,0),C3DVector(1,0,0),deg2rad(279.822),deg2rad(250.702),deg2rad(94.152),deg2rad(344.347),
                C3DMatrix(0.0237095,-0.000293466,0.00151589,-0.331274,
                    0,0.0270214,0.00683796,0.547778,
                    -0.00116477,-0.00597363,0.0308566,-0.00411732,
                    0,0,0,1),
                C3DMatrix(0.025,0,0,-0.322192,
                    0,0.025,0,0.555079,
                    0,0,0.025,0.00317131,
                    0,0,0,1));
        mBones[BONE_THUMB2]->AddJoint(JOINTTYPE_TWIST,C3DVector(1,0,0),C3DVector(0,1,0));
        mBones[BONE_THUMB3]=new CBone("rThumb3",C3DVector(-0.321596,0.561863,-0.0104257),C3DVector(-0.332921,0.556559,-0.0104379),
                deg2rad(C3DRotation(29.707,-0.040411,-0.0105934)));
        mBones[BONE_THUMB3]->AddJoint(JOINTTYPE_SIDE,C3DVector(0,0,1),C3DVector(1,0,0));
        mBones[BONE_THUMB3]->AddJoint(JOINTTYPE_BEND,C3DVector(0,1,0),C3DVector(1,0,0),deg2rad(-45),deg2rad(-135),deg2rad(-224.07),deg2rad(78.9275));
        mBones[BONE_THUMB3]->AddJoint(JOINTTYPE_TWIST,C3DVector(1,0,0),C3DVector(0,1,0));

        mBones[BONE_INDEX1]=new CBone("rIndex1",C3DVector(-0.326,0.584,-0.013),C3DVector(-0.341154,0.581247,-0.01228),
                deg2rad(C3DRotation(0,0,0)));
        mBones[BONE_INDEX1]->AddJoint(JOINTTYPE_BEND,C3DVector(0,0,1),C3DVector(1,0,0),deg2rad(-77.605),deg2rad(-121.261),deg2rad(-225),deg2rad(45),
                C3DMatrix(-0.0257839,1.51089e-12,-0.0011017,-0.345,
                    2.31522e-12,0.0149913,9.07063e-13,0.578451,
                    0.00318014,1.33599e-12,-0.00893232,-0.009,
                    0,0,0,1),
                C3DMatrix(-0.0338192,1.78232e-12,-0.00080856,-0.336478,
                    3.03674e-12,0.0180272,7.9095e-13,0.590452,
                    0.0034357,1.61043e-12,-0.00795905,-0.011,
                    0,0,0,1));
        mBones[BONE_INDEX1]->AddJoint(JOINTTYPE_SIDE,C3DVector(0,1,0),C3DVector(1,0,0));
        mBones[BONE_INDEX1]->AddJoint(JOINTTYPE_TWIST,C3DVector(1,0,0),C3DVector(0,1,0));
        mBones[BONE_INDEX2]=new CBone("rIndex2",C3DVector(-0.341154,0.581247,-0.01228),C3DVector(-0.3509,0.577297,-0.013137),
                deg2rad(C3DRotation(20.1,0,0)));
        mBones[BONE_INDEX2]->AddJoint(JOINTTYPE_BEND,C3DVector(0,0,1),C3DVector(1,0,0),deg2rad(-69.131),deg2rad(-138.847),deg2rad(-226.963),deg2rad(74.29));
        mBones[BONE_INDEX2]->AddJoint(JOINTTYPE_SIDE,C3DVector(0,1,0),C3DVector(1,0,0));
        mBones[BONE_INDEX2]->AddJoint(JOINTTYPE_TWIST,C3DVector(1,0,0),C3DVector(0,1,0));
        mBones[BONE_INDEX3]=new CBone("rIndex3",C3DVector(-0.3509,0.577297,-0.013137),C3DVector(-0.357874,0.57153,-0.013137),
                deg2rad(C3DRotation(40.0781,0,0)));
        mBones[BONE_INDEX3]->AddJoint(JOINTTYPE_BEND,C3DVector(0,0,1),C3DVector(1,0,0),deg2rad(-60.5111),deg2rad(-135),deg2rad(-247.534),deg2rad(49.561));
        mBones[BONE_INDEX3]->AddJoint(JOINTTYPE_SIDE,C3DVector(0,1,0),C3DVector(1,0,0));
        mBones[BONE_INDEX3]->AddJoint(JOINTTYPE_TWIST,C3DVector(1,0,0),C3DVector(0,1,0));
        mBones[BONE_MIDDLE1]=new CBone("rMid1",C3DVector(-0.328,0.584,-0.023),C3DVector(-0.344,0.58,-0.023),
                deg2rad(C3DRotation(9.3,0,0)));
        mBones[BONE_MIDDLE1]->AddJoint(JOINTTYPE_BEND,C3DVector(0,0,1),C3DVector(1,0,0),deg2rad(-108.398),deg2rad(-128.623),deg2rad(-225),deg2rad(35.023),
                C3DMatrix(-0.0252622,-0.00341832,-0.000501486,-0.355081,
                    -0.00591757,0.0145929,-0.000117471,0.579248,
                    0.00167395,1.34302e-12,-0.0079834,-0.0230741,
                    0,0,0,1),
                C3DMatrix(-0.0330053,-0.00421266,-0.000357755,-0.353999,
                    -0.00794529,0.0174997,-8.61216e-05,0.578994,
                    0.00156338,1.61453e-12,-0.00799043,-0.022,
                    0,0,0,1));
        mBones[BONE_MIDDLE1]->AddJoint(JOINTTYPE_SIDE,C3DVector(0,1,0),C3DVector(1,0,0));
        mBones[BONE_MIDDLE1]->AddJoint(JOINTTYPE_TWIST,C3DVector(1,0,0),C3DVector(0,1,0));
        mBones[BONE_MIDDLE2]=new CBone("rMid2",C3DVector(-0.344,0.58,-0.023),C3DVector(-0.351546,0.571702,-0.022812),
                deg2rad(C3DRotation(44.1,0,0)));
        mBones[BONE_MIDDLE2]->AddJoint(JOINTTYPE_BEND,C3DVector(0,0,1),C3DVector(1,0,0),deg2rad(-78.032),deg2rad(-142.924),deg2rad(-204.562),deg2rad(15.878));
        mBones[BONE_MIDDLE2]->AddJoint(JOINTTYPE_SIDE,C3DVector(0,1,0),C3DVector(1,0,0));
        mBones[BONE_MIDDLE2]->AddJoint(JOINTTYPE_TWIST,C3DVector(1,0,0),C3DVector(0,1,0));
        mBones[BONE_MIDDLE3]=new CBone("rMid3",C3DVector(-0.351546,0.571702,-0.022812),C3DVector(-0.355334,0.56203,-0.0219511),
                deg2rad(C3DRotation(68.4776,4.73336,3.21431)));
        mBones[BONE_MIDDLE3]->AddJoint(JOINTTYPE_BEND,C3DVector(0,0,1),C3DVector(1,0,0),deg2rad(-55.204),deg2rad(-125.268),deg2rad(-240.872),deg2rad(38.606));
        mBones[BONE_MIDDLE3]->AddJoint(JOINTTYPE_SIDE,C3DVector(0,1,0),C3DVector(1,0,0));
        mBones[BONE_MIDDLE3]->AddJoint(JOINTTYPE_TWIST,C3DVector(1,0,0),C3DVector(0,1,0));
        mBones[BONE_RING1]=new CBone("rRing1",C3DVector(-0.328,0.582,-0.033),C3DVector(-0.341,0.577,-0.032),
                deg2rad(C3DRotation(8.8,0,0)));
        mBones[BONE_RING1]->AddJoint(JOINTTYPE_BEND,C3DVector(0,0,1),C3DVector(1,0,0),deg2rad(-81.507),deg2rad(-130.327),deg2rad(-225),deg2rad(45),
                C3DMatrix(-0.0244573,-0.00466738,-0.000262016,-0.351,
                    -0.0080796,0.0141283,-8.65584e-05,0.574938,
                    0.00118617,1.33465e-12,-0.00599201,-0.0321619,
                    0,0,0,1),
                C3DMatrix(-0.0335683,-0.00280356,-9.79463e-14,-0.352283,
                    -0.00529559,0.0177715,6.20874e-13,0.57647,
                    0,1.6155e-12,-0.007,-0.0333242,
                    0,0,0,1));
        mBones[BONE_RING1]->AddJoint(JOINTTYPE_SIDE,C3DVector(0,1,0),C3DVector(1,0,0));
        mBones[BONE_RING1]->AddJoint(JOINTTYPE_TWIST,C3DVector(1,0,0),C3DVector(0,1,0));
        mBones[BONE_RING2]=new CBone("rRing2",C3DVector(-0.341,0.577,-0.032),C3DVector(-0.346693,0.56873,-0.030852),
                deg2rad(C3DRotation(56.05,0,0)));
        mBones[BONE_RING2]->AddJoint(JOINTTYPE_BEND,C3DVector(0,0,1),C3DVector(1,0,0),deg2rad(-109.009),deg2rad(-138.499),deg2rad(-230.209),deg2rad(7.43201));
        mBones[BONE_RING2]->AddJoint(JOINTTYPE_SIDE,C3DVector(0,1,0),C3DVector(1,0,0));
        mBones[BONE_RING2]->AddJoint(JOINTTYPE_TWIST,C3DVector(1,0,0),C3DVector(0,1,0));
        mBones[BONE_RING3]=new CBone("rRing3",C3DVector(-0.346693,0.56873,-0.030852),C3DVector(-0.349224,0.560336,-0.030852),
                deg2rad(C3DRotation(69.4336,0,0)));
        mBones[BONE_RING3]->AddJoint(JOINTTYPE_BEND,C3DVector(0,0,1),C3DVector(1,0,0),deg2rad(-69.1946),deg2rad(-135),deg2rad(-225),deg2rad(32.8515));
        mBones[BONE_RING3]->AddJoint(JOINTTYPE_SIDE,C3DVector(0,1,0),C3DVector(1,0,0));
        mBones[BONE_RING3]->AddJoint(JOINTTYPE_TWIST,C3DVector(1,0,0),C3DVector(0,1,0));
        mBones[BONE_PINKY1]=new CBone("rPinky1",C3DVector(-0.324,0.577,-0.041),C3DVector(-0.33416,0.573738,-0.040596),
                deg2rad(C3DRotation(11.8,0,0)));
        mBones[BONE_PINKY1]->AddJoint(JOINTTYPE_BEND,C3DVector(0,0,1),C3DVector(1,0,0),deg2rad(-64.562),deg2rad(-123),deg2rad(-243.833),deg2rad(39.117),
                C3DMatrix(-0.0209107,-0.00572543,-0.000413835,-0.340063,
                    -0.00957782,0.0124834,0.000950032,0.566185,
                    -7.05629e-05,0.00225302,-0.00631554,-0.0415074,
                    0,0,0,1),
                C3DMatrix(-0.0333225,-0.00319124,0.000307597,-0.347,
                    -0.00588737,0.0175841,0.000628849,0.575,
                    -0.00197473,0.00142613,-0.00706535,-0.044,
                    0,0,0,1));
        mBones[BONE_PINKY1]->AddJoint(JOINTTYPE_SIDE,C3DVector(0,1,0),C3DVector(1,0,0));
        mBones[BONE_PINKY1]->AddJoint(JOINTTYPE_TWIST,C3DVector(1,0,0),C3DVector(0,1,0));
        mBones[BONE_PINKY2]=new CBone("rPinky2",C3DVector(-0.33416,0.573738,-0.040596),C3DVector(-0.339,0.568,-0.04),
                deg2rad(C3DRotation(44.1,0,0)));
        mBones[BONE_PINKY2]->AddJoint(JOINTTYPE_BEND,C3DVector(0,0,1),C3DVector(1,0,0),deg2rad(-55.306),deg2rad(-135),deg2rad(-225),deg2rad(50.251));
        mBones[BONE_PINKY2]->AddJoint(JOINTTYPE_SIDE,C3DVector(0,1,0),C3DVector(1,0,0));
        mBones[BONE_PINKY2]->AddJoint(JOINTTYPE_TWIST,C3DVector(1,0,0),C3DVector(0,1,0));
        mBones[BONE_PINKY3]=new CBone("rPinky3",C3DVector(-0.339,0.568,-0.04),C3DVector(-0.343951,0.561392,-0.039545),
                deg2rad(C3DRotation(51.8555,0,0)));
        mBones[BONE_PINKY3]->AddJoint(JOINTTYPE_BEND,C3DVector(0,0,1),C3DVector(1,0,0),deg2rad(-46.793),deg2rad(-118.851),deg2rad(-225),deg2rad(45));
        mBones[BONE_PINKY3]->AddJoint(JOINTTYPE_SIDE,C3DVector(0,1,0),C3DVector(1,0,0));
        mBones[BONE_PINKY3]->AddJoint(JOINTTYPE_TWIST,C3DVector(1,0,0),C3DVector(0,1,0));

        mSkeleton->SetRoot(mBones[BONE_FOREARM]);
        mBones[BONE_FOREARM]->AddChild(mBones[BONE_HAND]);
        mBones[BONE_HAND]->AddChild(mBones[BONE_THUMB1]);
        mBones[BONE_THUMB1]->AddChild(mBones[BONE_THUMB2]);
        mBones[BONE_THUMB2]->AddChild(mBones[BONE_THUMB3]);
        mBones[BONE_HAND]->AddChild(mBones[BONE_INDEX1]);
        mBones[BONE_INDEX1]->AddChild(mBones[BONE_INDEX2]);
        mBones[BONE_INDEX2]->AddChild(mBones[BONE_INDEX3]);
        mBones[BONE_HAND]->AddChild(mBones[BONE_MIDDLE1]);
        mBones[BONE_MIDDLE1]->AddChild(mBones[BONE_MIDDLE2]);
        mBones[BONE_MIDDLE2]->AddChild(mBones[BONE_MIDDLE3]);
        mBones[BONE_HAND]->AddChild(mBones[BONE_RING1]);
        mBones[BONE_RING1]->AddChild(mBones[BONE_RING2]);
        mBones[BONE_RING2]->AddChild(mBones[BONE_RING3]);
        mBones[BONE_HAND]->AddChild(mBones[BONE_PINKY1]);
        mBones[BONE_PINKY1]->AddChild(mBones[BONE_PINKY2]);
        mBones[BONE_PINKY2]->AddChild(mBones[BONE_PINKY3]);

        mMesh->CalcNormals();
        mMesh->SmoothNormals();
        img::CImage_rgb8 lTexImage;
        load(lTexImage,pTexturePath);
        mMesh->SetTexture(lTexImage);
        mSkeleton->Prepare(mMesh);

    }
        const buola::scene::PSkeleton& GetSkeleton() const{return mSkeleton;}

        buola::scene::PBone& operator[] (unsigned i){return mBones[i];}
        const buola::scene::PBone& operator[] (unsigned i) const {return mBones[i];}

    private:
        buola::scene::PSkeleton mSkeleton;
        boost::array<buola::scene::PBone,BONE_COUNT> mBones;
        buola::scene::PMesh mMesh;
};

#endif
