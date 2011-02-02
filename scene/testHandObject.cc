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
#include <buola/image/io_pgm.h>
#include <buola/scene/crttransform.h>

#include <buola/geometry/clookatmatrix.h>
#include <buola/geometry/cperspectiveprojectionmatrix.h>

using namespace buola;

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

scene::PSkeleton create_skeleton()
{
    using namespace buola::scene;
    
    PSkeleton lSkeleton=new CSkeleton;
    PBone lBones[BONE_COUNT];
    
    lBones[BONE_FOREARM]=new CBone("rForeArm",C3DVector(-0.276,0.58,-0.026),C3DVector(-0.295,0.578,-0.026),deg2rad(C3DRotation(0,0,0)));
    lBones[BONE_FOREARM]->AddJoint(JOINTTYPE_BEND,C3DVector(0,1,0),C3DVector(1,0,0));
    lBones[BONE_FOREARM]->AddJoint(JOINTTYPE_SIDE,C3DVector(0,0,1),C3DVector(1,0,0));
    lBones[BONE_FOREARM]->AddJoint(JOINTTYPE_TWIST,C3DVector(1,0,0),C3DVector(0,1,0));
    lBones[BONE_HAND]=new CBone("rHand",C3DVector(-0.295,0.578,-0.026),C3DVector(-0.302,0.572,-0.016),
                                deg2rad(C3DRotation(0,0,0)));
    lBones[BONE_HAND]->AddJoint(JOINTTYPE_BEND,C3DVector(0,0,1),C3DVector(1,0,0),deg2rad(-45),deg2rad(-80.82),deg2rad(-228.28),deg2rad(54.42));
    lBones[BONE_HAND]->AddJoint(JOINTTYPE_SIDE,C3DVector(0,1,0),C3DVector(1,0,0));
    lBones[BONE_HAND]->AddJoint(JOINTTYPE_TWIST,C3DVector(1,0,0),C3DVector(0,1,0));
    lBones[BONE_THUMB1]=new CBone("rThumb1",C3DVector(-0.302,0.572,-0.016),C3DVector(-0.312,0.567,-0.011),
                                  deg2rad(C3DRotation(26.4735,24.0263,5.68781)));
    lBones[BONE_THUMB1]->AddJoint(JOINTTYPE_SIDE,C3DVector(0,0,1),C3DVector(1,0,0));
    lBones[BONE_THUMB1]->AddJoint(JOINTTYPE_BEND,C3DVector(0,1,0),C3DVector(1,0,0),deg2rad(-86),deg2rad(-151),deg2rad(-241.4),deg2rad(54.25),
                                  C3DMatrix(0.0224738,0,0,-0.322685,
                                            0,0.0218958,0,0.558036,
                                            0,0,0.0169274,-0.0102757,
                                            0,0,0,1),
                                  C3DMatrix(-0.0193263,-0.00475875,-0.00196137,-0.322151,
                                            -0.00130642,-0.00283651,0.0197546,0.569583,
                                            -0.00497855,0.0192174,0.00243011,-0.00492872,
                                            0,0,0,1));
    lBones[BONE_THUMB1]->AddJoint(JOINTTYPE_TWIST,C3DVector(1,0,0),C3DVector(0,1,0));
    lBones[BONE_THUMB2]=new CBone("rThumb2",C3DVector(-0.312,0.567,-0.011),C3DVector(-0.321596,0.561863,-0.0104257),
                                  deg2rad(C3DRotation(23.35,0,0)));
    lBones[BONE_THUMB2]->AddJoint(JOINTTYPE_SIDE,C3DVector(0,0,1),C3DVector(1,0,0));
    lBones[BONE_THUMB2]->AddJoint(JOINTTYPE_BEND,C3DVector(0,1,0),C3DVector(1,0,0),deg2rad(279.822),deg2rad(250.702),deg2rad(94.152),deg2rad(344.347),
                                  C3DMatrix(0.0237095,-0.000293466,0.00151589,-0.331274,
                                            0,0.0270214,0.00683796,0.547778,
                                            -0.00116477,-0.00597363,0.0308566,-0.00411732,
                                            0,0,0,1),
                                  C3DMatrix(0.025,0,0,-0.322192,
                                            0,0.025,0,0.555079,
                                            0,0,0.025,0.00317131,
                                            0,0,0,1));
    lBones[BONE_THUMB2]->AddJoint(JOINTTYPE_TWIST,C3DVector(1,0,0),C3DVector(0,1,0));
    lBones[BONE_THUMB3]=new CBone("rThumb3",C3DVector(-0.321596,0.561863,-0.0104257),C3DVector(-0.332921,0.556559,-0.0104379),
                                  deg2rad(C3DRotation(29.707,-0.040411,-0.0105934)));
    lBones[BONE_THUMB3]->AddJoint(JOINTTYPE_SIDE,C3DVector(0,0,1),C3DVector(1,0,0));
    lBones[BONE_THUMB3]->AddJoint(JOINTTYPE_BEND,C3DVector(0,1,0),C3DVector(1,0,0),deg2rad(-45),deg2rad(-135),deg2rad(-224.07),deg2rad(78.9275));
    lBones[BONE_THUMB3]->AddJoint(JOINTTYPE_TWIST,C3DVector(1,0,0),C3DVector(0,1,0));

    lBones[BONE_INDEX1]=new CBone("rIndex1",C3DVector(-0.326,0.584,-0.013),C3DVector(-0.341154,0.581247,-0.01228),
                                  deg2rad(C3DRotation(0,0,0)));
    lBones[BONE_INDEX1]->AddJoint(JOINTTYPE_BEND,C3DVector(0,0,1),C3DVector(1,0,0),deg2rad(-77.605),deg2rad(-121.261),deg2rad(-225),deg2rad(45),
                                  C3DMatrix(-0.0257839,1.51089e-12,-0.0011017,-0.345,
                                            2.31522e-12,0.0149913,9.07063e-13,0.578451,
                                            0.00318014,1.33599e-12,-0.00893232,-0.009,
                                            0,0,0,1),
                                  C3DMatrix(-0.0338192,1.78232e-12,-0.00080856,-0.336478,
                                            3.03674e-12,0.0180272,7.9095e-13,0.590452,
                                            0.0034357,1.61043e-12,-0.00795905,-0.011,
                                            0,0,0,1));
    lBones[BONE_INDEX1]->AddJoint(JOINTTYPE_SIDE,C3DVector(0,1,0),C3DVector(1,0,0));
    lBones[BONE_INDEX1]->AddJoint(JOINTTYPE_TWIST,C3DVector(1,0,0),C3DVector(0,1,0));
    lBones[BONE_INDEX2]=new CBone("rIndex2",C3DVector(-0.341154,0.581247,-0.01228),C3DVector(-0.3509,0.577297,-0.013137),
                                  deg2rad(C3DRotation(20.1,0,0)));
    lBones[BONE_INDEX2]->AddJoint(JOINTTYPE_BEND,C3DVector(0,0,1),C3DVector(1,0,0),deg2rad(-69.131),deg2rad(-138.847),deg2rad(-226.963),deg2rad(74.29));
    lBones[BONE_INDEX2]->AddJoint(JOINTTYPE_SIDE,C3DVector(0,1,0),C3DVector(1,0,0));
    lBones[BONE_INDEX2]->AddJoint(JOINTTYPE_TWIST,C3DVector(1,0,0),C3DVector(0,1,0));
    lBones[BONE_INDEX3]=new CBone("rIndex3",C3DVector(-0.3509,0.577297,-0.013137),C3DVector(-0.357874,0.57153,-0.013137),
                                  deg2rad(C3DRotation(40.0781,0,0)));
    lBones[BONE_INDEX3]->AddJoint(JOINTTYPE_BEND,C3DVector(0,0,1),C3DVector(1,0,0),deg2rad(-60.5111),deg2rad(-135),deg2rad(-247.534),deg2rad(49.561));
    lBones[BONE_INDEX3]->AddJoint(JOINTTYPE_SIDE,C3DVector(0,1,0),C3DVector(1,0,0));
    lBones[BONE_INDEX3]->AddJoint(JOINTTYPE_TWIST,C3DVector(1,0,0),C3DVector(0,1,0));
    lBones[BONE_MIDDLE1]=new CBone("rMid1",C3DVector(-0.328,0.584,-0.023),C3DVector(-0.344,0.58,-0.023),
                                   deg2rad(C3DRotation(9.3,0,0)));
    lBones[BONE_MIDDLE1]->AddJoint(JOINTTYPE_BEND,C3DVector(0,0,1),C3DVector(1,0,0),deg2rad(-108.398),deg2rad(-128.623),deg2rad(-225),deg2rad(35.023),
                                   C3DMatrix(-0.0252622,-0.00341832,-0.000501486,-0.355081,
                                             -0.00591757,0.0145929,-0.000117471,0.579248,
                                             0.00167395,1.34302e-12,-0.0079834,-0.0230741,
                                             0,0,0,1),
                                   C3DMatrix(-0.0330053,-0.00421266,-0.000357755,-0.353999,
                                             -0.00794529,0.0174997,-8.61216e-05,0.578994,
                                             0.00156338,1.61453e-12,-0.00799043,-0.022,
                                             0,0,0,1));
    lBones[BONE_MIDDLE1]->AddJoint(JOINTTYPE_SIDE,C3DVector(0,1,0),C3DVector(1,0,0));
    lBones[BONE_MIDDLE1]->AddJoint(JOINTTYPE_TWIST,C3DVector(1,0,0),C3DVector(0,1,0));
    lBones[BONE_MIDDLE2]=new CBone("rMid2",C3DVector(-0.344,0.58,-0.023),C3DVector(-0.351546,0.571702,-0.022812),
                                   deg2rad(C3DRotation(44.1,0,0)));
    lBones[BONE_MIDDLE2]->AddJoint(JOINTTYPE_BEND,C3DVector(0,0,1),C3DVector(1,0,0),deg2rad(-78.032),deg2rad(-142.924),deg2rad(-204.562),deg2rad(15.878));
    lBones[BONE_MIDDLE2]->AddJoint(JOINTTYPE_SIDE,C3DVector(0,1,0),C3DVector(1,0,0));
    lBones[BONE_MIDDLE2]->AddJoint(JOINTTYPE_TWIST,C3DVector(1,0,0),C3DVector(0,1,0));
    lBones[BONE_MIDDLE3]=new CBone("rMid3",C3DVector(-0.351546,0.571702,-0.022812),C3DVector(-0.355334,0.56203,-0.0219511),
                                   deg2rad(C3DRotation(68.4776,4.73336,3.21431)));
    lBones[BONE_MIDDLE3]->AddJoint(JOINTTYPE_BEND,C3DVector(0,0,1),C3DVector(1,0,0),deg2rad(-55.204),deg2rad(-125.268),deg2rad(-240.872),deg2rad(38.606));
    lBones[BONE_MIDDLE3]->AddJoint(JOINTTYPE_SIDE,C3DVector(0,1,0),C3DVector(1,0,0));
    lBones[BONE_MIDDLE3]->AddJoint(JOINTTYPE_TWIST,C3DVector(1,0,0),C3DVector(0,1,0));
    lBones[BONE_RING1]=new CBone("rRing1",C3DVector(-0.328,0.582,-0.033),C3DVector(-0.341,0.577,-0.032),
                                 deg2rad(C3DRotation(8.8,0,0)));
    lBones[BONE_RING1]->AddJoint(JOINTTYPE_BEND,C3DVector(0,0,1),C3DVector(1,0,0),deg2rad(-81.507),deg2rad(-130.327),deg2rad(-225),deg2rad(45),
                                 C3DMatrix(-0.0244573,-0.00466738,-0.000262016,-0.351,
                                           -0.0080796,0.0141283,-8.65584e-05,0.574938,
                                           0.00118617,1.33465e-12,-0.00599201,-0.0321619,
                                           0,0,0,1),
                                 C3DMatrix(-0.0335683,-0.00280356,-9.79463e-14,-0.352283,
                                           -0.00529559,0.0177715,6.20874e-13,0.57647,
                                           0,1.6155e-12,-0.007,-0.0333242,
                                           0,0,0,1));
    lBones[BONE_RING1]->AddJoint(JOINTTYPE_SIDE,C3DVector(0,1,0),C3DVector(1,0,0));
    lBones[BONE_RING1]->AddJoint(JOINTTYPE_TWIST,C3DVector(1,0,0),C3DVector(0,1,0));
    lBones[BONE_RING2]=new CBone("rRing2",C3DVector(-0.341,0.577,-0.032),C3DVector(-0.346693,0.56873,-0.030852),
                                 deg2rad(C3DRotation(56.05,0,0)));
    lBones[BONE_RING2]->AddJoint(JOINTTYPE_BEND,C3DVector(0,0,1),C3DVector(1,0,0),deg2rad(-109.009),deg2rad(-138.499),deg2rad(-230.209),deg2rad(7.43201));
    lBones[BONE_RING2]->AddJoint(JOINTTYPE_SIDE,C3DVector(0,1,0),C3DVector(1,0,0));
    lBones[BONE_RING2]->AddJoint(JOINTTYPE_TWIST,C3DVector(1,0,0),C3DVector(0,1,0));
    lBones[BONE_RING3]=new CBone("rRing3",C3DVector(-0.346693,0.56873,-0.030852),C3DVector(-0.349224,0.560336,-0.030852),
                                 deg2rad(C3DRotation(69.4336,0,0)));
    lBones[BONE_RING3]->AddJoint(JOINTTYPE_BEND,C3DVector(0,0,1),C3DVector(1,0,0),deg2rad(-69.1946),deg2rad(-135),deg2rad(-225),deg2rad(32.8515));
    lBones[BONE_RING3]->AddJoint(JOINTTYPE_SIDE,C3DVector(0,1,0),C3DVector(1,0,0));
    lBones[BONE_RING3]->AddJoint(JOINTTYPE_TWIST,C3DVector(1,0,0),C3DVector(0,1,0));
    lBones[BONE_PINKY1]=new CBone("rPinky1",C3DVector(-0.324,0.577,-0.041),C3DVector(-0.33416,0.573738,-0.040596),
                                  deg2rad(C3DRotation(11.8,0,0)));
    lBones[BONE_PINKY1]->AddJoint(JOINTTYPE_BEND,C3DVector(0,0,1),C3DVector(1,0,0),deg2rad(-64.562),deg2rad(-123),deg2rad(-243.833),deg2rad(39.117),
                                  C3DMatrix(-0.0209107,-0.00572543,-0.000413835,-0.340063,
                                            -0.00957782,0.0124834,0.000950032,0.566185,
                                            -7.05629e-05,0.00225302,-0.00631554,-0.0415074,
                                            0,0,0,1),
                                  C3DMatrix(-0.0333225,-0.00319124,0.000307597,-0.347,
                                            -0.00588737,0.0175841,0.000628849,0.575,
                                            -0.00197473,0.00142613,-0.00706535,-0.044,
                                            0,0,0,1));
    lBones[BONE_PINKY1]->AddJoint(JOINTTYPE_SIDE,C3DVector(0,1,0),C3DVector(1,0,0));
    lBones[BONE_PINKY1]->AddJoint(JOINTTYPE_TWIST,C3DVector(1,0,0),C3DVector(0,1,0));
    lBones[BONE_PINKY2]=new CBone("rPinky2",C3DVector(-0.33416,0.573738,-0.040596),C3DVector(-0.339,0.568,-0.04),
                                  deg2rad(C3DRotation(44.1,0,0)));
    lBones[BONE_PINKY2]->AddJoint(JOINTTYPE_BEND,C3DVector(0,0,1),C3DVector(1,0,0),deg2rad(-55.306),deg2rad(-135),deg2rad(-225),deg2rad(50.251));
    lBones[BONE_PINKY2]->AddJoint(JOINTTYPE_SIDE,C3DVector(0,1,0),C3DVector(1,0,0));
    lBones[BONE_PINKY2]->AddJoint(JOINTTYPE_TWIST,C3DVector(1,0,0),C3DVector(0,1,0));
    lBones[BONE_PINKY3]=new CBone("rPinky3",C3DVector(-0.339,0.568,-0.04),C3DVector(-0.343951,0.561392,-0.039545),
                                  deg2rad(C3DRotation(51.8555,0,0)));
    lBones[BONE_PINKY3]->AddJoint(JOINTTYPE_BEND,C3DVector(0,0,1),C3DVector(1,0,0),deg2rad(-46.793),deg2rad(-118.851),deg2rad(-225),deg2rad(45));
    lBones[BONE_PINKY3]->AddJoint(JOINTTYPE_SIDE,C3DVector(0,1,0),C3DVector(1,0,0));
    lBones[BONE_PINKY3]->AddJoint(JOINTTYPE_TWIST,C3DVector(1,0,0),C3DVector(0,1,0));

    lSkeleton->SetRoot(lBones[BONE_FOREARM]);
    lBones[BONE_FOREARM]->AddChild(lBones[BONE_HAND]);
    lBones[BONE_HAND]->AddChild(lBones[BONE_THUMB1]);
    lBones[BONE_THUMB1]->AddChild(lBones[BONE_THUMB2]);
    lBones[BONE_THUMB2]->AddChild(lBones[BONE_THUMB3]);
    lBones[BONE_HAND]->AddChild(lBones[BONE_INDEX1]);
    lBones[BONE_INDEX1]->AddChild(lBones[BONE_INDEX2]);
    lBones[BONE_INDEX2]->AddChild(lBones[BONE_INDEX3]);
    lBones[BONE_HAND]->AddChild(lBones[BONE_MIDDLE1]);
    lBones[BONE_MIDDLE1]->AddChild(lBones[BONE_MIDDLE2]);
    lBones[BONE_MIDDLE2]->AddChild(lBones[BONE_MIDDLE3]);
    lBones[BONE_HAND]->AddChild(lBones[BONE_RING1]);
    lBones[BONE_RING1]->AddChild(lBones[BONE_RING2]);
    lBones[BONE_RING2]->AddChild(lBones[BONE_RING3]);
    lBones[BONE_HAND]->AddChild(lBones[BONE_PINKY1]);
    lBones[BONE_PINKY1]->AddChild(lBones[BONE_PINKY2]);
    lBones[BONE_PINKY2]->AddChild(lBones[BONE_PINKY3]);

    double lJointValues[]=
    {
        0.000,0.000,0.000,
        2.100,7.102,-3.408,
        2.872,30.377,24.070,
        -1.300,0,0,
        -23.800,0,0,
        48.200,6.300,10.000,
        11.000,0,0,
        10.200,0,0,
        35.200,0.000,0.000,
        0.000,0,0,
        0.500,0,0,
        41.300,-17.000,0.000,
        6.600,0,0,
        7.800,0,0,
        73.000,8.000,-6.000,
        10.300,0,0,
        36.800,0,0
    };

    for(int i=0;i<17;i++)
    {
        lBones[i]->SetJointValue(JOINTTYPE_BEND,deg2rad(lJointValues[i*3]));
        lBones[i]->SetJointValue(JOINTTYPE_SIDE,deg2rad(lJointValues[i*3+1]));
        lBones[i]->SetJointValue(JOINTTYPE_TWIST,deg2rad(lJointValues[i*3+2]));
    }

    msg_info() << lBones[BONE_FOREARM]->GetTransformedOrigin() << "\n";
    msg_info() << lBones[BONE_THUMB3]->GetTransformedEndPoint()-lBones[BONE_FOREARM]->GetTransformedOrigin() << "\n";
    msg_info() << lBones[BONE_INDEX3]->GetTransformedEndPoint()-lBones[BONE_FOREARM]->GetTransformedOrigin() << "\n";
    msg_info() << lBones[BONE_MIDDLE3]->GetTransformedEndPoint()-lBones[BONE_FOREARM]->GetTransformedOrigin() << "\n";
    msg_info() << lBones[BONE_RING3]->GetTransformedEndPoint()-lBones[BONE_FOREARM]->GetTransformedOrigin() << "\n";
    msg_info() << lBones[BONE_PINKY3]->GetTransformedEndPoint()-lBones[BONE_FOREARM]->GetTransformedOrigin() << "\n";

    return lSkeleton;
}

int main(int pNArg,char **pArgs)
{
    buola_init(pNArg,pArgs);

    try
    {
        create_skeleton();
        
        CApp lApp;

        scene::PPerspectiveCamera lCamera=new scene::CPerspectiveCamera;
        lCamera->LookAt(C3DVector(-0.27,0.59,0),C3DVector(-0.5,0.5,0.0),C3DVector(0,0,1));
//        lCamera->LookAt(C3DVector(-0.27,0.59,0),C3DVector(2,0,0),C3DVector(0,0,1));
        lCamera->SetClipping(0.01,20000);

        scene::PMesh lMesh=scene::CMesh::Import("/home/javier/tmp/testHandObject/rHandP3.obj");
        lMesh->CalcNormals();
        lMesh->SmoothNormals();

        img::CImage_rgb8 lTexImage;
        load_pgm(lTexImage,"/home/javier/tmp/testHandObject/hand_texture.ppm");
        lMesh->SetTexture(lTexImage);
        
        scene::PScene lScene=new scene::CScene;
        scene::PSkeleton lSkeleton=create_skeleton();
        lSkeleton->Prepare(lMesh);
        scene::PRTTransform lHandTransform=new scene::CRTTransform;
        lHandTransform->SetTranslation({0.2,0.2,0});
        lScene->GetWorld()->AddChild(lHandTransform);
        lHandTransform->AddChild(lSkeleton->GetRoot()->GetTransform());
        lScene->AddObject(lSkeleton);
        
        scene::PRTTransform lTransform=new scene::CRTTransform;
        lTransform->SetTranslation({0.5,0.8,0});
        scene::PGeode lGeode=scene::CGeode::Import("/home/javier/tmp/objAfterFeedback/objs/adductedThumb_onlyObject.obj",0.2);
        lGeode->AttachTo(lTransform);
        lScene->GetWorld()->AddChild(lTransform);
        
        scene::CSceneView lView;
        lView.Create(NULL,CSize(800,600));
        lView.SetCamera(lCamera);
        lView.SetScene(lScene);

        lView.Show();

        lApp.Run();
    }
    catch(std::exception &pE)
    {
        msg_info() << "caught exception " << pE.what() << "\n";
    }

    return buola_finish();
}
