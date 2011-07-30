#ifndef __HANDRENDERER_H
#define __HANDRENDERER_H

class HandRenderer
{
  public:
    HandRenderer():
      mCamera(new scene::CPerspectiveCamera),
      mScene(new scene::CScene),
      mRenderer(),
      mImage({400,400}),
      mHandTransf(new scene::CRTTransform),
      mObjTransf(new scene::CRTTransform)
  {
    mCamera->SetClipping(0.01,200);
    mRenderer.SetClearColor(buola::CColor(0,0,0));
    mObjTrans.SetTranslation(buola::C3DVector(0,0,0));
  }
    void render(const C3DVector &pCamAt,
                const C3DVector &pCamFrom,
                const C3DVector &pCamUp,
                const tFullPoseV &pPose,
                const buola::C3DVector &pHandTranslation)
    {
      mHandTransf->SetTranslation(pHandTranslation);
      mScene->AddObject(mSkeleton.GetSkeleton());
    }

  private:
    scene::PPerspectiveCamera mCamera;
    scene::PScene mScene;
    scene::CImageRenderer mRenderer;
    buola::img::CImage_rgb8 mImage;
    scene::PRTTransform mHandTransf;
    scene::PRTTransform mObjTransf;
};

#endif // __HANDRENDERER_H
