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
/**
 * @file cTestGUI2.h
 * Contains class CTESTGUI2 and its specialization handtrackerGUI.
 * Handles the GUI and contains a Handclass instance
 *
 * @brief Handles the GUI and contains a Handclass instance
 *
 * @author Javier Romero
 */
#ifndef CTESTGUI2_H_
#define CTESTGUI2_H_


#include "exactNN.h"
#include "approxNNflann.h"
#include "approxNNlshkit.h" // problems with boost headers; i have to put it here
#include "handclass_config.h"
#include <buola/gui/cdisplaywindow.h>
#ifdef CAM_ACCESS
#include <oldrobot/ccorbacamclient.h>
#endif
#include <buola/io/cfolder.h>
#include <buola/utility/string.h>
#include <buola/gui/cicon.h>
#include <buola/gui/cmenu.h>
#include <buola/gl.h>
//#include <buola/image/cgc.h>

#include "approxNN.h"
#include "handclass_config.h"
#include "poselist_multi.h"
#include "handclass.h"
#include "processHog.h"

namespace acc=boost::accumulators;
/** @brief General class handling the GUI and the images to be shown.
 * It has an instance of Handclass which does all the process.
 */
template <typename PL,typename tType>
class CTestGUI2 : public buola::gui::CDisplayWindow
{
    public:
        /** Gets the path to all images (in case of online execution) and
         * stores it in mFiles
         */
        void GetFilePaths()
        {
            buola::TString lPath;
            std::cout << "Write the path to the folder that contains the images, or press ENTER for online recognition" << std::endl;
            std::cin >> lPath;
            std::cout << "Write the inverse probability of worsen the mask (1 out of N)" << std::endl;
            std::cin >> mWorsen;
            mWorsen = (mWorsen>1000)?INVPROBDEFAULT:mWorsen; 
            std::cout << mWorsen << std::endl;
            mOfflineImages = (lPath.size()>2);
            if(!mOfflineImages)
            {
#ifdef CAM_ACCESS
                mCam.Init();
#else
                std::cerr << "Recompile with CAM_ACCESS ON with ccmake" << std::endl;
                exit(-1);
#endif
            }
            else
            {
                buola::PFolder lFolder=buola::CFolder::Get(lPath);
                std::vector<buola::CURL> lURLs;
                lFolder->GetAllURLs(lURLs);
                std::vector<std::string> lInfoPaths;
                for(int i=0;i<lURLs.size();++i)
                {
                    if(boost::ends_with(lURLs[i].GetName(),".jpg"))
                        mFiles.push_back(lURLs[i].GetFullPath());
                    else if(boost::ends_with(lURLs[i].GetName(),".info"))
                    {
                        mGroundTruth = true;
                        lInfoPaths.push_back(lURLs[i].GetFullPath());
                    }
                }
                std::sort(mFiles.begin(),mFiles.end());
                if(mGroundTruth)
                {
                    std::string lString;
                    std::sort(lInfoPaths.begin(),lInfoPaths.end());
                    for(int i=0;i<lInfoPaths.size();i++)
                    {              
                        std::ifstream lInfoStream(lInfoPaths[i].c_str());       
                        mGroundTruthPoses.push_back(tVectorD(0,0));
                        getline(lInfoStream,lString); //comment
                        lString.clear();
                        getline(lInfoStream,lString); //ori
                        std::istringstream lORIS(lString);
                        std::copy(std::istream_iterator<double>(lORIS),std::istream_iterator<double>(),back_inserter(mGroundTruthPoses[i]));
                        getline(lInfoStream,lString); //comment
                        lString.clear();
                        getline(lInfoStream,lString); //joints
                        std::istringstream lJOINTS(lString);
                        std::copy(std::istream_iterator<double>(lJOINTS),std::istream_iterator<double>(),back_inserter(mGroundTruthPoses[i]));
                        lString.clear();
                    }
                    mGroundTruthItr=mGroundTruthPoses.begin();
                }
                mFilesItr=mFiles.begin();
            }
        }
        /** Initializes mHandClass and all the pointers it could need.
         * Version for PF without approxNN
         */ 
        CTestGUI2(Feature<tType>* pFeat):
            mHaveImages(false),
            mStep(false),
            mGroundTruth(false),
            mHandClass("out",&mProcessHog,pFeat),
            mProcessHog(&mPoselist)
            {GetFilePaths();StartRecording();}

        /** Initializes mHandClass and all the pointers it could need.
         * Version for non-PF with approxNN
         */
        CTestGUI2(nn<tType>* pApproxNN,Feature<tType>* pFeat):
            mHaveImages(false),
            mStep(false),
            mGroundTruth(false),
            mHandClass("out",&mProcessHog,pFeat),
            mProcessHog(pApproxNN,&mPoselist)
            {GetFilePaths();StartRecording();}
        virtual  ~CTestGUI2(){};

        /** Check if there is a new frame, process it and create images for showing.
        */
        virtual  void DoProcessing()
        {
            if(!mOfflineImages)
            {
#ifdef CAM_ACCESS
                mHaveImages = mCam.IsNextFrameReady(CCamClient::CAM_ALL);
                if(mHaveImages)
                    mCam.CopyCurrentImage(mImage,CCamClient::CAM_WLEFT);
                else
                    return;
#else
                std::cerr << "Recompile with CAM_ACCESS ON with ccmake" << std::endl;
                exit(-1);
#endif
            }
            else
            {
                if(((mStep && mHaveImages)||!mStep) && mFilesItr!=mFiles.end())
                    boost::gil::jpeg_read_image(*(mFilesItr++), mImage);
                else if (mFilesItr == mFiles.end())
                {
                    if(mGroundTruth)
                        std::cout << "(mean,var) = " << acc::extract::mean(mErrorPose) <<
                           " \t" <<  acc::extract::variance(mErrorPose) << 
                           " \t" <<  acc::extract::mean(mErrorOri) << 
                           " \t" <<  acc::extract::variance(mErrorOri) << std::endl;
                    exit(0);
                }
                else
                    return;
            }
            tVectorD *lExpectedPosePtr = NULL;
            if(mGroundTruth)
                lExpectedPosePtr = &(*(mGroundTruthItr++));
            if(mHandClass.computeNextFrame(mImage,lExpectedPosePtr,mWorsen)!=UNDEFGRASP)
            {
                if(mGroundTruth)
                {
                    mErrorOri(lExpectedPosePtr->back());
                    lExpectedPosePtr->pop_back();
                    mErrorPose(lExpectedPosePtr->back());
                    lExpectedPosePtr->pop_back();
                }
                mHandClass.getTracker().drawTrackingPic(mTrackingIm);
                mHandClass.getFeat().draw(mHogIm);
                //mHandClass.getHog().drawHog(mHogIm);
                mHandClass.getProcessHog()->getPoselist()->montage(1,1,mMontageIm);
                //mHandClass.getProcessHog()->getPoselist()->montage(9,3,mMontageIm);
                mHandClass.getProcessHog()->getPoselist()->getBestPose().drawHogImage(mBestHogIm);
                std::cout << mHandClass << std::endl;
            }
            mHaveImages=false;
            Refresh();
        }

        /** If there are images, it shows them
        */
        virtual  void DoExpose(buola::CGC *pGC)
        {
            DrawImage(view(mTrackingIm),buola::CPoint(0,0),buola::CSize(640,480));
            DrawImage(mHandClass.getSegmView(),buola::CPoint(640,0),buola::CSize(320,240));
            DrawImage(view(mHogIm),buola::CPoint(640,240),buola::CSize(320,240));
            DrawImage(view(mMontageIm),buola::CPoint(960,0),buola::CSize(320,240));
            DrawImage(view(mBestHogIm),buola::CPoint(960,240),buola::CSize(320,240));
        }

        /** Menu for running step-by-step or continuously
        */
        virtual  void DoFillMenu(buola::gui::PMenu pMenu)
        {
            Maximize();
            pMenu->Add(buola::gui::new_menu_item(L"next image",ICON_STOCK_NEXT))->
                eTriggered.connect(MEMBER_THIS(OnNext,1));
            pMenu->Add(buola::gui::new_menu_item(L"toggle step",ICON_STOCK_RUN))->
                eTriggered.connect(MEMBER_THIS(OnRun));
        }

        /** Process next image (only in step-by-step mode)
        */
        void OnNext(int pStep)
        {
            mHaveImages=true;
        }

        /** Toggles step-by-step
        */
        void OnRun()
        {
            mStep=!mStep;
            mHaveImages=true;
        }

    protected:

#ifdef CAM_ACCESS
        CCorbaCamClient mCam;
#endif
        unsigned int mWorsen;
        tAcc_MeanVar mErrorPose;        /**< Error accumulator if ground truth provided*/
        tAcc_MeanVar mErrorOri;        /**< Error accumulator if ground truth provided*/
        boost::gil::rgb8_image_t mImage;                 /**< Input Image*/
        boost::gil::rgb8_image_t mTrackingIm;            /**< Input Image with tracked object overlayed*/
        boost::gil::gray8_image_t mHogIm;                /**< Hog Representation of input segmented image*/
        boost::gil::rgb8_image_t mMontageIm;             /**< Nearest Neighbors image*/
        boost::gil::gray8_image_t mBestHogIm;            /**< Best NN Hog*/
        std::vector<buola::TString> mFiles;              /**< Image Paths (offline)*/
        std::vector<tVectorD> mGroundTruthPoses;  /**< Image Paths (offline)*/
        std::vector<buola::TString>::iterator mFilesItr; /**< Current Image iterator (offline)*/
        std::vector<tVectorD>::iterator mGroundTruthItr; /**< Current Image iterator (offline)*/
        bool mOfflineImages,mHaveImages,mStep,mGroundTruth;    /**< Flags for offline, image ready, step-by-step and ground truth available*/
        PL mPoselist;                             /**< Poselist containing best poses */
        HandClass<PL,tType>  mHandClass;                /**< HandClass that process the frames*/
        ProcessHog<PL,tType> mProcessHog;               /**< ProcessHog (given a hog output a grasp class)*/
};

/** @brief CTestGUI2 specialization for non-PF poselists 
*/
template<typename PL, typename Feat, typename NN,typename tType>
class handTrackerGUI : public CTestGUI2<PL,tType>
{
    public:
        // with GUI, we should have a working default constructor, not like in NOGUI
        handTrackerGUI():
            CTestGUI2<PL,tType>(&mNN,(Feature<tType>*) &mFeat), mNN(500,106920,512,"data.bin"){}
    private:
        NN mNN;
        Feat mFeat;
};

/** @brief CTestGUI2 specialization for PF poselists 
*/
template<typename Feat,typename NN,typename tType>
class handTrackerGUI<Poselist_PF,Feat,NN,tType> : public CTestGUI2<Poselist_PF,tType>
{
    public:
        handTrackerGUI():
            CTestGUI2<Poselist_PF,tType>((Feature<tType>*) &mFeat){}
    private:
        Feat mFeat;
};

#endif
