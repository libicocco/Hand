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
 * @file handclass.h
 * Contains class HandClass.
 * Calls the different functions involved in the frame process:
 * segmentation, tracking, hog computation, hog processing. * 
 *
 * @brief Calls the different functions in the frame process
 *
 * @author Javier Romero
 */
#ifndef _HANDCLASS_H_
#define _HANDCLASS_H_

#include <boost/format.hpp>
#include <boost/gil/extension/io/jpeg_io.hpp>
#include <buola/image.h>
#include <buola/cv/opencv.h>
#include <iostream>
#include <boost/progress.hpp>

#include "nn.h"
#include "hog.h"
#include "utils.h"
#include "pose.h"
#include "chandtracker.h"
#include "processHog.h"
#include "constants.h"
#include "handclass_config.h"

/** Compute grasp code, just 1of3 for visit to Karlsruhe purpose*/
unsigned int computeGraspCode(unsigned int pGraspClass, std::vector<double> pWPose)
{
#ifndef NDEBUG
    boost::progress_timer t;
#endif
    unsigned int lGraspCode;
//    double az = (pWPose[0])*DEG2RAD;
//    double el = (pWPose[1])*DEG2RAD;
//    double ro = (pWPose[2])*DEG2RAD;
//    double x = sin(ro)*cos(az) + sin(el)*cos(ro)*sin(az);
//    double y = sin(el)*cos(ro)*cos(az) - sin(ro)*sin(az);
//    double z = cos(ro)*cos(el);

#ifndef NDEBUG
    std::cout << "(" << pWPose[0] << "," << pWPose[1] << "," << pWPose[2]	<< "," << pWPose[3]	<< "," << pWPose[4]	<< "," << pWPose[5]	<< "," << pWPose[6]	<< "," << pWPose[7]	<< "," << pWPose[8]	<< ")" << std::endl;
#endif 
    std::cout <<  pWPose[7]	<<  std::endl;

    // now pWPose[0-8] has a rotation matrix; z is approximately the 8th component
    if(fabs(pWPose[7])>0.6)//roll bigger than 30 degrees
        lGraspCode = 2;
    else if(pGraspClass==1||pGraspClass==2||pGraspClass==3||pGraspClass==4||pGraspClass==5||pGraspClass==10||pGraspClass==11||pGraspClass==12||pGraspClass==13||pGraspClass==15||pGraspClass==18||pGraspClass==19||pGraspClass==20||pGraspClass==21||pGraspClass==22||pGraspClass==23||pGraspClass==26||pGraspClass==27||pGraspClass==28)//large diameter
        lGraspCode = 1;
    else if(pGraspClass==6||pGraspClass==7||pGraspClass==8||pGraspClass==9||pGraspClass==14||pGraspClass==16||pGraspClass==17||pGraspClass==24||pGraspClass==25||pGraspClass==29||pGraspClass==30||pGraspClass==31||pGraspClass==32||pGraspClass==33)//palmar pinch
        lGraspCode = 3;
    else
    {
        std::cerr << "Grasp code: " << lGraspCode << ", Grasp class: " << pGraspClass << std::endl;
        throw UNKNOWNGRASPERROR;
    }
#ifndef NDEBUG
    std::cout << boost::format("COMPUTENEXTFRAME TIME: %1%s.") % t.elapsed() << std::endl;
#endif
    return lGraspCode;
}

/** @brief Class that call the different steps conforming the frame process sequentially.
*/
template<typename PL,typename tType>
class HandClass
{
    /** Print operator: prints the poselist and the weighted orientation*/
    friend std::ostream& operator << ( std::ostream& pOutput, const HandClass& pHandclass)
    {
        pOutput << "Frame: " << pHandclass.mFrame << std::endl;
        pOutput << "Pose list is: " << *(pHandclass.mProcessHog->getPoselist()) << std::endl;
        std::vector<double>::const_iterator lItr;
        //pOutput << std::endl << "Weighted orientation: ";
        //for (lItr=(pHandclass.mWpose).begin();lItr!=(pHandclass.mWpose).end();++lItr)
        //    pOutput << "\t" <<  *lItr ;
        return pOutput;
    }
    public:

    //const size_t getFrameNumber const(){return mFrame;}

    /** 
     * @brief The constructor just fills the pointers with the parameters
     * 
     * @param pOutdir Path to the output folder
     * @param pProcessHog): ProcessHog instance pointer, created in cTestGUI2
     */
    HandClass(const char* pOutdir, ProcessHog<PL,tType>* pProcessHog, Feature<tType>* pFeat):
        mTracker(),mFeat(pFeat),mProcessHog(pProcessHog),mOutdir(pOutdir),mFrame(0),mWpose(){}

    /** 
     * @brief Function that calls internally computeNextFrame, but it accepts a path instead of a gil image 
     * 
     * @param pImagePath Path to the image to be processed
     * 
     * @return Grasp class
     */
    unsigned int computeNextFrame(char *pImagePath)
    {
        boost::gil::rgb8_image_t lImage;
        boost::gil::jpeg_read_image(pImagePath,lImage);
        return computeNextFrame(lImage);
    }
    /** 
     * @brief Main function of the class; Process the next frame given the input image
     * 
     * @param pImage Gil image to be processed
     * 
     * @return Grasp class
     */
    unsigned int computeNextFrame(boost::gil::rgb8_image_t &pImage, tVectorD *pExpectedPose=NULL, unsigned int pInvProb=INVPROBDEFAULT)
    {
#ifndef NDEBUG
        boost::progress_timer t;
#endif
//         boost::progress_timer t;
        mFrame++;
        try
        {
#ifndef NDEBUG
#endif
            char lCropName[50];
            char lCropMask[50];
            sprintf(lCropName,"%s/crop%04zu.jpg",mOutdir,mFrame);
            sprintf(lCropMask,"%s/mask%04zu.jpg",mOutdir,mFrame);
            std::pair<boost::gil::rgb8_view_t,boost::gil::gray8_view_t> lImMask = mTracker.getHand(pImage);
            mTracker.saveCroppedPics(lCropName,lCropMask);
            char lMaskName[50];
            sprintf(lMaskName,"%s/mask%04zu.jpg",mOutdir,mFrame);
//            mTracker.saveMask(lMaskName);
            mSegmImage.recreate((lImMask.first).dimensions());
            boost::gil::copy_and_convert_pixels(lImMask.first,view(mSegmImage));
//    std::cout << boost::format("GETHAND: %1%s.") % t.elapsed() << std::endl;

            std::vector<tType> lVHog = mFeat->compute(buola::cvi::ipl_wrap(view(mSegmImage)),buola::cvi::ipl_wrap(lImMask.second),pInvProb);
//             std::cout << "size of lVHog: " << lVHog.size() << std::endl;
//             std::cout << boost::format("GETHOG: %1%s.") % t.elapsed() << std::endl;
            //std::cout << mFeat << std::endl;
            //IplImage* a = mFeat->getGray();

            unsigned int lGraspClass = mProcessHog->UpdatePoselist(lVHog,mFrame,pExpectedPose);
            tVectorD lWPose = mProcessHog->getPoselist()->getWPose();
//             std::cout << boost::format("GETPOSE: %1%s.") % t.elapsed() << std::endl;
            //unsigned int lGraspCode = lGraspClass; // just because i don't care now
            unsigned int lGraspCode = computeGraspCode(lGraspClass,lWPose);

//            std::cout << *(mProcessHog->getPoselist()) << std::endl;
            
            
            std::ofstream outfile;
            outfile.open("out/logWPose", std::ios::app);
//             outfile << "Frame " << mFrame << ": ";
            //outfile << (mProcessHog->getPoselist()->getPoselist()->front()).getImagePath() << endl;
            tVectorD::iterator itrwpose;
            for(itrwpose=lWPose.begin();itrwpose<lWPose.end();++itrwpose)
                outfile << *itrwpose << " ";
            outfile << std::endl;
            outfile.close();

#ifndef NDEBUG
            std::cout << *(mProcessHog->getPoselist()) << std::endl;
            std::cout << *mFeat << std::endl;
            //std::cout << *mNearn << std::endl;
            std::cout << lGraspCode << std::endl;
#endif
//            IplImage* b = mFeat->getGray();
            std::cout << lGraspCode << " \t" << std::endl;
            return lGraspCode;
        }
        catch(EError e)
        {
            if(e==EMPTYPOSELISTERROR)
            {
                std::cerr << "Empty pose list " << std::endl;
                mWpose = std::vector<double>(0,0);
                //return mWpose;
                return UNDEFGRASP;
            }
            if(e==NOHANDERROR)
            {
                std::cout << "No hand found in the image " << std::endl;
                //std::cerr << "No hand found in the image " << std::endl;
                mWpose = std::vector<double>(0,0);
                //return mWpose;
                return UNDEFGRASP;
            }
            if(e==NOTFOUNDDBERROR || e==NOTFOUNDINFOERROR)
            {
                std::cerr << "Problems retrieving info from file|DB" << std::endl;
                exit(0);
            }
            if(e==TREEFILEANNERROR)
            {
                std::cerr << "Problems getting a dataStream in ANN" << std::endl;
                exit(0);
            }
            if(e==NANWEIGHTERROR)
            {
                std::cerr << "Got a nan weight in poselist creation" << std::endl;
                exit(0);
            }
            if(e==NOMATCHINGSIZEERROR)
            {
                std::cerr << "Two vectors of different sizes were compared" << std::endl;
                exit(0);
            }
            if(e==UNKNOWNGRASPERROR)
            {
                std::cerr << "Grasp code is not one of the two expected" << std::endl;
                exit(0);
            }
        }
        return -1; // can it reach this point?
    }
    const boost::gil::gray8_view_t & getSegmView(){return view(mSegmImage);}
    const Feature<tType> & getFeat() const{return *mFeat;}
    const CHandTracker & getTracker()const {return mTracker;}
    const ProcessHog<PL,tType> * getProcessHog()const {return mProcessHog;}

    private:
    CHandTracker mTracker;      /**< Tracker instance*/
    Feature<tType>* mFeat;                   /**< Feature instance*/
    //Hog mFeat;                   /**< Hog instance*/
    ProcessHog<PL,tType>* mProcessHog;/**< ProcessHog instance (includes poselist and nn)*/
    const char* mOutdir;        /**< Path to ouput folder*/
    size_t mFrame;              /**< Frame number*/
    std::vector<double> mWpose; /**< Computed weighted pose*/
    boost::gil::gray8_image_t mSegmImage;/**< Segmented image*/
};
#endif
