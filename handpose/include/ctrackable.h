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
#ifndef _CTRACKABLE_H_
#define _CTRACKABLE_H_
#include <opencv2/core/core.hpp>
#include <iostream>

/** 
 * @brief Subtract two points
 * 
 * @param s1 First Point
 * @param s2 Second Point
 * 
 * @return Difference between the points
 */
  template <typename CAnyPoint>
CAnyPoint subtract(const CAnyPoint &s1, const CAnyPoint &s2)
{
  CAnyPoint lTmp = s1;
  lTmp -= s2;
  return lTmp; 
}

/** 
 * @brief Subtract two OpenCV rectangles
 * 
 * @param s1 First Rectangle
 * @param s2 Second Rectangle
 * 
 * @return Subtraction of rectangle centers
 */
    template <typename CAnyPoint>
CAnyPoint subtract(const cv::Rect &s1, const cv::Rect &s2)
{
	return CAnyPoint((s1.x+(s1.width/2))-(s2.x+(s2.width/2)),
											 (s1.y+(s1.height/2))-(s2.y+(s2.height/2)));
}


/** 
 * @brief Class that defines any object that can be tracked (Rectangle, Points...)
 */
template<typename CShape, typename CAnyPoint>
class CTrackable
{
  friend std::ostream& operator << ( std::ostream& output, const CTrackable& pT)
  //{return output;}
  {output << pT.mReal.x << "," << pT.mReal.y << "," << pT.mReal.width << "," << pT.mReal.height;return output;}
  public:
  
  /** 
   * @brief Default constructor
   */
  CTrackable():mReal(),mPredicted(),mVelocity(),mAge(TIMEOUT){;}

  /** 
   * @brief Constructor from a shape with predictions, velocity and age
   * 
   * @param pReal Real position
   * @param pPredicted Predicted position
   * @param pVelocity Velocity (difference between previous prediction and real position)
   * @param pAge Age
   */
  CTrackable(const CShape &pReal,const CShape &pPredicted,const CAnyPoint &pVelocity,const unsigned int pAge=0):mReal(pReal),mPredicted(pPredicted),mVelocity(pVelocity),mAge(pAge){;}

  void setReal(const CShape &pReal){mReal=pReal;}
  void setPred(const CShape &pPredicted){mPredicted=pPredicted;}
  void setVel(const CAnyPoint &pVelocity){mVelocity=pVelocity;}
  void setDead(){mAge=TIMEOUT;}
  void setNew(){mAge=0;}
  CShape getReal() const {return mReal;}
  CShape getPred() const {return mPredicted;}
  CShape getPrevPred() const 
  {
    CShape lPrevPred = mPredicted;
		lPrevPred.x -= mVelocity.x;
		lPrevPred.y -= mVelocity.y;
    return lPrevPred;
  }
  CAnyPoint getVel() const {return mVelocity;}
  unsigned int getAge() const {return mAge;}
  bool isNew() const {return mAge==0;}
  void updatePred()
  {
    mPredicted = mReal;
    mPredicted.x += mVelocity.x;
    mPredicted.y += mVelocity.y;
  }
  void updateVel()
  {
    CShape lPrevPred = getPrevPred();
    mVelocity = subtract<CAnyPoint>(mReal,lPrevPred);
  }
  void incAge(){mAge++;}
  bool isDead() const {return mAge>=TIMEOUT;}
  public:
  static const unsigned int TIMEOUT=1;
  private:
  CShape mReal;       /**< Actual position*/
  CShape mPredicted;  /**< Predicted position*/
  CAnyPoint mVelocity;/**< Diference between prev. prediction and mReal*/
  unsigned int mAge;  /**< Frames since it was created*/
};
#endif
