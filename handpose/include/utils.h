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
#ifndef UTILSH
#define UTILSH

#include <list>
#include <vector>
#include <string>
#include <queue>
#include "opencv2/core/core.hpp"
#include "ctrackable.h"
#include "constants.h"
#include "typeDefinitions.h"


/**
 * @brief Find files that match file_name regex recursively in folder dir_path
 *
 * @param dir_path Folder where we are searching
 * @param file_name Regex pattern the file/s should match
 * @param paths_found Out vector to be filled with matching file paths
 * @return True if file was found
 **/
bool find_file( const fsystem::path & dir_path,
        const std::string & file_name,
        tPriorityPathQ &paths_found );
//         std::vector<fsystem::path> &paths_found );

template <typename CAnyPoint>
void vpair2vpoint(const std::vector<std::pair<int,int> > &pVPair, std::vector<CAnyPoint> &pVPoint)
{
  pVPoint.clear();
  std::vector<std::pair<int,int> >::const_iterator itrin;
  for(itrin=pVPair.begin();itrin!=pVPair.end();++itrin)
  {
    CAnyPoint lTmp;
    lTmp.Set(itrin->first,itrin->second);
    pVPoint.push_back(lTmp);
  }
}

template <typename CAnyPoint>
void rejectSmallBlobs(std::vector<int> &pBlobSizes, std::vector<CAnyPoint> &pBlobCenters, const int pSizeTh)
{
  if(pBlobSizes.size()!=pBlobCenters.size())
	  throw NOMATCHINGSIZEERROR;
  std::vector<int>::iterator lItrSizes = pBlobSizes.begin();
  typename std::vector<CAnyPoint >::iterator lItrCenters = pBlobCenters.begin();
  for(int i=pBlobSizes.size()-1;i>=0;--i)
  {
    if(pBlobSizes[i] < pSizeTh)
    {
      pBlobSizes.erase(lItrSizes+i);
      pBlobCenters.erase(lItrCenters+i);
    }
  }
}

#endif
