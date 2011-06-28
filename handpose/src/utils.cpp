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
#include "opencv2/core/core_c.h"
#include "opencv2/imgproc/imgproc_c.h"
#include <queue>
#include <math.h>
#include <iostream>
#include <boost/progress.hpp>
#include "utils.h"
#include "boost/regex.hpp"
#include "boost/filesystem.hpp"

namespace fsystem=boost::filesystem;

bool find_file(const fsystem::path & dir_path,         // in this directory,
							const std::string & file_name,           // search for this name,
							tPriorityPathQ &paths_found)// placing paths here if found
// 							std::vector<fsystem::path> &paths_found)// placing paths here if found
{
	if(!fsystem::exists(dir_path)) return false;
	fsystem::directory_iterator end_itr; // default construction yields past-the-end
	const boost::regex my_filter(file_name.c_str());
	boost::smatch what;
	bool found(false);
	for(fsystem::directory_iterator itr(dir_path);itr!=end_itr;++itr)
	{
		if(fsystem::is_directory(itr->status()))
		{
			if(find_file(itr->path(),file_name,paths_found)) found=true;
		}
		else
		{
			if(boost::regex_match(itr->path().filename().string(),what,my_filter))
			{
				paths_found.push(itr->path());
				found=true;
			}
		}
	}
	return found;
}