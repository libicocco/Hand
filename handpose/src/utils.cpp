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