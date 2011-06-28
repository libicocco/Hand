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
#include <boost/program_options.hpp>
#include "cPoseEstimator.h"
#include "cPoselistMulti.h"
#include "processFeat.h"
#include "feature.h"
#include "hog.h"
#include "nn.h"
#include "exactNN.h"
#include "approxNNflann.h"

namespace po = boost::program_options;
int main(int argc,char* argv[])
{
  std::string lDBPath,lFeatPath,lTestFolder,lTestPattern,lMetricPath;
  unsigned lNNN,lDBsize,lFeatDim,lDimMetric;
  bool lTemporalSmoothing;
  // Declare the supported options.
  po::options_description desc("Allowed options");
  desc.add_options()
    ("help,h", "produce help message")
    ("dbpath,d", po::value<std::string>(&lDBPath)->default_value("/home/javier/hand/scene/hands.db"),"Path to db file")
    ("featpath,f", po::value<std::string>(&lFeatPath)->default_value("/home/javier/hand/scene/hog.txt"),"Path to feature binary file")
    ("testfolder,t", po::value<std::string>(&lTestFolder)->default_value("/home/javier/synthetic_test/test1/"),"Path to folder with test images")
    ("testpattern,p", po::value<std::string>(&lTestPattern)->default_value(".*.png"),"Test files common pattern")
    ("nnn,n", po::value<unsigned>(&lNNN)->default_value(20), "Number of NN to show")
    ("dbn", po::value<unsigned>(&lDBsize)->default_value(155000), "Number of db elements")
    ("fn", po::value<unsigned>(&lFeatDim)->default_value(512), "Dimensionality of the features")
    ("temporal", po::value<bool>(&lTemporalSmoothing)->default_value(true), "Should the temporal likelihood be used?")
    ("dimMetric", po::value<unsigned>(&lDimMetric)->default_value(0), "Dimensionality of the joint metric used")
    ("metric", po::value<std::string>(&lMetricPath)->default_value(""), "Matrix with features for different joint configurations (nnn x dimMetric)")
    ;
  
  po::variables_map vm;
  po::store(po::parse_command_line(argc, argv, desc), vm);
  po::notify(vm);    
  
  if (vm.count("help")) {
      std::cout << desc << "\n";
    return 1;
  }
  CPoselistMulti lPList(lDBPath,lTemporalSmoothing,lDimMetric,lMetricPath);
  exactNN<float> lNN(lNNN,lDBsize,lFeatDim,lFeatPath.c_str());
  //exactNN<float> lNN(20,106920,512,"/home/javier/handDB_5stages/log_4_8_5stages.HOGnew.bin");
  //approxNNflann<float> lNN(20,106920,512,"/home/javier/handDB_5stages/log_4_8_5stages.HOGnew.bin",
  // 	"/home/javier/handDB_5stages/HOGnew_flann_9_1.index");
  nn<float> *lNNptr=&lNN;
  ProcessFeat<CPoselistMulti,float> lProcFeat(lNNptr,&lPList);
  Hog<float> lHog;
  Feature<float> *lFeat=&lHog;
  
  CPoseEstimator<CPoselistMulti,float> lTrackerHogExact(lFeat,&lProcFeat,lTestFolder,lTestPattern,true);
  lTrackerHogExact.Run();
}
