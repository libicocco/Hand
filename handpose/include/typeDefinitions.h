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
#include <vector>
#include <queue>
#include <Eigen/Dense>
#include <boost/filesystem.hpp>

#include <boost/accumulators/numeric/functional/vector.hpp>
#include <boost/accumulators/statistics/stats.hpp>
#include <boost/accumulators/framework/features.hpp>
#include <boost/accumulators/statistics/variance.hpp>
#include <boost/accumulators/accumulators.hpp>
#include <boost/accumulators/statistics/weighted_sum.hpp>
#include <boost/accumulators/statistics/weighted_mean.hpp>
#include <boost/accumulators/statistics/weighted_variance.hpp>
#include <boost/accumulators/statistics/min.hpp>
#include <boost/accumulators/statistics/max.hpp>

#include <opencv2/core/core.hpp>

#include "constants.h"


namespace fsystem=boost::filesystem;
namespace acc=boost::accumulators;

typedef std::priority_queue<fsystem::path,std::vector<fsystem::path>,std::greater<fsystem::path>> tPriorityPathQ;

typedef std::pair<cv::Mat,cv::Mat> tMatMat;
typedef cv::Rect tRect;
typedef cv::Point tPoint;

typedef std::vector<double> tVectorD;
typedef std::pair<unsigned,double> tUnsDouble;
typedef std::vector< tUnsDouble > tPairV;

typedef acc::accumulator_set< float, acc::stats<acc::tag::mean,acc::tag::variance> > tAcc_MeanVar;
typedef acc::accumulator_set< std::vector<double>,
	acc::stats<acc::tag::weighted_mean,acc::tag::weighted_variance >, double > tAccV_MeanVar;
typedef acc::accumulator_set< double, acc::stats<acc::tag::variance> > tAcc_Var;
typedef acc::accumulator_set< double, acc::stats<acc::tag::max> > tAcc_Max;
typedef acc::accumulator_set< std::vector<double>, acc::stats<acc::tag::weighted_variance >, double > tAccV_Var;
typedef acc::accumulator_set< double, acc::stats<acc::tag::min,acc::tag::variance> > tAcc_MinVar;
typedef acc::accumulator_set< double, acc::stats<acc::tag::sum> > tAcc_Sum;
