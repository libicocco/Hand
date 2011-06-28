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
#ifndef APPROXNNH
#define APPROXNNH

#include <vector>
#include <fstream>
#include "ANN/ANN.h"
#include "nn.h"
#include "constants.h"
#include <stdlib.h>
#include <boost/progress.hpp>
#include <boost/format.hpp>

/** 
 * @brief Interface to the ANN library (http://www.cs.umd.edu/~mount/ANN/)
 */
template<typename tType>
class approxNN: public nn<tType>
{
  private:
    int maxPts;
    int k;
    ANNpointArray dataPts;
    ANNpoint      queryPt;
    ANNidxArray   nnIdx;
    ANNdistArray  dists;
    ANNkd_tree*   kdTree;
    bool initialized;
    const char* treePath;
    bool readTreeFromFile;
  private:
bool readPt(std::istream &in, ANNpoint p)			// read point (false on EOF)
{
	for (int i = 0; i < this->pointsDimension; i++) 
		if(!(in >> p[i])) return false;
	return true;
}
void printPt(std::ostream &out, ANNpoint p)			// print point
{
	out << "(" << p[0];
	for (int i = 1; i < this->pointsDimension; i++) {
		out << ", " << p[i];
	}
	out << ")\n";
}
  public:
    approxNN():maxPts(),k(),initialized(false),treePath("anntree.txt"),readTreeFromFile(true){initialize();}
//    approxNN(int pPointsDimension, int pNPoints, const char* pDataPath, int maxPts, int k,bool readTreeFromFile=false,const char *treePath="anntree.txt"):maxPts(maxPts),k(k),initialized(false),treePath(treePath),readTreeFromFile(readTreeFromFile)  {this->nn_set_values(pNPoints,pPointsDimension,pDataPath);initialize();}
    approxNN(int k,int pNPoints,int pPointsDimension, const char* pDataPath,const char *treePath="anntree.txt",bool readTreeFromFile=false):maxPts(pNPoints+1),k(k),initialized(false),treePath(treePath),readTreeFromFile(readTreeFromFile)  {this->nn_set_values(pNPoints,pPointsDimension,pDataPath);initialize();}
void set_values(int pPointsDimension, int pNPoints, const char* pDataPath, int maxPts, int k)
{
	this->maxPts = maxPts;
	this->k = k;
	this->nn_set_values(pNPoints,pPointsDimension,pDataPath);
}
~approxNN()
{
	if(initialized)
	{
		delete [] nnIdx;							// clean things up
		delete [] dists;
		delete kdTree;
		annClose();									// done with ANN
	}
}

bool initialize()
{
	nnIdx   = new ANNidx[k];				    // near neighbor indices
	dists   = new ANNdist[k];					  // near neighbor distances
    std::ifstream treePathStream;
	treePathStream.open(treePath,std::ios::in);
	if(readTreeFromFile && !treePathStream.fail())
	{
		kdTree = new ANNkd_tree(treePathStream);
	}
	else
	{
		dataPts = annAllocPts(maxPts, this->pointsDimension); // data points

        std::ifstream dataStream;					// data file stream
		dataStream.open(this->dataPath, std::ios::in);// open data file
        std::cout << this->dataPath <<  std::endl;
        std::cout << treePath <<  std::endl;
		if(dataStream.fail())
		{
			throw TREEFILEANNERROR;
			//cerr << "file " << this->dataPath << " couldn't be opened!" << std::endl;
			//exit(1);
		}

		//std::cout << "Data Points:\n";
		int i=0;
		while (i < maxPts && readPt(dataStream, dataPts[i])) {
			//printPt(std::cout, dataPts[i]);
			i++;
		}
		//std::cout << "Data points read" << std::endl;
		kdTree = new ANNkd_tree(					// build search structure
				dataPts,					// the data points
				this->nPoints,						// number of points
				this->pointsDimension);						// dimension of space
		//std::cout << "Tree created" << std::endl;
        std::ofstream treePathStream;
		treePathStream.open(treePath,std::ios::out);
		kdTree->Dump((ANNbool)true,treePathStream);
		treePathStream.close();
		dataStream.close();
	}
	initialized = true;
    return true;
}
const std::vector<std::pair<int,float> >& computeNN(const std::vector<tType>& pFeat)
{
#ifndef NDEBUG
	boost::progress_timer t;
  t.restart();
#endif
	this->nndist.clear();
	queryPt = annAllocPt(this->pointsDimension);				  // query point
    typename std::vector<tType>::const_iterator itrfeat;
	int i;
	for(i=0,itrfeat=pFeat.begin();i<this->pointsDimension;++i,++itrfeat)
		queryPt[i] = *itrfeat;
	//kdTree->Print((ANNbool) false, std::cout);
	kdTree->annkSearch(queryPt, k, nnIdx, dists, 0);
	/*kdTree->annkSearch(						// search
	  queryPt,						// query point
	  k,								// number of near neighbors
	  nnIdx,							// nearest neighbors (returned)
	  dists,							// distance (returned)
	  0);							// error bound
	  */
	//std::cout << "\tNN:\tIndex\tDistance\n";
	//std::cout << k << std::endl;
	for (i = 0; i < k; i++) {			// print summary
		dists[i] = sqrt(dists[i]);			// unsquare distance
        std::pair<int,float> nn(nnIdx[i],dists[i]);
		this->nndist.push_back(nn);
		//std::cout << "\t" << nnIdx[i] << "\t" << dists[i];
	}
#ifndef NDEBUG
    std::cout << boost::format("COMPUTENN TIME: %1%s.") % t.elapsed() << std::endl;
#endif
	return this->nndist;
	//std::cout << std::endl;
}
};

#endif
