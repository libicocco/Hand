#ifndef APPROXNNLSHKITH
#define APPROXNNLSHKITH

#include <lshkit.h>
#include <boost/timer.hpp>
#include <boost/format.hpp>
#include <vector>
#include <iostream>
#include <fstream>
#include <gsl/gsl_multifit.h>
#include "nn.h"
#include "handclass_config.h"
#include "createLSHindex.h"

using namespace lshkit;
typedef Tail<RepeatHash<ThresholdingLsh> > MyLsh;
typedef MultiProbeLshIndex<unsigned> Index;

/** 
 * @brief Interface to library lshkit (http://lshkit.sourceforge.net/)
 */
template<typename tType>
class approxNNlshkit: public nn<tType>
{
private:
	bool mInitialized;
	const char* mDataPath;
	const char* mIndexPath;
	int mK;
	Index *mIndex;
	FloatMatrix *data;
	int mNBins;
	int mNTables;
	int mNPoints;
	static const float mR = 3.40282347e+38; 
	static const float mDesiredRecall = 0.8; 
public:
	// pNPoints and pDimPoints are dummy parameters
	// given for keeping a standard interface
	approxNNlshkit(int pK,int pNPoints=106920,int pDimPoints=512,
								 const char *pDataPath=LSHBINPATH,const char *pIndexPath=LSHINDEXPATH,
								 const int pNBins=50,const int pNTables=10):
								 mInitialized(false),mDataPath(pDataPath),mIndexPath(pIndexPath),mK(pK),
								 mNBins(pNBins),mNTables(pNTables),
								 mNPoints(pNPoints){initialize();} /**< @todo data.bin and data.idx should be configured with cmake*/
	~approxNNlshkit(){delete mIndex;delete data;}
	
	bool initialize()
	{
		#ifndef NDEBUG
		std::cout << "LOADING DATA..." << std::endl;
		boost::timer t;
		t.restart();
		#endif
		data = new FloatMatrix(mDataPath);
		#ifndef NDEBUG
		std::cout << boost::format("LOAD TIME: %1%s.") % t.elapsed() << std::endl;
		#endif
		
		mIndex = new Index;
		std::ifstream is(mIndexPath, std::ios_base::binary);
		if (is) {
			is.exceptions(std::ios_base::eofbit | std::ios_base::failbit | std::ios_base::badbit);
			#ifndef NDEBUG
			std::cout << "LOADING INDEX..." << std::endl;
			t.restart();
			#endif
			mIndex->load(is);
			BOOST_VERIFY(is);
			#ifndef NDEBUG
			std::cout << boost::format("LOAD TIME: %1%s.") % t.elapsed() << std::endl;
			#endif
		}
		else
		{
			#ifndef NDEBUG
			std::cout << "CREATING INDEX..." << std::endl;
			t.restart();
			#endif
			gsl_vector *vM = gsl_vector_alloc(3);
			gsl_vector *vG = gsl_vector_alloc(3);
			double gM = 0.0;
			double gG = 0.0;
			
			fitdata(mDataPath,gM,gG,vM,vG);
			int lM;
			double lW;
			mytune(mNBins,mNTables,mNPoints,gM,gG,vM,vG,lM,lW,mDesiredRecall,mK);
			run(mDataPath,mIndexPath,lW,lM,mNTables,mIndex);
			#ifndef NDEBUG
			std::cout << boost::format("LOAD TIME: %1%s.") % t.elapsed() << std::endl;
			#endif
		}
		return true;
	}// i had to do it in the constructor
	const tPairV& computeNN(const std::vector<tType>& pFeat)
	{
		#ifndef NDEBUG
		boost::timer t;
		t.restart();
		#endif
		this->nndist.clear();
		
// 		Topk<unsigned> topk;
		metric::l2sqr<float> l2sqr(data->getDim());
		FloatMatrix::Accessor accessor(*data);
		TopkScanner<FloatMatrix::Accessor, metric::l2sqr<float> > query(accessor, l2sqr, mK, mR);
		
		tType *lHogArray;
		lHogArray = new tType [pFeat.size()]; /**< @attention Copy of the hog each frame: can be improved*/
		std::copy( pFeat.begin(), pFeat.end(), lHogArray);
		{
// 			topk.reset(mK, mR);
// 			mIndex->query(lHogArray, &topk, mDesiredRecall,&cnt);
			query.reset(lHogArray);
			mIndex->query_recall(lHogArray, mDesiredRecall, query);
// 			cost << double(query.cnt())/double(data.getSize());
			for (unsigned k=0;k<mK;k++)
			{
				tUnsDouble nn(int(query.topk()[k].key),double(query.topk()[k].dist));
				this->nndist.push_back(nn);
				//std::cout << topk[k].key << "-" << topk[k].dist << " ";
			}
			//std::cout << std::endl;
		}
		#ifndef NDEBUG
		std::cout << boost::format("QUERY TIME: %1%s.") % t.elapsed() << std::endl;
		#endif
		delete [] lHogArray;
		return this->nndist;
	}
};

#endif

