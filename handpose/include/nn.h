#ifndef NNH
#define NNH

#include <list>
#include <vector>
#include <iostream>
#include "typeDefinitions.h"

bool comppair (std::pair<int,float> p1,std::pair<int,float> p2) { return (p1.second<p2.second); }
template<typename tType>

class nn
{
	friend inline std::ostream& operator << ( std::ostream& output, const nn<tType>& nn)
	{
		const std::vector<std::pair<int,float> >& nndist = nn.getNNDist();
		std::vector<std::pair<int,float> >::const_iterator pairItr;
		for (pairItr=nndist.begin();pairItr!=nndist.end();++pairItr)
			output << "\t" << "(" << (*pairItr).first << "," << (*pairItr).second << ")";
		output << std::endl;
		return output;
	}
protected:
	int           nPoints;
	int           pointsDimension;
	const char*         dataPath;
	tPairV nndist;
public:
	void nn_set_values(int a, int b, const char* c){nPoints=a;pointsDimension=b;dataPath=c;}
	virtual bool initialize()=0;
	virtual const tPairV& computeNN(const std::vector<tType>& pFeat)=0;
	const tPairV& getNNDist() const {return nndist;}
	virtual ~nn(){}
};

#endif
