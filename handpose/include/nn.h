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
