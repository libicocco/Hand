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
#include <sqlite3.h>
#include <iterator>
#include <fstream>
#include <sstream>
#include <iostream>
#include <vector>
#include <string>
#include <stdlib.h>
#include <algorithm>
#include "boost/filesystem.hpp"
#include "cDB.h"
#include "cDBelement.h"
#include "cDBregisterQuery.h"
#include "utils.h"

namespace fsystem=boost::filesystem;

static const unsigned gBufferSz=1024;


void CDB::insertElement(const CDBelement &pElem)
{
  CDBregisterInsert lRegister(mStatement);
  pElem.insert(lRegister);
}

CDBelement CDB::query(const unsigned pIndex)
{
  if(!mReady4Query)
  {
    std::cerr << "The db is not ready for query; exiting" << std::endl;
    exit(-1);
  }
  CDBregisterQuery lRegister(mStatement,pIndex); 
  CDBelement lElement(lRegister);
  return lElement;
}

CDB::CDB(const fsystem::path &pDBPath):
mDB(),mReady4Query(false)
{
  openDB(pDBPath);
  prepareQuery();
}


CDB::CDB(const fsystem::path &pDBPath,const fsystem::path &pInfosFolder,const std::string &pInfoPattern):
mDB(),mReady4Query(false)
{
  char *lError=new char[256];
  openDB(pDBPath);
  createAndInsert();
  
  tPriorityPathQ lFound;
  std::cout << "finding info files" << std::endl;
  if(!find_file(pInfosFolder,pInfoPattern,lFound))
  {
    std::cerr << "No info files matching " << pInfoPattern << "found in " << pInfosFolder << std::endl;
    exit(-1);
  }
  std::cout << "Found " << lFound.size() << " entries" << std::endl;
  unsigned i=0;
  while(!lFound.empty())
  {
    fsystem::path lInfoPath=lFound.top();
    lFound.pop();
    CDBregisterInsert lRegister(mStatement);
    CDBelement lDBelem(lInfoPath,i);
    lDBelem.insert(lRegister);
    ++i;
  }
  std::cout << "entries inserted" << std::endl;
  delete []lError;
  
  finalizeStatement();
  prepareQuery();
}