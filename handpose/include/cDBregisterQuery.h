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
#ifndef __CDBREGISTERQUERY_H
#define __CDBREGISTERQUERY_H

#include <stdlib.h>
#include <iostream>
#include <string>
#include <sqlite3.h>

class CDBregisterQuery
{
public:
  CDBregisterQuery(sqlite3_stmt *pStatement,unsigned pIndex):mStatement(pStatement),mIndex(pIndex)
  {
    if(sqlite3_bind_int64(mStatement,1,mIndex)!=SQLITE_OK)
    {
      std::cerr << "SQL bind error" << std::endl;
      exit(-1);
    }
    int lRC=sqlite3_step(mStatement);
    if(lRC!=SQLITE_ROW)
    {
      std::cerr << "SQL step error " << lRC << std::endl;
      exit(-1);
    }
  }
  ~CDBregisterQuery()
  {
    if(sqlite3_reset(mStatement)!=SQLITE_OK)
    {
      std::cerr << "SQL reset error" << std::endl;
      exit(-1);
    }
  }
  const char* getText (unsigned pColumn) const {return reinterpret_cast<const char*>(sqlite3_column_text(mStatement,pColumn));}
  int getInt(unsigned pColumn) const{return sqlite3_column_int64(mStatement,pColumn);}
  const void* getBlob(unsigned pColumn) const{return sqlite3_column_blob(mStatement,pColumn);}
  int getSize(unsigned pColumn) const{return sqlite3_column_bytes(mStatement,pColumn);}
  unsigned getIndex() const{return mIndex;}
private:
  sqlite3_stmt *mStatement;
  unsigned mIndex;
};

#endif
