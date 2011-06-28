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
#ifndef __CDBREGISTERINSERT_H
#define __CDBREGISTERINSERT_H

#include <stdlib.h>
#include <iostream>
#include <string>
#include <sqlite3.h>

class CDBregisterInsert
{
public:
  CDBregisterInsert(sqlite3_stmt *pStatement):mStatement(pStatement){}
  void bindInt(const int pInt, const unsigned pIndex) const
  {
    int lRC=sqlite3_bind_int64(mStatement,pIndex,pInt);
    if(lRC!=SQLITE_OK)
    {
      std::cerr << "FAIL: SQL bind int index " << pIndex << " error: " << lRC << std::endl;
      exit(-1);
    }
  }
  
  void bindText(const std::string &pText, const unsigned pIndex) const
  {
    // strings can be destroyed before step is called, so let's copy them with TRANSIENT
    //     if(sqlite3_bind_text(mStatement,pIndex,pText.c_str(),-1,SQLITE_TRANSIENT)!=SQLITE_OK)
    // back to STATIC since the function was moved out of the destructor
    int lRC=sqlite3_bind_text(mStatement,pIndex,pText.c_str(),-1,SQLITE_STATIC);
    if(lRC!=SQLITE_OK)
    {
      std::cerr << "FAIL: SQL bind text index " << pIndex << " error; " << lRC << std::endl;
      exit(-1);
    }
  }
  void bindBlob(const void *pData, const unsigned pSize, const unsigned pIndex) const
  {
    int lRC=sqlite3_bind_blob(mStatement,pIndex,pData,pSize,SQLITE_STATIC);
    if(lRC!=SQLITE_OK)
    {
      std::cerr << "FAIL: SQL bind blob index " << pIndex << " error; " << lRC << std::endl;
      exit(-1);
    }
  }
  
  // moved from destructor to avoid depending on other strings
  void stepReset() const
  {
    int lRC=sqlite3_step(mStatement);
    if(lRC!=SQLITE_DONE)
    {
      std::cerr << "SQL step error " << lRC << std::endl;
      exit(-1);
    }
    if(sqlite3_reset(mStatement)!=SQLITE_OK)
    {
      std::cerr << "SQL reset error" << std::endl;
      exit(-1);
    }
  }
private:
  sqlite3_stmt *mStatement;
};

#endif
