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
