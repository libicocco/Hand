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
