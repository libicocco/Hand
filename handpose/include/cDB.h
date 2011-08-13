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

#ifndef __CDB_H
#define __CDB_H

#include <vector>
#include <string>
#include <stdlib.h>
#include <sqlite3.h>
#include "boost/filesystem.hpp"
#include "constants.h"
#include "typeDefinitions.h"
#include "cDBelement.h"

namespace fsystem=boost::filesystem;

class CDB
{
public:
  
  /**
   * @brief Ctor which reads sqlite db from pDBPath
   *
   * @param pDBPath Path to existing database
   **/
  CDB(const fsystem::path &pDBPath);
  
  /**
   * @brief Ctor which creates sqlite db from info file (matching pInfoPattern) in pInfosFolder
   * and saves it in pDBPath
   *
   * @param pDBPath Path where the database will be created
   * @param pInfosFolder Path to folder holding the info files
   * @param pInfoPattern Pattern which info files match
   **/
  CDB(const fsystem::path &pDBPath,const fsystem::path &pInfosFolder,const std::string &pInfoPattern);
  
  /**
   * @brief Query the database for index and stores the results as class members
   *
   * @param index Index to query from the database
   **/
  ~CDB(){sqlite3_finalize(mStatement);sqlite3_close(mDB);}
  
  void openDB(const fsystem::path &pDBPath)
  {
    char *lError;
    int lRC;
    
    std::cout << "opening database" << std::endl;
    lRC=sqlite3_open(pDBPath.string().c_str(),&mDB);
    if(lRC!=SQLITE_OK)
    {
      std::cerr << "Problem opening database " << pDBPath.string() << " with error code " << lRC << std::endl;
      exit(-1);
    }
    
    lRC=sqlite3_exec(mDB,sPragmas,NULL,NULL,&lError);
    if(lRC!=SQLITE_OK)
    {
      std::cerr << "SQL exec error " << lError << " with error code " << lRC << std::endl;
      exit(-1);
    }
  }
  
  void createAndInsert()
  {
    char *lError;
    int lRC;
    std::cout << "executing create pragma" << std::endl;
    lRC=sqlite3_exec(mDB,sCreate,NULL,NULL,&lError);
    if(lRC!=SQLITE_OK)
    {
      std::cerr << "SQL error " << lError << " with error code " << lRC << std::endl;
      exit(-1);
    }
    std::cout << "preparing insert statement" << std::endl;
    lRC=sqlite3_prepare_v2(mDB,sWrite,-1,&mStatement,NULL);
    if(lRC!=SQLITE_OK)
    {
      std::cerr << "SQL prepare error " << lRC << std::endl;
      exit(-1);
    }
  }
  
  void finalizeStatement()
  {
    int lRC;
    lRC=sqlite3_finalize(mStatement);
    if(lRC!=SQLITE_OK)
    {
      std::cerr << "SQL finalize error " << lRC << std::endl;
      exit(-1);
    }
  }
  
  void prepareQuery()
  {
    int lRC=sqlite3_prepare_v2(mDB,sGetInfo,-1,&mStatement,NULL);
    // if OK, the db is ready to be queried
    if(lRC!=SQLITE_OK)
    {
      // if the db doesn't exist, the db is created and prepared to be filled
      if(lRC==SQLITE_ERROR) // missing db
      {
        std::cout << "database does not exist: creating a new one" << std::endl;
        createAndInsert();
        mReady4Query=false;
      }
      else
      {
       std::cerr << "Prepare error : " << lRC << std::endl;
       exit(-1);
      }
    }
    else
      mReady4Query=true;
  }
  
  CDBelement query(const unsigned index);
  void insertElement(const CDBelement &pElem);
private:
  sqlite3 *mDB;
  sqlite3_stmt *mStatement;
  bool mReady4Query;
  
};

#endif
