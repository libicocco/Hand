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
#include "cDB.h"
#include "cDBelement.h"
#include "handclass_config.h"

int main(int argc,char* argv[])
{
  fsystem::path lTstDBPath(SCENEPATH);
  lTstDBPath/="taxonomy.db";
  fsystem::path lPosesPath(SCENEPATH);
  lPosesPath/="poses";
  CDB lTstDBNew(lTstDBPath,lPosesPath.string(),".*.txt");
}
