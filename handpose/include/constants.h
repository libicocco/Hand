/** 
 * @file File containing constants used in different classes 
 */
#ifndef CONSTANTSH
#define CONSTANTSH
#include <string>
#include <stdint.h>
static const int BUFFERSIZE 			= 200;

enum EError{LOADIMAGEERROR, LOADMASKERROR, NOHANDERROR, EMPTYPOSELISTERROR,
    NOTFOUNDDBERROR, NOTFOUNDINFOERROR, TREEFILEANNERROR,
    NANWEIGHTERROR, NOMATCHINGSIZEERROR, UNKNOWNGRASPERROR, WRONGSIZEFEAT};

//static const int NLEVELS    			= 4;
//static const int NBINS      			= 8;
//static const bool MULTILEVEL			= false;
//static const int NHISTS      			= MULTILEVEL?(((1<<(2*NLEVELS))-1)/3):(1<<(2*(NLEVELS-1)));
static const int NCELLS                 = 32; // same as NHISTS, but 8x8
static const int NPOSES    				= 3240;
static const int NREALGRASPS 		    = 33;
static const int NILUS 					= 1;
static const int NGRASPS   				= NREALGRASPS*NILUS;
static const int NVIEWS         = 648;
//static const int NNN        			= 5;

static const int     UNDEFGRASP 			= 11111;
static const int     UNDEFWEIGHT 		= -999;
static const std::string UNDEFPATH 		= "";
static const int    UNDEFINDEX 			= -1;
static const int    UNDEFDIST 			= -999;

static const double PI					= 3.14159265;
static const double PHI					= 1.61803399;
static const double DEG2RAD 			= 3.14159/180.0;

static const uint32_t CELLSIZE         = 100;
static const uint32_t LINESIZE         = CELLSIZE*0.9;
static const size_t NO_ORI             = (size_t)-1;

static const uint32_t NPARTICLES      = 1000;
static const uint32_t NUNIFORMPARTICLES = 200;
static const uint32_t NSUCESSORS      = 5;
static const uint32_t NSUCESSORS_FILE = 5;
static const uint32_t MAXINFLUENCENN  = 20;

static const uint32_t MAXDISTTRACKER = 200; 
static const uint32_t INVPROBDEFAULT = 999999;

#endif
