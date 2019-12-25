// @author Duanshuai (duanshuai2013@163.com)
// @brief test
//
// mapio.h

#ifndef MAP_KQ_COMMON_FUNC_H_
#define MAP_KQ_COMMON_FUNC_H_
#include "../proto/map.pb.h"

#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <cstdio>
#include <fstream>
#include <iostream>
#include "geoAlgorithmFunc.h"

using namespace std;
using namespace hdmap_proto;
namespace hdmap_kq_op {
    
class mapCommonFunc
{
public:
   static  vector<Section*> GetAllSection(Map* pMap);
   static  vector<Zone*> getAllZoneObjs(Map* pMap);
    
   static  int  GetSectionID(Section* pSection);
   static  int  GetSectionPreID(Section* pSection);
   static  int  GetSectionSucID(Section* pSection);
    
   static  vector<int> GetRelatedIDs(Zone* pZone,bool bPreOrSuc); //true means pre;false means suc
   static  Lane* getRefLineSet(Section* pSection); //One
   static  vector<Vector3d> getPtsFromLineSet(Lane* pLane);
   static  double getSectionDistance(Section* pSection);
};


}  // namespace hdmap_kq_proto
#endif  // MAPIO_H_
