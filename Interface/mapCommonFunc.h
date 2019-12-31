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
#include "RTree.h"

using namespace std;
using namespace hdmap_proto;
namespace hdmap_kq_op {
    
enum mapType
{
    mt_UnKnown          = 0,
    mt_Section          = 1,
    mt_Lane             = 2,
    mt_Zone             = 3,
    mt_Obstacle         = 4,
    mt_SemanticPoint    = 5
};

struct GeoInfo
{
    mapType m_eType;
    int m_nSectionIndex;
    int m_nLaneIndex;
    int m_nZoneIndex;
    int m_nSegPtIndex;
    int m_nObstacleIndex;
    GeoInfo()
    {
        m_eType = mt_UnKnown;
        m_nSectionIndex = -1;
        m_nLaneIndex = -1;
        m_nZoneIndex = -1;
        m_nSegPtIndex = -1;
        m_nObstacleIndex = -1;
    }
};
    
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


class geoSpatialSearch
{
public:
    geoSpatialSearch(){}
    ~geoSpatialSearch(){clearTree();}

public:
    void InitialMap(const Map* pMap);
    void Search(Vector3d pt,double dRadius,vector<GeoInfo*>& results);
    void Search(Vector3d MinPt,Vector3d MaxPt,vector<GeoInfo*>& results);
    
protected:
    void clearTree();
    void getLaneBBox(const Lane& lane,double* min,double* max);
    void getPolygonBBox(const Polygon& polygon,double* min,double* max);
private:
   const Map* m_MapData;
   RTree<GeoInfo*,double,3>  m_mapRtree;
   vector<GeoInfo*> m_vecTempDatas;//用于释放内存
    
};
   
}  // namespace hdmap_kq_proto
#endif  // MAPIO_H_
