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
#include <hash_map>

using namespace std;
using namespace hdmap_proto;
using namespace __gnu_cxx;
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

struct SlopeInfo
{
    bool m_bValid; // false指无效，返回默认值，true有效
    uint  m_flag;
    double m_SlopeValue;
    Vector3d m_pos;
    Vector3d m_normal;
    SlopeInfo(){m_bValid = false;}
};

class mapCommonFunc
{
public:
   static  vector<Section*> GetAllSection(Map* pMap);
   static  vector<Zone*>    getAllZoneObjs(Map* pMap);
    
   static  int              GetSectionID(Section* pSection);
   static  int              GetSectionPreID(Section* pSection);
   static  int              GetSectionSucID(Section* pSection);
   static  vector<int>      GetRelatedIDs(Zone* pZone,bool bPreOrSuc); //true means pre;false means suc
   static  Lane*            getRefLineSet(Section* pSection); //One
   static  vector<Vector3d> getPtsFromLineSet(Lane* pLane);
   static  double           getSectionDistance(Section* pSection);
   
    //根据ID进行查询
   static  Section*         GetSectionFromID(Map* pMap,const int& nID);
   static  Zone*            GetZoneFromID(Map* pMap,const int& nID);
   static  Obstacle*        GetObstacleFromID(Map* pMap,const int& nID);
   static  SemanticPoint*   GetSemanticPointFromID(Map* pMap,const int& nID);
   static  int              getTargetIndex(Map* pMap,mapType eType,const int& nID);//内部实现
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
    
    //坡度信息查询
    SlopeInfo  getSlopeInfo(Vector3d curPos);
    
protected:
    void clearTree();
    void getLaneBBox(const Lane& lane,double* min,double* max);
    void getPolygonBBox(const Polygon& polygon,double* min,double* max);
private:
   const Map* m_MapData;
   RTree<GeoInfo*,double,3>  m_mapRtree;
   vector<GeoInfo*> m_vecTempDatas;//用于释放内存
   hash_map<long int,int> m_SlopeKey2Index; //存储SlopeKey与index映射
   double m_dSlopePixSize;
   Vector3d m_low,m_high;
};
   
}  // namespace hdmap_kq_proto
#endif  // MAPIO_H_
