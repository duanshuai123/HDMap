
#include "mapCommonFunc.h"
#include <float.h>
using std::ios;
using std::ios_base;

namespace hdmap_kq_op {

vector<Section*> mapCommonFunc::GetAllSection(Map* pMap)
{
    vector<Section*> results;
    if(pMap==0)
        return results;
    
    int nSize = pMap->sections_size();
    for(int i = 0;i < nSize;i++)
    {
        Section*  pSection = pMap->mutable_sections(i);
        results.push_back(pSection);
    }
    return results;
}

vector<Zone*> mapCommonFunc::getAllZoneObjs(Map* pMap)
{
    vector<Zone*> results;
    if(pMap==0)
        return results;
    
    int nSize = pMap->zones_size();
    for(int i = 0;i < nSize;i++)
    {
        Zone*  pZone = pMap->mutable_zones(i);
        results.push_back(pZone);
    }
    return results;
}

int  mapCommonFunc::GetSectionID(Section* pSection)
{
     if(pSection==0)
        return -1;
     
     bool bHas = pSection->has_id();
     if(!bHas)
         return -1;
     
      int nID = pSection->mutable_id()->id();
    
    return  nID;
}

int  mapCommonFunc::GetSectionPreID(Section* pSection)
{
    if(pSection==0)
        return -1;
    int nSize = pSection->pred_indices_size();
    if(nSize<1)
        return -1;

    return  pSection->pred_indices(0);
}

int  mapCommonFunc::GetSectionSucID(Section* pSection)
{
    if(pSection==0)
        return -1;
    int nSize = pSection->succ_indices_size();
    if(nSize<1)
        return -1;

    return  pSection->succ_indices(0);
}
    
vector<int> mapCommonFunc::GetRelatedIDs(Zone* pZone,bool bPreOrSuc)
{
    vector<int> results;
    if(pZone==0)
        return results;
    if(bPreOrSuc)
    {
        int nPreSize = pZone->link_pre_ids_size();
        for(int i = 0;i<nPreSize;i++)
        {
            int nID = pZone->mutable_link_pre_ids(i)->id();
            results.push_back(nID);
        }
    }
    else 
    {
        int nSucSize = pZone->link_suc_ids_size();
        for(int i = 0;i<nSucSize;i++)
        {
            int nID = pZone->mutable_link_suc_ids(i)->id();
            results.push_back(nID);
        }
    }
    
    return results;
    
}

Lane* mapCommonFunc::getRefLineSet(Section* pSection)
{
    if(pSection==0)
        return 0;
    int nSize = pSection->lanes_size();
    if(nSize<1)
        return 0;

    return pSection->mutable_lanes(0);
}

vector<Vector3d> mapCommonFunc::getPtsFromLineSet(Lane* pLane)
{
    vector<Vector3d> results;
    if(pLane==0)
        return results;
    
    int nSize = pLane->lines_size();
    for(int i = 0; i < nSize;i++)
    {
        CurveLine*  pCurveLine = pLane->mutable_lines(i);
        int nPtSize = pCurveLine->points_size();
        if(nPtSize<1)
            continue;

        for(int j = 0; j < nPtSize; j++)
        {
            hdmap_proto::Vector3d protoPt = pCurveLine->points(j);
            results.push_back(protoPt);
        }
    }
    return results;
}

double mapCommonFunc::getSectionDistance(Section* pSection)
{
    Lane* pLane = getRefLineSet(pSection);
    vector<Vector3d> vecPts = getPtsFromLineSet(pLane);
    return geoAlgorithm::DisTance(vecPts);
}

//---------------------------------------------------------
Section*  mapCommonFunc::GetSectionFromID(Map* pMap,const int& nID)
{
    int nIndex = getTargetIndex(pMap,mt_Section,nID);
    if(nIndex<0)
        return nullptr;
    
    return  pMap->mutable_sections(nIndex);
}

Zone* mapCommonFunc::GetZoneFromID(Map* pMap,const int& nID)
{
    int nIndex = getTargetIndex(pMap,mt_Zone,nID);
    if(nIndex<0)
        return nullptr;
    
    return  pMap->mutable_zones(nIndex);
}

Obstacle*  mapCommonFunc::GetObstacleFromID(Map* pMap,const int& nID)
{
    int nIndex = getTargetIndex(pMap,mt_Obstacle,nID);
    if(nIndex<0)
        return nullptr;
    
    return  pMap->mutable_obstacles(nIndex);
}

SemanticPoint*   mapCommonFunc::GetSemanticPointFromID(Map* pMap,const int& nID)
{
    int nIndex = getTargetIndex(pMap,mt_SemanticPoint,nID);
    if(nIndex<0)
        return nullptr;
    
    return  pMap->mutable_segpoint(nIndex);
}

int  mapCommonFunc::getTargetIndex(Map* pMap,mapType eType,const int& nID)
{
    int  nSize = -1;
    if(eType == mt_Section)
    {
        nSize = pMap->sections_size(); //0 ~ nSize-1
    }
    else if(eType == mt_Zone)
    {
        nSize = pMap->zones_size(); //0 ~ nSize-1
    }
    else if(eType == mt_Obstacle)
    {
        nSize = pMap->obstacles_size(); //0 ~ nSize-1
    }
    else if(eType == mt_SemanticPoint)
    {
        nSize = pMap->segpoint_size(); //0 ~ nSize-1
    }
    
    int low = 0,high = nSize-1, mid = 0;
    while(low <= high)
    {
        mid=(low+high)/2;
        int nTempID = -1;
        if(eType== mt_Section)
        {
            Section section = pMap->sections(mid); //0 ~ nSize-1
            nTempID = section.id().id();
        }
        else if(eType == mt_Zone)
        {
            Zone zone = pMap->zones(mid); //0 ~ nSize-1
            nTempID = zone.id().id();
        }
        else if(eType == mt_Obstacle)
        {
            Obstacle obj = pMap->obstacles(mid);
            nTempID = obj.id().id(); //0 ~ nSize-1
        }
        else if(eType == mt_SemanticPoint)
        {
            SemanticPoint pt = pMap->segpoint(mid); //0 ~ nSize-1
            nTempID = pt.id().id(); //0 ~ nSize-1
        }
        
        if (nTempID == nID)
            return mid;
        
        if (nTempID < nID)
            low = mid+1;
        else
            high = mid-1;
    }
    
    return -1;
}

//---------------------------------------------------------
void geoSpatialSearch::clearTree()
{
    for(int i = 0;i < m_vecTempDatas.size();i++)
    {
        delete m_vecTempDatas[i];
    }
    m_vecTempDatas.clear();
    m_mapRtree.RemoveAll();//回归到初始化
}

void geoSpatialSearch::InitialMap(const Map* pMap) 
{
    m_MapData = pMap;
    clearTree();
    
    int nSectionSize = pMap->sections_size();
    for(int i = 0;i<nSectionSize;i++)
    {
        const Section section =  pMap->sections(i);
        int nLaneSize = section.lanes_size();
        for(int j = 0; j < nLaneSize; j++)
        {
           const Lane lane = section.lanes(j);
           double a_min[3],a_max[3];
           getLaneBBox(lane,a_min,a_max);
           GeoInfo* info = new GeoInfo();
           info->m_eType = mt_Lane;
           info->m_nLaneIndex = j;
           info->m_nSectionIndex = i;
           m_mapRtree.Insert(a_min,a_max,info);
           m_vecTempDatas.push_back(info);
        }
    }
    
    int nZoneSize = pMap->zones_size();
    for(int i = 0;i<nZoneSize;i++)
    {
        const Zone zone =  pMap->zones(i);
        Polygon polygon = zone.border();
        double a_min[3],a_max[3];
        getPolygonBBox(polygon,a_min,a_max);
        GeoInfo* info = new GeoInfo();
        info->m_eType = mt_Zone;
        info->m_nZoneIndex = i;
        m_mapRtree.Insert(a_min,a_max,info);
        m_vecTempDatas.push_back(info);
    }
    
    int nObstaclesSize = pMap->obstacles_size();
    for(int i = 0;i<nObstaclesSize;i++)
    {
        const Obstacle obs =  pMap->obstacles(i);
        Polygon polygon = obs.border();
        double a_min[3],a_max[3];
        getPolygonBBox(polygon,a_min,a_max);
        GeoInfo* info = new GeoInfo();
        info->m_eType = mt_Obstacle;
        info->m_nObstacleIndex = i;
        m_mapRtree.Insert(a_min,a_max,info);
        m_vecTempDatas.push_back(info);
    }
    
    int nSegPtSize = pMap->segpoint_size();
    for(int i = 0;i<nSegPtSize;i++)
    {
        const SemanticPoint obs =  pMap->segpoint(i);
        Vector3d pos = obs.pos();
        double a_min[3] = { pos.x(),pos.y(),pos.z() };
        double a_max[3] = { pos.x(),pos.y(),pos.z() };
        
        GeoInfo* info = new GeoInfo();
        info->m_eType = mt_SemanticPoint;
        info->m_nSegPtIndex = i;
        m_mapRtree.Insert(a_min,a_max,info);
        m_vecTempDatas.push_back(info);
    }
    //other ...
}

void geoSpatialSearch::Search(Vector3d pt,double dRadius,vector<GeoInfo*>& results)
{
    //查询区域向外拓展固定容差，防止漏选
    double ExtentRangMin[3] = { pt.x()-dRadius, pt.y()-dRadius, pt.z()-dRadius };
    double ExtentRangMax[3] = { pt.x()+dRadius, pt.y()+dRadius, pt.z()+dRadius };

    int nSize = m_mapRtree.Search(ExtentRangMin,ExtentRangMax,results);
    return ;
}

void geoSpatialSearch::Search(Vector3d MinPt,Vector3d MaxPt,vector<GeoInfo*>& results)
{
    //查询区域向外拓展固定容差，防止漏选
    double ExtentRangMin[3] = { MinPt.x(), MinPt.y(), MinPt.z() };
    double ExtentRangMax[3] = { MaxPt.x(), MaxPt.y(), MaxPt.z() };

    int nSize = m_mapRtree.Search(ExtentRangMin,ExtentRangMax,results);
    return ;
}

void geoSpatialSearch::getPolygonBBox(const Polygon& polygon,double* min,double* max)
{
    double dMinX=DBL_MAX,dMinY=DBL_MAX,dMinZ=DBL_MAX;
    double dMaxX=DBL_MIN,dMaxY=DBL_MIN,dMaxZ=DBL_MIN;
    
    int nSize = polygon.points_size();
    for(int i = 0; i < nSize;i++)
    {
        Vector3d  protoPt = polygon.points(i);
        dMinX = protoPt.x() < dMinX ? protoPt.x() : dMinX;
        dMinY = protoPt.y() < dMinY ? protoPt.y() : dMinY;
        dMinZ = protoPt.z() < dMinZ ? protoPt.z() : dMinZ;
            
        dMaxX = protoPt.x() > dMaxX ? protoPt.x() : dMaxX;
        dMaxY = protoPt.y() > dMaxY ? protoPt.y() : dMaxY;
        dMaxZ = protoPt.z() > dMaxZ ? protoPt.z() : dMaxZ;
    }
    
    min[0]= dMinX;
    min[1]= dMinY;
    min[2]= dMinZ;
    max[0]= dMaxX;
    max[1]= dMaxY;
    max[2]= dMaxZ;
    return;
}

void geoSpatialSearch::getLaneBBox(const Lane& lane,double* min,double* max)
{
    double dMinX=DBL_MAX,dMinY=DBL_MAX,dMinZ=DBL_MAX;
    double dMaxX=DBL_MIN,dMaxY=DBL_MIN,dMaxZ=DBL_MIN;
    
    int nSize = lane.lines_size();
    for(int i = 0; i < nSize;i++)
    {
        CurveLine  pcurveLine = lane.lines(i);
        int nPtSize = pcurveLine.points_size();
        if(nPtSize<1)
            continue;

        for(int j = 0; j < nPtSize; j++)
        {
            hdmap_proto::Vector3d protoPt = pcurveLine.points(j);
            dMinX = protoPt.x() < dMinX ? protoPt.x() : dMinX;
            dMinY = protoPt.y() < dMinY ? protoPt.y() : dMinY;
            dMinZ = protoPt.z() < dMinZ ? protoPt.z() : dMinZ;
            
            dMaxX = protoPt.x() > dMaxX ? protoPt.x() : dMaxX;
            dMaxY = protoPt.y() > dMaxY ? protoPt.y() : dMaxY;
            dMaxZ = protoPt.z() > dMaxZ ? protoPt.z() : dMaxZ;
        }
    }
    
    min[0]= dMinX;
    min[1]= dMinY;
    min[2]= dMinZ;
    max[0]= dMaxX;
    max[1]= dMaxY;
    max[2]= dMaxZ;
    return;
}

}  // namespace hdmap_op
