
#include "mapCommonFunc.h"

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


}  // namespace hdmap_op
