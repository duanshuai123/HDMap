
#include "mapTopoHelper.h"

using std::ios;
using std::ios_base;
#include <stack>
#include <iostream>
const double Default_Max = 999999999.0;
namespace hdmap_kq_op {
    
HDMapTopoHelper::HDMapTopoHelper()
{
    m_pMap = 0;
}

HDMapTopoHelper::~HDMapTopoHelper()
{
}

//初始化地图数据
void HDMapTopoHelper::InitalMapData(Map* pMap)
{
    m_pMap = pMap;
    //构建路网
    creatRoadGraph();
}

//创建连接Junction
void HDMapTopoHelper::creatRoadGraph()
{
    if(m_pMap==0)
        return ;
    
    m_vecSections =  mapCommonFunc::GetAllSection(m_pMap);
    vector<Zone*> vecZoneObjs = mapCommonFunc::getAllZoneObjs(m_pMap);
    
    m_SectionId2info.clear();
    m_vecSectionIDs.clear();
    m_MapId2Obj.clear();
    for(int i = 0;i<m_vecSections.size();i++)
    {
        Section* pSection = m_vecSections[i];

        int nID = mapCommonFunc::GetSectionID(pSection);
        int  nHeadID = mapCommonFunc::GetSectionPreID(pSection);
        int  nSucID = mapCommonFunc::GetSectionSucID(pSection);
        m_vecSectionIDs.push_back(nID);
        
        sectionInfo info;
        info.m_nPreID = nHeadID;
        info.m_nSucID = nSucID;
        m_SectionId2info[nID] = info;
        m_MapId2Obj.insert(std::make_pair(nID,pSection));
    }
    
    m_zoneInfos.clear();
    for(int i = 0;i < vecZoneObjs.size();i++)
    {
        Zone* pZone = vecZoneObjs[i];
        int nID = pZone->mutable_id()->id();
        vector<int> vecPreIDs = mapCommonFunc::GetRelatedIDs(pZone,true);
        vector<int> vecSucIDs = mapCommonFunc::GetRelatedIDs(pZone,false);
        
        ZoneInfo zoninfo;
        zoninfo.m_nID = nID;
        zoninfo.m_vecPreIDs = vecPreIDs;
        zoninfo.m_vecSucIDs = vecSucIDs;
        m_zoneInfos.push_back(zoninfo);
    }
    
    creatEdge();  //m_vecEdge
    
    creatGraph(); //Edge node
}

vector<Vector3d>  HDMapTopoHelper::Search(Vector3d startPt, Vector3d endPt)
{
    vector<Vector3d>  allPathPoints;
    //查询起始点临近的边和结点
    double dMinDis,dMinDis2;
    Vector3d nearPt,nearPt2;
    int nSectionID,nSectionID2;
    int nSegIndex,nSegIndex2;
    bool bFind = findNeartestSection(startPt,dMinDis,nearPt,nSectionID,nSegIndex);
    bool bFind2 = findNeartestSection(endPt,dMinDis2,nearPt2,nSectionID2,nSegIndex2);
    if(!bFind || !bFind2)
        return allPathPoints;
    int nEdgeIndex,nEdgeIndex2;
    int nSectionIndex,nSectionIndex2;
    getEdgeFormSectionID(nSectionID,nEdgeIndex,nSectionIndex);
    getEdgeFormSectionID(nSectionID2,nEdgeIndex2,nSectionIndex2);

    if(nEdgeIndex<0 || nEdgeIndex2<0)
        return allPathPoints;
    if(m_edgeInfos.find(nEdgeIndex)==m_edgeInfos.end())
        return allPathPoints;
    if(m_edgeInfos.find(nEdgeIndex2)==m_edgeInfos.end())
        return allPathPoints;
    
    //当路径不跨越Zone结点时
    if(nEdgeIndex==nEdgeIndex2)
    {
        vector<Vector3d> outResult;
        bool bFind = getSameEdgePoint(nEdgeIndex,nSectionIndex,nSegIndex,nSectionIndex2,nSegIndex2,outResult);
        if(bFind)
        {
            allPathPoints.push_back(startPt);
            allPathPoints.push_back(nearPt);
            allPathPoints.insert(allPathPoints.end(),outResult.begin(),outResult.end());
            allPathPoints.push_back(nearPt2);
            allPathPoints.push_back(endPt);
            return allPathPoints;
        }
    }
    
    //startPt--nearPt--seg+1 ,size-1  other sections   //前面距离
    allPathPoints.push_back(startPt);
    allPathPoints.push_back(nearPt);
    
    Section* pStartSection = m_MapId2Obj[nSectionID];
    Lane* refLineSet = mapCommonFunc::getRefLineSet(pStartSection); //One
    vector<Vector3d> vecPts = mapCommonFunc::getPtsFromLineSet(refLineSet);
    for(int i = nSegIndex+1; i < vecPts.size(); i++)
    {
        allPathPoints.push_back(vecPts[i]);
    }
    
    for(int i = nSectionIndex+1;i < m_vecEdge[nEdgeIndex].size();i++)
    {
        int ntempID = m_vecEdge[nEdgeIndex][i];
        Section* pSection = m_MapId2Obj[ntempID];
        
        Lane* refLineSet = mapCommonFunc::getRefLineSet(pSection); //One
        vector<Vector3d> vecPts = mapCommonFunc::getPtsFromLineSet(refLineSet);
        for(int i = 0; i < vecPts.size(); i++)
        {
            allPathPoints.push_back(vecPts[i]);
        }
    }

    int nStartNode = m_edgeInfos[nEdgeIndex].m_nEnterZoneId; //前面的Node
    int nEndNode = m_edgeInfos[nEdgeIndex2].m_nExitZoneId; //前面的Node
    
    m_vecOutPath.clear();//消除
    //test fps
    m_dMinPath = Default_Max;
    m_vecMark.resize(m_zoneInfos.size());
    m_vecMark[nStartNode] = 1;
    vector<int> vecPath;
    vecPath.push_back(nStartNode);
    dfsNodeSeatch(nStartNode,nEndNode,0,vecPath);
    
    for(int i = 0;i<m_vecOutPath.size()-1;i++)
    {
        int nExitNode =  m_vecOutPath[i];
        int nEnterNode =  m_vecOutPath[i+1];
        int nEdgeIndex = findEdge(nExitNode,nEnterNode);
        for(int j = 0;j < m_vecEdge[nEdgeIndex].size();j++)
        {
            int nSectionId = m_vecEdge[nEdgeIndex][j];
            Section* pSection = m_MapId2Obj[nSectionId];
            Lane* refLineSet = mapCommonFunc::getRefLineSet(pSection); //One
            vector<Vector3d> vecPts = mapCommonFunc::getPtsFromLineSet(refLineSet);
            for(int k = 0; k < vecPts.size(); k++)
            {
                allPathPoints.push_back(vecPts[k]);
            }
        }
    }
    
    //endNode other sections 0---seg---nearPt2---endpt //后面距离
    for(int i = 0;i < nSectionIndex2;i++)
    {
        int ntempID = m_vecEdge[nEdgeIndex2][i];
        Section* pSection = m_MapId2Obj[ntempID];
        
        Lane* refLineSet = mapCommonFunc::getRefLineSet(pSection); //One
        vector<Vector3d> vecPts = mapCommonFunc::getPtsFromLineSet(refLineSet);
        for(int i = 0; i < vecPts.size(); i++)
        {
            allPathPoints.push_back(vecPts[i]);
        }
    }
    Section* pEndSection = m_MapId2Obj[nSectionID2];
    Lane* refLineSet2 = mapCommonFunc::getRefLineSet(pEndSection); //One
    vector<Vector3d> vecPts2 = mapCommonFunc::getPtsFromLineSet(refLineSet2);
    for(int i = 0; i <= nSegIndex2; i++)
    {
        allPathPoints.push_back(vecPts2[i]);
    }
    
    allPathPoints.push_back(nearPt2);
    allPathPoints.push_back(endPt);
}

int HDMapTopoHelper::findEdge(int nExitNode,int nEnterNode)
{
    std::map<int,edgeInfo>::iterator itor = m_edgeInfos.begin();
    while(itor!=m_edgeInfos.end())
    {
        edgeInfo info = itor->second;
        if(info.m_nEnterZoneId == nEnterNode && info.m_nExitZoneId==nExitNode)
            return itor->first;
        
        ++itor;
    }
      
    return -1;
}

bool HDMapTopoHelper::findNeartestSection(Vector3d& Pt,double& dMinDis,Vector3d& nearPt,int& nSectionID,int& nSegIndex)
{
    if(m_vecSections.size()<1)
        return false;
    
    bool bFind = false;
    dMinDis = Default_Max;
    for(int i = 0;i < m_vecSections.size();i++)
    {
       Section* pSection =  m_vecSections[i];
       Lane* refLineSet = mapCommonFunc::getRefLineSet(pSection); //One
       if(refLineSet==0)
           continue;
       vector<Vector3d> vecPts = mapCommonFunc::getPtsFromLineSet(refLineSet);
       double dDis;
       Vector3d PtTemp;
       int nSegIndexTemp;
       geoAlgorithm::DistanceofPointToPolyLine(Pt,vecPts,dDis,PtTemp,nSegIndexTemp);
       if(dDis<dMinDis)
       {
           dMinDis = dDis;
           nearPt = PtTemp;
           nSectionID = i;
           nSegIndex = nSegIndexTemp;
           bFind = true;
       }
    }
    
    return bFind;
}

//最dfs的改造
void HDMapTopoHelper::dfsNodeSeatch(int startNode,int nEndNode, int dst,vector<int>& vecPath)
{
    if(m_dMinPath < dst)//当前走过路径大于之前最短路径，没必要再走下去
        return;
    
    if(startNode == nEndNode)//临界条件 
    {
        if(m_dMinPath > dst)
        {
            m_dMinPath = dst;
            m_vecOutPath = vecPath;
            return;
        }
    }
    else
    {
        int i;
        for(i = 0; i < m_zoneInfos.size(); i++)
        {
            if(m_EdgeMatrix[startNode][i] != Default_Max 
                && m_EdgeMatrix[startNode][i] != 0 
                && m_vecMark[i] == 0 )
            {
                m_vecMark[i] = 1;
                vecPath.push_back(i);
                int nIndex = vecPath.size()-1;
                dfsNodeSeatch(i, nEndNode ,dst+m_EdgeMatrix[startNode][i],vecPath);//di gui
                m_vecMark[i] = 0;
                vecPath.erase(vecPath.begin()+nIndex);
            }
        }
        return;
    }
} 

void  HDMapTopoHelper::creatEdge()
{
    m_vecEdge.clear();
    vector<int> vecUsed,vecUnUsed;
    vecUnUsed = m_vecSectionIDs;
    while (vecUnUsed.size()>0)
    {
        vector<int>  vecEdgeIDs;//一条边
        int nSeedID = vecUnUsed[0];
        if(vecUnUsed.size()==1)
        {
            vecEdgeIDs.push_back(nSeedID);
            m_vecEdge.push_back(vecEdgeIDs);
            m_MapId2EdgeIndex[nSeedID] = m_vecEdge.size()-1;
            break;
        }
        
        vecUsed.push_back(nSeedID);
        eraseID(vecUnUsed,nSeedID);
        
        std::stack<int> mystack;
        int indexPre = getPreSectionID(nSeedID);
        while(indexPre>0)
        {
            vecUsed.push_back(indexPre);
            mystack.push(indexPre);
            eraseID(vecUnUsed,indexPre);
            indexPre = getPreSectionID(indexPre);
        }
        
        std::vector<int> sucIDs;
        sucIDs.push_back(nSeedID);
        int indexSuc = getSucSectionID(nSeedID);
        while(indexSuc>0)
        {
            vecUsed.push_back(indexSuc);
            sucIDs.push_back(indexSuc);
            eraseID(vecUnUsed,indexSuc);
            indexSuc = getSucSectionID(indexSuc);
        }
        
        while (!mystack.empty())
        {
            int nID = mystack.top();
            vecEdgeIDs.push_back(nID);
            m_MapId2EdgeIndex[nID] = m_vecEdge.size(); //sectionID与EdgeIndex映射
            mystack.pop();
        }
        
        for(int i = 0;i<sucIDs.size();i++)
        {
            vecEdgeIDs.push_back(sucIDs[i]);
            m_MapId2EdgeIndex[sucIDs[i]] = m_vecEdge.size();//sectionID与EdgeIndex映射
        }
        m_vecEdge.push_back(vecEdgeIDs);
    }
    
    return ;
}

void HDMapTopoHelper::creatGraph()
{
    int nEdge = m_vecEdge.size();
    int mNode = m_zoneInfos.size();
    if(nEdge < 1)
        return;
    
    //初始化邻接矩阵
    m_EdgeMatrix.clear();
    int i, j;
    m_EdgeMatrix.resize(mNode);
    for(i = 0; i < mNode; i++)
    {
        m_EdgeMatrix[i].resize(mNode);
        for(j = 0; j < mNode; j++)
        {
            m_EdgeMatrix[i][j] = Default_Max;
        }
        m_EdgeMatrix[i][i] = 0;
    }
    m_edgeInfos.clear();
    std::map<int,int>  mapZoneIDAndIndex;
    for(int i = 0;i < m_zoneInfos.size();i++)
    {
        ZoneInfo zoneinfo = m_zoneInfos[i];
        for(int j = 0;j<zoneinfo.m_vecPreIDs.size();j++)
        {
           int sectionID = zoneinfo.m_vecPreIDs[j];
           int nEdgeIndex =  getEdgeIndexFormSecID(sectionID);
           if(nEdgeIndex<0)
               continue;
           m_edgeInfos[nEdgeIndex].m_nEnterZoneId = zoneinfo.m_nID;
        }
        
        for(int k = 0;k<zoneinfo.m_vecSucIDs.size();k++)
        {
           int sectionID = zoneinfo.m_vecSucIDs[k];
           int nEdgeIndex =  getEdgeIndexFormSecID(sectionID);
           if(nEdgeIndex<0)
               continue;
           m_edgeInfos[nEdgeIndex].m_nExitZoneId = zoneinfo.m_nID;
        }
        mapZoneIDAndIndex[zoneinfo.m_nID] = i;
    }
    
    map<int,edgeInfo>::iterator iter = m_edgeInfos.begin();
    while(iter != m_edgeInfos.end())
    {
        int nEdgeIndex = iter->first;
        edgeInfo info =  iter->second;
        if(info.m_nEnterZoneId>-1 && info.m_nExitZoneId>-1)
        {
           double dDis = getEdgeDistance(nEdgeIndex);
           int firstNode = mapZoneIDAndIndex[info.m_nExitZoneId];
           int secondNode = mapZoneIDAndIndex[info.m_nEnterZoneId];
           m_EdgeMatrix[firstNode][secondNode] = dDis;
        }
        iter++;
    }
    for(int i = 0;i<m_EdgeMatrix.size();i++)
    {
        for(int j = 0;j<m_EdgeMatrix[i].size();j++)
        {
            cout << m_EdgeMatrix[i][j] << " ";
        }
        cout <<  endl;
    }
}

double HDMapTopoHelper::getEdgeDistance(int nEdgeIndex)
{
   if(m_vecEdge.size() <= nEdgeIndex || nEdgeIndex<0)
       return -1;
   vector<int> vecSectionIDs = m_vecEdge[nEdgeIndex];
   double dDistance = 0.0;
   for(int i = 0;i<vecSectionIDs.size();i++)
   {
       int nTempID = vecSectionIDs[i];
       Section* pSection = m_MapId2Obj[nTempID];
       dDistance += mapCommonFunc::getSectionDistance(pSection);
   }
   return dDistance;
}

//可以优化
bool HDMapTopoHelper::getEdgeFormSectionID(int nSectionId,int& nEdgeIndex,int& nSectionIndex)
{
    if(m_MapId2EdgeIndex.find(nSectionId)==m_MapId2EdgeIndex.end())
        return false;
    
    nEdgeIndex = m_MapId2EdgeIndex[nSectionId];
    if(m_vecEdge.size()<=nEdgeIndex)
        return false;
    
    vector<int>& sectionIDs = m_vecEdge[nEdgeIndex];
    if(sectionIDs.size()<1)
        return false;
       
    for(int j = 0; j < sectionIDs.size(); j++)
    {
        if(sectionIDs[j] == nSectionId)
        {
            nSectionIndex = j;
            return true;
        }
    }
    return false;
}    


int HDMapTopoHelper::getEdgeIndexFormSecID(int nSectionId)
{
    if(m_MapId2EdgeIndex.find(nSectionId)==m_MapId2EdgeIndex.end())
        return -1;
    
    return m_MapId2EdgeIndex[nSectionId];
}    

void  HDMapTopoHelper::eraseID(vector<int>& vecIDs,int nID)
{
    for(int i = vecIDs.size()-1;i>=0;i--)
    {
        if(vecIDs[i]==nID)
        {
            vecIDs.erase(vecIDs.begin()+i);
            return;
        }
    }
}

int HDMapTopoHelper::getSucSectionID(int nCurrentId)
{
    //查找ID
    if(m_SectionId2info.find(nCurrentId)==m_SectionId2info.end())
        return -1;

    return m_SectionId2info[nCurrentId].m_nSucID;
}

int HDMapTopoHelper::getPreSectionID(int nCurrentId)
{
    //查找ID
    if(m_SectionId2info.find(nCurrentId)==m_SectionId2info.end())
        return -1;

    return m_SectionId2info[nCurrentId].m_nPreID;
}

bool  HDMapTopoHelper::getSameEdgePoint(int nEdgeIndex,int nSecIndex1,int nSeg1,
                                        int nSecIndex2,int nSeg2,vector<Vector3d>& outResult)
{
    vector<int> vecSectionIDs = m_vecEdge[nEdgeIndex];
    // 1为起始点，2为目标点
    if(nSecIndex1 < nSecIndex2)
    {
        //1--nSecIndex1上剩余的点
        int nSectionID1 = vecSectionIDs[nSecIndex1];
        Section* pStartSection = m_MapId2Obj[nSectionID1];
        Lane* refLineSet = mapCommonFunc::getRefLineSet(pStartSection); //One
        vector<Vector3d> vecPts = mapCommonFunc::getPtsFromLineSet(refLineSet);
        for(int i = nSeg1+1; i < vecPts.size(); i++)
        {
            outResult.push_back(vecPts[i]);
        }
        //2--中间section上的点
        for(int i = nSecIndex1+1; i < nSecIndex2; i++)
        {
            int ntempID = vecSectionIDs[i];
            Section* pSection = m_MapId2Obj[ntempID];
        
            Lane* refLineSet = mapCommonFunc::getRefLineSet(pSection); //One
            vector<Vector3d> vecPts = mapCommonFunc::getPtsFromLineSet(refLineSet);
            for(int i = 0; i < vecPts.size(); i++)
            {
                outResult.push_back(vecPts[i]);
            }
        }
        
         //3--nSecIndex2上的点
        int nSectionID2 = vecSectionIDs[nSecIndex2];
        Section* pEndSection = m_MapId2Obj[nSectionID2];
        Lane* EndrefLineSet = mapCommonFunc::getRefLineSet(pEndSection); //One
        vector<Vector3d> vecPts2 = mapCommonFunc::getPtsFromLineSet(EndrefLineSet);
        for(int i = 0; i <= nSeg2; i++)
        {
            outResult.push_back(vecPts2[i]);
        }
        return true;
    }
    else if(nSecIndex1==nSecIndex2 && nSeg1<nSeg2)
    {
        int nSectionID = vecSectionIDs[nSecIndex1];
        Section* pSection = m_MapId2Obj[nSectionID];
        Lane* refLineSet = mapCommonFunc::getRefLineSet(pSection); //One
        vector<Vector3d> vecPts = mapCommonFunc::getPtsFromLineSet(refLineSet);
        for(int i = nSeg1+1; i <= nSeg2; i++)
        {
            outResult.push_back(vecPts[i]);
        }
        return true;
    }
    else if(nSecIndex1==nSecIndex2 && nSeg1==nSeg2)//距离比较近不导航
        return true;
    
    return false;
}


}  // namespace hdmap_op
