
#include "mapTopoHelper.h"

using std::ios;
using std::ios_base;
#include <stack>
#include <iostream>
const double Default_Max = 999999999.0;
namespace hdmap_kq_op {
    HDMapTopoHelper::HDMapTopoHelper()
    {
    }
    HDMapTopoHelper::~HDMapTopoHelper()
    {
    }

//创建连接Junction
    void HDMapTopoHelper::creatRoadGraph()
    {
        dpxMap* pMap = dpxGeoEngine::Instance()->GetMap();
        if(pMap == nullptr)
            return;
        m_vecSections = dpxMapCommonFunc::GetAllSection(pMap);
        vector<ccHObject*> vecZoneObjs = dpxMapCommonFunc::getLyrAllObjs(eOT_Zone,eObj_Zone);

        m_sectionInfos.clear();
        m_vecSectionIDs.clear();
        for(int i = 0;i<m_vecSections.size();i++)
        {
            ccHObject* pSection = m_vecSections[i];
            int nID = -1;
            if(pSection->hasMetaData(DPX_UID))
                nID = pSection->getMetaData(DPX_UID).toInt();

            int  nHeadID = dpxMapCommonFunc::GetSectionPreID(pSection);
            int  nSucID = dpxMapCommonFunc::GetSectionSucID(pSection);
            m_vecSectionIDs.push_back(nID);

            sectionInfo info;
            info.m_nID = nID;
            info.m_nPreID = nHeadID;
            info.m_nSucID = nSucID;
            m_sectionInfos.push_back(info);
            m_MapId2Obj.insert(std::make_pair(nID,pSection));
        }

        m_vecZoneIDs.clear();
        m_zoneInfos.clear();
        for(int i = 0;i < vecZoneObjs.size();i++)
        {
            ccHObject* pZone = vecZoneObjs[i];
            int nID = -1;
            if(pZone->hasMetaData(DPX_UID))
                nID = pZone->getMetaData(DPX_UID).toInt();

            vector<int> vecPreIDs = dpxMapCommonFunc::GetRelatedIDs(pZone,PRE_RELATED_UID);
            vector<int> vecSucIDs = dpxMapCommonFunc::GetRelatedIDs(pZone,SUC_RELATED_UID);

            m_vecZoneIDs.push_back(nID);

            ZoneInfo zoninfo;
            zoninfo.m_nID = nID;
            zoninfo.m_vecPreIDs = vecPreIDs;
            zoninfo.m_vecSucIDs = vecSucIDs;
            m_zoneInfos.push_back(zoninfo);
        }

        creatEdge();  //m_vecEdge

        creatGraph(); //Edge node
    }

    vector<CCVector3>  HDMapTopoHelper::Search(CCVector3 startPt, CCVector3 endPt)
    {
        vector<CCVector3>  allPathPoints;
        //构建路网
        creatRoadGraph();

        //查询起始点临近的边和结点
        double dMinDis,dMinDis2;
        CCVector3 nearPt,nearPt2;
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

        //startPt--nearPt--seg+1 ,size-1  other sections   //前面距离

        allPathPoints.push_back(startPt);
        allPathPoints.push_back(nearPt);

        ccHObject* pStartSection = m_MapId2Obj[nSectionID];
        ccHObject* refLineSet = dpxMapCommonFunc::getRefLineSet(pStartSection); //One
        vector<CCVector3> vecPts = dpxMapCommonFunc::getPtsFromLineSet(refLineSet);
        for(int i = nSegIndex+1; i < vecPts.size(); i++)
        {
            allPathPoints.push_back(vecPts[i]);
        }

        for(int i = nSectionIndex+1;i < m_vecEdge[nEdgeIndex].size();i++)
        {
            int ntempID = m_vecEdge[nEdgeIndex][i];
            ccHObject* pSection = m_MapId2Obj[ntempID];

            ccHObject* refLineSet = dpxMapCommonFunc::getRefLineSet(pSection); //One
            vector<CCVector3> vecPts = dpxMapCommonFunc::getPtsFromLineSet(refLineSet);
            for(int i = 0; i < vecPts.size(); i++)
            {
                allPathPoints.push_back(vecPts[i]);
            }
        }

        int nStartNode = m_edgeInfos[nEdgeIndex].m_nEnterZoneId; //前面的Node
        int nEndNode = m_edgeInfos[nEdgeIndex2].m_nExitZoneId; //前面的Node

        //test fps
        m_dMinPath = DBL_MAX;
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
            for(int j = 0;j<m_vecEdge[nEdgeIndex].size();j++)
            {
                int nSectionId = m_vecEdge[nEdgeIndex][j];
                ccHObject* pSection = m_MapId2Obj[nSectionId];
                ccHObject* refLineSet = dpxMapCommonFunc::getRefLineSet(pSection); //One
                vector<CCVector3> vecPts = dpxMapCommonFunc::getPtsFromLineSet(refLineSet);
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
            ccHObject* pSection = m_MapId2Obj[ntempID];

            ccHObject* refLineSet = dpxMapCommonFunc::getRefLineSet(pSection); //One
            vector<CCVector3> vecPts = dpxMapCommonFunc::getPtsFromLineSet(refLineSet);
            for(int i = 0; i < vecPts.size(); i++)
            {
                allPathPoints.push_back(vecPts[i]);
            }
        }
        ccHObject* pEndSection = m_MapId2Obj[nSectionID2];
        ccHObject* refLineSet2 = dpxMapCommonFunc::getRefLineSet(pEndSection); //One
        vector<CCVector3> vecPts2 = dpxMapCommonFunc::getPtsFromLineSet(refLineSet2);
        for(int i = 0; i <= nSegIndex2; i++)
        {
            allPathPoints.push_back(vecPts2[i]);
        }

        allPathPoints.push_back(nearPt2);
        allPathPoints.push_back(endPt);

        QString strOut2;
        for(int i = 0;i< m_vecOutPath.size();i++)
            strOut2 +=  " " + QString::number(m_vecOutPath[i]);
        cout << strOut2.toStdString() << endl;
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

    bool HDMapTopoHelper::findNeartestSection(CCVector3& Pt,double& dMinDis,CCVector3& nearPt,int& nSectionID,int& nSegIndex)
    {
        if(m_vecSections.size()<1)
            return false;

        bool bFind = false;
        dMinDis = DBL_MAX;
        for(int i = 0;i < m_vecSections.size();i++)
        {
            ccHObject* pSection =  m_vecSections[i];
            ccHObject* refLineSet = dpxMapCommonFunc::getRefLineSet(pSection); //One
            if(refLineSet==0)
                continue;
            vector<CCVector3> vecPts = dpxMapCommonFunc::getPtsFromLineSet(refLineSet);
            double dDis;
            CCVector3 PtTemp;
            int nSegIndexTemp;
            dpxAlgorithmFun::DistanceofPointToPolyLine(Pt,vecPts,dDis,PtTemp,nSegIndexTemp);
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
                indexSuc = getPreSectionID(indexSuc);
            }

            while (!mystack.empty())
            {
                vecEdgeIDs.push_back( mystack.top());
                mystack.pop();
            }

            vecEdgeIDs.insert(vecEdgeIDs.end(),sucIDs.begin(),sucIDs.end());
            m_vecEdge.push_back(vecEdgeIDs);
        }

        return ;
    }

    void HDMapTopoHelper::creatGraph()
    {
        int nEdge = m_vecEdge.size();
        int mNode = m_zoneInfos.size();
        if( nEdge < 1 )
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
                int nEdgeIndex =  getEdgeIndexFormSecID(sectionID,true);
                if(nEdgeIndex<0)
                    continue;
                m_edgeInfos[nEdgeIndex].m_nEnterZoneId = zoneinfo.m_nID;
            }

            for(int k = 0;k<zoneinfo.m_vecSucIDs.size();k++)
            {
                int sectionID = zoneinfo.m_vecSucIDs[k];
                int nEdgeIndex =  getEdgeIndexFormSecID(sectionID,false);
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
        if(m_vecEdge.size()<=nEdgeIndex || nEdgeIndex<0)
            return -1;
        vector<int> vecSectionIDs = m_vecEdge[nEdgeIndex];
        double dDistance = 0.0;
        for(int i = 0;i<vecSectionIDs.size();i++)
        {
            int nTempID = vecSectionIDs[i];
            for(int j = 0;j<m_vecSections.size();j++)
            {
                ccHObject* pSection = m_vecSections[j];
                int nID = -1;
                if(pSection->hasMetaData(DPX_UID))
                    nID = pSection->getMetaData(DPX_UID).toInt();
                if(nID == nTempID)
                {
                    dDistance += dpxMapCommonFunc::getSectionDistance(pSection);
                    break;
                }
            }
        }
        return dDistance;
    }

//可以优化
    bool HDMapTopoHelper::getEdgeFormSectionID(int nSectionId,int& nEdgeIndex,int& nSectionIndex)
    {
        if(m_vecEdge.size()<1)
            return false;

        for(int i = 0;i < m_vecEdge.size();i++)
        {
            vector<int>& sectionIDs = m_vecEdge[i];
            if(sectionIDs.size()<1)
                continue;

            for(int j = 0;j < sectionIDs.size(); j++)
            {
                if(sectionIDs[j] == nSectionId)
                {
                    nEdgeIndex = i;
                    nSectionIndex = j;
                    return true;
                }
            }
        }
        return false;
    }


    int HDMapTopoHelper::getEdgeIndexFormSecID(int nSectionId,bool bIsPre)
    {
        if(m_vecEdge.size()<1)
            return -1;

        for(int i = 0;i<m_vecEdge.size();i++)
        {
            vector<int>& sectionIDs = m_vecEdge[i];
            if(sectionIDs.size()<1)
                continue;

            int index =  bIsPre ? sectionIDs.size()-1 : 0;
            if(sectionIDs[index]==nSectionId)
                return i;
        }
        return -1;
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
        for(int i = 0;i< m_vecSectionIDs.size();i++)
        {
            if(m_vecSectionIDs[i] == nCurrentId)
            {
                return m_sectionInfos[i].m_nSucID;
            }
        }
        return -1;
    }

    int HDMapTopoHelper::getPreSectionID(int nCurrentId)
    {
        //查找ID
        for(int i = 0;i< m_vecSectionIDs.size();i++)
        {
            if(m_vecSectionIDs[i] == nCurrentId)
            {
                return m_sectionInfos[i].m_nPreID;
            }
        }
        return -1;
    }

}  // namespace hdmap_op
