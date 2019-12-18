// @author Duanshuai (duanshuai2013@163.com)
// @brief test
//
// HDMapTopoHelper.h

#ifndef MAP_KQ_TOPO_HELPER_FUNC_H_
#define MAP_KQ_TOPO_HELPER_FUNC_H_
#include "../proto/map.pb.h"

#include <fcntl.h>
#include <google/protobuf/io/zero_copy_stream_impl.h>
#include <google/protobuf/text_format.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <cstdio>
#include <fstream>
#include <iostream>
#include <string>
#include <hash_map>
using namespace std;
using namespace __gnu_cxx;

namespace hdmap_kq_op {

struct sectionInfo{
    int m_nID;
    int m_nPreID;
    int m_nSucID;
    sectionInfo()
    {
        m_nID = -1;
        m_nPreID = -1;
        m_nSucID = -1;
    }
};

struct ZoneInfo{
    int m_nID;
    vector<int> m_vecPreIDs;
    vector<int> m_vecSucIDs;

    ZoneInfo()
    {
        m_nID = -1;
        m_vecPreIDs.clear();
        m_vecSucIDs.clear();
    }
};

struct edgeInfo{
    int m_nEnterZoneId;
    int m_nExitZoneId;
    edgeInfo()
    {
        m_nEnterZoneId = -1;
        m_nExitZoneId = -1;
    }
};
//与地图要素相关的公共方法
class  HDMapTopoHelper{
public:
    HDMapTopoHelper();
    ~HDMapTopoHelper();
public:
    void creatRoadGraph();
    vector<CCVector3>  Search(CCVector3  startPt, CCVector3 endPt);
    void dfsNodeSeatch(int startNode,int nEndNode, int dst,vector<int>& vecPath);
protected:
    void creatEdge();
    void creatGraph(); //根据Zone关联道路

    int getSucSectionID(int nCurrentId);
    int getPreSectionID(int nCurrentId);
    void eraseID(vector<int>& vecIDs,int nID);
    bool getEdgeFormSectionID(int nSectionId,int& nEdgeIndex,int& nSectionIndex);
    int getEdgeIndexFormSecID(int nSectionId,bool bIsPre);

    double getEdgeDistance(int nEdgeIndex);
    bool findNeartestSection(CCVector3& Pt,double& dMinDis,CCVector3& nearPt,int& nSectionID,int& nSegIndex);

    int findEdge(int nExitNode,int nEnterNode);
private:
    vector<int> m_vecSectionIDs;
    vector<sectionInfo> m_sectionInfos;

    vector<int> m_vecZoneIDs;
    vector<ZoneInfo> m_zoneInfos;
    vector<vector<int>> m_vecEdge; //Edge=vector<sectionID>;m_vecEdge = vector<Edge>;

    vector<vector<double>> m_EdgeMatrix;
    vector<int> m_vecMark; //标记结点是否被遍历
    vector<ccHObject*> m_vecSections;

    double m_dMinPath;
    vector<int> m_vecOutPath; //查询结果

    std::map<int,edgeInfo> m_edgeInfos;
    hash_map<int,ccHObject*> m_MapId2Obj; //ID与Section的关联
};

}  // namespace hdmap_kq_proto
#endif
