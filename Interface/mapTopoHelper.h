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
#include "mapCommonFunc.h"
#include "geoAlgorithmFunc.h"
#include "../proto/map.pb.h"
using namespace std;
using namespace __gnu_cxx;
using namespace hdmap_proto;

namespace hdmap_kq_op {

struct sectionInfo
{
    int m_nPreID;
    int m_nSucID;
    sectionInfo()
    {
        m_nPreID = -1;
        m_nSucID = -1;
    }
};

struct ZoneInfo
{
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

struct edgeInfo
{
    int m_nEnterZoneId;
    int m_nExitZoneId;
    edgeInfo()
    {
        m_nEnterZoneId = -1;
        m_nExitZoneId = -1;
    }
};
//与地图要素相关的公共方法
class  HDMapTopoHelper
{
public:
    HDMapTopoHelper();
    ~HDMapTopoHelper();
public:

    //点对点路径查询，输出路径点
    vector<Vector3d>  Search(Vector3d  startPt, Vector3d endPt);
    //初始化地图数据，并构建路网
    void InitalMapData(Map* pMap);
    
protected:
    void creatRoadGraph();
    void dfsNodeSeatch(int startNode,int nEndNode, int dst,vector<int>& vecPath);
    void creatEdge();
    void creatGraph(); //根据Zone关联道路xiang
    
    int getSucSectionID(int nCurrentId);
    int getPreSectionID(int nCurrentId);
    void eraseID(vector<int>& vecIDs,int nID);
    bool getEdgeFormSectionID(int nSectionId,int& nEdgeIndex,int& nSectionIndex);
    int getEdgeIndexFormSecID(int nSectionId);
    
    double getEdgeDistance(int nEdgeIndex);
    bool findNeartestSection(Vector3d& Pt,double& dMinDis,Vector3d& nearPt,
                             int& nSectionID,int& nSegIndex);
    
    int findEdge(int nExitNode,int nEnterNode);
    //不跨越结点的路径查询，同一条道路上那种
    bool  getSameEdgePoint(int nEdgeIndex,int nSecIndex1,int nSeg1,int nSecIndex2,
                           int nSeg2,vector<Vector3d>& outResult);
protected:
    vector<int> m_vecSectionIDs;
    vector<Section*> m_vecSections;
    hash_map<int,sectionInfo> m_SectionId2info; //ID与Section的关联

    vector<ZoneInfo> m_zoneInfos;
    vector<vector<double>> m_EdgeMatrix;//邻接矩阵
    
    vector<vector<int>> m_vecEdge; //Edge=vector<sectionID>;m_vecEdge = vector<Edge>;
    std::map<int,edgeInfo> m_edgeInfos; //edgeIndex与信息关联 ,多余的
    
    hash_map<int,Section*> m_MapId2Obj; //Section ID与Section的关联
    hash_map<int,int> m_MapId2EdgeIndex; //Section ID与Edge的关联
    
private:
    vector<int> m_vecMark; //标记结点是否被遍历
    double m_dMinPath;
    vector<int> m_vecOutPath; //查询结果
    Map* m_pMap;//必须先传入地图数据
};

}  // namespace hdmap_kq_proto
#endif
