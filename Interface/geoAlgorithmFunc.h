// @author Duanshuai (duanshuai2013@163.com)
// @brief test
//
// mapio.h

#ifndef MAP_KQ_Geo_Algorithm_func_
#define MAP_KQ_Geo_Algorithm_func_
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
#include <math.h>
using  namespace std;
using  namespace hdmap_proto;

namespace hdmap_kq_op {

class geoAlgorithm
{
public:
    static  void DistanceofPointToPolyLine(const Vector3d& Pt,const vector<Vector3d>& vecLine2,
                                        double& dDis,Vector3d& PtNearest,int& nSegIndex);
    
    static  void DistanceOfPointToSegment(const Vector3d& PtA,const Vector3d& PtB,const Vector3d& PtS,
                                        double& dDis,Vector3d& PtNearest);
    
    static  double NormalizeValue(Vector3d v);
    
    static  double DisTance(const vector<Vector3d>& vecPts);
    static  double DisTance(Vector3d pt1,Vector3d pt2);
};
 
};  // namespace hdmap_kq_proto
#endif  // 
