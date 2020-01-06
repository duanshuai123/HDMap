
#include "../proto/map.pb.h"
#include "../proto/object.pb.h"
#include "../Interface/mapio.h"
#include "../Interface/mapTopoHelper.h"
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <cstdio>
#include <fstream>
#include <iostream>

int id_num = 0;
using namespace hdmap_proto;
using namespace hdmap_kq_op;

int testTopoSearch()
{
  Map proto_map;
  std::string txt_file = "../../test/testData_ZGC.txt";
  MapIO io(&proto_map);
  if (!io.readTextFileToProtobufMap(txt_file))
    return -1;
  
  HDMapTopoHelper* pHelper = new HDMapTopoHelper();
  pHelper->InitalMapData(&proto_map);
  Vector3d start,end;
  start.set_x(-148.99);
  start.set_y(211.07);
  start.set_z(-1.472);
  
  end.set_x(-330.78);
  end.set_y(188.27);
  end.set_z(-2.03);
  
  vector<Vector3d> vecResults =  pHelper->Search(start,end);
  
  std::cout << "path Points" << std::endl;
  for ( auto pt : vecResults )
  {
     std::cout <<  pt.x() << " " <<  pt.y() << " " << pt.z() << std::endl;
  }
  
  delete pHelper;
  return 0;
}

void testSlope()
{
    //----------------1:mapHeader-------------------------
	Map* pProtoMap = new hdmap_proto::Map();
    Header* pHeader = pProtoMap->mutable_header();
    hdmap_proto::Vector3d* pLow = pHeader->mutable_low();
    hdmap_proto::Vector3d* pHigh = pHeader->mutable_high();
    //range 0,0,0---100,100,100
    pLow->set_x(0);
    pLow->set_y(0);
    pLow->set_z(0);
    pHigh->set_x(100);
    pHigh->set_y(100);
    pHigh->set_z(100);
    
    SlopeSets* slopes = pProtoMap->mutable_slopes();
    slopes->set_pixsize(1); //设置网格为1×1米大小 ,每个坐标对应的格子需要自己计算
    //----------------2:添加网格坡度-------------------------
    //2行3列 坡向1 坡度值0.2
    hdmap_proto::Slope* pSlope1 = slopes->add_item();
    pSlope1->set_nx(2);
    pSlope1->set_ny(3);
    pSlope1->set_flag(1);
    pSlope1->set_slopevalue(0.2);
    
    //10行8列 坡向0 坡度值0.15
    hdmap_proto::Slope* pSlope2 = slopes->add_item();
    pSlope2->set_nx(10);
    pSlope2->set_ny(8);
    pSlope2->set_flag(1);
    pSlope2->set_slopevalue(0.15);
    
    //50行3列 坡向2 坡度值0.32
    hdmap_proto::Slope* pSlope3 = slopes->add_item();
    pSlope3->set_nx(50);
    pSlope3->set_ny(3);
    pSlope3->set_flag(1);
    pSlope3->set_slopevalue(0.32);
    
    //能计算的网格都一一添加坡度
    //....
    
	//---------3保存地图文档 文本与二进制各输出一份-----------
	MapIO mapIO(pProtoMap);
	std::string strFileName = "outmap.txt";
	mapIO.saveProtobufMapToTextFile(strFileName);
    
	std::string strFile2 = "outmap.bin";
	mapIO.saveProtobufMapToBinaryFile(strFile2);
    
    delete pProtoMap; //释放内存
}

int main(void) 
{
  testSlope();
  
  return 0;
}
