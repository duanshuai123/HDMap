
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

int main(void) 
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
