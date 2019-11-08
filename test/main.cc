
// @brief test
//
// main.cc

#include "proto/map.pb.h"
#include "include/mapio.h"

#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <cstdio>
#include <fstream>
#include <iostream>

int id_num = 0;

void initialMap(hdmap_proto::Map *map) {
  // ************  Set head  ************************************
  hdmap_proto::Header *head = map->mutable_header();
  head->set_version("1.0");
  head->set_date("30-10-2018");
  head->set_projection("proj");
  hdmap_proto::Vector3d *low = head->mutable_low();
  low->set_x(0.);
  low->set_y(0.);
  low->set_z(0.);
  hdmap_proto::Vector3d *high = head->mutable_high();
  high->set_x(200);
  high->set_y(160);
  high->set_z(0.);

  hdmap_proto::Zone *zone;

  zone = map->add_zones();
  id = zone->mutable_id();
  id->set_id(id_num++);
  id->set_name("zone a");

  // section
  hdmap_proto::Section *section;

  section = map->add_sections();
  id = section->mutable_id();
  id->set_id(id_num++);
  id->set_name("section a");
}

int main(void) {
  // ********************************   test save function
  // ********************************
  hdmap_proto::Map proto_map;
  //initialMap(&proto_map);
  std::cout << "initial OK" << std::endl;

  std::string txt_file = "map.txt";
  hdmap_op::MapIO io(&proto_map);
  /*if (!io.saveProtobufMapToTextFile(txt_file)) {
    return -1; // Failed to save data
  }
  if (!io.saveProtobufMapToBinaryFile(bin_file)) {
    return -1; // Failed to save data
  }*/
  // ********************************   test read function
  // ********************************

  if (!io.readTextFileToProtobufMap(txt_file)) {
    return -1;  // Failed to read file
  }
  if (!io.saveProtobufMapToBinaryFile(bin_file)) {
    return -1;
  }
  // if (!io.readBinaryFileToProtobufMap(bin_file)) {
  //  return -1; // Failed to read file
  //}

  return 0;
}
