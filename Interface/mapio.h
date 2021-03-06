// @author Duanshuai (duanshuai2013@163.com)
// @brief test
//
// mapio.h

#ifndef MAP_KQ_IO_H_
#define MAP_KQ_IO_H_
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
using namespace hdmap_proto;

namespace hdmap_kq_op {
class MapIO {
 public:
  MapIO() {}
  explicit MapIO(Map *map) : map_(map) {}

  bool readTextFileToProtobufMap(const std::string &file_path);
  bool saveProtobufMapToTextFile(const std::string &save_path);

  bool readBinaryFileToProtobufMap(const std::string &file_path);
  bool saveProtobufMapToBinaryFile(const std::string &save_path);

  std::string toString();

 private:
  Map *map_;
};

}  // namespace hdmap_kq_proto
#endif  // MAPIO_H_
