// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: map.proto

#ifndef PROTOBUF_map_2eproto__INCLUDED
#define PROTOBUF_map_2eproto__INCLUDED

#include <string>

#include <google/protobuf/stubs/common.h>

#if GOOGLE_PROTOBUF_VERSION < 2006000
#error This file was generated by a newer version of protoc which is
#error incompatible with your Protocol Buffer headers.  Please update
#error your headers.
#endif
#if 2006000 < GOOGLE_PROTOBUF_MIN_PROTOC_VERSION
#error This file was generated by an older version of protoc which is
#error incompatible with your Protocol Buffer headers.  Please
#error regenerate this file with a newer version of protoc.
#endif

#include <google/protobuf/generated_message_util.h>
#include <google/protobuf/message.h>
#include <google/protobuf/repeated_field.h>
#include <google/protobuf/extension_set.h>
#include <google/protobuf/unknown_field_set.h>
#include "geometry.pb.h"
#include "section.pb.h"
#include "object.pb.h"
// @@protoc_insertion_point(includes)

namespace hdmap_proto {

// Internal implementation detail -- do not call these.
void  protobuf_AddDesc_map_2eproto();
void protobuf_AssignDesc_map_2eproto();
void protobuf_ShutdownFile_map_2eproto();

class Header;
class Map;

// ===================================================================

class Header : public ::google::protobuf::Message {
 public:
  Header();
  virtual ~Header();

  Header(const Header& from);

  inline Header& operator=(const Header& from) {
    CopyFrom(from);
    return *this;
  }

  inline const ::google::protobuf::UnknownFieldSet& unknown_fields() const {
    return _unknown_fields_;
  }

  inline ::google::protobuf::UnknownFieldSet* mutable_unknown_fields() {
    return &_unknown_fields_;
  }

  static const ::google::protobuf::Descriptor* descriptor();
  static const Header& default_instance();

  void Swap(Header* other);

  // implements Message ----------------------------------------------

  Header* New() const;
  void CopyFrom(const ::google::protobuf::Message& from);
  void MergeFrom(const ::google::protobuf::Message& from);
  void CopyFrom(const Header& from);
  void MergeFrom(const Header& from);
  void Clear();
  bool IsInitialized() const;

  int ByteSize() const;
  bool MergePartialFromCodedStream(
      ::google::protobuf::io::CodedInputStream* input);
  void SerializeWithCachedSizes(
      ::google::protobuf::io::CodedOutputStream* output) const;
  ::google::protobuf::uint8* SerializeWithCachedSizesToArray(::google::protobuf::uint8* output) const;
  int GetCachedSize() const { return _cached_size_; }
  private:
  void SharedCtor();
  void SharedDtor();
  void SetCachedSize(int size) const;
  public:
  ::google::protobuf::Metadata GetMetadata() const;

  // nested types ----------------------------------------------------

  // accessors -------------------------------------------------------

  // optional string version = 1;
  inline bool has_version() const;
  inline void clear_version();
  static const int kVersionFieldNumber = 1;
  inline const ::std::string& version() const;
  inline void set_version(const ::std::string& value);
  inline void set_version(const char* value);
  inline void set_version(const char* value, size_t size);
  inline ::std::string* mutable_version();
  inline ::std::string* release_version();
  inline void set_allocated_version(::std::string* version);

  // optional string date = 2;
  inline bool has_date() const;
  inline void clear_date();
  static const int kDateFieldNumber = 2;
  inline const ::std::string& date() const;
  inline void set_date(const ::std::string& value);
  inline void set_date(const char* value);
  inline void set_date(const char* value, size_t size);
  inline ::std::string* mutable_date();
  inline ::std::string* release_date();
  inline void set_allocated_date(::std::string* date);

  // optional string projection = 3;
  inline bool has_projection() const;
  inline void clear_projection();
  static const int kProjectionFieldNumber = 3;
  inline const ::std::string& projection() const;
  inline void set_projection(const ::std::string& value);
  inline void set_projection(const char* value);
  inline void set_projection(const char* value, size_t size);
  inline ::std::string* mutable_projection();
  inline ::std::string* release_projection();
  inline void set_allocated_projection(::std::string* projection);

  // required .hdmap_proto.Vector3d low = 4;
  inline bool has_low() const;
  inline void clear_low();
  static const int kLowFieldNumber = 4;
  inline const ::hdmap_proto::Vector3d& low() const;
  inline ::hdmap_proto::Vector3d* mutable_low();
  inline ::hdmap_proto::Vector3d* release_low();
  inline void set_allocated_low(::hdmap_proto::Vector3d* low);

  // required .hdmap_proto.Vector3d high = 5;
  inline bool has_high() const;
  inline void clear_high();
  static const int kHighFieldNumber = 5;
  inline const ::hdmap_proto::Vector3d& high() const;
  inline ::hdmap_proto::Vector3d* mutable_high();
  inline ::hdmap_proto::Vector3d* release_high();
  inline void set_allocated_high(::hdmap_proto::Vector3d* high);

  // @@protoc_insertion_point(class_scope:hdmap_proto.Header)
 private:
  inline void set_has_version();
  inline void clear_has_version();
  inline void set_has_date();
  inline void clear_has_date();
  inline void set_has_projection();
  inline void clear_has_projection();
  inline void set_has_low();
  inline void clear_has_low();
  inline void set_has_high();
  inline void clear_has_high();

  ::google::protobuf::UnknownFieldSet _unknown_fields_;

  ::google::protobuf::uint32 _has_bits_[1];
  mutable int _cached_size_;
  ::std::string* version_;
  ::std::string* date_;
  ::std::string* projection_;
  ::hdmap_proto::Vector3d* low_;
  ::hdmap_proto::Vector3d* high_;
  friend void  protobuf_AddDesc_map_2eproto();
  friend void protobuf_AssignDesc_map_2eproto();
  friend void protobuf_ShutdownFile_map_2eproto();

  void InitAsDefaultInstance();
  static Header* default_instance_;
};
// -------------------------------------------------------------------

class Map : public ::google::protobuf::Message {
 public:
  Map();
  virtual ~Map();

  Map(const Map& from);

  inline Map& operator=(const Map& from) {
    CopyFrom(from);
    return *this;
  }

  inline const ::google::protobuf::UnknownFieldSet& unknown_fields() const {
    return _unknown_fields_;
  }

  inline ::google::protobuf::UnknownFieldSet* mutable_unknown_fields() {
    return &_unknown_fields_;
  }

  static const ::google::protobuf::Descriptor* descriptor();
  static const Map& default_instance();

  void Swap(Map* other);

  // implements Message ----------------------------------------------

  Map* New() const;
  void CopyFrom(const ::google::protobuf::Message& from);
  void MergeFrom(const ::google::protobuf::Message& from);
  void CopyFrom(const Map& from);
  void MergeFrom(const Map& from);
  void Clear();
  bool IsInitialized() const;

  int ByteSize() const;
  bool MergePartialFromCodedStream(
      ::google::protobuf::io::CodedInputStream* input);
  void SerializeWithCachedSizes(
      ::google::protobuf::io::CodedOutputStream* output) const;
  ::google::protobuf::uint8* SerializeWithCachedSizesToArray(::google::protobuf::uint8* output) const;
  int GetCachedSize() const { return _cached_size_; }
  private:
  void SharedCtor();
  void SharedDtor();
  void SetCachedSize(int size) const;
  public:
  ::google::protobuf::Metadata GetMetadata() const;

  // nested types ----------------------------------------------------

  // accessors -------------------------------------------------------

  // optional .hdmap_proto.Header header = 1;
  inline bool has_header() const;
  inline void clear_header();
  static const int kHeaderFieldNumber = 1;
  inline const ::hdmap_proto::Header& header() const;
  inline ::hdmap_proto::Header* mutable_header();
  inline ::hdmap_proto::Header* release_header();
  inline void set_allocated_header(::hdmap_proto::Header* header);

  // repeated .hdmap_proto.Section sections = 2;
  inline int sections_size() const;
  inline void clear_sections();
  static const int kSectionsFieldNumber = 2;
  inline const ::hdmap_proto::Section& sections(int index) const;
  inline ::hdmap_proto::Section* mutable_sections(int index);
  inline ::hdmap_proto::Section* add_sections();
  inline const ::google::protobuf::RepeatedPtrField< ::hdmap_proto::Section >&
      sections() const;
  inline ::google::protobuf::RepeatedPtrField< ::hdmap_proto::Section >*
      mutable_sections();

  // repeated .hdmap_proto.Zone zones = 3;
  inline int zones_size() const;
  inline void clear_zones();
  static const int kZonesFieldNumber = 3;
  inline const ::hdmap_proto::Zone& zones(int index) const;
  inline ::hdmap_proto::Zone* mutable_zones(int index);
  inline ::hdmap_proto::Zone* add_zones();
  inline const ::google::protobuf::RepeatedPtrField< ::hdmap_proto::Zone >&
      zones() const;
  inline ::google::protobuf::RepeatedPtrField< ::hdmap_proto::Zone >*
      mutable_zones();

  // repeated .hdmap_proto.Obstacle obstacles = 4;
  inline int obstacles_size() const;
  inline void clear_obstacles();
  static const int kObstaclesFieldNumber = 4;
  inline const ::hdmap_proto::Obstacle& obstacles(int index) const;
  inline ::hdmap_proto::Obstacle* mutable_obstacles(int index);
  inline ::hdmap_proto::Obstacle* add_obstacles();
  inline const ::google::protobuf::RepeatedPtrField< ::hdmap_proto::Obstacle >&
      obstacles() const;
  inline ::google::protobuf::RepeatedPtrField< ::hdmap_proto::Obstacle >*
      mutable_obstacles();

  // repeated .hdmap_proto.SemanticPoint segPoint = 5;
  inline int segpoint_size() const;
  inline void clear_segpoint();
  static const int kSegPointFieldNumber = 5;
  inline const ::hdmap_proto::SemanticPoint& segpoint(int index) const;
  inline ::hdmap_proto::SemanticPoint* mutable_segpoint(int index);
  inline ::hdmap_proto::SemanticPoint* add_segpoint();
  inline const ::google::protobuf::RepeatedPtrField< ::hdmap_proto::SemanticPoint >&
      segpoint() const;
  inline ::google::protobuf::RepeatedPtrField< ::hdmap_proto::SemanticPoint >*
      mutable_segpoint();

  // optional .hdmap_proto.SlopeSets slopes = 6;
  inline bool has_slopes() const;
  inline void clear_slopes();
  static const int kSlopesFieldNumber = 6;
  inline const ::hdmap_proto::SlopeSets& slopes() const;
  inline ::hdmap_proto::SlopeSets* mutable_slopes();
  inline ::hdmap_proto::SlopeSets* release_slopes();
  inline void set_allocated_slopes(::hdmap_proto::SlopeSets* slopes);

  // @@protoc_insertion_point(class_scope:hdmap_proto.Map)
 private:
  inline void set_has_header();
  inline void clear_has_header();
  inline void set_has_slopes();
  inline void clear_has_slopes();

  ::google::protobuf::UnknownFieldSet _unknown_fields_;

  ::google::protobuf::uint32 _has_bits_[1];
  mutable int _cached_size_;
  ::hdmap_proto::Header* header_;
  ::google::protobuf::RepeatedPtrField< ::hdmap_proto::Section > sections_;
  ::google::protobuf::RepeatedPtrField< ::hdmap_proto::Zone > zones_;
  ::google::protobuf::RepeatedPtrField< ::hdmap_proto::Obstacle > obstacles_;
  ::google::protobuf::RepeatedPtrField< ::hdmap_proto::SemanticPoint > segpoint_;
  ::hdmap_proto::SlopeSets* slopes_;
  friend void  protobuf_AddDesc_map_2eproto();
  friend void protobuf_AssignDesc_map_2eproto();
  friend void protobuf_ShutdownFile_map_2eproto();

  void InitAsDefaultInstance();
  static Map* default_instance_;
};
// ===================================================================


// ===================================================================

// Header

// optional string version = 1;
inline bool Header::has_version() const {
  return (_has_bits_[0] & 0x00000001u) != 0;
}
inline void Header::set_has_version() {
  _has_bits_[0] |= 0x00000001u;
}
inline void Header::clear_has_version() {
  _has_bits_[0] &= ~0x00000001u;
}
inline void Header::clear_version() {
  if (version_ != &::google::protobuf::internal::GetEmptyStringAlreadyInited()) {
    version_->clear();
  }
  clear_has_version();
}
inline const ::std::string& Header::version() const {
  // @@protoc_insertion_point(field_get:hdmap_proto.Header.version)
  return *version_;
}
inline void Header::set_version(const ::std::string& value) {
  set_has_version();
  if (version_ == &::google::protobuf::internal::GetEmptyStringAlreadyInited()) {
    version_ = new ::std::string;
  }
  version_->assign(value);
  // @@protoc_insertion_point(field_set:hdmap_proto.Header.version)
}
inline void Header::set_version(const char* value) {
  set_has_version();
  if (version_ == &::google::protobuf::internal::GetEmptyStringAlreadyInited()) {
    version_ = new ::std::string;
  }
  version_->assign(value);
  // @@protoc_insertion_point(field_set_char:hdmap_proto.Header.version)
}
inline void Header::set_version(const char* value, size_t size) {
  set_has_version();
  if (version_ == &::google::protobuf::internal::GetEmptyStringAlreadyInited()) {
    version_ = new ::std::string;
  }
  version_->assign(reinterpret_cast<const char*>(value), size);
  // @@protoc_insertion_point(field_set_pointer:hdmap_proto.Header.version)
}
inline ::std::string* Header::mutable_version() {
  set_has_version();
  if (version_ == &::google::protobuf::internal::GetEmptyStringAlreadyInited()) {
    version_ = new ::std::string;
  }
  // @@protoc_insertion_point(field_mutable:hdmap_proto.Header.version)
  return version_;
}
inline ::std::string* Header::release_version() {
  clear_has_version();
  if (version_ == &::google::protobuf::internal::GetEmptyStringAlreadyInited()) {
    return NULL;
  } else {
    ::std::string* temp = version_;
    version_ = const_cast< ::std::string*>(&::google::protobuf::internal::GetEmptyStringAlreadyInited());
    return temp;
  }
}
inline void Header::set_allocated_version(::std::string* version) {
  if (version_ != &::google::protobuf::internal::GetEmptyStringAlreadyInited()) {
    delete version_;
  }
  if (version) {
    set_has_version();
    version_ = version;
  } else {
    clear_has_version();
    version_ = const_cast< ::std::string*>(&::google::protobuf::internal::GetEmptyStringAlreadyInited());
  }
  // @@protoc_insertion_point(field_set_allocated:hdmap_proto.Header.version)
}

// optional string date = 2;
inline bool Header::has_date() const {
  return (_has_bits_[0] & 0x00000002u) != 0;
}
inline void Header::set_has_date() {
  _has_bits_[0] |= 0x00000002u;
}
inline void Header::clear_has_date() {
  _has_bits_[0] &= ~0x00000002u;
}
inline void Header::clear_date() {
  if (date_ != &::google::protobuf::internal::GetEmptyStringAlreadyInited()) {
    date_->clear();
  }
  clear_has_date();
}
inline const ::std::string& Header::date() const {
  // @@protoc_insertion_point(field_get:hdmap_proto.Header.date)
  return *date_;
}
inline void Header::set_date(const ::std::string& value) {
  set_has_date();
  if (date_ == &::google::protobuf::internal::GetEmptyStringAlreadyInited()) {
    date_ = new ::std::string;
  }
  date_->assign(value);
  // @@protoc_insertion_point(field_set:hdmap_proto.Header.date)
}
inline void Header::set_date(const char* value) {
  set_has_date();
  if (date_ == &::google::protobuf::internal::GetEmptyStringAlreadyInited()) {
    date_ = new ::std::string;
  }
  date_->assign(value);
  // @@protoc_insertion_point(field_set_char:hdmap_proto.Header.date)
}
inline void Header::set_date(const char* value, size_t size) {
  set_has_date();
  if (date_ == &::google::protobuf::internal::GetEmptyStringAlreadyInited()) {
    date_ = new ::std::string;
  }
  date_->assign(reinterpret_cast<const char*>(value), size);
  // @@protoc_insertion_point(field_set_pointer:hdmap_proto.Header.date)
}
inline ::std::string* Header::mutable_date() {
  set_has_date();
  if (date_ == &::google::protobuf::internal::GetEmptyStringAlreadyInited()) {
    date_ = new ::std::string;
  }
  // @@protoc_insertion_point(field_mutable:hdmap_proto.Header.date)
  return date_;
}
inline ::std::string* Header::release_date() {
  clear_has_date();
  if (date_ == &::google::protobuf::internal::GetEmptyStringAlreadyInited()) {
    return NULL;
  } else {
    ::std::string* temp = date_;
    date_ = const_cast< ::std::string*>(&::google::protobuf::internal::GetEmptyStringAlreadyInited());
    return temp;
  }
}
inline void Header::set_allocated_date(::std::string* date) {
  if (date_ != &::google::protobuf::internal::GetEmptyStringAlreadyInited()) {
    delete date_;
  }
  if (date) {
    set_has_date();
    date_ = date;
  } else {
    clear_has_date();
    date_ = const_cast< ::std::string*>(&::google::protobuf::internal::GetEmptyStringAlreadyInited());
  }
  // @@protoc_insertion_point(field_set_allocated:hdmap_proto.Header.date)
}

// optional string projection = 3;
inline bool Header::has_projection() const {
  return (_has_bits_[0] & 0x00000004u) != 0;
}
inline void Header::set_has_projection() {
  _has_bits_[0] |= 0x00000004u;
}
inline void Header::clear_has_projection() {
  _has_bits_[0] &= ~0x00000004u;
}
inline void Header::clear_projection() {
  if (projection_ != &::google::protobuf::internal::GetEmptyStringAlreadyInited()) {
    projection_->clear();
  }
  clear_has_projection();
}
inline const ::std::string& Header::projection() const {
  // @@protoc_insertion_point(field_get:hdmap_proto.Header.projection)
  return *projection_;
}
inline void Header::set_projection(const ::std::string& value) {
  set_has_projection();
  if (projection_ == &::google::protobuf::internal::GetEmptyStringAlreadyInited()) {
    projection_ = new ::std::string;
  }
  projection_->assign(value);
  // @@protoc_insertion_point(field_set:hdmap_proto.Header.projection)
}
inline void Header::set_projection(const char* value) {
  set_has_projection();
  if (projection_ == &::google::protobuf::internal::GetEmptyStringAlreadyInited()) {
    projection_ = new ::std::string;
  }
  projection_->assign(value);
  // @@protoc_insertion_point(field_set_char:hdmap_proto.Header.projection)
}
inline void Header::set_projection(const char* value, size_t size) {
  set_has_projection();
  if (projection_ == &::google::protobuf::internal::GetEmptyStringAlreadyInited()) {
    projection_ = new ::std::string;
  }
  projection_->assign(reinterpret_cast<const char*>(value), size);
  // @@protoc_insertion_point(field_set_pointer:hdmap_proto.Header.projection)
}
inline ::std::string* Header::mutable_projection() {
  set_has_projection();
  if (projection_ == &::google::protobuf::internal::GetEmptyStringAlreadyInited()) {
    projection_ = new ::std::string;
  }
  // @@protoc_insertion_point(field_mutable:hdmap_proto.Header.projection)
  return projection_;
}
inline ::std::string* Header::release_projection() {
  clear_has_projection();
  if (projection_ == &::google::protobuf::internal::GetEmptyStringAlreadyInited()) {
    return NULL;
  } else {
    ::std::string* temp = projection_;
    projection_ = const_cast< ::std::string*>(&::google::protobuf::internal::GetEmptyStringAlreadyInited());
    return temp;
  }
}
inline void Header::set_allocated_projection(::std::string* projection) {
  if (projection_ != &::google::protobuf::internal::GetEmptyStringAlreadyInited()) {
    delete projection_;
  }
  if (projection) {
    set_has_projection();
    projection_ = projection;
  } else {
    clear_has_projection();
    projection_ = const_cast< ::std::string*>(&::google::protobuf::internal::GetEmptyStringAlreadyInited());
  }
  // @@protoc_insertion_point(field_set_allocated:hdmap_proto.Header.projection)
}

// required .hdmap_proto.Vector3d low = 4;
inline bool Header::has_low() const {
  return (_has_bits_[0] & 0x00000008u) != 0;
}
inline void Header::set_has_low() {
  _has_bits_[0] |= 0x00000008u;
}
inline void Header::clear_has_low() {
  _has_bits_[0] &= ~0x00000008u;
}
inline void Header::clear_low() {
  if (low_ != NULL) low_->::hdmap_proto::Vector3d::Clear();
  clear_has_low();
}
inline const ::hdmap_proto::Vector3d& Header::low() const {
  // @@protoc_insertion_point(field_get:hdmap_proto.Header.low)
  return low_ != NULL ? *low_ : *default_instance_->low_;
}
inline ::hdmap_proto::Vector3d* Header::mutable_low() {
  set_has_low();
  if (low_ == NULL) low_ = new ::hdmap_proto::Vector3d;
  // @@protoc_insertion_point(field_mutable:hdmap_proto.Header.low)
  return low_;
}
inline ::hdmap_proto::Vector3d* Header::release_low() {
  clear_has_low();
  ::hdmap_proto::Vector3d* temp = low_;
  low_ = NULL;
  return temp;
}
inline void Header::set_allocated_low(::hdmap_proto::Vector3d* low) {
  delete low_;
  low_ = low;
  if (low) {
    set_has_low();
  } else {
    clear_has_low();
  }
  // @@protoc_insertion_point(field_set_allocated:hdmap_proto.Header.low)
}

// required .hdmap_proto.Vector3d high = 5;
inline bool Header::has_high() const {
  return (_has_bits_[0] & 0x00000010u) != 0;
}
inline void Header::set_has_high() {
  _has_bits_[0] |= 0x00000010u;
}
inline void Header::clear_has_high() {
  _has_bits_[0] &= ~0x00000010u;
}
inline void Header::clear_high() {
  if (high_ != NULL) high_->::hdmap_proto::Vector3d::Clear();
  clear_has_high();
}
inline const ::hdmap_proto::Vector3d& Header::high() const {
  // @@protoc_insertion_point(field_get:hdmap_proto.Header.high)
  return high_ != NULL ? *high_ : *default_instance_->high_;
}
inline ::hdmap_proto::Vector3d* Header::mutable_high() {
  set_has_high();
  if (high_ == NULL) high_ = new ::hdmap_proto::Vector3d;
  // @@protoc_insertion_point(field_mutable:hdmap_proto.Header.high)
  return high_;
}
inline ::hdmap_proto::Vector3d* Header::release_high() {
  clear_has_high();
  ::hdmap_proto::Vector3d* temp = high_;
  high_ = NULL;
  return temp;
}
inline void Header::set_allocated_high(::hdmap_proto::Vector3d* high) {
  delete high_;
  high_ = high;
  if (high) {
    set_has_high();
  } else {
    clear_has_high();
  }
  // @@protoc_insertion_point(field_set_allocated:hdmap_proto.Header.high)
}

// -------------------------------------------------------------------

// Map

// optional .hdmap_proto.Header header = 1;
inline bool Map::has_header() const {
  return (_has_bits_[0] & 0x00000001u) != 0;
}
inline void Map::set_has_header() {
  _has_bits_[0] |= 0x00000001u;
}
inline void Map::clear_has_header() {
  _has_bits_[0] &= ~0x00000001u;
}
inline void Map::clear_header() {
  if (header_ != NULL) header_->::hdmap_proto::Header::Clear();
  clear_has_header();
}
inline const ::hdmap_proto::Header& Map::header() const {
  // @@protoc_insertion_point(field_get:hdmap_proto.Map.header)
  return header_ != NULL ? *header_ : *default_instance_->header_;
}
inline ::hdmap_proto::Header* Map::mutable_header() {
  set_has_header();
  if (header_ == NULL) header_ = new ::hdmap_proto::Header;
  // @@protoc_insertion_point(field_mutable:hdmap_proto.Map.header)
  return header_;
}
inline ::hdmap_proto::Header* Map::release_header() {
  clear_has_header();
  ::hdmap_proto::Header* temp = header_;
  header_ = NULL;
  return temp;
}
inline void Map::set_allocated_header(::hdmap_proto::Header* header) {
  delete header_;
  header_ = header;
  if (header) {
    set_has_header();
  } else {
    clear_has_header();
  }
  // @@protoc_insertion_point(field_set_allocated:hdmap_proto.Map.header)
}

// repeated .hdmap_proto.Section sections = 2;
inline int Map::sections_size() const {
  return sections_.size();
}
inline void Map::clear_sections() {
  sections_.Clear();
}
inline const ::hdmap_proto::Section& Map::sections(int index) const {
  // @@protoc_insertion_point(field_get:hdmap_proto.Map.sections)
  return sections_.Get(index);
}
inline ::hdmap_proto::Section* Map::mutable_sections(int index) {
  // @@protoc_insertion_point(field_mutable:hdmap_proto.Map.sections)
  return sections_.Mutable(index);
}
inline ::hdmap_proto::Section* Map::add_sections() {
  // @@protoc_insertion_point(field_add:hdmap_proto.Map.sections)
  return sections_.Add();
}
inline const ::google::protobuf::RepeatedPtrField< ::hdmap_proto::Section >&
Map::sections() const {
  // @@protoc_insertion_point(field_list:hdmap_proto.Map.sections)
  return sections_;
}
inline ::google::protobuf::RepeatedPtrField< ::hdmap_proto::Section >*
Map::mutable_sections() {
  // @@protoc_insertion_point(field_mutable_list:hdmap_proto.Map.sections)
  return &sections_;
}

// repeated .hdmap_proto.Zone zones = 3;
inline int Map::zones_size() const {
  return zones_.size();
}
inline void Map::clear_zones() {
  zones_.Clear();
}
inline const ::hdmap_proto::Zone& Map::zones(int index) const {
  // @@protoc_insertion_point(field_get:hdmap_proto.Map.zones)
  return zones_.Get(index);
}
inline ::hdmap_proto::Zone* Map::mutable_zones(int index) {
  // @@protoc_insertion_point(field_mutable:hdmap_proto.Map.zones)
  return zones_.Mutable(index);
}
inline ::hdmap_proto::Zone* Map::add_zones() {
  // @@protoc_insertion_point(field_add:hdmap_proto.Map.zones)
  return zones_.Add();
}
inline const ::google::protobuf::RepeatedPtrField< ::hdmap_proto::Zone >&
Map::zones() const {
  // @@protoc_insertion_point(field_list:hdmap_proto.Map.zones)
  return zones_;
}
inline ::google::protobuf::RepeatedPtrField< ::hdmap_proto::Zone >*
Map::mutable_zones() {
  // @@protoc_insertion_point(field_mutable_list:hdmap_proto.Map.zones)
  return &zones_;
}

// repeated .hdmap_proto.Obstacle obstacles = 4;
inline int Map::obstacles_size() const {
  return obstacles_.size();
}
inline void Map::clear_obstacles() {
  obstacles_.Clear();
}
inline const ::hdmap_proto::Obstacle& Map::obstacles(int index) const {
  // @@protoc_insertion_point(field_get:hdmap_proto.Map.obstacles)
  return obstacles_.Get(index);
}
inline ::hdmap_proto::Obstacle* Map::mutable_obstacles(int index) {
  // @@protoc_insertion_point(field_mutable:hdmap_proto.Map.obstacles)
  return obstacles_.Mutable(index);
}
inline ::hdmap_proto::Obstacle* Map::add_obstacles() {
  // @@protoc_insertion_point(field_add:hdmap_proto.Map.obstacles)
  return obstacles_.Add();
}
inline const ::google::protobuf::RepeatedPtrField< ::hdmap_proto::Obstacle >&
Map::obstacles() const {
  // @@protoc_insertion_point(field_list:hdmap_proto.Map.obstacles)
  return obstacles_;
}
inline ::google::protobuf::RepeatedPtrField< ::hdmap_proto::Obstacle >*
Map::mutable_obstacles() {
  // @@protoc_insertion_point(field_mutable_list:hdmap_proto.Map.obstacles)
  return &obstacles_;
}

// repeated .hdmap_proto.SemanticPoint segPoint = 5;
inline int Map::segpoint_size() const {
  return segpoint_.size();
}
inline void Map::clear_segpoint() {
  segpoint_.Clear();
}
inline const ::hdmap_proto::SemanticPoint& Map::segpoint(int index) const {
  // @@protoc_insertion_point(field_get:hdmap_proto.Map.segPoint)
  return segpoint_.Get(index);
}
inline ::hdmap_proto::SemanticPoint* Map::mutable_segpoint(int index) {
  // @@protoc_insertion_point(field_mutable:hdmap_proto.Map.segPoint)
  return segpoint_.Mutable(index);
}
inline ::hdmap_proto::SemanticPoint* Map::add_segpoint() {
  // @@protoc_insertion_point(field_add:hdmap_proto.Map.segPoint)
  return segpoint_.Add();
}
inline const ::google::protobuf::RepeatedPtrField< ::hdmap_proto::SemanticPoint >&
Map::segpoint() const {
  // @@protoc_insertion_point(field_list:hdmap_proto.Map.segPoint)
  return segpoint_;
}
inline ::google::protobuf::RepeatedPtrField< ::hdmap_proto::SemanticPoint >*
Map::mutable_segpoint() {
  // @@protoc_insertion_point(field_mutable_list:hdmap_proto.Map.segPoint)
  return &segpoint_;
}

// optional .hdmap_proto.SlopeSets slopes = 6;
inline bool Map::has_slopes() const {
  return (_has_bits_[0] & 0x00000020u) != 0;
}
inline void Map::set_has_slopes() {
  _has_bits_[0] |= 0x00000020u;
}
inline void Map::clear_has_slopes() {
  _has_bits_[0] &= ~0x00000020u;
}
inline void Map::clear_slopes() {
  if (slopes_ != NULL) slopes_->::hdmap_proto::SlopeSets::Clear();
  clear_has_slopes();
}
inline const ::hdmap_proto::SlopeSets& Map::slopes() const {
  // @@protoc_insertion_point(field_get:hdmap_proto.Map.slopes)
  return slopes_ != NULL ? *slopes_ : *default_instance_->slopes_;
}
inline ::hdmap_proto::SlopeSets* Map::mutable_slopes() {
  set_has_slopes();
  if (slopes_ == NULL) slopes_ = new ::hdmap_proto::SlopeSets;
  // @@protoc_insertion_point(field_mutable:hdmap_proto.Map.slopes)
  return slopes_;
}
inline ::hdmap_proto::SlopeSets* Map::release_slopes() {
  clear_has_slopes();
  ::hdmap_proto::SlopeSets* temp = slopes_;
  slopes_ = NULL;
  return temp;
}
inline void Map::set_allocated_slopes(::hdmap_proto::SlopeSets* slopes) {
  delete slopes_;
  slopes_ = slopes;
  if (slopes) {
    set_has_slopes();
  } else {
    clear_has_slopes();
  }
  // @@protoc_insertion_point(field_set_allocated:hdmap_proto.Map.slopes)
}


// @@protoc_insertion_point(namespace_scope)

}  // namespace hdmap_proto

#ifndef SWIG
namespace google {
namespace protobuf {


}  // namespace google
}  // namespace protobuf
#endif  // SWIG

// @@protoc_insertion_point(global_scope)

#endif  // PROTOBUF_map_2eproto__INCLUDED
