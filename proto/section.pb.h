// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: section.proto

#ifndef PROTOBUF_section_2eproto__INCLUDED
#define PROTOBUF_section_2eproto__INCLUDED

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
#include <google/protobuf/generated_enum_reflection.h>
#include <google/protobuf/unknown_field_set.h>
#include "id.pb.h"
#include "geometry.pb.h"
// @@protoc_insertion_point(includes)

namespace hdmap_proto {

// Internal implementation detail -- do not call these.
void  protobuf_AddDesc_section_2eproto();
void protobuf_AssignDesc_section_2eproto();
void protobuf_ShutdownFile_section_2eproto();

class Section;

enum Section_LaneDirection {
  Section_LaneDirection_FORWARD = 1,
  Section_LaneDirection_BACKWARD = 2,
  Section_LaneDirection_TWOWAY = 3
};
bool Section_LaneDirection_IsValid(int value);
const Section_LaneDirection Section_LaneDirection_LaneDirection_MIN = Section_LaneDirection_FORWARD;
const Section_LaneDirection Section_LaneDirection_LaneDirection_MAX = Section_LaneDirection_TWOWAY;
const int Section_LaneDirection_LaneDirection_ARRAYSIZE = Section_LaneDirection_LaneDirection_MAX + 1;

const ::google::protobuf::EnumDescriptor* Section_LaneDirection_descriptor();
inline const ::std::string& Section_LaneDirection_Name(Section_LaneDirection value) {
  return ::google::protobuf::internal::NameOfEnum(
    Section_LaneDirection_descriptor(), value);
}
inline bool Section_LaneDirection_Parse(
    const ::std::string& name, Section_LaneDirection* value) {
  return ::google::protobuf::internal::ParseNamedEnum<Section_LaneDirection>(
    Section_LaneDirection_descriptor(), name, value);
}
// ===================================================================

class Section : public ::google::protobuf::Message {
 public:
  Section();
  virtual ~Section();

  Section(const Section& from);

  inline Section& operator=(const Section& from) {
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
  static const Section& default_instance();

  void Swap(Section* other);

  // implements Message ----------------------------------------------

  Section* New() const;
  void CopyFrom(const ::google::protobuf::Message& from);
  void MergeFrom(const ::google::protobuf::Message& from);
  void CopyFrom(const Section& from);
  void MergeFrom(const Section& from);
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

  typedef Section_LaneDirection LaneDirection;
  static const LaneDirection FORWARD = Section_LaneDirection_FORWARD;
  static const LaneDirection BACKWARD = Section_LaneDirection_BACKWARD;
  static const LaneDirection TWOWAY = Section_LaneDirection_TWOWAY;
  static inline bool LaneDirection_IsValid(int value) {
    return Section_LaneDirection_IsValid(value);
  }
  static const LaneDirection LaneDirection_MIN =
    Section_LaneDirection_LaneDirection_MIN;
  static const LaneDirection LaneDirection_MAX =
    Section_LaneDirection_LaneDirection_MAX;
  static const int LaneDirection_ARRAYSIZE =
    Section_LaneDirection_LaneDirection_ARRAYSIZE;
  static inline const ::google::protobuf::EnumDescriptor*
  LaneDirection_descriptor() {
    return Section_LaneDirection_descriptor();
  }
  static inline const ::std::string& LaneDirection_Name(LaneDirection value) {
    return Section_LaneDirection_Name(value);
  }
  static inline bool LaneDirection_Parse(const ::std::string& name,
      LaneDirection* value) {
    return Section_LaneDirection_Parse(name, value);
  }

  // accessors -------------------------------------------------------

  // required .hdmap_proto.Id id = 1;
  inline bool has_id() const;
  inline void clear_id();
  static const int kIdFieldNumber = 1;
  inline const ::hdmap_proto::Id& id() const;
  inline ::hdmap_proto::Id* mutable_id();
  inline ::hdmap_proto::Id* release_id();
  inline void set_allocated_id(::hdmap_proto::Id* id);

  // optional .hdmap_proto.Section.LaneDirection direction = 2;
  inline bool has_direction() const;
  inline void clear_direction();
  static const int kDirectionFieldNumber = 2;
  inline ::hdmap_proto::Section_LaneDirection direction() const;
  inline void set_direction(::hdmap_proto::Section_LaneDirection value);

  // repeated .hdmap_proto.CurveLine lines = 3;
  inline int lines_size() const;
  inline void clear_lines();
  static const int kLinesFieldNumber = 3;
  inline const ::hdmap_proto::CurveLine& lines(int index) const;
  inline ::hdmap_proto::CurveLine* mutable_lines(int index);
  inline ::hdmap_proto::CurveLine* add_lines();
  inline const ::google::protobuf::RepeatedPtrField< ::hdmap_proto::CurveLine >&
      lines() const;
  inline ::google::protobuf::RepeatedPtrField< ::hdmap_proto::CurveLine >*
      mutable_lines();

  // repeated uint32 pred_indices = 10;
  inline int pred_indices_size() const;
  inline void clear_pred_indices();
  static const int kPredIndicesFieldNumber = 10;
  inline ::google::protobuf::uint32 pred_indices(int index) const;
  inline void set_pred_indices(int index, ::google::protobuf::uint32 value);
  inline void add_pred_indices(::google::protobuf::uint32 value);
  inline const ::google::protobuf::RepeatedField< ::google::protobuf::uint32 >&
      pred_indices() const;
  inline ::google::protobuf::RepeatedField< ::google::protobuf::uint32 >*
      mutable_pred_indices();

  // repeated uint32 succ_indices = 11;
  inline int succ_indices_size() const;
  inline void clear_succ_indices();
  static const int kSuccIndicesFieldNumber = 11;
  inline ::google::protobuf::uint32 succ_indices(int index) const;
  inline void set_succ_indices(int index, ::google::protobuf::uint32 value);
  inline void add_succ_indices(::google::protobuf::uint32 value);
  inline const ::google::protobuf::RepeatedField< ::google::protobuf::uint32 >&
      succ_indices() const;
  inline ::google::protobuf::RepeatedField< ::google::protobuf::uint32 >*
      mutable_succ_indices();

  // @@protoc_insertion_point(class_scope:hdmap_proto.Section)
 private:
  inline void set_has_id();
  inline void clear_has_id();
  inline void set_has_direction();
  inline void clear_has_direction();

  ::google::protobuf::UnknownFieldSet _unknown_fields_;

  ::google::protobuf::uint32 _has_bits_[1];
  mutable int _cached_size_;
  ::hdmap_proto::Id* id_;
  ::google::protobuf::RepeatedPtrField< ::hdmap_proto::CurveLine > lines_;
  ::google::protobuf::RepeatedField< ::google::protobuf::uint32 > pred_indices_;
  ::google::protobuf::RepeatedField< ::google::protobuf::uint32 > succ_indices_;
  int direction_;
  friend void  protobuf_AddDesc_section_2eproto();
  friend void protobuf_AssignDesc_section_2eproto();
  friend void protobuf_ShutdownFile_section_2eproto();

  void InitAsDefaultInstance();
  static Section* default_instance_;
};
// ===================================================================


// ===================================================================

// Section

// required .hdmap_proto.Id id = 1;
inline bool Section::has_id() const {
  return (_has_bits_[0] & 0x00000001u) != 0;
}
inline void Section::set_has_id() {
  _has_bits_[0] |= 0x00000001u;
}
inline void Section::clear_has_id() {
  _has_bits_[0] &= ~0x00000001u;
}
inline void Section::clear_id() {
  if (id_ != NULL) id_->::hdmap_proto::Id::Clear();
  clear_has_id();
}
inline const ::hdmap_proto::Id& Section::id() const {
  // @@protoc_insertion_point(field_get:hdmap_proto.Section.id)
  return id_ != NULL ? *id_ : *default_instance_->id_;
}
inline ::hdmap_proto::Id* Section::mutable_id() {
  set_has_id();
  if (id_ == NULL) id_ = new ::hdmap_proto::Id;
  // @@protoc_insertion_point(field_mutable:hdmap_proto.Section.id)
  return id_;
}
inline ::hdmap_proto::Id* Section::release_id() {
  clear_has_id();
  ::hdmap_proto::Id* temp = id_;
  id_ = NULL;
  return temp;
}
inline void Section::set_allocated_id(::hdmap_proto::Id* id) {
  delete id_;
  id_ = id;
  if (id) {
    set_has_id();
  } else {
    clear_has_id();
  }
  // @@protoc_insertion_point(field_set_allocated:hdmap_proto.Section.id)
}

// optional .hdmap_proto.Section.LaneDirection direction = 2;
inline bool Section::has_direction() const {
  return (_has_bits_[0] & 0x00000002u) != 0;
}
inline void Section::set_has_direction() {
  _has_bits_[0] |= 0x00000002u;
}
inline void Section::clear_has_direction() {
  _has_bits_[0] &= ~0x00000002u;
}
inline void Section::clear_direction() {
  direction_ = 1;
  clear_has_direction();
}
inline ::hdmap_proto::Section_LaneDirection Section::direction() const {
  // @@protoc_insertion_point(field_get:hdmap_proto.Section.direction)
  return static_cast< ::hdmap_proto::Section_LaneDirection >(direction_);
}
inline void Section::set_direction(::hdmap_proto::Section_LaneDirection value) {
  assert(::hdmap_proto::Section_LaneDirection_IsValid(value));
  set_has_direction();
  direction_ = value;
  // @@protoc_insertion_point(field_set:hdmap_proto.Section.direction)
}

// repeated .hdmap_proto.CurveLine lines = 3;
inline int Section::lines_size() const {
  return lines_.size();
}
inline void Section::clear_lines() {
  lines_.Clear();
}
inline const ::hdmap_proto::CurveLine& Section::lines(int index) const {
  // @@protoc_insertion_point(field_get:hdmap_proto.Section.lines)
  return lines_.Get(index);
}
inline ::hdmap_proto::CurveLine* Section::mutable_lines(int index) {
  // @@protoc_insertion_point(field_mutable:hdmap_proto.Section.lines)
  return lines_.Mutable(index);
}
inline ::hdmap_proto::CurveLine* Section::add_lines() {
  // @@protoc_insertion_point(field_add:hdmap_proto.Section.lines)
  return lines_.Add();
}
inline const ::google::protobuf::RepeatedPtrField< ::hdmap_proto::CurveLine >&
Section::lines() const {
  // @@protoc_insertion_point(field_list:hdmap_proto.Section.lines)
  return lines_;
}
inline ::google::protobuf::RepeatedPtrField< ::hdmap_proto::CurveLine >*
Section::mutable_lines() {
  // @@protoc_insertion_point(field_mutable_list:hdmap_proto.Section.lines)
  return &lines_;
}

// repeated uint32 pred_indices = 10;
inline int Section::pred_indices_size() const {
  return pred_indices_.size();
}
inline void Section::clear_pred_indices() {
  pred_indices_.Clear();
}
inline ::google::protobuf::uint32 Section::pred_indices(int index) const {
  // @@protoc_insertion_point(field_get:hdmap_proto.Section.pred_indices)
  return pred_indices_.Get(index);
}
inline void Section::set_pred_indices(int index, ::google::protobuf::uint32 value) {
  pred_indices_.Set(index, value);
  // @@protoc_insertion_point(field_set:hdmap_proto.Section.pred_indices)
}
inline void Section::add_pred_indices(::google::protobuf::uint32 value) {
  pred_indices_.Add(value);
  // @@protoc_insertion_point(field_add:hdmap_proto.Section.pred_indices)
}
inline const ::google::protobuf::RepeatedField< ::google::protobuf::uint32 >&
Section::pred_indices() const {
  // @@protoc_insertion_point(field_list:hdmap_proto.Section.pred_indices)
  return pred_indices_;
}
inline ::google::protobuf::RepeatedField< ::google::protobuf::uint32 >*
Section::mutable_pred_indices() {
  // @@protoc_insertion_point(field_mutable_list:hdmap_proto.Section.pred_indices)
  return &pred_indices_;
}

// repeated uint32 succ_indices = 11;
inline int Section::succ_indices_size() const {
  return succ_indices_.size();
}
inline void Section::clear_succ_indices() {
  succ_indices_.Clear();
}
inline ::google::protobuf::uint32 Section::succ_indices(int index) const {
  // @@protoc_insertion_point(field_get:hdmap_proto.Section.succ_indices)
  return succ_indices_.Get(index);
}
inline void Section::set_succ_indices(int index, ::google::protobuf::uint32 value) {
  succ_indices_.Set(index, value);
  // @@protoc_insertion_point(field_set:hdmap_proto.Section.succ_indices)
}
inline void Section::add_succ_indices(::google::protobuf::uint32 value) {
  succ_indices_.Add(value);
  // @@protoc_insertion_point(field_add:hdmap_proto.Section.succ_indices)
}
inline const ::google::protobuf::RepeatedField< ::google::protobuf::uint32 >&
Section::succ_indices() const {
  // @@protoc_insertion_point(field_list:hdmap_proto.Section.succ_indices)
  return succ_indices_;
}
inline ::google::protobuf::RepeatedField< ::google::protobuf::uint32 >*
Section::mutable_succ_indices() {
  // @@protoc_insertion_point(field_mutable_list:hdmap_proto.Section.succ_indices)
  return &succ_indices_;
}


// @@protoc_insertion_point(namespace_scope)

}  // namespace hdmap_proto

#ifndef SWIG
namespace google {
namespace protobuf {

template <> struct is_proto_enum< ::hdmap_proto::Section_LaneDirection> : ::google::protobuf::internal::true_type {};
template <>
inline const EnumDescriptor* GetEnumDescriptor< ::hdmap_proto::Section_LaneDirection>() {
  return ::hdmap_proto::Section_LaneDirection_descriptor();
}

}  // namespace google
}  // namespace protobuf
#endif  // SWIG

// @@protoc_insertion_point(global_scope)

#endif  // PROTOBUF_section_2eproto__INCLUDED
