// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: id.proto

#ifndef PROTOBUF_id_2eproto__INCLUDED
#define PROTOBUF_id_2eproto__INCLUDED

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
// @@protoc_insertion_point(includes)

namespace hdmap_proto {

// Internal implementation detail -- do not call these.
void  protobuf_AddDesc_id_2eproto();
void protobuf_AssignDesc_id_2eproto();
void protobuf_ShutdownFile_id_2eproto();

class Id;

// ===================================================================

class Id : public ::google::protobuf::Message {
 public:
  Id();
  virtual ~Id();

  Id(const Id& from);

  inline Id& operator=(const Id& from) {
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
  static const Id& default_instance();

  void Swap(Id* other);

  // implements Message ----------------------------------------------

  Id* New() const;
  void CopyFrom(const ::google::protobuf::Message& from);
  void MergeFrom(const ::google::protobuf::Message& from);
  void CopyFrom(const Id& from);
  void MergeFrom(const Id& from);
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

  // required uint64 id = 1;
  inline bool has_id() const;
  inline void clear_id();
  static const int kIdFieldNumber = 1;
  inline ::google::protobuf::uint64 id() const;
  inline void set_id(::google::protobuf::uint64 value);

  // optional string name = 2;
  inline bool has_name() const;
  inline void clear_name();
  static const int kNameFieldNumber = 2;
  inline const ::std::string& name() const;
  inline void set_name(const ::std::string& value);
  inline void set_name(const char* value);
  inline void set_name(const char* value, size_t size);
  inline ::std::string* mutable_name();
  inline ::std::string* release_name();
  inline void set_allocated_name(::std::string* name);

  // @@protoc_insertion_point(class_scope:hdmap_proto.Id)
 private:
  inline void set_has_id();
  inline void clear_has_id();
  inline void set_has_name();
  inline void clear_has_name();

  ::google::protobuf::UnknownFieldSet _unknown_fields_;

  ::google::protobuf::uint32 _has_bits_[1];
  mutable int _cached_size_;
  ::google::protobuf::uint64 id_;
  ::std::string* name_;
  friend void  protobuf_AddDesc_id_2eproto();
  friend void protobuf_AssignDesc_id_2eproto();
  friend void protobuf_ShutdownFile_id_2eproto();

  void InitAsDefaultInstance();
  static Id* default_instance_;
};
// ===================================================================


// ===================================================================

// Id

// required uint64 id = 1;
inline bool Id::has_id() const {
  return (_has_bits_[0] & 0x00000001u) != 0;
}
inline void Id::set_has_id() {
  _has_bits_[0] |= 0x00000001u;
}
inline void Id::clear_has_id() {
  _has_bits_[0] &= ~0x00000001u;
}
inline void Id::clear_id() {
  id_ = GOOGLE_ULONGLONG(0);
  clear_has_id();
}
inline ::google::protobuf::uint64 Id::id() const {
  // @@protoc_insertion_point(field_get:hdmap_proto.Id.id)
  return id_;
}
inline void Id::set_id(::google::protobuf::uint64 value) {
  set_has_id();
  id_ = value;
  // @@protoc_insertion_point(field_set:hdmap_proto.Id.id)
}

// optional string name = 2;
inline bool Id::has_name() const {
  return (_has_bits_[0] & 0x00000002u) != 0;
}
inline void Id::set_has_name() {
  _has_bits_[0] |= 0x00000002u;
}
inline void Id::clear_has_name() {
  _has_bits_[0] &= ~0x00000002u;
}
inline void Id::clear_name() {
  if (name_ != &::google::protobuf::internal::GetEmptyStringAlreadyInited()) {
    name_->clear();
  }
  clear_has_name();
}
inline const ::std::string& Id::name() const {
  // @@protoc_insertion_point(field_get:hdmap_proto.Id.name)
  return *name_;
}
inline void Id::set_name(const ::std::string& value) {
  set_has_name();
  if (name_ == &::google::protobuf::internal::GetEmptyStringAlreadyInited()) {
    name_ = new ::std::string;
  }
  name_->assign(value);
  // @@protoc_insertion_point(field_set:hdmap_proto.Id.name)
}
inline void Id::set_name(const char* value) {
  set_has_name();
  if (name_ == &::google::protobuf::internal::GetEmptyStringAlreadyInited()) {
    name_ = new ::std::string;
  }
  name_->assign(value);
  // @@protoc_insertion_point(field_set_char:hdmap_proto.Id.name)
}
inline void Id::set_name(const char* value, size_t size) {
  set_has_name();
  if (name_ == &::google::protobuf::internal::GetEmptyStringAlreadyInited()) {
    name_ = new ::std::string;
  }
  name_->assign(reinterpret_cast<const char*>(value), size);
  // @@protoc_insertion_point(field_set_pointer:hdmap_proto.Id.name)
}
inline ::std::string* Id::mutable_name() {
  set_has_name();
  if (name_ == &::google::protobuf::internal::GetEmptyStringAlreadyInited()) {
    name_ = new ::std::string;
  }
  // @@protoc_insertion_point(field_mutable:hdmap_proto.Id.name)
  return name_;
}
inline ::std::string* Id::release_name() {
  clear_has_name();
  if (name_ == &::google::protobuf::internal::GetEmptyStringAlreadyInited()) {
    return NULL;
  } else {
    ::std::string* temp = name_;
    name_ = const_cast< ::std::string*>(&::google::protobuf::internal::GetEmptyStringAlreadyInited());
    return temp;
  }
}
inline void Id::set_allocated_name(::std::string* name) {
  if (name_ != &::google::protobuf::internal::GetEmptyStringAlreadyInited()) {
    delete name_;
  }
  if (name) {
    set_has_name();
    name_ = name;
  } else {
    clear_has_name();
    name_ = const_cast< ::std::string*>(&::google::protobuf::internal::GetEmptyStringAlreadyInited());
  }
  // @@protoc_insertion_point(field_set_allocated:hdmap_proto.Id.name)
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

#endif  // PROTOBUF_id_2eproto__INCLUDED
