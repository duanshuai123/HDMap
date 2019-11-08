// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: geometry.proto

#define INTERNAL_SUPPRESS_PROTOBUF_FIELD_DEPRECATION
#include "geometry.pb.h"

#include <algorithm>

#include <google/protobuf/stubs/common.h>
#include <google/protobuf/stubs/once.h>
#include <google/protobuf/io/coded_stream.h>
#include <google/protobuf/wire_format_lite_inl.h>
#include <google/protobuf/descriptor.h>
#include <google/protobuf/generated_message_reflection.h>
#include <google/protobuf/reflection_ops.h>
#include <google/protobuf/wire_format.h>
// @@protoc_insertion_point(includes)

namespace hdmap_proto {

namespace {

const ::google::protobuf::Descriptor* Vector3d_descriptor_ = NULL;
const ::google::protobuf::internal::GeneratedMessageReflection*
  Vector3d_reflection_ = NULL;
const ::google::protobuf::Descriptor* CurveLine_descriptor_ = NULL;
const ::google::protobuf::internal::GeneratedMessageReflection*
  CurveLine_reflection_ = NULL;
const ::google::protobuf::Descriptor* Polygon_descriptor_ = NULL;
const ::google::protobuf::internal::GeneratedMessageReflection*
  Polygon_reflection_ = NULL;

}  // namespace


void protobuf_AssignDesc_geometry_2eproto() {
  protobuf_AddDesc_geometry_2eproto();
  const ::google::protobuf::FileDescriptor* file =
    ::google::protobuf::DescriptorPool::generated_pool()->FindFileByName(
      "geometry.proto");
  GOOGLE_CHECK(file != NULL);
  Vector3d_descriptor_ = file->message_type(0);
  static const int Vector3d_offsets_[3] = {
    GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(Vector3d, x_),
    GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(Vector3d, y_),
    GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(Vector3d, z_),
  };
  Vector3d_reflection_ =
    new ::google::protobuf::internal::GeneratedMessageReflection(
      Vector3d_descriptor_,
      Vector3d::default_instance_,
      Vector3d_offsets_,
      GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(Vector3d, _has_bits_[0]),
      GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(Vector3d, _unknown_fields_),
      -1,
      ::google::protobuf::DescriptorPool::generated_pool(),
      ::google::protobuf::MessageFactory::generated_factory(),
      sizeof(Vector3d));
  CurveLine_descriptor_ = file->message_type(1);
  static const int CurveLine_offsets_[4] = {
    GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(CurveLine, points_),
    GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(CurveLine, pred_points_),
    GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(CurveLine, succ_points_),
    GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(CurveLine, width_),
  };
  CurveLine_reflection_ =
    new ::google::protobuf::internal::GeneratedMessageReflection(
      CurveLine_descriptor_,
      CurveLine::default_instance_,
      CurveLine_offsets_,
      GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(CurveLine, _has_bits_[0]),
      GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(CurveLine, _unknown_fields_),
      -1,
      ::google::protobuf::DescriptorPool::generated_pool(),
      ::google::protobuf::MessageFactory::generated_factory(),
      sizeof(CurveLine));
  Polygon_descriptor_ = file->message_type(2);
  static const int Polygon_offsets_[2] = {
    GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(Polygon, points_),
    GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(Polygon, width_),
  };
  Polygon_reflection_ =
    new ::google::protobuf::internal::GeneratedMessageReflection(
      Polygon_descriptor_,
      Polygon::default_instance_,
      Polygon_offsets_,
      GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(Polygon, _has_bits_[0]),
      GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(Polygon, _unknown_fields_),
      -1,
      ::google::protobuf::DescriptorPool::generated_pool(),
      ::google::protobuf::MessageFactory::generated_factory(),
      sizeof(Polygon));
}

namespace {

GOOGLE_PROTOBUF_DECLARE_ONCE(protobuf_AssignDescriptors_once_);
inline void protobuf_AssignDescriptorsOnce() {
  ::google::protobuf::GoogleOnceInit(&protobuf_AssignDescriptors_once_,
                 &protobuf_AssignDesc_geometry_2eproto);
}

void protobuf_RegisterTypes(const ::std::string&) {
  protobuf_AssignDescriptorsOnce();
  ::google::protobuf::MessageFactory::InternalRegisterGeneratedMessage(
    Vector3d_descriptor_, &Vector3d::default_instance());
  ::google::protobuf::MessageFactory::InternalRegisterGeneratedMessage(
    CurveLine_descriptor_, &CurveLine::default_instance());
  ::google::protobuf::MessageFactory::InternalRegisterGeneratedMessage(
    Polygon_descriptor_, &Polygon::default_instance());
}

}  // namespace

void protobuf_ShutdownFile_geometry_2eproto() {
  delete Vector3d::default_instance_;
  delete Vector3d_reflection_;
  delete CurveLine::default_instance_;
  delete CurveLine_reflection_;
  delete Polygon::default_instance_;
  delete Polygon_reflection_;
}

void protobuf_AddDesc_geometry_2eproto() {
  static bool already_here = false;
  if (already_here) return;
  already_here = true;
  GOOGLE_PROTOBUF_VERIFY_VERSION;

  ::google::protobuf::DescriptorPool::InternalAddGeneratedFile(
    "\n\016geometry.proto\022\013hdmap_proto\"+\n\010Vector3"
    "d\022\t\n\001x\030\001 \002(\001\022\t\n\001y\030\002 \002(\001\022\t\n\001z\030\003 \002(\001\"\231\001\n\tC"
    "urveLine\022%\n\006points\030\001 \003(\0132\025.hdmap_proto.V"
    "ector3d\022*\n\013pred_points\030\002 \003(\0132\025.hdmap_pro"
    "to.Vector3d\022*\n\013succ_points\030\003 \003(\0132\025.hdmap"
    "_proto.Vector3d\022\r\n\005width\030\004 \001(\002\"\?\n\007Polygo"
    "n\022%\n\006points\030\001 \003(\0132\025.hdmap_proto.Vector3d"
    "\022\r\n\005width\030\002 \001(\002", 295);
  ::google::protobuf::MessageFactory::InternalRegisterGeneratedFile(
    "geometry.proto", &protobuf_RegisterTypes);
  Vector3d::default_instance_ = new Vector3d();
  CurveLine::default_instance_ = new CurveLine();
  Polygon::default_instance_ = new Polygon();
  Vector3d::default_instance_->InitAsDefaultInstance();
  CurveLine::default_instance_->InitAsDefaultInstance();
  Polygon::default_instance_->InitAsDefaultInstance();
  ::google::protobuf::internal::OnShutdown(&protobuf_ShutdownFile_geometry_2eproto);
}

// Force AddDescriptors() to be called at static initialization time.
struct StaticDescriptorInitializer_geometry_2eproto {
  StaticDescriptorInitializer_geometry_2eproto() {
    protobuf_AddDesc_geometry_2eproto();
  }
} static_descriptor_initializer_geometry_2eproto_;

// ===================================================================

#ifndef _MSC_VER
const int Vector3d::kXFieldNumber;
const int Vector3d::kYFieldNumber;
const int Vector3d::kZFieldNumber;
#endif  // !_MSC_VER

Vector3d::Vector3d()
  : ::google::protobuf::Message() {
  SharedCtor();
  // @@protoc_insertion_point(constructor:hdmap_proto.Vector3d)
}

void Vector3d::InitAsDefaultInstance() {
}

Vector3d::Vector3d(const Vector3d& from)
  : ::google::protobuf::Message() {
  SharedCtor();
  MergeFrom(from);
  // @@protoc_insertion_point(copy_constructor:hdmap_proto.Vector3d)
}

void Vector3d::SharedCtor() {
  _cached_size_ = 0;
  x_ = 0;
  y_ = 0;
  z_ = 0;
  ::memset(_has_bits_, 0, sizeof(_has_bits_));
}

Vector3d::~Vector3d() {
  // @@protoc_insertion_point(destructor:hdmap_proto.Vector3d)
  SharedDtor();
}

void Vector3d::SharedDtor() {
  if (this != default_instance_) {
  }
}

void Vector3d::SetCachedSize(int size) const {
  GOOGLE_SAFE_CONCURRENT_WRITES_BEGIN();
  _cached_size_ = size;
  GOOGLE_SAFE_CONCURRENT_WRITES_END();
}
const ::google::protobuf::Descriptor* Vector3d::descriptor() {
  protobuf_AssignDescriptorsOnce();
  return Vector3d_descriptor_;
}

const Vector3d& Vector3d::default_instance() {
  if (default_instance_ == NULL) protobuf_AddDesc_geometry_2eproto();
  return *default_instance_;
}

Vector3d* Vector3d::default_instance_ = NULL;

Vector3d* Vector3d::New() const {
  return new Vector3d;
}

void Vector3d::Clear() {
#define OFFSET_OF_FIELD_(f) (reinterpret_cast<char*>(      \
  &reinterpret_cast<Vector3d*>(16)->f) - \
   reinterpret_cast<char*>(16))

#define ZR_(first, last) do {                              \
    size_t f = OFFSET_OF_FIELD_(first);                    \
    size_t n = OFFSET_OF_FIELD_(last) - f + sizeof(last);  \
    ::memset(&first, 0, n);                                \
  } while (0)

  ZR_(x_, z_);

#undef OFFSET_OF_FIELD_
#undef ZR_

  ::memset(_has_bits_, 0, sizeof(_has_bits_));
  mutable_unknown_fields()->Clear();
}

bool Vector3d::MergePartialFromCodedStream(
    ::google::protobuf::io::CodedInputStream* input) {
#define DO_(EXPRESSION) if (!(EXPRESSION)) goto failure
  ::google::protobuf::uint32 tag;
  // @@protoc_insertion_point(parse_start:hdmap_proto.Vector3d)
  for (;;) {
    ::std::pair< ::google::protobuf::uint32, bool> p = input->ReadTagWithCutoff(127);
    tag = p.first;
    if (!p.second) goto handle_unusual;
    switch (::google::protobuf::internal::WireFormatLite::GetTagFieldNumber(tag)) {
      // required double x = 1;
      case 1: {
        if (tag == 9) {
          DO_((::google::protobuf::internal::WireFormatLite::ReadPrimitive<
                   double, ::google::protobuf::internal::WireFormatLite::TYPE_DOUBLE>(
                 input, &x_)));
          set_has_x();
        } else {
          goto handle_unusual;
        }
        if (input->ExpectTag(17)) goto parse_y;
        break;
      }

      // required double y = 2;
      case 2: {
        if (tag == 17) {
         parse_y:
          DO_((::google::protobuf::internal::WireFormatLite::ReadPrimitive<
                   double, ::google::protobuf::internal::WireFormatLite::TYPE_DOUBLE>(
                 input, &y_)));
          set_has_y();
        } else {
          goto handle_unusual;
        }
        if (input->ExpectTag(25)) goto parse_z;
        break;
      }

      // required double z = 3;
      case 3: {
        if (tag == 25) {
         parse_z:
          DO_((::google::protobuf::internal::WireFormatLite::ReadPrimitive<
                   double, ::google::protobuf::internal::WireFormatLite::TYPE_DOUBLE>(
                 input, &z_)));
          set_has_z();
        } else {
          goto handle_unusual;
        }
        if (input->ExpectAtEnd()) goto success;
        break;
      }

      default: {
      handle_unusual:
        if (tag == 0 ||
            ::google::protobuf::internal::WireFormatLite::GetTagWireType(tag) ==
            ::google::protobuf::internal::WireFormatLite::WIRETYPE_END_GROUP) {
          goto success;
        }
        DO_(::google::protobuf::internal::WireFormat::SkipField(
              input, tag, mutable_unknown_fields()));
        break;
      }
    }
  }
success:
  // @@protoc_insertion_point(parse_success:hdmap_proto.Vector3d)
  return true;
failure:
  // @@protoc_insertion_point(parse_failure:hdmap_proto.Vector3d)
  return false;
#undef DO_
}

void Vector3d::SerializeWithCachedSizes(
    ::google::protobuf::io::CodedOutputStream* output) const {
  // @@protoc_insertion_point(serialize_start:hdmap_proto.Vector3d)
  // required double x = 1;
  if (has_x()) {
    ::google::protobuf::internal::WireFormatLite::WriteDouble(1, this->x(), output);
  }

  // required double y = 2;
  if (has_y()) {
    ::google::protobuf::internal::WireFormatLite::WriteDouble(2, this->y(), output);
  }

  // required double z = 3;
  if (has_z()) {
    ::google::protobuf::internal::WireFormatLite::WriteDouble(3, this->z(), output);
  }

  if (!unknown_fields().empty()) {
    ::google::protobuf::internal::WireFormat::SerializeUnknownFields(
        unknown_fields(), output);
  }
  // @@protoc_insertion_point(serialize_end:hdmap_proto.Vector3d)
}

::google::protobuf::uint8* Vector3d::SerializeWithCachedSizesToArray(
    ::google::protobuf::uint8* target) const {
  // @@protoc_insertion_point(serialize_to_array_start:hdmap_proto.Vector3d)
  // required double x = 1;
  if (has_x()) {
    target = ::google::protobuf::internal::WireFormatLite::WriteDoubleToArray(1, this->x(), target);
  }

  // required double y = 2;
  if (has_y()) {
    target = ::google::protobuf::internal::WireFormatLite::WriteDoubleToArray(2, this->y(), target);
  }

  // required double z = 3;
  if (has_z()) {
    target = ::google::protobuf::internal::WireFormatLite::WriteDoubleToArray(3, this->z(), target);
  }

  if (!unknown_fields().empty()) {
    target = ::google::protobuf::internal::WireFormat::SerializeUnknownFieldsToArray(
        unknown_fields(), target);
  }
  // @@protoc_insertion_point(serialize_to_array_end:hdmap_proto.Vector3d)
  return target;
}

int Vector3d::ByteSize() const {
  int total_size = 0;

  if (_has_bits_[0 / 32] & (0xffu << (0 % 32))) {
    // required double x = 1;
    if (has_x()) {
      total_size += 1 + 8;
    }

    // required double y = 2;
    if (has_y()) {
      total_size += 1 + 8;
    }

    // required double z = 3;
    if (has_z()) {
      total_size += 1 + 8;
    }

  }
  if (!unknown_fields().empty()) {
    total_size +=
      ::google::protobuf::internal::WireFormat::ComputeUnknownFieldsSize(
        unknown_fields());
  }
  GOOGLE_SAFE_CONCURRENT_WRITES_BEGIN();
  _cached_size_ = total_size;
  GOOGLE_SAFE_CONCURRENT_WRITES_END();
  return total_size;
}

void Vector3d::MergeFrom(const ::google::protobuf::Message& from) {
  GOOGLE_CHECK_NE(&from, this);
  const Vector3d* source =
    ::google::protobuf::internal::dynamic_cast_if_available<const Vector3d*>(
      &from);
  if (source == NULL) {
    ::google::protobuf::internal::ReflectionOps::Merge(from, this);
  } else {
    MergeFrom(*source);
  }
}

void Vector3d::MergeFrom(const Vector3d& from) {
  GOOGLE_CHECK_NE(&from, this);
  if (from._has_bits_[0 / 32] & (0xffu << (0 % 32))) {
    if (from.has_x()) {
      set_x(from.x());
    }
    if (from.has_y()) {
      set_y(from.y());
    }
    if (from.has_z()) {
      set_z(from.z());
    }
  }
  mutable_unknown_fields()->MergeFrom(from.unknown_fields());
}

void Vector3d::CopyFrom(const ::google::protobuf::Message& from) {
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

void Vector3d::CopyFrom(const Vector3d& from) {
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

bool Vector3d::IsInitialized() const {
  if ((_has_bits_[0] & 0x00000007) != 0x00000007) return false;

  return true;
}

void Vector3d::Swap(Vector3d* other) {
  if (other != this) {
    std::swap(x_, other->x_);
    std::swap(y_, other->y_);
    std::swap(z_, other->z_);
    std::swap(_has_bits_[0], other->_has_bits_[0]);
    _unknown_fields_.Swap(&other->_unknown_fields_);
    std::swap(_cached_size_, other->_cached_size_);
  }
}

::google::protobuf::Metadata Vector3d::GetMetadata() const {
  protobuf_AssignDescriptorsOnce();
  ::google::protobuf::Metadata metadata;
  metadata.descriptor = Vector3d_descriptor_;
  metadata.reflection = Vector3d_reflection_;
  return metadata;
}


// ===================================================================

#ifndef _MSC_VER
const int CurveLine::kPointsFieldNumber;
const int CurveLine::kPredPointsFieldNumber;
const int CurveLine::kSuccPointsFieldNumber;
const int CurveLine::kWidthFieldNumber;
#endif  // !_MSC_VER

CurveLine::CurveLine()
  : ::google::protobuf::Message() {
  SharedCtor();
  // @@protoc_insertion_point(constructor:hdmap_proto.CurveLine)
}

void CurveLine::InitAsDefaultInstance() {
}

CurveLine::CurveLine(const CurveLine& from)
  : ::google::protobuf::Message() {
  SharedCtor();
  MergeFrom(from);
  // @@protoc_insertion_point(copy_constructor:hdmap_proto.CurveLine)
}

void CurveLine::SharedCtor() {
  _cached_size_ = 0;
  width_ = 0;
  ::memset(_has_bits_, 0, sizeof(_has_bits_));
}

CurveLine::~CurveLine() {
  // @@protoc_insertion_point(destructor:hdmap_proto.CurveLine)
  SharedDtor();
}

void CurveLine::SharedDtor() {
  if (this != default_instance_) {
  }
}

void CurveLine::SetCachedSize(int size) const {
  GOOGLE_SAFE_CONCURRENT_WRITES_BEGIN();
  _cached_size_ = size;
  GOOGLE_SAFE_CONCURRENT_WRITES_END();
}
const ::google::protobuf::Descriptor* CurveLine::descriptor() {
  protobuf_AssignDescriptorsOnce();
  return CurveLine_descriptor_;
}

const CurveLine& CurveLine::default_instance() {
  if (default_instance_ == NULL) protobuf_AddDesc_geometry_2eproto();
  return *default_instance_;
}

CurveLine* CurveLine::default_instance_ = NULL;

CurveLine* CurveLine::New() const {
  return new CurveLine;
}

void CurveLine::Clear() {
  width_ = 0;
  points_.Clear();
  pred_points_.Clear();
  succ_points_.Clear();
  ::memset(_has_bits_, 0, sizeof(_has_bits_));
  mutable_unknown_fields()->Clear();
}

bool CurveLine::MergePartialFromCodedStream(
    ::google::protobuf::io::CodedInputStream* input) {
#define DO_(EXPRESSION) if (!(EXPRESSION)) goto failure
  ::google::protobuf::uint32 tag;
  // @@protoc_insertion_point(parse_start:hdmap_proto.CurveLine)
  for (;;) {
    ::std::pair< ::google::protobuf::uint32, bool> p = input->ReadTagWithCutoff(127);
    tag = p.first;
    if (!p.second) goto handle_unusual;
    switch (::google::protobuf::internal::WireFormatLite::GetTagFieldNumber(tag)) {
      // repeated .hdmap_proto.Vector3d points = 1;
      case 1: {
        if (tag == 10) {
         parse_points:
          DO_(::google::protobuf::internal::WireFormatLite::ReadMessageNoVirtual(
                input, add_points()));
        } else {
          goto handle_unusual;
        }
        if (input->ExpectTag(10)) goto parse_points;
        if (input->ExpectTag(18)) goto parse_pred_points;
        break;
      }

      // repeated .hdmap_proto.Vector3d pred_points = 2;
      case 2: {
        if (tag == 18) {
         parse_pred_points:
          DO_(::google::protobuf::internal::WireFormatLite::ReadMessageNoVirtual(
                input, add_pred_points()));
        } else {
          goto handle_unusual;
        }
        if (input->ExpectTag(18)) goto parse_pred_points;
        if (input->ExpectTag(26)) goto parse_succ_points;
        break;
      }

      // repeated .hdmap_proto.Vector3d succ_points = 3;
      case 3: {
        if (tag == 26) {
         parse_succ_points:
          DO_(::google::protobuf::internal::WireFormatLite::ReadMessageNoVirtual(
                input, add_succ_points()));
        } else {
          goto handle_unusual;
        }
        if (input->ExpectTag(26)) goto parse_succ_points;
        if (input->ExpectTag(37)) goto parse_width;
        break;
      }

      // optional float width = 4;
      case 4: {
        if (tag == 37) {
         parse_width:
          DO_((::google::protobuf::internal::WireFormatLite::ReadPrimitive<
                   float, ::google::protobuf::internal::WireFormatLite::TYPE_FLOAT>(
                 input, &width_)));
          set_has_width();
        } else {
          goto handle_unusual;
        }
        if (input->ExpectAtEnd()) goto success;
        break;
      }

      default: {
      handle_unusual:
        if (tag == 0 ||
            ::google::protobuf::internal::WireFormatLite::GetTagWireType(tag) ==
            ::google::protobuf::internal::WireFormatLite::WIRETYPE_END_GROUP) {
          goto success;
        }
        DO_(::google::protobuf::internal::WireFormat::SkipField(
              input, tag, mutable_unknown_fields()));
        break;
      }
    }
  }
success:
  // @@protoc_insertion_point(parse_success:hdmap_proto.CurveLine)
  return true;
failure:
  // @@protoc_insertion_point(parse_failure:hdmap_proto.CurveLine)
  return false;
#undef DO_
}

void CurveLine::SerializeWithCachedSizes(
    ::google::protobuf::io::CodedOutputStream* output) const {
  // @@protoc_insertion_point(serialize_start:hdmap_proto.CurveLine)
  // repeated .hdmap_proto.Vector3d points = 1;
  for (int i = 0; i < this->points_size(); i++) {
    ::google::protobuf::internal::WireFormatLite::WriteMessageMaybeToArray(
      1, this->points(i), output);
  }

  // repeated .hdmap_proto.Vector3d pred_points = 2;
  for (int i = 0; i < this->pred_points_size(); i++) {
    ::google::protobuf::internal::WireFormatLite::WriteMessageMaybeToArray(
      2, this->pred_points(i), output);
  }

  // repeated .hdmap_proto.Vector3d succ_points = 3;
  for (int i = 0; i < this->succ_points_size(); i++) {
    ::google::protobuf::internal::WireFormatLite::WriteMessageMaybeToArray(
      3, this->succ_points(i), output);
  }

  // optional float width = 4;
  if (has_width()) {
    ::google::protobuf::internal::WireFormatLite::WriteFloat(4, this->width(), output);
  }

  if (!unknown_fields().empty()) {
    ::google::protobuf::internal::WireFormat::SerializeUnknownFields(
        unknown_fields(), output);
  }
  // @@protoc_insertion_point(serialize_end:hdmap_proto.CurveLine)
}

::google::protobuf::uint8* CurveLine::SerializeWithCachedSizesToArray(
    ::google::protobuf::uint8* target) const {
  // @@protoc_insertion_point(serialize_to_array_start:hdmap_proto.CurveLine)
  // repeated .hdmap_proto.Vector3d points = 1;
  for (int i = 0; i < this->points_size(); i++) {
    target = ::google::protobuf::internal::WireFormatLite::
      WriteMessageNoVirtualToArray(
        1, this->points(i), target);
  }

  // repeated .hdmap_proto.Vector3d pred_points = 2;
  for (int i = 0; i < this->pred_points_size(); i++) {
    target = ::google::protobuf::internal::WireFormatLite::
      WriteMessageNoVirtualToArray(
        2, this->pred_points(i), target);
  }

  // repeated .hdmap_proto.Vector3d succ_points = 3;
  for (int i = 0; i < this->succ_points_size(); i++) {
    target = ::google::protobuf::internal::WireFormatLite::
      WriteMessageNoVirtualToArray(
        3, this->succ_points(i), target);
  }

  // optional float width = 4;
  if (has_width()) {
    target = ::google::protobuf::internal::WireFormatLite::WriteFloatToArray(4, this->width(), target);
  }

  if (!unknown_fields().empty()) {
    target = ::google::protobuf::internal::WireFormat::SerializeUnknownFieldsToArray(
        unknown_fields(), target);
  }
  // @@protoc_insertion_point(serialize_to_array_end:hdmap_proto.CurveLine)
  return target;
}

int CurveLine::ByteSize() const {
  int total_size = 0;

  if (_has_bits_[3 / 32] & (0xffu << (3 % 32))) {
    // optional float width = 4;
    if (has_width()) {
      total_size += 1 + 4;
    }

  }
  // repeated .hdmap_proto.Vector3d points = 1;
  total_size += 1 * this->points_size();
  for (int i = 0; i < this->points_size(); i++) {
    total_size +=
      ::google::protobuf::internal::WireFormatLite::MessageSizeNoVirtual(
        this->points(i));
  }

  // repeated .hdmap_proto.Vector3d pred_points = 2;
  total_size += 1 * this->pred_points_size();
  for (int i = 0; i < this->pred_points_size(); i++) {
    total_size +=
      ::google::protobuf::internal::WireFormatLite::MessageSizeNoVirtual(
        this->pred_points(i));
  }

  // repeated .hdmap_proto.Vector3d succ_points = 3;
  total_size += 1 * this->succ_points_size();
  for (int i = 0; i < this->succ_points_size(); i++) {
    total_size +=
      ::google::protobuf::internal::WireFormatLite::MessageSizeNoVirtual(
        this->succ_points(i));
  }

  if (!unknown_fields().empty()) {
    total_size +=
      ::google::protobuf::internal::WireFormat::ComputeUnknownFieldsSize(
        unknown_fields());
  }
  GOOGLE_SAFE_CONCURRENT_WRITES_BEGIN();
  _cached_size_ = total_size;
  GOOGLE_SAFE_CONCURRENT_WRITES_END();
  return total_size;
}

void CurveLine::MergeFrom(const ::google::protobuf::Message& from) {
  GOOGLE_CHECK_NE(&from, this);
  const CurveLine* source =
    ::google::protobuf::internal::dynamic_cast_if_available<const CurveLine*>(
      &from);
  if (source == NULL) {
    ::google::protobuf::internal::ReflectionOps::Merge(from, this);
  } else {
    MergeFrom(*source);
  }
}

void CurveLine::MergeFrom(const CurveLine& from) {
  GOOGLE_CHECK_NE(&from, this);
  points_.MergeFrom(from.points_);
  pred_points_.MergeFrom(from.pred_points_);
  succ_points_.MergeFrom(from.succ_points_);
  if (from._has_bits_[3 / 32] & (0xffu << (3 % 32))) {
    if (from.has_width()) {
      set_width(from.width());
    }
  }
  mutable_unknown_fields()->MergeFrom(from.unknown_fields());
}

void CurveLine::CopyFrom(const ::google::protobuf::Message& from) {
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

void CurveLine::CopyFrom(const CurveLine& from) {
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

bool CurveLine::IsInitialized() const {

  if (!::google::protobuf::internal::AllAreInitialized(this->points())) return false;
  if (!::google::protobuf::internal::AllAreInitialized(this->pred_points())) return false;
  if (!::google::protobuf::internal::AllAreInitialized(this->succ_points())) return false;
  return true;
}

void CurveLine::Swap(CurveLine* other) {
  if (other != this) {
    points_.Swap(&other->points_);
    pred_points_.Swap(&other->pred_points_);
    succ_points_.Swap(&other->succ_points_);
    std::swap(width_, other->width_);
    std::swap(_has_bits_[0], other->_has_bits_[0]);
    _unknown_fields_.Swap(&other->_unknown_fields_);
    std::swap(_cached_size_, other->_cached_size_);
  }
}

::google::protobuf::Metadata CurveLine::GetMetadata() const {
  protobuf_AssignDescriptorsOnce();
  ::google::protobuf::Metadata metadata;
  metadata.descriptor = CurveLine_descriptor_;
  metadata.reflection = CurveLine_reflection_;
  return metadata;
}


// ===================================================================

#ifndef _MSC_VER
const int Polygon::kPointsFieldNumber;
const int Polygon::kWidthFieldNumber;
#endif  // !_MSC_VER

Polygon::Polygon()
  : ::google::protobuf::Message() {
  SharedCtor();
  // @@protoc_insertion_point(constructor:hdmap_proto.Polygon)
}

void Polygon::InitAsDefaultInstance() {
}

Polygon::Polygon(const Polygon& from)
  : ::google::protobuf::Message() {
  SharedCtor();
  MergeFrom(from);
  // @@protoc_insertion_point(copy_constructor:hdmap_proto.Polygon)
}

void Polygon::SharedCtor() {
  _cached_size_ = 0;
  width_ = 0;
  ::memset(_has_bits_, 0, sizeof(_has_bits_));
}

Polygon::~Polygon() {
  // @@protoc_insertion_point(destructor:hdmap_proto.Polygon)
  SharedDtor();
}

void Polygon::SharedDtor() {
  if (this != default_instance_) {
  }
}

void Polygon::SetCachedSize(int size) const {
  GOOGLE_SAFE_CONCURRENT_WRITES_BEGIN();
  _cached_size_ = size;
  GOOGLE_SAFE_CONCURRENT_WRITES_END();
}
const ::google::protobuf::Descriptor* Polygon::descriptor() {
  protobuf_AssignDescriptorsOnce();
  return Polygon_descriptor_;
}

const Polygon& Polygon::default_instance() {
  if (default_instance_ == NULL) protobuf_AddDesc_geometry_2eproto();
  return *default_instance_;
}

Polygon* Polygon::default_instance_ = NULL;

Polygon* Polygon::New() const {
  return new Polygon;
}

void Polygon::Clear() {
  width_ = 0;
  points_.Clear();
  ::memset(_has_bits_, 0, sizeof(_has_bits_));
  mutable_unknown_fields()->Clear();
}

bool Polygon::MergePartialFromCodedStream(
    ::google::protobuf::io::CodedInputStream* input) {
#define DO_(EXPRESSION) if (!(EXPRESSION)) goto failure
  ::google::protobuf::uint32 tag;
  // @@protoc_insertion_point(parse_start:hdmap_proto.Polygon)
  for (;;) {
    ::std::pair< ::google::protobuf::uint32, bool> p = input->ReadTagWithCutoff(127);
    tag = p.first;
    if (!p.second) goto handle_unusual;
    switch (::google::protobuf::internal::WireFormatLite::GetTagFieldNumber(tag)) {
      // repeated .hdmap_proto.Vector3d points = 1;
      case 1: {
        if (tag == 10) {
         parse_points:
          DO_(::google::protobuf::internal::WireFormatLite::ReadMessageNoVirtual(
                input, add_points()));
        } else {
          goto handle_unusual;
        }
        if (input->ExpectTag(10)) goto parse_points;
        if (input->ExpectTag(21)) goto parse_width;
        break;
      }

      // optional float width = 2;
      case 2: {
        if (tag == 21) {
         parse_width:
          DO_((::google::protobuf::internal::WireFormatLite::ReadPrimitive<
                   float, ::google::protobuf::internal::WireFormatLite::TYPE_FLOAT>(
                 input, &width_)));
          set_has_width();
        } else {
          goto handle_unusual;
        }
        if (input->ExpectAtEnd()) goto success;
        break;
      }

      default: {
      handle_unusual:
        if (tag == 0 ||
            ::google::protobuf::internal::WireFormatLite::GetTagWireType(tag) ==
            ::google::protobuf::internal::WireFormatLite::WIRETYPE_END_GROUP) {
          goto success;
        }
        DO_(::google::protobuf::internal::WireFormat::SkipField(
              input, tag, mutable_unknown_fields()));
        break;
      }
    }
  }
success:
  // @@protoc_insertion_point(parse_success:hdmap_proto.Polygon)
  return true;
failure:
  // @@protoc_insertion_point(parse_failure:hdmap_proto.Polygon)
  return false;
#undef DO_
}

void Polygon::SerializeWithCachedSizes(
    ::google::protobuf::io::CodedOutputStream* output) const {
  // @@protoc_insertion_point(serialize_start:hdmap_proto.Polygon)
  // repeated .hdmap_proto.Vector3d points = 1;
  for (int i = 0; i < this->points_size(); i++) {
    ::google::protobuf::internal::WireFormatLite::WriteMessageMaybeToArray(
      1, this->points(i), output);
  }

  // optional float width = 2;
  if (has_width()) {
    ::google::protobuf::internal::WireFormatLite::WriteFloat(2, this->width(), output);
  }

  if (!unknown_fields().empty()) {
    ::google::protobuf::internal::WireFormat::SerializeUnknownFields(
        unknown_fields(), output);
  }
  // @@protoc_insertion_point(serialize_end:hdmap_proto.Polygon)
}

::google::protobuf::uint8* Polygon::SerializeWithCachedSizesToArray(
    ::google::protobuf::uint8* target) const {
  // @@protoc_insertion_point(serialize_to_array_start:hdmap_proto.Polygon)
  // repeated .hdmap_proto.Vector3d points = 1;
  for (int i = 0; i < this->points_size(); i++) {
    target = ::google::protobuf::internal::WireFormatLite::
      WriteMessageNoVirtualToArray(
        1, this->points(i), target);
  }

  // optional float width = 2;
  if (has_width()) {
    target = ::google::protobuf::internal::WireFormatLite::WriteFloatToArray(2, this->width(), target);
  }

  if (!unknown_fields().empty()) {
    target = ::google::protobuf::internal::WireFormat::SerializeUnknownFieldsToArray(
        unknown_fields(), target);
  }
  // @@protoc_insertion_point(serialize_to_array_end:hdmap_proto.Polygon)
  return target;
}

int Polygon::ByteSize() const {
  int total_size = 0;

  if (_has_bits_[1 / 32] & (0xffu << (1 % 32))) {
    // optional float width = 2;
    if (has_width()) {
      total_size += 1 + 4;
    }

  }
  // repeated .hdmap_proto.Vector3d points = 1;
  total_size += 1 * this->points_size();
  for (int i = 0; i < this->points_size(); i++) {
    total_size +=
      ::google::protobuf::internal::WireFormatLite::MessageSizeNoVirtual(
        this->points(i));
  }

  if (!unknown_fields().empty()) {
    total_size +=
      ::google::protobuf::internal::WireFormat::ComputeUnknownFieldsSize(
        unknown_fields());
  }
  GOOGLE_SAFE_CONCURRENT_WRITES_BEGIN();
  _cached_size_ = total_size;
  GOOGLE_SAFE_CONCURRENT_WRITES_END();
  return total_size;
}

void Polygon::MergeFrom(const ::google::protobuf::Message& from) {
  GOOGLE_CHECK_NE(&from, this);
  const Polygon* source =
    ::google::protobuf::internal::dynamic_cast_if_available<const Polygon*>(
      &from);
  if (source == NULL) {
    ::google::protobuf::internal::ReflectionOps::Merge(from, this);
  } else {
    MergeFrom(*source);
  }
}

void Polygon::MergeFrom(const Polygon& from) {
  GOOGLE_CHECK_NE(&from, this);
  points_.MergeFrom(from.points_);
  if (from._has_bits_[1 / 32] & (0xffu << (1 % 32))) {
    if (from.has_width()) {
      set_width(from.width());
    }
  }
  mutable_unknown_fields()->MergeFrom(from.unknown_fields());
}

void Polygon::CopyFrom(const ::google::protobuf::Message& from) {
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

void Polygon::CopyFrom(const Polygon& from) {
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

bool Polygon::IsInitialized() const {

  if (!::google::protobuf::internal::AllAreInitialized(this->points())) return false;
  return true;
}

void Polygon::Swap(Polygon* other) {
  if (other != this) {
    points_.Swap(&other->points_);
    std::swap(width_, other->width_);
    std::swap(_has_bits_[0], other->_has_bits_[0]);
    _unknown_fields_.Swap(&other->_unknown_fields_);
    std::swap(_cached_size_, other->_cached_size_);
  }
}

::google::protobuf::Metadata Polygon::GetMetadata() const {
  protobuf_AssignDescriptorsOnce();
  ::google::protobuf::Metadata metadata;
  metadata.descriptor = Polygon_descriptor_;
  metadata.reflection = Polygon_reflection_;
  return metadata;
}


// @@protoc_insertion_point(namespace_scope)

}  // namespace hdmap_proto

// @@protoc_insertion_point(global_scope)
