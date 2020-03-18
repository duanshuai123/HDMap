// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: section.proto

#define INTERNAL_SUPPRESS_PROTOBUF_FIELD_DEPRECATION
#include "section.pb.h"

#include <algorithm>

#include <google/protobuf/stubs/common.h>
#include <google/protobuf/stubs/port.h>
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

const ::google::protobuf::Descriptor* Section_descriptor_ = NULL;
const ::google::protobuf::internal::GeneratedMessageReflection*
  Section_reflection_ = NULL;
const ::google::protobuf::EnumDescriptor* Section_LaneDirection_descriptor_ = NULL;

}  // namespace


void protobuf_AssignDesc_section_2eproto() GOOGLE_ATTRIBUTE_COLD;
void protobuf_AssignDesc_section_2eproto() {
  protobuf_AddDesc_section_2eproto();
  const ::google::protobuf::FileDescriptor* file =
    ::google::protobuf::DescriptorPool::generated_pool()->FindFileByName(
      "section.proto");
  GOOGLE_CHECK(file != NULL);
  Section_descriptor_ = file->message_type(0);
  static const int Section_offsets_[5] = {
    GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(Section, id_),
    GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(Section, direction_),
    GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(Section, lanes_),
    GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(Section, pred_indices_),
    GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(Section, succ_indices_),
  };
  Section_reflection_ =
    ::google::protobuf::internal::GeneratedMessageReflection::NewGeneratedMessageReflection(
      Section_descriptor_,
      Section::default_instance_,
      Section_offsets_,
      GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(Section, _has_bits_[0]),
      -1,
      -1,
      sizeof(Section),
      GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(Section, _internal_metadata_),
      -1);
  Section_LaneDirection_descriptor_ = Section_descriptor_->enum_type(0);
}

namespace {

GOOGLE_PROTOBUF_DECLARE_ONCE(protobuf_AssignDescriptors_once_);
inline void protobuf_AssignDescriptorsOnce() {
  ::google::protobuf::GoogleOnceInit(&protobuf_AssignDescriptors_once_,
                 &protobuf_AssignDesc_section_2eproto);
}

void protobuf_RegisterTypes(const ::std::string&) GOOGLE_ATTRIBUTE_COLD;
void protobuf_RegisterTypes(const ::std::string&) {
  protobuf_AssignDescriptorsOnce();
  ::google::protobuf::MessageFactory::InternalRegisterGeneratedMessage(
      Section_descriptor_, &Section::default_instance());
}

}  // namespace

void protobuf_ShutdownFile_section_2eproto() {
  delete Section::default_instance_;
  delete Section_reflection_;
}

void protobuf_AddDesc_section_2eproto() GOOGLE_ATTRIBUTE_COLD;
void protobuf_AddDesc_section_2eproto() {
  static bool already_here = false;
  if (already_here) return;
  already_here = true;
  GOOGLE_PROTOBUF_VERIFY_VERSION;

  ::hdmap_proto::protobuf_AddDesc_id_2eproto();
  ::hdmap_proto::protobuf_AddDesc_lane_2eproto();
  ::google::protobuf::DescriptorPool::InternalAddGeneratedFile(
    "\n\rsection.proto\022\013hdmap_proto\032\010id.proto\032\n"
    "lane.proto\"\361\001\n\007Section\022\033\n\002id\030\001 \002(\0132\017.hdm"
    "ap_proto.Id\0225\n\tdirection\030\002 \001(\0162\".hdmap_p"
    "roto.Section.LaneDirection\022 \n\005lanes\030\003 \003("
    "\0132\021.hdmap_proto.Lane\022\024\n\014pred_indices\030\n \003"
    "(\r\022\024\n\014succ_indices\030\013 \003(\r\"D\n\rLaneDirectio"
    "n\022\014\n\010UN_KNOWN\020\000\022\013\n\007FORWARD\020\001\022\014\n\010BACKWARD"
    "\020\002\022\n\n\006TWOWAY\020\003", 294);
  ::google::protobuf::MessageFactory::InternalRegisterGeneratedFile(
    "section.proto", &protobuf_RegisterTypes);
  Section::default_instance_ = new Section();
  Section::default_instance_->InitAsDefaultInstance();
  ::google::protobuf::internal::OnShutdown(&protobuf_ShutdownFile_section_2eproto);
}

// Force AddDescriptors() to be called at static initialization time.
struct StaticDescriptorInitializer_section_2eproto {
  StaticDescriptorInitializer_section_2eproto() {
    protobuf_AddDesc_section_2eproto();
  }
} static_descriptor_initializer_section_2eproto_;

// ===================================================================

const ::google::protobuf::EnumDescriptor* Section_LaneDirection_descriptor() {
  protobuf_AssignDescriptorsOnce();
  return Section_LaneDirection_descriptor_;
}
bool Section_LaneDirection_IsValid(int value) {
  switch(value) {
    case 0:
    case 1:
    case 2:
    case 3:
      return true;
    default:
      return false;
  }
}

#if !defined(_MSC_VER) || _MSC_VER >= 1900
const Section_LaneDirection Section::UN_KNOWN;
const Section_LaneDirection Section::FORWARD;
const Section_LaneDirection Section::BACKWARD;
const Section_LaneDirection Section::TWOWAY;
const Section_LaneDirection Section::LaneDirection_MIN;
const Section_LaneDirection Section::LaneDirection_MAX;
const int Section::LaneDirection_ARRAYSIZE;
#endif  // !defined(_MSC_VER) || _MSC_VER >= 1900
#if !defined(_MSC_VER) || _MSC_VER >= 1900
const int Section::kIdFieldNumber;
const int Section::kDirectionFieldNumber;
const int Section::kLanesFieldNumber;
const int Section::kPredIndicesFieldNumber;
const int Section::kSuccIndicesFieldNumber;
#endif  // !defined(_MSC_VER) || _MSC_VER >= 1900

Section::Section()
  : ::google::protobuf::Message(), _internal_metadata_(NULL) {
  SharedCtor();
  // @@protoc_insertion_point(constructor:hdmap_proto.Section)
}

void Section::InitAsDefaultInstance() {
  id_ = const_cast< ::hdmap_proto::Id*>(&::hdmap_proto::Id::default_instance());
}

Section::Section(const Section& from)
  : ::google::protobuf::Message(),
    _internal_metadata_(NULL) {
  SharedCtor();
  MergeFrom(from);
  // @@protoc_insertion_point(copy_constructor:hdmap_proto.Section)
}

void Section::SharedCtor() {
  _cached_size_ = 0;
  id_ = NULL;
  direction_ = 0;
  ::memset(_has_bits_, 0, sizeof(_has_bits_));
}

Section::~Section() {
  // @@protoc_insertion_point(destructor:hdmap_proto.Section)
  SharedDtor();
}

void Section::SharedDtor() {
  if (this != default_instance_) {
    delete id_;
  }
}

void Section::SetCachedSize(int size) const {
  GOOGLE_SAFE_CONCURRENT_WRITES_BEGIN();
  _cached_size_ = size;
  GOOGLE_SAFE_CONCURRENT_WRITES_END();
}
const ::google::protobuf::Descriptor* Section::descriptor() {
  protobuf_AssignDescriptorsOnce();
  return Section_descriptor_;
}

const Section& Section::default_instance() {
  if (default_instance_ == NULL) protobuf_AddDesc_section_2eproto();
  return *default_instance_;
}

Section* Section::default_instance_ = NULL;

Section* Section::New(::google::protobuf::Arena* arena) const {
  Section* n = new Section;
  if (arena != NULL) {
    arena->Own(n);
  }
  return n;
}

void Section::Clear() {
// @@protoc_insertion_point(message_clear_start:hdmap_proto.Section)
  if (_has_bits_[0 / 32] & 3u) {
    if (has_id()) {
      if (id_ != NULL) id_->::hdmap_proto::Id::Clear();
    }
    direction_ = 0;
  }
  lanes_.Clear();
  pred_indices_.Clear();
  succ_indices_.Clear();
  ::memset(_has_bits_, 0, sizeof(_has_bits_));
  if (_internal_metadata_.have_unknown_fields()) {
    mutable_unknown_fields()->Clear();
  }
}

bool Section::MergePartialFromCodedStream(
    ::google::protobuf::io::CodedInputStream* input) {
#define DO_(EXPRESSION) if (!GOOGLE_PREDICT_TRUE(EXPRESSION)) goto failure
  ::google::protobuf::uint32 tag;
  // @@protoc_insertion_point(parse_start:hdmap_proto.Section)
  for (;;) {
    ::std::pair< ::google::protobuf::uint32, bool> p = input->ReadTagWithCutoff(127);
    tag = p.first;
    if (!p.second) goto handle_unusual;
    switch (::google::protobuf::internal::WireFormatLite::GetTagFieldNumber(tag)) {
      // required .hdmap_proto.Id id = 1;
      case 1: {
        if (tag == 10) {
          DO_(::google::protobuf::internal::WireFormatLite::ReadMessageNoVirtual(
               input, mutable_id()));
        } else {
          goto handle_unusual;
        }
        if (input->ExpectTag(16)) goto parse_direction;
        break;
      }

      // optional .hdmap_proto.Section.LaneDirection direction = 2;
      case 2: {
        if (tag == 16) {
         parse_direction:
          int value;
          DO_((::google::protobuf::internal::WireFormatLite::ReadPrimitive<
                   int, ::google::protobuf::internal::WireFormatLite::TYPE_ENUM>(
                 input, &value)));
          if (::hdmap_proto::Section_LaneDirection_IsValid(value)) {
            set_direction(static_cast< ::hdmap_proto::Section_LaneDirection >(value));
          } else {
            mutable_unknown_fields()->AddVarint(2, value);
          }
        } else {
          goto handle_unusual;
        }
        if (input->ExpectTag(26)) goto parse_lanes;
        break;
      }

      // repeated .hdmap_proto.Lane lanes = 3;
      case 3: {
        if (tag == 26) {
         parse_lanes:
          DO_(input->IncrementRecursionDepth());
         parse_loop_lanes:
          DO_(::google::protobuf::internal::WireFormatLite::ReadMessageNoVirtualNoRecursionDepth(
                input, add_lanes()));
        } else {
          goto handle_unusual;
        }
        if (input->ExpectTag(26)) goto parse_loop_lanes;
        input->UnsafeDecrementRecursionDepth();
        if (input->ExpectTag(80)) goto parse_pred_indices;
        break;
      }

      // repeated uint32 pred_indices = 10;
      case 10: {
        if (tag == 80) {
         parse_pred_indices:
          DO_((::google::protobuf::internal::WireFormatLite::ReadRepeatedPrimitive<
                   ::google::protobuf::uint32, ::google::protobuf::internal::WireFormatLite::TYPE_UINT32>(
                 1, 80, input, this->mutable_pred_indices())));
        } else if (tag == 82) {
          DO_((::google::protobuf::internal::WireFormatLite::ReadPackedPrimitiveNoInline<
                   ::google::protobuf::uint32, ::google::protobuf::internal::WireFormatLite::TYPE_UINT32>(
                 input, this->mutable_pred_indices())));
        } else {
          goto handle_unusual;
        }
        if (input->ExpectTag(80)) goto parse_pred_indices;
        if (input->ExpectTag(88)) goto parse_succ_indices;
        break;
      }

      // repeated uint32 succ_indices = 11;
      case 11: {
        if (tag == 88) {
         parse_succ_indices:
          DO_((::google::protobuf::internal::WireFormatLite::ReadRepeatedPrimitive<
                   ::google::protobuf::uint32, ::google::protobuf::internal::WireFormatLite::TYPE_UINT32>(
                 1, 88, input, this->mutable_succ_indices())));
        } else if (tag == 90) {
          DO_((::google::protobuf::internal::WireFormatLite::ReadPackedPrimitiveNoInline<
                   ::google::protobuf::uint32, ::google::protobuf::internal::WireFormatLite::TYPE_UINT32>(
                 input, this->mutable_succ_indices())));
        } else {
          goto handle_unusual;
        }
        if (input->ExpectTag(88)) goto parse_succ_indices;
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
  // @@protoc_insertion_point(parse_success:hdmap_proto.Section)
  return true;
failure:
  // @@protoc_insertion_point(parse_failure:hdmap_proto.Section)
  return false;
#undef DO_
}

void Section::SerializeWithCachedSizes(
    ::google::protobuf::io::CodedOutputStream* output) const {
  // @@protoc_insertion_point(serialize_start:hdmap_proto.Section)
  // required .hdmap_proto.Id id = 1;
  if (has_id()) {
    ::google::protobuf::internal::WireFormatLite::WriteMessageMaybeToArray(
      1, *this->id_, output);
  }

  // optional .hdmap_proto.Section.LaneDirection direction = 2;
  if (has_direction()) {
    ::google::protobuf::internal::WireFormatLite::WriteEnum(
      2, this->direction(), output);
  }

  // repeated .hdmap_proto.Lane lanes = 3;
  for (unsigned int i = 0, n = this->lanes_size(); i < n; i++) {
    ::google::protobuf::internal::WireFormatLite::WriteMessageMaybeToArray(
      3, this->lanes(i), output);
  }

  // repeated uint32 pred_indices = 10;
  for (int i = 0; i < this->pred_indices_size(); i++) {
    ::google::protobuf::internal::WireFormatLite::WriteUInt32(
      10, this->pred_indices(i), output);
  }

  // repeated uint32 succ_indices = 11;
  for (int i = 0; i < this->succ_indices_size(); i++) {
    ::google::protobuf::internal::WireFormatLite::WriteUInt32(
      11, this->succ_indices(i), output);
  }

  if (_internal_metadata_.have_unknown_fields()) {
    ::google::protobuf::internal::WireFormat::SerializeUnknownFields(
        unknown_fields(), output);
  }
  // @@protoc_insertion_point(serialize_end:hdmap_proto.Section)
}

::google::protobuf::uint8* Section::InternalSerializeWithCachedSizesToArray(
    bool deterministic, ::google::protobuf::uint8* target) const {
  // @@protoc_insertion_point(serialize_to_array_start:hdmap_proto.Section)
  // required .hdmap_proto.Id id = 1;
  if (has_id()) {
    target = ::google::protobuf::internal::WireFormatLite::
      InternalWriteMessageNoVirtualToArray(
        1, *this->id_, false, target);
  }

  // optional .hdmap_proto.Section.LaneDirection direction = 2;
  if (has_direction()) {
    target = ::google::protobuf::internal::WireFormatLite::WriteEnumToArray(
      2, this->direction(), target);
  }

  // repeated .hdmap_proto.Lane lanes = 3;
  for (unsigned int i = 0, n = this->lanes_size(); i < n; i++) {
    target = ::google::protobuf::internal::WireFormatLite::
      InternalWriteMessageNoVirtualToArray(
        3, this->lanes(i), false, target);
  }

  // repeated uint32 pred_indices = 10;
  for (int i = 0; i < this->pred_indices_size(); i++) {
    target = ::google::protobuf::internal::WireFormatLite::
      WriteUInt32ToArray(10, this->pred_indices(i), target);
  }

  // repeated uint32 succ_indices = 11;
  for (int i = 0; i < this->succ_indices_size(); i++) {
    target = ::google::protobuf::internal::WireFormatLite::
      WriteUInt32ToArray(11, this->succ_indices(i), target);
  }

  if (_internal_metadata_.have_unknown_fields()) {
    target = ::google::protobuf::internal::WireFormat::SerializeUnknownFieldsToArray(
        unknown_fields(), target);
  }
  // @@protoc_insertion_point(serialize_to_array_end:hdmap_proto.Section)
  return target;
}

int Section::ByteSize() const {
// @@protoc_insertion_point(message_byte_size_start:hdmap_proto.Section)
  int total_size = 0;

  // required .hdmap_proto.Id id = 1;
  if (has_id()) {
    total_size += 1 +
      ::google::protobuf::internal::WireFormatLite::MessageSizeNoVirtual(
        *this->id_);
  }
  // optional .hdmap_proto.Section.LaneDirection direction = 2;
  if (has_direction()) {
    total_size += 1 +
      ::google::protobuf::internal::WireFormatLite::EnumSize(this->direction());
  }

  // repeated .hdmap_proto.Lane lanes = 3;
  total_size += 1 * this->lanes_size();
  for (int i = 0; i < this->lanes_size(); i++) {
    total_size +=
      ::google::protobuf::internal::WireFormatLite::MessageSizeNoVirtual(
        this->lanes(i));
  }

  // repeated uint32 pred_indices = 10;
  {
    int data_size = 0;
    for (int i = 0; i < this->pred_indices_size(); i++) {
      data_size += ::google::protobuf::internal::WireFormatLite::
        UInt32Size(this->pred_indices(i));
    }
    total_size += 1 * this->pred_indices_size() + data_size;
  }

  // repeated uint32 succ_indices = 11;
  {
    int data_size = 0;
    for (int i = 0; i < this->succ_indices_size(); i++) {
      data_size += ::google::protobuf::internal::WireFormatLite::
        UInt32Size(this->succ_indices(i));
    }
    total_size += 1 * this->succ_indices_size() + data_size;
  }

  if (_internal_metadata_.have_unknown_fields()) {
    total_size +=
      ::google::protobuf::internal::WireFormat::ComputeUnknownFieldsSize(
        unknown_fields());
  }
  GOOGLE_SAFE_CONCURRENT_WRITES_BEGIN();
  _cached_size_ = total_size;
  GOOGLE_SAFE_CONCURRENT_WRITES_END();
  return total_size;
}

void Section::MergeFrom(const ::google::protobuf::Message& from) {
// @@protoc_insertion_point(generalized_merge_from_start:hdmap_proto.Section)
  if (GOOGLE_PREDICT_FALSE(&from == this)) {
    ::google::protobuf::internal::MergeFromFail(__FILE__, __LINE__);
  }
  const Section* source = 
      ::google::protobuf::internal::DynamicCastToGenerated<const Section>(
          &from);
  if (source == NULL) {
  // @@protoc_insertion_point(generalized_merge_from_cast_fail:hdmap_proto.Section)
    ::google::protobuf::internal::ReflectionOps::Merge(from, this);
  } else {
  // @@protoc_insertion_point(generalized_merge_from_cast_success:hdmap_proto.Section)
    MergeFrom(*source);
  }
}

void Section::MergeFrom(const Section& from) {
// @@protoc_insertion_point(class_specific_merge_from_start:hdmap_proto.Section)
  if (GOOGLE_PREDICT_FALSE(&from == this)) {
    ::google::protobuf::internal::MergeFromFail(__FILE__, __LINE__);
  }
  lanes_.MergeFrom(from.lanes_);
  pred_indices_.MergeFrom(from.pred_indices_);
  succ_indices_.MergeFrom(from.succ_indices_);
  if (from._has_bits_[0 / 32] & (0xffu << (0 % 32))) {
    if (from.has_id()) {
      mutable_id()->::hdmap_proto::Id::MergeFrom(from.id());
    }
    if (from.has_direction()) {
      set_direction(from.direction());
    }
  }
  if (from._internal_metadata_.have_unknown_fields()) {
    mutable_unknown_fields()->MergeFrom(from.unknown_fields());
  }
}

void Section::CopyFrom(const ::google::protobuf::Message& from) {
// @@protoc_insertion_point(generalized_copy_from_start:hdmap_proto.Section)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

void Section::CopyFrom(const Section& from) {
// @@protoc_insertion_point(class_specific_copy_from_start:hdmap_proto.Section)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

bool Section::IsInitialized() const {
  if ((_has_bits_[0] & 0x00000001) != 0x00000001) return false;

  if (has_id()) {
    if (!this->id_->IsInitialized()) return false;
  }
  if (!::google::protobuf::internal::AllAreInitialized(this->lanes())) return false;
  return true;
}

void Section::Swap(Section* other) {
  if (other == this) return;
  InternalSwap(other);
}
void Section::InternalSwap(Section* other) {
  std::swap(id_, other->id_);
  std::swap(direction_, other->direction_);
  lanes_.UnsafeArenaSwap(&other->lanes_);
  pred_indices_.UnsafeArenaSwap(&other->pred_indices_);
  succ_indices_.UnsafeArenaSwap(&other->succ_indices_);
  std::swap(_has_bits_[0], other->_has_bits_[0]);
  _internal_metadata_.Swap(&other->_internal_metadata_);
  std::swap(_cached_size_, other->_cached_size_);
}

::google::protobuf::Metadata Section::GetMetadata() const {
  protobuf_AssignDescriptorsOnce();
  ::google::protobuf::Metadata metadata;
  metadata.descriptor = Section_descriptor_;
  metadata.reflection = Section_reflection_;
  return metadata;
}

#if PROTOBUF_INLINE_NOT_IN_HEADERS
// Section

// required .hdmap_proto.Id id = 1;
bool Section::has_id() const {
  return (_has_bits_[0] & 0x00000001u) != 0;
}
void Section::set_has_id() {
  _has_bits_[0] |= 0x00000001u;
}
void Section::clear_has_id() {
  _has_bits_[0] &= ~0x00000001u;
}
void Section::clear_id() {
  if (id_ != NULL) id_->::hdmap_proto::Id::Clear();
  clear_has_id();
}
const ::hdmap_proto::Id& Section::id() const {
  // @@protoc_insertion_point(field_get:hdmap_proto.Section.id)
  return id_ != NULL ? *id_ : *default_instance_->id_;
}
::hdmap_proto::Id* Section::mutable_id() {
  set_has_id();
  if (id_ == NULL) {
    id_ = new ::hdmap_proto::Id;
  }
  // @@protoc_insertion_point(field_mutable:hdmap_proto.Section.id)
  return id_;
}
::hdmap_proto::Id* Section::release_id() {
  // @@protoc_insertion_point(field_release:hdmap_proto.Section.id)
  clear_has_id();
  ::hdmap_proto::Id* temp = id_;
  id_ = NULL;
  return temp;
}
void Section::set_allocated_id(::hdmap_proto::Id* id) {
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
bool Section::has_direction() const {
  return (_has_bits_[0] & 0x00000002u) != 0;
}
void Section::set_has_direction() {
  _has_bits_[0] |= 0x00000002u;
}
void Section::clear_has_direction() {
  _has_bits_[0] &= ~0x00000002u;
}
void Section::clear_direction() {
  direction_ = 0;
  clear_has_direction();
}
 ::hdmap_proto::Section_LaneDirection Section::direction() const {
  // @@protoc_insertion_point(field_get:hdmap_proto.Section.direction)
  return static_cast< ::hdmap_proto::Section_LaneDirection >(direction_);
}
 void Section::set_direction(::hdmap_proto::Section_LaneDirection value) {
  assert(::hdmap_proto::Section_LaneDirection_IsValid(value));
  set_has_direction();
  direction_ = value;
  // @@protoc_insertion_point(field_set:hdmap_proto.Section.direction)
}

// repeated .hdmap_proto.Lane lanes = 3;
int Section::lanes_size() const {
  return lanes_.size();
}
void Section::clear_lanes() {
  lanes_.Clear();
}
const ::hdmap_proto::Lane& Section::lanes(int index) const {
  // @@protoc_insertion_point(field_get:hdmap_proto.Section.lanes)
  return lanes_.Get(index);
}
::hdmap_proto::Lane* Section::mutable_lanes(int index) {
  // @@protoc_insertion_point(field_mutable:hdmap_proto.Section.lanes)
  return lanes_.Mutable(index);
}
::hdmap_proto::Lane* Section::add_lanes() {
  // @@protoc_insertion_point(field_add:hdmap_proto.Section.lanes)
  return lanes_.Add();
}
::google::protobuf::RepeatedPtrField< ::hdmap_proto::Lane >*
Section::mutable_lanes() {
  // @@protoc_insertion_point(field_mutable_list:hdmap_proto.Section.lanes)
  return &lanes_;
}
const ::google::protobuf::RepeatedPtrField< ::hdmap_proto::Lane >&
Section::lanes() const {
  // @@protoc_insertion_point(field_list:hdmap_proto.Section.lanes)
  return lanes_;
}

// repeated uint32 pred_indices = 10;
int Section::pred_indices_size() const {
  return pred_indices_.size();
}
void Section::clear_pred_indices() {
  pred_indices_.Clear();
}
 ::google::protobuf::uint32 Section::pred_indices(int index) const {
  // @@protoc_insertion_point(field_get:hdmap_proto.Section.pred_indices)
  return pred_indices_.Get(index);
}
 void Section::set_pred_indices(int index, ::google::protobuf::uint32 value) {
  pred_indices_.Set(index, value);
  // @@protoc_insertion_point(field_set:hdmap_proto.Section.pred_indices)
}
 void Section::add_pred_indices(::google::protobuf::uint32 value) {
  pred_indices_.Add(value);
  // @@protoc_insertion_point(field_add:hdmap_proto.Section.pred_indices)
}
 const ::google::protobuf::RepeatedField< ::google::protobuf::uint32 >&
Section::pred_indices() const {
  // @@protoc_insertion_point(field_list:hdmap_proto.Section.pred_indices)
  return pred_indices_;
}
 ::google::protobuf::RepeatedField< ::google::protobuf::uint32 >*
Section::mutable_pred_indices() {
  // @@protoc_insertion_point(field_mutable_list:hdmap_proto.Section.pred_indices)
  return &pred_indices_;
}

// repeated uint32 succ_indices = 11;
int Section::succ_indices_size() const {
  return succ_indices_.size();
}
void Section::clear_succ_indices() {
  succ_indices_.Clear();
}
 ::google::protobuf::uint32 Section::succ_indices(int index) const {
  // @@protoc_insertion_point(field_get:hdmap_proto.Section.succ_indices)
  return succ_indices_.Get(index);
}
 void Section::set_succ_indices(int index, ::google::protobuf::uint32 value) {
  succ_indices_.Set(index, value);
  // @@protoc_insertion_point(field_set:hdmap_proto.Section.succ_indices)
}
 void Section::add_succ_indices(::google::protobuf::uint32 value) {
  succ_indices_.Add(value);
  // @@protoc_insertion_point(field_add:hdmap_proto.Section.succ_indices)
}
 const ::google::protobuf::RepeatedField< ::google::protobuf::uint32 >&
Section::succ_indices() const {
  // @@protoc_insertion_point(field_list:hdmap_proto.Section.succ_indices)
  return succ_indices_;
}
 ::google::protobuf::RepeatedField< ::google::protobuf::uint32 >*
Section::mutable_succ_indices() {
  // @@protoc_insertion_point(field_mutable_list:hdmap_proto.Section.succ_indices)
  return &succ_indices_;
}

#endif  // PROTOBUF_INLINE_NOT_IN_HEADERS

// @@protoc_insertion_point(namespace_scope)

}  // namespace hdmap_proto

// @@protoc_insertion_point(global_scope)
