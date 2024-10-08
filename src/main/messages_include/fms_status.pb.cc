// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: fms_status.proto

#include "fms_status.pb.h"

#include <algorithm>

#include <google/protobuf/io/coded_stream.h>
#include <google/protobuf/extension_set.h>
#include <google/protobuf/wire_format_lite.h>
#include <google/protobuf/descriptor.h>
#include <google/protobuf/generated_message_reflection.h>
#include <google/protobuf/reflection_ops.h>
#include <google/protobuf/wire_format.h>
// @@protoc_insertion_point(includes)
#include <google/protobuf/port_def.inc>

PROTOBUF_PRAGMA_INIT_SEG

namespace _pb = ::PROTOBUF_NAMESPACE_ID;
namespace _pbi = _pb::internal;

namespace messages {
PROTOBUF_CONSTEXPR FMS_Status::FMS_Status(
    ::_pbi::ConstantInitialized): _impl_{
    /*decltype(_impl_._has_bits_)*/{}
  , /*decltype(_impl_._cached_size_)*/{}
  , /*decltype(_impl_.event_name_)*/{&::_pbi::fixed_address_empty_string, ::_pbi::ConstantInitialized{}}
  , /*decltype(_impl_.game_message_)*/{&::_pbi::fixed_address_empty_string, ::_pbi::ConstantInitialized{}}
  , /*decltype(_impl_.alliance_)*/0
  , /*decltype(_impl_.match_type_)*/0
  , /*decltype(_impl_.battery_volts_)*/0
  , /*decltype(_impl_.driver_station_)*/0
  , /*decltype(_impl_.match_numer_)*/0
  , /*decltype(_impl_.match_seconds_)*/0
  , /*decltype(_impl_.replay_number_)*/0
  , /*decltype(_impl_.is_ds_attached_)*/false
  , /*decltype(_impl_.is_fms_attached_)*/false
  , /*decltype(_impl_.is_enabled_)*/false
  , /*decltype(_impl_.is_disabled_)*/false
  , /*decltype(_impl_.is_auto_)*/false
  , /*decltype(_impl_.is_teleop_)*/false
  , /*decltype(_impl_.is_test_)*/false
  , /*decltype(_impl_.is_e_stopped_)*/false} {}
struct FMS_StatusDefaultTypeInternal {
  PROTOBUF_CONSTEXPR FMS_StatusDefaultTypeInternal()
      : _instance(::_pbi::ConstantInitialized{}) {}
  ~FMS_StatusDefaultTypeInternal() {}
  union {
    FMS_Status _instance;
  };
};
PROTOBUF_ATTRIBUTE_NO_DESTROY PROTOBUF_CONSTINIT PROTOBUF_ATTRIBUTE_INIT_PRIORITY1 FMS_StatusDefaultTypeInternal _FMS_Status_default_instance_;
}  // namespace messages
static ::_pb::Metadata file_level_metadata_fms_5fstatus_2eproto[1];
static const ::_pb::EnumDescriptor* file_level_enum_descriptors_fms_5fstatus_2eproto[3];
static constexpr ::_pb::ServiceDescriptor const** file_level_service_descriptors_fms_5fstatus_2eproto = nullptr;

const uint32_t TableStruct_fms_5fstatus_2eproto::offsets[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) = {
  PROTOBUF_FIELD_OFFSET(::messages::FMS_Status, _impl_._has_bits_),
  PROTOBUF_FIELD_OFFSET(::messages::FMS_Status, _internal_metadata_),
  ~0u,  // no _extensions_
  ~0u,  // no _oneof_case_
  ~0u,  // no _weak_field_map_
  ~0u,  // no _inlined_string_donated_
  PROTOBUF_FIELD_OFFSET(::messages::FMS_Status, _impl_.alliance_),
  PROTOBUF_FIELD_OFFSET(::messages::FMS_Status, _impl_.match_type_),
  PROTOBUF_FIELD_OFFSET(::messages::FMS_Status, _impl_.driver_station_),
  PROTOBUF_FIELD_OFFSET(::messages::FMS_Status, _impl_.battery_volts_),
  PROTOBUF_FIELD_OFFSET(::messages::FMS_Status, _impl_.event_name_),
  PROTOBUF_FIELD_OFFSET(::messages::FMS_Status, _impl_.game_message_),
  PROTOBUF_FIELD_OFFSET(::messages::FMS_Status, _impl_.match_numer_),
  PROTOBUF_FIELD_OFFSET(::messages::FMS_Status, _impl_.match_seconds_),
  PROTOBUF_FIELD_OFFSET(::messages::FMS_Status, _impl_.replay_number_),
  PROTOBUF_FIELD_OFFSET(::messages::FMS_Status, _impl_.is_ds_attached_),
  PROTOBUF_FIELD_OFFSET(::messages::FMS_Status, _impl_.is_fms_attached_),
  PROTOBUF_FIELD_OFFSET(::messages::FMS_Status, _impl_.is_enabled_),
  PROTOBUF_FIELD_OFFSET(::messages::FMS_Status, _impl_.is_disabled_),
  PROTOBUF_FIELD_OFFSET(::messages::FMS_Status, _impl_.is_auto_),
  PROTOBUF_FIELD_OFFSET(::messages::FMS_Status, _impl_.is_teleop_),
  PROTOBUF_FIELD_OFFSET(::messages::FMS_Status, _impl_.is_test_),
  PROTOBUF_FIELD_OFFSET(::messages::FMS_Status, _impl_.is_e_stopped_),
  2,
  3,
  5,
  4,
  0,
  1,
  6,
  7,
  8,
  9,
  10,
  11,
  12,
  13,
  14,
  15,
  16,
};
static const ::_pbi::MigrationSchema schemas[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) = {
  { 0, 23, -1, sizeof(::messages::FMS_Status)},
};

static const ::_pb::Message* const file_default_instances[] = {
  &::messages::_FMS_Status_default_instance_._instance,
};

const char descriptor_table_protodef_fms_5fstatus_2eproto[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) =
  "\n\020fms_status.proto\022\010messages\"\236\006\n\nFMS_Sta"
  "tus\022)\n\010alliance\030\001 \001(\0162\022.messages.Allianc"
  "eH\000\210\001\001\022-\n\nmatch_type\030\002 \001(\0162\024.messages.Ma"
  "tch_TypeH\001\210\001\001\0225\n\016driver_station\030\003 \001(\0162\030."
  "messages.Driver_StationH\002\210\001\001\022\032\n\rbattery_"
  "volts\030\004 \001(\001H\003\210\001\001\022\027\n\nevent_name\030\005 \001(\tH\004\210\001"
  "\001\022\031\n\014game_message\030\006 \001(\tH\005\210\001\001\022\030\n\013match_nu"
  "mer\030\007 \001(\005H\006\210\001\001\022\032\n\rmatch_seconds\030\010 \001(\001H\007\210"
  "\001\001\022\032\n\rreplay_number\030\t \001(\005H\010\210\001\001\022\033\n\016is_ds_"
  "attached\030\n \001(\010H\t\210\001\001\022\034\n\017is_fms_attached\030\013"
  " \001(\010H\n\210\001\001\022\027\n\nis_enabled\030\014 \001(\010H\013\210\001\001\022\030\n\013is"
  "_disabled\030\r \001(\010H\014\210\001\001\022\024\n\007is_auto\030\016 \001(\010H\r\210"
  "\001\001\022\026\n\tis_teleop\030\017 \001(\010H\016\210\001\001\022\024\n\007is_test\030\020 "
  "\001(\010H\017\210\001\001\022\031\n\014is_e_stopped\030\021 \001(\010H\020\210\001\001B\013\n\t_"
  "allianceB\r\n\013_match_typeB\021\n\017_driver_stati"
  "onB\020\n\016_battery_voltsB\r\n\013_event_nameB\017\n\r_"
  "game_messageB\016\n\014_match_numerB\020\n\016_match_s"
  "econdsB\020\n\016_replay_numberB\021\n\017_is_ds_attac"
  "hedB\022\n\020_is_fms_attachedB\r\n\013_is_enabledB\016"
  "\n\014_is_disabledB\n\n\010_is_autoB\014\n\n_is_teleop"
  "B\n\n\010_is_testB\017\n\r_is_e_stopped*B\n\010Allianc"
  "e\022\021\n\rALLIANCE_NONE\020\000\022\020\n\014ALLIANCE_RED\020\001\022\021"
  "\n\rALLIANCE_BLUE\020\002*t\n\nMatch_Type\022\023\n\017MATCH"
  "_TYPE_NONE\020\000\022\027\n\023MATCH_TYPE_PRACTICE\020\001\022\034\n"
  "\030MATCH_TYPE_QUALIFICATION\020\002\022\032\n\026MATCH_TYP"
  "E_ELIMINATION\020\003*C\n\016Driver_Station\022\013\n\007DS_"
  "NONE\020\000\022\n\n\006DS_ONE\020\001\022\n\n\006DS_TWO\020\002\022\014\n\010DS_THR"
  "EE\020\003b\006proto3"
  ;
static ::_pbi::once_flag descriptor_table_fms_5fstatus_2eproto_once;
const ::_pbi::DescriptorTable descriptor_table_fms_5fstatus_2eproto = {
    false, false, 1092, descriptor_table_protodef_fms_5fstatus_2eproto,
    "fms_status.proto",
    &descriptor_table_fms_5fstatus_2eproto_once, nullptr, 0, 1,
    schemas, file_default_instances, TableStruct_fms_5fstatus_2eproto::offsets,
    file_level_metadata_fms_5fstatus_2eproto, file_level_enum_descriptors_fms_5fstatus_2eproto,
    file_level_service_descriptors_fms_5fstatus_2eproto,
};
PROTOBUF_ATTRIBUTE_WEAK const ::_pbi::DescriptorTable* descriptor_table_fms_5fstatus_2eproto_getter() {
  return &descriptor_table_fms_5fstatus_2eproto;
}

// Force running AddDescriptors() at dynamic initialization time.
PROTOBUF_ATTRIBUTE_INIT_PRIORITY2 static ::_pbi::AddDescriptorsRunner dynamic_init_dummy_fms_5fstatus_2eproto(&descriptor_table_fms_5fstatus_2eproto);
namespace messages {
const ::PROTOBUF_NAMESPACE_ID::EnumDescriptor* Alliance_descriptor() {
  ::PROTOBUF_NAMESPACE_ID::internal::AssignDescriptors(&descriptor_table_fms_5fstatus_2eproto);
  return file_level_enum_descriptors_fms_5fstatus_2eproto[0];
}
bool Alliance_IsValid(int value) {
  switch (value) {
    case 0:
    case 1:
    case 2:
      return true;
    default:
      return false;
  }
}

const ::PROTOBUF_NAMESPACE_ID::EnumDescriptor* Match_Type_descriptor() {
  ::PROTOBUF_NAMESPACE_ID::internal::AssignDescriptors(&descriptor_table_fms_5fstatus_2eproto);
  return file_level_enum_descriptors_fms_5fstatus_2eproto[1];
}
bool Match_Type_IsValid(int value) {
  switch (value) {
    case 0:
    case 1:
    case 2:
    case 3:
      return true;
    default:
      return false;
  }
}

const ::PROTOBUF_NAMESPACE_ID::EnumDescriptor* Driver_Station_descriptor() {
  ::PROTOBUF_NAMESPACE_ID::internal::AssignDescriptors(&descriptor_table_fms_5fstatus_2eproto);
  return file_level_enum_descriptors_fms_5fstatus_2eproto[2];
}
bool Driver_Station_IsValid(int value) {
  switch (value) {
    case 0:
    case 1:
    case 2:
    case 3:
      return true;
    default:
      return false;
  }
}


// ===================================================================

class FMS_Status::_Internal {
 public:
  using HasBits = decltype(std::declval<FMS_Status>()._impl_._has_bits_);
  static void set_has_alliance(HasBits* has_bits) {
    (*has_bits)[0] |= 4u;
  }
  static void set_has_match_type(HasBits* has_bits) {
    (*has_bits)[0] |= 8u;
  }
  static void set_has_driver_station(HasBits* has_bits) {
    (*has_bits)[0] |= 32u;
  }
  static void set_has_battery_volts(HasBits* has_bits) {
    (*has_bits)[0] |= 16u;
  }
  static void set_has_event_name(HasBits* has_bits) {
    (*has_bits)[0] |= 1u;
  }
  static void set_has_game_message(HasBits* has_bits) {
    (*has_bits)[0] |= 2u;
  }
  static void set_has_match_numer(HasBits* has_bits) {
    (*has_bits)[0] |= 64u;
  }
  static void set_has_match_seconds(HasBits* has_bits) {
    (*has_bits)[0] |= 128u;
  }
  static void set_has_replay_number(HasBits* has_bits) {
    (*has_bits)[0] |= 256u;
  }
  static void set_has_is_ds_attached(HasBits* has_bits) {
    (*has_bits)[0] |= 512u;
  }
  static void set_has_is_fms_attached(HasBits* has_bits) {
    (*has_bits)[0] |= 1024u;
  }
  static void set_has_is_enabled(HasBits* has_bits) {
    (*has_bits)[0] |= 2048u;
  }
  static void set_has_is_disabled(HasBits* has_bits) {
    (*has_bits)[0] |= 4096u;
  }
  static void set_has_is_auto(HasBits* has_bits) {
    (*has_bits)[0] |= 8192u;
  }
  static void set_has_is_teleop(HasBits* has_bits) {
    (*has_bits)[0] |= 16384u;
  }
  static void set_has_is_test(HasBits* has_bits) {
    (*has_bits)[0] |= 32768u;
  }
  static void set_has_is_e_stopped(HasBits* has_bits) {
    (*has_bits)[0] |= 65536u;
  }
};

FMS_Status::FMS_Status(::PROTOBUF_NAMESPACE_ID::Arena* arena,
                         bool is_message_owned)
  : ::PROTOBUF_NAMESPACE_ID::Message(arena, is_message_owned) {
  SharedCtor(arena, is_message_owned);
  // @@protoc_insertion_point(arena_constructor:messages.FMS_Status)
}
FMS_Status::FMS_Status(const FMS_Status& from)
  : ::PROTOBUF_NAMESPACE_ID::Message() {
  FMS_Status* const _this = this; (void)_this;
  new (&_impl_) Impl_{
      decltype(_impl_._has_bits_){from._impl_._has_bits_}
    , /*decltype(_impl_._cached_size_)*/{}
    , decltype(_impl_.event_name_){}
    , decltype(_impl_.game_message_){}
    , decltype(_impl_.alliance_){}
    , decltype(_impl_.match_type_){}
    , decltype(_impl_.battery_volts_){}
    , decltype(_impl_.driver_station_){}
    , decltype(_impl_.match_numer_){}
    , decltype(_impl_.match_seconds_){}
    , decltype(_impl_.replay_number_){}
    , decltype(_impl_.is_ds_attached_){}
    , decltype(_impl_.is_fms_attached_){}
    , decltype(_impl_.is_enabled_){}
    , decltype(_impl_.is_disabled_){}
    , decltype(_impl_.is_auto_){}
    , decltype(_impl_.is_teleop_){}
    , decltype(_impl_.is_test_){}
    , decltype(_impl_.is_e_stopped_){}};

  _internal_metadata_.MergeFrom<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>(from._internal_metadata_);
  _impl_.event_name_.InitDefault();
  #ifdef PROTOBUF_FORCE_COPY_DEFAULT_STRING
    _impl_.event_name_.Set("", GetArenaForAllocation());
  #endif // PROTOBUF_FORCE_COPY_DEFAULT_STRING
  if (from._internal_has_event_name()) {
    _this->_impl_.event_name_.Set(from._internal_event_name(), 
      _this->GetArenaForAllocation());
  }
  _impl_.game_message_.InitDefault();
  #ifdef PROTOBUF_FORCE_COPY_DEFAULT_STRING
    _impl_.game_message_.Set("", GetArenaForAllocation());
  #endif // PROTOBUF_FORCE_COPY_DEFAULT_STRING
  if (from._internal_has_game_message()) {
    _this->_impl_.game_message_.Set(from._internal_game_message(), 
      _this->GetArenaForAllocation());
  }
  ::memcpy(&_impl_.alliance_, &from._impl_.alliance_,
    static_cast<size_t>(reinterpret_cast<char*>(&_impl_.is_e_stopped_) -
    reinterpret_cast<char*>(&_impl_.alliance_)) + sizeof(_impl_.is_e_stopped_));
  // @@protoc_insertion_point(copy_constructor:messages.FMS_Status)
}

inline void FMS_Status::SharedCtor(
    ::_pb::Arena* arena, bool is_message_owned) {
  (void)arena;
  (void)is_message_owned;
  new (&_impl_) Impl_{
      decltype(_impl_._has_bits_){}
    , /*decltype(_impl_._cached_size_)*/{}
    , decltype(_impl_.event_name_){}
    , decltype(_impl_.game_message_){}
    , decltype(_impl_.alliance_){0}
    , decltype(_impl_.match_type_){0}
    , decltype(_impl_.battery_volts_){0}
    , decltype(_impl_.driver_station_){0}
    , decltype(_impl_.match_numer_){0}
    , decltype(_impl_.match_seconds_){0}
    , decltype(_impl_.replay_number_){0}
    , decltype(_impl_.is_ds_attached_){false}
    , decltype(_impl_.is_fms_attached_){false}
    , decltype(_impl_.is_enabled_){false}
    , decltype(_impl_.is_disabled_){false}
    , decltype(_impl_.is_auto_){false}
    , decltype(_impl_.is_teleop_){false}
    , decltype(_impl_.is_test_){false}
    , decltype(_impl_.is_e_stopped_){false}
  };
  _impl_.event_name_.InitDefault();
  #ifdef PROTOBUF_FORCE_COPY_DEFAULT_STRING
    _impl_.event_name_.Set("", GetArenaForAllocation());
  #endif // PROTOBUF_FORCE_COPY_DEFAULT_STRING
  _impl_.game_message_.InitDefault();
  #ifdef PROTOBUF_FORCE_COPY_DEFAULT_STRING
    _impl_.game_message_.Set("", GetArenaForAllocation());
  #endif // PROTOBUF_FORCE_COPY_DEFAULT_STRING
}

FMS_Status::~FMS_Status() {
  // @@protoc_insertion_point(destructor:messages.FMS_Status)
  if (auto *arena = _internal_metadata_.DeleteReturnArena<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>()) {
  (void)arena;
    return;
  }
  SharedDtor();
}

inline void FMS_Status::SharedDtor() {
  GOOGLE_DCHECK(GetArenaForAllocation() == nullptr);
  _impl_.event_name_.Destroy();
  _impl_.game_message_.Destroy();
}

void FMS_Status::SetCachedSize(int size) const {
  _impl_._cached_size_.Set(size);
}

void FMS_Status::Clear() {
// @@protoc_insertion_point(message_clear_start:messages.FMS_Status)
  uint32_t cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  cached_has_bits = _impl_._has_bits_[0];
  if (cached_has_bits & 0x00000003u) {
    if (cached_has_bits & 0x00000001u) {
      _impl_.event_name_.ClearNonDefaultToEmpty();
    }
    if (cached_has_bits & 0x00000002u) {
      _impl_.game_message_.ClearNonDefaultToEmpty();
    }
  }
  if (cached_has_bits & 0x000000fcu) {
    ::memset(&_impl_.alliance_, 0, static_cast<size_t>(
        reinterpret_cast<char*>(&_impl_.match_seconds_) -
        reinterpret_cast<char*>(&_impl_.alliance_)) + sizeof(_impl_.match_seconds_));
  }
  if (cached_has_bits & 0x0000ff00u) {
    ::memset(&_impl_.replay_number_, 0, static_cast<size_t>(
        reinterpret_cast<char*>(&_impl_.is_test_) -
        reinterpret_cast<char*>(&_impl_.replay_number_)) + sizeof(_impl_.is_test_));
  }
  _impl_.is_e_stopped_ = false;
  _impl_._has_bits_.Clear();
  _internal_metadata_.Clear<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>();
}

const char* FMS_Status::_InternalParse(const char* ptr, ::_pbi::ParseContext* ctx) {
#define CHK_(x) if (PROTOBUF_PREDICT_FALSE(!(x))) goto failure
  _Internal::HasBits has_bits{};
  while (!ctx->Done(&ptr)) {
    uint32_t tag;
    ptr = ::_pbi::ReadTag(ptr, &tag);
    switch (tag >> 3) {
      // optional .messages.Alliance alliance = 1;
      case 1:
        if (PROTOBUF_PREDICT_TRUE(static_cast<uint8_t>(tag) == 8)) {
          uint64_t val = ::PROTOBUF_NAMESPACE_ID::internal::ReadVarint64(&ptr);
          CHK_(ptr);
          _internal_set_alliance(static_cast<::messages::Alliance>(val));
        } else
          goto handle_unusual;
        continue;
      // optional .messages.Match_Type match_type = 2;
      case 2:
        if (PROTOBUF_PREDICT_TRUE(static_cast<uint8_t>(tag) == 16)) {
          uint64_t val = ::PROTOBUF_NAMESPACE_ID::internal::ReadVarint64(&ptr);
          CHK_(ptr);
          _internal_set_match_type(static_cast<::messages::Match_Type>(val));
        } else
          goto handle_unusual;
        continue;
      // optional .messages.Driver_Station driver_station = 3;
      case 3:
        if (PROTOBUF_PREDICT_TRUE(static_cast<uint8_t>(tag) == 24)) {
          uint64_t val = ::PROTOBUF_NAMESPACE_ID::internal::ReadVarint64(&ptr);
          CHK_(ptr);
          _internal_set_driver_station(static_cast<::messages::Driver_Station>(val));
        } else
          goto handle_unusual;
        continue;
      // optional double battery_volts = 4;
      case 4:
        if (PROTOBUF_PREDICT_TRUE(static_cast<uint8_t>(tag) == 33)) {
          _Internal::set_has_battery_volts(&has_bits);
          _impl_.battery_volts_ = ::PROTOBUF_NAMESPACE_ID::internal::UnalignedLoad<double>(ptr);
          ptr += sizeof(double);
        } else
          goto handle_unusual;
        continue;
      // optional string event_name = 5;
      case 5:
        if (PROTOBUF_PREDICT_TRUE(static_cast<uint8_t>(tag) == 42)) {
          auto str = _internal_mutable_event_name();
          ptr = ::_pbi::InlineGreedyStringParser(str, ptr, ctx);
          CHK_(ptr);
          CHK_(::_pbi::VerifyUTF8(str, "messages.FMS_Status.event_name"));
        } else
          goto handle_unusual;
        continue;
      // optional string game_message = 6;
      case 6:
        if (PROTOBUF_PREDICT_TRUE(static_cast<uint8_t>(tag) == 50)) {
          auto str = _internal_mutable_game_message();
          ptr = ::_pbi::InlineGreedyStringParser(str, ptr, ctx);
          CHK_(ptr);
          CHK_(::_pbi::VerifyUTF8(str, "messages.FMS_Status.game_message"));
        } else
          goto handle_unusual;
        continue;
      // optional int32 match_numer = 7;
      case 7:
        if (PROTOBUF_PREDICT_TRUE(static_cast<uint8_t>(tag) == 56)) {
          _Internal::set_has_match_numer(&has_bits);
          _impl_.match_numer_ = ::PROTOBUF_NAMESPACE_ID::internal::ReadVarint32(&ptr);
          CHK_(ptr);
        } else
          goto handle_unusual;
        continue;
      // optional double match_seconds = 8;
      case 8:
        if (PROTOBUF_PREDICT_TRUE(static_cast<uint8_t>(tag) == 65)) {
          _Internal::set_has_match_seconds(&has_bits);
          _impl_.match_seconds_ = ::PROTOBUF_NAMESPACE_ID::internal::UnalignedLoad<double>(ptr);
          ptr += sizeof(double);
        } else
          goto handle_unusual;
        continue;
      // optional int32 replay_number = 9;
      case 9:
        if (PROTOBUF_PREDICT_TRUE(static_cast<uint8_t>(tag) == 72)) {
          _Internal::set_has_replay_number(&has_bits);
          _impl_.replay_number_ = ::PROTOBUF_NAMESPACE_ID::internal::ReadVarint32(&ptr);
          CHK_(ptr);
        } else
          goto handle_unusual;
        continue;
      // optional bool is_ds_attached = 10;
      case 10:
        if (PROTOBUF_PREDICT_TRUE(static_cast<uint8_t>(tag) == 80)) {
          _Internal::set_has_is_ds_attached(&has_bits);
          _impl_.is_ds_attached_ = ::PROTOBUF_NAMESPACE_ID::internal::ReadVarint64(&ptr);
          CHK_(ptr);
        } else
          goto handle_unusual;
        continue;
      // optional bool is_fms_attached = 11;
      case 11:
        if (PROTOBUF_PREDICT_TRUE(static_cast<uint8_t>(tag) == 88)) {
          _Internal::set_has_is_fms_attached(&has_bits);
          _impl_.is_fms_attached_ = ::PROTOBUF_NAMESPACE_ID::internal::ReadVarint64(&ptr);
          CHK_(ptr);
        } else
          goto handle_unusual;
        continue;
      // optional bool is_enabled = 12;
      case 12:
        if (PROTOBUF_PREDICT_TRUE(static_cast<uint8_t>(tag) == 96)) {
          _Internal::set_has_is_enabled(&has_bits);
          _impl_.is_enabled_ = ::PROTOBUF_NAMESPACE_ID::internal::ReadVarint64(&ptr);
          CHK_(ptr);
        } else
          goto handle_unusual;
        continue;
      // optional bool is_disabled = 13;
      case 13:
        if (PROTOBUF_PREDICT_TRUE(static_cast<uint8_t>(tag) == 104)) {
          _Internal::set_has_is_disabled(&has_bits);
          _impl_.is_disabled_ = ::PROTOBUF_NAMESPACE_ID::internal::ReadVarint64(&ptr);
          CHK_(ptr);
        } else
          goto handle_unusual;
        continue;
      // optional bool is_auto = 14;
      case 14:
        if (PROTOBUF_PREDICT_TRUE(static_cast<uint8_t>(tag) == 112)) {
          _Internal::set_has_is_auto(&has_bits);
          _impl_.is_auto_ = ::PROTOBUF_NAMESPACE_ID::internal::ReadVarint64(&ptr);
          CHK_(ptr);
        } else
          goto handle_unusual;
        continue;
      // optional bool is_teleop = 15;
      case 15:
        if (PROTOBUF_PREDICT_TRUE(static_cast<uint8_t>(tag) == 120)) {
          _Internal::set_has_is_teleop(&has_bits);
          _impl_.is_teleop_ = ::PROTOBUF_NAMESPACE_ID::internal::ReadVarint64(&ptr);
          CHK_(ptr);
        } else
          goto handle_unusual;
        continue;
      // optional bool is_test = 16;
      case 16:
        if (PROTOBUF_PREDICT_TRUE(static_cast<uint8_t>(tag) == 128)) {
          _Internal::set_has_is_test(&has_bits);
          _impl_.is_test_ = ::PROTOBUF_NAMESPACE_ID::internal::ReadVarint64(&ptr);
          CHK_(ptr);
        } else
          goto handle_unusual;
        continue;
      // optional bool is_e_stopped = 17;
      case 17:
        if (PROTOBUF_PREDICT_TRUE(static_cast<uint8_t>(tag) == 136)) {
          _Internal::set_has_is_e_stopped(&has_bits);
          _impl_.is_e_stopped_ = ::PROTOBUF_NAMESPACE_ID::internal::ReadVarint64(&ptr);
          CHK_(ptr);
        } else
          goto handle_unusual;
        continue;
      default:
        goto handle_unusual;
    }  // switch
  handle_unusual:
    if ((tag == 0) || ((tag & 7) == 4)) {
      CHK_(ptr);
      ctx->SetLastTag(tag);
      goto message_done;
    }
    ptr = UnknownFieldParse(
        tag,
        _internal_metadata_.mutable_unknown_fields<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>(),
        ptr, ctx);
    CHK_(ptr != nullptr);
  }  // while
message_done:
  _impl_._has_bits_.Or(has_bits);
  return ptr;
failure:
  ptr = nullptr;
  goto message_done;
#undef CHK_
}

uint8_t* FMS_Status::_InternalSerialize(
    uint8_t* target, ::PROTOBUF_NAMESPACE_ID::io::EpsCopyOutputStream* stream) const {
  // @@protoc_insertion_point(serialize_to_array_start:messages.FMS_Status)
  uint32_t cached_has_bits = 0;
  (void) cached_has_bits;

  // optional .messages.Alliance alliance = 1;
  if (_internal_has_alliance()) {
    target = stream->EnsureSpace(target);
    target = ::_pbi::WireFormatLite::WriteEnumToArray(
      1, this->_internal_alliance(), target);
  }

  // optional .messages.Match_Type match_type = 2;
  if (_internal_has_match_type()) {
    target = stream->EnsureSpace(target);
    target = ::_pbi::WireFormatLite::WriteEnumToArray(
      2, this->_internal_match_type(), target);
  }

  // optional .messages.Driver_Station driver_station = 3;
  if (_internal_has_driver_station()) {
    target = stream->EnsureSpace(target);
    target = ::_pbi::WireFormatLite::WriteEnumToArray(
      3, this->_internal_driver_station(), target);
  }

  // optional double battery_volts = 4;
  if (_internal_has_battery_volts()) {
    target = stream->EnsureSpace(target);
    target = ::_pbi::WireFormatLite::WriteDoubleToArray(4, this->_internal_battery_volts(), target);
  }

  // optional string event_name = 5;
  if (_internal_has_event_name()) {
    ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::VerifyUtf8String(
      this->_internal_event_name().data(), static_cast<int>(this->_internal_event_name().length()),
      ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::SERIALIZE,
      "messages.FMS_Status.event_name");
    target = stream->WriteStringMaybeAliased(
        5, this->_internal_event_name(), target);
  }

  // optional string game_message = 6;
  if (_internal_has_game_message()) {
    ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::VerifyUtf8String(
      this->_internal_game_message().data(), static_cast<int>(this->_internal_game_message().length()),
      ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::SERIALIZE,
      "messages.FMS_Status.game_message");
    target = stream->WriteStringMaybeAliased(
        6, this->_internal_game_message(), target);
  }

  // optional int32 match_numer = 7;
  if (_internal_has_match_numer()) {
    target = stream->EnsureSpace(target);
    target = ::_pbi::WireFormatLite::WriteInt32ToArray(7, this->_internal_match_numer(), target);
  }

  // optional double match_seconds = 8;
  if (_internal_has_match_seconds()) {
    target = stream->EnsureSpace(target);
    target = ::_pbi::WireFormatLite::WriteDoubleToArray(8, this->_internal_match_seconds(), target);
  }

  // optional int32 replay_number = 9;
  if (_internal_has_replay_number()) {
    target = stream->EnsureSpace(target);
    target = ::_pbi::WireFormatLite::WriteInt32ToArray(9, this->_internal_replay_number(), target);
  }

  // optional bool is_ds_attached = 10;
  if (_internal_has_is_ds_attached()) {
    target = stream->EnsureSpace(target);
    target = ::_pbi::WireFormatLite::WriteBoolToArray(10, this->_internal_is_ds_attached(), target);
  }

  // optional bool is_fms_attached = 11;
  if (_internal_has_is_fms_attached()) {
    target = stream->EnsureSpace(target);
    target = ::_pbi::WireFormatLite::WriteBoolToArray(11, this->_internal_is_fms_attached(), target);
  }

  // optional bool is_enabled = 12;
  if (_internal_has_is_enabled()) {
    target = stream->EnsureSpace(target);
    target = ::_pbi::WireFormatLite::WriteBoolToArray(12, this->_internal_is_enabled(), target);
  }

  // optional bool is_disabled = 13;
  if (_internal_has_is_disabled()) {
    target = stream->EnsureSpace(target);
    target = ::_pbi::WireFormatLite::WriteBoolToArray(13, this->_internal_is_disabled(), target);
  }

  // optional bool is_auto = 14;
  if (_internal_has_is_auto()) {
    target = stream->EnsureSpace(target);
    target = ::_pbi::WireFormatLite::WriteBoolToArray(14, this->_internal_is_auto(), target);
  }

  // optional bool is_teleop = 15;
  if (_internal_has_is_teleop()) {
    target = stream->EnsureSpace(target);
    target = ::_pbi::WireFormatLite::WriteBoolToArray(15, this->_internal_is_teleop(), target);
  }

  // optional bool is_test = 16;
  if (_internal_has_is_test()) {
    target = stream->EnsureSpace(target);
    target = ::_pbi::WireFormatLite::WriteBoolToArray(16, this->_internal_is_test(), target);
  }

  // optional bool is_e_stopped = 17;
  if (_internal_has_is_e_stopped()) {
    target = stream->EnsureSpace(target);
    target = ::_pbi::WireFormatLite::WriteBoolToArray(17, this->_internal_is_e_stopped(), target);
  }

  if (PROTOBUF_PREDICT_FALSE(_internal_metadata_.have_unknown_fields())) {
    target = ::_pbi::WireFormat::InternalSerializeUnknownFieldsToArray(
        _internal_metadata_.unknown_fields<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>(::PROTOBUF_NAMESPACE_ID::UnknownFieldSet::default_instance), target, stream);
  }
  // @@protoc_insertion_point(serialize_to_array_end:messages.FMS_Status)
  return target;
}

size_t FMS_Status::ByteSizeLong() const {
// @@protoc_insertion_point(message_byte_size_start:messages.FMS_Status)
  size_t total_size = 0;

  uint32_t cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  cached_has_bits = _impl_._has_bits_[0];
  if (cached_has_bits & 0x000000ffu) {
    // optional string event_name = 5;
    if (cached_has_bits & 0x00000001u) {
      total_size += 1 +
        ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::StringSize(
          this->_internal_event_name());
    }

    // optional string game_message = 6;
    if (cached_has_bits & 0x00000002u) {
      total_size += 1 +
        ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::StringSize(
          this->_internal_game_message());
    }

    // optional .messages.Alliance alliance = 1;
    if (cached_has_bits & 0x00000004u) {
      total_size += 1 +
        ::_pbi::WireFormatLite::EnumSize(this->_internal_alliance());
    }

    // optional .messages.Match_Type match_type = 2;
    if (cached_has_bits & 0x00000008u) {
      total_size += 1 +
        ::_pbi::WireFormatLite::EnumSize(this->_internal_match_type());
    }

    // optional double battery_volts = 4;
    if (cached_has_bits & 0x00000010u) {
      total_size += 1 + 8;
    }

    // optional .messages.Driver_Station driver_station = 3;
    if (cached_has_bits & 0x00000020u) {
      total_size += 1 +
        ::_pbi::WireFormatLite::EnumSize(this->_internal_driver_station());
    }

    // optional int32 match_numer = 7;
    if (cached_has_bits & 0x00000040u) {
      total_size += ::_pbi::WireFormatLite::Int32SizePlusOne(this->_internal_match_numer());
    }

    // optional double match_seconds = 8;
    if (cached_has_bits & 0x00000080u) {
      total_size += 1 + 8;
    }

  }
  if (cached_has_bits & 0x0000ff00u) {
    // optional int32 replay_number = 9;
    if (cached_has_bits & 0x00000100u) {
      total_size += ::_pbi::WireFormatLite::Int32SizePlusOne(this->_internal_replay_number());
    }

    // optional bool is_ds_attached = 10;
    if (cached_has_bits & 0x00000200u) {
      total_size += 1 + 1;
    }

    // optional bool is_fms_attached = 11;
    if (cached_has_bits & 0x00000400u) {
      total_size += 1 + 1;
    }

    // optional bool is_enabled = 12;
    if (cached_has_bits & 0x00000800u) {
      total_size += 1 + 1;
    }

    // optional bool is_disabled = 13;
    if (cached_has_bits & 0x00001000u) {
      total_size += 1 + 1;
    }

    // optional bool is_auto = 14;
    if (cached_has_bits & 0x00002000u) {
      total_size += 1 + 1;
    }

    // optional bool is_teleop = 15;
    if (cached_has_bits & 0x00004000u) {
      total_size += 1 + 1;
    }

    // optional bool is_test = 16;
    if (cached_has_bits & 0x00008000u) {
      total_size += 2 + 1;
    }

  }
  // optional bool is_e_stopped = 17;
  if (cached_has_bits & 0x00010000u) {
    total_size += 2 + 1;
  }

  return MaybeComputeUnknownFieldsSize(total_size, &_impl_._cached_size_);
}

const ::PROTOBUF_NAMESPACE_ID::Message::ClassData FMS_Status::_class_data_ = {
    ::PROTOBUF_NAMESPACE_ID::Message::CopyWithSourceCheck,
    FMS_Status::MergeImpl
};
const ::PROTOBUF_NAMESPACE_ID::Message::ClassData*FMS_Status::GetClassData() const { return &_class_data_; }


void FMS_Status::MergeImpl(::PROTOBUF_NAMESPACE_ID::Message& to_msg, const ::PROTOBUF_NAMESPACE_ID::Message& from_msg) {
  auto* const _this = static_cast<FMS_Status*>(&to_msg);
  auto& from = static_cast<const FMS_Status&>(from_msg);
  // @@protoc_insertion_point(class_specific_merge_from_start:messages.FMS_Status)
  GOOGLE_DCHECK_NE(&from, _this);
  uint32_t cached_has_bits = 0;
  (void) cached_has_bits;

  cached_has_bits = from._impl_._has_bits_[0];
  if (cached_has_bits & 0x000000ffu) {
    if (cached_has_bits & 0x00000001u) {
      _this->_internal_set_event_name(from._internal_event_name());
    }
    if (cached_has_bits & 0x00000002u) {
      _this->_internal_set_game_message(from._internal_game_message());
    }
    if (cached_has_bits & 0x00000004u) {
      _this->_impl_.alliance_ = from._impl_.alliance_;
    }
    if (cached_has_bits & 0x00000008u) {
      _this->_impl_.match_type_ = from._impl_.match_type_;
    }
    if (cached_has_bits & 0x00000010u) {
      _this->_impl_.battery_volts_ = from._impl_.battery_volts_;
    }
    if (cached_has_bits & 0x00000020u) {
      _this->_impl_.driver_station_ = from._impl_.driver_station_;
    }
    if (cached_has_bits & 0x00000040u) {
      _this->_impl_.match_numer_ = from._impl_.match_numer_;
    }
    if (cached_has_bits & 0x00000080u) {
      _this->_impl_.match_seconds_ = from._impl_.match_seconds_;
    }
    _this->_impl_._has_bits_[0] |= cached_has_bits;
  }
  if (cached_has_bits & 0x0000ff00u) {
    if (cached_has_bits & 0x00000100u) {
      _this->_impl_.replay_number_ = from._impl_.replay_number_;
    }
    if (cached_has_bits & 0x00000200u) {
      _this->_impl_.is_ds_attached_ = from._impl_.is_ds_attached_;
    }
    if (cached_has_bits & 0x00000400u) {
      _this->_impl_.is_fms_attached_ = from._impl_.is_fms_attached_;
    }
    if (cached_has_bits & 0x00000800u) {
      _this->_impl_.is_enabled_ = from._impl_.is_enabled_;
    }
    if (cached_has_bits & 0x00001000u) {
      _this->_impl_.is_disabled_ = from._impl_.is_disabled_;
    }
    if (cached_has_bits & 0x00002000u) {
      _this->_impl_.is_auto_ = from._impl_.is_auto_;
    }
    if (cached_has_bits & 0x00004000u) {
      _this->_impl_.is_teleop_ = from._impl_.is_teleop_;
    }
    if (cached_has_bits & 0x00008000u) {
      _this->_impl_.is_test_ = from._impl_.is_test_;
    }
    _this->_impl_._has_bits_[0] |= cached_has_bits;
  }
  if (cached_has_bits & 0x00010000u) {
    _this->_internal_set_is_e_stopped(from._internal_is_e_stopped());
  }
  _this->_internal_metadata_.MergeFrom<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>(from._internal_metadata_);
}

void FMS_Status::CopyFrom(const FMS_Status& from) {
// @@protoc_insertion_point(class_specific_copy_from_start:messages.FMS_Status)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

bool FMS_Status::IsInitialized() const {
  return true;
}

void FMS_Status::InternalSwap(FMS_Status* other) {
  using std::swap;
  auto* lhs_arena = GetArenaForAllocation();
  auto* rhs_arena = other->GetArenaForAllocation();
  _internal_metadata_.InternalSwap(&other->_internal_metadata_);
  swap(_impl_._has_bits_[0], other->_impl_._has_bits_[0]);
  ::PROTOBUF_NAMESPACE_ID::internal::ArenaStringPtr::InternalSwap(
      &_impl_.event_name_, lhs_arena,
      &other->_impl_.event_name_, rhs_arena
  );
  ::PROTOBUF_NAMESPACE_ID::internal::ArenaStringPtr::InternalSwap(
      &_impl_.game_message_, lhs_arena,
      &other->_impl_.game_message_, rhs_arena
  );
  ::PROTOBUF_NAMESPACE_ID::internal::memswap<
      PROTOBUF_FIELD_OFFSET(FMS_Status, _impl_.is_e_stopped_)
      + sizeof(FMS_Status::_impl_.is_e_stopped_)
      - PROTOBUF_FIELD_OFFSET(FMS_Status, _impl_.alliance_)>(
          reinterpret_cast<char*>(&_impl_.alliance_),
          reinterpret_cast<char*>(&other->_impl_.alliance_));
}

::PROTOBUF_NAMESPACE_ID::Metadata FMS_Status::GetMetadata() const {
  return ::_pbi::AssignDescriptors(
      &descriptor_table_fms_5fstatus_2eproto_getter, &descriptor_table_fms_5fstatus_2eproto_once,
      file_level_metadata_fms_5fstatus_2eproto[0]);
}

// @@protoc_insertion_point(namespace_scope)
}  // namespace messages
PROTOBUF_NAMESPACE_OPEN
template<> PROTOBUF_NOINLINE ::messages::FMS_Status*
Arena::CreateMaybeMessage< ::messages::FMS_Status >(Arena* arena) {
  return Arena::CreateMessageInternal< ::messages::FMS_Status >(arena);
}
PROTOBUF_NAMESPACE_CLOSE

// @@protoc_insertion_point(global_scope)
#include <google/protobuf/port_undef.inc>
