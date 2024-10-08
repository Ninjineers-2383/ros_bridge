// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: fms_status.proto

#ifndef GOOGLE_PROTOBUF_INCLUDED_fms_5fstatus_2eproto
#define GOOGLE_PROTOBUF_INCLUDED_fms_5fstatus_2eproto

#include <limits>
#include <string>

#include <google/protobuf/port_def.inc>
#if PROTOBUF_VERSION < 3021000
#error This file was generated by a newer version of protoc which is
#error incompatible with your Protocol Buffer headers. Please update
#error your headers.
#endif
#if 3021012 < PROTOBUF_MIN_PROTOC_VERSION
#error This file was generated by an older version of protoc which is
#error incompatible with your Protocol Buffer headers. Please
#error regenerate this file with a newer version of protoc.
#endif

#include <google/protobuf/port_undef.inc>
#include <google/protobuf/io/coded_stream.h>
#include <google/protobuf/arena.h>
#include <google/protobuf/arenastring.h>
#include <google/protobuf/generated_message_util.h>
#include <google/protobuf/metadata_lite.h>
#include <google/protobuf/generated_message_reflection.h>
#include <google/protobuf/message.h>
#include <google/protobuf/repeated_field.h>  // IWYU pragma: export
#include <google/protobuf/extension_set.h>  // IWYU pragma: export
#include <google/protobuf/generated_enum_reflection.h>
#include <google/protobuf/unknown_field_set.h>
// @@protoc_insertion_point(includes)
#include <google/protobuf/port_def.inc>
#define PROTOBUF_INTERNAL_EXPORT_fms_5fstatus_2eproto
PROTOBUF_NAMESPACE_OPEN
namespace internal {
class AnyMetadata;
}  // namespace internal
PROTOBUF_NAMESPACE_CLOSE

// Internal implementation detail -- do not use these members.
struct TableStruct_fms_5fstatus_2eproto {
  static const uint32_t offsets[];
};
extern const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable descriptor_table_fms_5fstatus_2eproto;
namespace messages {
class FMS_Status;
struct FMS_StatusDefaultTypeInternal;
extern FMS_StatusDefaultTypeInternal _FMS_Status_default_instance_;
}  // namespace messages
PROTOBUF_NAMESPACE_OPEN
template<> ::messages::FMS_Status* Arena::CreateMaybeMessage<::messages::FMS_Status>(Arena*);
PROTOBUF_NAMESPACE_CLOSE
namespace messages {

enum Alliance : int {
  ALLIANCE_NONE = 0,
  ALLIANCE_RED = 1,
  ALLIANCE_BLUE = 2,
  Alliance_INT_MIN_SENTINEL_DO_NOT_USE_ = std::numeric_limits<int32_t>::min(),
  Alliance_INT_MAX_SENTINEL_DO_NOT_USE_ = std::numeric_limits<int32_t>::max()
};
bool Alliance_IsValid(int value);
constexpr Alliance Alliance_MIN = ALLIANCE_NONE;
constexpr Alliance Alliance_MAX = ALLIANCE_BLUE;
constexpr int Alliance_ARRAYSIZE = Alliance_MAX + 1;

const ::PROTOBUF_NAMESPACE_ID::EnumDescriptor* Alliance_descriptor();
template<typename T>
inline const std::string& Alliance_Name(T enum_t_value) {
  static_assert(::std::is_same<T, Alliance>::value ||
    ::std::is_integral<T>::value,
    "Incorrect type passed to function Alliance_Name.");
  return ::PROTOBUF_NAMESPACE_ID::internal::NameOfEnum(
    Alliance_descriptor(), enum_t_value);
}
inline bool Alliance_Parse(
    ::PROTOBUF_NAMESPACE_ID::ConstStringParam name, Alliance* value) {
  return ::PROTOBUF_NAMESPACE_ID::internal::ParseNamedEnum<Alliance>(
    Alliance_descriptor(), name, value);
}
enum Match_Type : int {
  MATCH_TYPE_NONE = 0,
  MATCH_TYPE_PRACTICE = 1,
  MATCH_TYPE_QUALIFICATION = 2,
  MATCH_TYPE_ELIMINATION = 3,
  Match_Type_INT_MIN_SENTINEL_DO_NOT_USE_ = std::numeric_limits<int32_t>::min(),
  Match_Type_INT_MAX_SENTINEL_DO_NOT_USE_ = std::numeric_limits<int32_t>::max()
};
bool Match_Type_IsValid(int value);
constexpr Match_Type Match_Type_MIN = MATCH_TYPE_NONE;
constexpr Match_Type Match_Type_MAX = MATCH_TYPE_ELIMINATION;
constexpr int Match_Type_ARRAYSIZE = Match_Type_MAX + 1;

const ::PROTOBUF_NAMESPACE_ID::EnumDescriptor* Match_Type_descriptor();
template<typename T>
inline const std::string& Match_Type_Name(T enum_t_value) {
  static_assert(::std::is_same<T, Match_Type>::value ||
    ::std::is_integral<T>::value,
    "Incorrect type passed to function Match_Type_Name.");
  return ::PROTOBUF_NAMESPACE_ID::internal::NameOfEnum(
    Match_Type_descriptor(), enum_t_value);
}
inline bool Match_Type_Parse(
    ::PROTOBUF_NAMESPACE_ID::ConstStringParam name, Match_Type* value) {
  return ::PROTOBUF_NAMESPACE_ID::internal::ParseNamedEnum<Match_Type>(
    Match_Type_descriptor(), name, value);
}
enum Driver_Station : int {
  DS_NONE = 0,
  DS_ONE = 1,
  DS_TWO = 2,
  DS_THREE = 3,
  Driver_Station_INT_MIN_SENTINEL_DO_NOT_USE_ = std::numeric_limits<int32_t>::min(),
  Driver_Station_INT_MAX_SENTINEL_DO_NOT_USE_ = std::numeric_limits<int32_t>::max()
};
bool Driver_Station_IsValid(int value);
constexpr Driver_Station Driver_Station_MIN = DS_NONE;
constexpr Driver_Station Driver_Station_MAX = DS_THREE;
constexpr int Driver_Station_ARRAYSIZE = Driver_Station_MAX + 1;

const ::PROTOBUF_NAMESPACE_ID::EnumDescriptor* Driver_Station_descriptor();
template<typename T>
inline const std::string& Driver_Station_Name(T enum_t_value) {
  static_assert(::std::is_same<T, Driver_Station>::value ||
    ::std::is_integral<T>::value,
    "Incorrect type passed to function Driver_Station_Name.");
  return ::PROTOBUF_NAMESPACE_ID::internal::NameOfEnum(
    Driver_Station_descriptor(), enum_t_value);
}
inline bool Driver_Station_Parse(
    ::PROTOBUF_NAMESPACE_ID::ConstStringParam name, Driver_Station* value) {
  return ::PROTOBUF_NAMESPACE_ID::internal::ParseNamedEnum<Driver_Station>(
    Driver_Station_descriptor(), name, value);
}
// ===================================================================

class FMS_Status final :
    public ::PROTOBUF_NAMESPACE_ID::Message /* @@protoc_insertion_point(class_definition:messages.FMS_Status) */ {
 public:
  inline FMS_Status() : FMS_Status(nullptr) {}
  ~FMS_Status() override;
  explicit PROTOBUF_CONSTEXPR FMS_Status(::PROTOBUF_NAMESPACE_ID::internal::ConstantInitialized);

  FMS_Status(const FMS_Status& from);
  FMS_Status(FMS_Status&& from) noexcept
    : FMS_Status() {
    *this = ::std::move(from);
  }

  inline FMS_Status& operator=(const FMS_Status& from) {
    CopyFrom(from);
    return *this;
  }
  inline FMS_Status& operator=(FMS_Status&& from) noexcept {
    if (this == &from) return *this;
    if (GetOwningArena() == from.GetOwningArena()
  #ifdef PROTOBUF_FORCE_COPY_IN_MOVE
        && GetOwningArena() != nullptr
  #endif  // !PROTOBUF_FORCE_COPY_IN_MOVE
    ) {
      InternalSwap(&from);
    } else {
      CopyFrom(from);
    }
    return *this;
  }

  static const ::PROTOBUF_NAMESPACE_ID::Descriptor* descriptor() {
    return GetDescriptor();
  }
  static const ::PROTOBUF_NAMESPACE_ID::Descriptor* GetDescriptor() {
    return default_instance().GetMetadata().descriptor;
  }
  static const ::PROTOBUF_NAMESPACE_ID::Reflection* GetReflection() {
    return default_instance().GetMetadata().reflection;
  }
  static const FMS_Status& default_instance() {
    return *internal_default_instance();
  }
  static inline const FMS_Status* internal_default_instance() {
    return reinterpret_cast<const FMS_Status*>(
               &_FMS_Status_default_instance_);
  }
  static constexpr int kIndexInFileMessages =
    0;

  friend void swap(FMS_Status& a, FMS_Status& b) {
    a.Swap(&b);
  }
  inline void Swap(FMS_Status* other) {
    if (other == this) return;
  #ifdef PROTOBUF_FORCE_COPY_IN_SWAP
    if (GetOwningArena() != nullptr &&
        GetOwningArena() == other->GetOwningArena()) {
   #else  // PROTOBUF_FORCE_COPY_IN_SWAP
    if (GetOwningArena() == other->GetOwningArena()) {
  #endif  // !PROTOBUF_FORCE_COPY_IN_SWAP
      InternalSwap(other);
    } else {
      ::PROTOBUF_NAMESPACE_ID::internal::GenericSwap(this, other);
    }
  }
  void UnsafeArenaSwap(FMS_Status* other) {
    if (other == this) return;
    GOOGLE_DCHECK(GetOwningArena() == other->GetOwningArena());
    InternalSwap(other);
  }

  // implements Message ----------------------------------------------

  FMS_Status* New(::PROTOBUF_NAMESPACE_ID::Arena* arena = nullptr) const final {
    return CreateMaybeMessage<FMS_Status>(arena);
  }
  using ::PROTOBUF_NAMESPACE_ID::Message::CopyFrom;
  void CopyFrom(const FMS_Status& from);
  using ::PROTOBUF_NAMESPACE_ID::Message::MergeFrom;
  void MergeFrom( const FMS_Status& from) {
    FMS_Status::MergeImpl(*this, from);
  }
  private:
  static void MergeImpl(::PROTOBUF_NAMESPACE_ID::Message& to_msg, const ::PROTOBUF_NAMESPACE_ID::Message& from_msg);
  public:
  PROTOBUF_ATTRIBUTE_REINITIALIZES void Clear() final;
  bool IsInitialized() const final;

  size_t ByteSizeLong() const final;
  const char* _InternalParse(const char* ptr, ::PROTOBUF_NAMESPACE_ID::internal::ParseContext* ctx) final;
  uint8_t* _InternalSerialize(
      uint8_t* target, ::PROTOBUF_NAMESPACE_ID::io::EpsCopyOutputStream* stream) const final;
  int GetCachedSize() const final { return _impl_._cached_size_.Get(); }

  private:
  void SharedCtor(::PROTOBUF_NAMESPACE_ID::Arena* arena, bool is_message_owned);
  void SharedDtor();
  void SetCachedSize(int size) const final;
  void InternalSwap(FMS_Status* other);

  private:
  friend class ::PROTOBUF_NAMESPACE_ID::internal::AnyMetadata;
  static ::PROTOBUF_NAMESPACE_ID::StringPiece FullMessageName() {
    return "messages.FMS_Status";
  }
  protected:
  explicit FMS_Status(::PROTOBUF_NAMESPACE_ID::Arena* arena,
                       bool is_message_owned = false);
  public:

  static const ClassData _class_data_;
  const ::PROTOBUF_NAMESPACE_ID::Message::ClassData*GetClassData() const final;

  ::PROTOBUF_NAMESPACE_ID::Metadata GetMetadata() const final;

  // nested types ----------------------------------------------------

  // accessors -------------------------------------------------------

  enum : int {
    kEventNameFieldNumber = 5,
    kGameMessageFieldNumber = 6,
    kAllianceFieldNumber = 1,
    kMatchTypeFieldNumber = 2,
    kBatteryVoltsFieldNumber = 4,
    kDriverStationFieldNumber = 3,
    kMatchNumerFieldNumber = 7,
    kMatchSecondsFieldNumber = 8,
    kReplayNumberFieldNumber = 9,
    kIsDsAttachedFieldNumber = 10,
    kIsFmsAttachedFieldNumber = 11,
    kIsEnabledFieldNumber = 12,
    kIsDisabledFieldNumber = 13,
    kIsAutoFieldNumber = 14,
    kIsTeleopFieldNumber = 15,
    kIsTestFieldNumber = 16,
    kIsEStoppedFieldNumber = 17,
  };
  // optional string event_name = 5;
  bool has_event_name() const;
  private:
  bool _internal_has_event_name() const;
  public:
  void clear_event_name();
  const std::string& event_name() const;
  template <typename ArgT0 = const std::string&, typename... ArgT>
  void set_event_name(ArgT0&& arg0, ArgT... args);
  std::string* mutable_event_name();
  PROTOBUF_NODISCARD std::string* release_event_name();
  void set_allocated_event_name(std::string* event_name);
  private:
  const std::string& _internal_event_name() const;
  inline PROTOBUF_ALWAYS_INLINE void _internal_set_event_name(const std::string& value);
  std::string* _internal_mutable_event_name();
  public:

  // optional string game_message = 6;
  bool has_game_message() const;
  private:
  bool _internal_has_game_message() const;
  public:
  void clear_game_message();
  const std::string& game_message() const;
  template <typename ArgT0 = const std::string&, typename... ArgT>
  void set_game_message(ArgT0&& arg0, ArgT... args);
  std::string* mutable_game_message();
  PROTOBUF_NODISCARD std::string* release_game_message();
  void set_allocated_game_message(std::string* game_message);
  private:
  const std::string& _internal_game_message() const;
  inline PROTOBUF_ALWAYS_INLINE void _internal_set_game_message(const std::string& value);
  std::string* _internal_mutable_game_message();
  public:

  // optional .messages.Alliance alliance = 1;
  bool has_alliance() const;
  private:
  bool _internal_has_alliance() const;
  public:
  void clear_alliance();
  ::messages::Alliance alliance() const;
  void set_alliance(::messages::Alliance value);
  private:
  ::messages::Alliance _internal_alliance() const;
  void _internal_set_alliance(::messages::Alliance value);
  public:

  // optional .messages.Match_Type match_type = 2;
  bool has_match_type() const;
  private:
  bool _internal_has_match_type() const;
  public:
  void clear_match_type();
  ::messages::Match_Type match_type() const;
  void set_match_type(::messages::Match_Type value);
  private:
  ::messages::Match_Type _internal_match_type() const;
  void _internal_set_match_type(::messages::Match_Type value);
  public:

  // optional double battery_volts = 4;
  bool has_battery_volts() const;
  private:
  bool _internal_has_battery_volts() const;
  public:
  void clear_battery_volts();
  double battery_volts() const;
  void set_battery_volts(double value);
  private:
  double _internal_battery_volts() const;
  void _internal_set_battery_volts(double value);
  public:

  // optional .messages.Driver_Station driver_station = 3;
  bool has_driver_station() const;
  private:
  bool _internal_has_driver_station() const;
  public:
  void clear_driver_station();
  ::messages::Driver_Station driver_station() const;
  void set_driver_station(::messages::Driver_Station value);
  private:
  ::messages::Driver_Station _internal_driver_station() const;
  void _internal_set_driver_station(::messages::Driver_Station value);
  public:

  // optional int32 match_numer = 7;
  bool has_match_numer() const;
  private:
  bool _internal_has_match_numer() const;
  public:
  void clear_match_numer();
  int32_t match_numer() const;
  void set_match_numer(int32_t value);
  private:
  int32_t _internal_match_numer() const;
  void _internal_set_match_numer(int32_t value);
  public:

  // optional double match_seconds = 8;
  bool has_match_seconds() const;
  private:
  bool _internal_has_match_seconds() const;
  public:
  void clear_match_seconds();
  double match_seconds() const;
  void set_match_seconds(double value);
  private:
  double _internal_match_seconds() const;
  void _internal_set_match_seconds(double value);
  public:

  // optional int32 replay_number = 9;
  bool has_replay_number() const;
  private:
  bool _internal_has_replay_number() const;
  public:
  void clear_replay_number();
  int32_t replay_number() const;
  void set_replay_number(int32_t value);
  private:
  int32_t _internal_replay_number() const;
  void _internal_set_replay_number(int32_t value);
  public:

  // optional bool is_ds_attached = 10;
  bool has_is_ds_attached() const;
  private:
  bool _internal_has_is_ds_attached() const;
  public:
  void clear_is_ds_attached();
  bool is_ds_attached() const;
  void set_is_ds_attached(bool value);
  private:
  bool _internal_is_ds_attached() const;
  void _internal_set_is_ds_attached(bool value);
  public:

  // optional bool is_fms_attached = 11;
  bool has_is_fms_attached() const;
  private:
  bool _internal_has_is_fms_attached() const;
  public:
  void clear_is_fms_attached();
  bool is_fms_attached() const;
  void set_is_fms_attached(bool value);
  private:
  bool _internal_is_fms_attached() const;
  void _internal_set_is_fms_attached(bool value);
  public:

  // optional bool is_enabled = 12;
  bool has_is_enabled() const;
  private:
  bool _internal_has_is_enabled() const;
  public:
  void clear_is_enabled();
  bool is_enabled() const;
  void set_is_enabled(bool value);
  private:
  bool _internal_is_enabled() const;
  void _internal_set_is_enabled(bool value);
  public:

  // optional bool is_disabled = 13;
  bool has_is_disabled() const;
  private:
  bool _internal_has_is_disabled() const;
  public:
  void clear_is_disabled();
  bool is_disabled() const;
  void set_is_disabled(bool value);
  private:
  bool _internal_is_disabled() const;
  void _internal_set_is_disabled(bool value);
  public:

  // optional bool is_auto = 14;
  bool has_is_auto() const;
  private:
  bool _internal_has_is_auto() const;
  public:
  void clear_is_auto();
  bool is_auto() const;
  void set_is_auto(bool value);
  private:
  bool _internal_is_auto() const;
  void _internal_set_is_auto(bool value);
  public:

  // optional bool is_teleop = 15;
  bool has_is_teleop() const;
  private:
  bool _internal_has_is_teleop() const;
  public:
  void clear_is_teleop();
  bool is_teleop() const;
  void set_is_teleop(bool value);
  private:
  bool _internal_is_teleop() const;
  void _internal_set_is_teleop(bool value);
  public:

  // optional bool is_test = 16;
  bool has_is_test() const;
  private:
  bool _internal_has_is_test() const;
  public:
  void clear_is_test();
  bool is_test() const;
  void set_is_test(bool value);
  private:
  bool _internal_is_test() const;
  void _internal_set_is_test(bool value);
  public:

  // optional bool is_e_stopped = 17;
  bool has_is_e_stopped() const;
  private:
  bool _internal_has_is_e_stopped() const;
  public:
  void clear_is_e_stopped();
  bool is_e_stopped() const;
  void set_is_e_stopped(bool value);
  private:
  bool _internal_is_e_stopped() const;
  void _internal_set_is_e_stopped(bool value);
  public:

  // @@protoc_insertion_point(class_scope:messages.FMS_Status)
 private:
  class _Internal;

  template <typename T> friend class ::PROTOBUF_NAMESPACE_ID::Arena::InternalHelper;
  typedef void InternalArenaConstructable_;
  typedef void DestructorSkippable_;
  struct Impl_ {
    ::PROTOBUF_NAMESPACE_ID::internal::HasBits<1> _has_bits_;
    mutable ::PROTOBUF_NAMESPACE_ID::internal::CachedSize _cached_size_;
    ::PROTOBUF_NAMESPACE_ID::internal::ArenaStringPtr event_name_;
    ::PROTOBUF_NAMESPACE_ID::internal::ArenaStringPtr game_message_;
    int alliance_;
    int match_type_;
    double battery_volts_;
    int driver_station_;
    int32_t match_numer_;
    double match_seconds_;
    int32_t replay_number_;
    bool is_ds_attached_;
    bool is_fms_attached_;
    bool is_enabled_;
    bool is_disabled_;
    bool is_auto_;
    bool is_teleop_;
    bool is_test_;
    bool is_e_stopped_;
  };
  union { Impl_ _impl_; };
  friend struct ::TableStruct_fms_5fstatus_2eproto;
};
// ===================================================================


// ===================================================================

#ifdef __GNUC__
  #pragma GCC diagnostic push
  #pragma GCC diagnostic ignored "-Wstrict-aliasing"
#endif  // __GNUC__
// FMS_Status

// optional .messages.Alliance alliance = 1;
inline bool FMS_Status::_internal_has_alliance() const {
  bool value = (_impl_._has_bits_[0] & 0x00000004u) != 0;
  return value;
}
inline bool FMS_Status::has_alliance() const {
  return _internal_has_alliance();
}
inline void FMS_Status::clear_alliance() {
  _impl_.alliance_ = 0;
  _impl_._has_bits_[0] &= ~0x00000004u;
}
inline ::messages::Alliance FMS_Status::_internal_alliance() const {
  return static_cast< ::messages::Alliance >(_impl_.alliance_);
}
inline ::messages::Alliance FMS_Status::alliance() const {
  // @@protoc_insertion_point(field_get:messages.FMS_Status.alliance)
  return _internal_alliance();
}
inline void FMS_Status::_internal_set_alliance(::messages::Alliance value) {
  _impl_._has_bits_[0] |= 0x00000004u;
  _impl_.alliance_ = value;
}
inline void FMS_Status::set_alliance(::messages::Alliance value) {
  _internal_set_alliance(value);
  // @@protoc_insertion_point(field_set:messages.FMS_Status.alliance)
}

// optional .messages.Match_Type match_type = 2;
inline bool FMS_Status::_internal_has_match_type() const {
  bool value = (_impl_._has_bits_[0] & 0x00000008u) != 0;
  return value;
}
inline bool FMS_Status::has_match_type() const {
  return _internal_has_match_type();
}
inline void FMS_Status::clear_match_type() {
  _impl_.match_type_ = 0;
  _impl_._has_bits_[0] &= ~0x00000008u;
}
inline ::messages::Match_Type FMS_Status::_internal_match_type() const {
  return static_cast< ::messages::Match_Type >(_impl_.match_type_);
}
inline ::messages::Match_Type FMS_Status::match_type() const {
  // @@protoc_insertion_point(field_get:messages.FMS_Status.match_type)
  return _internal_match_type();
}
inline void FMS_Status::_internal_set_match_type(::messages::Match_Type value) {
  _impl_._has_bits_[0] |= 0x00000008u;
  _impl_.match_type_ = value;
}
inline void FMS_Status::set_match_type(::messages::Match_Type value) {
  _internal_set_match_type(value);
  // @@protoc_insertion_point(field_set:messages.FMS_Status.match_type)
}

// optional .messages.Driver_Station driver_station = 3;
inline bool FMS_Status::_internal_has_driver_station() const {
  bool value = (_impl_._has_bits_[0] & 0x00000020u) != 0;
  return value;
}
inline bool FMS_Status::has_driver_station() const {
  return _internal_has_driver_station();
}
inline void FMS_Status::clear_driver_station() {
  _impl_.driver_station_ = 0;
  _impl_._has_bits_[0] &= ~0x00000020u;
}
inline ::messages::Driver_Station FMS_Status::_internal_driver_station() const {
  return static_cast< ::messages::Driver_Station >(_impl_.driver_station_);
}
inline ::messages::Driver_Station FMS_Status::driver_station() const {
  // @@protoc_insertion_point(field_get:messages.FMS_Status.driver_station)
  return _internal_driver_station();
}
inline void FMS_Status::_internal_set_driver_station(::messages::Driver_Station value) {
  _impl_._has_bits_[0] |= 0x00000020u;
  _impl_.driver_station_ = value;
}
inline void FMS_Status::set_driver_station(::messages::Driver_Station value) {
  _internal_set_driver_station(value);
  // @@protoc_insertion_point(field_set:messages.FMS_Status.driver_station)
}

// optional double battery_volts = 4;
inline bool FMS_Status::_internal_has_battery_volts() const {
  bool value = (_impl_._has_bits_[0] & 0x00000010u) != 0;
  return value;
}
inline bool FMS_Status::has_battery_volts() const {
  return _internal_has_battery_volts();
}
inline void FMS_Status::clear_battery_volts() {
  _impl_.battery_volts_ = 0;
  _impl_._has_bits_[0] &= ~0x00000010u;
}
inline double FMS_Status::_internal_battery_volts() const {
  return _impl_.battery_volts_;
}
inline double FMS_Status::battery_volts() const {
  // @@protoc_insertion_point(field_get:messages.FMS_Status.battery_volts)
  return _internal_battery_volts();
}
inline void FMS_Status::_internal_set_battery_volts(double value) {
  _impl_._has_bits_[0] |= 0x00000010u;
  _impl_.battery_volts_ = value;
}
inline void FMS_Status::set_battery_volts(double value) {
  _internal_set_battery_volts(value);
  // @@protoc_insertion_point(field_set:messages.FMS_Status.battery_volts)
}

// optional string event_name = 5;
inline bool FMS_Status::_internal_has_event_name() const {
  bool value = (_impl_._has_bits_[0] & 0x00000001u) != 0;
  return value;
}
inline bool FMS_Status::has_event_name() const {
  return _internal_has_event_name();
}
inline void FMS_Status::clear_event_name() {
  _impl_.event_name_.ClearToEmpty();
  _impl_._has_bits_[0] &= ~0x00000001u;
}
inline const std::string& FMS_Status::event_name() const {
  // @@protoc_insertion_point(field_get:messages.FMS_Status.event_name)
  return _internal_event_name();
}
template <typename ArgT0, typename... ArgT>
inline PROTOBUF_ALWAYS_INLINE
void FMS_Status::set_event_name(ArgT0&& arg0, ArgT... args) {
 _impl_._has_bits_[0] |= 0x00000001u;
 _impl_.event_name_.Set(static_cast<ArgT0 &&>(arg0), args..., GetArenaForAllocation());
  // @@protoc_insertion_point(field_set:messages.FMS_Status.event_name)
}
inline std::string* FMS_Status::mutable_event_name() {
  std::string* _s = _internal_mutable_event_name();
  // @@protoc_insertion_point(field_mutable:messages.FMS_Status.event_name)
  return _s;
}
inline const std::string& FMS_Status::_internal_event_name() const {
  return _impl_.event_name_.Get();
}
inline void FMS_Status::_internal_set_event_name(const std::string& value) {
  _impl_._has_bits_[0] |= 0x00000001u;
  _impl_.event_name_.Set(value, GetArenaForAllocation());
}
inline std::string* FMS_Status::_internal_mutable_event_name() {
  _impl_._has_bits_[0] |= 0x00000001u;
  return _impl_.event_name_.Mutable(GetArenaForAllocation());
}
inline std::string* FMS_Status::release_event_name() {
  // @@protoc_insertion_point(field_release:messages.FMS_Status.event_name)
  if (!_internal_has_event_name()) {
    return nullptr;
  }
  _impl_._has_bits_[0] &= ~0x00000001u;
  auto* p = _impl_.event_name_.Release();
#ifdef PROTOBUF_FORCE_COPY_DEFAULT_STRING
  if (_impl_.event_name_.IsDefault()) {
    _impl_.event_name_.Set("", GetArenaForAllocation());
  }
#endif // PROTOBUF_FORCE_COPY_DEFAULT_STRING
  return p;
}
inline void FMS_Status::set_allocated_event_name(std::string* event_name) {
  if (event_name != nullptr) {
    _impl_._has_bits_[0] |= 0x00000001u;
  } else {
    _impl_._has_bits_[0] &= ~0x00000001u;
  }
  _impl_.event_name_.SetAllocated(event_name, GetArenaForAllocation());
#ifdef PROTOBUF_FORCE_COPY_DEFAULT_STRING
  if (_impl_.event_name_.IsDefault()) {
    _impl_.event_name_.Set("", GetArenaForAllocation());
  }
#endif // PROTOBUF_FORCE_COPY_DEFAULT_STRING
  // @@protoc_insertion_point(field_set_allocated:messages.FMS_Status.event_name)
}

// optional string game_message = 6;
inline bool FMS_Status::_internal_has_game_message() const {
  bool value = (_impl_._has_bits_[0] & 0x00000002u) != 0;
  return value;
}
inline bool FMS_Status::has_game_message() const {
  return _internal_has_game_message();
}
inline void FMS_Status::clear_game_message() {
  _impl_.game_message_.ClearToEmpty();
  _impl_._has_bits_[0] &= ~0x00000002u;
}
inline const std::string& FMS_Status::game_message() const {
  // @@protoc_insertion_point(field_get:messages.FMS_Status.game_message)
  return _internal_game_message();
}
template <typename ArgT0, typename... ArgT>
inline PROTOBUF_ALWAYS_INLINE
void FMS_Status::set_game_message(ArgT0&& arg0, ArgT... args) {
 _impl_._has_bits_[0] |= 0x00000002u;
 _impl_.game_message_.Set(static_cast<ArgT0 &&>(arg0), args..., GetArenaForAllocation());
  // @@protoc_insertion_point(field_set:messages.FMS_Status.game_message)
}
inline std::string* FMS_Status::mutable_game_message() {
  std::string* _s = _internal_mutable_game_message();
  // @@protoc_insertion_point(field_mutable:messages.FMS_Status.game_message)
  return _s;
}
inline const std::string& FMS_Status::_internal_game_message() const {
  return _impl_.game_message_.Get();
}
inline void FMS_Status::_internal_set_game_message(const std::string& value) {
  _impl_._has_bits_[0] |= 0x00000002u;
  _impl_.game_message_.Set(value, GetArenaForAllocation());
}
inline std::string* FMS_Status::_internal_mutable_game_message() {
  _impl_._has_bits_[0] |= 0x00000002u;
  return _impl_.game_message_.Mutable(GetArenaForAllocation());
}
inline std::string* FMS_Status::release_game_message() {
  // @@protoc_insertion_point(field_release:messages.FMS_Status.game_message)
  if (!_internal_has_game_message()) {
    return nullptr;
  }
  _impl_._has_bits_[0] &= ~0x00000002u;
  auto* p = _impl_.game_message_.Release();
#ifdef PROTOBUF_FORCE_COPY_DEFAULT_STRING
  if (_impl_.game_message_.IsDefault()) {
    _impl_.game_message_.Set("", GetArenaForAllocation());
  }
#endif // PROTOBUF_FORCE_COPY_DEFAULT_STRING
  return p;
}
inline void FMS_Status::set_allocated_game_message(std::string* game_message) {
  if (game_message != nullptr) {
    _impl_._has_bits_[0] |= 0x00000002u;
  } else {
    _impl_._has_bits_[0] &= ~0x00000002u;
  }
  _impl_.game_message_.SetAllocated(game_message, GetArenaForAllocation());
#ifdef PROTOBUF_FORCE_COPY_DEFAULT_STRING
  if (_impl_.game_message_.IsDefault()) {
    _impl_.game_message_.Set("", GetArenaForAllocation());
  }
#endif // PROTOBUF_FORCE_COPY_DEFAULT_STRING
  // @@protoc_insertion_point(field_set_allocated:messages.FMS_Status.game_message)
}

// optional int32 match_numer = 7;
inline bool FMS_Status::_internal_has_match_numer() const {
  bool value = (_impl_._has_bits_[0] & 0x00000040u) != 0;
  return value;
}
inline bool FMS_Status::has_match_numer() const {
  return _internal_has_match_numer();
}
inline void FMS_Status::clear_match_numer() {
  _impl_.match_numer_ = 0;
  _impl_._has_bits_[0] &= ~0x00000040u;
}
inline int32_t FMS_Status::_internal_match_numer() const {
  return _impl_.match_numer_;
}
inline int32_t FMS_Status::match_numer() const {
  // @@protoc_insertion_point(field_get:messages.FMS_Status.match_numer)
  return _internal_match_numer();
}
inline void FMS_Status::_internal_set_match_numer(int32_t value) {
  _impl_._has_bits_[0] |= 0x00000040u;
  _impl_.match_numer_ = value;
}
inline void FMS_Status::set_match_numer(int32_t value) {
  _internal_set_match_numer(value);
  // @@protoc_insertion_point(field_set:messages.FMS_Status.match_numer)
}

// optional double match_seconds = 8;
inline bool FMS_Status::_internal_has_match_seconds() const {
  bool value = (_impl_._has_bits_[0] & 0x00000080u) != 0;
  return value;
}
inline bool FMS_Status::has_match_seconds() const {
  return _internal_has_match_seconds();
}
inline void FMS_Status::clear_match_seconds() {
  _impl_.match_seconds_ = 0;
  _impl_._has_bits_[0] &= ~0x00000080u;
}
inline double FMS_Status::_internal_match_seconds() const {
  return _impl_.match_seconds_;
}
inline double FMS_Status::match_seconds() const {
  // @@protoc_insertion_point(field_get:messages.FMS_Status.match_seconds)
  return _internal_match_seconds();
}
inline void FMS_Status::_internal_set_match_seconds(double value) {
  _impl_._has_bits_[0] |= 0x00000080u;
  _impl_.match_seconds_ = value;
}
inline void FMS_Status::set_match_seconds(double value) {
  _internal_set_match_seconds(value);
  // @@protoc_insertion_point(field_set:messages.FMS_Status.match_seconds)
}

// optional int32 replay_number = 9;
inline bool FMS_Status::_internal_has_replay_number() const {
  bool value = (_impl_._has_bits_[0] & 0x00000100u) != 0;
  return value;
}
inline bool FMS_Status::has_replay_number() const {
  return _internal_has_replay_number();
}
inline void FMS_Status::clear_replay_number() {
  _impl_.replay_number_ = 0;
  _impl_._has_bits_[0] &= ~0x00000100u;
}
inline int32_t FMS_Status::_internal_replay_number() const {
  return _impl_.replay_number_;
}
inline int32_t FMS_Status::replay_number() const {
  // @@protoc_insertion_point(field_get:messages.FMS_Status.replay_number)
  return _internal_replay_number();
}
inline void FMS_Status::_internal_set_replay_number(int32_t value) {
  _impl_._has_bits_[0] |= 0x00000100u;
  _impl_.replay_number_ = value;
}
inline void FMS_Status::set_replay_number(int32_t value) {
  _internal_set_replay_number(value);
  // @@protoc_insertion_point(field_set:messages.FMS_Status.replay_number)
}

// optional bool is_ds_attached = 10;
inline bool FMS_Status::_internal_has_is_ds_attached() const {
  bool value = (_impl_._has_bits_[0] & 0x00000200u) != 0;
  return value;
}
inline bool FMS_Status::has_is_ds_attached() const {
  return _internal_has_is_ds_attached();
}
inline void FMS_Status::clear_is_ds_attached() {
  _impl_.is_ds_attached_ = false;
  _impl_._has_bits_[0] &= ~0x00000200u;
}
inline bool FMS_Status::_internal_is_ds_attached() const {
  return _impl_.is_ds_attached_;
}
inline bool FMS_Status::is_ds_attached() const {
  // @@protoc_insertion_point(field_get:messages.FMS_Status.is_ds_attached)
  return _internal_is_ds_attached();
}
inline void FMS_Status::_internal_set_is_ds_attached(bool value) {
  _impl_._has_bits_[0] |= 0x00000200u;
  _impl_.is_ds_attached_ = value;
}
inline void FMS_Status::set_is_ds_attached(bool value) {
  _internal_set_is_ds_attached(value);
  // @@protoc_insertion_point(field_set:messages.FMS_Status.is_ds_attached)
}

// optional bool is_fms_attached = 11;
inline bool FMS_Status::_internal_has_is_fms_attached() const {
  bool value = (_impl_._has_bits_[0] & 0x00000400u) != 0;
  return value;
}
inline bool FMS_Status::has_is_fms_attached() const {
  return _internal_has_is_fms_attached();
}
inline void FMS_Status::clear_is_fms_attached() {
  _impl_.is_fms_attached_ = false;
  _impl_._has_bits_[0] &= ~0x00000400u;
}
inline bool FMS_Status::_internal_is_fms_attached() const {
  return _impl_.is_fms_attached_;
}
inline bool FMS_Status::is_fms_attached() const {
  // @@protoc_insertion_point(field_get:messages.FMS_Status.is_fms_attached)
  return _internal_is_fms_attached();
}
inline void FMS_Status::_internal_set_is_fms_attached(bool value) {
  _impl_._has_bits_[0] |= 0x00000400u;
  _impl_.is_fms_attached_ = value;
}
inline void FMS_Status::set_is_fms_attached(bool value) {
  _internal_set_is_fms_attached(value);
  // @@protoc_insertion_point(field_set:messages.FMS_Status.is_fms_attached)
}

// optional bool is_enabled = 12;
inline bool FMS_Status::_internal_has_is_enabled() const {
  bool value = (_impl_._has_bits_[0] & 0x00000800u) != 0;
  return value;
}
inline bool FMS_Status::has_is_enabled() const {
  return _internal_has_is_enabled();
}
inline void FMS_Status::clear_is_enabled() {
  _impl_.is_enabled_ = false;
  _impl_._has_bits_[0] &= ~0x00000800u;
}
inline bool FMS_Status::_internal_is_enabled() const {
  return _impl_.is_enabled_;
}
inline bool FMS_Status::is_enabled() const {
  // @@protoc_insertion_point(field_get:messages.FMS_Status.is_enabled)
  return _internal_is_enabled();
}
inline void FMS_Status::_internal_set_is_enabled(bool value) {
  _impl_._has_bits_[0] |= 0x00000800u;
  _impl_.is_enabled_ = value;
}
inline void FMS_Status::set_is_enabled(bool value) {
  _internal_set_is_enabled(value);
  // @@protoc_insertion_point(field_set:messages.FMS_Status.is_enabled)
}

// optional bool is_disabled = 13;
inline bool FMS_Status::_internal_has_is_disabled() const {
  bool value = (_impl_._has_bits_[0] & 0x00001000u) != 0;
  return value;
}
inline bool FMS_Status::has_is_disabled() const {
  return _internal_has_is_disabled();
}
inline void FMS_Status::clear_is_disabled() {
  _impl_.is_disabled_ = false;
  _impl_._has_bits_[0] &= ~0x00001000u;
}
inline bool FMS_Status::_internal_is_disabled() const {
  return _impl_.is_disabled_;
}
inline bool FMS_Status::is_disabled() const {
  // @@protoc_insertion_point(field_get:messages.FMS_Status.is_disabled)
  return _internal_is_disabled();
}
inline void FMS_Status::_internal_set_is_disabled(bool value) {
  _impl_._has_bits_[0] |= 0x00001000u;
  _impl_.is_disabled_ = value;
}
inline void FMS_Status::set_is_disabled(bool value) {
  _internal_set_is_disabled(value);
  // @@protoc_insertion_point(field_set:messages.FMS_Status.is_disabled)
}

// optional bool is_auto = 14;
inline bool FMS_Status::_internal_has_is_auto() const {
  bool value = (_impl_._has_bits_[0] & 0x00002000u) != 0;
  return value;
}
inline bool FMS_Status::has_is_auto() const {
  return _internal_has_is_auto();
}
inline void FMS_Status::clear_is_auto() {
  _impl_.is_auto_ = false;
  _impl_._has_bits_[0] &= ~0x00002000u;
}
inline bool FMS_Status::_internal_is_auto() const {
  return _impl_.is_auto_;
}
inline bool FMS_Status::is_auto() const {
  // @@protoc_insertion_point(field_get:messages.FMS_Status.is_auto)
  return _internal_is_auto();
}
inline void FMS_Status::_internal_set_is_auto(bool value) {
  _impl_._has_bits_[0] |= 0x00002000u;
  _impl_.is_auto_ = value;
}
inline void FMS_Status::set_is_auto(bool value) {
  _internal_set_is_auto(value);
  // @@protoc_insertion_point(field_set:messages.FMS_Status.is_auto)
}

// optional bool is_teleop = 15;
inline bool FMS_Status::_internal_has_is_teleop() const {
  bool value = (_impl_._has_bits_[0] & 0x00004000u) != 0;
  return value;
}
inline bool FMS_Status::has_is_teleop() const {
  return _internal_has_is_teleop();
}
inline void FMS_Status::clear_is_teleop() {
  _impl_.is_teleop_ = false;
  _impl_._has_bits_[0] &= ~0x00004000u;
}
inline bool FMS_Status::_internal_is_teleop() const {
  return _impl_.is_teleop_;
}
inline bool FMS_Status::is_teleop() const {
  // @@protoc_insertion_point(field_get:messages.FMS_Status.is_teleop)
  return _internal_is_teleop();
}
inline void FMS_Status::_internal_set_is_teleop(bool value) {
  _impl_._has_bits_[0] |= 0x00004000u;
  _impl_.is_teleop_ = value;
}
inline void FMS_Status::set_is_teleop(bool value) {
  _internal_set_is_teleop(value);
  // @@protoc_insertion_point(field_set:messages.FMS_Status.is_teleop)
}

// optional bool is_test = 16;
inline bool FMS_Status::_internal_has_is_test() const {
  bool value = (_impl_._has_bits_[0] & 0x00008000u) != 0;
  return value;
}
inline bool FMS_Status::has_is_test() const {
  return _internal_has_is_test();
}
inline void FMS_Status::clear_is_test() {
  _impl_.is_test_ = false;
  _impl_._has_bits_[0] &= ~0x00008000u;
}
inline bool FMS_Status::_internal_is_test() const {
  return _impl_.is_test_;
}
inline bool FMS_Status::is_test() const {
  // @@protoc_insertion_point(field_get:messages.FMS_Status.is_test)
  return _internal_is_test();
}
inline void FMS_Status::_internal_set_is_test(bool value) {
  _impl_._has_bits_[0] |= 0x00008000u;
  _impl_.is_test_ = value;
}
inline void FMS_Status::set_is_test(bool value) {
  _internal_set_is_test(value);
  // @@protoc_insertion_point(field_set:messages.FMS_Status.is_test)
}

// optional bool is_e_stopped = 17;
inline bool FMS_Status::_internal_has_is_e_stopped() const {
  bool value = (_impl_._has_bits_[0] & 0x00010000u) != 0;
  return value;
}
inline bool FMS_Status::has_is_e_stopped() const {
  return _internal_has_is_e_stopped();
}
inline void FMS_Status::clear_is_e_stopped() {
  _impl_.is_e_stopped_ = false;
  _impl_._has_bits_[0] &= ~0x00010000u;
}
inline bool FMS_Status::_internal_is_e_stopped() const {
  return _impl_.is_e_stopped_;
}
inline bool FMS_Status::is_e_stopped() const {
  // @@protoc_insertion_point(field_get:messages.FMS_Status.is_e_stopped)
  return _internal_is_e_stopped();
}
inline void FMS_Status::_internal_set_is_e_stopped(bool value) {
  _impl_._has_bits_[0] |= 0x00010000u;
  _impl_.is_e_stopped_ = value;
}
inline void FMS_Status::set_is_e_stopped(bool value) {
  _internal_set_is_e_stopped(value);
  // @@protoc_insertion_point(field_set:messages.FMS_Status.is_e_stopped)
}

#ifdef __GNUC__
  #pragma GCC diagnostic pop
#endif  // __GNUC__

// @@protoc_insertion_point(namespace_scope)

}  // namespace messages

PROTOBUF_NAMESPACE_OPEN

template <> struct is_proto_enum< ::messages::Alliance> : ::std::true_type {};
template <>
inline const EnumDescriptor* GetEnumDescriptor< ::messages::Alliance>() {
  return ::messages::Alliance_descriptor();
}
template <> struct is_proto_enum< ::messages::Match_Type> : ::std::true_type {};
template <>
inline const EnumDescriptor* GetEnumDescriptor< ::messages::Match_Type>() {
  return ::messages::Match_Type_descriptor();
}
template <> struct is_proto_enum< ::messages::Driver_Station> : ::std::true_type {};
template <>
inline const EnumDescriptor* GetEnumDescriptor< ::messages::Driver_Station>() {
  return ::messages::Driver_Station_descriptor();
}

PROTOBUF_NAMESPACE_CLOSE

// @@protoc_insertion_point(global_scope)

#include <google/protobuf/port_undef.inc>
#endif  // GOOGLE_PROTOBUF_INCLUDED_GOOGLE_PROTOBUF_INCLUDED_fms_5fstatus_2eproto
