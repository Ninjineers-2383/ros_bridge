// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: motor_control.proto

#ifndef GOOGLE_PROTOBUF_INCLUDED_motor_5fcontrol_2eproto
#define GOOGLE_PROTOBUF_INCLUDED_motor_5fcontrol_2eproto

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
#define PROTOBUF_INTERNAL_EXPORT_motor_5fcontrol_2eproto
PROTOBUF_NAMESPACE_OPEN
namespace internal {
class AnyMetadata;
}  // namespace internal
PROTOBUF_NAMESPACE_CLOSE

// Internal implementation detail -- do not use these members.
struct TableStruct_motor_5fcontrol_2eproto {
  static const uint32_t offsets[];
};
extern const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable descriptor_table_motor_5fcontrol_2eproto;
namespace messages {
namespace motor {
class Motor_Control_Rep;
struct Motor_Control_RepDefaultTypeInternal;
extern Motor_Control_RepDefaultTypeInternal _Motor_Control_Rep_default_instance_;
class Motor_Control_Req;
struct Motor_Control_ReqDefaultTypeInternal;
extern Motor_Control_ReqDefaultTypeInternal _Motor_Control_Req_default_instance_;
}  // namespace motor
}  // namespace messages
PROTOBUF_NAMESPACE_OPEN
template<> ::messages::motor::Motor_Control_Rep* Arena::CreateMaybeMessage<::messages::motor::Motor_Control_Rep>(Arena*);
template<> ::messages::motor::Motor_Control_Req* Arena::CreateMaybeMessage<::messages::motor::Motor_Control_Req>(Arena*);
PROTOBUF_NAMESPACE_CLOSE
namespace messages {
namespace motor {

enum MotorControlType : int {
  CONTROL_OFF = 0,
  CONTROL_VOLTAGE = 1,
  CONTROL_PID_FF = 2,
  CONTROL_EFFORT = 3,
  MotorControlType_INT_MIN_SENTINEL_DO_NOT_USE_ = std::numeric_limits<int32_t>::min(),
  MotorControlType_INT_MAX_SENTINEL_DO_NOT_USE_ = std::numeric_limits<int32_t>::max()
};
bool MotorControlType_IsValid(int value);
constexpr MotorControlType MotorControlType_MIN = CONTROL_OFF;
constexpr MotorControlType MotorControlType_MAX = CONTROL_EFFORT;
constexpr int MotorControlType_ARRAYSIZE = MotorControlType_MAX + 1;

const ::PROTOBUF_NAMESPACE_ID::EnumDescriptor* MotorControlType_descriptor();
template<typename T>
inline const std::string& MotorControlType_Name(T enum_t_value) {
  static_assert(::std::is_same<T, MotorControlType>::value ||
    ::std::is_integral<T>::value,
    "Incorrect type passed to function MotorControlType_Name.");
  return ::PROTOBUF_NAMESPACE_ID::internal::NameOfEnum(
    MotorControlType_descriptor(), enum_t_value);
}
inline bool MotorControlType_Parse(
    ::PROTOBUF_NAMESPACE_ID::ConstStringParam name, MotorControlType* value) {
  return ::PROTOBUF_NAMESPACE_ID::internal::ParseNamedEnum<MotorControlType>(
    MotorControlType_descriptor(), name, value);
}
// ===================================================================

class Motor_Control_Req final :
    public ::PROTOBUF_NAMESPACE_ID::Message /* @@protoc_insertion_point(class_definition:messages.motor.Motor_Control_Req) */ {
 public:
  inline Motor_Control_Req() : Motor_Control_Req(nullptr) {}
  ~Motor_Control_Req() override;
  explicit PROTOBUF_CONSTEXPR Motor_Control_Req(::PROTOBUF_NAMESPACE_ID::internal::ConstantInitialized);

  Motor_Control_Req(const Motor_Control_Req& from);
  Motor_Control_Req(Motor_Control_Req&& from) noexcept
    : Motor_Control_Req() {
    *this = ::std::move(from);
  }

  inline Motor_Control_Req& operator=(const Motor_Control_Req& from) {
    CopyFrom(from);
    return *this;
  }
  inline Motor_Control_Req& operator=(Motor_Control_Req&& from) noexcept {
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
  static const Motor_Control_Req& default_instance() {
    return *internal_default_instance();
  }
  static inline const Motor_Control_Req* internal_default_instance() {
    return reinterpret_cast<const Motor_Control_Req*>(
               &_Motor_Control_Req_default_instance_);
  }
  static constexpr int kIndexInFileMessages =
    0;

  friend void swap(Motor_Control_Req& a, Motor_Control_Req& b) {
    a.Swap(&b);
  }
  inline void Swap(Motor_Control_Req* other) {
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
  void UnsafeArenaSwap(Motor_Control_Req* other) {
    if (other == this) return;
    GOOGLE_DCHECK(GetOwningArena() == other->GetOwningArena());
    InternalSwap(other);
  }

  // implements Message ----------------------------------------------

  Motor_Control_Req* New(::PROTOBUF_NAMESPACE_ID::Arena* arena = nullptr) const final {
    return CreateMaybeMessage<Motor_Control_Req>(arena);
  }
  using ::PROTOBUF_NAMESPACE_ID::Message::CopyFrom;
  void CopyFrom(const Motor_Control_Req& from);
  using ::PROTOBUF_NAMESPACE_ID::Message::MergeFrom;
  void MergeFrom( const Motor_Control_Req& from) {
    Motor_Control_Req::MergeImpl(*this, from);
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
  void InternalSwap(Motor_Control_Req* other);

  private:
  friend class ::PROTOBUF_NAMESPACE_ID::internal::AnyMetadata;
  static ::PROTOBUF_NAMESPACE_ID::StringPiece FullMessageName() {
    return "messages.motor.Motor_Control_Req";
  }
  protected:
  explicit Motor_Control_Req(::PROTOBUF_NAMESPACE_ID::Arena* arena,
                       bool is_message_owned = false);
  public:

  static const ClassData _class_data_;
  const ::PROTOBUF_NAMESPACE_ID::Message::ClassData*GetClassData() const final;

  ::PROTOBUF_NAMESPACE_ID::Metadata GetMetadata() const final;

  // nested types ----------------------------------------------------

  // accessors -------------------------------------------------------

  enum : int {
    kVoltageFieldNumber = 2,
    kPositionFieldNumber = 3,
    kVelocityFieldNumber = 4,
    kControlFieldNumber = 1,
    kMotorIdFieldNumber = 7,
    kAccelerationFieldNumber = 5,
    kEffortFieldNumber = 6,
  };
  // optional double voltage = 2;
  bool has_voltage() const;
  private:
  bool _internal_has_voltage() const;
  public:
  void clear_voltage();
  double voltage() const;
  void set_voltage(double value);
  private:
  double _internal_voltage() const;
  void _internal_set_voltage(double value);
  public:

  // optional double position = 3;
  bool has_position() const;
  private:
  bool _internal_has_position() const;
  public:
  void clear_position();
  double position() const;
  void set_position(double value);
  private:
  double _internal_position() const;
  void _internal_set_position(double value);
  public:

  // optional double velocity = 4;
  bool has_velocity() const;
  private:
  bool _internal_has_velocity() const;
  public:
  void clear_velocity();
  double velocity() const;
  void set_velocity(double value);
  private:
  double _internal_velocity() const;
  void _internal_set_velocity(double value);
  public:

  // optional .messages.motor.MotorControlType control = 1;
  bool has_control() const;
  private:
  bool _internal_has_control() const;
  public:
  void clear_control();
  ::messages::motor::MotorControlType control() const;
  void set_control(::messages::motor::MotorControlType value);
  private:
  ::messages::motor::MotorControlType _internal_control() const;
  void _internal_set_control(::messages::motor::MotorControlType value);
  public:

  // optional int32 motor_id = 7;
  bool has_motor_id() const;
  private:
  bool _internal_has_motor_id() const;
  public:
  void clear_motor_id();
  int32_t motor_id() const;
  void set_motor_id(int32_t value);
  private:
  int32_t _internal_motor_id() const;
  void _internal_set_motor_id(int32_t value);
  public:

  // optional double acceleration = 5;
  bool has_acceleration() const;
  private:
  bool _internal_has_acceleration() const;
  public:
  void clear_acceleration();
  double acceleration() const;
  void set_acceleration(double value);
  private:
  double _internal_acceleration() const;
  void _internal_set_acceleration(double value);
  public:

  // optional double effort = 6;
  bool has_effort() const;
  private:
  bool _internal_has_effort() const;
  public:
  void clear_effort();
  double effort() const;
  void set_effort(double value);
  private:
  double _internal_effort() const;
  void _internal_set_effort(double value);
  public:

  // @@protoc_insertion_point(class_scope:messages.motor.Motor_Control_Req)
 private:
  class _Internal;

  template <typename T> friend class ::PROTOBUF_NAMESPACE_ID::Arena::InternalHelper;
  typedef void InternalArenaConstructable_;
  typedef void DestructorSkippable_;
  struct Impl_ {
    ::PROTOBUF_NAMESPACE_ID::internal::HasBits<1> _has_bits_;
    mutable ::PROTOBUF_NAMESPACE_ID::internal::CachedSize _cached_size_;
    double voltage_;
    double position_;
    double velocity_;
    int control_;
    int32_t motor_id_;
    double acceleration_;
    double effort_;
  };
  union { Impl_ _impl_; };
  friend struct ::TableStruct_motor_5fcontrol_2eproto;
};
// -------------------------------------------------------------------

class Motor_Control_Rep final :
    public ::PROTOBUF_NAMESPACE_ID::Message /* @@protoc_insertion_point(class_definition:messages.motor.Motor_Control_Rep) */ {
 public:
  inline Motor_Control_Rep() : Motor_Control_Rep(nullptr) {}
  ~Motor_Control_Rep() override;
  explicit PROTOBUF_CONSTEXPR Motor_Control_Rep(::PROTOBUF_NAMESPACE_ID::internal::ConstantInitialized);

  Motor_Control_Rep(const Motor_Control_Rep& from);
  Motor_Control_Rep(Motor_Control_Rep&& from) noexcept
    : Motor_Control_Rep() {
    *this = ::std::move(from);
  }

  inline Motor_Control_Rep& operator=(const Motor_Control_Rep& from) {
    CopyFrom(from);
    return *this;
  }
  inline Motor_Control_Rep& operator=(Motor_Control_Rep&& from) noexcept {
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
  static const Motor_Control_Rep& default_instance() {
    return *internal_default_instance();
  }
  static inline const Motor_Control_Rep* internal_default_instance() {
    return reinterpret_cast<const Motor_Control_Rep*>(
               &_Motor_Control_Rep_default_instance_);
  }
  static constexpr int kIndexInFileMessages =
    1;

  friend void swap(Motor_Control_Rep& a, Motor_Control_Rep& b) {
    a.Swap(&b);
  }
  inline void Swap(Motor_Control_Rep* other) {
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
  void UnsafeArenaSwap(Motor_Control_Rep* other) {
    if (other == this) return;
    GOOGLE_DCHECK(GetOwningArena() == other->GetOwningArena());
    InternalSwap(other);
  }

  // implements Message ----------------------------------------------

  Motor_Control_Rep* New(::PROTOBUF_NAMESPACE_ID::Arena* arena = nullptr) const final {
    return CreateMaybeMessage<Motor_Control_Rep>(arena);
  }
  using ::PROTOBUF_NAMESPACE_ID::Message::CopyFrom;
  void CopyFrom(const Motor_Control_Rep& from);
  using ::PROTOBUF_NAMESPACE_ID::Message::MergeFrom;
  void MergeFrom( const Motor_Control_Rep& from) {
    Motor_Control_Rep::MergeImpl(*this, from);
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
  void InternalSwap(Motor_Control_Rep* other);

  private:
  friend class ::PROTOBUF_NAMESPACE_ID::internal::AnyMetadata;
  static ::PROTOBUF_NAMESPACE_ID::StringPiece FullMessageName() {
    return "messages.motor.Motor_Control_Rep";
  }
  protected:
  explicit Motor_Control_Rep(::PROTOBUF_NAMESPACE_ID::Arena* arena,
                       bool is_message_owned = false);
  public:

  static const ClassData _class_data_;
  const ::PROTOBUF_NAMESPACE_ID::Message::ClassData*GetClassData() const final;

  ::PROTOBUF_NAMESPACE_ID::Metadata GetMetadata() const final;

  // nested types ----------------------------------------------------

  // accessors -------------------------------------------------------

  enum : int {
    kSuccessFieldNumber = 1,
  };
  // bool success = 1;
  void clear_success();
  bool success() const;
  void set_success(bool value);
  private:
  bool _internal_success() const;
  void _internal_set_success(bool value);
  public:

  // @@protoc_insertion_point(class_scope:messages.motor.Motor_Control_Rep)
 private:
  class _Internal;

  template <typename T> friend class ::PROTOBUF_NAMESPACE_ID::Arena::InternalHelper;
  typedef void InternalArenaConstructable_;
  typedef void DestructorSkippable_;
  struct Impl_ {
    bool success_;
    mutable ::PROTOBUF_NAMESPACE_ID::internal::CachedSize _cached_size_;
  };
  union { Impl_ _impl_; };
  friend struct ::TableStruct_motor_5fcontrol_2eproto;
};
// ===================================================================


// ===================================================================

#ifdef __GNUC__
  #pragma GCC diagnostic push
  #pragma GCC diagnostic ignored "-Wstrict-aliasing"
#endif  // __GNUC__
// Motor_Control_Req

// optional .messages.motor.MotorControlType control = 1;
inline bool Motor_Control_Req::_internal_has_control() const {
  bool value = (_impl_._has_bits_[0] & 0x00000008u) != 0;
  return value;
}
inline bool Motor_Control_Req::has_control() const {
  return _internal_has_control();
}
inline void Motor_Control_Req::clear_control() {
  _impl_.control_ = 0;
  _impl_._has_bits_[0] &= ~0x00000008u;
}
inline ::messages::motor::MotorControlType Motor_Control_Req::_internal_control() const {
  return static_cast< ::messages::motor::MotorControlType >(_impl_.control_);
}
inline ::messages::motor::MotorControlType Motor_Control_Req::control() const {
  // @@protoc_insertion_point(field_get:messages.motor.Motor_Control_Req.control)
  return _internal_control();
}
inline void Motor_Control_Req::_internal_set_control(::messages::motor::MotorControlType value) {
  _impl_._has_bits_[0] |= 0x00000008u;
  _impl_.control_ = value;
}
inline void Motor_Control_Req::set_control(::messages::motor::MotorControlType value) {
  _internal_set_control(value);
  // @@protoc_insertion_point(field_set:messages.motor.Motor_Control_Req.control)
}

// optional int32 motor_id = 7;
inline bool Motor_Control_Req::_internal_has_motor_id() const {
  bool value = (_impl_._has_bits_[0] & 0x00000010u) != 0;
  return value;
}
inline bool Motor_Control_Req::has_motor_id() const {
  return _internal_has_motor_id();
}
inline void Motor_Control_Req::clear_motor_id() {
  _impl_.motor_id_ = 0;
  _impl_._has_bits_[0] &= ~0x00000010u;
}
inline int32_t Motor_Control_Req::_internal_motor_id() const {
  return _impl_.motor_id_;
}
inline int32_t Motor_Control_Req::motor_id() const {
  // @@protoc_insertion_point(field_get:messages.motor.Motor_Control_Req.motor_id)
  return _internal_motor_id();
}
inline void Motor_Control_Req::_internal_set_motor_id(int32_t value) {
  _impl_._has_bits_[0] |= 0x00000010u;
  _impl_.motor_id_ = value;
}
inline void Motor_Control_Req::set_motor_id(int32_t value) {
  _internal_set_motor_id(value);
  // @@protoc_insertion_point(field_set:messages.motor.Motor_Control_Req.motor_id)
}

// optional double voltage = 2;
inline bool Motor_Control_Req::_internal_has_voltage() const {
  bool value = (_impl_._has_bits_[0] & 0x00000001u) != 0;
  return value;
}
inline bool Motor_Control_Req::has_voltage() const {
  return _internal_has_voltage();
}
inline void Motor_Control_Req::clear_voltage() {
  _impl_.voltage_ = 0;
  _impl_._has_bits_[0] &= ~0x00000001u;
}
inline double Motor_Control_Req::_internal_voltage() const {
  return _impl_.voltage_;
}
inline double Motor_Control_Req::voltage() const {
  // @@protoc_insertion_point(field_get:messages.motor.Motor_Control_Req.voltage)
  return _internal_voltage();
}
inline void Motor_Control_Req::_internal_set_voltage(double value) {
  _impl_._has_bits_[0] |= 0x00000001u;
  _impl_.voltage_ = value;
}
inline void Motor_Control_Req::set_voltage(double value) {
  _internal_set_voltage(value);
  // @@protoc_insertion_point(field_set:messages.motor.Motor_Control_Req.voltage)
}

// optional double position = 3;
inline bool Motor_Control_Req::_internal_has_position() const {
  bool value = (_impl_._has_bits_[0] & 0x00000002u) != 0;
  return value;
}
inline bool Motor_Control_Req::has_position() const {
  return _internal_has_position();
}
inline void Motor_Control_Req::clear_position() {
  _impl_.position_ = 0;
  _impl_._has_bits_[0] &= ~0x00000002u;
}
inline double Motor_Control_Req::_internal_position() const {
  return _impl_.position_;
}
inline double Motor_Control_Req::position() const {
  // @@protoc_insertion_point(field_get:messages.motor.Motor_Control_Req.position)
  return _internal_position();
}
inline void Motor_Control_Req::_internal_set_position(double value) {
  _impl_._has_bits_[0] |= 0x00000002u;
  _impl_.position_ = value;
}
inline void Motor_Control_Req::set_position(double value) {
  _internal_set_position(value);
  // @@protoc_insertion_point(field_set:messages.motor.Motor_Control_Req.position)
}

// optional double velocity = 4;
inline bool Motor_Control_Req::_internal_has_velocity() const {
  bool value = (_impl_._has_bits_[0] & 0x00000004u) != 0;
  return value;
}
inline bool Motor_Control_Req::has_velocity() const {
  return _internal_has_velocity();
}
inline void Motor_Control_Req::clear_velocity() {
  _impl_.velocity_ = 0;
  _impl_._has_bits_[0] &= ~0x00000004u;
}
inline double Motor_Control_Req::_internal_velocity() const {
  return _impl_.velocity_;
}
inline double Motor_Control_Req::velocity() const {
  // @@protoc_insertion_point(field_get:messages.motor.Motor_Control_Req.velocity)
  return _internal_velocity();
}
inline void Motor_Control_Req::_internal_set_velocity(double value) {
  _impl_._has_bits_[0] |= 0x00000004u;
  _impl_.velocity_ = value;
}
inline void Motor_Control_Req::set_velocity(double value) {
  _internal_set_velocity(value);
  // @@protoc_insertion_point(field_set:messages.motor.Motor_Control_Req.velocity)
}

// optional double acceleration = 5;
inline bool Motor_Control_Req::_internal_has_acceleration() const {
  bool value = (_impl_._has_bits_[0] & 0x00000020u) != 0;
  return value;
}
inline bool Motor_Control_Req::has_acceleration() const {
  return _internal_has_acceleration();
}
inline void Motor_Control_Req::clear_acceleration() {
  _impl_.acceleration_ = 0;
  _impl_._has_bits_[0] &= ~0x00000020u;
}
inline double Motor_Control_Req::_internal_acceleration() const {
  return _impl_.acceleration_;
}
inline double Motor_Control_Req::acceleration() const {
  // @@protoc_insertion_point(field_get:messages.motor.Motor_Control_Req.acceleration)
  return _internal_acceleration();
}
inline void Motor_Control_Req::_internal_set_acceleration(double value) {
  _impl_._has_bits_[0] |= 0x00000020u;
  _impl_.acceleration_ = value;
}
inline void Motor_Control_Req::set_acceleration(double value) {
  _internal_set_acceleration(value);
  // @@protoc_insertion_point(field_set:messages.motor.Motor_Control_Req.acceleration)
}

// optional double effort = 6;
inline bool Motor_Control_Req::_internal_has_effort() const {
  bool value = (_impl_._has_bits_[0] & 0x00000040u) != 0;
  return value;
}
inline bool Motor_Control_Req::has_effort() const {
  return _internal_has_effort();
}
inline void Motor_Control_Req::clear_effort() {
  _impl_.effort_ = 0;
  _impl_._has_bits_[0] &= ~0x00000040u;
}
inline double Motor_Control_Req::_internal_effort() const {
  return _impl_.effort_;
}
inline double Motor_Control_Req::effort() const {
  // @@protoc_insertion_point(field_get:messages.motor.Motor_Control_Req.effort)
  return _internal_effort();
}
inline void Motor_Control_Req::_internal_set_effort(double value) {
  _impl_._has_bits_[0] |= 0x00000040u;
  _impl_.effort_ = value;
}
inline void Motor_Control_Req::set_effort(double value) {
  _internal_set_effort(value);
  // @@protoc_insertion_point(field_set:messages.motor.Motor_Control_Req.effort)
}

// -------------------------------------------------------------------

// Motor_Control_Rep

// bool success = 1;
inline void Motor_Control_Rep::clear_success() {
  _impl_.success_ = false;
}
inline bool Motor_Control_Rep::_internal_success() const {
  return _impl_.success_;
}
inline bool Motor_Control_Rep::success() const {
  // @@protoc_insertion_point(field_get:messages.motor.Motor_Control_Rep.success)
  return _internal_success();
}
inline void Motor_Control_Rep::_internal_set_success(bool value) {
  
  _impl_.success_ = value;
}
inline void Motor_Control_Rep::set_success(bool value) {
  _internal_set_success(value);
  // @@protoc_insertion_point(field_set:messages.motor.Motor_Control_Rep.success)
}

#ifdef __GNUC__
  #pragma GCC diagnostic pop
#endif  // __GNUC__
// -------------------------------------------------------------------


// @@protoc_insertion_point(namespace_scope)

}  // namespace motor
}  // namespace messages

PROTOBUF_NAMESPACE_OPEN

template <> struct is_proto_enum< ::messages::motor::MotorControlType> : ::std::true_type {};
template <>
inline const EnumDescriptor* GetEnumDescriptor< ::messages::motor::MotorControlType>() {
  return ::messages::motor::MotorControlType_descriptor();
}

PROTOBUF_NAMESPACE_CLOSE

// @@protoc_insertion_point(global_scope)

#include <google/protobuf/port_undef.inc>
#endif  // GOOGLE_PROTOBUF_INCLUDED_GOOGLE_PROTOBUF_INCLUDED_motor_5fcontrol_2eproto
