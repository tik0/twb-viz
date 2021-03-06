// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: pose.proto

#ifndef PROTOBUF_pose_2eproto__INCLUDED
#define PROTOBUF_pose_2eproto__INCLUDED

#include <string>

#include <google/protobuf/stubs/common.h>

#if GOOGLE_PROTOBUF_VERSION < 2005000
#error This file was generated by a newer version of protoc which is
#error incompatible with your Protocol Buffer headers.  Please update
#error your headers.
#endif
#if 2005000 < GOOGLE_PROTOBUF_MIN_PROTOC_VERSION
#error This file was generated by an older version of protoc which is
#error incompatible with your Protocol Buffer headers.  Please
#error regenerate this file with a newer version of protoc.
#endif

#include <google/protobuf/generated_message_util.h>
#include <google/protobuf/message.h>
#include <google/protobuf/repeated_field.h>
#include <google/protobuf/extension_set.h>
#include <google/protobuf/unknown_field_set.h>
#include "vertex.pb.h"
#include "rotation.pb.h"
// @@protoc_insertion_point(includes)

namespace twbTracking {
namespace proto {

// Internal implementation detail -- do not call these.
void  protobuf_AddDesc_pose_2eproto();
void protobuf_AssignDesc_pose_2eproto();
void protobuf_ShutdownFile_pose_2eproto();

class Pose;

// ===================================================================

class Pose : public ::google::protobuf::Message {
 public:
  Pose();
  virtual ~Pose();

  Pose(const Pose& from);

  inline Pose& operator=(const Pose& from) {
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
  static const Pose& default_instance();

  void Swap(Pose* other);

  // implements Message ----------------------------------------------

  Pose* New() const;
  void CopyFrom(const ::google::protobuf::Message& from);
  void MergeFrom(const ::google::protobuf::Message& from);
  void CopyFrom(const Pose& from);
  void MergeFrom(const Pose& from);
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

  // required .twbTracking.proto.Translation translation = 1;
  inline bool has_translation() const;
  inline void clear_translation();
  static const int kTranslationFieldNumber = 1;
  inline const ::twbTracking::proto::Translation& translation() const;
  inline ::twbTracking::proto::Translation* mutable_translation();
  inline ::twbTracking::proto::Translation* release_translation();
  inline void set_allocated_translation(::twbTracking::proto::Translation* translation);

  // required .twbTracking.proto.Rotation rotation = 2;
  inline bool has_rotation() const;
  inline void clear_rotation();
  static const int kRotationFieldNumber = 2;
  inline const ::twbTracking::proto::Rotation& rotation() const;
  inline ::twbTracking::proto::Rotation* mutable_rotation();
  inline ::twbTracking::proto::Rotation* release_rotation();
  inline void set_allocated_rotation(::twbTracking::proto::Rotation* rotation);

  // @@protoc_insertion_point(class_scope:twbTracking.proto.Pose)
 private:
  inline void set_has_translation();
  inline void clear_has_translation();
  inline void set_has_rotation();
  inline void clear_has_rotation();

  ::google::protobuf::UnknownFieldSet _unknown_fields_;

  ::twbTracking::proto::Translation* translation_;
  ::twbTracking::proto::Rotation* rotation_;

  mutable int _cached_size_;
  ::google::protobuf::uint32 _has_bits_[(2 + 31) / 32];

  friend void  protobuf_AddDesc_pose_2eproto();
  friend void protobuf_AssignDesc_pose_2eproto();
  friend void protobuf_ShutdownFile_pose_2eproto();

  void InitAsDefaultInstance();
  static Pose* default_instance_;
};
// ===================================================================


// ===================================================================

// Pose

// required .twbTracking.proto.Translation translation = 1;
inline bool Pose::has_translation() const {
  return (_has_bits_[0] & 0x00000001u) != 0;
}
inline void Pose::set_has_translation() {
  _has_bits_[0] |= 0x00000001u;
}
inline void Pose::clear_has_translation() {
  _has_bits_[0] &= ~0x00000001u;
}
inline void Pose::clear_translation() {
  if (translation_ != NULL) translation_->::twbTracking::proto::Translation::Clear();
  clear_has_translation();
}
inline const ::twbTracking::proto::Translation& Pose::translation() const {
  return translation_ != NULL ? *translation_ : *default_instance_->translation_;
}
inline ::twbTracking::proto::Translation* Pose::mutable_translation() {
  set_has_translation();
  if (translation_ == NULL) translation_ = new ::twbTracking::proto::Translation;
  return translation_;
}
inline ::twbTracking::proto::Translation* Pose::release_translation() {
  clear_has_translation();
  ::twbTracking::proto::Translation* temp = translation_;
  translation_ = NULL;
  return temp;
}
inline void Pose::set_allocated_translation(::twbTracking::proto::Translation* translation) {
  delete translation_;
  translation_ = translation;
  if (translation) {
    set_has_translation();
  } else {
    clear_has_translation();
  }
}

// required .twbTracking.proto.Rotation rotation = 2;
inline bool Pose::has_rotation() const {
  return (_has_bits_[0] & 0x00000002u) != 0;
}
inline void Pose::set_has_rotation() {
  _has_bits_[0] |= 0x00000002u;
}
inline void Pose::clear_has_rotation() {
  _has_bits_[0] &= ~0x00000002u;
}
inline void Pose::clear_rotation() {
  if (rotation_ != NULL) rotation_->::twbTracking::proto::Rotation::Clear();
  clear_has_rotation();
}
inline const ::twbTracking::proto::Rotation& Pose::rotation() const {
  return rotation_ != NULL ? *rotation_ : *default_instance_->rotation_;
}
inline ::twbTracking::proto::Rotation* Pose::mutable_rotation() {
  set_has_rotation();
  if (rotation_ == NULL) rotation_ = new ::twbTracking::proto::Rotation;
  return rotation_;
}
inline ::twbTracking::proto::Rotation* Pose::release_rotation() {
  clear_has_rotation();
  ::twbTracking::proto::Rotation* temp = rotation_;
  rotation_ = NULL;
  return temp;
}
inline void Pose::set_allocated_rotation(::twbTracking::proto::Rotation* rotation) {
  delete rotation_;
  rotation_ = rotation;
  if (rotation) {
    set_has_rotation();
  } else {
    clear_has_rotation();
  }
}


// @@protoc_insertion_point(namespace_scope)

}  // namespace proto
}  // namespace twbTracking

#ifndef SWIG
namespace google {
namespace protobuf {


}  // namespace google
}  // namespace protobuf
#endif  // SWIG

// @@protoc_insertion_point(global_scope)

#endif  // PROTOBUF_pose_2eproto__INCLUDED
