/* Generated by the protocol buffer compiler.  DO NOT EDIT! */
/* Generated from: subframe2file.proto */

/* Do not generate deprecated warnings for self */
#ifndef PROTOBUF_C__NO_DEPRECATED
#define PROTOBUF_C__NO_DEPRECATED
#endif

#include "subframe2file.pb-c.h"
void   dump__init
                     (Dump         *message)
{
  static const Dump init_value = DUMP__INIT;
  *message = init_value;
}
size_t dump__get_packed_size
                     (const Dump *message)
{
  assert(message->base.descriptor == &dump__descriptor);
  return protobuf_c_message_get_packed_size ((const ProtobufCMessage*)(message));
}
size_t dump__pack
                     (const Dump *message,
                      uint8_t       *out)
{
  assert(message->base.descriptor == &dump__descriptor);
  return protobuf_c_message_pack ((const ProtobufCMessage*)message, out);
}
size_t dump__pack_to_buffer
                     (const Dump *message,
                      ProtobufCBuffer *buffer)
{
  assert(message->base.descriptor == &dump__descriptor);
  return protobuf_c_message_pack_to_buffer ((const ProtobufCMessage*)message, buffer);
}
Dump *
       dump__unpack
                     (ProtobufCAllocator  *allocator,
                      size_t               len,
                      const uint8_t       *data)
{
  return (Dump *)
     protobuf_c_message_unpack (&dump__descriptor,
                                allocator, len, data);
}
void   dump__free_unpacked
                     (Dump *message,
                      ProtobufCAllocator *allocator)
{
  if(!message)
    return;
  assert(message->base.descriptor == &dump__descriptor);
  protobuf_c_message_free_unpacked ((ProtobufCMessage*)message, allocator);
}
void   frameparms__init
                     (Frameparms         *message)
{
  static const Frameparms init_value = FRAMEPARMS__INIT;
  *message = init_value;
}
size_t frameparms__get_packed_size
                     (const Frameparms *message)
{
  assert(message->base.descriptor == &frameparms__descriptor);
  return protobuf_c_message_get_packed_size ((const ProtobufCMessage*)(message));
}
size_t frameparms__pack
                     (const Frameparms *message,
                      uint8_t       *out)
{
  assert(message->base.descriptor == &frameparms__descriptor);
  return protobuf_c_message_pack ((const ProtobufCMessage*)message, out);
}
size_t frameparms__pack_to_buffer
                     (const Frameparms *message,
                      ProtobufCBuffer *buffer)
{
  assert(message->base.descriptor == &frameparms__descriptor);
  return protobuf_c_message_pack_to_buffer ((const ProtobufCMessage*)message, buffer);
}
Frameparms *
       frameparms__unpack
                     (ProtobufCAllocator  *allocator,
                      size_t               len,
                      const uint8_t       *data)
{
  return (Frameparms *)
     protobuf_c_message_unpack (&frameparms__descriptor,
                                allocator, len, data);
}
void   frameparms__free_unpacked
                     (Frameparms *message,
                      ProtobufCAllocator *allocator)
{
  if(!message)
    return;
  assert(message->base.descriptor == &frameparms__descriptor);
  protobuf_c_message_free_unpacked ((ProtobufCMessage*)message, allocator);
}
static const ProtobufCFieldDescriptor dump__field_descriptors[1] =
{
  {
    "frame_param",
    1,
    PROTOBUF_C_LABEL_REQUIRED,
    PROTOBUF_C_TYPE_MESSAGE,
    0,   /* quantifier_offset */
    offsetof(Dump, frame_param),
    &frameparms__descriptor,
    NULL,
    0,             /* flags */
    0,NULL,NULL    /* reserved1,reserved2, etc */
  },
};
static const unsigned dump__field_indices_by_name[] = {
  0,   /* field[0] = frame_param */
};
static const ProtobufCIntRange dump__number_ranges[1 + 1] =
{
  { 1, 0 },
  { 0, 1 }
};
const ProtobufCMessageDescriptor dump__descriptor =
{
  PROTOBUF_C__MESSAGE_DESCRIPTOR_MAGIC,
  "dump",
  "Dump",
  "Dump",
  "",
  sizeof(Dump),
  1,
  dump__field_descriptors,
  dump__field_indices_by_name,
  1,  dump__number_ranges,
  (ProtobufCMessageInit) dump__init,
  NULL,NULL,NULL    /* reserved[123] */
};
static const ProtobufCFieldDescriptor frameparms__field_descriptors[2] =
{
  {
    "N_RB_DL",
    1,
    PROTOBUF_C_LABEL_REQUIRED,
    PROTOBUF_C_TYPE_UINT32,
    0,   /* quantifier_offset */
    offsetof(Frameparms, n_rb_dl),
    NULL,
    NULL,
    0,             /* flags */
    0,NULL,NULL    /* reserved1,reserved2, etc */
  },
  {
    "N_RB_UL",
    2,
    PROTOBUF_C_LABEL_REQUIRED,
    PROTOBUF_C_TYPE_UINT32,
    0,   /* quantifier_offset */
    offsetof(Frameparms, n_rb_ul),
    NULL,
    NULL,
    0,             /* flags */
    0,NULL,NULL    /* reserved1,reserved2, etc */
  },
};
static const unsigned frameparms__field_indices_by_name[] = {
  0,   /* field[0] = N_RB_DL */
  1,   /* field[1] = N_RB_UL */
};
static const ProtobufCIntRange frameparms__number_ranges[1 + 1] =
{
  { 1, 0 },
  { 0, 2 }
};
const ProtobufCMessageDescriptor frameparms__descriptor =
{
  PROTOBUF_C__MESSAGE_DESCRIPTOR_MAGIC,
  "Frameparms",
  "Frameparms",
  "Frameparms",
  "",
  sizeof(Frameparms),
  2,
  frameparms__field_descriptors,
  frameparms__field_indices_by_name,
  1,  frameparms__number_ranges,
  (ProtobufCMessageInit) frameparms__init,
  NULL,NULL,NULL    /* reserved[123] */
};