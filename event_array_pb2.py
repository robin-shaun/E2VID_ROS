# -*- coding: utf-8 -*-
# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: event_array.proto
"""Generated protocol buffer code."""
from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from google.protobuf import reflection as _reflection
from google.protobuf import symbol_database as _symbol_database
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()




DESCRIPTOR = _descriptor.FileDescriptor(
  name='event_array.proto',
  package='event_array.protobuf',
  syntax='proto3',
  serialized_options=None,
  create_key=_descriptor._internal_create_key,
  serialized_pb=b'\n\x11\x65vent_array.proto\x12\x14\x65vent_array.protobuf\"H\n\x0b\x45vent_Array\x12\x11\n\ttimestamp\x18\x01 \x03(\x01\x12\t\n\x01x\x18\x02 \x03(\x05\x12\t\n\x01y\x18\x03 \x03(\x05\x12\x10\n\x08polarity\x18\x04 \x03(\x05\x62\x06proto3'
)




_EVENT_ARRAY = _descriptor.Descriptor(
  name='Event_Array',
  full_name='event_array.protobuf.Event_Array',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  create_key=_descriptor._internal_create_key,
  fields=[
    _descriptor.FieldDescriptor(
      name='timestamp', full_name='event_array.protobuf.Event_Array.timestamp', index=0,
      number=1, type=1, cpp_type=5, label=3,
      has_default_value=False, default_value=[],
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='x', full_name='event_array.protobuf.Event_Array.x', index=1,
      number=2, type=5, cpp_type=1, label=3,
      has_default_value=False, default_value=[],
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='y', full_name='event_array.protobuf.Event_Array.y', index=2,
      number=3, type=5, cpp_type=1, label=3,
      has_default_value=False, default_value=[],
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='polarity', full_name='event_array.protobuf.Event_Array.polarity', index=3,
      number=4, type=5, cpp_type=1, label=3,
      has_default_value=False, default_value=[],
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
  ],
  extensions=[
  ],
  nested_types=[],
  enum_types=[
  ],
  serialized_options=None,
  is_extendable=False,
  syntax='proto3',
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=43,
  serialized_end=115,
)

DESCRIPTOR.message_types_by_name['Event_Array'] = _EVENT_ARRAY
_sym_db.RegisterFileDescriptor(DESCRIPTOR)

Event_Array = _reflection.GeneratedProtocolMessageType('Event_Array', (_message.Message,), {
  'DESCRIPTOR' : _EVENT_ARRAY,
  '__module__' : 'event_array_pb2'
  # @@protoc_insertion_point(class_scope:event_array.protobuf.Event_Array)
  })
_sym_db.RegisterMessage(Event_Array)


# @@protoc_insertion_point(module_scope)