#!/usr/bin/env python

#####################################################################
# Software License Agreement (BSD License)
#
# Copyright (c) 2011, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#  notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#  copyright notice, this list of conditions and the following
#  disclaimer in the documentation and/or other materials provided
#  with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#  contributors may be used to endorse or promote products derived
#  from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

__author__ = "mferguson@willowgarage.com (Michael Ferguson)"

__usage__ = """
make_library.py generates the Arduino rosserial library files.  It
requires the location of your arduino libraries folder and the name of
one or more packages for which you want to make libraries.

rosrun rosserial_client make_library.py <library_path>  pkg_name [pkg2 pkg3 ...]
"""

import os
import re
import sys

import roslib; roslib.load_manifest('rosserial_client')

_TYPES = {
    'bool': ('bool', 1),
    'byte': ('unsigned char', 1),
    'char': ('char', 1),
    'int8': ('int8_t', 1),
    'uint8': ('uint8_t', 1),
    'int16': ('int16_t', 2),
    'uint16': ('uint16_t', 2),
    'int32': ('int32_t', 4),
    'uint32': ('uint32_t', 4),
    'float32': ('float', 4),
    }


def number_of_bytes_to_type(number_of_bytes):
  lookup = {
      1: 'uint8_t',
      2: 'uint16_t',
      4: 'uint32_t',
      }
  return lookup[number_of_bytes]


class EnumerationType(object):
  """For data values."""

  def __init__(self, name, ty, value):
    self.name = name
    self.type = ty
    self.value = value

  def make_declaration(self, stream):
    stream.write('  enum { %s = %s };\n' % (self.name, str(self.value)))


class PrimitiveDataType(object):
  """Our datatype is a C/C++ primitive."""

  def __init__(self, name, ty, number_of_bytes):
    self.name = name
    self.type = ty
    self.number_of_bytes = number_of_bytes

  def make_declaration(self, stream):
    stream.write('  %s %s;\n' % (self.type, self.name))

  def serialize(self, stream):
    stream.write('    if (offset + sizeof(%s) > limit) {\n' % self.type)
    stream.write('      return -1;\n')
    stream.write('    }\n')
    stream.write('    {\n');
    bits_type = number_of_bytes_to_type(self.number_of_bytes)
    stream.write('      %s bits = *reinterpret_cast<%s*>(&this->%s);\n' % (bits_type, bits_type, self.name))
    for i in xrange(self.number_of_bytes):
      stream.write('      buffer[offset++] = bits >> (8 * %d);\n' % i)
    stream.write('    }\n');

  def deserialize(self, stream):
    stream.write('    if (offset + sizeof(%s) > limit) {\n' % self.type)
    stream.write('      return -1;\n')
    stream.write('    }\n')
    stream.write('    {\n');
    stream.write('      %s bits = 0;\n' % number_of_bytes_to_type(self.number_of_bytes))
    for i in xrange(self.number_of_bytes):
      stream.write('      bits |= %s(buffer[offset++]) << (8 * %d);\n' % (number_of_bytes_to_type(self.number_of_bytes), i))
    stream.write('      this->%s = *reinterpret_cast<%s*>(&bits);\n' % (self.name, self.type))
    stream.write('    }\n');


class MessageDataType(PrimitiveDataType):
  """For when our data type is another message."""

  def serialize(self, stream):
    stream.write('    {\n')
    stream.write('      int result = %s.serialize(buffer + offset, limit - offset);\n' % self.name)
    stream.write('      if (result < 0) {\n')
    stream.write('        return -1;\n')
    stream.write('      }\n')
    stream.write('      offset += result;\n')
    stream.write('    }\n')

  def deserialize(self, stream):
    stream.write('    {\n')
    stream.write('      int result = %s.deserialize(buffer + offset, limit - offset);\n' % self.name)
    stream.write('      if (result < 0) {\n')
    stream.write('        return -1;\n')
    stream.write('      }\n')
    stream.write('      offset += result;\n')
    stream.write('    }\n')


class Float64DataType(PrimitiveDataType):
  """AVR C/C++ has no native 64-bit support, we automatically convert to 32-bit float."""

  def make_declaration(self, stream):
    stream.write('  float %s;\n' % self.name )

  def serialize(self, stream):
    stream.write('    if (offset + 8 > limit) {\n')
    stream.write('      return -1;\n')
    stream.write('    }\n')
    stream.write('    {\n')
    stream.write('      uint32_t bits = *reinterpret_cast<uint32_t*>(&this->%s);\n' % self.name)
    stream.write('      uint32_t exponent = (bits >> 23) & 255;\n')
    stream.write('      if (exponent != 0) {\n')
    stream.write('        exponent += 1023 - 127;\n')
    stream.write('      }\n')
    stream.write('      buffer[offset++] = 0;\n')  # 29 blank bits
    stream.write('      buffer[offset++] = 0;\n')
    stream.write('      buffer[offset++] = 0;\n')
    stream.write('      buffer[offset++] = (bits << 5) & 0xff;\n')
    stream.write('      buffer[offset++] = (bits >> 3) & 0xff;\n')
    stream.write('      buffer[offset++] = (bits >> 11) & 0xff;\n')
    stream.write('      buffer[offset++] = ((exponent << 4) & 0xf0) | ((bits >> 19) & 0x0f);\n')
    stream.write('      buffer[offset++] = (exponent >> 4) & 0x7f;\n')
    stream.write('      if (this->%s < 0) {\n' % self.name)
    stream.write('        buffer[offset - 1] |= 0x80;\n')
    stream.write('      }\n')
    stream.write('    }\n')

  def deserialize(self, stream):
    # TODO(whess): Check for under and overflow.
    stream.write('    if (offset + 8 > limit) {\n')
    stream.write('      return -1;\n')
    stream.write('    }\n')
    stream.write('    {\n')
    stream.write('      offset += 3;\n')  # Skip the least significant part of the mantissa.
    stream.write('      uint32_t bits = (static_cast<uint32_t>(buffer[offset++]) >> 5) & 0x07;\n')
    stream.write('      bits |= (static_cast<uint32_t>(buffer[offset++]) & 0xff) << 3;\n')
    stream.write('      bits |= (static_cast<uint32_t>(buffer[offset++]) & 0xff) << 11;\n')
    stream.write('      bits |= (static_cast<uint32_t>(buffer[offset]) & 0x0f) << 19;\n')
    stream.write('      uint32_t exponent = (static_cast<uint32_t>(buffer[offset++]) & 0xf0) >> 4;\n')
    stream.write('      exponent |= (static_cast<uint32_t>(buffer[offset]) & 0x7f) << 4;\n')
    stream.write('      if (exponent != 0) {\n')
    stream.write('        bits |= (exponent - 1023 + 127) << 23;\n')
    stream.write('      }\n')
    stream.write('      if ((buffer[offset++] & 0x80) > 0) {\n')
    stream.write('        bits |= 1ul << 31;')
    stream.write('      }\n')
    stream.write('      this->%s = *reinterpret_cast<%s*>(&bits);\n' % (self.name, self.type))
    stream.write('    }\n')


class Int64DataType(PrimitiveDataType):
  """AVR C/C++ has no native 64-bit support."""

  def make_declaration(self, stream):
    stream.write('  int32_t %s;\n' % self.name )

  def serialize(self, stream):
    stream.write('    if (offset + sizeof(float) > limit) {\n')
    stream.write('      return -1;\n')
    stream.write('    }\n')
    for i in xrange(4):
      stream.write('    buffer[offset++] = (this->%s >> (8 * %d)) & 0xff;\n' % (self.name, i))
    for i in xrange(4):
      stream.write('    buffer[offset++] = (this->%s > 0) ? 0: 255;\n' % self.name)

  def deserialize(self, stream):
    stream.write('    this->%s = 0;\n' % self.name)
    for i in xrange(4):
      stream.write('    this->%s += ((int32_t) buffer[offset++]) >> (8 * %d);\n' % (self.name, i))
    stream.write('    offset += 4;\n')


class StringDataType(PrimitiveDataType):
  """Need to convert to signed char *."""

  def make_declaration(self, stream):
    stream.write('  char* %s;\n' % self.name)

  def serialize(self, stream):
    # TODO(damonkohler): Pull out a variable for the length of the string? Have to make sure names don't clash.
    # Add an additional 4 bytes for the length field we just parsed.
    stream.write('    if (offset + 4 + strlen((const char*) this->%s) > limit) {\n' % self.name)
    stream.write('      return -1;\n')
    stream.write('    }\n')
    stream.write('    {\n')
    stream.write('      uint32_t* length = reinterpret_cast<uint32_t*>(&buffer[offset]);\n')
    stream.write('      *length = strlen((const char*) this->%s);\n' % self.name)
    stream.write('      offset += 4;\n')
    stream.write('      memcpy(buffer + offset, this->%s, *length);\n' % self.name)
    stream.write('      offset += *length;\n')
    stream.write('    }\n')

  def deserialize(self, stream):
    stream.write('    {\n')
    stream.write('      uint32_t length = *((uint32_t*) &buffer[offset]);\n')
    # Add an additional 4 bytes for the length field we just parsed.
    stream.write('      if (offset + length + 4 > limit) {\n')
    stream.write('        return -1;\n')
    stream.write('      }\n')
    stream.write('      offset += 4;\n')
    # Shift to the left to make room for the null character.
    stream.write('      for (int k = offset; k < offset + length; k++) {\n')
    stream.write('        buffer[k-1] = buffer[k];\n')
    stream.write('      }\n')
    stream.write('      buffer[offset + length - 1] = 0;\n')
    stream.write('      this->%s = (char*) buffer[offset - 1];\n' % self.name)
    stream.write('      offset += length;\n')
    stream.write('    }\n')


class TimeDataType(PrimitiveDataType):

  def __init__(self, name, ty, number_of_bytes):
    PrimitiveDataType.__init__(self, name, ty, number_of_bytes)
    self.sec = PrimitiveDataType(name + '.sec', 'uint32_t', 4)
    self.nsec = PrimitiveDataType(name + '.nsec', 'uint32_t', 4)

  def make_declaration(self, stream):
    stream.write('  %s %s;\n' % (self.type, self.name))

  def serialize(self, stream):
    self.sec.serialize(stream)
    self.nsec.serialize(stream)

  def deserialize(self, stream):
    self.sec.deserialize(stream)
    self.nsec.deserialize(stream)


class DurationDataType(PrimitiveDataType):

  def __init__(self, name, ty, number_of_bytes):
    PrimitiveDataType.__init__(self, name, ty, number_of_bytes)
    self.sec = PrimitiveDataType(name + '.sec', 'int32_t', 4)
    self.nsec = PrimitiveDataType(name + '.nsec', 'int32_t', 4)

  def make_declaration(self, stream):
    stream.write('  %s %s;\n' % (self.type, self.name))

  def serialize(self, stream):
    self.sec.serialize(stream)
    self.nsec.serialize(stream)

  def deserialize(self, stream):
    self.sec.deserialize(stream)
    self.nsec.deserialize(stream)


class ArrayDataType(PrimitiveDataType):

  def __init__(self, name, ty, number_of_bytes, cls, array_size=None):
    PrimitiveDataType.__init__(self, name, ty, number_of_bytes)
    self.size = array_size
    self.cls = cls

  def make_declaration(self, stream):
    if self.size == None:
      stream.write('  unsigned char %s_length;\n' % self.name)
      stream.write('  %s st_%s;\n' % (self.type, self.name))  # Static instance for copy.
      stream.write('  %s* %s;\n' % (self.type, self.name))
    else:
      stream.write('  %s %s[%d];\n' % (self.type, self.name, self.size))

  def serialize(self, stream):
    c = self.cls(self.name + '[i]', self.type, self.number_of_bytes)
    if self.size == None:
      # Serialize length.
      stream.write('    *(buffer + offset++) = %s_length;\n' % self.name)
      stream.write('    *(buffer + offset++) = 0;\n')
      stream.write('    *(buffer + offset++) = 0;\n')
      stream.write('    *(buffer + offset++) = 0;\n')
      stream.write('    for (unsigned char i = 0; i < %s_length; i++) {\n' % self.name)
      c.serialize(stream)
      stream.write('    }\n')
    else:
      stream.write('    unsigned char * %s_val = (unsigned char *) this->%s;\n' % (self.name, self.name))
      stream.write('    for (unsigned char i = 0; i < %d; i++) {\n' % self.size)
      c.serialize(stream)
      stream.write('    }\n')

  def deserialize(self, stream):
    if self.size == None:
      c = self.cls('st_' + self.name, self.type, self.number_of_bytes)
      # Deserialize length.
      stream.write('    {\n')
      stream.write('      unsigned char length = buffer[offset++];\n')
      stream.write('      if (length > %s_length) {\n' % self.name)
      stream.write('        this->%s = (%s*) realloc(this->%s, length * sizeof(%s));\n' % (self.name, self.type, self.name, self.type))
      stream.write('      }\n')
      stream.write('      offset += 3;\n')
      stream.write('      %s_length = length;\n' % self.name)
      stream.write('    }\n')
      # Copy to array.
      stream.write('    for (unsigned char i = 0; i < %s_length; i++) {\n' % (self.name) )
      c.deserialize(stream)
      stream.write('      memcpy(&(this->%s[i]), &(this->st_%s), sizeof(%s));\n' % (self.name, self.name, self.type))
      stream.write('    }\n')
    else:
      c = self.cls(self.name + '[i]', self.type, self.number_of_bytes)
      stream.write('    unsigned char* %s_val = (unsigned char*) this->%s;\n' % (self.name, self.name))
      stream.write('    for (unsigned char i = 0; i < %d; i++) {\n' % self.size)
      c.deserialize(stream)
      stream.write('    }\n')


class Message(object):
  """Parses message definitions into something we can export. """

  def __init__(self, name, package, definition):
    self.name = name      # name of message/class
    self.package = package    # package we reside in
    self.includes = list()    # other files we must include

    self.data = list()      # data types for code generation
    self.enums = list()

    # parse definition
    for line in definition:
      # prep work
      line = line.strip().rstrip()
      value = None
      if line.find("#") > -1:
        line = line[0:line.find("#")]
      if line.find("=") > -1:
        value = int(line[line.find("=")+1:])
        line = line[0:line.find("=")]

      # find package/class name
      line = line.replace("\t", " ")
      l = line.split(" ")
      while "" in l:
        l.remove("")
      if len(l) < 2:
        continue
      ty, name = l[0:2]
      if value != None:
        self.enums.append( EnumerationType(name, ty, value))
        continue

      try:
        type_package, type_name = ty.split("/")
      except:
        type_package = None
        type_name = ty
      type_array = False
      if type_name.find('[') > 0:
        type_array = True
        try:
          type_array_size = int(type_name[type_name.find('[')+1:type_name.find(']')])
        except:
          type_array_size = None
        type_name = type_name[0:type_name.find('[')]

      # convert to C/Arduino type if primitive, expand name otherwise
      try:
        # primitive type
        cls = PrimitiveDataType
        code_type = type_name
        size = 0
        if type_package:
          cls = MessageDataType
        if type_name == 'float64':
          cls = Float64DataType
          code_type = 'float'
        elif type_name == 'time':
          cls = TimeDataType
          code_type = 'ros::Time'
          if "ros/time" not in self.includes:
            self.includes.append("ros/time")
        elif type_name == 'duration':
          cls = DurationDataType
          code_type = 'ros::Duration'
          if "ros/duration" not in self.includes:
            self.includes.append("ros/duration")
        elif type_name == 'string':
          cls = StringDataType
          code_type = 'char*'
        elif type_name == 'uint64' or type_name == 'int64':
          cls = Int64DataType
          code_type = 'long'
        else:
          code_type = _TYPES[type_name][0]
          size = _TYPES[type_name][1]
        if type_array:
          self.data.append( ArrayDataType(name, code_type, size, cls, type_array_size ) )
        else:
          self.data.append(cls(name, code_type, size))
      except:
        if type_name == 'Header':
          self.data.append( MessageDataType(name, 'std_msgs::Header', 0) )
          if "std_msgs/Header" not in self.includes:
            self.includes.append("std_msgs/Header")
        else:
          if type_package == None or type_package == package:
            type_package = package
            cls = MessageDataType
            if self.package+"/"+type_name not in self.includes:
              self.includes.append(self.package+"/"+type_name)
          if type_package+"/"+type_name not in self.includes:
            self.includes.append(type_package+"/"+type_name)
          if type_array:
            self.data.append( ArrayDataType(name, type_package + "::" + type_name, size, cls, type_array_size) )
          else:
            self.data.append( MessageDataType(name, type_package + "::" + type_name, 0) )

  def _write_serializer(self, stream):
    stream.write('\n')
    stream.write('  virtual int serialize(unsigned char* buffer, int limit) {\n')
    stream.write('    int offset = 0;\n')
    for d in self.data:
      d.serialize(stream)
    stream.write('    return offset;\n');
    stream.write('  }\n')
    stream.write('\n')

  def _write_deserializer(self, stream):
    stream.write('  virtual int deserialize(unsigned char* buffer, int limit) {\n')
    stream.write('    int offset = 0;\n')
    for d in self.data:
      d.deserialize(stream)
    stream.write('    return offset;\n');
    stream.write('  }\n')
    stream.write('\n')

  def _write_std_includes(self, stream):
    stream.write('#include <stdint.h>\n')
    stream.write('#include <string.h>\n')
    stream.write('#include <stdlib.h>\n')
    stream.write('\n')
    stream.write('#include "ros/msg.h"\n')

  def _write_msg_includes(self, stream):
    for include in self.includes:
      stream.write('#include "%s.h"\n' % include)

  def _write_data(self, stream):
    for d in self.data:
      d.make_declaration(stream)
    for e in self.enums:
      e.make_declaration(stream)

  def _write_getType(self, stream):
    stream.write('  const char* getType() { return "%s/%s"; };\n' % (self.package, self.name))

  def _write_impl(self, stream):
    stream.write('class %s : public ros::Msg {\n' % self.name)
    stream.write(' public:\n')
    self._write_data(stream)
    self._write_serializer(stream)
    self._write_deserializer(stream)
    self._write_getType(stream)
    stream.write('\n')
    stream.write('};\n')

  def make_header(self, stream):
    guard = 'ROS_%s_%s_H_' % (self.package.upper(), self.name.upper())
    stream.write('#ifndef %s\n' % guard)
    stream.write('#define %s\n' % guard)
    stream.write('\n')
    self._write_std_includes(stream)
    self._write_msg_includes(stream)
    stream.write('\n')
    stream.write('namespace %s {\n' % self.package)
    stream.write('\n')
    self._write_impl(stream)
    stream.write('\n')
    stream.write('}  // namespace %s\n' % self.package)
    stream.write('\n')
    stream.write('#endif  // %s\n' % guard)


class Service(object):

  def __init__(self, name, package, definition):
    """
    @param name -  name of service
    @param package - name of service package
    @param definition - list of lines of  definition
    """

    self.name = name
    self.package = package

    sep_line = None
    sep = re.compile('---*')
    for i in xrange(len(definition)):
      if re.match(sep, definition[i]):
        sep_line = i
        break
    self.req_def = definition[0:sep_line]
    self.resp_def = definition[sep_line+1:]

    self.req = Message(name + "Request", package, self.req_def)
    self.resp = Message(name + "Response", package, self.resp_def)

  def make_header(self, stream):
    guard = 'ROS_SERVICE_%s_H_' % self.name.upper()
    stream.write('#ifndef %s\n' % guard)
    stream.write('#define %s\n' % guard)

    self.req._write_std_includes(stream)
    includes = self.req.includes
    includes.extend(self.resp.includes)
    includes = list(set(includes))
    for inc in includes:
      stream.write('#include "%s.h"\n' % inc)

    stream.write('\n')
    stream.write('namespace %s {\n' % self.package)
    stream.write('\n')
    stream.write('static const char %s[] = "%s/%s";\n' % (self.name.upper(), self.package, self.name))

    def write_type(out, name):
      out.write('  const char* getType() { return %s; };\n' % name)

    _write_getType = lambda out: write_type(out, self.name.upper())
    self.req._write_getType = _write_getType
    self.resp._write_getType = _write_getType

    stream.write('\n')
    self.req._write_impl(stream)
    stream.write('\n')
    self.resp._write_impl(stream)
    stream.write('\n')
    stream.write('}  // namespace %s\n' % self.package)
    stream.write('\n')
    stream.write('#endif  // %s\n' % guard)


class ArduinoLibraryMaker(object):
  """Create an Arduino Library from a set of Message Definitions. """

  def __init__(self, package):
    """Initialize by finding location and all messages in this package. """
    self.name = package
    print "\nExporting " + package +"\n",

    self.pkg_dir = roslib.packages.get_pkg_dir(package)

    sys.stdout.write('Messages:\n  ')
    # find the messages in this package
    self.messages = list()
    if (os.path.exists(self.pkg_dir+"/msg")):
      for path in os.listdir(self.pkg_dir+"/msg"):
        if path.endswith(".msg"):
          # Add to list of messages.
          print "%s," % path[0:-4],
          definition = open(self.pkg_dir + "/msg/" + path).readlines()
          self.messages.append(Message(path[0:-4], self.name, definition))
      print "\n"

    sys.stdout.write('Services:\n  ')
    # find the services in this package
    self.services = list()
    if (os.path.exists(self.pkg_dir+"/srv/")):
      for path in os.listdir(self.pkg_dir+"/srv"):
        if path.endswith(".srv"):
          # add to list of messages
          print "%s," % path[0:-4],
          definition = open(self.pkg_dir + "/srv/" + path).readlines()
          self.messages.append( Service(path[0:-4], self.name, definition) )
      print "\n"

  def generate(self, path_to_output):
    """Generate header and source files for this package. """
    # generate for each message
    for msg in self.messages:
      if not os.path.exists(path_to_output + "/" + self.name):
        os.makedirs(path_to_output + "/" + self.name)
      header = open(path_to_output + "/" + self.name + "/" + msg.name + ".h", "w")
      msg.make_header(header)
      header.close()


if __name__== "__main__":
  if (len(sys.argv) <3):
    print __usage__
    exit()

  path = sys.argv[1]
  if path[-1] == "/":
    path = path[0:-1]
  print "\nExporting to %s" % path

  # make libraries
  packages = sys.argv[2:]
  for msg_package in packages:
    lm = ArduinoLibraryMaker(msg_package)
    lm.generate(path)

