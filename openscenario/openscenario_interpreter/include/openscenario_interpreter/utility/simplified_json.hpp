// Copyright 2015 TIER IV, Inc. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

// This is a very simplified JSON serializer.

#ifndef OPENSCENARIO_INTERPRETER__UTILITY__SIMPLIFIED_JSON_HPP_
#define OPENSCENARIO_INTERPRETER__UTILITY__SIMPLIFIED_JSON_HPP_

#include <memory>
#include <sstream>
#include <string>

namespace openscenario_interpreter
{
inline namespace utility
{

class SimplifiedJSON
{
  enum class Type { Object, Array };

private:
  std::shared_ptr<std::ostringstream> ss;
  bool is_heading, added_trailer;
  Type type;

public:
  explicit SimplifiedJSON()
  : ss(std::make_shared<std::ostringstream>()), is_heading(true), type(Type::Object)
  {
    // Root is expected to be object
    *ss << "{";
  }

  explicit SimplifiedJSON(SimplifiedJSON & parent, Type type)
  : ss(parent.ss), is_heading(true), added_trailer(false), type(type)
  {
    if (type == Type::Object) {
      *ss << "{";
    } else {
      *ss << "[";
    }
  }

  // Remove the copy constructor
  SimplifiedJSON(const SimplifiedJSON &) = delete;

  ~SimplifiedJSON() { finish(); }

  void finish()
  {
    if (added_trailer) return;

    if (type == Type::Object) {
      *ss << "}";
    } else {
      *ss << "]";
    }

    added_trailer = true;
  }

  std::string str() const { return ss->str(); }

  [[nodiscard]] SimplifiedJSON add_object(const std::string key)
  {
    assert_type(Type::Object);
    add_key(key);
    return SimplifiedJSON(*this, Type::Object);
  }

  [[nodiscard]] SimplifiedJSON add_array(const std::string key)
  {
    if (type == Type::Object) {
      add_key(key);
    }
    return SimplifiedJSON(*this, Type::Array);
  }

  SimplifiedJSON & add(const std::string & key, const std::string value)
  {
    assert_type(Type::Object);
    add_key(key);
    *ss << "\"" << value << "\"";
    return *this;
  }

  SimplifiedJSON & add(const std::string & key, const double value)
  {
    assert_type(Type::Object);
    add_key(key);
    *ss << value;
    return *this;
  }

  SimplifiedJSON & add(const std::string & key, const int value)
  {
    assert_type(Type::Object);
    add_key(key);
    *ss << value;
    return *this;
  }

  SimplifiedJSON & add(const std::string & key, const size_t value)
  {
    assert_type(Type::Object);
    add_key(key);
    *ss << value;
    return *this;
  }

  SimplifiedJSON & add(const std::string & key, const long value)
  {
    assert_type(Type::Object);
    add_key(key);
    *ss << value;
    return *this;
  }

  void append(const std::string value)
  {
    assert_type(Type::Array);
    add_separator();
    *ss << "\"" << value << "\"";
  }

  void append(const double value)
  {
    assert_type(Type::Array);
    add_separator();
    *ss << value;
  }

  void append(const int value)
  {
    assert_type(Type::Array);
    add_separator();
    *ss << value;
  }

  SimplifiedJSON append_object()
  {
    assert_type(Type::Array);
    add_separator();
    return SimplifiedJSON(*this, Type::Object);
  }

protected:
  void add_key(const std::string & key)
  {
    add_separator();
    *ss << "\"" << key << "\":";
  }

  void add_separator()
  {
    if (is_heading) {
      is_heading = false;
    } else {
      *ss << ",";
    }
  }

  void assert_type(Type expected)
  {
    if (type != expected) {
      throw std::runtime_error("Type mismatch");
    }
  }
};

}  // namespace utility
}  // namespace openscenario_interpreter

#endif  // OPENSCENARIO_INTERPRETER__UTILITY__SIMPLIFIED_JSON_HPP_
