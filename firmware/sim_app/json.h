// Minimal JSON reader for SIL scenario files (objects, arrays, numbers, strings, bools, null).
#pragma once
#include <cctype>
#include <cstdlib>
#include <map>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

namespace mjson {

struct Value {
  enum class T { Null, Bool, Num, Str, Arr, Obj } t = T::Null;
  bool b = false;
  double n = 0;
  std::string s;
  std::vector<Value> a;
  std::map<std::string, Value> o;

  bool has(const std::string& k) const { return t == T::Obj && o.count(k) > 0; }
  const Value& operator[](const std::string& k) const {
    static const Value null;
    auto it = o.find(k);
    return it == o.end() ? null : it->second;
  }
  double num(const std::string& k, double def) const { return has(k) && (*this)[k].t == T::Num ? (*this)[k].n : def; }
  bool boolean(const std::string& k, bool def) const { return has(k) && (*this)[k].t == T::Bool ? (*this)[k].b : def; }
  std::string str(const std::string& k, const std::string& def) const { return has(k) && (*this)[k].t == T::Str ? (*this)[k].s : def; }
};

class Parser {
 public:
  explicit Parser(const std::string& txt) : s_(txt) {}
  Value parse() {
    Value v = value();
    ws();
    if (i_ != s_.size()) throw std::runtime_error("trailing characters in JSON");
    return v;
  }

 private:
  const std::string& s_;
  size_t i_ = 0;
  void ws() {
    while (i_ < s_.size() && std::isspace(static_cast<unsigned char>(s_[i_]))) ++i_;
  }
  char peek() {
    ws();
    if (i_ >= s_.size()) throw std::runtime_error("unexpected end of JSON");
    return s_[i_];
  }
  void expect(char c) {
    if (peek() != c) throw std::runtime_error(std::string("expected '") + c + "' at " + std::to_string(i_));
    ++i_;
  }
  std::string string_() {
    expect('"');
    std::string out;
    while (i_ < s_.size() && s_[i_] != '"') {
      if (s_[i_] == '\\' && i_ + 1 < s_.size()) {
        ++i_;
        const char e = s_[i_];
        out += e == 'n' ? '\n' : e == 't' ? '\t' : e;
      } else {
        out += s_[i_];
      }
      ++i_;
    }
    ++i_;
    return out;
  }
  Value value() {
    const char c = peek();
    Value v;
    if (c == '{') {
      ++i_;
      v.t = Value::T::Obj;
      if (peek() == '}') {
        ++i_;
        return v;
      }
      while (true) {
        std::string k = string_();
        expect(':');
        v.o[k] = value();
        if (peek() == ',') {
          ++i_;
          continue;
        }
        expect('}');
        return v;
      }
    }
    if (c == '[') {
      ++i_;
      v.t = Value::T::Arr;
      if (peek() == ']') {
        ++i_;
        return v;
      }
      while (true) {
        v.a.push_back(value());
        if (peek() == ',') {
          ++i_;
          continue;
        }
        expect(']');
        return v;
      }
    }
    if (c == '"') {
      v.t = Value::T::Str;
      v.s = string_();
      return v;
    }
    if (s_.compare(i_, 4, "true") == 0) {
      i_ += 4;
      v.t = Value::T::Bool;
      v.b = true;
      return v;
    }
    if (s_.compare(i_, 5, "false") == 0) {
      i_ += 5;
      v.t = Value::T::Bool;
      return v;
    }
    if (s_.compare(i_, 4, "null") == 0) {
      i_ += 4;
      return v;
    }
    char* end = nullptr;
    v.n = std::strtod(s_.c_str() + i_, &end);
    if (end == s_.c_str() + i_) throw std::runtime_error("bad JSON value at " + std::to_string(i_));
    i_ = static_cast<size_t>(end - s_.c_str());
    v.t = Value::T::Num;
    return v;
  }
};

}  // namespace mjson
