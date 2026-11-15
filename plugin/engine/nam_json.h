// Minimal JSON reader for .nam files (load time only; never used on the audio thread).
// Dependency-free so the engine builds anywhere a C++17 compiler does.
#pragma once
#include <cstdlib>
#include <map>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

namespace namjson {

struct Value {
    enum Type { Null, Bool, Number, String, Array, Object } type = Null;
    bool b = false;
    double num = 0.0;
    std::string str;
    std::vector<Value> arr;
    std::map<std::string, Value> obj;

    bool isNull() const { return type == Null; }
    bool has(const std::string& k) const { return type == Object && obj.count(k) > 0; }
    const Value& operator[](const std::string& k) const {
        static const Value null;
        if (type != Object) return null;
        auto it = obj.find(k);
        return it == obj.end() ? null : it->second;
    }
    const Value& operator[](size_t i) const {
        if (type != Array || i >= arr.size()) throw std::runtime_error("JSON index out of range");
        return arr[i];
    }
    size_t size() const { return type == Array ? arr.size() : (type == Object ? obj.size() : 0); }
    double asNum(double def = 0.0) const { return type == Number ? num : (type == Bool ? (b ? 1.0 : 0.0) : def); }
    int asInt(int def = 0) const { return type == Number ? (int)num : def; }
    bool asBool(bool def = false) const { return type == Bool ? b : (type == Number ? num != 0.0 : def); }
    std::string asStr(const std::string& def = "") const { return type == String ? str : def; }
};

class Parser {
  public:
    explicit Parser(const std::string& s) : s_(s) {}
    Value parse() {
        Value v = value();
        ws();
        if (i_ != s_.size()) fail("trailing characters");
        return v;
    }

  private:
    const std::string& s_;
    size_t i_ = 0;

    [[noreturn]] void fail(const char* what) {
        throw std::runtime_error(std::string("JSON parse error: ") + what + " at offset " + std::to_string(i_));
    }
    void ws() {
        while (i_ < s_.size() && (s_[i_] == ' ' || s_[i_] == '\n' || s_[i_] == '\r' || s_[i_] == '\t')) i_++;
    }
    bool lit(const char* w) {
        size_t n = std::char_traits<char>::length(w);
        if (s_.compare(i_, n, w) == 0) { i_ += n; return true; }
        return false;
    }
    Value value() {
        ws();
        if (i_ >= s_.size()) fail("unexpected end");
        Value v;
        char c = s_[i_];
        if (c == '{') {
            v.type = Value::Object; i_++; ws();
            if (s_[i_] == '}') { i_++; return v; }
            for (;;) {
                ws();
                if (s_[i_] != '"') fail("expected key");
                std::string k = string();
                ws();
                if (s_[i_++] != ':') fail("expected ':'");
                v.obj[k] = value();
                ws();
                if (s_[i_] == ',') { i_++; continue; }
                if (s_[i_] == '}') { i_++; return v; }
                fail("expected ',' or '}'");
            }
        }
        if (c == '[') {
            v.type = Value::Array; i_++; ws();
            if (s_[i_] == ']') { i_++; return v; }
            for (;;) {
                v.arr.push_back(value());
                ws();
                if (s_[i_] == ',') { i_++; continue; }
                if (s_[i_] == ']') { i_++; return v; }
                fail("expected ',' or ']'");
            }
        }
        if (c == '"') { v.type = Value::String; v.str = string(); return v; }
        if (lit("true")) { v.type = Value::Bool; v.b = true; return v; }
        if (lit("false")) { v.type = Value::Bool; v.b = false; return v; }
        if (lit("null")) return v;
        if (lit("NaN")) { v.type = Value::Number; v.num = std::strtod("nan", nullptr); return v; }
        const char* start = s_.c_str() + i_;
        char* end = nullptr;
        v.num = std::strtod(start, &end);
        if (end == start) fail("bad value");
        i_ += (size_t)(end - start);
        v.type = Value::Number;
        return v;
    }
    std::string string() {
        std::string out;
        i_++; // opening quote
        while (i_ < s_.size() && s_[i_] != '"') {
            char c = s_[i_++];
            if (c != '\\') { out += c; continue; }
            char e = s_[i_++];
            switch (e) {
                case 'n': out += '\n'; break;
                case 't': out += '\t'; break;
                case 'r': out += '\r'; break;
                case 'b': out += '\b'; break;
                case 'f': out += '\f'; break;
                case 'u': {
                    unsigned cp = (unsigned)std::strtoul(s_.substr(i_, 4).c_str(), nullptr, 16);
                    i_ += 4;
                    if (cp < 0x80) out += (char)cp;
                    else if (cp < 0x800) { out += (char)(0xC0 | (cp >> 6)); out += (char)(0x80 | (cp & 0x3F)); }
                    else { out += (char)(0xE0 | (cp >> 12)); out += (char)(0x80 | ((cp >> 6) & 0x3F)); out += (char)(0x80 | (cp & 0x3F)); }
                    break;
                }
                default: out += e;
            }
        }
        if (i_ >= s_.size()) fail("unterminated string");
        i_++;
        return out;
    }
};

inline Value parse(const std::string& s) { return Parser(s).parse(); }

} // namespace namjson
