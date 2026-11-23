#include "message_json.hpp"

#include <rosidl_typesupport_introspection_cpp/field_types.hpp>

#include <array>
#include <charconv>
#include <cmath>
#include <limits>
#include <string_view>

namespace mcp_ros2 {

namespace {

namespace ts = rosidl_typesupport_introspection_cpp;
using nlohmann::json;

const MessageMembers& nested(const MessageMember& member) {
    return *static_cast<const MessageMembers*>(member.members_->data);
}

/// @p text with each byte that is not part of valid UTF-8 replaced by U+FFFD, as JSON needs.
std::string valid_utf8(std::string_view text) {
    std::string out;
    out.reserve(text.size());
    for (std::size_t i = 0; i < text.size();) {
        const auto byte = static_cast<unsigned char>(text[i]);
        std::size_t length = 0;
        if (byte < 0x80) {
            length = 1;
        } else if ((byte & 0xE0) == 0xC0 && byte >= 0xC2) {
            length = 2;
        } else if ((byte & 0xF0) == 0xE0) {
            length = 3;
        } else if ((byte & 0xF8) == 0xF0 && byte <= 0xF4) {
            length = 4;
        }
        bool ok = length > 0 && i + length <= text.size();
        for (std::size_t k = 1; ok && k < length; ++k) {
            ok = (static_cast<unsigned char>(text[i + k]) & 0xC0) == 0x80;
        }
        if (ok && length > 1) {
            const auto second = static_cast<unsigned char>(text[i + 1]);
            ok = !((byte == 0xE0 && second < 0xA0) || (byte == 0xED && second >= 0xA0) ||
                   (byte == 0xF0 && second < 0x90) || (byte == 0xF4 && second >= 0x90));
        }
        if (ok) {
            out.append(text.substr(i, length));
            i += length;
        } else {
            out += "\xEF\xBF\xBD";
            ++i;
        }
    }
    return out;
}

std::string utf16_to_utf8(const std::u16string& text) {
    std::string out;
    for (std::size_t i = 0; i < text.size(); ++i) {
        char32_t code = text[i];
        if (code >= 0xD800 && code <= 0xDBFF && i + 1 < text.size() && text[i + 1] >= 0xDC00 &&
            text[i + 1] <= 0xDFFF) {
            code = 0x10000 + ((code - 0xD800) << 10) + (text[i + 1] - 0xDC00);
            ++i;
        } else if (code >= 0xD800 && code <= 0xDFFF) {
            code = 0xFFFD;
        }
        if (code < 0x80) {
            out += static_cast<char>(code);
        } else if (code < 0x800) {
            out += static_cast<char>(0xC0 | (code >> 6));
            out += static_cast<char>(0x80 | (code & 0x3F));
        } else if (code < 0x10000) {
            out += static_cast<char>(0xE0 | (code >> 12));
            out += static_cast<char>(0x80 | ((code >> 6) & 0x3F));
            out += static_cast<char>(0x80 | (code & 0x3F));
        } else {
            out += static_cast<char>(0xF0 | (code >> 18));
            out += static_cast<char>(0x80 | ((code >> 12) & 0x3F));
            out += static_cast<char>(0x80 | ((code >> 6) & 0x3F));
            out += static_cast<char>(0x80 | (code & 0x3F));
        }
    }
    return out;
}

std::u16string utf8_to_utf16(const std::string& text) {
    std::u16string out;
    const std::string checked = valid_utf8(text);
    for (std::size_t i = 0; i < checked.size();) {
        const auto byte = static_cast<unsigned char>(checked[i]);
        char32_t code = 0;
        std::size_t length = 1;
        if (byte < 0x80) {
            code = byte;
        } else if ((byte & 0xE0) == 0xC0) {
            code = byte & 0x1F;
            length = 2;
        } else if ((byte & 0xF0) == 0xE0) {
            code = byte & 0x0F;
            length = 3;
        } else {
            code = byte & 0x07;
            length = 4;
        }
        for (std::size_t k = 1; k < length; ++k) {
            code = (code << 6) | (static_cast<unsigned char>(checked[i + k]) & 0x3F);
        }
        i += length;
        if (code >= 0x10000) {
            code -= 0x10000;
            out += static_cast<char16_t>(0xD800 + (code >> 10));
            out += static_cast<char16_t>(0xDC00 + (code & 0x3FF));
        } else {
            out += static_cast<char16_t>(code);
        }
    }
    return out;
}

/// @p value as JSON. JSON has no NaN or infinity, so those become "nan", "inf" and "-inf", which
/// from_json accepts.
json real_json(double value) {
    if (std::isnan(value)) {
        return "nan";
    }
    if (std::isinf(value)) {
        return value > 0 ? "inf" : "-inf";
    }
    return value;
}

/// The double with the fewest digits that reads back as @p value, so that 0.1f shows as 0.1 rather
/// than 0.10000000149011612.
double shortest(float value) {
    std::array<char, 32> text{};
    const auto written = std::to_chars(text.data(), text.data() + text.size(), value);
    double out = value;
    std::from_chars(text.data(), written.ptr, out);
    return out;
}

/// The primitive at @p value, of field type @p type_id, as JSON.
json primitive_json(std::uint8_t type_id, const void* value) {
    switch (type_id) {
        case ts::ROS_TYPE_FLOAT: {
            const float number = *static_cast<const float*>(value);
            return real_json(std::isfinite(number) ? shortest(number) : number);
        }
        case ts::ROS_TYPE_DOUBLE:
            return real_json(*static_cast<const double*>(value));
        case ts::ROS_TYPE_LONG_DOUBLE:
            return real_json(static_cast<double>(*static_cast<const long double*>(value)));
        case ts::ROS_TYPE_CHAR:
            return static_cast<int>(*static_cast<const char*>(value));
        case ts::ROS_TYPE_WCHAR:
            return static_cast<unsigned>(*static_cast<const char16_t*>(value));
        case ts::ROS_TYPE_BOOLEAN:
            return *static_cast<const bool*>(value);
        case ts::ROS_TYPE_OCTET:
        case ts::ROS_TYPE_UINT8:
            return *static_cast<const std::uint8_t*>(value);
        case ts::ROS_TYPE_INT8:
            return *static_cast<const std::int8_t*>(value);
        case ts::ROS_TYPE_UINT16:
            return *static_cast<const std::uint16_t*>(value);
        case ts::ROS_TYPE_INT16:
            return *static_cast<const std::int16_t*>(value);
        case ts::ROS_TYPE_UINT32:
            return *static_cast<const std::uint32_t*>(value);
        case ts::ROS_TYPE_INT32:
            return *static_cast<const std::int32_t*>(value);
        case ts::ROS_TYPE_UINT64:
            return *static_cast<const std::uint64_t*>(value);
        case ts::ROS_TYPE_INT64:
            return *static_cast<const std::int64_t*>(value);
        case ts::ROS_TYPE_STRING:
            return valid_utf8(*static_cast<const std::string*>(value));
        case ts::ROS_TYPE_WSTRING:
            return utf16_to_utf8(*static_cast<const std::u16string*>(value));
        default:
            return nullptr;
    }
}

/// Room for one primitive of any type, for the fetch and assign functions of arrays.
union PrimitiveSlot {
    float f;
    double d;
    long double ld;
    char c;
    char16_t wc;
    bool b;
    std::uint8_t u8;
    std::int8_t i8;
    std::uint16_t u16;
    std::int16_t i16;
    std::uint32_t u32;
    std::int32_t i32;
    std::uint64_t u64;
    std::int64_t i64;
};

bool is_string(std::uint8_t type_id) {
    return type_id == ts::ROS_TYPE_STRING || type_id == ts::ROS_TYPE_WSTRING;
}

json field_json(const MessageMember& member, const void* field, const JsonLimits& limits);

json element_json(const MessageMember& member, const void* field, std::size_t index,
                  const JsonLimits& limits) {
    if (member.type_id_ == ts::ROS_TYPE_MESSAGE) {
        return to_json(nested(member), member.get_const_function(field, index), limits);
    }
    if (member.type_id_ == ts::ROS_TYPE_STRING) {
        std::string value;
        member.fetch_function(field, index, &value);
        return primitive_json(member.type_id_, &value);
    }
    if (member.type_id_ == ts::ROS_TYPE_WSTRING) {
        std::u16string value;
        member.fetch_function(field, index, &value);
        return primitive_json(member.type_id_, &value);
    }
    PrimitiveSlot slot{};
    member.fetch_function(field, index, &slot);
    return primitive_json(member.type_id_, &slot);
}

json field_json(const MessageMember& member, const void* field, const JsonLimits& limits) {
    if (!member.is_array_) {
        if (member.type_id_ == ts::ROS_TYPE_MESSAGE) {
            return to_json(nested(member), field, limits);
        }
        return primitive_json(member.type_id_, field);
    }
    const std::size_t size = member.size_function(field);
    const std::size_t shown = std::min(size, limits.max_array);
    json values = json::array();
    for (std::size_t i = 0; i < shown; ++i) {
        values.push_back(element_json(member, field, i, limits));
    }
    if (shown < size) {
        return json{{"length", size}, {"head", std::move(values)}};
    }
    return values;
}

[[noreturn]] void fail(const std::string& path, const std::string& problem) {
    throw MessageError((path.empty() ? std::string("the message") : path) + ": " + problem);
}

template <typename T>
T integer(const json& value, const std::string& path) {
    if (value.is_boolean()) {
        return static_cast<T>(value.get<bool>() ? 1 : 0);
    }
    if (value.is_number_integer()) {
        if (value.is_number_unsigned()) {
            const auto number = value.get<std::uint64_t>();
            if (number > static_cast<std::uint64_t>(std::numeric_limits<T>::max())) {
                fail(path, "the value " + value.dump() + " is out of range");
            }
            return static_cast<T>(number);
        }
        const auto number = value.get<std::int64_t>();
        if constexpr (std::is_signed_v<T>) {
            if (number < static_cast<std::int64_t>(std::numeric_limits<T>::min()) ||
                number > static_cast<std::int64_t>(std::numeric_limits<T>::max())) {
                fail(path, "the value " + value.dump() + " is out of range");
            }
        } else if (number < 0 || static_cast<std::uint64_t>(number) > std::numeric_limits<T>::max()) {
            fail(path, "the value " + value.dump() + " is out of range");
        }
        return static_cast<T>(number);
    }
    fail(path, "expected an integer, not " + value.dump());
}

double floating(const json& value, const std::string& path) {
    if (value.is_number()) {
        return value.get<double>();
    }
    if (value.is_string()) {
        // JSON has no NaN or infinity.
        const std::string text = value.get<std::string>();
        if (text == "nan" || text == "NaN") {
            return std::numeric_limits<double>::quiet_NaN();
        }
        if (text == "inf" || text == "Infinity") {
            return std::numeric_limits<double>::infinity();
        }
        if (text == "-inf" || text == "-Infinity") {
            return -std::numeric_limits<double>::infinity();
        }
    }
    fail(path, "expected a number, not " + value.dump());
}

/// Writes the JSON @p value into the primitive at @p target, of field type @p type_id.
void set_primitive(std::uint8_t type_id, const json& value, void* target, const std::string& path) {
    switch (type_id) {
        case ts::ROS_TYPE_FLOAT:
            *static_cast<float*>(target) = static_cast<float>(floating(value, path));
            return;
        case ts::ROS_TYPE_DOUBLE:
            *static_cast<double*>(target) = floating(value, path);
            return;
        case ts::ROS_TYPE_LONG_DOUBLE:
            *static_cast<long double*>(target) = floating(value, path);
            return;
        case ts::ROS_TYPE_CHAR:
            if (value.is_string() && value.get<std::string>().size() == 1) {
                *static_cast<char*>(target) = value.get<std::string>()[0];
            } else {
                *static_cast<char*>(target) = static_cast<char>(integer<std::int8_t>(value, path));
            }
            return;
        case ts::ROS_TYPE_WCHAR:
            *static_cast<char16_t*>(target) =
                static_cast<char16_t>(integer<std::uint16_t>(value, path));
            return;
        case ts::ROS_TYPE_BOOLEAN:
            if (!value.is_boolean()) {
                fail(path, "expected true or false, not " + value.dump());
            }
            *static_cast<bool*>(target) = value.get<bool>();
            return;
        case ts::ROS_TYPE_OCTET:
        case ts::ROS_TYPE_UINT8:
            *static_cast<std::uint8_t*>(target) = integer<std::uint8_t>(value, path);
            return;
        case ts::ROS_TYPE_INT8:
            *static_cast<std::int8_t*>(target) = integer<std::int8_t>(value, path);
            return;
        case ts::ROS_TYPE_UINT16:
            *static_cast<std::uint16_t*>(target) = integer<std::uint16_t>(value, path);
            return;
        case ts::ROS_TYPE_INT16:
            *static_cast<std::int16_t*>(target) = integer<std::int16_t>(value, path);
            return;
        case ts::ROS_TYPE_UINT32:
            *static_cast<std::uint32_t*>(target) = integer<std::uint32_t>(value, path);
            return;
        case ts::ROS_TYPE_INT32:
            *static_cast<std::int32_t*>(target) = integer<std::int32_t>(value, path);
            return;
        case ts::ROS_TYPE_UINT64:
            *static_cast<std::uint64_t*>(target) = integer<std::uint64_t>(value, path);
            return;
        case ts::ROS_TYPE_INT64:
            *static_cast<std::int64_t*>(target) = integer<std::int64_t>(value, path);
            return;
        case ts::ROS_TYPE_STRING:
            if (!value.is_string()) {
                fail(path, "expected a string, not " + value.dump());
            }
            *static_cast<std::string*>(target) = value.get<std::string>();
            return;
        case ts::ROS_TYPE_WSTRING:
            if (!value.is_string()) {
                fail(path, "expected a string, not " + value.dump());
            }
            *static_cast<std::u16string*>(target) = utf8_to_utf16(value.get<std::string>());
            return;
        default:
            fail(path, "has a field type this server cannot write");
    }
}

void check_string_bound(const MessageMember& member, const json& value, const std::string& path) {
    if (member.string_upper_bound_ > 0 && value.is_string() &&
        value.get<std::string>().size() > member.string_upper_bound_) {
        fail(path, "longer than its bound of " + std::to_string(member.string_upper_bound_));
    }
}

void field_from_json(const MessageMember& member, const json& value, void* field,
                     const std::string& path, const std::function<Stamp()>& now) {
    if (!member.is_array_) {
        if (member.type_id_ == ts::ROS_TYPE_MESSAGE) {
            from_json(value, nested(member), field, now);
            return;
        }
        check_string_bound(member, value, path);
        set_primitive(member.type_id_, value, field, path);
        return;
    }
    if (!value.is_array()) {
        fail(path, "expected an array, not " + value.dump());
    }
    const std::size_t size = value.size();
    if (member.array_size_ > 0 && !member.is_upper_bound_) {
        // A fixed-size array: fewer values leave the rest at their defaults.
        if (size > member.array_size_) {
            fail(path, "holds " + std::to_string(member.array_size_) + " values, not " +
                           std::to_string(size));
        }
    } else {
        if (member.is_upper_bound_ && size > member.array_size_) {
            fail(path, "holds at most " + std::to_string(member.array_size_) + " values, not " +
                           std::to_string(size));
        }
        member.resize_function(field, size);
    }
    // Elements report errors with an empty path, and the element's path is prefixed only when one
    // is thrown, so that no string is built per element.
    for (std::size_t i = 0; i < size; ++i) {
        try {
            if (member.type_id_ == ts::ROS_TYPE_MESSAGE) {
                from_json(value[i], nested(member), member.get_function(field, i), now);
            } else if (member.type_id_ == ts::ROS_TYPE_STRING) {
                check_string_bound(member, value[i], {});
                std::string text;
                set_primitive(member.type_id_, value[i], &text, {});
                member.assign_function(field, i, &text);
            } else if (member.type_id_ == ts::ROS_TYPE_WSTRING) {
                std::u16string text;
                set_primitive(member.type_id_, value[i], &text, {});
                member.assign_function(field, i, &text);
            } else {
                PrimitiveSlot slot{};
                set_primitive(member.type_id_, value[i], &slot, {});
                member.assign_function(field, i, &slot);
            }
        } catch (const MessageError& error) {
            const std::string at = path + "[" + std::to_string(i) + "]";
            const std::string what = error.what();
            constexpr std::string_view whole = "the message";
            throw MessageError(what.starts_with(whole) ? at + what.substr(whole.size())
                                                       : at + "." + what);
        }
    }
}

bool is_time(const MessageMembers& members) {
    return std::string_view(members.message_namespace_) == "builtin_interfaces::msg" &&
           std::string_view(members.message_name_) == "Time";
}

std::string field_type_name(const MessageMember& member) {
    std::string name;
    switch (member.type_id_) {
        case ts::ROS_TYPE_FLOAT:
            name = "float32";
            break;
        case ts::ROS_TYPE_DOUBLE:
            name = "float64";
            break;
        case ts::ROS_TYPE_LONG_DOUBLE:
            name = "long double";
            break;
        case ts::ROS_TYPE_CHAR:
            name = "char";
            break;
        case ts::ROS_TYPE_WCHAR:
            name = "wchar";
            break;
        case ts::ROS_TYPE_BOOLEAN:
            name = "bool";
            break;
        case ts::ROS_TYPE_OCTET:
            name = "byte";
            break;
        case ts::ROS_TYPE_UINT8:
            name = "uint8";
            break;
        case ts::ROS_TYPE_INT8:
            name = "int8";
            break;
        case ts::ROS_TYPE_UINT16:
            name = "uint16";
            break;
        case ts::ROS_TYPE_INT16:
            name = "int16";
            break;
        case ts::ROS_TYPE_UINT32:
            name = "uint32";
            break;
        case ts::ROS_TYPE_INT32:
            name = "int32";
            break;
        case ts::ROS_TYPE_UINT64:
            name = "uint64";
            break;
        case ts::ROS_TYPE_INT64:
            name = "int64";
            break;
        case ts::ROS_TYPE_STRING:
            name = "string";
            break;
        case ts::ROS_TYPE_WSTRING:
            name = "wstring";
            break;
        case ts::ROS_TYPE_MESSAGE:
            name = type_name(nested(member));
            break;
        default:
            name = "unknown";
    }
    if (is_string(member.type_id_) && member.string_upper_bound_ > 0) {
        name += "<=" + std::to_string(member.string_upper_bound_);
    }
    if (member.is_array_) {
        if (member.is_upper_bound_) {
            name += "[<=" + std::to_string(member.array_size_) + "]";
        } else if (member.array_size_ > 0) {
            name += "[" + std::to_string(member.array_size_) + "]";
        } else {
            name += "[]";
        }
    }
    return name;
}

}  // namespace

json to_json(const MessageMembers& members, const void* data, const JsonLimits& limits) {
    json out = json::object();
    for (std::uint32_t i = 0; i < members.member_count_; ++i) {
        const MessageMember& member = members.members_[i];
        out[member.name_] = field_json(member, static_cast<const char*>(data) + member.offset_, limits);
    }
    return out;
}

void from_json(const json& value, const MessageMembers& members, void* data,
               const std::function<Stamp()>& now) {
    if (is_time(members) && value.is_string() && value.get<std::string>() == "now") {
        const Stamp stamp = now();
        json fields{{"sec", stamp.sec}, {"nanosec", stamp.nanosec}};
        from_json(fields, members, data, now);
        return;
    }
    if (!value.is_object()) {
        fail("", "expected an object of fields for " + type_name(members) + ", not " + value.dump());
    }
    for (const auto& [key, field_value] : value.items()) {
        const MessageMember* member = nullptr;
        for (std::uint32_t i = 0; i < members.member_count_; ++i) {
            if (key == members.members_[i].name_) {
                member = &members.members_[i];
                break;
            }
        }
        if (member == nullptr) {
            std::string names;
            for (std::uint32_t i = 0; i < members.member_count_; ++i) {
                names += (i == 0 ? "" : ", ") + std::string(members.members_[i].name_);
            }
            fail(key, type_name(members) + " has no such field; its fields are " + names);
        }
        try {
            field_from_json(*member, field_value, static_cast<char*>(data) + member->offset_, key, now);
        } catch (const MessageError& error) {
            const std::string what = error.what();
            // Prefix the nested message's field path with this field's name.
            if (member->type_id_ == ts::ROS_TYPE_MESSAGE && !member->is_array_ &&
                !what.starts_with("the message")) {
                throw MessageError(key + "." + what);
            }
            if (member->type_id_ == ts::ROS_TYPE_MESSAGE && !member->is_array_) {
                throw MessageError(key + what.substr(std::string_view("the message").size()));
            }
            throw;
        }
    }
}

json describe(const MessageMembers& members) {
    json out = json::object();
    for (std::uint32_t i = 0; i < members.member_count_; ++i) {
        const MessageMember& member = members.members_[i];
        if (member.type_id_ == ts::ROS_TYPE_MESSAGE) {
            out[member.name_] = {{"type", field_type_name(member)},
                                 {"fields", describe(nested(member))}};
        } else {
            out[member.name_] = field_type_name(member);
        }
    }
    return out;
}

}  // namespace mcp_ros2
