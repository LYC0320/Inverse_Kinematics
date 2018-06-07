#include "param_config.h"
#include "math_type.h"
#include "gui_type.h"
#include "gui_color.h"
#include "param_xml_parser.h"

namespace param {

// public func.

Config::Config(
        const std::string &file_name,
        const std::string &parent_path
        )
    :parser_(new XmlParser),
    parent_path_(new std::string(parent_path))
{
    parser_->ReadFile(file_name);
}

Config::~Config()
{
}

template<typename Data>
Data Config::value(const Key_t &key) const
{
    return parser_->value<Data>(*parent_path_ + key);
}
template double Config::value<double>(const Key_t &) const;
template float Config::value<float>(const Key_t &) const;
template int Config::value<int>(const Key_t &) const;
template unsigned char Config::value<unsigned char>(const Key_t &) const;
template bool Config::value<bool>(const Key_t &) const;
template std::string Config::value<std::string>(const Key_t &) const;
template<> math::Vector3d_t Config::value<math::Vector3d_t>(const Key_t &key) const
{
    auto child_node = parser_->child_node(*parent_path_ + key);
    return math::Vector3d_t(
            child_node->get_optional<math::Vector3d_t::RealScalar>(XmlParser::attr_key() + "x").get(),
            child_node->get_optional<math::Vector3d_t::RealScalar>(XmlParser::attr_key() + "y").get(),
            child_node->get_optional<math::Vector3d_t::RealScalar>(XmlParser::attr_key() + "z").get()
            );
}
template<> gui::Color_t Config::value<gui::Color_t>(const Key_t &key) const
{
    auto child_node = parser_->child_node(*parent_path_ + key);
    return gui::Color_t(
            child_node->get_optional<float>(XmlParser::attr_key() + "r").get(),
            child_node->get_optional<float>(XmlParser::attr_key() + "g").get(),
            child_node->get_optional<float>(XmlParser::attr_key() + "b").get(),
            child_node->get_optional<float>(XmlParser::attr_key() + "a").get()
            );
}

// protected func.

// private func.

} // namespace param {
