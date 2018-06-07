#ifndef _PARAM_XML_PARSER_H_
#define _PARAM_XML_PARSER_H_

#include <string>
#include <memory>
#include <typeinfo>
#include "boost/optional.hpp"
#include "param_type.h"

namespace param {

class XmlParser final
{

public:

    typedef std::string Key_t;
    typedef boost::optional<const Ptree_t &> ConstNodeHdl_t;

    XmlParser();
    XmlParser(const XmlParser &) = delete;
    virtual ~XmlParser();
    XmlParser &operator=(const XmlParser &) = delete;

    static Key_t attr_key();

    void ReadFile(const std::string &file_name);

    template<typename Data>
    Data value(const Key_t &key) const;
    ConstNodeHdl_t child_node(const Key_t &key) const;

protected:

private:

    std::unique_ptr<Ptree_t> ptree_;
};

} // namespace param {

namespace param {

// public func.

template<typename Data>
Data XmlParser::value(const Key_t &key) const
{
    //return ptree_->get_optional<Data>(key).get();
    const boost::optional<Data> target_value = ptree_->get_optional<Data>(key);
    if (!target_value.is_initialized())
    {
        LOGERR << "invalid key: " << key << std::endl;
        assert(FALSE);
    }

    return target_value.get();
}

} // namespace param {

#endif // #ifndef _PARAM_XML_PARSER_H_
