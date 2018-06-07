#ifndef _PARAM_CONFIG_H_
#define _PARAM_CONFIG_H_

#include <memory>
#include "param_type.h"

namespace param {

class XmlParser;

class Config final
{

public:

    Config(const std::string &file_name, const std::string &parent_path);
    Config(const Config &) = delete;
    virtual ~Config();
    Config &operator=(const Config &) = delete;

    template<typename Data>
    Data value(const Key_t &key) const;

protected:

private:

    std::unique_ptr<XmlParser> parser_;
    std::unique_ptr<std::string> parent_path_;
};

} // namespace param {

#endif // #ifndef _PARAM_CONFIG_H_
