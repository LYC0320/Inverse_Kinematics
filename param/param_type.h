#ifndef _PARAM_TYPE_H_
#define _PARAM_TYPE_H_

#include "global_def.h"
#include <string>
#include <vector>
#include "boost/property_tree/xml_parser.hpp"

namespace param {
typedef boost::property_tree::ptree Ptree_t;
typedef std::string Key_t;
typedef std::vector<Key_t> KeyColl_t;
} // namespace param {

#endif // #ifndef _PARAM_TYPE_H_
