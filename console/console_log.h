#ifndef _CONSOLE_LOG_H_
#define _CONSOLE_LOG_H_

#include <string>
#include <iostream>
#include "console_color.h"

#define LOGMSG  (console::colorMsg("Message", __FILE__, __LINE__, console::Green()))
#define LOGDBG  (console::colorMsg("Debug",   __FILE__, __LINE__, console::Cyan()))
#define LOGWARN (console::colorMsg("Warning", __FILE__, __LINE__, console::Yellow()))
#define LOGERR  (console::colorMsg("Error",   __FILE__, __LINE__, console::LogErrColor()))

namespace console {

/**
 * \brief
 * \param[in] message
 * \param[in] color
 * \return
 */
std::ostream &colorMsg(
        const std::string &message,
        int32_t color
        );
/**
 * \brief
 * \param[in] message
 * \param[in] file_name
 * \param[in] line_num
 * \param[in] color
 * \return
 */
std::ostream &colorMsg(
        const std::string &message,
        const std::string &file_name,
        const uint32_t line_num,
        const int32_t color
        );

} // namespace console {

#endif // #ifndef _CONSOLE_LOG_H_
