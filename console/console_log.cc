#include <windows.h>
#include "console_log.h"

namespace console {

std::ostream &colorMsg(
        const std::string &message,
        int32_t color
        )
{
    HANDLE console_handle;
    console_handle = GetStdHandle(STD_OUTPUT_HANDLE);
    SetConsoleTextAttribute(console_handle, color);
    std::cout << message;

    return std::cout;
}

std::ostream &colorMsg(
        const std::string &message,
        const std::string &file_name,
        const uint32_t line_num,
        const int32_t color
        )
{
    HANDLE handle;
    handle = GetStdHandle(STD_OUTPUT_HANDLE);
    SetConsoleTextAttribute(handle, color);
    std::size_t index = file_name.find_last_of("\\") + 1;

    std::cout << "["
        << file_name.substr(index, file_name.size() - index) << ":"
        << line_num << "] ";
    SetConsoleTextAttribute(handle, console::White());
    return std::cout;
}

} // namespace console {
