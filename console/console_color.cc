#include "console_color.h"
#include <windows.h>

namespace console {

int32_t White()
{
    return int32_t{7};
}

int32_t Red()
{
    return FOREGROUND_RED;
}

int32_t Yellow()
{
    return int32_t{14};
}

int32_t Cyan()
{
    return int32_t{11};
}

int32_t Green()
{
    return int32_t{10};
}

int32_t Blue()
{
    return int32_t{9};
}

int32_t LogErrColor()
{
    return int32_t{FOREGROUND_RED | BACKGROUND_INTENSITY};
}

} // namespace console {
