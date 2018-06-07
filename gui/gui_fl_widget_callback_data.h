#ifndef _GUI_FL_WIDGET_CALLBACK_DATA_H_
#define _GUI_FL_WIDGET_CALLBACK_DATA_H_

#include "gui_def.h"
#include <memory>
#include "gui_type.h"

namespace gui {

typedef struct FlWidgetCallbackData final
{
    WidgetSignal_t signal;
} FlWidgetCallbackData_t;

} // namespace gui {

#endif // #ifndef _GUI_FL_WIDGET_CALLBACK_DATA_H_
