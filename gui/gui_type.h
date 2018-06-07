#ifndef _GUI_TYPE_H_
#define _GUI_TYPE_H_

#include <string>
#include <unordered_map>
#include <vector>
#include "boost/signals2.hpp"
#include "gui_fwd.h"

class Fl_Widget;

namespace gui {
typedef std::string Id_t;
typedef boost::signals2::signal<void (Fl_Widget *)> WidgetSignal_t;
typedef boost::signals2::signal<void (Fl_Widget *, void *)> WidgetVoidSignal_t;
typedef boost::signals2::signal<void (Fl_Widget *, long *)> WidgetLongSignal_t;
typedef boost::signals2::signal<void ()> GlRedrawSignal_t;
typedef boost::signals2::signal<void (void *)> TimeoutSignal_t;
typedef boost::signals2::signal<void ()> DisplayShowSignal_t;
typedef boost::signals2::signal<void ()> MainWindowTimeoutSignal_t;
typedef std::unordered_map<std::string, bool> SwitchMap_t;
typedef Color<float> Color_t;
typedef std::vector<Color_t> ColorColl_t;
} // namespace gui {

#endif // #ifndef _GUI_TYPE_H_
