#include "gui_signal.h"
#include <utility>
#include <boost/assign/ptr_map_inserter.hpp>
#include <boost/assign/list_of.hpp> 
#include "FL/Fl_Widget.H"
#include "console_log.h"
#include "gui_fl_widget_callback_data.h"

namespace gui {

// public func.

Signal::Signal()
    :display_show_signal_(new DisplayShowSignal_t),
    main_window_timeout_signal_(new MainWindowTimeoutSignal_t)
{
}

Signal::~Signal()
{
}

void Signal::Send(Fl_Widget *fl_widget)
{
    FlWidgetCallbackData_t *callback_data
        = static_cast<FlWidgetCallbackData_t *>(fl_widget->user_data());
    callback_data->signal(fl_widget);
}

void Signal::DisplayShow()
{
    (*display_show_signal_)();
}

boost::signals2::connection Signal::ConnectDisplayShow(
        const DisplayShowSignal_t::slot_type &slot
        )
{
    return display_show_signal_->connect(slot);
}

void Signal::MainWindowTimeout()
{
    (*main_window_timeout_signal_)();
}

boost::signals2::connection Signal::ConnectMainWindowTimeout(
        const MainWindowTimeoutSignal_t::slot_type &slot
        )
{
    return main_window_timeout_signal_->connect(slot);
}

// protected func.

// private func.

} // namespace gui {
