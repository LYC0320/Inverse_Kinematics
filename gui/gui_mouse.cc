#include "gui_mouse.h"

namespace gui {

// public func.

Mouse::Mouse()
    :pos_(new math::ScreenPos_t),
    button_(0)
{
}

Mouse::Mouse(const Mouse &other)
    :pos_(new math::ScreenPos_t(other.pos())),
    button_(other.button())
{
}

Mouse::~Mouse()
{
}

Mouse &Mouse::operator=(const Mouse &other)
{
    if (&other == this)
    {
        return *this;
    }

    *pos_ = other.pos();
    button_ = other.button();

    return *this;
}

math::ScreenPos_t Mouse::pos() const
{
    return *pos_;
}

int Mouse::button() const
{
    return button_;
}

void Mouse::set_pos(const math::ScreenPos_t &a_kPos)
{
    *pos_ = a_kPos;
}

void Mouse::set_button(const int button)
{
    button_ = button;
}

// protected func.

// private func.

} // namespace gui {
