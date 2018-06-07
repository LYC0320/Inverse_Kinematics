#ifndef _GUI_MOUSE_H_
#define _GUI_MOUSE_H_

#include "gui_def.h"
#include <memory>
#include "math_type.h"

namespace gui {

class Mouse final
{

public:
    /**
     * \brief
     */
    Mouse();
    /**
     * \brief
     */
    Mouse(const Mouse &other);
    /**
     * \brief
     */
    virtual ~Mouse();
    /**
     * \brief
     * \param[in] other
     */
    Mouse &operator=(const Mouse &other);
    /**
     * \brief Get the mouse position
     * \return Mouse position
     */
    math::ScreenPos_t pos() const;
    /**
     * \brief Get the mouse button
     * \return Mouse button
     */
    int button() const;
    /**
     * \brief Set the mouse position
     * \param[in] pos
     */
    void set_pos(const math::ScreenPos_t &pos);
    /**
     * \brief Set the mouse button
     * \param[in] button
     */
    void set_button(const int button);

protected:

private:

    std::unique_ptr<math::ScreenPos_t> pos_;
    int button_;
};

} // namespace gui {

#endif // #ifndef _GUI_MOUSE_H_
