#ifndef _GUI_UTILS_H_
#define _GUI_UTILS_H_

#include "gui_def.h"
#include "math_type.h"
#include "gui_fwd.h"
#include "gui_type.h"

namespace gui {

Color_t Red();
Color_t Green();
Color_t Blue();
Color_t Cyan();
Color_t Magenta();
Color_t Yellow();
Color_t Orange();
/**
 * \brief
 * \param[in]
 * \return
 */
math::Vector3d_t EvalCursorPos(const math::ScreenPos_t &mouse_pos);
/**
 * \brief
 * \param[in]
 * \param[out]
 */
void SelectColor(
        const math::ScreenPos_t &mouse_pos,
        Marker *marker
        );
/**
* \brief
* \param[in]
* \param[out]
*/
void RaySelect(
        const math::ScreenPos_t &mouse_pos,
        Marker *marker
        );
/**
 * \brief
 * \param[in]
 * \param[in]
 * \param[in]
 * \param[out]
 */
void DragMarker(
        const math::ScreenPos_t &mouse_pos,
        const math::ScreenPos_t &previous_pos,
        const int mode,
        Marker *marker
        );

} // namespace gui {

#endif // #ifndef _GUI_UTILS_H_
