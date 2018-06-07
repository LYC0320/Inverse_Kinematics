#include "gui_utils.h"
#include <windows.h>
#include <GL/gl.h>
#include <GL/glu.h>
#include "boost/numeric/conversion/cast.hpp"
#include "FL/Enumerations.H"
#include "FL/Fl.H"
#include "math_utils.h"
#include "gl_utils.h"
#include "gui_marker.h"
#include "gui_signal.h"
#include "gui_color.h"

namespace gui {

Color_t Red()
{
    return Color_t(
            Color_t::Scalar_t{1.0f},
            Color_t::Scalar_t{0.0f},
            Color_t::Scalar_t{0.0f},
            Color_t::Scalar_t{1.0f}
            );
}

Color_t Green()
{
    return Color_t(
            Color_t::Scalar_t{0.0f},
            Color_t::Scalar_t{1.0f},
            Color_t::Scalar_t{0.0f},
            Color_t::Scalar_t{1.0f}
            );
}

Color_t Blue()
{
    return Color_t(
            Color_t::Scalar_t{0.0f},
            Color_t::Scalar_t{0.0f},
            Color_t::Scalar_t{1.0f},
            Color_t::Scalar_t{1.0f}
            );
}

Color_t Cyan()
{
    return Color_t(
            Color_t::Scalar_t{0.0f},
            Color_t::Scalar_t{1.0f},
            Color_t::Scalar_t{1.0f},
            Color_t::Scalar_t{1.0f}
            );
}

Color_t Magenta()
{
    return Color_t(
            Color_t::Scalar_t{1.0f},
            Color_t::Scalar_t{0.0f},
            Color_t::Scalar_t{1.0f},
            Color_t::Scalar_t{1.0f}
            );
}

Color_t Yellow()
{
    return Color_t(
            Color_t::Scalar_t{1.0f},
            Color_t::Scalar_t{1.0f},
            Color_t::Scalar_t{0.0f},
            Color_t::Scalar_t{0.9f}
            );
}

Color_t Orange()
{
    return Color_t(
            Color_t::Scalar_t{1.0f},
            Color_t::Scalar_t{0.54f},
            Color_t::Scalar_t{0.1f},
            Color_t::Scalar_t{0.9f}
            );
}

math::Vector3d_t EvalCursorPos(const math::ScreenPos_t &mouse_pos)
{
    GLdouble model_view_mat[glmat4::Size] = {0.0};
    GLdouble projection_mat[glmat4::Size] = {0.0};
    GLint viewport[4] = {0};
    glGetDoublev(GL_MODELVIEW_MATRIX, model_view_mat);
    glGetDoublev(GL_PROJECTION_MATRIX, projection_mat);
    glGetIntegerv(GL_VIEWPORT, viewport);

    GLfloat win_x = boost::numeric_cast<GLfloat>(mouse_pos.x());
    GLfloat win_y = boost::numeric_cast<GLfloat>(viewport[3])
                  - boost::numeric_cast<GLfloat>(mouse_pos.y());
    GLfloat win_z = GLfloat{0.0};
    glReadPixels(
            mouse_pos.x(),               // window coordinate x
            viewport[3] - mouse_pos.y(), // window coordinate y
            1,                           // width
            1,                           // height
            GL_DEPTH_COMPONENT,          // format
            GL_FLOAT,                    // type
            &win_z                       // data, the pixel data
            );

    //get world position of cursor
    GLdouble pos_x = GLdouble{0.0};
    GLdouble pos_y = GLdouble{0.0};
    GLdouble pos_z = GLdouble{0.0};
    gluUnProject(
            win_x,
            win_y,
            0.98,
            model_view_mat,
            projection_mat,
            viewport,
            &pos_x,
            &pos_y,
            &pos_z
            );

    return math::Vector3d_t(pos_x, pos_y, pos_z);
}

void SelectColor(
        const math::ScreenPos_t &mouse_pos,
        Marker *marker
        )
{
    GLint viewport[4] = {0};
    glGetIntegerv(GL_VIEWPORT, viewport);

    const math::ScreenPos_t screen_pos(
            mouse_pos.x(),
            viewport[3] - mouse_pos.y()
            );
    float pixel[3] = {0.0};
    glReadPixels(
            screen_pos.x(),
            screen_pos.y(),
            1,
            1,
            GL_RGB,
            //GL_UNSIGNED_BYTE,
            GL_FLOAT,
            pixel
            );

    const float color_epsilon = float{1.0f} / float{255.0};

//    if (!math::IsApprox(marker->color().red(), pixel[0], color_epsilon))
//    {
//        LOGDBG << "diff red" << '\n'
//            << "marker " << marker->color().red() << '\n'
//            << "read " << pixel[0] << std::endl;
//    }
//    if (!math::IsApprox(marker->color().green(), pixel[1], color_epsilon))
//    {
//        LOGDBG << "diff green" << '\n'
//            << "marker " << marker->color().green() << '\n'
//            << "read " << pixel[1] << std::endl;
//    }
//    if (!math::IsApprox(marker->color().blue(), pixel[2], color_epsilon))
//    {
//        LOGDBG << "diff blue" << '\n'
//            << "marker " << marker->color().blue() << '\n'
//            << "read " << pixel[2] << std::endl;
//    }

    const bool is_select
        = math::IsApprox(marker->color().red(), pixel[0], color_epsilon)
        && math::IsApprox(marker->color().green(), pixel[1], color_epsilon)
        && math::IsApprox(marker->color().blue(), pixel[2], color_epsilon);
    if (is_select)
    {
        marker->Select();
    }
}

void RaySelect(
        const math::ScreenPos_t &mouse_pos,
        Marker *marker
        )
{
    double model_view_mat[16], projection_mat[16];
    GLint viewport[4] = { 0 };
    glGetDoublev(GL_MODELVIEW_MATRIX, model_view_mat);
    glGetDoublev(GL_PROJECTION_MATRIX, projection_mat);
    glGetIntegerv(GL_VIEWPORT, viewport);
    /*
    const math::ScreenPos_t screen_pos(
        mouse_pos.x(),
        viewport[3] - mouse_pos.y()
        );

    math::Vector3d_t ray_start_pos, ray_end_pos;
    gluUnProject(
        screen_pos.x(), screen_pos.y(), 0.0, model_view_mat, projection_mat,
        viewport, &ray_start_pos.x(), &ray_start_pos.y(), &ray_start_pos.z()
        );
    gluUnProject(
        screen_pos.x(), screen_pos.y(), 1.0, model_view_mat, projection_mat,
        viewport, &ray_end_pos.x(), &ray_end_pos.y(), &ray_end_pos.z()
        );

    math::Vector3d_t start_to_end_vector = (ray_end_pos - ray_start_pos).normalized();
    Eigen::ParametrizedLine<double, 3> ray_line(
        ray_start_pos,
        start_to_end_vector
        );
    double point_to_ray_distance = ray_line.distance(marker->target_pos());
    if (point_to_ray_distance < param::ConfigValue<double>("ik_marker_radius"))
    {
        LOGMSG << "hit ball!\n";
    }
    */

    math::Vector3d_t ray_vec(
        (2.0f * mouse_pos.x()) / -1.0f,
        1.0f - (2.0f * mouse_pos.y()) / viewport[3],
        1.0f
        );

}

void DragMarker(
        const math::ScreenPos_t &mouse_pos,
        const math::ScreenPos_t &previous_pos,
        const int mode,
        Marker *marker
        )
{
    static const math::Vector3d_t::RealScalar kZero = math::Vector3d_t::RealScalar{0.0};
    const math::Vector3d_t cur_cursor_pos = EvalCursorPos(mouse_pos);
    const math::Vector3d_t previous_cursor_pos = EvalCursorPos(previous_pos);
    const math::Vector3d_t diff_pos = cur_cursor_pos - previous_cursor_pos;

    if (FL_LEFT_MOUSE == mode)
    {
        marker->Move(math::Vector3d_t(diff_pos.x(), diff_pos.y(), kZero));
    }
    else
    {
        marker->Move(math::Vector3d_t(diff_pos.x(), kZero, diff_pos.y()));
    }
}

} // namespace gui {
