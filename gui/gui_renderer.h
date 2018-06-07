#ifndef _GUI_RENDERER_H_
#define _GUI_RENDERER_H_

#include "gui_def.h"
#include <memory>
#include "FL/glu.h"
#include "math_type.h"
#include "geometry_fwd.h"
#include "gui_fwd.h"
#include "gui_type.h"
#include "gui_polar_rotation.h"

namespace gui {

class Renderer final
{

public:

    Renderer();
    Renderer(const Renderer &) = delete;
    virtual ~Renderer();
    Renderer &operator=(const Renderer &) = delete;

    double camera_zoom() const;
    PolarRotation camera_rotation() const;
    math::Vector3d_t camera_translation() const;
    math::Vector3d_t camera_look_at() const;

    void set_color(const Color_t &color);
    void set_camera_zoom(const double zoom);
    void set_camera_rotation(
            const double azimuth,
            const double elevation,
            const double twist
            );
    void set_camera_translation(const math::Vector3d_t &xlt);
    void set_camera_look_at(const math::Vector3d_t &pos);

    void set_translation(const math::Vector3d_t &xlt);
    void set_rotation(const double radian, const math::Vector3d_t &rotation_axis);
    void set_point_size(const double size);
    void set_line_width(const double width);

    void ShiftCameraRotation(
            const double azimuth,
            const double elevation,
            const double twist
            );
    void ShiftCameraTranslation(const math::Vector3d_t &shift_xlt);
    void ShiftCameraLookAt(const math::Vector3d_t &shift_pos);
    void ApplyCameraView();

    void DrawLine(
            const math::Vector3d_t &start_pos,
            const math::Vector3d_t &end_pos
            ) const;
    void DrawCylinder(
            const math::Vector3d_t &start_pos,
            const math::Vector3d_t &end_pos,
            const double radius
            ) const;
    void DrawCylinder(
            const math::RotMat3d_t &rot_mat,
            const math::Vector3d_t &pos,
            const double height,
            const double radius
            );
    void DrawArrow(const geometry::Arrow &arrow);
    void DrawSphere(const math::Vector3d_t &pos, const double radius);
    void DrawPolyline(const math::Vector3dColl_t &vertex_coll, const bool is_closed);
    void DrawPointSet(const math::Vector3dColl_t &point_set);
    void DrawTriad();
    void DrawGround();

protected:

private:

    std::unique_ptr<Camera> camera_;
    GLUquadricObj *glu_quadric_obj_;
};

} // namespace gui {

#endif // #ifndef _GUI_RENDERER_H_
