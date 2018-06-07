#include "gui_renderer.h"
#include <algorithm>
#include "FL/gl.h"
#include "FL/glut.H"
#include "FL/glu.h"
#include "math_utils.h"
#include "geometry_arrow.h"
#include "gui_camera.h"
#include "gui_polar_rotation.h"
#include "gui_color.h"

namespace gui {

// public func.

Renderer::Renderer()
    :camera_(new Camera),
    glu_quadric_obj_(gluNewQuadric())
{
}

Renderer::~Renderer()
{
    gluDeleteQuadric(glu_quadric_obj_);
}

double Renderer::camera_zoom() const
{
    return camera_->zoom();
}

PolarRotation Renderer::camera_rotation() const
{
    return camera_->polar_rotation();
}

math::Vector3d_t Renderer::camera_translation() const
{
    return camera_->translation();
}

math::Vector3d_t Renderer::camera_look_at() const
{
    return camera_->look_at_pos();
}

void Renderer::set_color(const Color_t &color)
{
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glColor4f(
            color.red(),
            color.green(),
            color.blue(),
            color.alpha()
            );
}

void Renderer::set_camera_zoom(const double zoom)
{
    camera_->set_zoom(zoom);
}

void Renderer::set_camera_rotation(
        const double azimuth,
        const double elevation,
        const double twist
        )
{
    camera_->set_polar_rotation(
            PolarRotation(azimuth, elevation, twist)
            );
}

void Renderer::set_camera_translation(const math::Vector3d_t &xlt)
{
    camera_->set_translation(xlt);
}

void Renderer::set_camera_look_at(const math::Vector3d_t &pos)
{
    camera_->set_look_at_pos(pos);
}

void Renderer::set_translation(const math::Vector3d_t &xlt)
{
    glTranslated(xlt.x(), xlt.y(), xlt.z());
}

void Renderer::set_rotation(const double radian, const math::Vector3d_t &rotation_axis)
{
    glRotated(radian, rotation_axis.x(), rotation_axis.y(), rotation_axis.z());
}

void Renderer::set_point_size(const double size)
{
    glPointSize(boost::numeric_cast<GLfloat>(size));
}

void Renderer::set_line_width(const double width)
{
    glLineWidth(boost::numeric_cast<GLfloat>(width));
}

void Renderer::ShiftCameraRotation(
            const double azimuth,
            const double elevation,
            const double twist
            )
{
    camera_->ShiftRotation(azimuth, elevation, twist);
}

void Renderer::ShiftCameraTranslation(const math::Vector3d_t &shift_xlt)
{
    camera_->ShiftTranslation(shift_xlt);
}

void Renderer::ShiftCameraLookAt(const math::Vector3d_t &shift_pos)
{
    camera_->ShiftLookAt(shift_pos);
}

void Renderer::ApplyCameraView()
{
    glTranslated(camera_->translation().x(), camera_->translation().y(), camera_->translation().z());
    glTranslated(camera_->look_at_pos().x(), camera_->look_at_pos().y(), camera_->look_at_pos().z());

    glRotated(-camera_->twist(), 0.0, 1.0, 0.0);
    glRotated(-camera_->elevation(), 1.0, 0.0, 0.0);
    glRotated(camera_->azimuth(), 0.0, 1.0, 0.0);

    glTranslated(-camera_->look_at_pos().x(), -camera_->look_at_pos().y(), -camera_->look_at_pos().z());
    glScaled(camera_->zoom(), camera_->zoom(), camera_->zoom());
}

void Renderer::DrawLine(
        const math::Vector3d_t &start_pos,
        const math::Vector3d_t &end_pos
        ) const
{
    glBegin(GL_LINES);
    glVertex3d(start_pos.x(), start_pos.y(), start_pos.z());
    glVertex3d(end_pos.x(), end_pos.y(), end_pos.z());
    glEnd();
}

void Renderer::DrawCylinder(
        const math::Vector3d_t &start_pos,
        const math::Vector3d_t &end_pos,
        const double radius
        ) const
{
    //the same quadric can be re-used for drawing many cylinders
    gluQuadricNormals(glu_quadric_obj_, GLU_SMOOTH);
    static const int kSubdivisions = 18;

    math::Vector3d_t cyl_vec;
    cyl_vec = end_pos - start_pos;
    double vx = cyl_vec.x();
    double vy = cyl_vec.y();
    double vz = cyl_vec.z();
    double ax,rx,ry,rz;
    double cyl_len = cyl_vec.norm();

    glPushMatrix();

    glTranslated(start_pos.x(), start_pos.y(), start_pos.z());
    if (fabs(vz) < 0.0001)
    {
        glRotated(90, 0, 1, 0);
        ax = 57.2957795*-atan( vy / vx );
        if (vx < 0){
            ax = ax + 180;
        }
        rx = 1;
        ry = 0;
        rz = 0;
    }
    else
    {
        ax = 57.2957795 * acos(vz/cyl_len);
        if (vz < 0.0)
        {
            ax = -ax;
        }
        rx = -vy*vz;
        ry = vx*vz;
        rz = 0;
    }
    glRotated(ax, rx, ry, rz);
    gluQuadricOrientation(glu_quadric_obj_,GLU_OUTSIDE);
    gluCylinder(glu_quadric_obj_, radius, radius, cyl_len, kSubdivisions, 1);

    glPopMatrix();
}

void Renderer::DrawCylinder(
        const math::RotMat3d_t &rot_mat,
        const math::Vector3d_t &pos,
        const double height,
        const double radius
        )
{
    glPushMatrix();

    static const int s_lSubdivisions = 18;
 
    double aeHomoXfmMat[glmat4::Size] = {0.0};
    math::ToGlMatrix4(rot_mat, pos, aeHomoXfmMat);
    glMultMatrixd(aeHomoXfmMat);
    glTranslated(0.0, 0.0, -0.5 * height);
    gluCylinder(
            glu_quadric_obj_,     // quad, specifies the quadrics object
            radius,       // base, specifies the radius of the cylinder at z = 0
            radius,       // top, specifies the radius of the cylinder at z = height
            height,       // height, specifies the height of the cylinder
            s_lSubdivisions, // slices, specifies the number of subdivisions around the z axis
            1                // stacks, specifies the number of subdivisions along the z axis
            );

    glPopMatrix();
}

void Renderer::DrawArrow(const geometry::Arrow &arrow)
{
    static const double kEpsilon = 1.0e-4;
    math::Vector3d_t dirVec = arrow.end_pos() - arrow.start_pos();
    if (dirVec.norm() < kEpsilon)
    {
        return;
    }

    math::Vector3d_t unitDirVec = dirVec.normalized();
    glEnable(GL_COLOR_MATERIAL);
    glPushMatrix();
    math::RotMat3d_t rotMatFromAxisZ = math::ComputeRelativeRotMat(
            math::UnitZ(),
            unitDirVec
            );
    double aeToStartPosHomoXfm[glmat4::Size] = {0.0};
    math::ToGlMatrix4(
            rotMatFromAxisZ,
            arrow.start_pos(),
            aeToStartPosHomoXfm
            );
    glMultMatrixd(aeToStartPosHomoXfm);
    gluCylinder(
            glu_quadric_obj_,
            arrow.cylinder_radius(),
            arrow.cylinder_radius(),
            arrow.cylinder_height(),
            geometry::Arrow::DefaultSlices(),
            geometry::Arrow::DefaultStacks()
            );
    glTranslated(0.0, 0.0, arrow.cylinder_height());
    gluCylinder(
            glu_quadric_obj_,
            arrow.cone_radius(),
            0.0,
            arrow.cone_height(),
            geometry::Arrow::DefaultSlices(),
            geometry::Arrow::DefaultStacks()
            );

    math::RotMat3d_t negZRotMat = math::ComputeRelativeRotMat(
            math::UnitZ(),
            -math::UnitZ()
            );
    double aeDiskNormalHomoXfm[glmat4::Size] = {0.0};
    math::ToGlMatrix4(
            negZRotMat,
            math::Vector3d_t::Zero(),
            aeDiskNormalHomoXfm
            );
    glMultMatrixd(aeDiskNormalHomoXfm);
    gluDisk(
            glu_quadric_obj_,                  // qobj, specifies the quadrics object
            0.0,                          // innerRadius, specifies the inner radius (may be 0)
            arrow.cone_radius(),       // outerRadius, specifies the outer radius of the disk
            geometry::Arrow::DefaultSlices(), // slices, specifies the number of subdivisions around the z-axis
            geometry::Arrow::DefaultRings()   // loops, specifies the number of concentric rings about the origin into which the disk is subdivided
            );
    glTranslated(0.0, 0.0, arrow.cylinder_height());
    gluDisk(
            glu_quadric_obj_,
            0.0,
            arrow.cylinder_radius(),
            geometry::Arrow::DefaultSlices(),
            geometry::Arrow::DefaultRings()
            );

    glPopMatrix();
    glDisable(GL_COLOR_MATERIAL);
}

void Renderer::DrawSphere(const math::Vector3d_t &pos, const double radius)
{
    glPushMatrix();
    glTranslated(pos.x(), pos.y(), pos.z());
    gluQuadricDrawStyle(glu_quadric_obj_, (GLenum)GLU_FILL);
    gluQuadricNormals(glu_quadric_obj_, (GLenum)GLU_SMOOTH);
    glScaled(.3, .3, .3);
    gluSphere(glu_quadric_obj_, radius, 25, 25);
    glPopMatrix();
}

void Renderer::DrawTriad()
{
    static const double s_eCylinderHeight = 0.7;
    static const double s_eCylinderRadius = 0.02;
    static const double s_eConeHeight = 0.3;
    static const double s_eConeRadius = 0.04;

    // draw x axis in red, y axis in green, z axis in blue
    glColor3d(1., .2, .2);
    geometry::Arrow arrowX(math::Vector3d_t::Zero(), math::Vector3d_t(1.0, 0.0, 0.0));
    arrowX.set_cylinder_height(s_eCylinderHeight);
    arrowX.set_cylinder_radius(s_eCylinderRadius);
    arrowX.set_cone_height(s_eConeHeight);
    arrowX.set_cone_radius(s_eConeRadius);
    this->DrawArrow(arrowX);

    glColor3d(.2, 1., .2);
    geometry::Arrow arrowY(math::Vector3d_t::Zero(), math::Vector3d_t(0.0, 1.0, 0.0));
    arrowY.set_cylinder_height(s_eCylinderHeight);
    arrowY.set_cylinder_radius(s_eCylinderRadius);
    arrowY.set_cone_height(s_eConeHeight);
    arrowY.set_cone_radius(s_eConeRadius);
    this->DrawArrow(arrowY);

    glColor3d(.2, .2, 1.);
    geometry::Arrow arrowZ(math::Vector3d_t::Zero(), math::Vector3d_t(0.0, 0.0, 1.0));
    arrowZ.set_cylinder_height(s_eCylinderHeight);
    arrowZ.set_cylinder_radius(s_eCylinderRadius);
    arrowZ.set_cone_height(s_eConeHeight);
    arrowZ.set_cone_radius(s_eConeRadius);
    this->DrawArrow(arrowZ);
}

void Renderer::DrawPolyline(const math::Vector3dColl_t &vertex_coll, const bool is_closed)
{
    if (vertex_coll.empty())
    {
        return;
    }

    const int32_t vertex_num = boost::numeric_cast<int32_t>(vertex_coll.size());
    const int32_t array_size = vertex_num * 3;
    std::vector<double> vertex_element_coll;
    vertex_element_coll.reserve(array_size);

    std::for_each(
            vertex_coll.begin(),
            vertex_coll.end(),
            [&](const math::Vector3d_t &vertex)
            {
                vertex_element_coll.push_back(vertex.x());
                vertex_element_coll.push_back(vertex.y());
                vertex_element_coll.push_back(vertex.z());
            }
            );
    glEnableClientState(GL_VERTEX_ARRAY);
    glVertexPointer(
            int32_t{3},
            GL_DOUBLE,
            int32_t{0},
            &vertex_element_coll[0]
            );
    if (is_closed)
    {
        glDrawArrays(
                GL_LINE_LOOP,
                int32_t{0},
                vertex_num
                );
    }
    else
    {
        glDrawArrays(
                GL_LINE_STRIP,
                int32_t{0},
                vertex_num
                );
    }

    glDisableClientState(GL_VERTEX_ARRAY);
}

void Renderer::DrawPointSet(const math::Vector3dColl_t &point_coll)
{
    if (point_coll.empty())
    {
        return;
    }

    const int32_t point_num = boost::numeric_cast<int32_t>(point_coll.size());
    const int32_t array_size = point_num * 3;
    std::vector<double> point_element_coll;
    point_element_coll.reserve(array_size);

    std::for_each(
            point_coll.begin(),
            point_coll.end(),
            [&](const math::Vector3d_t &point)
            {
                point_element_coll.push_back(point.x());
                point_element_coll.push_back(point.y());
                point_element_coll.push_back(point.z());
            }
            );
    glEnableClientState(GL_VERTEX_ARRAY);
    glVertexPointer(
            int32_t{3},
            GL_DOUBLE,
            int32_t{0},
            &point_element_coll[0]
            );

    glDrawArrays(
            GL_POINTS,
            int32_t{0},
            point_num
            );
}

void Renderer::DrawGround()
{
    float i = 0.0, j = 0.0;
    int count = 0;
    
    GLfloat white4[] = {.4f, .4f, .4f, 1.f};
    //GLfloat white1[] = {.1, .1, .1, 1.};
    GLfloat green5[] = {0.f, .5f, 0.f, 1.f};
    //GLfloat green2[] = {0., .2, 0., 1.};
    GLfloat black[] = {0.f, 0.f, 0.f, 1.f};
    //GLfloat mat_shininess[] = {7.};		/* Phong exponent */

    glBegin(GL_QUADS);

    for(i=-15.;i<=15.;i+=1)
    {
        for(j=-15.;j<=15.;j+=1)
        {
            if((count%2) == 0)
            {
                glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT, black);
                glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, white4);
                //			glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, white1);
                //			glMaterialfv(GL_FRONT_AND_BACK, GL_SHININESS, mat_shininess);
                glColor3d(.6, .6, .6);
            }
            else
            {
                glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT, black);
                glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, green5);
                //			glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, green2);
                //			glMaterialfv(GL_FRONT_AND_BACK, GL_SHININESS, mat_shininess);
                glColor3d(.9, .9, .9);
            }

            glNormal3d(0.,0.,1.);

            glVertex3d(j,  0, i);
            glVertex3d(j,  0, i+1);
            glVertex3d(j+1,0, i+1);
            glVertex3d(j+1,0, i);
            count++;
        }
    }

    glEnd();
}

// protected func.

// private func.

} // namespace gui {
