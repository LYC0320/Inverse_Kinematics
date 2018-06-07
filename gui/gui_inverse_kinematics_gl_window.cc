#include "gui_inverse_kinematics_gl_window.h"
#include "FL/gl.h"
#include "FL/glu.h"
#include "FL/glut.H"
#include "boost/numeric/conversion/cast.hpp"
#include "console_log.h"
#include "param_config.h"
#include "math_utils.h"
#include "gui_utils.h"
#include "gui_marker.h"
#include "gui_mouse.h"
#include "gui_signal.h"
#include "gui_screenshot.h"
#include "gui_color.h"
#include "gui_renderer.h"
#include "gui_signal.h"

namespace gui {

InverseKinematicsGlWindow::InverseKinematicsGlWindow(int x, int y, int w, int h, const char *l)
    :Fl_Gl_Window(x, y, w, h, l),
    switch_map_(new gui::SwitchMap_t),
    screenshot_(new gui::Screenshot),
    mouse_(new Mouse),
    ik_marker_(nullptr),
    renderer_(nullptr),
    param_(nullptr),
    signal_(new Signal)
{
}

InverseKinematicsGlWindow::~InverseKinematicsGlWindow()
{
}

void InverseKinematicsGlWindow::set_ik_marker(const std::shared_ptr<Marker> &ik_marker)
{
    ik_marker_ = ik_marker;
}

void InverseKinematicsGlWindow::set_switch(const std::string &id, const bool is_on)
{
    auto found = switch_map_->find(id);
    if (switch_map_->end() == found)
    {
        LOGERR << "invalid id: " << id << std::endl;
        assert(FALSE);
    }

    found->second = is_on;
}

void InverseKinematicsGlWindow::Configure(
        const std::shared_ptr<param::Config> &param,
        const std::shared_ptr<Signal> &signal,
        const std::shared_ptr<Renderer> &renderer
        )
{
    param_ = param;
    renderer_ = renderer;
    signal_ = signal;

    *switch_map_ = SwitchMap_t(
        {
            {"ground",         true},
            {"light",          false},
            {"ik_marker",      false},
            {"skeleton_exist", false},
        }
    );

    screenshot_->set_size(this->w(), this->h());
    screenshot_->set_file_name(param_->value<std::string>("result_picture_path"));
    screenshot_->set_file_idx_digit_num(gui::Screenshot::default_file_idx_digit_num());
}

void InverseKinematicsGlWindow::SaveScreenshot()
{
    screenshot_->Record();
}

bool InverseKinematicsGlWindow::is_switch_on(const std::string &id) const
{
    auto found = switch_map_->find(id);
    if (switch_map_->end() == found)
    {
        LOGERR << "invalid id: " << id << std::endl;
        assert(FALSE);
    }

    return found->second;
}

// protected func.

void InverseKinematicsGlWindow::draw()
{
    //Upon setup of the window (or when Fl_Gl_Window->invalidate is called),
    //the set of functions inside the if block are executed.
    if (!Fl_Gl_Window::valid())
    {
        this->InitGL();
        this->InitLight();
    }

    // Redisplay the screen then put the proper buffer on the screen.
    this->Redisplay();
}

int32_t InverseKinematicsGlWindow::handle(int32_t event)
{
    // retval 0 if the event was not used or understood
    // retval 1 if the event was used and can be deleted
    int32_t used_event = 1;
    static math::ScreenPos_t prev_pos;
    math::ScreenPos_t delta_pos;

    switch (event)
    {
        case FL_RELEASE:
            mouse_->set_pos(
                    math::ScreenPos_t(Fl::event_x(), Fl::event_y())
                    );
            mouse_->set_button(0);
            if (this->is_enabled_selected_ik_marker(ik_marker_))
            {
                ik_marker_->Release();
            }
            break;

        case FL_PUSH:
            mouse_->set_pos(
                    math::ScreenPos_t(Fl::event_x(), Fl::event_y())
                    );
            mouse_->set_button(Fl::event_button());
            if (this->is_enabled_ik_marker(ik_marker_))
            {
                gui::SelectColor(
                        mouse_->pos(),
                        ik_marker_.get()
                        );
                gui::RaySelect(
                        mouse_->pos(),
                        ik_marker_.get()
                        );
            }
            break;

        case FL_DRAG:
            mouse_->set_pos(
                    math::ScreenPos_t(Fl::event_x(), Fl::event_y())
                    );
            delta_pos = mouse_->pos() - prev_pos;

            if (this->is_selected_ik_marker(ik_marker_))
            {
                gui::DragMarker(
                        mouse_->pos(),
                        prev_pos,
                        mouse_->button(),
                        ik_marker_.get()
                        );
            }
            else
            {
                this->ProcessMouseDrag(
                        event,
                        delta_pos
                        );
            }
            break;

        case FL_KEYBOARD:
            switch (Fl::event_key())
            {
                case 'q':
                case 'Q':
                case 65307:
                    exit(0);
                default:
                    break;
            }
            break;

        default:
            // pass other events to the base class...
            used_event = Fl_Gl_Window::handle(event);
            break;
    }

    prev_pos = mouse_->pos();
    //used_event = Fl_Gl_Window::handle(event);
    this->redraw();

    return used_event;  // Returning one acknowledges that we handled this event
}

// private func.

bool InverseKinematicsGlWindow::is_selected_ik_marker(
        const std::shared_ptr<Marker> &marker
        ) const
{
    if (!marker)
    {
        return false;
    }

    if (!marker->is_select())
    {
        return false;
    }

    return true;
}

bool InverseKinematicsGlWindow::is_enabled_ik_marker(
        const std::shared_ptr<Marker> &marker
        ) const
{
    if (!marker)
    {
        return false;
    }

    return this->is_switch_on("ik_marker");
}

bool InverseKinematicsGlWindow::is_enabled_selected_ik_marker(
        const std::shared_ptr<Marker> &marker
        ) const
{
    if (!marker)
    {
        return false;
    }

    if (!this->is_switch_on("ik_marker"))
    {
        return false;
    }

    if (!marker->is_select())
    {
        return false;
    }

    return true;
}

void InverseKinematicsGlWindow::Redisplay()
{
    if (this->is_switch_on("light"))
    {
        glEnable(GL_LIGHTING);
    }
    else
    {
        glDisable(GL_LIGHTING);
    }

    glClearColor(.2f, .2f, .2f, 0.0f);
    glClear(GL_DEPTH_BUFFER_BIT | GL_COLOR_BUFFER_BIT); /* clear image, zbuf */

    glPushMatrix();			/* save current transform matrix */

    renderer_->ApplyCameraView();

    glLineWidth(2.);		/* we'll draw background with thick lines */

    if (this->is_switch_on("ground"))
    {
        renderer_->DrawTriad(); // draw a triad in the origin of the world coord
        renderer_->DrawGround();
    }

    if (this->is_enabled_ik_marker(ik_marker_))
    {
        renderer_->set_color(ik_marker_->color());
        renderer_->DrawSphere(
                ik_marker_->target_pos(),
                param_->value<double>("ik.marker_radius")
                );
    }

    if (this->is_switch_on("skeleton_exist"))
    {
        signal_->DisplayShow();
    }

    glPopMatrix();
}

void InverseKinematicsGlWindow::InitGL()
{
    int red_bits, green_bits, blue_bits;
    struct {GLint x, y, width, height;} viewport;
    glEnable(GL_DEPTH_TEST);	/* turn on z-buffer */

    glGetIntegerv(GL_RED_BITS, &red_bits);
    glGetIntegerv(GL_GREEN_BITS, &green_bits);
    glGetIntegerv(GL_BLUE_BITS, &blue_bits);
    glGetIntegerv(GL_VIEWPORT, &viewport.x);

    LOGMSG << "OpenGL window has "
        << red_bits << " bits red, "
        << green_bits << " green, "
        << blue_bits << " blue; viewport is "
        << viewport.width << "x"
        << viewport.height << std::endl;

    /* setup perspective camera with OpenGL */
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    double width_to_height_ratio
        = boost::numeric_cast<double>(viewport.width)
        / boost::numeric_cast<double>(viewport.height);
    gluPerspective(
            45.,                // vertical field of view
            width_to_height_ratio, // aspect ratio
            .1,                 // znear
            50.                 // zfar
            );

    /* from here on we're setting modeling transformations */
    glMatrixMode(GL_MODELVIEW);

    // Move away from center
    glTranslatef(0., 0., -5.);

    renderer_->set_camera_zoom(1.0);
    double azimuth = -25.0;
    double elevation = -15.0;
    double twist = 0.0;
    renderer_->set_camera_rotation(azimuth, elevation, twist);
    renderer_->set_camera_look_at(math::Vector3d_t::Zero());
}

void InverseKinematicsGlWindow::InitLight()
{
    // set up OpenGL to do lighting
    // we've set up three lights

    //set material properties
    GLfloat white8[] = {.8f, .8f, .8f, 1.f};
    GLfloat white2[] = {.2f, .2f, .2f, 1.f};
    //GLfloat black[] = {0., 0., 0., 1.};
    GLfloat mat_shininess[] = {50.f};		// Phong exponent

    GLfloat light0_position[] = {-25.f, 25.f, 25.f, 0.f};  //directional light (w=0)
    GLfloat white[] = {11.f, 11.f, 11.f, 5.f};

    GLfloat light1_position[] = {-25.f, 25.f, -25.f, 0.f};
    GLfloat red[] = {1.f, .3f, .3f, 5.f};

    GLfloat light2_position[] = {25.f, 25.f, -5.f, 0.f};
    GLfloat blue[] = {.3f, .4f, 1.f, 25.f};

    glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT, white2);	// no ambient
    glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, white8);
    glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, white2);
    glMaterialfv(GL_FRONT_AND_BACK, GL_SHININESS, mat_shininess);

    // set up several lights
    // one white light for the front, red and blue lights for the left & top

    glLightfv(GL_LIGHT0, GL_POSITION, light0_position);
    glLightfv(GL_LIGHT0, GL_DIFFUSE, white);
    glLightfv(GL_LIGHT0, GL_SPECULAR, white);
    glEnable(GL_LIGHT0);

    glLightfv(GL_LIGHT1, GL_POSITION, light1_position);
    glLightfv(GL_LIGHT1, GL_DIFFUSE, red);
    glLightfv(GL_LIGHT1, GL_SPECULAR, red);
    glEnable(GL_LIGHT1);

    glLightfv(GL_LIGHT2, GL_POSITION, light2_position);
    glLightfv(GL_LIGHT2, GL_DIFFUSE, blue);
    glLightfv(GL_LIGHT2, GL_SPECULAR, blue);
    glEnable(GL_LIGHT2);

    GLfloat light3_position[] = {0.f, -25.f, 0.f, 0.6f};
    glLightfv(GL_LIGHT3, GL_POSITION, light3_position);
    glLightfv(GL_LIGHT3, GL_DIFFUSE, white);
    glLightfv(GL_LIGHT3, GL_SPECULAR, white);
    glEnable(GL_LIGHT3);

    glEnable(GL_NORMALIZE);	// normalize normal vectors
    glLightModeli(GL_LIGHT_MODEL_TWO_SIDE, GL_TRUE);	// two-sided lighting

    // do the following when you want to turn on lighting
    if (this->is_switch_on("light"))
    {
        glEnable(GL_LIGHTING);
    }
    else
    {
        glDisable(GL_LIGHTING);
    }
}

void InverseKinematicsGlWindow::ProcessMouseDrag(
        const int32_t event,
        const math::ScreenPos_t &delta_pos
        )
{
    if (FL_LEFT_MOUSE == mouse_->button())
    {
        if (abs(delta_pos.x()) > abs(delta_pos.y()))
        {
            renderer_->ShiftCameraRotation(
                    boost::numeric_cast<double>(delta_pos.x()), // azimuth
                    double{0.0},                                // elevation
                    double{0.0}                                 // twist
                    );
        }
        else
        {
            renderer_->ShiftCameraRotation(
                    double{0.0},                                 // azimuth
                    -boost::numeric_cast<double>(delta_pos.y()), // elevation
                    double{0.0}                                  // twist
                    );
        }
    }
    else if (FL_MIDDLE_MOUSE == mouse_->button())
    {
        if(abs(delta_pos.y()) > abs(delta_pos.x()))
        {
            static const double kDeltaScale = double{0.01};
            const double scale
                = double{1.0}
            + boost::numeric_cast<double>(delta_pos.y()) * kDeltaScale;
            glScaled(scale, scale, scale);
        }
    }
    else if (FL_RIGHT_MOUSE == mouse_->button())
    {
        static const double kDeltaScale = 0.1;
        double rot_radian = renderer_->camera_rotation().azimuth() * math::ToRadian<double>();
        double rot_scale_xy = boost::numeric_cast<double>(delta_pos.x()) * kDeltaScale;
        renderer_->ShiftCameraTranslation(
                math::Vector3d_t(
                    cos(rot_radian) * rot_scale_xy,
                    -boost::numeric_cast<double>(delta_pos.y()) * kDeltaScale,
                    sin(rot_radian) * rot_scale_xy
                    )
                );

        renderer_->set_camera_look_at(-renderer_->camera_translation());
    }
}

} // namespace gui {

