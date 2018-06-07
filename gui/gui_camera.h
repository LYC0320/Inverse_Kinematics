#ifndef _GUI_CAMERA_H_
#define _GUI_CAMERA_H_

#include "gui_def.h"
#include <memory>
#include "math_type.h"
#include "gui_fwd.h"

namespace gui {

class PolarRotation;

class Camera final
{

public:
    /**
     * \brief Default constructor
     */
    Camera();
    /**
     * \breif Copy constructor
     * \param[in] other
     */
    Camera(const Camera &other);
    /**
     * \brief Destructor
     */
    virtual ~Camera();
    /**
     * \brief Assignment operator
     * \param[in]
     */
    Camera &operator=(const Camera &other);
    /**
     * \brief
     * \return
     */
    double zoom() const;
    /**
     * \brief
     * \return
     */
    double azimuth() const;
    /**
     * \brief
     * \return
     */
    double elevation() const;
    /**
     * \brief
     * \return
     */
    double twist() const;
    /**
     * \brief
     * \return
     */
    PolarRotation polar_rotation() const;
    /**
     * \brief
     * \return
     */
    math::Vector3d_t translation() const;
    /**
     * \brief
     * \return
     */
    math::Vector3d_t look_at_pos() const;
    /**
     * \brief
     * \param[in]
     */
    void set_zoom(const double zoom);
    /**
     * \brief
     * \param[in]
     */
    void set_polar_rotation(const PolarRotation &polar_rotation);
    /**
     * \brief
     * \param[in]
     */
    void set_translation(const math::Vector3d_t &translation);
    /**
     * \brief
     * \param[in]
     */
    void set_look_at_pos(const math::Vector3d_t &look_at_pos);
    /**
     * \brief
     * \param[in]
     */
    void ShiftRotation(
            const double azimuth,
            const double elevation,
            const double twist
            );
    /**
     * \brief
     * \param[in]
     */
    void ShiftRotation(const PolarRotation &polar_rotation);
    /**
     * \brief
     * \param[in]
     */
    void ShiftTranslation(const math::Vector3d_t &shift_translation);
    /**
     * \brief
     * \param[in]
     */
    void ShiftLookAt(const math::Vector3d_t &shift_pos);

protected:

private:

    double zoom_;
    std::unique_ptr<PolarRotation> polar_rotation_;
    std::unique_ptr<math::Vector3d_t> translation_;
    std::unique_ptr<math::Vector3d_t> look_at_pos_;
};

} // namespace gui {

#endif // #ifndef _GUI_CAMERA_H_
