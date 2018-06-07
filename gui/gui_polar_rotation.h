#ifndef _GUI_POLAR_ROTATION_H_
#define _GUI_POLAR_ROTATION_H_

#include "gui_def.h"

namespace gui {

class PolarRotation
{

public:
    /**
     * \brief Default constructor
     */
    PolarRotation();
    /**
     * \brief Constructor
     * \param[in] azimuth
     * \param[in] elevation
     * \param[in] twist
     */
    PolarRotation(
            const double azimuth,
            const double elevation,
            const double twist
            );
    /**
     * \brief Copy constructor
     * \param[in] other
     */
    PolarRotation(const PolarRotation &other);
    /**
     * \brief Destructor
     */
    virtual ~PolarRotation();
    /**
     * \brief Copy assignment operator
     * \param[in] other
     */
    PolarRotation &operator=(const PolarRotation &other);
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
     * \param[in] azimuth
     * \param[in] elevation
     * \param[in] twist
     */
    void set(
            const double azimuth,
            const double elevation,
            const double twist
            );
    /**
     * \brief
     * \param[in] azimuth
     */
    void set_azimuth(const double azimuth);
    /**
     * \brief
     * \param[in] elevation
     */
    void set_elevation(const double elevation);
    /**
     * \brief
     * \param[in] twist
     */
    void set_twist(const double twist);

protected:

private:

    double azimuth_;
    double elevation_;
    double twist_;
};

} // namespace gui {

#endif // #ifndef _GUI_POLAR_ROTATION_H_
