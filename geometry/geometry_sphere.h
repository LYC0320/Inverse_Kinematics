#ifndef _GEOMETRY_SPHERE_H_
#define _GEOMETRY_SPHERE_H_

#include "geometry_def.h"
#include <memory>
#include "math_type.h"

namespace geometry {

class Sphere final
{

public:

    Sphere();
    explicit Sphere(const double radius);
    explicit Sphere(const math::HomoXfm3d_t &xfm, const double radius);
    Sphere(const Sphere &other);
    virtual ~Sphere();
    Sphere &operator=(const Sphere &other);

    math::HomoXfm3d_t xfm() const;
    double radius() const;

    void set_xfm(const math::HomoXfm3d_t &xfm);
    void set_radius(const double radius);

protected:

private:

    std::unique_ptr<math::HomoXfm3d_t> xfm_;
    double radius_;
};

} // namespace geometry {

#endif // #ifndef _GEOMETRY_SPHERE_H_
