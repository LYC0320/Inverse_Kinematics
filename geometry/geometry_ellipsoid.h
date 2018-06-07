#ifndef _GEOMETRY_ELLIPSOID_H_
#define _GEOMETRY_ELLIPSOID_H_

#include "geometry_def.h"
#include <memory>
#include "math_type.h"

namespace geometry {

class Ellipsoid final
{

public:

    Ellipsoid();
    explicit Ellipsoid(const math::Vector3d_t &size);
    explicit Ellipsoid(const math::HomoXfm3d_t &xfm, const math::Vector3d_t &size);
    Ellipsoid(const Ellipsoid &other);
    virtual ~Ellipsoid();
    Ellipsoid &operator=(const Ellipsoid &other);

    math::HomoXfm3d_t xfm() const;
    math::Vector3d_t size() const;

    void set_xfm(const math::HomoXfm3d_t &xfm);
    void set_size(const math::Vector3d_t &size);

protected:

private:

    std::unique_ptr<math::HomoXfm3d_t> xfm_;
    std::unique_ptr<math::Vector3d_t> size_;
};

} // namespace geometry {

#endif // #ifndef _GEOMETRY_ELLIPSOID_H_
