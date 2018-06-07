#include "gui_color.h"
#include <typeinfo>
#include <typeindex>
#include <map>
#include <limits>
#include "boost/numeric/conversion/cast.hpp"

namespace gui {

// public func.

template<typename Scalar>
float Color<Scalar>::ConvertToFloatValue(const int32_t value)
{
    const float output_value
        = boost::numeric_cast<float>(value)
        / boost::numeric_cast<float>(std::numeric_limits<uint8_t>::max());

    return output_value;
}

template<typename Scalar>
Scalar Color<Scalar>::max_intensity()
{
    static const int32_t kFloatMaxIntensity = 1;
    static const int32_t kIntegralMaxIntensity = boost::numeric_cast<int32_t>(
            std::numeric_limits<uint8_t>::max()
            );
    static const std::map<std::type_index, int32_t> kMaxIntensityMap =
    {
        {typeid(uint8_t), kIntegralMaxIntensity},
        {typeid(int32_t), kIntegralMaxIntensity},
        {typeid(float),   kFloatMaxIntensity},
        {typeid(double),  kFloatMaxIntensity},
    };

    auto found = kMaxIntensityMap.find(typeid(Scalar));

    return boost::numeric_cast<Scalar>(found->second);
}

template<typename Scalar>
Color<Scalar>::Color()
    :red_(Scalar{0}),
    green_(Scalar{0}),
    blue_(Scalar{0}),
    alpha_(Color<Scalar>::max_intensity())
{
}

template<typename Scalar>
Color<Scalar>::Color(
        const Scalar red,
        const Scalar green,
        const Scalar blue,
        const Scalar alpha /* = Color<Scalar>::max_intensity() */
        )
    :red_(red),
    green_(green),
    blue_(blue),
    alpha_(alpha)
{
}

template<typename Scalar>
Color<Scalar>::Color(const Color<Scalar> &other)
    :red_(other.red()),
    green_(other.green()),
    blue_(other.blue()),
    alpha_(other.alpha())
{
}

template<typename Scalar>
Color<Scalar>::~Color()
{
}

template<typename Scalar>
Color<Scalar> &Color<Scalar>::operator=(const Color<Scalar> &other)
{
    if (&other == this)
    {
        return *this;
    }

    red_ = other.red();
    green_ = other.green();
    blue_ = other.blue();
    alpha_ = other.alpha();

    return *this;
}

template<typename Scalar>
Scalar Color<Scalar>::red() const
{
    return red_;
}

template<typename Scalar>
Scalar Color<Scalar>::green() const
{
    return green_;
}

template<typename Scalar>
Scalar Color<Scalar>::blue() const
{
    return blue_;
}

template<typename Scalar>
Scalar Color<Scalar>::alpha() const
{
    return alpha_;
}

template<typename Scalar>
void Color<Scalar>::set_red(const Scalar red)
{
    red_ = red;
}

template<typename Scalar>
void Color<Scalar>::set_green(const Scalar green)
{
    green_ = green;
}

template<typename Scalar>
void Color<Scalar>::set_blue(const Scalar blue)
{
    blue_ = blue;
}

template<typename Scalar>
void Color<Scalar>::set_alpha(const Scalar alpha)
{
    alpha_ = alpha;
}

// protected func.

// private func.

// explicit instantiation

template class Color<uint8_t>;
template class Color<int32_t>;
template class Color<float>;
template class Color<double>;

} // namespace gui {
