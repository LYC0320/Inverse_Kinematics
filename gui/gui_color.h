#ifndef _GUI_COLOR_H_
#define _GUI_COLOR_H_

#include <gui_def.h>

namespace gui {

template<typename Scalar>
class Color final
{

public:

    typedef Scalar Scalar_t;

    static int32_t rgb_bit_num()
    {return int32_t{3};}
    static int32_t rgba_bit_num()
    {return int32_t{4};}

    static float ConvertToFloatValue(const int32_t value);
    static Scalar max_intensity();

    Color();
    explicit Color(
            const Scalar red,
            const Scalar green,
            const Scalar blue,
            const Scalar alpha = Color<Scalar>::max_intensity()
            );
    Color(const Color<Scalar> &other);
    virtual ~Color();

    Color<Scalar> &operator=(const Color<Scalar> &other);

    Scalar red() const;
    Scalar green() const;
    Scalar blue() const;
    Scalar alpha() const;

    void set_red(const Scalar red);
    void set_green(const Scalar green);
    void set_blue(const Scalar blue);
    void set_alpha(const Scalar alpha);

protected:

private:

    Scalar red_;
    Scalar green_;
    Scalar blue_;
    Scalar alpha_;
};

} // namespace gui {

#endif // #ifndef _GUI_COLOR_H_
