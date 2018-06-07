#include "gui_screenshot.h"
#include <cstdio>
#include <cstring>
#include <vector>
#include <iostream>
#include <iomanip>
#include <sstream>
#include <FL/gl.h>
#include "lodepng.h"

namespace gui {

// public func.

Screenshot::Screenshot()
    :count_(0),
    file_idx_digit_num_(Screenshot::default_file_idx_digit_num()),
    screen_width_(boost::none),
    screen_height_(boost::none),
    screen_color_(new ColorData_t),
    image_color_(new ColorData_t),
    file_name_(new std::string)
{
}

Screenshot::~Screenshot()
{
}

int32_t Screenshot::default_file_idx_digit_num()
{
    static const int32_t kZeroPadDigit = 5;
    return kZeroPadDigit;
}

int32_t Screenshot::width() const
{
    return screen_width_.get();
}

int32_t Screenshot::height() const
{
    return screen_height_.get();
}

int32_t Screenshot::file_idx_digit_num() const
{
    return file_idx_digit_num_;
}

void Screenshot::set_file_name(const std::string &file_name)
{
    *file_name_ = file_name;
}

void Screenshot::set_file_idx_digit_num(const int32_t digit_num)
{
    file_idx_digit_num_ = digit_num;
}

void Screenshot::set_size(const int32_t width, const int32_t height)
{
    screen_width_ = width;
    screen_height_ = height;
    *screen_color_ = ColorData_t(
            screen_width_.get() * screen_height_.get() * this->png_channel_num(),
            static_cast<Intensity_t>(0)
            );
    *image_color_ = *screen_color_;
}

bool Screenshot::Record()
{
    glReadPixels(
            0, 0,
            screen_width_.get(),
            screen_height_.get(),
            GL_RGBA,
            GL_UNSIGNED_BYTE,
            &(*screen_color_)[0]
            );
    this->ToImageColorFormat(
            *screen_color_,
            image_color_.get()
            );

    const std::string image_name = this->EvalFileName();
    bool is_save_png = this->SavePng(
            image_name,
            *image_color_,
            screen_width_.get(),
            screen_height_.get()
            );
    
    if (is_save_png)
    {
        std::cout << "Success in saving " << image_name << std::endl;
    }
    else
    {
        std::cout << "Fail in saving " << image_name << std::endl;
    }

    ++count_;
    return is_save_png;
}

// protected func.

// private func.

int32_t Screenshot::png_channel_num()
{
    static const int32_t kChannelNum = 4;
    return kChannelNum;
}

std::string Screenshot::EvalFileName() const
{
    static const std::string kPngExtension = std::string(".png");
    return *file_name_ + this->EvalZeroPaddedNum(count_) + kPngExtension;
}

std::string Screenshot::EvalZeroPaddedNum(const int32_t num) const
{
    std::ostringstream output_stream;
    output_stream << std::setw(file_idx_digit_num_) << std::setfill('0') << num;
    return output_stream.str();
}

void Screenshot::ToImageColorFormat(
        const Screenshot::ColorData_t &screen_color,
        Screenshot::ColorData_t *image_color
        ) const
{
    for (int32_t row_idx = 0; row_idx < screen_height_.get(); ++row_idx)
    {
        memcpy(
                &(*image_color)[row_idx * screen_width_.get() * this->png_channel_num()],
                &screen_color[(screen_height_.get() - row_idx - 1) * screen_width_.get() * this->png_channel_num()],
                screen_width_.get() * this->png_channel_num()
                );
    }
}

bool Screenshot::SavePng(
        const std::string &file_name,
        const ColorData_t &image_color,
        const int32_t image_width,
        const int32_t image_height
        ) const
{
    uint32_t error_code = lodepng::encode(
            file_name.c_str(),
            image_color,
            image_width,
            image_height
            );

    if (0 != error_code)
    {
        std::cout << "encoder error " << error_code << ": "
            << lodepng_error_text(error_code) << std::endl
            << std::endl;

        return FALSE;
    }

    return TRUE;
}

} // namespace gui {
