#ifndef _GUI_SCREENSHOT_H_
#define _GUI_SCREENSHOT_H_

#include "gui_def.h"
#include <memory>
#include <string>
#include <vector>
#include "boost/optional.hpp"

namespace gui {

class Screenshot final
{

public:

    Screenshot();
    Screenshot(const Screenshot &) = delete;
    virtual ~Screenshot();
    Screenshot &operator=(const Screenshot &) = delete;
    /**
     * \brief Default zero padding digit of screenshot files
     * \return Default value of zero padding digit
     */
    static int32_t default_file_idx_digit_num();
    /**
     * \brief Get width of screenshot
     * \return Image width
     */
    int32_t width() const;
    /**
     * \brief Get height of screenshot
     * \return Image height
     */
    int32_t height() const;
    /**
     * \brief Get file index digit num of screenshot files
     * \return file index digit num
     */
    int32_t file_idx_digit_num() const;
    /**
     * \brief Set the saved file name
     * The file name would be file_name0001.png, file_name0002.png......
     * if no any setting, default name is 0001.png, 0002.png..
     * \param[in] file_name
     */
    void set_file_name(const std::string &file_name);
    /**
     * \brief
     * \param[in] digit_num
     */
    void set_file_idx_digit_num(const int32_t digit_num);
    /**
     * \brief Set the screenshot size
     * \param[in] width
     * \param[in] height
     */
    void set_size(
            const int32_t width,
            const int32_t height
            );
    /**
     * \brief Save the screenshot to PNG file
     * \return Success or failure of screenshot saving
     * \retval TRUE Success to save screenshot according to the given file name
     * \retval FALSE Failure to save screenshot
     */
    bool Record();

protected:

private:

    typedef unsigned char Intensity_t;
    typedef std::vector<Intensity_t> ColorData_t;

    static int32_t png_channel_num();
    std::string EvalFileName() const;
    std::string EvalZeroPaddedNum(const int32_t num) const;
    void ToImageColorFormat(
            const ColorData_t &screen_color,
            ColorData_t *image_color
            ) const;
    bool SavePng(
            const std::string &file_name,
            const ColorData_t &image_color,
            const int32_t image_width,
            const int32_t image_height
            ) const;

    int32_t count_;
    int32_t file_idx_digit_num_;
    boost::optional<int32_t> screen_width_;
    boost::optional<int32_t> screen_height_;
    // Screen color, RGBA
    std::unique_ptr<ColorData_t> screen_color_;
    std::unique_ptr<ColorData_t> image_color_;
    std::unique_ptr<std::string> file_name_;
};

} // namespace gui {

#endif // #ifndef _GUI_SCREENSHOT_H_
