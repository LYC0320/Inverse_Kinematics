#ifndef _GUI_SKELETON_COLOR_H_
#define _GUI_SKELETON_COLOR_H_

#include "gui_def.h"
#include <memory>
#include "gui_type.h"

namespace gui {

class SkeletonColor final
{

public:

    SkeletonColor();
    SkeletonColor(const SkeletonColor &other);
    virtual ~SkeletonColor();
    SkeletonColor &operator=(const SkeletonColor &other);

    Color_t operator()(const int32_t skeleton_idx) const;

    ColorColl_t color_coll() const;

    void Configure(const ColorColl_t &color_coll_);

protected:

private:

    std::unique_ptr<ColorColl_t> color_coll_;
};

} // namespace gui {

#endif // #ifndef _GUI_SKELETON_COLOR_H_
