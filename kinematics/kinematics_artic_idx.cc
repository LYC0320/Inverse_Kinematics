#include "kinematics_artic_idx.h"

namespace kinematics {

// public func.

ArticIdx::ArticIdx(const int32_t idx)
    :idx_(idx),
    parent_idx_(boost::none)
{
}

ArticIdx::ArticIdx(const ArticIdx &other)
    :idx_(other),
    parent_idx_(other.parent_idx())
{
}

ArticIdx::~ArticIdx()
{
}

ArticIdx &ArticIdx::operator=(const ArticIdx &other)
{
    if (&other == this)
    {
        return *this;
    }

    idx_ = other;
    parent_idx_ = other.parent_idx();

    return *this;
}

ArticIdx::operator const int32_t() const
{
    return idx_;
}

boost::optional<int32_t> ArticIdx::parent_idx() const
{
    return parent_idx_;
}

void ArticIdx::set_parent_idx(const int32_t parent_idx)
{
    parent_idx_ = parent_idx;
}

bool ArticIdx::is_root() const
{
    return !parent_idx_.is_initialized();
}

// protected func.

// private func.

// non-member func.

bool operator==(
        const ArticIdx &artic_idx_1st,
        const ArticIdx &artic_idx_2nd
        )
{
    const int32_t target_idx_1st = artic_idx_1st;
    const int32_t target_idx_2nd = artic_idx_2nd;
    if (target_idx_1st != target_idx_2nd)
    {
        return FALSE;
    }

    if (artic_idx_1st.parent_idx().is_initialized() != artic_idx_2nd.parent_idx().is_initialized())
    {
        return FALSE;
    }

    if (artic_idx_1st.parent_idx().is_initialized())
    {
        //if (artic_idx_1st.ParentIdx() != artic_idx_2nd.ParentIdx())
        if (artic_idx_1st.parent_idx().get() != artic_idx_2nd.parent_idx().get())
        {
            return FALSE;
        }
    }

    return TRUE;
}

bool operator!=(
        const ArticIdx &artic_idx_1st,
        const ArticIdx &artic_idx_2nd
        )
{
    return !(artic_idx_1st == artic_idx_2nd);
}

} // namespace kinematics {
