#ifndef _KINEMATICS_ARTIC_IDX_H_
#define _KINEMATICS_ARTIC_IDX_H_

#include "kinematics_def.h"
#include "boost/optional.hpp"
#include "boost/optional/optional_io.hpp"
#include "kinematics_type.h"

namespace kinematics {
/**
 * \brief The articulated index for a articulated figure
 */
class ArticIdx final
{

public:
    /**
     * \brief Constructor
     * \param[in] idx
     */
    explicit ArticIdx(const int32_t idx);
    /**
     * \brief Copy constructor
     * \param[in] other
     */
    ArticIdx(const ArticIdx &other);
    /**
     * \brief Destructor
     */
    virtual ~ArticIdx();
    /**
     * \brief Assignment operator
     */
    ArticIdx &operator=(const ArticIdx &other);
    /**
     * \brief Get the integral articualted index
     * \return The articulated index integer
     */
    operator const int32_t() const;
    /**
     * \brief Get the handle of the parent articulated index.
     * Since the some ArticIdx might have no parent ArticIdx, using optonal as handle to represent
     * no parent index
     * \return The handle of the articulated index
     */
    boost::optional<int32_t> parent_idx() const;
    /**
     * \brief Set the parent articulated index
     * \param[in] parent_idx The target articualted index
     */
    void set_parent_idx(const int32_t parent_idx);
    /**
     * \brief Judge whether the articualted index is root index
     * \retval TRUE, this articualted index is the root index
     * \retval FALSE, this articualted index is NOT a root index
     */
    bool is_root() const;

protected:

private:

    int32_t idx_;
    boost::optional<int32_t> parent_idx_;
};

bool operator==(
        const ArticIdx &artic_idx_1st,
        const ArticIdx &artic_idx_2nd
        );
bool operator!=(
        const ArticIdx &artic_idx_1st,
        const ArticIdx &artic_idx_2nd
        );

template<typename charT, typename traits>
std::basic_ostream<charT, traits> &operator<<(
        std::basic_ostream<charT, traits> &ostream,
        const ArticIdx &artic_idx
        )
{
    ostream << "artic idx: " << static_cast<int32_t>(artic_idx)
        << ", parent idx: " << artic_idx.parent_idx();
    return ostream;
}

} // namespace kinematics {

#endif // #ifndef _KINEMATICS_ARTIC_IDX_H_
