/*
 * This define will disable the ability for expressions to have comments.
 * Expressions that have comments when parsed with a build that has this
 * option, will result in a compilation failure.
 */
// #define exprtk_disable_comments
/*
 * This define will disable the loop-wise 'break' and 'continue'
 * capabilities. Any expression that contains those keywords will result
 * in a compilation failure.
 */
#define exprtk_disable_break_continue
/*
 * This define will disable the short-circuit '&' (and) and '|' (or)
 * operators
 */
#define exprtk_disable_sc_andor
/*
 * This define will disable all enhanced features such as strength
 * reduction and special function optimisations and expression specific
 * type instantiations. This feature will reduce compilation times and
 * binary sizes but will also result in massive performance degradation
 * of expression evaluations.
 */
#define exprtk_disable_enhanced_features
/*
 * This define will disable all string processing capabilities. Any
 * expression that contains a string or string related syntax will result
 * in a compilation failure.
 */
#define exprtk_disable_string_capabilities

#define exprtk_disable_rtl_io_file
#define exprtk_disable_return_statement
#define exprtk_disable_rtl_io
#define exprtk_disable_superscalar_unroll

/* include main exprtk header library */
#include <exprtk.hpp>

/* For floating types. (float, double, long double et al) */
template <typename T>
struct FuncMaxIgnoreNaN : public exprtk::ivararg_function<T>
{
    FuncMaxIgnoreNaN()
    {
        exprtk::set_min_num_args(*this, 2);
        exprtk::set_max_num_args(*this, 255);
    }

    inline T operator()(const std::vector<T>& arglist)
    {
        T result = std::numeric_limits<double>::quiet_NaN();
        for (std::size_t i = 0; i < arglist.size(); ++i)
        {
            if (!std::isnan(arglist[i]))
            {
                if (!std::isnan(result))
                {
                    if (result < arglist[i])
                    {
                        result = arglist[i];
                    }
                }
                else
                {
                    result = arglist[i];
                }
            }
        }
        return result;
    }
};

template <typename T>
struct FuncSumIgnoreNaN : public exprtk::ivararg_function<T>
{
    inline T operator()(const std::vector<T>& argList)
    {
        T result = std::numeric_limits<double>::quiet_NaN();
        for (std::size_t i = 0; i < argList.size(); ++i)
        {
            if (!std::isnan(argList[i]))
            {
                if (!std::isnan(result))
                {
                    result += argList[i];
                }
                else
                {
                    result = argList[i];
                }
            }
        }
        return result;
    }
};
