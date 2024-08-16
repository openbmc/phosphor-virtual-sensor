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

#include <cmath>
#include <limits>
#include <numeric>

/* For floating types. (float, double, long double et al) */
template <typename T>
struct FuncMaxIgnoreNaN : public exprtk::ivararg_function<T>
{
    FuncMaxIgnoreNaN()
    {
        exprtk::set_min_num_args(*this, 2);
        exprtk::set_max_num_args(*this, 255);
    }

    inline T operator()(const std::vector<T>& argList)
    {
        return std::reduce(std::begin(argList), std::end(argList),
                           std::numeric_limits<double>::quiet_NaN(),
                           [](auto a, auto b) {
                               if (std::isnan(b))
                               {
                                   return a;
                               }
                               if (std::isnan(a))
                               {
                                   return b;
                               }
                               return std::max(a, b);
                           });
    }
};

template <typename T>
struct FuncSumIgnoreNaN : public exprtk::ivararg_function<T>
{
    inline T operator()(const std::vector<T>& argList)
    {
        return std::reduce(std::begin(argList), std::end(argList),
                           std::numeric_limits<double>::quiet_NaN(),
                           [](auto a, auto b) {
                               if (std::isnan(b))
                               {
                                   return a;
                               }
                               if (std::isnan(a))
                               {
                                   return b;
                               }
                               return a + b;
                           });
    }
};

template <typename T>
struct FuncIfNan : public exprtk::ifunction<T>
{
    using exprtk::ifunction<T>::operator();

    FuncIfNan() : exprtk::ifunction<T>(2) {}

    inline T operator()(const T& arg1, const T& arg2)
    {
        if (std::isnan(arg1))
        {
            return arg2;
        }
        else
        {
            return arg1;
        }
    }
};
