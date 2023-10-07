#include "exprtkTools.hpp"

#include <gmock/gmock.h>
#include <gtest/gtest.h>

TEST(TestExprtkTools, max)
{
    std::vector<double> v(3, 0.1);
    FuncMaxIgnoreNaN<double> funcMaxIgnoreNaN;
    EXPECT_DOUBLE_EQ(funcMaxIgnoreNaN(v), 0.1);

    v[0] = std::numeric_limits<double>::quiet_NaN();
    EXPECT_DOUBLE_EQ(funcMaxIgnoreNaN(v), 0.1);

    v[1] = std::numeric_limits<double>::quiet_NaN();
    EXPECT_DOUBLE_EQ(funcMaxIgnoreNaN(v), 0.1);

    v[2] = std::numeric_limits<double>::quiet_NaN();
    EXPECT_TRUE(std::isnan(funcMaxIgnoreNaN(v)));
}

TEST(TestExprtkTools, sum)
{
    std::vector<double> v(3, 0.1);
    FuncSumIgnoreNaN<double> funcSumIgnoreNaN;
    EXPECT_DOUBLE_EQ(funcSumIgnoreNaN(v), 0.3);

    v[0] = std::numeric_limits<double>::quiet_NaN();
    EXPECT_DOUBLE_EQ(funcSumIgnoreNaN(v), 0.2);

    v[1] = std::numeric_limits<double>::quiet_NaN();
    EXPECT_DOUBLE_EQ(funcSumIgnoreNaN(v), 0.1);

    v[2] = std::numeric_limits<double>::quiet_NaN();
    EXPECT_TRUE(std::isnan(funcSumIgnoreNaN(v)));
}

TEST(TestExprtkTools, ifNan)
{
    double a = 1.0;
    double b = 2.0;

    FuncIfNan<double> funcIfNan;
    EXPECT_DOUBLE_EQ(funcIfNan(a, b), 1.0);

    a = std::numeric_limits<double>::quiet_NaN();
    EXPECT_DOUBLE_EQ(funcIfNan(a, b), 2.0);

    b = std::numeric_limits<double>::quiet_NaN();
    EXPECT_TRUE(std::isnan(funcIfNan(a, b)));

    a = 1.0;
    EXPECT_DOUBLE_EQ(funcIfNan(a, b), 1.0);
}
