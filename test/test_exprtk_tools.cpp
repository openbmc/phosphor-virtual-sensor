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

TEST(TestExprtkTools, inrange)
{
    double a = 1.0;
    double b = 2.0;
    double c = 3.0;
    FuncInrangeIgnoreNaN<double> funcInrangeIgnoreNaN;

    EXPECT_TRUE(funcInrangeIgnoreNaN(a, b, c));

    b = 5.0;
    EXPECT_FALSE(funcInrangeIgnoreNaN(0.0, 1.5, 1.0));

    b = std::numeric_limits<double>::quiet_NaN();
    EXPECT_FALSE(funcInrangeIgnoreNaN(a, b, c));

    a = std::numeric_limits<double>::quiet_NaN();
    b = 2.0;
    EXPECT_FALSE(funcInrangeIgnoreNaN(a, b, c));

    a = 1.0;
    c = std::numeric_limits<double>::quiet_NaN();
    EXPECT_FALSE(funcInrangeIgnoreNaN(a, b, c));
}
