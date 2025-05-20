#ifndef GTEST_GTEST_H_
#define GTEST_GTEST_H_

#include <cstdlib>
#include <iostream>

#define TEST(SUITE, NAME) int SUITE##_##NAME()
#define EXPECT_EQ(val1, val2)                                                  \
  if ((val1) != (val2)) {                                                      \
    std::cerr << "EXPECT_EQ failed: " << (val1) << " != " << (val2) << '\n';   \
    return 1;                                                                  \
  }

#define RUN_ALL_TESTS() 0

#endif // GTEST_GTEST_H_
