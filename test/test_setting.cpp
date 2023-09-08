#include <gtest/gtest.h>

#include <iostream>
#include <string>

#include "setting.hpp"


TEST( SettingTest, GetParam )
{
  // 测试获取参数函数
  std::string config_file_path = "/home/oem/Projects/lvio/config/test_config.yaml";
  ASSERT_TRUE( lvio::Setting::getSingleton()->initParamSetting( config_file_path ) );

  // 测试获取整型参数
  int int_param = lvio::Setting::getParam<int>( "int_param" );
  EXPECT_EQ( int_param, 1234 );

  // 测试获取浮点型参数
  float float_param = lvio::Setting::getParam<float>( "float_param" );
  EXPECT_FLOAT_EQ( float_param, 3.14 );

  // 测试获取字符串参数
  std::string string_param = lvio::Setting::getParam<std::string>( "string_param" );
  EXPECT_EQ( string_param, "hello world" );
}

int main( int argc, char **argv )
{
  testing::InitGoogleTest( &argc, argv );
  return RUN_ALL_TESTS();
}