add_executable(i2c "i2c_test.c;i2c_testutils.c;dif_i2c.c;dif_i2c_autogen.c;print.c;math.c;memory.c;status.c;qtmatrix-text.cpp;Adafruit_IS31FL3741/Adafruit_IS31FL3741.cpp;SparkFun_BMA400_Arduino_Library/src/SparkFun_BMA400_Arduino_Library.cpp;SparkFun_BMA400_Arduino_Library/src/bma400_api/bma400.c")
target_link_libraries(i2c common)
