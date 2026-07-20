add_test([=[LinkCodec.WireBitExact]=]  C:/Users/Rakete/Documents/Drohnenversuchsstand/DROMA/Simulation/scripts/sitl/build_cg/Release/test_link_codec.exe [==[--gtest_filter=LinkCodec.WireBitExact]==] --gtest_also_run_disabled_tests)
set_tests_properties([=[LinkCodec.WireBitExact]=]
  PROPERTIES
    
    DEF_SOURCE_LINE [==[C:\Users\Rakete\Documents\Drohnenversuchsstand\DROMA\Simulation\scripts\sitl\test\test_link_codec.cpp:48]==]
    WORKING_DIRECTORY [==[C:/Users/Rakete/Documents/Drohnenversuchsstand/DROMA/Simulation/scripts/sitl/build_cg]==]
    SKIP_REGULAR_EXPRESSION [==[\[  SKIPPED \]]==]
    
)
add_test([=[LinkCodec.DecodeMatchesRx]=]  C:/Users/Rakete/Documents/Drohnenversuchsstand/DROMA/Simulation/scripts/sitl/build_cg/Release/test_link_codec.exe [==[--gtest_filter=LinkCodec.DecodeMatchesRx]==] --gtest_also_run_disabled_tests)
set_tests_properties([=[LinkCodec.DecodeMatchesRx]=]
  PROPERTIES
    
    DEF_SOURCE_LINE [==[C:\Users\Rakete\Documents\Drohnenversuchsstand\DROMA\Simulation\scripts\sitl\test\test_link_codec.cpp:82]==]
    WORKING_DIRECTORY [==[C:/Users/Rakete/Documents/Drohnenversuchsstand/DROMA/Simulation/scripts/sitl/build_cg]==]
    SKIP_REGULAR_EXPRESSION [==[\[  SKIPPED \]]==]
    
)
add_test([=[LinkCodec.HeaderRoundTrip]=]  C:/Users/Rakete/Documents/Drohnenversuchsstand/DROMA/Simulation/scripts/sitl/build_cg/Release/test_link_codec.exe [==[--gtest_filter=LinkCodec.HeaderRoundTrip]==] --gtest_also_run_disabled_tests)
set_tests_properties([=[LinkCodec.HeaderRoundTrip]=]
  PROPERTIES
    
    DEF_SOURCE_LINE [==[C:\Users\Rakete\Documents\Drohnenversuchsstand\DROMA\Simulation\scripts\sitl\test\test_link_codec.cpp:120]==]
    WORKING_DIRECTORY [==[C:/Users/Rakete/Documents/Drohnenversuchsstand/DROMA/Simulation/scripts/sitl/build_cg]==]
    SKIP_REGULAR_EXPRESSION [==[\[  SKIPPED \]]==]
    
)
set(test_link_codec_TESTS [==[LinkCodec.WireBitExact]==] [==[LinkCodec.DecodeMatchesRx]==] [==[LinkCodec.HeaderRoundTrip]==])
