add_test([=[QuatGolden.Dcm2Quat_MatchesGolden]=]  C:/Users/Rakete/Documents/Drohnenversuchsstand/DROMA/Simulation/scripts/sitl/build_cg/Release/test_quat_golden.exe [==[--gtest_filter=QuatGolden.Dcm2Quat_MatchesGolden]==] --gtest_also_run_disabled_tests)
set_tests_properties([=[QuatGolden.Dcm2Quat_MatchesGolden]=]
  PROPERTIES
    
    DEF_SOURCE_LINE [==[C:\Users\Rakete\Documents\Drohnenversuchsstand\DROMA\Simulation\scripts\sitl\test\test_quat_golden.cpp:35]==]
    WORKING_DIRECTORY [==[C:/Users/Rakete/Documents/Drohnenversuchsstand/DROMA/Simulation/scripts/sitl/build_cg]==]
    SKIP_REGULAR_EXPRESSION [==[\[  SKIPPED \]]==]
    
)
add_test([=[QuatGolden.Quat2Dcm_MatchesGolden]=]  C:/Users/Rakete/Documents/Drohnenversuchsstand/DROMA/Simulation/scripts/sitl/build_cg/Release/test_quat_golden.exe [==[--gtest_filter=QuatGolden.Quat2Dcm_MatchesGolden]==] --gtest_also_run_disabled_tests)
set_tests_properties([=[QuatGolden.Quat2Dcm_MatchesGolden]=]
  PROPERTIES
    
    DEF_SOURCE_LINE [==[C:\Users\Rakete\Documents\Drohnenversuchsstand\DROMA\Simulation\scripts\sitl\test\test_quat_golden.cpp:62]==]
    WORKING_DIRECTORY [==[C:/Users/Rakete/Documents/Drohnenversuchsstand/DROMA/Simulation/scripts/sitl/build_cg]==]
    SKIP_REGULAR_EXPRESSION [==[\[  SKIPPED \]]==]
    
)
add_test([=[QuatGolden.QuatMul_MatchesGolden]=]  C:/Users/Rakete/Documents/Drohnenversuchsstand/DROMA/Simulation/scripts/sitl/build_cg/Release/test_quat_golden.exe [==[--gtest_filter=QuatGolden.QuatMul_MatchesGolden]==] --gtest_also_run_disabled_tests)
set_tests_properties([=[QuatGolden.QuatMul_MatchesGolden]=]
  PROPERTIES
    
    DEF_SOURCE_LINE [==[C:\Users\Rakete\Documents\Drohnenversuchsstand\DROMA\Simulation\scripts\sitl\test\test_quat_golden.cpp:75]==]
    WORKING_DIRECTORY [==[C:/Users/Rakete/Documents/Drohnenversuchsstand/DROMA/Simulation/scripts/sitl/build_cg]==]
    SKIP_REGULAR_EXPRESSION [==[\[  SKIPPED \]]==]
    
)
add_test([=[QuatGolden.QuatConj_MatchesGolden]=]  C:/Users/Rakete/Documents/Drohnenversuchsstand/DROMA/Simulation/scripts/sitl/build_cg/Release/test_quat_golden.exe [==[--gtest_filter=QuatGolden.QuatConj_MatchesGolden]==] --gtest_also_run_disabled_tests)
set_tests_properties([=[QuatGolden.QuatConj_MatchesGolden]=]
  PROPERTIES
    
    DEF_SOURCE_LINE [==[C:\Users\Rakete\Documents\Drohnenversuchsstand\DROMA\Simulation\scripts\sitl\test\test_quat_golden.cpp:91]==]
    WORKING_DIRECTORY [==[C:/Users/Rakete/Documents/Drohnenversuchsstand/DROMA/Simulation/scripts/sitl/build_cg]==]
    SKIP_REGULAR_EXPRESSION [==[\[  SKIPPED \]]==]
    
)
add_test([=[QuatGolden.QuatRotate_MatchesGolden]=]  C:/Users/Rakete/Documents/Drohnenversuchsstand/DROMA/Simulation/scripts/sitl/build_cg/Release/test_quat_golden.exe [==[--gtest_filter=QuatGolden.QuatRotate_MatchesGolden]==] --gtest_also_run_disabled_tests)
set_tests_properties([=[QuatGolden.QuatRotate_MatchesGolden]=]
  PROPERTIES
    
    DEF_SOURCE_LINE [==[C:\Users\Rakete\Documents\Drohnenversuchsstand\DROMA\Simulation\scripts\sitl\test\test_quat_golden.cpp:106]==]
    WORKING_DIRECTORY [==[C:/Users/Rakete/Documents/Drohnenversuchsstand/DROMA/Simulation/scripts/sitl/build_cg]==]
    SKIP_REGULAR_EXPRESSION [==[\[  SKIPPED \]]==]
    
)
add_test([=[QuatProps.RoundTripsAndIdentities]=]  C:/Users/Rakete/Documents/Drohnenversuchsstand/DROMA/Simulation/scripts/sitl/build_cg/Release/test_quat_golden.exe [==[--gtest_filter=QuatProps.RoundTripsAndIdentities]==] --gtest_also_run_disabled_tests)
set_tests_properties([=[QuatProps.RoundTripsAndIdentities]=]
  PROPERTIES
    
    DEF_SOURCE_LINE [==[C:\Users\Rakete\Documents\Drohnenversuchsstand\DROMA\Simulation\scripts\sitl\test\test_quat_golden.cpp:123]==]
    WORKING_DIRECTORY [==[C:/Users/Rakete/Documents/Drohnenversuchsstand/DROMA/Simulation/scripts/sitl/build_cg]==]
    SKIP_REGULAR_EXPRESSION [==[\[  SKIPPED \]]==]
    
)
set(test_quat_golden_TESTS [==[QuatGolden.Dcm2Quat_MatchesGolden]==] [==[QuatGolden.Quat2Dcm_MatchesGolden]==] [==[QuatGolden.QuatMul_MatchesGolden]==] [==[QuatGolden.QuatConj_MatchesGolden]==] [==[QuatGolden.QuatRotate_MatchesGolden]==] [==[QuatProps.RoundTripsAndIdentities]==])
