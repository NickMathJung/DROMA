add_test([=[GcsFrame.ParseMatchesGolden]=]  C:/Users/Rakete/Documents/Drohnenversuchsstand/DROMA/Simulation/scripts/sitl/build_cg/Release/test_gcs_frame.exe [==[--gtest_filter=GcsFrame.ParseMatchesGolden]==] --gtest_also_run_disabled_tests)
set_tests_properties([=[GcsFrame.ParseMatchesGolden]=]
  PROPERTIES
    
    DEF_SOURCE_LINE [==[C:\Users\Rakete\Documents\Drohnenversuchsstand\DROMA\Simulation\scripts\sitl\test\test_gcs_frame.cpp:30]==]
    WORKING_DIRECTORY [==[C:/Users/Rakete/Documents/Drohnenversuchsstand/DROMA/Simulation/scripts/sitl/build_cg]==]
    SKIP_REGULAR_EXPRESSION [==[\[  SKIPPED \]]==]
    
)
add_test([=[GcsFrame.RejectsCorruption]=]  C:/Users/Rakete/Documents/Drohnenversuchsstand/DROMA/Simulation/scripts/sitl/build_cg/Release/test_gcs_frame.exe [==[--gtest_filter=GcsFrame.RejectsCorruption]==] --gtest_also_run_disabled_tests)
set_tests_properties([=[GcsFrame.RejectsCorruption]=]
  PROPERTIES
    
    DEF_SOURCE_LINE [==[C:\Users\Rakete\Documents\Drohnenversuchsstand\DROMA\Simulation\scripts\sitl\test\test_gcs_frame.cpp:54]==]
    WORKING_DIRECTORY [==[C:/Users/Rakete/Documents/Drohnenversuchsstand/DROMA/Simulation/scripts/sitl/build_cg]==]
    SKIP_REGULAR_EXPRESSION [==[\[  SKIPPED \]]==]
    
)
set(test_gcs_frame_TESTS [==[GcsFrame.ParseMatchesGolden]==] [==[GcsFrame.RejectsCorruption]==])
