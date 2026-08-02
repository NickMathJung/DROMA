cmake_minimum_required(VERSION 3.16)
# WICHTIG: C UND CXX aktivieren -> Coders .c-Dateien werden als C kompiliert
# (unmangled), passend zu den extern-"C"-Deklarationen im Test.
project(sitl_golden LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# --- Optionen -------------------------------------------------------------
set(QUAT_IMPL   "ref" CACHE STRING "ref | codegen")
set(SAFETY_IMPL "ref" CACHE STRING "ref | codegen")
set(CODEGEN_ROOT "${CMAKE_SOURCE_DIR}/codegen" CACHE PATH "MATLAB Coder output root")
# MATLAB-Wurzel: Coders rtwtypes.h zieht tmwtypes.h aus <MATLAB_ROOT>/extern/include.
# In MATLAB herausfinden mit:  matlabroot
# Setzen z.B.:  -DMATLAB_ROOT="C:/Program Files/MATLAB/R2025b"
set(MATLAB_ROOT "" CACHE PATH "MATLAB root (fuer tmwtypes.h in extern/include)")
set(GOLDEN_TOL "1e-9" CACHE STRING "abs tolerance golden vs impl")
# Wo die Golden-CSVs liegen. Default: mitgelieferter data/-Ordner (kein Flag noetig).
set(GOLDEN_DIR "${CMAKE_SOURCE_DIR}/data" CACHE PATH "Ordner mit test_data_quat*.csv")

# CODEGEN_ROOT robust normalisieren (Backslash -> Slash, relativ -> absolut).
file(TO_CMAKE_PATH "${CODEGEN_ROOT}" _CGR)
get_filename_component(CGROOT "${_CGR}" ABSOLUTE BASE_DIR "${CMAKE_SOURCE_DIR}")

# MATLAB extern/include aufloesen (nur noetig bei *_IMPL=codegen).
function(matlab_include out_var)
  if(MATLAB_ROOT STREQUAL "")
    message(FATAL_ERROR
      "codegen gewaehlt, aber MATLAB_ROOT ist leer. In MATLAB 'matlabroot' "
      "eingeben und z.B. -DMATLAB_ROOT=\"C:/Program Files/MATLAB/R2025b\" setzen.")
  endif()
  file(TO_CMAKE_PATH "${MATLAB_ROOT}" _mr)
  set(_inc "${_mr}/extern/include")
  if(NOT EXISTS "${_inc}/tmwtypes.h")
    message(FATAL_ERROR "tmwtypes.h nicht unter ${_inc} gefunden — MATLAB_ROOT falsch?")
  endif()
  set(${out_var} "${_inc}" PARENT_SCOPE)
endfunction()

# --- GoogleTest via FetchContent -----------------------------------------
include(FetchContent)
FetchContent_Declare(googletest
  GIT_REPOSITORY https://github.com/google/googletest.git
  GIT_TAG        v1.15.2)
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(googletest)
enable_testing()
include(GoogleTest)

# --- Helfer: generierte C-Quellen + Include-Dirs je Funktion einsammeln ----
# Sammelt DIR/lib/<fn>/*.c (nicht rekursiv -> examples/main.c bleibt aussen vor)
# und alle Ordner mit Headern (fuer rtwtypes.h etc.).
function(collect_codegen fns out_src out_inc)
  set(_src "")
  set(_inc "")
  foreach(fn ${fns})
    set(_dir "${CGROOT}/lib/${fn}")
    file(GLOB _c "${_dir}/*.c")
    if(_c STREQUAL "")
      message(WARNING "collect_codegen: keine .c unter ${_dir} - schon generiert?")
    else()
      message(STATUS "  ${fn}: ${_c}")
    endif()
    list(APPEND _src ${_c})
    file(GLOB _h "${_dir}/*.h")
    if(NOT _h STREQUAL "")
      list(APPEND _inc "${_dir}")
    endif()
  endforeach()
  if(NOT _inc STREQUAL "")
    list(REMOVE_DUPLICATES _inc)
  endif()
  set(${out_src} "${_src}" PARENT_SCOPE)
  set(${out_inc} "${_inc}" PARENT_SCOPE)
endfunction()

# --- Quaternion-Impl waehlen ---------------------------------------------
set(QUAT_INC "")
if(QUAT_IMPL STREQUAL "ref")
  set(QUAT_SRC src/quat_helpers_ref.cpp)
else()
  collect_codegen("dcm2quat_local;quat2dcm_local;quatMul;quatConj;quatRotate" QUAT_SRC QUAT_INC)
  matlab_include(_MINC)
  list(APPEND QUAT_INC "${_MINC}")
endif()

# --- Safety-Impl waehlen --------------------------------------------------
set(SAFETY_INC "")
set(SAFETY_DEF "")
if(SAFETY_IMPL STREQUAL "ref")
  set(SAFETY_SRC src/safety_helpers_ref.cpp)
else()
  collect_codegen("safety_overspeed;safety_battery" _SAF_GEN SAFETY_INC)
  matlab_include(_MINC2)
  list(APPEND SAFETY_INC "${_MINC2}")
  set(SAFETY_SRC src/codegen_shim_overspeed.cpp src/codegen_shim_battery.cpp ${_SAF_GEN})
  # Deine Header haben Params via coder.Constant einkompiliert -> S9 (use_norm=true)
  # ist im generierten Code nicht schaltbar. Test blendet S9 dann aus.
  set(SAFETY_DEF SAFETY_CODEGEN_CONST_PARAMS)
endif()

# --- Test-Targets ---------------------------------------------------------
add_executable(test_quat_golden test/test_quat_golden.cpp ${QUAT_SRC})
target_include_directories(test_quat_golden PRIVATE include test ${QUAT_INC})
target_compile_definitions(test_quat_golden PRIVATE
  GOLDEN_DIR="${GOLDEN_DIR}" GOLDEN_TOL=${GOLDEN_TOL})
target_link_libraries(test_quat_golden PRIVATE GTest::gtest_main)
gtest_discover_tests(test_quat_golden)

add_executable(test_safety test/test_safety.cpp ${SAFETY_SRC})
target_include_directories(test_safety PRIVATE include test ${SAFETY_INC})
target_compile_definitions(test_safety PRIVATE ${SAFETY_DEF})
target_link_libraries(test_safety PRIVATE GTest::gtest_main)
gtest_discover_tests(test_safety)

# --- Codec-Cross-Check: MATLAB link_tx/link_rx == C++ pkt (mcu_packet.hpp) ----
# Header-only (include/mcu_packet.hpp) gegen Golden aus dump_link_codec_golden.m.
# Keine Codegen-Abhaengigkeit -> laeuft immer.
add_executable(test_link_codec test/test_link_codec.cpp)
target_include_directories(test_link_codec PRIVATE include test)
target_compile_definitions(test_link_codec PRIVATE GOLDEN_DIR="${GOLDEN_DIR}")
target_link_libraries(test_link_codec PRIVATE GTest::gtest_main)
gtest_discover_tests(test_link_codec)

# --- GS-Frame-Cross-Check: MATLAB pack_gcs_frame == C++ gcs::parse (gcs_frame.hpp)
add_executable(test_gcs_frame test/test_gcs_frame.cpp)
target_include_directories(test_gcs_frame PRIVATE include test)
target_compile_definitions(test_gcs_frame PRIVATE GOLDEN_DIR="${GOLDEN_DIR}")
target_link_libraries(test_gcs_frame PRIVATE GTest::gtest_main)
gtest_discover_tests(test_gcs_frame)

set(MODEL_ROOT "${CMAKE_SOURCE_DIR}/mcu_ert_rtw" CACHE PATH
    "slbuild/packNGo-Ausgabe von mcu (enthaelt mcu.cpp, mcu_data.cpp, mcu.h ...)")
set(GOLDEN_MCU_CSV "${GOLDEN_DIR}/golden_mcu_io.csv" CACHE FILEPATH
    "Golden-I/O aus log_mcu_golden.m")
 
file(TO_CMAKE_PATH "${MODEL_ROOT}" _MR)
if(EXISTS "${_MR}/mcu.cpp")
  # Alle generierten .cpp AUSSER ert_main.cpp (dessen main() kollidiert mit gtest_main).
  file(GLOB _MCU_SRC "${_MR}/*.cpp")
  list(FILTER _MCU_SRC EXCLUDE REGEX "ert_main\\.cpp$")
 
  add_executable(test_mcu_model test/test_mcu_model.cpp ${_MCU_SRC})
  target_include_directories(test_mcu_model PRIVATE include test "${_MR}")
  target_compile_definitions(test_mcu_model PRIVATE
      GOLDEN_MCU_CSV="${GOLDEN_MCU_CSV}" GOLDEN_TOL=${GOLDEN_TOL})
  target_link_libraries(test_mcu_model PRIVATE GTest::gtest_main)
  gtest_discover_tests(test_mcu_model)
else()
  message(STATUS
    "test_mcu_model uebersprungen: ${_MR}/mcu.cpp fehlt "
    "(erst configure_mcu_codegen -> slbuild('mcu') laufen lassen).")
endif()

# --- Diagnose-Executable (kein Test): lokalisiert die erste Golden-Divergenz ---
if(EXISTS "${_MR}/mcu.cpp")
  add_executable(diag_mcu_replay test/diag_mcu_replay.cpp ${_MCU_SRC})
  target_include_directories(diag_mcu_replay PRIVATE include test "${_MR}")
  target_compile_definitions(diag_mcu_replay PRIVATE
      GOLDEN_MCU_CSV="${GOLDEN_MCU_CSV}" DIAG_TOL=${GOLDEN_TOL})
endif()

# --- FLATNESS-MCU: Modell-Golden (mcu_flat_ert_rtw, Klasse MCU_FLAT) ----------
set(MODEL_FLAT_ROOT "${CMAKE_SOURCE_DIR}/mcu_flat_ert_rtw" CACHE PATH
    "slbuild-Ausgabe von mcu_flat (enthaelt mcu_flat.cpp, mcu_flat.h ...)")
set(GOLDEN_MCU_FLAT_CSV "${GOLDEN_DIR}/golden_mcu_flat_io.csv" CACHE FILEPATH
    "Golden-I/O aus log_mcu_flat_golden.m")

file(TO_CMAKE_PATH "${MODEL_FLAT_ROOT}" _MFR)
if(EXISTS "${_MFR}/mcu_flat.cpp")
  file(GLOB _MCUF_SRC "${_MFR}/*.cpp")
  list(FILTER _MCUF_SRC EXCLUDE REGEX "ert_main\\.cpp$")

  add_executable(test_mcu_flat_model test/test_mcu_flat_model.cpp ${_MCUF_SRC})
  target_include_directories(test_mcu_flat_model PRIVATE include test "${_MFR}")
  target_compile_definitions(test_mcu_flat_model PRIVATE
      GOLDEN_MCU_FLAT_CSV="${GOLDEN_MCU_FLAT_CSV}" GOLDEN_TOL=${GOLDEN_TOL})
  target_link_libraries(test_mcu_flat_model PRIVATE GTest::gtest_main)
  gtest_discover_tests(test_mcu_flat_model)
else()
  message(STATUS
    "test_mcu_flat_model uebersprungen: ${_MFR}/mcu_flat.cpp fehlt "
    "(erst configure_mcu_flat_codegen -> slbuild('mcu_flat') laufen lassen).")
endif()
