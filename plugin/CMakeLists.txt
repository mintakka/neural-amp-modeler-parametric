cmake_minimum_required(VERSION 3.22)
project(HyperNAM VERSION 0.1.0 LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
set(CMAKE_OSX_DEPLOYMENT_TARGET "11.0" CACHE STRING "")

include(FetchContent)
FetchContent_Declare(JUCE
  URL https://github.com/juce-framework/JUCE/archive/refs/tags/8.0.15.tar.gz
  DOWNLOAD_EXTRACT_TIMESTAMP TRUE)
FetchContent_MakeAvailable(JUCE)

# Portable DSP core (no JUCE): also what a Daisy port would reuse.
add_library(hypernam_engine STATIC engine/hyper_engine.cpp)
target_include_directories(hypernam_engine PUBLIC engine)
target_compile_options(hypernam_engine PRIVATE -O3)
set_target_properties(hypernam_engine PROPERTIES POSITION_INDEPENDENT_CODE ON)

add_executable(engine_cli tests/engine_cli.cpp)
target_link_libraries(engine_cli PRIVATE hypernam_engine)

juce_add_plugin(HyperNAM
  COMPANY_NAME "NAM Parametric"
  BUNDLE_ID com.namparametric.hypernam
  PLUGIN_MANUFACTURER_CODE Nmpr
  PLUGIN_CODE Hnam
  FORMATS AU VST3 Standalone
  PRODUCT_NAME "HyperNAM"
  IS_SYNTH FALSE
  NEEDS_MIDI_INPUT FALSE
  NEEDS_MIDI_OUTPUT FALSE
  IS_MIDI_EFFECT FALSE
  AU_MAIN_TYPE kAudioUnitType_Effect
  VST3_CATEGORIES Fx Distortion
  COPY_PLUGIN_AFTER_BUILD TRUE)

target_sources(HyperNAM PRIVATE
  src/PluginProcessor.cpp
  src/PluginEditor.cpp)

target_compile_definitions(HyperNAM PUBLIC
  JUCE_WEB_BROWSER=0
  JUCE_USE_CURL=0
  JUCE_VST3_CAN_REPLACE_VST2=0
  JUCE_DISPLAY_SPLASH_SCREEN=0)

target_link_libraries(HyperNAM
  PRIVATE
    hypernam_engine
    juce::juce_audio_utils
  PUBLIC
    juce::juce_recommended_config_flags
    juce::juce_recommended_lto_flags
    juce::juce_recommended_warning_flags)

enable_testing()
find_program(NAM_PYTHON NAMES python3 python)
add_test(NAME engine_matches_pytorch
  COMMAND ${NAM_PYTHON} ${CMAKE_CURRENT_SOURCE_DIR}/tests/verify_engine.py)
