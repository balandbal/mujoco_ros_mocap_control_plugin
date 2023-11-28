# Software License Agreement (BSD 3-Clause License)
# 
#  Copyright (c) 2023, Bielefeld University
#  All rights reserved.
# 
#  Redistribution and use in source and binary forms, with or without
#  modification, are permitted provided that the following conditions
#  are met:
# 
#   * Redistributions of source code must retain the above copyright
#     notice, this list of conditions and the following disclaimer.
# 
#   * Redistributions in binary form must reproduce the above
#     copyright notice, this list of conditions and the following
#     disclaimer in the documentation and/or other materials provided
#     with the distribution.
# 
#   * Neither the name of Bielefeld University nor the names of its
#     contributors may be used to endorse or promote products derived
#     from this software without specific prior written permission.
# 
#  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
#  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
#  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
#  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
#  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
#  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
#  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
#  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
#  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
#  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
#  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
#  POSSIBILITY OF SUCH DAMAGE.

include_guard()

include(StandardProjectSetting)
include(CompilerCache)
include(Linker)
include(Sanitizers)
include(Optimization)
include(CompilerWarning)

function(configure_ros_project_option)
  cmake_parse_arguments("" "" "TARGET" "" ${ARGN})

  target_include_directories(${_TARGET} SYSTEM INTERFACE ${catkin_INCLUDE_DIRS} ${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_INCLUDE_DESTINATION})
  target_include_directories(${_TARGET} INTERFACE ${PROJECT_SOURCE_DIR}/include)
  target_link_libraries(${_TARGET} INTERFACE ${catkin_LIBRARIES})

  target_compile_options(
    ${_TARGET}

    # This is a temporary solution to suppress warnings from 3rd party library in MSVC
    # see https://gitlab.kitware.com/cmake/cmake/-/issues/17904, this will probably be fixed in 3.24
    INTERFACE $<$<AND:$<CXX_COMPILER_ID:MSVC>,$<NOT:$<VERSION_LESS:$<CXX_COMPILER_VERSION>,19.14>>>:/external:W0 /external:anglebrackets>

    # vs 16.10 (19.29.30037) no longer need the /experimental:external flag to use the /external:*
    $<$<AND:$<CXX_COMPILER_ID:MSVC>,$<VERSION_LESS:$<CXX_COMPILER_VERSION>,19.29.30037>>:/experimental:external>)
endfunction()

function(configure_project_option)
  set(groups
    CATKIN_ROS
    WARNINGS
    AVX
    LINKER
    COMPILER_CACHE
    SANITIZER
    IPO
  )
  cmake_parse_arguments(GRP "" "" "${groups}" ${ARGN})

  cmake_parse_arguments(WARNING "" "TARGET;PROJECT_WARNINGS" "" "${GRP_WARNINGS}")

  cmake_parse_arguments(LINKER "" "TARGET" "" "${GRP_LINKER}")
  cmake_parse_arguments(CCACHE "" "LAUNCHER;CCACHE_BASE_DIR" "" "${GRP_COMPILER_CACHE}")
  cmake_parse_arguments(SANITIZER "" "TARGET" "" "${GRP_SANITIZER}")

  cmake_parse_arguments(IPO "" "" "DISABLE_FOR_CONFIG" "${GRP_IPO}")
  cmake_parse_arguments(CATKIN_ROS "" "TARGET" "" "${GRP_CATKIN_ROS}")
  cmake_parse_arguments(AVX "" "TARGET" "" "${GRP_AVX}")

  foreach(target_name ${WARNING_TARGET} ${LINKER_TARGET} ${SANITIZER_TARGET} ${CATKIN_ROS_TARGET} ${AVX_TARGET})
    if(NOT TARGET ${target_name})
      add_library(${target_name} INTERFACE)
      set_target_properties(${target_name} PROPERTIES INTERFACE_INCLUDE_DIRECTORIES "")
    endif()
  endforeach()

  configure_project_setting()

  configure_interprocedural_optimization(DISABLE_FOR_CONFIG ${IPO_DISABLE_FOR_CONFIG})
  if(CMAKE_VERSION VERSION_GREATER_EQUAL "3.18")
    configure_compiler_cache(OPTION ${CCACHE_LAUNCHER} BASE_DIR ${CCACHE_CCACHE_BASE_DIR})
  endif()
  configure_project_warnings(TARGET ${WARNING_TARGET} WARNINGS ${WARNING_PROJECT_WARNINGS})
  configure_project_avx_support(TARGET ${AVX_TARGET})

  configure_linker(TARGET ${LINKER_TARGET})
  configure_sanitizers(TARGET ${SANITIZER_TARGET})

  if(${ENABLE_CLANG_TIDY})
    configure_clang_tidy(EXTRA_ARG ${CLANG_TIDY_EXTRA_ARG} EXTRA_OPTIONS ${CLANG_TIDY_EXTRA_OPTIONS})
  endif()

  configure_ros_project_option(TARGET ${CATKIN_ROS_TARGET})
endfunction()
