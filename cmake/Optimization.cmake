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
include(CheckAVXSupport)

option(ENABLE_IPO "Enable Interprocedural Optimization (IPO), a.k.a Link Time Optimizaition (LTO)" OFF)
option(ENABLE_AVX "Build binaries that require AVX instructions, if possible" ON)
option(ENABLE_AVX_INTRINSICS "Make use of hand-written AVX intrinsics, if possible" ON)

cmake_policy(PUSH)

if (POLICY CMP0069)
  cmake_policy(SET CMP0069 NEW)
endif ()

function (configure_interprocedural_optimization)
  cmake_parse_arguments("" "" "" "DISABLE_FOR_CONFIG" ${ARGN})

  foreach (config_type IN LISTS _DISABLE_FOR_CONFIG)
    string(TOUPPER ${config_type} config_type)
    set(CMAKE_INTERPROCEDURAL_OPTIMIZATION_${config_type} OFF CACHE INTERNAL "IPO is disabled for ${config_type}" FORCE)
  endforeach ()

  if (NOT ${ENABLE_IPO})
    set(CMAKE_INTERPROCEDURAL_OPTIMIZATION OFF CACHE INTERNAL "" FORCE)
    return()
  endif ()

  include(CheckIPOSupported)
  check_ipo_supported(RESULT result OUTPUT output LANGUAGES CXX)
  if (result)
    set(CMAKE_INTERPROCEDURAL_OPTIMIZATION ON CACHE INTERNAL "" FORCE)
  else ()
    message(WARNING "Interprocedural Optimization is not supported. Reason: ${output}")
  endif ()
endfunction ()

function(configure_project_avx_support)
  cmake_parse_arguments("" "" "TARGET" "AVX" ${ARGN})

  if(NOT _TARGET)
    message(FATAL_ERROR "No target specified")
  endif()

  if(ENABLE_AVX)
    get_avx_compile_options(AVX_COMPILE_OPTIONS)
  else()
    message(STATUS "AVX support disabled manually")
    set(AVX_COMPILE_OPTIONS "")
  endif()

  target_compile_options(${_TARGET} INTERFACE "$<IF:$<BOOL:${_AVX}>,${_AVX},${AVX_COMPILE_OPTIONS}>")
endfunction()

cmake_policy(POP)
