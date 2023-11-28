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

find_package(mujoco QUIET NO_MODULE)

if (mujoco_FOUND AND mujoco_FIND_VERSION)
	if (NOT mujoco_FIND_VERSION_MAJOR EQUAL mujoco_VERSION_MAJOR OR mujoco_FIND_VERSION_MINOR GREATER mujoco_VERSION_MINOR OR mujoco_FIND_VERSION_PATCH GREATER mujoco_VERSION_PATCH)
		message(WARNING "Requested MuJoCo version ${mujoco_FIND_VERSION} but found incompatible version ${mujoco_VERSION}")
		unset(mujoco_FOUND)
	endif()
endif()

if(NOT mujoco_FOUND)
	message(STATUS "Looking for MuJoCo tar install ...")
	# Find headers
	find_file(mujoco_INCLUDE_DIRS include/mujoco/mujoco.h PATHS ENV MUJOCO_DIR)
	if(mujoco_INCLUDE_DIRS)
		get_filename_component(mujoco_INCLUDE_DIRS ${mujoco_INCLUDE_DIRS} PATH)
		get_filename_component(mujoco_INCLUDE_DIRS ${mujoco_INCLUDE_DIRS} PATH)
	endif()

	# Find library
	find_library(mujoco_LIBRARIES lib/libmujoco.so PATHS ENV MUJOCO_DIR)

	# Find dependencies
	cmake_policy(SET CMP0072 NEW)
	include(CMakeFindDependencyMacro)
	find_dependency(OpenGL REQUIRED)

	if(mujoco_INCLUDE_DIRS AND mujoco_LIBRARIES)
		set(mujoco_FOUND TRUE)
		add_library(mujoco::mujoco SHARED IMPORTED)
		set_property(TARGET mujoco::mujoco PROPERTY IMPORTED_LOCATION "${mujoco_LIBRARIES}")
		set_target_properties(mujoco::mujoco PROPERTIES
			INTERFACE_INCLUDE_DIRECTORIES "${mujoco_INCLUDE_DIRS}"
		)
	else()
		message(FATAL_ERROR "Failed to find mujoco (MUJOCO_DIR=${MUJOCO_DIR})")
	endif()

	if(mujoco_FIND_VERSION)
		message(NOTICE "MuJoCo tar install found, but version can not be detected. If experiencing errors check that version ${mujoco_FIND_VERSION} or a newer, compatible version is installed")
	endif()
endif()

set(mujoco_LIBRARIES mujoco::mujoco)
get_target_property(mujoco_INCLUDE_DIRS mujoco::mujoco INTERFACE_INCLUDE_DIRECTORIES)
