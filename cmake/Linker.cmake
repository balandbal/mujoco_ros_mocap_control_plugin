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

#
# configure_linker(
#   TARGET target
#   LINKER linker
#   [LINKER_PATH dir]
# )
#
# This function configures linker by checking if the linker is supported by compiler, if yes,
# link the target with -fuse-ld=linker, otherwise it links the target with -B/linker/path if
# compiler support -B option and LINKER_PATH is specified.
#
function(configure_linker)
  cmake_parse_arguments("" "" "TARGET" "" ${ARGN})

  if(NOT _TARGET)
    message(FATAL_ERROR "No target specified!")
  endif()

  if(MSVC)
    set(EXTRA_LINK_OPTIONS /OPT:REF /OPT:ICF=5)
  else()
    set(EXTRA_LINK_OPTIONS)

    if(WIN32)
      set(CMAKE_REQUIRED_FLAGS "-fuse-ld=lld-link")
      check_c_source_compiles("int main() {}" SUPPORTS_LLD)
      if(SUPPORTS_LLD)
        set(EXTRA_LINK_OPTIONS
            ${EXTRA_LINK_OPTIONS}
            -fuse-ld=lld-link
            -Wl,/OPT:REF
            -Wl,/OPT:ICF
        )
      endif()
    else()
      set(CMAKE_REQUIRED_FLAGS "-fuse-ld=lld")
      check_c_source_compiles("int main() {}" SUPPORTS_LLD)
      if(SUPPORTS_LLD)
        set(EXTRA_LINK_OPTIONS ${EXTRA_LINK_OPTIONS} -fuse-ld=lld)
      else()
        set(CMAKE_REQUIRED_FLAGS "-fuse-ld=gold")
        check_c_source_compiles("int main() {}" SUPPORTS_GOLD)
        if(SUPPORTS_GOLD)
          set(EXTRA_LINK_OPTIONS ${EXTRA_LINK_OPTIONS} -fuse-ld=gold)
        endif()
      endif()

      set(CMAKE_REQUIRED_FLAGS ${EXTRA_LINK_OPTIONS} "-Wl,--gc-sections")
      check_c_source_compiles("int main() {}" SUPPORTS_GC_SECTIONS)
      if(SUPPORTS_GC_SECTIONS)
        set(EXTRA_LINK_OPTIONS ${EXTRA_LINK_OPTIONS} -Wl,--gc-sections)
      else()
        set(CMAKE_REQUIRED_FLAGS ${EXTRA_LINK_OPTIONS} "-Wl,-dead_strip")
        check_c_source_compiles("int main() {}" SUPPORTS_DEAD_STRIP)
        if(SUPPORTS_DEAD_STRIP)
          set(EXTRA_LINK_OPTIONS ${EXTRA_LINK_OPTIONS} -Wl,-dead_strip)
        endif()
      endif()
    endif()
  endif()

  target_link_options(${_TARGET} INTERFACE ${EXTRA_LINK_OPTIONS})
endfunction ()
