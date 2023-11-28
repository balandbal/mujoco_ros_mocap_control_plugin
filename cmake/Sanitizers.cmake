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

option(ENABLE_ASAN "Enable address sanitizer" FALSE)
option(ENABLE_LSAN "Enable leak sanitizer" FALSE)
option(ENABLE_UBSAN "Enable undefined behavior sanitizer" FALSE)
option(ENABLE_TSAN "Enable thread sanitizer" FALSE)
option(ENABLE_MSAN "Enable memory sanitizer" FALSE)

function (configure_sanitizers)
  cmake_parse_arguments("" "" "TARGET" "" ${ARGN})

  if (NOT _TARGET)
    message(FATAL_ERROR "There should be at least one target specified")
  endif ()

  if (CMAKE_CXX_COMPILER_ID STREQUAL "GNU" OR CMAKE_CXX_COMPILER_ID MATCHES ".*Clang")
    set(SANITIZERS
        $<$<BOOL:${ENABLE_ASAN}>:address> $<$<BOOL:${ENABLE_LSAN}>:leak> $<$<BOOL:${ENABLE_UBSAN}>:undefined>
        $<$<AND:$<BOOL:${ENABLE_TSAN}>,$<NOT:$<BOOL:${ENABLE_ASAN}>>,$<NOT:$<BOOL:${ENABLE_LSAN}>>>:thread>
        $<$<AND:$<BOOL:${ENABLE_MSAN}>,$<NOT:$<BOOL:${ENABLE_ASAN}>>,$<NOT:$<BOOL:${ENABLE_TSAN}>>,$<NOT:$<BOOL:${ENABLE_LSAN}>>>:memory>)

    if (ENABLE_TSAN AND (ENABLE_ASAN OR ENABLE_LSAN))
      message(WARNING "Thread sanitizer does not work with Address and Leak sanitizer enabled, skipping TSAN")
    endif ()

    if (ENABLE_MSAN AND CMAKE_CXX_COMPILER_ID MATCHES ".*Clang")
      # see https://github.com/google/sanitizers/wiki/MemorySanitizerLibcxxHowTo
      message(WARNING "MSAN requires all the code (including libc++) to be MSan-instrumented otherwise it reports false positives")
      if (ENABLE_ASAN OR ENABLE_TSAN OR ENABLE_LSAN)
        message(WARNING "Memory sanitizer does not work with Address, Thread and Leak sanitizer enabled, skipping MSAN")
      endif ()
    endif ()

    # see https://clang.llvm.org/docs/ThreadSanitizer.html, https://clang.llvm.org/docs/AddressSanitizer.html and
    # https://clang.llvm.org/docs/MemorySanitizer.html
    set(msan_flag $<$<BOOL:${ENABLE_MSAN}>:-fPIE -pie>)
    set(perfect_stacktraces $<$<BOOL:${ENABLE_ASAN}>:-fno-omit-frame-pointer -fno-optimize-sibling-calls>)
    set(disable_inline_optimization $<$<OR:$<BOOL:${ENABLE_ASAN}>,$<BOOL:${ENABLE_TSAN}>,$<BOOL:${ENABLE_MSAN}>>:-O1>)
    set(sanitizer_flag "$<JOIN:${SANITIZERS},$<COMMA>>")
    target_compile_options(${_TARGET} INTERFACE $<$<BOOL:${sanitizer_flag}>:-fsanitize=${sanitizer_flag}> ${disable_inline_optimization}
                                                ${perfect_stacktraces} ${msan_flag})
    target_link_options(${_TARGET} INTERFACE $<$<BOOL:${sanitizer_flag}>:-fsanitize=${sanitizer_flag}>)
  elseif (MSVC)
    if (${ENABLE_LSAN} OR ${ENABLE_UBSAN} OR ${ENABLE_TSAN} OR ${ENABLE_MSAN})
      message(WARNING "MSVC only supports address sanitizer")
    endif ()

    # https://docs.microsoft.com/en-us/cpp/sanitizers/asan?view=msvc-170
    set(SANITIZERS $<$<BOOL:${ENABLE_ASAN}>:/fsanitize=address>)
    set(perfect_stacktrace $<$<BOOL:${ENABLE_ASAN}>:/Zi>)
    target_compile_options(${_TARGET} INTERFACE ${SANITIZERS} ${perfect_stacktrace})
    target_link_options(${_TARGET} INTERFACE $<$<BOOL:${ENABLE_ASAN}>:/INCREMENTAL:NO>)
  endif ()
endfunction ()
