# ******************************************************************************
# Copyright 2017-2019 Intel Corporation
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
# ******************************************************************************

set(CLANG_FORMAT_EXEC clang-format)
find_program(CLANG_FORMAT ${CLANG_FORMAT_EXEC} PATHS ENV PATH)

macro(style_check_file PATH)
    execute_process(
        COMMAND ${CLANG_FORMAT} -style=file -output-replacements-xml ${PATH}
        OUTPUT_VARIABLE STYLE_CHECK_RESULT
    )
    if("${STYLE_CHECK_RESULT}" MATCHES ".*<replacement .*")
        message(STATUS "Style error: ${PATH}")
        list(APPEND ERROR_LIST ${PATH})
    endif()
endmacro()

set(DIRECTORIES_OF_INTEREST
    src
    examples
    docs/_static
)

if (CLANG_FORMAT)
    foreach(DIRECTORY ${DIRECTORIES_OF_INTEREST})
        set(CPP_GLOB "${PROJECT_SOURCE_DIR}/${DIRECTORY}/*.cpp")
        set(H_GLOB "${PROJECT_SOURCE_DIR}/${DIRECTORY}/*.h")
        file(GLOB_RECURSE SRC_FILES ${CPP_GLOB} ${H_GLOB})
        foreach(FILE ${SRC_FILES})
            style_check_file(${FILE})
        endforeach(FILE)
    endforeach(DIRECTORY)
    if(ERROR_LIST)
        message(FATAL_ERROR "Style errors")
    endif()
else()
    message(STATUS "${CLANG_FORMAT_EXEC} not found, style not available")
endif()
