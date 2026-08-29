cmake_minimum_required (VERSION 3.8)

set_property(GLOBAL PROPERTY USE_FOLDERS ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Build directory
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY   		${CMAKE_BINARY_DIR}/artifacts)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY   		${CMAKE_BINARY_DIR}/artifacts)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY   		${CMAKE_BINARY_DIR}/artifacts)

# Function because lazy
function(adjust_bin_paths lib)
  set_target_properties(${lib}
		PROPERTIES
		RUNTIME_OUTPUT_DIRECTORY 			${CMAKE_BINARY_DIR}/artifacts
		LIBRARY_OUTPUT_DIRECTORY 			${CMAKE_BINARY_DIR}/artifacts
		ARCHIVE_OUTPUT_DIRECTORY 			${CMAKE_BINARY_DIR}/artifacts
	)
endfunction()

# windows prefers win8.1 sdk instead of win 10 sdk for some god forsaken reason
if(WIN32)
    set(CMAKE_SYSTEM_VERSION 10.0 CACHE STRING "Forcing Win10 SDK" FORCE)
else()
	set(CMAKE_POSITION_INDEPENDENT_CODE ON)
endif()

project("SpaceCalibrator")

# 3rd party stuff
add_subdirectory ("vendor")

# warnings for our code
if ("${CMAKE_CXX_COMPILER}" MATCHES "Clang" OR "${CMAKE_C_COMPILER_ID}" STREQUAL "Clang")
	# add_compile_options(-Werror=return-type -Wall -Wextra -Wpedantic /MP)
	add_compile_options(-Werror=return-type)
elseif ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "GNU" OR "${CMAKE_C_COMPILER_ID}" STREQUAL "GNU")
	# add_compile_options(-Werror=return-type -Wall -Wextra -Wpedantic)
	add_compile_options(-Werror=return-type -Wno-psabi)
elseif ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "Intel" OR "${CMAKE_C_COMPILER_ID}" STREQUAL "Intel")
	# add_compile_options(-Werror=return-type -Wall -Wextra -Wpedantic)
	add_compile_options(/Qdiag-error-return)
elseif ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "MSVC" OR "${CMAKE_C_COMPILER_ID}" STREQUAL "MSVC")
	add_compile_options(
		# treat warnings as errors
		/we4715 # not all control paths return a value
		/we4828 # disallow invalid characters
		# prinf-like functions: format mismatch
		/we4473 # : not enough arguments passed for format string
		/we4474 # : too many arguments passed for format string
		/we4475 # : length modifier cannot be used with type field character in format specifier
		/we4476 # : unknown type field character in format specifier
		/we4477 # : format string requires an argument of type , but variadic argument has type
		/we4478 # : positional and non-positional placeholders cannot be mixed in the same format string
		/we4775 # nonstandard extension used in format string of function
		/we4776 # % is not allowed in the format string of function
		/we4777 # : format string requires an argument of type , but variadic argument has type
		/we4778 # : unterminated format string
		# macro arg mismatch
		/we4002 # too many actual parameters for macro 'identifier'
		/we4003 # not enough actual parameters for macro 'identifier'
		/Zc:threadSafeInit- # https://connect.microsoft.com/VisualStudio/feedback/details/1789709/visual-c-2015-runtime-broken-on-windows-server-2003-c-11-magic-statics
		/MP # multiprocessor compilation
		/utf-8 # utf-8 source & exec
		/GF) # eliminate duplicate strings
endif()

# Match SteamVR build flags on MSVC
if(MSVC)
    add_compile_options(/guard:cf /GS)
    add_link_options("/GUARD:CF")
    add_compile_options($<$<CONFIG:Release>:/Ot> $<$<CONFIG:Release>:/Oi> $<$<CONFIG:Release>:/Gy>)
endif()

# force relative RPATH for linux so that shared objects may be found relative to the binary (fixes Steam launch)
if (UNIX AND NOT APPLE)
	SET(CMAKE_SKIP_BUILD_RPATH FALSE)
	SET(CMAKE_BUILD_WITH_INSTALL_RPATH FALSE)
	SET(CMAKE_INSTALL_RPATH "")
	SET(CMAKE_INSTALL_RPATH_USE_LINK_PATH FALSE)
    SET(CMAKE_BUILD_RPATH_USE_ORIGIN TRUE)
endif()

# add editorconfig to vcxprojs
if (WIN32)
	add_custom_target(FormattingRules SOURCES "${CMAKE_SOURCE_DIR}/.editorconfig" "${CMAKE_SOURCE_DIR}/.clang-format")
	set_property(TARGET FormattingRules PROPERTY FOLDER "Solution Items")
endif()

# Include project
add_subdirectory ("src")