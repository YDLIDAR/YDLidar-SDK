
include(CMakeParseArguments)

#=============================================================================
#
#	ydlidar_parse_function_args
#
#	This function simplifies usage of the cmake_parse_arguments module.
#	It is intended to be called by other functions.
#
#	Usage:
#		ydlidar_parse_function_args(
#			NAME <name>
#			[ OPTIONS <list> ]
#			[ ONE_VALUE <list> ]
#			[ MULTI_VALUE <list> ]
#			REQUIRED <list>
#			ARGN <ARGN>)
#
#	Input:
#		NAME		: the name of the calling function
#		OPTIONS		: boolean flags
#		ONE_VALUE	: single value variables
#		MULTI_VALUE	: multi value variables
#		REQUIRED	: required arguments
#		ARGN		: the function input arguments, typically ${ARGN}
#
#	Output:
#		The function arguments corresponding to the following are set:
#		${OPTIONS}, ${ONE_VALUE}, ${MULTI_VALUE}
#
#	Example:
#		function test()
#			ydlidar_parse_function_args(
#				NAME TEST
#				ONE_VALUE NAME
#				MULTI_VALUE LIST
#				REQUIRED NAME LIST
#				ARGN ${ARGN})
#			message(STATUS "name: ${NAME}")
#			message(STATUS "list: ${LIST}")
#		endfunction()
#
#		test(NAME "hello" LIST a b c)
#
#		OUTPUT:
#			name: hello
#			list: a b c
#
function(ydlidar_parse_function_args)
	cmake_parse_arguments(IN "" "NAME" "OPTIONS;ONE_VALUE;MULTI_VALUE;REQUIRED;ARGN" "${ARGN}")
	cmake_parse_arguments(OUT "${IN_OPTIONS}" "${IN_ONE_VALUE}" "${IN_MULTI_VALUE}" "${IN_ARGN}")
	if (OUT_UNPARSED_ARGUMENTS)
		message(FATAL_ERROR "${IN_NAME}: unparsed ${OUT_UNPARSED_ARGUMENTS}")
	endif()
	foreach(arg ${IN_REQUIRED})
		if (NOT OUT_${arg})
			if (NOT "${OUT_${arg}}" STREQUAL "0")
				message(FATAL_ERROR "${IN_NAME} requires argument ${arg}\nARGN: ${IN_ARGN}")
			endif()
		endif()
	endforeach()
	foreach(arg ${IN_OPTIONS} ${IN_ONE_VALUE} ${IN_MULTI_VALUE})
		set(${arg} ${OUT_${arg}} PARENT_SCOPE)
	endforeach()
endfunction()


macro( add_to_ydlidar_include_dirs )
foreach( dir ${ARGN} )
  set_property( GLOBAL APPEND PROPERTY YDLIDAR_INCLUDE_DIRS "${dir}" )
  endforeach()
endmacro()


macro( add_to_ydlidar_libraries )
    foreach( lib ${ARGN} )
      # Process targets correctly
      if (TARGET ${lib})
          # If the library is NOT imported, ie is in this project, we
          # want to depend on it directly rather than through its path
          get_target_property(is_lib_imported ${lib} IMPORTED)
          if (NOT ${is_lib_imported})
            set_property( GLOBAL APPEND PROPERTY YDLIDAR_LIBRARIES "${lib}" )
          else()
            # For imported targets, we just want to depend on the library directly
            get_target_property(libpath ${lib} LOCATION)
            if (libpath)
                set_property( GLOBAL APPEND PROPERTY YDLIDAR_LIBRARIES "${libpath}" )
              # This shouldn't really happen, but let's cover our bases.
            else()
                set_property( GLOBAL APPEND PROPERTY YDLIDAR_LIBRARIES "${lib}" )
            endif()
          endif()
      else()			# Just add the direct path/flag to the list
        set_property( GLOBAL APPEND PROPERTY YDLIDAR_LIBRARIES "${lib}" )
      endif()
    endforeach()

endmacro()


macro( add_to_ydlidar_sources )
    if("${SDK_SOURCE_DIR}" STREQUAL "")
        file(RELATIVE_PATH _relPath "${CMAKE_SOURCE_DIR}" "${CMAKE_CURRENT_SOURCE_DIR}")
    else()
        file(RELATIVE_PATH _relPath "${SDK_SOURCE_DIR}" "${CMAKE_CURRENT_SOURCE_DIR}")
    endif()
    foreach(_src ${ARGN})
        if(_relPath)
            set_property( GLOBAL APPEND PROPERTY YDLIDAR_SOURCES "${_relPath}/${_src}" )
        else()
            set_property( GLOBAL APPEND PROPERTY YDLIDAR_SOURCES "${_src}" )
        endif()
    endforeach()
endmacro()

macro( add_to_ydlidar_headers )
    if("${SDK_SOURCE_DIR}" STREQUAL "")
        file(RELATIVE_PATH _relPath "${CMAKE_SOURCE_DIR}" "${CMAKE_CURRENT_SOURCE_DIR}")
    else()
        file(RELATIVE_PATH _relPath "${SDK_SOURCE_DIR}" "${CMAKE_CURRENT_SOURCE_DIR}")
    endif()
    foreach(_hdr ${ARGN})
        if(_relPath)
            set_property( GLOBAL APPEND PROPERTY YDLIDAR_HEADERS "${_relPath}/${_hdr}" )
        else()
            set_property( GLOBAL APPEND PROPERTY YDLIDAR_HEADERS "${_hdr}" )
        endif()
    endforeach()
endmacro()

macro( ydlidar_set_compile_flags file flags )
    set_property( GLOBAL APPEND PROPERTY COMPILER_OPTS_SOURCES "${file}" )
    set_property( GLOBAL APPEND PROPERTY COMPILER_OPTS_FLAGS "${flags}" )
endmacro()


macro(subdirlist result curdir)
  file(GLOB children RELATIVE ${curdir} ${curdir}/*)
  set(dirlist "")
  foreach(child ${children})
    if( NOT child STREQUAL "CMakeFiles" )
      if(IS_DIRECTORY ${curdir}/${child})
          set(dirlist ${dirlist} ${child})
      endif()
    endif()
  endforeach()
  set(${result} ${dirlist})
endmacro()

macro(aux_include_directory dir result )
    set(curdir ${CMAKE_CURRENT_SOURCE_DIR})
    if(NOT (${dir} STREQUAL ".") )
        set(curdir ${dir})
    endif()
    FILE(GLOB INC_LIST "${curdir}/*.h")
    FILE(GLOB PRV_INC_LIST "${curdir}/*.hpp")
    set(INCS "")
    foreach(child ${INC_LIST})
        string(REPLACE "${curdir}/" "" child_LIST ${child})
        list(APPEND INCS ${child_LIST})
    endforeach()

    foreach(prvchild ${PRV_INC_LIST})
        string(REPLACE "${curdir}/" "" prvchild_LIST ${prvchild})
        list(APPEND INCS ${prvchild_LIST})
    endforeach()
    set(${result} ${INCS})
endmacro()

macro(aux_src_directory dir result )
    set(curdir ${CMAKE_CURRENT_SOURCE_DIR})
    if(NOT (${dir} STREQUAL ".") )
        set(curdir ${dir})
    endif()
    FILE(GLOB INC_LIST "${curdir}/*.c")
    FILE(GLOB PRV_INC_LIST "${curdir}/*.cpp")
    FILE(GLOB TPP_INC_LIST "${curdir}/*.tpp")
    set(INCS "")
    foreach(child ${INC_LIST})
        string(REPLACE "${curdir}/" "" child_LIST ${child})
        list(APPEND INCS ${child_LIST})
    endforeach()

    foreach(prvchild ${PRV_INC_LIST})
        string(REPLACE "${curdir}/" "" prvchild_LIST ${prvchild})
        list(APPEND INCS ${prvchild_LIST})
    endforeach()


    foreach(tppchild ${TPP_INC_LIST})
        string(REPLACE "${curdir}/" "" tppchild_LIST ${tppchild})
        list(APPEND INCS ${tppchild_LIST})
    endforeach()
    set(${result} ${INCS})
endmacro()


#=============================================================================
#
#	ydlidar_strip_optimization
#
function(ydlidar_strip_optimization name)
	set(_compile_flags)
	separate_arguments(_args UNIX_COMMAND ${ARGN})
	foreach(_flag ${_args})
		if(NOT "${_flag}" MATCHES "^-O")
			set(_compile_flags "${_compile_flags} ${_flag}")
		endif()
	endforeach()
	string(STRIP "${_compile_flags}" _compile_flags)
	set(${name} "${_compile_flags}" PARENT_SCOPE)
endfunction()



#=============================================================================
#
#	ydlidar_join
#
#	This function joins a list with a given separator. If list is not
#	passed, or is sent "", this will return the empty string.
#
#	Usage:
#		ydlidar_join(OUT ${OUT} [ LIST ${LIST} ] GLUE ${GLUE})
#
#	Input:
#		LIST		: list to join
#		GLUE		: separator to use
#
#	Output:
#		OUT			: joined list
#
#	Example:
#		ydlidar_join(OUT test_join LIST a b c GLUE ";")
#		test_join would then be:
#			"a;b;c"
#
function(ydlidar_join)
        ydlidar_parse_function_args(
                NAME ydlidar_join
                ONE_VALUE OUT GLUE
                MULTI_VALUE LIST
                REQUIRED GLUE OUT
                ARGN ${ARGN})
        string (REPLACE ";" "${GLUE}" _TMP_STR "${LIST}")
        set(${OUT} ${_TMP_STR} PARENT_SCOPE)
endfunction()

#=============================================================================
function(ydlidar_add_module)

        ydlidar_parse_function_args(
                NAME ydlidar_add_module
                ONE_VALUE MODULE
                MULTI_VALUE COMPILE_FLAGS LINK_FLAGS SRCS INCLUDES DEPENDS
                OPTIONS EXTERNAL
                REQUIRED MODULE
                ARGN ${ARGN})


        ydlidar_add_library(${MODULE} STATIC EXCLUDE_FROM_ALL ${SRCS})

        # Pass variable to the parent ydlidar_add_module.
        set(_no_optimization_for_target ${_no_optimization_for_target} PARENT_SCOPE)

        if(COMPILE_FLAGS)
           target_compile_options(${MODULE} PRIVATE ${COMPILE_FLAGS})
        endif()

        if(INCLUDES)
             target_include_directories(${MODULE} PRIVATE ${INCLUDES})
        endif()

        if(DEPENDS)
              add_dependencies(${MODULE} ${DEPENDS})
        endif()

        # join list variables to get ready to send to compiler
        foreach(prop LINK_FLAGS)
                if(${prop})
                        ydlidar_join(OUT ${prop} LIST ${${prop}} GLUE " ")
                endif()
        endforeach()

        if(COMPILE_FLAGS AND ${_no_optimization_for_target})
             ydlidar_strip_optimization(COMPILE_FLAGS ${COMPILE_FLAGS})
        endif()
        foreach (prop LINK_FLAGS )
           if (${prop})
               set_target_properties(${MODULE} PROPERTIES ${prop} ${${prop}})
            endif()
        endforeach()

	set_property(GLOBAL APPEND PROPERTY YDLIDAR_LIBRARIES ${MODULE})
        set_property(GLOBAL APPEND PROPERTY YDLIDAR_MODULE_PATHS ${CMAKE_CURRENT_SOURCE_DIR})
endfunction()

#=============================================================================
#
#	ydlidar_add_optimization_flags_for_target
#
set(all_posix_cmake_targets "" CACHE INTERNAL "All cmake targets for which optimization can be suppressed")
function(ydlidar_add_optimization_flags_for_target target)
    set(_no_optimization_for_target FALSE)
    # If the current CONFIG is posix_sitl_* then suppress optimization for certain targets.
    foreach(_regexp $ENV{CORE_NO_OPTIMIZATION})
        if("${target}" MATCHES "${_regexp}")
            set(_no_optimization_for_target TRUE)
            set(_matched_regexp "${_regexp}")
        endif()
    endforeach()
    # Create a full list of targets that optimization can be suppressed for.
    list(APPEND all_posix_cmake_targets ${target})
    set(all_posix_cmake_targets ${all_posix_cmake_targets} CACHE INTERNAL "All cmake targets for which optimization can be suppressed")
    if(NOT ${_no_optimization_for_target})
        target_compile_options(${target} PRIVATE ${optimization_flags})
	else()
		message(STATUS "Disabling optimization for target '${target}' because it matches the regexp '${_matched_regexp}' in env var CORE_NO_OPTIMIZATION")
		target_compile_options(${target} PRIVATE -O0)
	endif()
	# Pass variable to the parent ydlidar_add_library.
	set(_no_optimization_for_target ${_no_optimization_for_target} PARENT_SCOPE)
endfunction()

#=============================================================================
#
#	ydlidar_add_executable
#
#	Like add_executable but with optimization flag fixup.
#
function(ydlidar_add_executable target)
	add_executable(${target} ${ARGN})
	ydlidar_add_optimization_flags_for_target(${target})
endfunction()

#=============================================================================
#
#	ydlidar_add_library
#
#	Like add_library but with optimization flag fixup.
#
function(ydlidar_add_library target)
	add_library(${target} ${ARGN})
	ydlidar_add_optimization_flags_for_target(${target})

	# Pass variable to the parent ydlidar_add_module.
	set(_no_optimization_for_target ${_no_optimization_for_target} PARENT_SCOPE)

	set_property(GLOBAL APPEND PROPERTY YDLIDAR_LIBRARIES ${target})
        set_property(GLOBAL APPEND PROPERTY YDLIDAR_MODULE_PATHS ${CMAKE_CURRENT_SOURCE_DIR})

endfunction()


#=============================================================================
#
#	ydlidar_prebuild_targets
#
#	This function generates os dependent targets
#
#	Usage:
#		ydlidar_prebuild_targets(
#			OUT <out-list_of_targets>
#			BOARD <in-string>
#			)
#
#	Input:
#		BOARD		: board
#
#	Output:
#		OUT	: the target list
#
#	Example:
#		ydlidar_prebuild_targets(OUT target_list BOARD r16)
#
function(ydlidar_prebuild_targets)
        ydlidar_parse_function_args(
                        NAME ydlidar_prebuild_targets
                        ONE_VALUE OUT BOARD
                        REQUIRED OUT
                        ARGN ${ARGN})

        add_library(prebuild_targets INTERFACE)
        #add_dependencies(prebuild_targets DEPENDS)

endfunction()
