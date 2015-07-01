# - Try to find PMDSDK headers and libraries.
#
# Usage of this module as follows:
#
#     find_package(PMDSDK)
#
# Variables used by this module, they can change the default behaviour and need
# to be set before calling find_package:
#
#  PMDSDK_ROOT_DIR  Set this variable to the root installation of
#                   PMDSDK if the module has problems finding
#                   the proper installation path.
#
# Variables defined by this module:
#
#  PMDSDK_FOUND              System has PMDSDK libs/headers
#  PMDSDK_LIBRARIES          The PMDSDK libraries
#  PMDSDK_INCLUDE_DIR        The location of PMDSDK headers

find_path(PMDSDK_ROOT_DIR
    NAMES include/pmdsdk2.h
)

find_library(PMDSDK_LIBRARIES
    NAMES pmdaccess2
    HINTS ${PMDSDK_ROOT_DIR}/lib /usr/local/pmd/lib $ENV{PMDDIR}/lib
)

find_path(PMDSDK_INCLUDE_DIR
    NAMES pmdsdk2.h
    HINTS ${PMDSDK_ROOT_DIR}/include /usr/local/pmd/include $ENV{PMDDIR}/include
)

if(CMAKE_SIZEOF_VOID_P EQUAL 8)
    set(PMDSDK_PLUGIN_NAMES camboardnano.L64.pap camboardnanoproc.L64.ppp)
elseif(CMAKE_SIZEOF_VOID_P EQUAL 4)
    set(PMDSDK_PLUGIN_NAMES camboardnano.L32.pap camboardnanoproc.L32.ppp)
else()
    message(ERROR "Unable to determine whether the OS is 32 or 64 bit.")
endif()

find_path(PMDSDK_PLUGIN_DIR 
    NAMES ${PMDSDK_PLUGIN_NAMES}
    HINTS ${PMDSDK_ROOT_DIR}/lib ${PMDSDK_ROOT_DIR}/plugins
          /usr/local/pmd/lib /usr/local/pmd/plugins
          $ENV{PMDDIR}/lib $ENV{PMDDIR}/plugins
)

message(STATUS "PLUGIN_DIR = ${PMDSDK_PLUGIN_DIR}")

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(PMDSDK DEFAULT_MSG
    PMDSDK_LIBRARIES
    PMDSDK_INCLUDE_DIR
    PMDSDK_PLUGIN_DIR
)

mark_as_advanced(
    PMDSDK_ROOT_DIR
    PMDSDK_LIBRARIES
    PMDSDK_INCLUDE_DIR
    PMDSDK_PLUGIN_DIR
)