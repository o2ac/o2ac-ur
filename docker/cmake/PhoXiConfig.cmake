# - Config file for the PhoXi package
# It defines the following variables
#  PHOXI_INCLUDE_DIRS - include directories for PhoXi
#  PHOXI_LIBRARY    - libraries to link against
#  PHOXI_EXECUTABLE_DIRS   - the PhoXi executable
#  PHOXI_FOUND

# Compute paths
get_filename_component(PhoXiConfig_PATH "${CMAKE_CURRENT_LIST_FILE}" PATH)
if(MSVC)
    if(MSVC14)
        set(PHO_COMPILER_VERSION "msvc14")

    elseif(MSVC12)
        set(PHO_COMPILER_VERSION "msvc12")
    endif()
elseif(CMAKE_COMPILER_IS_GNUCXX)
    if (CMAKE_CXX_COMPILER_VERSION VERSION_LESS "5.0")
        set(PHO_COMPILER_VERSION "gcc4.9.3")
    else()
        set(PHO_COMPILER_VERSION "gcc5.4.0")
    endif()
endif()

include("${PhoXiConfig_PATH}/cmake/PhoXiConfig_${PHO_COMPILER_VERSION}.cmake")

set(PHOXI_LIB_RELEASE "${PHOXI_LIB_RELEASE_PER_COMPILER}")
set(PHOXI_LIB_DEBUG "${PHOXI_LIB_DEBUG_PER_COMPILER}")

list(APPEND PHOXI_LIBRARY
    optimized "${PHOXI_LIB_RELEASE}"
    debug "${PHOXI_LIB_DEBUG}"
)

if(MSVC)
    set(PHOXI_DLL_RELEASE "${PHOXI_DLL_RELEASE_PER_COMPILER}")
    set(PHOXI_DLL_DEBUG "${PHOXI_DLL_DEBUG_PER_COMPILER}")
endif()

set(PHOXI_INCLUDE_DIRS "${PhoXiConfig_PATH}/API/include")
set(PHOXI_EXECUTABLE_DIRS "${PhoXiConfig_PATH}/API/bin")
set(PHO_SOFTWARE_VERSION_MAJOR "1")
set(PHO_SOFTWARE_VERSION_MINOR "1")
set(PHO_SOFTWARE_VERSION_PATCH "32")
set(PHO_SOFTWARE_VERSION "1.1.32")


if(MSVC)
    include("${PhoXiConfig_PATH}/cmake/PhoXiConfig_CSharp.cmake")
endif()

set(PHOXI_FOUND TRUE CACHE BOOL "" FORCE)
