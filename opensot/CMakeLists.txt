#  This program is distributed in the hope that it will be useful,
#  but WITHOUT ANY WARRANTY; without even the implied warranty of
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
#  GNU Lesser General Public License for more details.
#
#  You should have received a copy of the GNU Lesser General Public License
#  along with this program. If not, see <http://www.gnu.org/licenses/>
#

# CMake 3.5.1 required 
cmake_minimum_required(VERSION 3.5.1)

# XBot RT Plugin
project(simpleIK)

# C++11
include(CheckCXXCompilerFlag)
check_cxx_compiler_flag("-std=c++11" COMPILER_SUPPORTS_CXX11)
check_cxx_compiler_flag("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
elseif(COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
else()
    message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

add_compile_options("-fPIC")

# Add cmake dir and include custom macros
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/cmake")
include(MacroYCMInstallLibrary)

# find required packages
find_package(XCM REQUIRED)
find_package(OpenSoT REQUIRED)

# XENOMAI
set(ENABLE_XENO CACHE BOOL "ON")
if ( ${ENABLE_XENO} )
   find_package(Xenomai)
endif()

# include
include_directories(include ${XCM_INCLUDE_DIRS})

# lib
add_library(${PROJECT_NAME} SHARED
    src/simpleIK_plugin.cpp
)
add_library(IKProblem STATIC
    src/IKProblem.cpp
)

add_library(simpleID SHARED
    src/simpleID_plugin.cpp
)
add_library(IDProblem STATIC
    src/IDProblem.cpp
)

# set Xenomain flags if Xenomai is found
if ( Xenomai_FOUND )
    set_xeno_flags(${PROJECT_NAME})
endif()

# linking
target_link_libraries(IKProblem ${OpenSoT_LIBRARIES} ${XCM_LIBRARIES})
target_link_libraries(${PROJECT_NAME} IKProblem ${XCM_LIBRARIES})

target_link_libraries(IDProblem ${OpenSoT_LIBRARIES} ${XCM_LIBRARIES})
target_link_libraries(simpleID IDProblem ${XCM_LIBRARIES})


#install
ycm_library_install(IKProblem 1 0 0)
ycm_library_install(${PROJECT_NAME} 1 0 0)

ycm_library_install(IDProblem 1 0 0)
ycm_library_install(simpleID 1 0 0)

