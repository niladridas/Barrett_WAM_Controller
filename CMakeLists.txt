cmake_minimum_required(VERSION 2.6)
project(gwam-simulator)

set(MY_CODES_DIRS /home/nilxwam/workspace/gwam-simulator)

set(SAM_LIBS constants samlibs)

## libbarrett
find_package(Barrett REQUIRED)
include_directories(${BARRETT_INCLUDE_DIRS} ${MY_CODES_DIRS}/include)
link_directories(${BARRETT_LIBRARY_DIRS} ${MY_CODES_DIRS}/builds)
add_definitions(${BARRETT_DEFINITIONS})
add_definitions(-o2 -g)

## curses
find_package(Curses REQUIRED)
include_directories(${CURSES_INCLUDE_DIR})


#add_library(QuadTS ${MY_CODES_DIRS}/samLibs/QuadTS.cpp)
add_library(constants ${MY_CODES_DIRS}/samLibs/constants.cpp)
add_library(samlibs ${MY_CODES_DIRS}/samLibs/samlibs.cpp)

#add_library(Dynamics ${MY_CODES_DIRS}/samLibs/Dynamics.cpp)

function(add_programs)
	foreach (prog ${ARGV})
		add_executable(${prog} src/${prog}.cpp)
		target_link_libraries(${prog} ${BARRETT_LIBRARIES} ${SAM_LIBS})
	endforeach()
endfunction()

add_programs(
nilu_slidingmode
nilu_sliding_compensator_on
nilu_optimal_sliding_mode
eigen_test
ncurses_example
test_optimal_sliding
test_differentiator
another_controller
playTenisLog_nilu
test_tact_finger
DMP_sliding_mode
sliding_mode_disturbance_observer
tactile_feedback_move
higher_order_SMC
)

target_link_libraries(ncurses_example ${CURSES_LIBRARIES})
