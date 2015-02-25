cmake_minimum_required(VERSION 2.6)
project(Mycodes)

set(MY_CODES_DIRS /home/robot/sam)
set(SAM_LIBS)
#set(SAM_LIBS ServeSocket Socket)
#set(ADD_INC_DIRS /usr/include/log4cxx/helpers)





## libbarrett
find_package(Barrett REQUIRED)
include_directories(${BARRETT_INCLUDE_DIRS} ${MY_CODES_DIRS}/include)
link_directories(${BARRETT_LIBRARY_DIRS} ${MY_CODES_DIRS}/builds)
add_definitions(${BARRETT_DEFINITIONS})
add_definitions(-o2 -g)
## curses
find_package(Curses REQUIRED)
include_directories(${CURSES_INCLUDE_DIR})


#add_library(Socket ${MY_CODES_DIRS}/src/Socket.cpp)

#add_library(ServerSocket ${MY_CODES_DIRS}/src/ServerSocket.cpp)

function(add_programs)
	foreach (prog ${ARGV})
		add_executable(${prog} main/${prog}.cpp)
		target_link_libraries(${prog} ${BARRETT_LIBRARIES} ${SAM_LIBS})
	endforeach()
endfunction()
add_programs(
#	nilex01
#	move_hand2
#	move_hand
#	testMain
#	hand_test
#	ex08_teach_and_play
#	ex12
#	ex13
#	FinalShow
#	tact
#	tact_new
#	grasp_move_hand
#	hand_control_complete
#       data_collection
#	display_jp
#	playTenisLog
#	sample_torque
#	testmain2
#	gmm_traj_test
#	network_test
# 	test_torque
#	interactive_torque
#	theta2sinu
#	outside_torque
#	theta_2_4
#	nilu_slidingmode
#	robowamcod_1
#	playTenisLog_nilu
#	Nil_C_1
	nilu_optimal_sliding_mode
)

#target_link_libraries(test_ex04 ${CURSES_LIBRARIES})
