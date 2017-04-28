cmake_minimum_required(VERSION 2.8.3)
project(um_acc)

# add c++11
set(CMAKE_CXX_FLAGS "-std=c++0x ${CMAKE_CXX_FLAGS}")

# Using OpenCV
find_package( OpenCV REQUIRED )

find_package(CUDA 6.5 REQUIRED)

include_directories(${CUDA_INCLUDE_DIRS} ${CUDA_CUT_INCLUDE_DIR})

# Depenencies
find_package(catkin REQUIRED COMPONENTS roscpp rosconsole std_msgs message_generation eiquadprog)

# Services and msgs
add_message_files(DIRECTORY msg FILES ControlSignal.msg CarState.msg Pwm.msg CarSignal.msg CarStateExtended.msg Sensor.msg CarState_L.msg Lane.msg)
add_service_files(DIRECTORY srv FILES ComputeControl.srv ComputeControl_L.srv)
generate_messages(DEPENDENCIES std_msgs um_acc)

# Declare catking package
catkin_package(CATKIN_DEPENDS roscpp std_msgs message_runtime)

# Declare and build targets
add_library(qp_vars STATIC src/qp_vars.cpp)
add_library(SimpleGPIO STATIC src/SimpleGPIO.cpp)
add_library(SimplePWM STATIC src/SimplePWM.cpp)
add_library(qphelper STATIC src/qphelper.cpp)
add_library(kalman STATIC src/kalman.cpp)
add_library(rs232 STATIC src/rs232.cpp)

include_directories(cudd-2.5.0/include ${catkin_INCLUDE_DIRS} ${OpenCV_DIR})

add_definitions(-lstdc++ -lm)

add_executable(compute_control_um src/compute_control_um.cpp)
add_dependencies(compute_control_um um_acc_generate_messages_cpp)
target_link_libraries(compute_control_um ${catkin_LIBRARIES} qphelper)
######

set(cudd_LIBRARIES_REL cudd dddmp epd util mtr st)
set(cudd_LIBRARIES)
foreach(lib ${cudd_LIBRARIES_REL})
  list(APPEND cudd_LIBRARIES ${CMAKE_CURRENT_SOURCE_DIR}/cudd-2.5.0/${lib}/lib${lib}.a)
endforeach()
#message("libs: ${cudd_LIBRARIES}")

#target_link_libraries(
 #compute_control_ucla
 #  ${catkin_LIBRARIES} 
 #  ${cudd_LIBRARIES}
 #)


#####

add_executable(Car_1 src/Car_1.cpp)
add_dependencies(Car_1 um_acc_generate_messages_cpp)
target_link_libraries(SimpleGPIO SimplePWM ${catkin_LIBRARIES})
target_link_libraries(Car_1 ${catkin_LIBRARIES} SimpleGPIO)

add_executable(Car_2 src/Car_2.cpp)
add_dependencies(Car_2 um_acc_generate_messages_cpp)
target_link_libraries(SimpleGPIO SimplePWM ${catkin_LIBRARIES})
target_link_libraries(Car_2 ${catkin_LIBRARIES} SimpleGPIO)

add_executable(Car_3 src/Car_3.cpp)
add_dependencies(Car_3 um_acc_generate_messages_cpp)
target_link_libraries(SimpleGPIO SimplePWM ${catkin_LIBRARIES})
target_link_libraries(Car_3 ${catkin_LIBRARIES} SimpleGPIO)

add_executable(Car_4 src/Car_4.cpp)
add_dependencies(Car_4 um_acc_generate_messages_cpp)
target_link_libraries(SimpleGPIO SimplePWM rs232 ${catkin_LIBRARIES})
target_link_libraries(Car_4 ${catkin_LIBRARIES} SimpleGPIO rs232)

add_executable(Car_id src/Car_id.cpp)
add_dependencies(Car_id um_acc_generate_messages_cpp)
target_link_libraries(SimpleGPIO SimplePWM rs232 ${catkin_LIBRARIES})
target_link_libraries(Car_id ${catkin_LIBRARIES} SimpleGPIO rs232)

add_executable(lane_publisher src/lane_publisher.cpp)
add_dependencies(lane_publisher um_acc_generate_messages_cpp)
target_link_libraries(lane_publisher ${catkin_LIBRARIES} ${OpenCV_LIBS})

add_executable(lane_publisher2 src/lane_publisher2.cpp)
add_dependencies(lane_publisher2 um_acc_generate_messages_cpp)
target_link_libraries(lane_publisher2 ${catkin_LIBRARIES} ${OpenCV_LIBS})

add_executable(compute_control_ucla src/compute_control_ucla.cpp)
add_dependencies(compute_control_ucla um_acc_generate_messages_cpp)
target_link_libraries(compute_control_ucla ${cudd_LIBRARIES} ${catkin_LIBRARIES} qphelper)

add_executable(car_simulator src/car_simulator.cpp)
add_dependencies(car_simulator um_acc_generate_messages_cpp)
target_link_libraries(car_simulator ${catkin_LIBRARIES})

add_executable(car_simulator_lanekeeping src/car_simulator_lanekeeping.cpp)
add_dependencies(car_simulator_lanekeeping um_acc_generate_messages_cpp)
target_link_libraries(car_simulator_lanekeeping ${catkin_LIBRARIES} qp_vars)

add_executable(compute_control_um_lanekeeping src/compute_control_um_lanekeeping.cpp)
add_dependencies(compute_control_um_lanekeeping src/compute_control_um_lanekeeping.cpp)
target_link_libraries(compute_control_um_lanekeeping ${catkin_LIBRARIES} qp_vars)

#add_executable(rc_interface_ucla src/rc_interface_ucla.cpp)
#add_dependencies(rc_interface_ucla um_acc_generate_messages_cpp)
#target_link_libraries(rc_interface_ucla ${cudd_LIBRARIES} ${catkin_LIBRARIES} kalman)

add_executable(rc_interface src/rc_interface.cpp)
add_dependencies(rc_interface um_acc_generate_messages_cpp)
target_link_libraries(rc_interface ${cudd_LIBRARIES} ${catkin_LIBRARIES} kalman)

add_executable(controller src/controller.cpp)
add_dependencies(controller um_acc_generate_messages_cpp)
target_link_libraries(controller ${catkin_LIBRARIES})

add_executable(controller_lanekeeping src/controller_lanekeeping.cpp)
add_dependencies(controller_lanekeeping um_acc_generate_messages_cpp)
target_link_libraries(controller_lanekeeping ${catkin_LIBRARIES})

add_executable(teleop_car src/teleop_car.cpp)
add_dependencies(teleop_car um_acc_generate_messages_cpp)
target_link_libraries(teleop_car ${catkin_LIBRARIES})

add_executable(teleop_car_control src/teleop_car_control.cpp)
add_dependencies(teleop_car_control um_acc_generate_messages_cpp)
target_link_libraries(teleop_car_control ${catkin_LIBRARIES})

add_executable(teleop_lanekeeping src/teleop_lanekeeping.cpp)
add_dependencies(teleop_lanekeeping um_acc_generate_messages_cpp)
target_link_libraries(teleop_lanekeeping ${catkin_LIBRARIES})

add_executable(car_data src/car_data.cpp)
add_dependencies(car_data um_acc_generate_messages_cpp)
target_link_libraries(car_data ${catkin_LIBRARIES})

## Specify libraries to link a library or executable target against



