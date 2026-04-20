add_executable(test_fsm
    test_fsm.cpp
    ${CMAKE_SOURCE_DIR}/src/GuardianStateMachine.cpp
)
target_include_directories(test_fsm PRIVATE ${CMAKE_SOURCE_DIR}/include)
target_link_libraries(test_fsm PRIVATE Threads::Threads)
add_test(NAME test_fsm COMMAND test_fsm)

add_executable(test_interlock
    test_interlock.cpp
    ${CMAKE_SOURCE_DIR}/src/RobotInterlock.cpp
)
target_include_directories(test_interlock PRIVATE ${CMAKE_SOURCE_DIR}/include)
target_link_libraries(test_interlock PRIVATE Threads::Threads)
add_test(NAME test_interlock COMMAND test_interlock)

add_executable(test_vision
    test_vision.cpp
    ${CMAKE_SOURCE_DIR}/src/VisionProcessor.cpp
)
target_include_directories(test_vision PRIVATE
    ${CMAKE_SOURCE_DIR}/include
    ${OpenCV_INCLUDE_DIRS}
)
target_link_libraries(test_vision PRIVATE ${OpenCV_LIBS} Threads::Threads)
add_test(NAME test_vision COMMAND test_vision)
