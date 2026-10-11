cmake_minimum_required(VERSION 3.8)
project(vision_exercise_1)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(OpenCV REQUIRED)

add_executable(vision_exercise_1 src/exercise_1.cpp)
target_include_directories(vision_exercise_1 PUBLIC ${OpenCV_INCLUDE_DIRS})
target_link_libraries(vision_exercise_1 ${OpenCV_LIBS})

install(TARGETS vision_exercise_1 DESTINATION lib/${PROJECT_NAME})

# Build tests if testing is enabled
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
