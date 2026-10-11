cmake_minimum_required(VERSION 3.8)
project(vision_exercise_3)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Dependencies
find_package(ament_cmake REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS})

# Executable
add_executable(vision_exercise_3 src/exercise_3.cpp)
target_compile_features(vision_exercise_3 PUBLIC cxx_std_17)
target_link_libraries(vision_exercise_3 ${OpenCV_LIBS})

install(TARGETS vision_exercise_3
  DESTINATION lib/${PROJECT_NAME}
)

# Build tests if testing is enabled
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
