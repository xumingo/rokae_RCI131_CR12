cmake_minimum_required(VERSION 3.14)

project(robotCR12_console_cmake LANGUAGES CXX)

set(CMAKE_INCLUDE_CURRENT_DIR ON)

set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(QT NAMES Qt6 Qt5 COMPONENTS Core REQUIRED)
find_package(Qt${QT_VERSION_MAJOR} COMPONENTS Core REQUIRED)

# 设置头文件的包含路径
include_directories(
    ${CMAKE_SOURCE_DIR}/rci_client/inc/include
    ${CMAKE_SOURCE_DIR}/rci_client/inc/rci_data
)

#20241213 xuminghao
LINK_DIRECTORIES(
    ${PROJECT_SOURCE_DIR}/rci_client/lib
)

# 指定外部库的路径
set(LIB_DIR "D:/dev/rokae/20241212_xCoreRCI131_Qt/robotCR12_console/robotCR12_console/rci_client/lib")

LINK_DIRECTORIES(${LIB_DIR})

# 添加源文件
set(SOURCES
    main.cpp
#    Robot.cpp  # 确保包含 Robot 实现文件
    # 加入其他源文件...
)

add_executable(robotCR12_console_cmake
  main.cpp
)
target_link_libraries(robotCR12_console_cmake Qt${QT_VERSION_MAJOR}::Core)

#20241213 xuminghao

# 添加需要链接的库
target_link_libraries(robotCR12_console_cmake
    RCI_Client
    xMateModel
    orocos-kdl
    glog
)
