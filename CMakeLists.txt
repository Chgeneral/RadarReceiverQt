cmake_minimum_required(VERSION 3.19)
project(RadarDataReceiver LANGUAGES CXX C)

#C++规范标准
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC ON)

find_package(Qt6 6.5 REQUIRED COMPONENTS
    Core
    Gui
    Widgets
    SerialPort
    Charts
)

#DSP库路径
set(CMSIS_DSP_PATH "${CMAKE_CURRENT_SOURCE_DIR}/CMSIS_DSP/" CACHE PATH "CMSIS-DSP library path")

# 添加CMSIS-DSP源文件
file(GLOB CMSIS_DSP_SOURCES
    "${CMSIS_DSP_PATH}/Source/TransformFunctions/*.c"
    "${CMSIS_DSP_PATH}/Source/FilteringFunctions/*.c"
    "${CMSIS_DSP_PATH}/Source/CommonTables/*.c"
    "${CMSIS_DSP_PATH}/Source/BasicMathFunctions/*.c"
    "${CMSIS_DSP_PATH}/Source/ComplexMathFunctions/arm_*.c"
    "${CMSIS_DSP_PATH}/Source/MatrixFunctions/arm_*.c"
    "${CMSIS_DSP_PATH}/Source/FastMathFunctions/arm_*.c"
    "${CMSIS_DSP_PATH}/Source/StatisticsFunctions/arm_*.c"
    "${CMSIS_DSP_PATH}/Source/SupportFunctions/arm_*.c"
)

# 排除聚合文件（避免之前的重定义问题）
list(REMOVE_ITEM CMSIS_DSP_SOURCES
    "${CMSIS_DSP_PATH}/Source/BasicMathFunctions/BasicMathFunctions.c"
    "${CMSIS_DSP_PATH}/Source/CommonTables/CommonTables.c"
    "${CMSIS_DSP_PATH}/Source/FilteringFunctions/FilteringFunctions.c"
    "${CMSIS_DSP_PATH}/Source/TransformFunctions/TransformFunctions.c"
)

qt_standard_project_setup()

qt_add_executable(RadarDataReceiver
    WIN32 MACOSX_BUNDLE
    mainwindow.h
    radar_processor.h
    main.cpp
    mainwindow.cpp
    radar_processor.cpp
    mainwindow.ui
    vital_signs_detector.h
    vital_sign_detector.cpp
    UsbReceiver.h
    UsbReceiver.cpp
    ${CMSIS_DSP_SOURCES}

)


target_include_directories(RadarDataReceiver PRIVATE
    ${CMSIS_DSP_PATH}/Include
    ${CMSIS_DSP_PATH}/PrivateInclude
    ${CMSIS_DSP_PATH}/Include/dsp
    ${CMAKE_CURRENT_SOURCE_DIR}/usb3p0 #添加库文件路径
)

link_directories(${CMAKE_CURRENT_SOURCE_DIR}/usb3p0)

# 添加源文件到目标（确保只添加一次）
target_sources(RadarDataReceiver PRIVATE ${CMSIS_DSP_SOURCES})

target_link_libraries(RadarDataReceiver
    PRIVATE
        Qt::Core
        Qt::Gui
        Qt::Widgets
        Qt::SerialPort
        Qt::Charts
        ${CMAKE_CURRENT_SOURCE_DIR}/usb3p0/CyAPI.lib
        SetupAPI
        legacy_stdio_definitions
)

include(GNUInstallDirs)

install(TARGETS RadarDataReceiver
    BUNDLE  DESTINATION .
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
)

qt_generate_deploy_app_script(
    TARGET RadarDataReceiver
    OUTPUT_SCRIPT deploy_script
    NO_UNSUPPORTED_PLATFORM_ERROR
)
install(SCRIPT ${deploy_script})
