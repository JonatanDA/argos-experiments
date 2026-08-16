cmake_minimum_required(VERSION 3.10)
project(argos3_qupa CXX)

# Por defecto, simulador
if(NOT DEFINED ARGOS_BUILD_FOR)
  set(ARGOS_BUILD_FOR simulator)
endif()

# --- PkgConfig ---
find_package(PkgConfig REQUIRED)

# --- ARGoS core ---
pkg_check_modules(ARGOS3 REQUIRED argos3_simulator)
include_directories(${ARGOS3_INCLUDE_DIRS})
link_directories(${ARGOS3_LIBRARY_DIRS})

# --- Lua (5.3 o 5.2) ---
pkg_check_modules(LUA53 lua5.3)
pkg_check_modules(LUA52 lua5.2)
if(LUA53_FOUND)
  set(LUA_INCLUDE_DIRS ${LUA53_INCLUDE_DIRS})
  set(LUA_LIBRARIES    ${LUA53_LIBRARIES})
elseif(LUA52_FOUND)
  message(WARNING "Lua 5.3 no encontrada, usando Lua 5.2")
  set(LUA_INCLUDE_DIRS ${LUA52_INCLUDE_DIRS})
  set(LUA_LIBRARIES    ${LUA52_LIBRARIES})
else()
  message(FATAL_ERROR "No se encontró Lua (instala liblua5.3-dev o liblua5.2-dev).")
endif()
include_directories(${LUA_INCLUDE_DIRS})

# --- Detecta ARGoS Qt-OpenGL por headers (no siempre hay .pc) ---
find_path(ARGOS_QTOPENGL_INCLUDE_DIR
  NAMES argos3/plugins/simulator/visualizations/qt-opengl/qtopengl_widget.h
  PATHS ${ARGOS3_INCLUDE_DIRS}
)
if(ARGOS_QTOPENGL_INCLUDE_DIR)
  message(STATUS "Found ARGoS Qt-OpenGL headers: ${ARGOS_QTOPENGL_INCLUDE_DIR}")
  set(HAVE_ARGOS_QTOPENGL TRUE)
  set(CMAKE_AUTOMOC ON)
  find_package(Qt5 REQUIRED COMPONENTS Core Gui Widgets OpenGL)
  include_directories(${ARGOS_QTOPENGL_INCLUDE_DIR})
else()
  message(WARNING "ARGoS Qt-OpenGL headers NO encontrados: el QUPA no se dibujará en la GUI")
endif()

# -------- Headers / Sources --------
set(ARGOS3_HEADERS_PLUGINS_ROBOTS_QUPA_CONTROLINTERFACE
  control_interface/ci_qupa_base_ground_sensor.h
  control_interface/ci_qupa_distance_scanner_actuator.h
  control_interface/ci_qupa_distance_scanner_sensor.h
  control_interface/ci_qupa_encoder_sensor.h
  control_interface/ci_qupa_gripper_actuator.h
  control_interface/ci_qupa_light_sensor.h
  control_interface/ci_qupa_motor_ground_sensor.h
  control_interface/ci_qupa_proximity_sensor.h
  control_interface/ci_qupa_turret_actuator.h
  control_interface/ci_qupa_turret_encoder_sensor.h
)

set(ARGOS3_HEADERS_PLUGINS_ROBOTS_QUPA_SIMULATOR
  simulator/dynamics2d_qupa_model.h
  simulator/pointmass3d_qupa_model.h
  simulator/qupa_base_ground_rotzonly_sensor.h
  simulator/qupa_distance_scanner_default_actuator.h
  simulator/qupa_distance_scanner_equipped_entity.h
  simulator/qupa_distance_scanner_rotzonly_sensor.h
  simulator/qupa_entity.h
  simulator/qupa_gripper_default_actuator.h
  simulator/qupa_light_rotzonly_sensor.h
  simulator/qupa_motor_ground_rotzonly_sensor.h
  simulator/qupa_proximity_default_sensor.h
  simulator/qupa_turret_default_actuator.h
  simulator/qupa_turret_encoder_default_sensor.h
  simulator/qupa_turret_entity.h
)

set(ARGOS3_SOURCES_PLUGINS_ROBOTS_QUPA
  ${ARGOS3_HEADERS_PLUGINS_ROBOTS_QUPA_CONTROLINTERFACE}
  control_interface/ci_qupa_base_ground_sensor.cpp
  control_interface/ci_qupa_distance_scanner_actuator.cpp
  control_interface/ci_qupa_distance_scanner_sensor.cpp
  control_interface/ci_qupa_encoder_sensor.cpp
  control_interface/ci_qupa_gripper_actuator.cpp
  control_interface/ci_qupa_light_sensor.cpp
  control_interface/ci_qupa_motor_ground_sensor.cpp
  control_interface/ci_qupa_proximity_sensor.cpp
  control_interface/ci_qupa_turret_actuator.cpp
  control_interface/ci_qupa_turret_encoder_sensor.cpp

  ${ARGOS3_HEADERS_PLUGINS_ROBOTS_QUPA_SIMULATOR}
  simulator/dynamics2d_qupa_model.cpp
  simulator/pointmass3d_qupa_model.cpp
  simulator/qupa_base_ground_rotzonly_sensor.cpp
  simulator/qupa_distance_scanner_default_actuator.cpp
  simulator/qupa_distance_scanner_equipped_entity.cpp
  simulator/qupa_distance_scanner_rotzonly_sensor.cpp
  simulator/qupa_entity.cpp
  simulator/qupa_gripper_default_actuator.cpp
  simulator/qupa_light_rotzonly_sensor.cpp
  simulator/qupa_motor_ground_rotzonly_sensor.cpp
  simulator/qupa_proximity_default_sensor.cpp
  simulator/qupa_turret_default_actuator.cpp
  simulator/qupa_turret_encoder_default_sensor.cpp
  simulator/qupa_turret_entity.cpp
)

# Añade el renderer si detectamos headers + Qt
if(HAVE_ARGOS_QTOPENGL)
  list(APPEND ARGOS3_HEADERS_PLUGINS_ROBOTS_QUPA_SIMULATOR simulator/qtopengl_qupa.h)
  list(APPEND ARGOS3_SOURCES_PLUGINS_ROBOTS_QUPA          simulator/qtopengl_qupa.cpp)
endif()

# -------- Plugin --------
add_library(argos3plugin_${ARGOS_BUILD_FOR}_qupa SHARED ${ARGOS3_SOURCES_PLUGINS_ROBOTS_QUPA})

target_link_libraries(argos3plugin_${ARGOS_BUILD_FOR}_qupa
  argos3plugin_${ARGOS_BUILD_FOR}_genericrobot
  argos3plugin_${ARGOS_BUILD_FOR}_dynamics2d
  argos3plugin_${ARGOS_BUILD_FOR}_pointmass3d
  ${LUA_LIBRARIES}
)

if(HAVE_ARGOS_QTOPENGL)
  target_link_libraries(argos3plugin_${ARGOS_BUILD_FOR}_qupa
    argos3plugin_${ARGOS_BUILD_FOR}_qtopengl
    Qt5::Core Qt5::Gui Qt5::Widgets Qt5::OpenGL
  )
endif()

# -------- Install --------
install(FILES ${ARGOS3_HEADERS_PLUGINS_ROBOTS_QUPA_CONTROLINTERFACE}
        DESTINATION include/argos3/plugins/robots/qupa/control_interface)
install(FILES ${ARGOS3_HEADERS_PLUGINS_ROBOTS_QUPA_SIMULATOR}
        DESTINATION include/argos3/plugins/robots/qupa/simulator)

install(TARGETS argos3plugin_${ARGOS_BUILD_FOR}_qupa
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib/argos3
  ARCHIVE DESTINATION lib/argos3
)
