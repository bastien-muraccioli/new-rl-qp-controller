# ---- ONNX Runtime (local vendored copy) ----
set(ONNXRUNTIME_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/../ext/onnxruntime)
set(ONNXRUNTIME_INCLUDE_DIR ${ONNXRUNTIME_ROOT}/include)
set(ONNXRUNTIME_LIB_DIR ${ONNXRUNTIME_ROOT}/lib)

find_library(ONNXRUNTIME_LIB
  NAMES onnxruntime
  PATHS ${ONNXRUNTIME_LIB_DIR}
  NO_DEFAULT_PATH
  REQUIRED
)

set(controller_SRC
  NewRLQPController.cpp
  ConfigurationHelpers.cpp
  observation/Observation.cpp
  observation/Observations.cpp
  observation/ObservationManager.cpp
  policy/PolicyConfig.cpp
  policy/RLPolicyInterface.cpp
  policy/RLPolicyRuntime.cpp
  policy/RLStateRunner.cpp
)

set(controller_HDR
  ../include/NewRLQPController.h
  ../include/api.h
  ../include/ConfigurationHelpers.h
  ../include/observation/Observation.h
  ../include/observation/Observations.h
  ../include/observation/ObservationManager.h
  ../include/policy/PolicyConfig.h
  ../include/policy/RLPolicyInterface.h
  ../include/policy/RLPolicyRuntime.h
  ../include/policy/RLStateRunner.h
)

add_library(${PROJECT_NAME} SHARED ${controller_SRC} ${controller_HDR})
set_target_properties(${PROJECT_NAME} PROPERTIES COMPILE_FLAGS
                                                 "-DNewRLQPController_EXPORTS")

target_link_libraries(${PROJECT_NAME} PUBLIC mc_rtc::mc_control_fsm)

target_link_libraries(${PROJECT_NAME}
  PUBLIC
    ${ONNXRUNTIME_LIB}
)

target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${ONNXRUNTIME_INCLUDE_DIR}>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)

install(
  TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION "${MC_RTC_LIBDIR}"
  LIBRARY DESTINATION "${MC_RTC_LIBDIR}"
  RUNTIME DESTINATION bin)

install(
  DIRECTORY ${ONNXRUNTIME_LIB_DIR}/
  DESTINATION ${MC_RTC_LIBDIR}
  FILES_MATCHING
    PATTERN "libonnxruntime*.so*"
)

add_controller(${PROJECT_NAME}_controller lib.cpp "")
target_link_libraries(${PROJECT_NAME}_controller PUBLIC ${PROJECT_NAME})

add_subdirectory(states)
