project(hikvisioncam)
set(LIBHIKVISION MediaProcess MvCameraControl
        MVGigEVisionSDK MVRender MvUsb3vTL)
#set(HIKVISION_PATH ${PROJECT_SOURCE_DIR}/libhikvision/lib64)
find_package(OpenCV REQUIRED)
#link_directories(${PROJECT_SOURCE_DIR}/libhikvision/lib64)
#include_directories(${PROJECT_SOURCE_DIR}/libhikvision/ImageProcess)
#file(COPY  ${LIBHIKVISION} DESTINATION lib FOLLOW_SYMLINK_CHAIN)

add_library(hikvisioncam SHARED hikvisioncam.cpp)
# target_link_libraries(tdt_hikvisioncam PRIVATE ${HIKVISION_PATH}/${HIKVISION_LIBS} ${OpenCV_LIBS})
target_link_libraries(hikvisioncam PRIVATE ${LIBHIKVISION} ${OpenCV_LIBS} -lpthread -lm)