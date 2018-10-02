include(ExternalProject)
set(autodifk_LIBRARIES "")

# Find Eigen3.
find_package( Eigen3 REQUIRED )
include_directories(SYSTEM ${EIGEN3_INCLUDE_DIR})
list(APPEND autodifk_LIBRARIES ${EIGEN3_LIBRARIES})

# Find Google-gflags.
include("cmake/Modules/FindGflags.cmake")
include_directories(SYSTEM ${GFLAGS_INCLUDE_DIRS})
list(APPEND autodifk_LIBRARIES ${GFLAGS_LIBRARIES})

# Find Google-glog.
include("cmake/Modules/FindGlog.cmake")
include_directories(SYSTEM ${GLOG_INCLUDE_DIRS})
list(APPEND autodifk_LIBRARIES ${GLOG_LIBRARIES})
