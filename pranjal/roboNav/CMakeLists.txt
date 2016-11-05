cmake_minimum_required(VERSION 2.8.12)

project(roboNav)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -std=c++11 -DDEBUG")

find_package(OpenCV REQUIRED)
find_package(Ceres REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS}
		    include
		    thirdParty/DBoW2
		    thirdParty/DUtils
		    thirdParty/ceres5pt/include)

add_library(helpers-cc thirdParty/ceres5pt/src/triangulation.cc
                       thirdParty/ceres5pt/src/sequential_probability_ratio.cc
                       thirdParty/ceres5pt/src/random.cc
                       thirdParty/ceres5pt/src/pose_util.cc
                       thirdParty/ceres5pt/src/fundamental_matrix_util.cc
                       thirdParty/ceres5pt/src/five_point.cpp
                       thirdParty/ceres5pt/src/estimate_uncalibrated_relative_pose.cc
                       thirdParty/ceres5pt/src/estimate_twoview_info.cc
                       thirdParty/ceres5pt/src/estimate_relative_pose.cpp
                       thirdParty/ceres5pt/src/essential_matrix_utils.cc
                       thirdParty/ceres5pt/src/eight_point_fundamental_matrix.cc
                       thirdParty/ceres5pt/src/correspondance.cc
                       thirdParty/ceres5pt/src/helpers.cc)

add_library(helpers src/cell.cpp
					src/edge.cpp
					src/IOfunc.cpp
					src/keyFrame.cpp
					src/keyPoint.cpp)

add_library(dbow2-dutils thirdParty/DUtils/Random.cpp
		         thirdParty/DUtils/Timestamp.cpp
				 thirdParty/DBoW2/BowVector.cpp
				 thirdParty/DBoW2/FeatureVector.cpp
				 thirdParty/DBoW2/FORB.cpp
				 thirdParty/DBoW2/main.cpp
				 thirdParty/DBoW2/QueryResults.cpp
				 thirdParty/DBoW2/ScoringObject.cpp)

add_executable(robo examples/main.cpp)
add_executable(testing.out test.cpp)

target_link_libraries(robo  ${OpenCV_LIBS} 
							helpers 
							dbow2-dutils
							helpers-cc
							${CERES_LIBRARIES}
							glog
							gflags)

target_link_libraries(testing.out ${OpenCV_LIBS})

