cmake_minimum_required( VERSION 2.8 )

project( CCBS )

set(Boost_USE_STATIC_LIBS OFF) 
find_package(Boost COMPONENTS multi_index)
include_directories( ${Boost_INCLUDE_DIRS} )
include_directories( "." )

set( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -O2 -Wall -Wextra" )
set( CMAKE_EXE_LINKER_FLAGS "-static -static-libgcc -static-libstdc++" )

set( SOURCE_FILES 
     src/main.cpp  
	src/config.cpp
     src/tinyxml2.cpp
     src/xml_logger.cpp 
     src/map.cpp 
     src/heuristic.cpp 
     src/sipp.cpp 
     src/task.cpp
     src/cbs.cpp 
	simplex/columnset.cpp
	simplex/constraint.cpp
	simplex/matrix.cpp
	simplex/objectivefunction.cpp
	simplex/simplex.cpp
	simplex/variable.cpp)

set( HEADER_FILES 
     src/const.h  
     src/tinyxml2.h
	src/config.h
     src/xml_logger.h 
     src/map.h 
     src/heuristic.h 
     src/sipp.h 
     src/task.h
     src/cbs.h
	src/structs.h
	simplex/columnset.h
	simplex/constraint.h
	simplex/datamismatchexception.h
	simplex/divisionbyzeroexception.h
	simplex/indexoutofboundexception.h
	simplex/matrix.h
	simplex/matrixissingularexception.h
	simplex/matrixnotsquareexception.h
     simplex/memoryreachedoutexception.h
     simplex/notavectorexception.h
     simplex/objectivefunction.h
     simplex/pilal.h
     simplex/pilalexceptions.h
	simplex/simplex.h
	simplex/simplexexceptions.h
	simplex/sizemismatchexception.h
	simplex/variable.h)

add_executable( CCBS ${SOURCE_FILES} ${HEADER_FILES} )
