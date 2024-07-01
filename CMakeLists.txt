cmake_minimum_required(VERSION 3.10)
project(GraphAlgorithms)

set(CMAKE_CXX_STANDARD 11) # Ensure C++11 or later is used

# Add all your source files (both .cpp and .h files)
set(SOURCE_FILES
    Graph.h
    Graph.inl
    main.cpp
)

# Create an executable from your source files
add_executable(GraphAlgorithms ${SOURCE_FILES})
