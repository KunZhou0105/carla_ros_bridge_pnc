add_subdirectory(matplot)

add_library(plot plot.cpp)
find_package(Eigen3)
target_include_directories(plot PUBLIC 
                            ${PROJECT_SOURCE_DIR}#工程目录
                            "usr/include"#系统目录
                            "usr/local/include"#安装文件目录
                             "/usr/include/python2.7"#python目录
)


target_link_libraries(plot PUBLIC
    matplotlib_cpp
    Eigen3::Eigen
)