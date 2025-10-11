# 设置 Zephyr 基础路径
set(ENV{ZEPHYR_BASE} "${CMAKE_CURRENT_SOURCE_DIR}/../zephyr")

# 导出编译命令
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -save-temps")

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -save-temps")

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_STANDARD 14)

set(CMAKE_C_EXTENSIONS OFF)
set(CMAKE_CXX_EXTENSIONS OFF)

# 查找 clang-tidy 可执行文件
find_program(CLANG_TIDY_BIN NAMES "clang-tidy")

# 如果找到 clang-tidy，则配置自动整理代码
if(CLANG_TIDY_BIN)
  # 定义要检查的文件类型
  file(GLOB_RECURSE TIDY_SOURCES "${CMAKE_CURRENT_SOURCE_DIR}/drivers/**/*.c"
       "${CMAKE_CURRENT_SOURCE_DIR}/drivers/**/*.cpp"
       "${CMAKE_CURRENT_SOURCE_DIR}/include/zephyr/drivers/*.h")
endif() # 结束 if(CLANG_TIDY_BIN)

# 包含目录
zephyr_include_directories(include)
zephyr_syscall_include_directories(include)

# 添加子目录
add_subdirectory(drivers)
add_subdirectory(lib)
add_subdirectory(include)
