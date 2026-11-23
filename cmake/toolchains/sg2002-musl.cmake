set(CMAKE_SYSTEM_NAME Linux)
set(CMAKE_SYSTEM_PROCESSOR riscv64)

set(SG2002_TOOLCHAIN_BIN
    "/home/pyosik/MaixCDK/dl/extracted/toolchains/maixcam/host-tools/gcc/riscv64-linux-musl-x86_64/bin"
    CACHE PATH "SG2002 RISC-V musl 工具链 bin 目录")

set(CMAKE_C_COMPILER "${SG2002_TOOLCHAIN_BIN}/riscv64-unknown-linux-musl-gcc")
set(CMAKE_CXX_COMPILER "${SG2002_TOOLCHAIN_BIN}/riscv64-unknown-linux-musl-g++")
set(CMAKE_AR "${SG2002_TOOLCHAIN_BIN}/riscv64-unknown-linux-musl-ar")
set(CMAKE_RANLIB "${SG2002_TOOLCHAIN_BIN}/riscv64-unknown-linux-musl-ranlib")
set(CMAKE_STRIP "${SG2002_TOOLCHAIN_BIN}/riscv64-unknown-linux-musl-strip")

# 显式指定 SG2002 使用的 64 位 RISC-V、LP64D ABI。
set(CMAKE_C_FLAGS_INIT "-march=rv64gc -mabi=lp64d")
set(CMAKE_CXX_FLAGS_INIT "-march=rv64gc -mabi=lp64d")

set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

