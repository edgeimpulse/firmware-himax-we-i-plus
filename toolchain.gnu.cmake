SET(CMAKE_SYSTEM_NAME Generic)

# Prevent CMake from testing the compiler
SET (CMAKE_C_COMPILER_WORKS 1)
SET (CMAKE_CXX_COMPILER_WORKS 1)

# Toolchain
set(CMAKE_C_COMPILER arc-elf32-gcc)
set(CMAKE_CXX_COMPILER arc-elf32-g++)
set(CMAKE_CROSSCOMPILING 1)

set(CMAKE_C_FLAGS "\
-mcpu=em4_fpus \
-mlittle-endian \
-mcode-density \
-mdiv-rem \
-mswap \
-mnorm \
-mmpy-option=6 \
-mbarrel-shifter \
-mfpu=fpus_all \
-fno-unwind-tables \
-ffunction-sections \
-fmessage-length=0 \
-DREDUCE_CODESIZE \
-DTF_LITE_STATIC_MEMORY \
-DTF_LITE_DISABLE_X86_NEON \
-O3 \
-mxy \
-include ../core_config.h \
-Wstrict-aliasing \
-Werror \
-Wsign-compare \
-Wdouble-promotion \
-Wshadow \
-Wunused-variable \
-Wmissing-field-initializers \
-Wunused-function \
-DSCRATCH_MEM_Z_SIZE=0 \
-DNDEBUG \
-g \
-DCPU_ARC \
-fdata-sections \
-DEI_PORTING_HIMAX=1 \
-DEI_C_LINKAGE=1 \
-DEIDSP_QUANTIZE_FILTERBANK=0 \
-DEI_CLASSIFIER_ALLOCATION_STATIC_HIMAX_GNU \
-DEI_DSP_IMAGE_BUFFER_STATIC_SIZE=1024")

set(CMAKE_CXX_FLAGS "\
-fno-rtti \
-fno-exceptions \
-fno-threadsafe-statics \
-fno-unwind-tables \
-ffunction-sections \
-fdata-sections \
-fmessage-length=0 \
-DSCRATCH_MEM_Z_SIZE=0 \
-DNDEBUG \
-DEI_PORTING_HIMAX=1 \
-DEI_C_LINKAGE=1 \
-DTF_LITE_STATIC_MEMORY \
-DTF_LITE_DISABLE_X86_NEON \
-DEIDSP_QUANTIZE_FILTERBANK=0 \
-DEI_SENSOR_AQ_STREAM=FILE \
-DEI_CLASSIFIER_ALLOCATION_STATIC_HIMAX_GNU \
-DEI_DSP_IMAGE_BUFFER_STATIC_SIZE=1024 \
-O3 \
-Wsign-compare \
-Wdouble-promotion \
-Wshadow \
-Wunused-variable \
-Wmissing-field-initializers \
-Wunused-function \
-Wswitch \
-Wvla \
-Wall \
-Wstrict-aliasing \
-Wno-unused-parameter \
-DREDUCE_CODESIZE \
-mxy \
-include ${CMAKE_SOURCE_DIR}/core_config.h \
-mcpu=em4_fpus \
-mlittle-endian \
-mcode-density \
-mdiv-rem \
-mswap \
-mnorm \
-mmpy-option=6 \
-mbarrel-shifter \
-mfpu=fpus_all \
-g \
-DCPU_ARC")

if(DEFINED DISABLE_WARNINGS)
    add_compile_options(-w)
endif(DEFINED DISABLE_WARNINGS)

set(MY_LINKER_FLAGS "\
-Wl,-lmli -Wl,-lmwdepend -Wl,-marcv2elfx -Wl,-Map=${PROJECT_NAME}.map -Wl,--strip-debug -Wl,--stats,--gc-sections -Wl,--cref \
-L${CMAKE_SOURCE_DIR}/ \
-L${CMAKE_SOURCE_DIR}/arc_mli_package/bin/himax_arcem9d_r16/release \
-L${CMAKE_SOURCE_DIR}/mw_gnu_dependencies/gnu_depend_lib \
-Wl,--start-group \
${CMAKE_SOURCE_DIR}/libcpuarc.a \
${CMAKE_SOURCE_DIR}/libbss.a \
${CMAKE_SOURCE_DIR}/libboard_open_socket.a \
${CMAKE_SOURCE_DIR}/libboard_socket.a \
${CMAKE_SOURCE_DIR}/liblibcommon.a \
${CMAKE_SOURCE_DIR}/liblibaudio.a \
${CMAKE_SOURCE_DIR}/liblibsecurity.a \
${CMAKE_SOURCE_DIR}/liblibsensordp.a \
${CMAKE_SOURCE_DIR}/liblibtflm.a \
-Wl,--end-group")
