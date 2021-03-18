SET(CMAKE_SYSTEM_NAME Generic)

# Prevent CMake from testing the compiler
SET (CMAKE_C_COMPILER_WORKS 1)
SET (CMAKE_CXX_COMPILER_WORKS 1)

# Toolchain
set(CMAKE_C_COMPILER ccac)
set(CMAKE_CXX_COMPILER ccac)
set(CMAKE_CROSSCOMPILING 1)

set(CMAKE_C_FLAGS "\
-Wstrict-aliasing \
-DTF_LITE_STATIC_MEMORY \
-Werror \
-Wsign-compare \
-Wdouble-promotion \
-Wshadow \
-Wunused-variable \
-Wmissing-field-initializers \
-Wunused-function \
-DNDEBUG \
-O3 \
-fno-rtti \
-DSCRATCH_MEM_Z_SIZE=0 \
-DNDEBUG \
-g \
-DCPU_ARC \
-Hnosdata \
-DTF_LITE_STATIC_MEMORY \
-tcf=../arcem9d_wei_r16.tcf \
-Hnocopyr \
-Hpurge \
-Hcl \
-fslp-vectorize-aggressive \
-ffunction-sections \
-fdata-sections \
-tcf_core_config \
-DEIDSP_QUANTIZE_FILTERBANK=0 \
-DEI_C_LINKAGE=1 \
-DEI_CLASSIFIER_ALLOCATION_STATIC_HIMAX \
-DEI_DSP_IMAGE_BUFFER_STATIC_SIZE=1024")

set(CMAKE_CXX_FLAGS "\
-DTF_LITE_STATIC_MEMORY \
-DNDEBUG \
-O3 \
-DNDEBUG \
-g \
-DCPU_ARC \
-Hnosdata \
-DTF_LITE_STATIC_MEMORY \
-tcf=../arcem9d_wei_r16.tcf \
-DSCRATCH_MEM_Z_SIZE=0 \
-Hnocopyr \
-Hpurge \
-Hcl \
-fslp-vectorize-aggressive \
-ffunction-sections \
-fdata-sections \
-tcf_core_config \
-DEIDSP_QUANTIZE_FILTERBANK=0 \
-DEI_C_LINKAGE=1 \
-DEI_SENSOR_AQ_STREAM=FILE \
-DEI_CLASSIFIER_ALLOCATION_STATIC_HIMAX \
-DEI_DSP_IMAGE_BUFFER_STATIC_SIZE=1024")

set(MY_LINKER_FLAGS "\
-Hheap=65536 \
-tcf=../arcem9d_wei_r16.tcf \
-Hnocopyr \
-m \
-Hldopt=-Coutput=${PROJECT_NAME}.map \
../memory.lcf \
-Hldopt=-Bgrouplib \
../libembarc.a \
../libbss.a \
../arc_mli_package/bin/himax_arcem9d_r16/release/libmli.a")
