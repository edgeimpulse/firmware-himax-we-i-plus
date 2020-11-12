cmake_minimum_required(VERSION 3.12)

# set the project name
project(edge-impulse-standalone-himax)

# Toolchain
set(CMAKE_C_COMPILER ccac)
set(CMAKE_CXX_COMPILER ccac)
set(CMAKE_CROSSCOMPILING 1)

# add the sources
file(GLOB EI_SDK "edge-impulse-sdk/porting/himax/*.cpp"
				 "ingestion-sdk-platform/himax/*.cpp"
				 "ingestion-sdk-platform/himax/*.cpp"
				 "repl/*.cpp"
				 "sensors/*.cpp")

file(GLOB TF "edge-impulse-sdk/tensorflow/lite/micro/*.cc"
			"edge-impulse-sdk/tensorflow/lite/micro/memory_planner/*.cc"
            "tflite-model/*.cpp"
			"edge-impulse-sdk/tensorflow/lite/micro/kernels/*.cc"
			"edge-impulse-sdk/tensorflow/lite/core/api/*.cc"
			"edge-impulse-sdk/tensorflow/lite/c/*.c"
			"edge-impulse-sdk/tensorflow/lite/kernels/internal/*.cc"
            "edge-impulse-sdk/tensorflow/lite/kernels/*.cc")
            #"edge-impulse-sdk/tensorflow/lite/micro/kernels/arc_mli/*.cc")

file(GLOB DSP "edge-impulse-sdk/dsp/kissfft/*.cpp"
			"edge-impulse-sdk/dsp/dct/*.cpp")

set(SOURCE_FILES ${EI_SDK} ${TF} ${DSP})

# include directories
include_directories(./)
include_directories(edge-impulse-sdk/)
include_directories(edge-impulse-sdk/classifier)
include_directories(edge-impulse-sdk/porting)
include_directories(edge-impulse-sdk/dsp)
include_directories(edge-impulse-sdk/dsp/kissfft)
include_directories(edge-impulse-sdk/dsp/dct)
include_directories(edge-impulse-sdk/third_party/flatbuffers/include/)
include_directories(edge-impulse-sdk/third_party/gemmlowp)
include_directories(edge-impulse-sdk/third_party/ruy)
include_directories(arc_mli_package/include)
include_directories(ingestion-sdk-platform/himax)
include_directories(repl)
include_directories(ingestion-sdk-c)
include_directories(sensors)

# add the executable
add_executable(${PROJECT_NAME}.elf main.cc ei_run_impulse.cpp ${SOURCE_FILES})


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
-DSCRATCH_MEM_Z_SIZE=0x10000 \
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
-DEIDSP_QUANTIZE_FILTERBANK=0")

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
-Hnocopyr \
-Hpurge \
-Hcl \
-fslp-vectorize-aggressive \
-ffunction-sections \
-fdata-sections \
-tcf_core_config \
-DEIDSP_QUANTIZE_FILTERBANK=0")


set(CMAKE_EXE_LINKER_FLAGS_INIT "${LINKER_FLAGS}")
set(CMAKE_EXE_LINKER_FLAGS "${LINKER_FLAGS} \
-Hheap=8192 \
-tcf=../arcem9d_wei_r16.tcf \
-Hnocopyr \
-m \
-Hldopt=-Coutput=${PROJECT_NAME}.map \
../memory.lcf \
-Hldopt=-Bgrouplib \
../libembarc.a \
../libbss.a \
../arc_mli_package/bin/himax_arcem9d_r16/release/libmli.a ")


add_custom_target(image DEPENDS ${PROJECT_NAME}.elf)
add_custom_command(TARGET image USES_TERMINAL 
	COMMAND cp ${PROJECT_NAME}.elf ../image_gen_linux_v3
	COMMAND cp ${PROJECT_NAME}.map ../image_gen_linux_v3
	COMMAND ${CMAKE_COMMAND} ARGS -E chdir ../image_gen_linux_v3 image_gen -e ${PROJECT_NAME}.elf -m ${PROJECT_NAME}.map -o out.img
	COMMAND ${CMAKE_COMMAND} ARGS -E chdir ../image_gen_linux_v3 rm ${PROJECT_NAME}.elf
	COMMAND ${CMAKE_COMMAND} ARGS -E chdir ../image_gen_linux_v3 rm ${PROJECT_NAME}.map)

#add_custom_command(TARGET image USES_TERMINAL COMMAND cp ${PROJECT_NAME}.map ../image_gen_linux_v3)

#add_custom_command(TARGET image USES_TERMINAL COMMAND ${CMAKE_COMMAND} ARGS -E chdir ../image_gen_linux_v3 image_gen -e ${PROJECT_NAME}.elf -m ${PROJECT_NAME}.map -o out.img)
