
export MODULE_acvImage=$(abspath acvImage)
export MODULE_MLNN=$(abspath MLNN)

CXX=g++
CC=gcc
GPMF_parser_PATH=..

target_bin=test01
ODIR=obj
VPATH=$(GPMF_parser_PATH)/ $(GPMF_parser_PATH)/demo Quaternion_Library
IDIR=./ $(VPATH)
_OBJ = GPMF_demo.o GPMF_parser.o GPMF_mp4reader.o GPMF_print.o imu_data_process.o mtQuaternions.o mtVector.o


ESS_TRACK= $(shell find $(IDIR) -name "*.h")
export MakeTemplate:= $(abspath Makefile.in)
export FLAGS= -w -O3 -fpermissive
include $(MakeTemplate)
