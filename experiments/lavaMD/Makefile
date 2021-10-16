#
# Makefile
#

CIRCLEHOME = ../..

OBJS	= main.o kernel.o kernel_cpu.o

LIBS	= ../libsdcard.a \
		../softserial.a \
	  $(CIRCLEHOME)/lib/fs/fat/libfatfs.a \
	  $(CIRCLEHOME)/lib/fs/libfs.a \
	  $(CIRCLEHOME)/lib/libcircle.a

include $(CIRCLEHOME)/Rules.mk

-include $(DEPS)
