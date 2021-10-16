#
# Makefile
#

CIRCLEHOME = ../

OBJS	= emmc.o mmchost.o sdhost.o


all: softserial.a libsdcard.a

libsdcard.a: $(OBJS)
	@echo "  AR    $@"
	@rm -f $@
	@$(AR) cr $@ $(OBJS)

softserial.a:  softserial.o
	@echo "  AR    $@"
	@rm -f $@
	@$(AR) cr $@ softserial.o

include $(CIRCLEHOME)/Rules.mk

-include $(DEPS)
