# TARGET:=sc002_charlieplex
ifeq ($(BUILD_ENV),OG)
TARGET:=sc002_charlieplex_alpha
else
TARGET:=sc002_charlieplex
endif


CH32V003FUN:=../ch32v003fun/ch32v003fun
MINICHLINK?=../ch32v003fun/minichlink

LDFLAGS+=-Wl,--print-memory-usage

# ADDITIONAL_C_FILES+=intflash.c

all : $(TARGET).bin

include $(CH32V003FUN)/ch32v003fun.mk

make_og : $(TARGETOG).bin
	$(FLASH_COMMAND)
flash : cv_flash
clean : cv_clean
