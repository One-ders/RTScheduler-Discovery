
DRIVERS+=stddrv.o
DRIVERS+=gpio_drv.o
DRIVERS+=led_drv.o
DRIVERS+=usart_drv.o
DRIVERS+=usb_core_drv.o
DRIVERS+=usb_serial_drv.o
DRIVERS+=hr_timer.o

DTARGETS=$(DRIVERS:.o=)
DOBJS=$(patsubst %, $(OBJ)/drv_obj/%, $(DRIVERS))

ifneq ($(KREL),)
ifneq ($(APPLICATION_DRIVERS),)
DTARGETS+=$(APPLICATION_DRIVERS)
DOBJS+=$(LOBJ)/drv_obj/$(APPLICATION_DRIVERS).o
endif
endif

BOARD=BLACKPILL
ifeq ($(KREL),)
TOP:=$(shell pwd)/../..
OBJ?=$(TOP)/boards/$(BOARD)/obj
BDIR:=$(TOP)/boards/$(BOARD)

GIT_VERSION:=$(shell git describe --abbrev=4 --dirty --always --tags)
VERSION:=$(shell echo "$(BOARD)-$(GIT_VERSION)")
LOBJ=$(OBJ)
else
TOP:=$(KREL)
BDIR:=$(KREL)
endif
GCC_BASE:=$(shell dirname `which arm-none-eabi-gcc`)/..

export AS:=arm-none-eabi-as
export CC:=arm-none-eabi-gcc
export LD:=arm-none-eabi-ld
export OBJCOPY:=arm-none-eabi-objcopy

CFLAGS=-I. -I$(BDIR)/incl -I$(BDIR)/incl/stm32/stm32f411/incl -I$(BDIR) -I$(BDIR)/incl/stm32 -I$(BDIR)/incl/stm32/incl -g -O2 -DDEBUG -DBOARD_$(BOARD) -Wall -mlittle-endian -mthumb -mcpu=cortex-m4 -mthumb-interwork -mfloat-abi=soft -fno-builtin

CFLAGS_USR:=-I$(TOP)/usr/incl -I$(BDIR) -I$(GCC_BASE)/lib/gcc/arm-none-eabi/4.8.3/include -g -Os -DDEBUG -DBOARD_$(BOARD) -fno-builtin -ffreestanding -nostdinc -Wall -mlittle-endian -mthumb -mcpu=cortex-m4 -mthumb-interwork -mfloat-abi=soft -fno-builtin

#LDFLAGS=-g -mlittle-endian -mthumb -mcpu=cortex-m4 -mthumb-interwork -mfloat-abi=hard -mfpu=fpv4-sp-d16 -fno-builtin

ifeq ($(KREL),)
LDFLAGS=-g -T./stm32_flash.ld -mlittle-endian -mthumb -mcpu=cortex-m4 -mthumb-interwork -fno-builtin -nostartfiles
BASEBUILD_INCL:=includes
else
LDFLAGS=-g -T$(KREL)/stm32_flash.ld -mlittle-endian -mthumb -mcpu=cortex-m4 -mthumb-interwork -fno-builtin -nostartfiles
BASEBUILD_INCL=
endif

LDFLAGS_USR=-g -r $(OBJ)/usr/lib/usrlib.o

RELFILES=obj/arch/stm32/stm32.o obj/arch/stm32/stm32f411/stm32f411.o \
	obj/drv_obj/*.o obj/os/end_sect.o obj/os/init_sect.o obj/os/os.o \
	obj/usr/lib/usrlib.o obj/usr/sys_cmd/sys_cmd.o Makefile \
	stm32_flash.ld \
	$(TOP)/usr/incl ./*.h \
	incl/*.h incl/stm32/*.h incl/stm32/incl

all: myCore.bin

myCore.bin: myCore
	arm-none-eabi-objcopy -O binary myCore myCore.bin


rel: myCore
	tar -cvzf krel-$(VERSION).tgz $(RELFILES)

myCore: $(BASEBUILD_INCL) os version stm32f411 stm32 $(DTARGETS) usr.bin.o
	$(CC) $(LDFLAGS) -o $@ $(OBJ)/os/init_sect.o \
				$(OBJ)/version.c \
				$(OBJ)/os/os.o \
				$(OBJ)/arch/stm32/stm32f411/stm32f411.o \
				$(OBJ)/arch/stm32/stm32.o \
				$(DOBJS) \
				$(OBJ)/os/end_sect.o \
				$(LOBJ)/usr/usr.bin.o

version:
	echo "const char *ver=\"$(BOARD)-$(GIT_VERSION)\";" > $(OBJ)/version.c

os: $(OBJ)/os/os.o
stddrv: $(OBJ)/drv_obj/stddrv.o
usb_serial_drv: $(OBJ)/drv_obj/usb_serial_drv.o
stm32: $(OBJ)/arch/stm32/stm32.o
stm32f411: stm32 $(OBJ)/arch/stm32/stm32f411/stm32f411.o
usb_core_drv: $(OBJ)/drv_obj/usb_core_drv.o
hr_timer: $(OBJ)/drv_obj/hr_timer.o
gpio_drv: $(OBJ)/drv_obj/gpio_drv.o
led_drv: $(OBJ)/drv_obj/led_drv.o
usart_drv: $(OBJ)/drv_obj/usart_drv.o

ifeq ($(KREL),)
includes: $(BDIR)/incl $(BDIR)/usr/incl

$(BDIR)/incl: $(TOP)/incl $(BDIR)/incl/stm32
	mkdir -p $@
	lndir $< $@

$(BDIR)/incl/stm32: $(TOP)/arch/stm32
	mkdir -p $@
	lndir $< $@

$(BDIR)/usr/incl: $(TOP)/usr/incl
	mkdir -p $@
	lndir $< $@

usr.bin.o: $(OBJ)/usr/sys_cmd/sys_cmd.o $(OBJ)/usr/blinky/blinky.o $(OBJ)/usr/pin_test/pin_test.o
	$(LD) -o $(OBJ)/usr/usr.bin.o $(LDFLAGS_USR) $^
	$(OBJCOPY) --prefix-symbols=__usr_ $(OBJ)/usr/usr.bin.o
endif

ifeq ($(KREL),)
$(OBJ)/usr/lib/usrlib.o: $(OBJ)/usr/lib $(OBJ)/usr/lib/arm/syscall_lib.o
	CFLAGS="$(CFLAGS_USR)" TOP=$(TOP) OBJ=$(OBJ) $(MAKE) -C $(OBJ)/usr/lib

$(OBJ)/usr/lib: $(TOP)/usr/lib
	mkdir -p $@
	lndir $< $@

$(OBJ)/usr/lib/arm/syscall_lib.o: $(TOP)/usr/lib
	mkdir -p $(OBJ)/usr/lib/arm
	lndir $< $(OBJ)/usr/lib
	CFLAGS="$(CFLAGS_USR)" TOP=$(TOP) OBJ=$(OBJ) $(MAKE) -C $(OBJ)/usr/lib/arm


$(OBJ)/usr/sys_cmd/sys_cmd.o: $(OBJ)/usr/lib/usrlib.o $(OBJ)/usr/sys_cmd
	CFLAGS="$(CFLAGS_USR)" TOP="$(TOP)" OBJ="$(OBJ)" LDFLAGS="$(LDFLAGS_USR)" $(MAKE) -C $(OBJ)/usr/sys_cmd

$(OBJ)/usr/sys_cmd: $(TOP)/usr/sys_cmd
	mkdir -p $@
	lndir $< $@

$(OBJ)/usr/blinky/blinky.o: $(OBJ)/usr/lib/usrlib.o $(OBJ)/usr/blinky
	CFLAGS="$(CFLAGS_USR)" TOP=$(TOP) OBJ=$(OBJ) $(MAKE) -C $(OBJ)/usr/blinky

$(OBJ)/usr/blinky: $(TOP)/usr/blinky
	mkdir -p $@
	lndir $< $@

$(OBJ)/usr/pin_test/pin_test.o: $(OBJ)/usr/lib/usrlib.o $(OBJ)/usr/pin_test
	CFLAGS="$(CFLAGS_USR)" TOP=$(TOP) OBJ=$(OBJ) $(MAKE) -C $(OBJ)/usr/pin_test

$(OBJ)/usr/pin_test: $(TOP)/usr/pin_test
	mkdir -p $@
	lndir $< $@
endif

ifeq ($(KREL),)
usr_drv:
endif

ifeq ($(KREL),)
$(OBJ)/drv_obj/stddrv.o: $(OBJ)/stddrv
	CFLAGS="$(CFLAGS)" TOP=$(TOP) OBJ=$(OBJ) $(MAKE) -C $<
	mkdir -p $(OBJ)/drv_obj
	cp  $(OBJ)/stddrv/stddrv.o $@


$(OBJ)/stddrv: $(TOP)/stddrv
	mkdir -p $@
	lndir $< $@

$(OBJ)/os/os.o: $(OBJ)/os
	CFLAGS="$(CFLAGS)" TOP=$(TOP) OBJ=$(OBJ) $(MAKE) -C $<

$(OBJ)/os: $(TOP)/os
	mkdir -p $@
	lndir $< $@
	cp $(TOP)/os/*.h $(BDIR)/incl/.

$(OBJ)/drv_obj/usb_serial_drv.o: $(OBJ)/drivers/usb_serial
	CFLAGS="$(CFLAGS)" TOP=$(TOP) OBJ=$(OBJ) $(MAKE) -C $<
	mkdir -p $(OBJ)/drv_obj
	cp $(OBJ)/drivers/usb_serial/usb_serial_drv.o  $@

$(OBJ)/drivers/usb_serial: $(TOP)/drivers/usb_serial
	mkdir -p $@
	lndir $< $@

$(OBJ)/arch/stm32/stm32.o: $(OBJ)/arch/stm32
	CFLAGS="$(CFLAGS)" TOP=$(TOP) OBJ=$(OBJ) $(MAKE) -C $<

$(OBJ)/arch/stm32: $(TOP)/arch/stm32
	mkdir -p $@
	lndir $< $@

$(OBJ)/drv_obj/usb_core_drv.o: $(OBJ)/arch/stm32/usb
	CFLAGS="$(CFLAGS)" TOP=$(TOP) OBJ=$(OBJ) $(MAKE) -C $<
	mkdir -p $(OBJ)/drv_obj
	cp  $(OBJ)/arch/stm32/usb/usb_drv.o $@

$(OBJ)/arch/stm32/usb: $(TOP)/arch/stm32/usb
	mkdir -p $@
	lndir $< $@

$(OBJ)/arch/stm32/stm32f411/stm32f411.o: $(OBJ)/arch/stm32/stm32f411
	CFLAGS="$(CFLAGS)" TOP=$(TOP) OBJ=$(OBJ) $(MAKE) -C $<

$(OBJ)/arch/stm32/stm32f411: $(TOP)/arch/stm32/stm32f411
	mkdir -p $@
	lndir $< $@

#$(OBJ)/drv_obj/gpio_drv.o: stm32f411 $(OBJ)/arch/stm32/gpio/gpio_drv.o
#	mkdir -p $(OBJ)/drv_obj
#	cp  $(OBJ)/arch/stm32/gpio/gpio_drv.o $@
#
#$(OBJ)/arch/stm32/gpio/gpio_drv.o: $(OBJ)/arch/stm32/gpio
#	CFLAGS="$(CFLAGS)" TOP=$(TOP) OBJ=$(OBJ) $(MAKE) -C $<
#
#$(OBJ)/arch/stm32/gpio: $(TOP)/arch/stm32/gpio
#	mkdir -p $@
#	lndir $< $@
#
#$(OBJ)/drv_obj/usart_drv.o: stm32f411 $(OBJ)/arch/stm32/usart/usart_drv.o
#	mkdir -p $(OBJ)/drv_obj
#	cp  $(OBJ)/arch/stm32/usart/usart_drv.o $@
#
#$(OBJ)/arch/stm32/usart/usart_drv.o: $(OBJ)/arch/stm32/usart
#	CFLAGS="$(CFLAGS)" TOP=$(TOP) OBJ=$(OBJ) $(MAKE) -C $<
#
#$(OBJ)/arch/stm32/usart: $(TOP)/arch/stm32/usart
#	mkdir -p $@
#	lndir $< $@

$(OBJ)/drv_obj/%_drv.o: $(OBJ)/arch/stm32/%/*_drv.o stm32f411
	mkdir -p $(OBJ)/drv_obj
	cp  $< $@

$(OBJ)/arch/stm32/%/%_drv.o: $(OBJ)/arch/stm32/%
	CFLAGS="$(CFLAGS)" TOP=$(TOP) OBJ=$(OBJ) $(MAKE) -C $<

$(OBJ)/arch/stm32/%: $(TOP)/arch/stm32/%
	mkdir -p $@
	lndir $< $@


$(OBJ)/drv_obj/hr_timer.o: stm32f411 $(OBJ)/arch/stm32/stm32f411/hr_timer/hr_timer.o
	mkdir -p $(OBJ)/drv_obj
	cp  $(OBJ)/arch/stm32/stm32f411/hr_timer/hr_timer.o $@

$(OBJ)/arch/stm32/stm32f411/hr_timer/hr_timer.o: $(OBJ)/arch/stm32/stm32f411/hr_timer
	CFLAGS="$(CFLAGS)" TOP=$(TOP) OBJ=$(OBJ) $(MAKE) -C $<

$(OBJ)/arch/stm32/stm32f411/hr_timer: $(TOP)/arch/stm32/stm32f411/hr_timer
	mkdir -p $@
	lndir $< $@

clean:
	rm -rf $(OBJECTS) myCore myCore.bin
	rm -rf tmp
	rm -rf obj
	rm -rf incl
	rm -rf usr
	rm -rf krel*
	rm  -f ../../incl/io.h
	rm  -f ../../incl/sys.h
endif
