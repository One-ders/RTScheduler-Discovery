
ODIR=$(OBJ)/arch/stm32/stm32f411

OUT=$(ODIR)/stm32f411.o

OBJECTS= \
	$(ODIR)/startup_stm32f4xx.s \
	$(ODIR)/iwdg_drv.o \
	$(ODIR)/startup.o

all: $(ODIR) $(OUT)

$(ODIR):
	mkdir -p $@

$(OUT): $(OBJECTS)
	$(CC) $(LDFLAGS) -r -nostdlib  -o $@ $^

%.o: %.c
	$(CC) $(CFLAGS) -c -o $@ $<

%.o: %.s
	$(CC) $(CFLAGS) -c -o $@ $<

