
ODIR=$(OBJ)/arch/stm32/led

OUT=$(ODIR)/led.o

OBJECTS= \
	$(ODIR)/led_drv.o

all: $(ODIR) $(OUT)

$(ODIR):
	mkdir -p $@

$(OUT): $(OBJECTS)
	$(CC) $(LDFLAGS) -r -nostdlib  -o $@ $^

%.o: %.c
	$(CC) $(CFLAGS) -c -o $@ $<

%.o: %.s
	$(CC) $(CFLAGS) -c -o $@ $<

