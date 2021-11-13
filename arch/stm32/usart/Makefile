
ODIR=$(OBJ)/arch/stm32/usart

OUT=$(ODIR)/usart.o

OBJECTS= \
	$(ODIR)/usart_drv.o

all: $(ODIR) $(OUT)

$(ODIR):
	mkdir -p $@

$(OUT): $(OBJECTS)
	$(CC) $(LDFLAGS) -r -nostdlib  -o $@ $^

%.o: %.c
	$(CC) $(CFLAGS) -c -o $@ $<

%.o: %.s
	$(CC) $(CFLAGS) -c -o $@ $<

