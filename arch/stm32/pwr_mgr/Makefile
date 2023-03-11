
ODIR=$(OBJ)/arch/stm32/pwr_mgr

OUT=$(ODIR)/pwr_mgr.o

OBJECTS= \
	$(ODIR)/pwr_mgr_drv.o

all: $(ODIR) $(OUT)

$(ODIR):
	mkdir -p $@

$(OUT): $(OBJECTS)
	$(CC) $(LDFLAGS) -r -nostdlib  -o $@ $^

%.o: %.c
	$(CC) $(CFLAGS) -c -o $@ $<

%.o: %.s
	$(CC) $(CFLAGS) -c -o $@ $<

