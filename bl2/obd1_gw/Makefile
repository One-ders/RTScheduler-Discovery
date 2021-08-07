

all: obd1_gw.o obd1_gw_drivers.o

obd1_gw.o: main.o
	$(CC) -r -nostdlib -o $@ $^

asynchio.o: asynchio.c asynchio.h
	$(CC) $(CFLAGS) -c -o $@ $<

obd1_gw_drivers.o: obd1_drv.o
	$(CC) -r -nostdlib -o $@ $^

%_drv.o: %_drv.c
	$(CC) $(CFLAGS_DRV) -c -o $@ $<

main.o: main.c
	$(CC) $(CFLAGS) -c -o $@ $<


clean:
	rm -rf *.o
