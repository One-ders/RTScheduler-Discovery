
USR_CMD=$(OBJ)/usr/blinky

all: $(USR_CMD) $(USR_CMD)/blinky.o

$(USR_CMD):
	mkdir $@

%.o: %.c
	$(CC) $(CFLAGS) -c -o $@ $<

%.o: %.S
	$(CC) $(CFLAGS) -c -o $@ $<

clean:
	rm -rf blinky *.o
