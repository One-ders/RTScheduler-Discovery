
USR_CMD=$(OBJ)/usr/pin_test

all: $(USR_CMD) $(USR_CMD)/pin_test.o

$(USR_CMD):
	mkdir $@

%.o: %.c
	$(CC) $(CFLAGS) -c -o $@ $<

%.o: %.S
	$(CC) $(CFLAGS) -c -o $@ $<

clean:
	rm -rf pin_test *.o
