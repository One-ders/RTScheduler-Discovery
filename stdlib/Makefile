
all: $(OBJ)/stdlib $(OBJ)/stdlib/stdlib.o

$(OBJ)/stdlib:
	mkdir -p $@

$(OBJ)/stdlib/stdlib.o: $(OBJ)/stdlib/abs.o
	$(CC) $(LDFLAGS) -r -nostdlib  -o $@ $^


%.o: %.c
	$(CC) $(CFLAGS) -c -o $@ $<
