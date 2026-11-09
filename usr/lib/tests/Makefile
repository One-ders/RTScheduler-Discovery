
LOCAL_INCL=../../incl


#all: vsnprintf_test vsnprintf_ref_test sprintf_test dprintf_test
all: vsnprintf_test sprintf_test dprintf_test readline_interactive

vsnprintf_ref_test: vsnprintf_test.o
	$(CC) -o $@ $^

vsnprintf_test: vsnprintf_test.o usr_io.o string.o
	$(CC) -o $@ $^

sprintf_test: sprintf_test.o usr_io.o string.o
	$(CC) -o $@ $^

dprintf_test: dprintf_test.o usr_io.o string.o
	$(CC) -o $@ $^

readline_interactive: readline_interactive.c usr_io.o string.o
	$(CC) -o $@ $^

usr_io.o: usr_io.c
	$(CC) -I ${LOCAL_INCL} -c -o $@ $^

string.o: string.c
	$(CC) -I ${LOCAL_INCL} -c -o $@ $^

usr_io.c: ../usr_io.c
	ln -s $^ $@

string.c: ../string.c
	ln -s $^ $@



clean:
	rm -f vsnprintf_test sprintf_test dprintf_test *.o usr_io.c string.c

.PHONY: all clean
