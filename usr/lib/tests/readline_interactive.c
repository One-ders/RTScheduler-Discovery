#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <termios.h>

#define BUFFER_SIZE 64

unsigned int get_current_tic(void) { return 360000; }
int io_write(int fd, const void *buf, int size) {
	return write(fd,buf,size);
}

int io_read(int fd, void *buf, int size) {
	return read(fd,buf,size);
}



// Deklaration av DIN pristina funktion med den korrekta signaturen!
extern int readline_r(int fd, char *prompt, char *buf, int buf_size);

static struct termios orig_termios;

void disable_raw_mode(void) {
    tcsetattr(STDIN_FILENO, TCSAFLUSH, &orig_termios);
}

void enable_raw_mode(void) {
    tcgetattr(STDIN_FILENO, &orig_termios);
    atexit(disable_raw_mode);

    struct termios raw = orig_termios;
    raw.c_lflag &= ~(ECHO | ICANON);

    raw.c_iflag &= ~ICRNL;

    tcsetattr(STDIN_FILENO, TCSAFLUSH, &raw);
}

int main(void) {
    char input_buffer[BUFFER_SIZE];
    char *my_prompt = "kernel_cli> ";

    printf("--- PRISTINE READLINE_R RAW TESTER ---\n");
    printf("Signatur: int readline_r(int fd, char *prompt, char *buf, int buf_size)\n");
    printf("Skriv kommandon och klättra manuellt i din historik.\n");
    printf("Skriv 'exit' för att stänga.\n\n");
    fflush(stdout);

    enable_raw_mode();

    while (1) {
        // Överlämna kontrollen 100 % till din readline_r (0 = stdin)
        int len = readline_r(0, my_prompt, input_buffer, sizeof(input_buffer));

        disable_raw_mode();

        if (strcmp(input_buffer, "exit") == 0) {
            printf("Avslutar.\n");
            break;
        }

        printf("[Returnerat: '%s' (%d bytes)]\n\n", input_buffer, len);
        fflush(stdout);

        enable_raw_mode();
    }

    return 0;
}

