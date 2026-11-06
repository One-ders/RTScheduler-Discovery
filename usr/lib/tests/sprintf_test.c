#include <stdio.h>
#include <stdarg.h>
#include <string.h>
#include <stdlib.h>

// Extern deklaration av din riktiga sprintf (länkas via din Makefile)
extern int sprintf(char *sbuf, const char *fmt, ...);
unsigned int get_current_tic(void) { return 360000; }
int io_write(int fd, const void *buf, int size) {
        return -1;
}

int io_read(int fd, void *buf, int size) {
        return -1;
}



// Hjälpfunktion för att verifiera resultat
void verify_sprintf(const char *test_name, char *buf, int expr_ret, const char *expr_str, int real_ret) {
    printf("Test: %s\n", test_name);
    printf("  Returnerat: %d (Forvantat: %d)\n", real_ret, expr_ret);
    printf("  Strang:    \"%s\" (Forvantat: \"%s\")\n", buf, expr_str);

    if (strcmp(buf, expr_str) != 0 || real_ret != expr_ret) {
        printf("  ❌ FAILED: Felaktigt resultat!\n\n");
    } else {
        printf("  ✅ SUCCESS\n\n");
    }
}

int main(void) {
    char buf[256];
    int ret;

    printf("==================================================\n");
    printf("STARTAR FUNKTIONELL TESTSVIT FÖR SPRINTF\n");
    printf("==================================================\n\n");

    // Test 1: Standard sträng och tecken
    memset(buf, 0x5A, sizeof(buf));
    ret = sprintf(buf, "User: %s (%c)", "Anders", 'A');
    verify_sprintf("Standard string/char", buf, 16, "User: Anders (A)", ret);

    // Test 2: Högerjusterad padding (Sträng)
    memset(buf, 0x5A, sizeof(buf));
    ret = sprintf(buf, "[%10s]", "CLI");
    verify_sprintf("Hogerjusterad %10s", buf, 12, "[       CLI]", ret);

    // Test 3: Vänsterjusterad padding (Sträng via -)
    memset(buf, 0x5A, sizeof(buf));
    ret = sprintf(buf, "[%-10s]", "CLI");
    verify_sprintf("Vansterjusterad %-10s", buf, 12, "[CLI       ]", ret);

    // Test 4: Decimaltal och Hexadecimaler
    memset(buf, 0x5A, sizeof(buf));
    ret = sprintf(buf, "Val: %d, Hex: %x", 42, 0xABC);
    verify_test: verify_sprintf("Tal %d och %x", buf, 17, "Val: 42, Hex: abc", ret);

    // Test 5: Nollprefix på tal (Ignorerar eventuella minus/vänsterjusteringar)
    memset(buf, 0x5A, sizeof(buf));
    ret = sprintf(buf, "Prio: %03d", 5);
    verify_sprintf("Nollprefix %03d", buf, 9, "Prio: 005", ret);

    // Test 6: Parameterlös tidstämpel %t (16 tecken baserat på "1:00:00.000")
    memset(buf, 0x5A, sizeof(buf));
    ret = sprintf(buf, "Time: %t");
    verify_sprintf("Tidstampel %t", buf, 17, "Time: 1:00:00.000", ret);

    printf("==================================================\n");
    printf("SPRINTF TESTSVIT AVSLUTAD\n");
    printf("==================================================\n");
    return 0;
}

