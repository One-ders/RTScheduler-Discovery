#include <stdio.h>
#include <stdarg.h>
#include <string.h>
#include <stdlib.h>
#include <assert.h>

unsigned int get_current_tic(void) { return 360000; }
int io_write(int fd, const void *buf, int size) {
	return -1;
}

int io_read(int fd, void *buf, int size) {
	return -1;
}

#include <stdio.h>
#include <stdarg.h>
#include <string.h>
#include <stdlib.h>

// Externa funktioner som länkas in från dina källkodsfiler via din Makefile
extern int vsnprintf(char *sbuf, size_t sbuf_size, const char *fmt, va_list ap);

// Enkel wrapper för variabla argument under testet
int run_vsnprintf(char *buf, size_t size, const char *fmt, ...) {
    va_list ap;
    va_start(ap, fmt);
    int ret = vsnprintf(buf, size, fmt, ap);
    va_end(ap);
    return ret;
}

// Hjälpfunktion för att verifiera resultat och minnessäkerhet
void verify_test(const char *test_name, char *buf, size_t size, int expr_ret, const char *expr_str, int real_ret) {
    printf("Test: %s\n", test_name);
    printf("  Returnerat: %d (Forvantat: %d)\n", real_ret, expr_ret);

    if (size > 0) {
        printf("  Strang:    \"%s\" (Forvantat: \"%s\")\n", buf, expr_str);
        if (strcmp(buf, expr_str) != 0) {
            printf("  ❌ FAILED: Strangen matchar inte!\n\n");
            return;
        }
    } else {
        printf("  Strang:    (Size=0, ingen buffer skriven)\n");
    }

    if (real_ret != expr_ret) {
        printf("  ⚠️ WARNING: Returvardet matchar inte forvantat, kolla ditt designval.\n");
    }

    printf("  ✅ SUCCESS\n\n");
}

int main(void) {
    char buf[128];
    int ret;

    printf("==================================================\n");
    printf("STARTAR SKRÄDDARSYDD FUNKTIONELL TESTSVIT\n");
    printf("==================================================\n\n");

    /* ------------------------------------------------------------
     * 1. TESTER FÖR STRÄNGAR (%s) OCH HÖGERJUSTERAD PADDING
     * ------------------------------------------------------------ */

    // Test 1.1: Standard %s
    memset(buf, 0x5A, sizeof(buf));
    ret = run_vsnprintf(buf, 50, "Hello %s", "World");
    verify_test("Strang bas %s", buf, 50, 11, "Hello World", ret);

    // Test 1.2: Strang med bredd (Hogerjusterad som standard POSIX)
    memset(buf, 0x5A, sizeof(buf));
    ret = run_vsnprintf(buf, 50, "[%10s]", "ABC");
    verify_test("Strang med bredd %10s", buf, 50, 12, "[       ABC]", ret);

    // Test 1.3: Strang med bredd och hogertrunkering vid full buffer
    memset(buf, 0x5A, sizeof(buf));
    ret = run_vsnprintf(buf, 10, "[%10s]", "ABC");
    verify_test("Strang padding trunkering", buf, 10, 12, "[       A", ret);

    /* ------------------------------------------------------------
     * 1.4 TESTER FÖR VÄNSTERJUSTERADE STRÄNGAR (%-10s)
     * ------------------------------------------------------------ */

    // Test 1.4: Vänsterjusterad sträng med bredd - texten först, mellanslag efteråt
    memset(buf, 0x5A, sizeof(buf));
    ret = run_vsnprintf(buf, 50, "[%-10s]", "ABC");
    verify_test("Vansterjusterad strang %-10s", buf, 50, 12, "[ABC       ]", ret);

    // Test 1.5: Vänsterjusterad sträng som trunkeras mitt i paddingen
    // Storlek 10 (index 0-9). "[ABC" skrivs, sen fylls det med mellanslag till index 8. Index 9 får \0.
    memset(buf, 0x5A, sizeof(buf));
    ret = run_vsnprintf(buf, 10, "[%-10s]", "ABC");
    verify_test("Vansterjusterad padding trunkering", buf, 10, 12, "[ABC     ", ret);


    /* ------------------------------------------------------------
     * 2. TESTER FÖR TECKEN OCH ESCAPES
     * ------------------------------------------------------------ */

    memset(buf, 0x5A, sizeof(buf));
    ret = run_vsnprintf(buf, 50, "%c%c%c", 'A', 'B', 'C');
    verify_test("Tecken %c", buf, 50, 3, "ABC", ret);

    memset(buf, 0x5A, sizeof(buf));
    ret = run_vsnprintf(buf, 50, "100%% %k", 'X');
    verify_test("Okarnt format & %%", buf, 50, 7, "100% %k", ret);

    /* ------------------------------------------------------------
     * 3. TESTER FÖR TAL MED NOLLPREFIX
     * ------------------------------------------------------------ */

    // Bytte ut 'Mål' till 'Mal' för att garantera 1 byte per tecken (ASCII istallet for UTF-8)
    memset(buf, 0x5A, sizeof(buf));
    ret = run_vsnprintf(buf, 50, "Mal: %d", 1234);
    verify_test("Decimaltal %d", buf, 50, 9, "Mal: 1234", ret);

    memset(buf, 0x5A, sizeof(buf));
    ret = run_vsnprintf(buf, 50, "Hex: %x", 0xDEAD);
    verify_test("Hexadecimal %x", buf, 50, 9, "Hex: dead", ret);

    memset(buf, 0x5A, sizeof(buf));
    ret = run_vsnprintf(buf, 50, "Kod: %05d", 42);
    verify_test("Nollfyllnad tal %05d", buf, 50, 10, "Kod: 00042", ret);

    /* ------------------------------------------------------------
     * 4. TEST FÖR DIN PARAMETERLÖSA TIDSTÄMPEL (%t)
     * ------------------------------------------------------------ */

    // Denna plockar inget argument fran va_arg utan kor internt get_current_tic()
    memset(buf, 0x5A, sizeof(buf));
    ret = run_vsnprintf(buf, 50, "Tid: %t");
    // ts_format forvantas returnera "1:00:00.000" (11 tecken). "Tid: " ar 5 tecken -> totalt 11 tecken
    verify_test("Parameterlos Tidstampel %t", buf, 50, 16, "Tid: 1:00:00.000", ret);

    /* ------------------------------------------------------------
     * 5. KANTFALL
     * ------------------------------------------------------------ */

    memset(buf, 0x5A, sizeof(buf));
    ret = run_vsnprintf(buf, 1, "Test");
    verify_test("Storlek 1 (Endast \\0)", buf, 1, 4, "", ret);

    memset(buf, 0x5A, sizeof(buf));
    ret = run_vsnprintf(buf, 12, "Info: %s %d", "Data", 99);
    verify_test("Exakt trunkeringsgrans med mixade typer", buf, 12, 13, "Info: Data ", ret);

    printf("==================================================\n");
    printf("TESTSVITEN AVSLUTAD\n");
    printf("==================================================\n");

    return 0;
}

