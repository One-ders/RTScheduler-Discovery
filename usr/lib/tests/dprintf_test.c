#include <stdio.h>
#include <stdarg.h>
#include <string.h>
#include <stdlib.h>

unsigned int get_current_tic(void) { return 360000; }
int io_read(int fd, void *buf, int size) {
        return -1;
}


// Extern deklaration av din riktiga dprintf
extern int dprintf(int fd, const char *fmt, ...);

// --- SIMULERAT DRIVRUTINSLAGER ---
#define DEVICE_BUF_SIZE 4096
static char test_device_buf[DEVICE_BUF_SIZE];
static int test_device_idx = 0;

void reset_device(void) {
    memset(test_device_buf, 0, DEVICE_BUF_SIZE);
    test_device_idx = 0;
}

// Din dumma trap/io_write som dprintf skjuter data till
int io_write(int fd, const void *buf, size_t count) {
    (void)fd; // Ignorera i testet

    // Förhindra buffer overflow i vår fejkade drivrutin
    if (test_device_idx + count >= DEVICE_BUF_SIZE) {
        count = DEVICE_BUF_SIZE - test_device_idx - 1;
    }

    if (count > 0) {
        memcpy(&test_device_buf[test_device_idx], buf, count);
        test_device_idx += count;
    }
    return count;
}

// Hjälpfunktion för att verifiera drivrutinens innehåll
void verify_dprintf(const char *test_name, int expr_ret, const char *expr_device_str, int real_ret) {
    printf("Test: %s\n", test_name);
    printf("  Returnerat längdvärde: %d (Forvantat: %d)\n", real_ret, expr_ret);
    printf("  Drivrutinens buffer:  \"%s\" (Forvantat: \"%s\")\n", test_device_buf, expr_device_str);

    if (strcmp(test_device_buf, expr_device_str) != 0 || real_ret != expr_ret) {
        printf("  ❌ FAILED: Felaktig data levererad till enheten!\n\n");
    } else {
        printf("  ✅ SUCCESS\n\n");
    }
}

int main(void) {
    int ret;

    printf("==================================================\n");
    printf("STARTAR FUNKTIONELL TESTSVIT FÖR DPRINTF\n");
    printf("==================================================\n\n");

    // Test 1: Standard synkron blockström
    reset_device();
    ret = dprintf(1, "Task %d: %s", 2, "cec_gw");
    verify_dprintf("Standard kernel-utskrift", 14, "Task 2: cec_gw", ret);

    // Test 2: Lång text (Verifierar att den matar på utan fast storleksbegränsning i printf)
    reset_device();
    ret = dprintf(1, "Watchdog sp=%x pc=%x prio=%d state=%c", 0x2000af18, 0x0800866d, 3, 't');
    verify_dprintf("Lång parameter-dump", 46, "Watchdog sp=2000af18 pc=800866d prio=3 state=t", ret);

    // Test 3: Vänsterjusterad kontra högerjusterad sträng i strömmen
    reset_device();
    ret = dprintf(1, "[%-8s][%8s]", "init", "sys_mon");
    verify_dprintf("Blandad justering i ström", 20, "[init    ][ sys_mon]", ret);

    // Test 4: Uptime med parameterlös tidstämpel
    reset_device();
    ret = dprintf(1, "uptime: %t");
    verify_dprintf("Uptime via %t", 19, "uptime: 1:00:00.000", ret);

    printf("==================================================\n");
    printf("DPRINTF TESTSVIT AVSLUTAD\n");
    printf("==================================================\n");
    return 0;
}

