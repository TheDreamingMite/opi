#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <errno.h>
#include <fcntl.h>
#include <unistd.h>
#include <string.h>
#include <sys/ioctl.h>
#include <linux/spi/spidev.h>

#define GPIO_EXPORT "/sys/class/gpio/export"
#define GPIO_DIRECTION "/sys/class/gpio/gpio%d/direction"
#define GPIO_VALUE "/sys/class/gpio/gpio%d/value"

int gpio_set_direction(int gpio, const char *dir) {
    char path[64];
    int fd = open(GPIO_EXPORT, O_WRONLY);
    if (fd < 0) {
        fprintf(stderr, "[ERROR] Failed to open GPIO export\n");
        return -1;
    }
    dprintf(fd, "%d", gpio);
    close(fd);

    snprintf(path, sizeof(path), GPIO_DIRECTION, gpio);
    fd = open(path, O_WRONLY);
    if (fd < 0) {
        fprintf(stderr, "[ERROR] Failed to open GPIO direction for %d\n", gpio);
        return -1;
    }
    dprintf(fd, "%s", dir);
    close(fd);
    return 0;
}

int gpio_write(int gpio, int value) {
    char path[64];
    snprintf(path, sizeof(path), GPIO_VALUE, gpio);
    int fd = open(path, O_WRONLY);
    if (fd < 0) {
        fprintf(stderr, "[ERROR] Failed to open GPIO value for %d\n", gpio);
        return -1;
    }
    dprintf(fd, "%d", value);
    close(fd);
    return 0;
}

int main() {
    fprintf(stdout, "[INFO] === SPI Diagnostic Tool for Orange Pi 3 LTS + LoRaWAN1302 v2.1 ===\n");

    // Step 1: Check SPI device
    fprintf(stdout, "[INFO] 1. Checking SPI device...\n");
    int fd_spi = open("/dev/spidev1.0", O_RDWR);
    if (fd_spi < 0) {
        fprintf(stderr, "[ERROR] Cannot open /dev/spidev1.0\n");
        fprintf(stderr, "errno = %d (%s)\n", errno, strerror(errno));
        return 1;
    }
    fprintf(stdout, "[INFO] ✓ /dev/spidev1.0 opened successfully\n");

    // Step 2: Configure GPIOs (PL8 = 360 for RESET)
    const int RESET_GPIO = 360;
    fprintf(stdout, "[INFO] 2. Configuring GPIO %d (RESET)...\n", RESET_GPIO);
    if (gpio_set_direction(RESET_GPIO, "out") != 0) {
        fprintf(stderr, "[ERROR] Failed to configure GPIO %d\n", RESET_GPIO);
        close(fd_spi);
        return 1;
    }

    // Step 3: Perform RESET sequence: HIGH → LOW → HIGH
    fprintf(stdout, "[INFO] 3. Performing RESET sequence (HIGH → LOW → HIGH)...\n");
    gpio_write(RESET_GPIO, 1);  // HIGH = disable reset
    usleep(100000);            // 100 ms
    gpio_write(RESET_GPIO, 0);  // LOW = active reset
    usleep(100000);            // 100 ms
    gpio_write(RESET_GPIO, 1);  // HIGH = release reset
    fprintf(stdout, "[INFO] ✓ RESET sequence completed\n");

    // Step 4: Try to read Chip ID (register 0x00)
    fprintf(stdout, "[INFO] 4. Reading Chip ID (register 0x00) via SPI...\n");
    uint8_t tx[] = {0x00};
    uint8_t rx[1] = {0};
    struct spi_ioc_transfer tr = {
        .tx_buf = (unsigned long)tx,
        .rx_buf = (unsigned long)rx,
        .len = 1,
        .speed_hz = 500000,
        .bits_per_word = 8,
    };

    int ret = ioctl(fd_spi, SPI_IOC_MESSAGE(1), &tr);
    if (ret < 0) {
        fprintf(stderr, "[ERROR] ioctl(SPI_IOC_MESSAGE) failed\n");
        fprintf(stderr, "errno = %d (%s)\n", errno, strerror(errno));
        close(fd_spi);
        return 1;
    }

    fprintf(stdout, "[INFO] ✓ SPI transfer succeeded\n");
    fprintf(stdout, "[INFO] Chip ID = 0x%02X\n", rx[0]);

    // Step 5: Interpret result
    if (rx[0] == 0x00) {
        fprintf(stderr, "[ERROR] Chip ID is 0x00 → SX1302 NOT RESPONDING!\n");
        fprintf(stdout, "[INFO] Possible causes:\n");
        fprintf(stdout, "  - MISO not connected (most common)\n");
        fprintf(stdout, "  - RESET not properly asserted\n");
        fprintf(stdout, "  - CS (HOST_CSN) on wrong pin (should be PH3/Pin 33)\n");
        fprintf(stdout, "  - Power supply unstable (< 4.8V on VCC)\n");
    } else if (rx[0] == 0x01 || rx[0] == 0x02 || rx[0] == 0x03) {
        fprintf(stdout, "[INFO] ✓ Chip ID = 0x%02X → SX1302 is responding!\n", rx[0]);
        fprintf(stdout, "[INFO] → HAL should now work if config is correct.\n");
    } else {
        fprintf(stdout, "[INFO] Chip ID = 0x%02X → Unknown chip or partial response.\n", rx[0]);
    }

    close(fd_spi);
    return 0;
}
