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

void log_info(const char *msg) {
    fprintf(stdout, "[INFO] %s\n", msg);
    fflush(stdout);
}

void log_error(const char *msg) {
    fprintf(stderr, "[ERROR] %s\n", msg);
    fflush(stderr);
}

int gpio_set_direction(int gpio, const char *dir) {
    char path[64];
    int fd = open(GPIO_EXPORT, O_WRONLY);
    if (fd < 0) {
        log_error("Failed to open GPIO export");
        return -1;
    }
    dprintf(fd, "%d", gpio);
    close(fd);

    snprintf(path, sizeof(path), GPIO_DIRECTION, gpio);
    fd = open(path, O_WRONLY);
    if (fd < 0) {
        log_error("Failed to open GPIO direction");
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
        log_error("Failed to open GPIO value");
        return -1;
    }
    dprintf(fd, "%d", value);
    close(fd);
    return 0;
}

int main() {
    log_info("=== SPI Diagnostic Tool for Orange Pi 3 LTS + LoRaWAN1302 v2.1 ===");

    // Step 1: Check SPI device
    log_info("1. Checking SPI device...");
    int fd_spi = open("/dev/spidev1.0", O_RDWR);
    if (fd_spi < 0) {
        log_error("Cannot open /dev/spidev1.0");
        fprintf(stderr, "errno = %d (%s)\n", errno, strerror(errno));
        return 1;
    }
    log_info("✓ /dev/spidev1.0 opened successfully");

    // Step 2: Configure GPIOs (PL8 = 360 for RESET)
    const int RESET_GPIO = 360;
    log_info("2. Configuring GPIO %d (RESET)...", RESET_GPIO);
    if (gpio_set_direction(RESET_GPIO, "out") != 0) {
        log_error("Failed to configure GPIO %d", RESET_GPIO);
        close(fd_spi);
        return 1;
    }

    // Step 3: Perform RESET sequence: HIGH → LOW → HIGH
    log_info("3. Performing RESET sequence (HIGH → LOW → HIGH)...");
    gpio_write(RESET_GPIO, 1);  // HIGH = disable reset
    usleep(100000);            // 100 ms
    gpio_write(RESET_GPIO, 0);  // LOW = active reset
    usleep(100000);            // 100 ms
    gpio_write(RESET_GPIO, 1);  // HIGH = release reset
    log_info("✓ RESET sequence completed");

    // Step 4: Try to read Chip ID (register 0x00)
    log_info("4. Reading Chip ID (register 0x00) via SPI...");
    uint8_t tx[] = {0x00};  // Read register 0x00
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
        log_error("ioctl(SPI_IOC_MESSAGE) failed");
        fprintf(stderr, "errno = %d (%s)\n", errno, strerror(errno));
        close(fd_spi);
        return 1;
    }

    log_info("✓ SPI transfer succeeded");
    log_info("Chip ID = 0x%02X", rx[0]);

    // Step 5: Interpret result
    if (rx[0] == 0x00) {
        log_error("Chip ID is 0x00 → SX1302 NOT RESPONDING!");
        log_info("Possible causes:");
        log_info("  - MISO not connected (most common)");
        log_info("  - RESET not properly asserted");
        log_info("  - CS (HOST_CSN) on wrong pin (should be PH3/Pin 33)");
        log_info("  - Power supply unstable (< 4.8V on VCC)");
    } else if (rx[0] == 0x01 || rx[0] == 0x02 || rx[0] == 0x03) {
        log_info("✓ Chip ID = 0x%02X → SX1302 is responding!", rx[0]);
        log_info("→ HAL should now work if config is correct.");
    } else {
        log_info("Chip ID = 0x%02X → Unknown chip or partial response.", rx[0]);
    }

    close(fd_spi);
    return 0;
}
