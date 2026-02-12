#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <errno.h>
#include <fcntl.h>
#include <unistd.h>
#include <string.h>
#include <time.h>
#include <sys/ioctl.h>
#include <sys/stat.h>
#include <linux/spi/spidev.h>
#include <linux/types.h>

// --- Configuration ---
#define SPI_DEVICE "/dev/spidev1.0"
#define RESET_GPIO 360 // PL8
#define RESET_GPIO_PATH "/sys/class/gpio/gpio360/value"
#define RESET_GPIO_DIR_PATH "/sys/class/gpio/gpio360/direction"
#define RESET_GPIO_EXPORT_PATH "/sys/class/gpio/export"

// --- Register definitions (from loragw_reg.h/.c) ---
#define SX1302_REG_COMMON_BASE_ADDR 0x5600
#define SX1302_REG_COMMON_VERSION_VERSION 16 // Index in the array
// Actual address: SX1302_REG_COMMON_BASE_ADDR + 6 = 0x5606
#define SX1302_VERSION_REG_ADDR 0x5600

// --- GPIO Helper Functions ---
int gpio_export(int gpio_num) {
    char buf[10];
    int fd = open("/sys/class/gpio/export", O_WRONLY);
    if (fd < 0) {
        fprintf(stderr, "[ERROR] Cannot open GPIO export file: %s\n", strerror(errno));
        return -1;
    }
    snprintf(buf, sizeof(buf), "%d", gpio_num);
    if (write(fd, buf, strlen(buf)) < 0) {
        fprintf(stderr, "[ERROR] Cannot export GPIO %d: %s\n", gpio_num, strerror(errno));
        close(fd);
        return -1;
    }
    close(fd);
    return 0;
}

int gpio_set_direction(int gpio_num, const char *direction) {
    char path[64];
    int fd;

    snprintf(path, sizeof(path), "/sys/class/gpio/gpio%d/direction", gpio_num);
    fd = open(path, O_WRONLY);
    if (fd < 0) {
        fprintf(stderr, "[ERROR] Cannot open GPIO direction file for %d: %s\n", gpio_num, strerror(errno));
        return -1;
    }
    if (write(fd, direction, strlen(direction)) < 0) {
        fprintf(stderr, "[ERROR] Cannot set GPIO %d direction to %s: %s\n", gpio_num, direction, strerror(errno));
        close(fd);
        return -1;
    }
    close(fd);
    return 0;
}

int gpio_write_value(int gpio_num, int value) {
    char path[64];
    int fd;

    snprintf(path, sizeof(path), "/sys/class/gpio/gpio%d/value", gpio_num);
    fd = open(path, O_WRONLY);
    if (fd < 0) {
        fprintf(stderr, "[ERROR] Cannot open GPIO value file for %d: %s\n", gpio_num, strerror(errno));
        return -1;
    }
    if (dprintf(fd, "%d", value) < 0) {
        fprintf(stderr, "[ERROR] Cannot write value %d to GPIO %d: %s\n", value, gpio_num, strerror(errno));
        close(fd);
        return -1;
    }
    close(fd);
    return 0;
}

// --- Utility Functions ---
void print_timestamp() {
    struct timespec ts;
    clock_gettime(CLOCK_REALTIME, &ts);
    printf("[%ld.%06ld] ", ts.tv_sec, ts.tv_nsec / 1000);
}

// --- SPI Helper Functions ---
// Function to read a single register using the correct SPI protocol
int spi_read_register(int fd_spi, uint16_t addr, uint8_t *value) {
    uint8_t tx[2];
    uint8_t rx[2];

    // Prepare the command: (addr & 0x7F) | 0x80 for read
    tx[0] = (addr & 0x7F) | 0x80;
    tx[1] = 0x00; // Dummy byte

    struct spi_ioc_transfer tr = {
        .tx_buf = (unsigned long)tx,
        .rx_buf = (unsigned long)rx,
        .len = 2, // 2 bytes for read command
        .speed_hz = 500000, // Use same speed as before
        .bits_per_word = 8,
        .delay_usecs = 0,
        .cs_change = 0,
    };

    int ret = ioctl(fd_spi, SPI_IOC_MESSAGE(1), &tr);
    if (ret < 0) {
        fprintf(stderr, "[ERROR] ioctl(SPI_IOC_MESSAGE) for READ failed: %s\n", strerror(errno));
        return -1;
    }

    // Data is returned in rx[1]
    *value = rx[1];
    return 0;
}

// Function to write a single register using the correct SPI protocol
int spi_write_register(int fd_spi, uint16_t addr, uint8_t value) {
    uint8_t tx[3];
    uint8_t rx[3]; // Even for write, we need to read back something

    // Prepare the command: (addr & 0x7F) for write, followed by data
    tx[0] = (addr & 0x7F);
    tx[1] = value;
    tx[2] = 0x00; // Dummy byte

    struct spi_ioc_transfer tr = {
        .tx_buf = (unsigned long)tx,
        .rx_buf = (unsigned long)rx,
        .len = 3, // 3 bytes for write command
        .speed_hz = 500000,
        .bits_per_word = 8,
        .delay_usecs = 0,
        .cs_change = 0,
    };

    int ret = ioctl(fd_spi, SPI_IOC_MESSAGE(1), &tr);
    if (ret < 0) {
        fprintf(stderr, "[ERROR] ioctl(SPI_IOC_MESSAGE) for WRITE failed: %s\n", strerror(errno));
        return -1;
    }
    // Optionally check rx[1] if the protocol expects an echo or status
    // For basic writes, we often don't care about the response here.
    return 0;
}


// --- Main Diagnostic Function ---
int main() {
    print_timestamp();
    fprintf(stdout, "[INFO] === Enhanced SPI Diagnostic Tool for Orange Pi 3 LTS + LoRaWAN1302 v2.1 (Corrected) ===\n");

    // Check if SPI device exists
    print_timestamp();
    fprintf(stdout, "[INFO] 1. Checking if SPI device %s exists...\n", SPI_DEVICE);
    struct stat st;
    if (stat(SPI_DEVICE, &st) < 0) {
        fprintf(stderr, "[ERROR] SPI device %s does not exist: %s\n", SPI_DEVICE, strerror(errno));
        return 1;
    }
    print_timestamp();
    fprintf(stdout, "[INFO] ✓ SPI device %s exists\n", SPI_DEVICE);

    // Open SPI device
    print_timestamp();
    fprintf(stdout, "[INFO] 2. Opening SPI device %s...\n", SPI_DEVICE);
    int fd_spi = open(SPI_DEVICE, O_RDWR);
    if (fd_spi < 0) {
        fprintf(stderr, "[ERROR] Cannot open %s: %s\n", SPI_DEVICE, strerror(errno));
        return 1;
    }
    print_timestamp();
    fprintf(stdout, "[INFO] ✓ SPI device %s opened successfully (fd=%d)\n", SPI_DEVICE, fd_spi);

    // Configure SPI Mode 0
    print_timestamp();
    fprintf(stdout, "[INFO] 3. Configuring SPI mode...\n");
    uint8_t mode = SPI_MODE_0; // CPOL=0, CPHA=0
    if (ioctl(fd_spi, SPI_IOC_WR_MODE, &mode) < 0) {
        fprintf(stderr, "[ERROR] Cannot set SPI mode %d: %s\n", mode, strerror(errno));
        close(fd_spi);
        return 1;
    }
    print_timestamp();
    fprintf(stdout, "[INFO] ✓ SPI mode set to MODE 0 (CPOL=0, CPHA=0)\n");

    // Configure SPI Speed
    uint32_t speed = 500000; // 500 kHz
    if (ioctl(fd_spi, SPI_IOC_WR_MAX_SPEED_HZ, &speed) < 0) {
        fprintf(stderr, "[ERROR] Cannot set SPI speed %u Hz: %s\n", speed, strerror(errno));
        close(fd_spi);
        return 1;
    }
    print_timestamp();
    fprintf(stdout, "[INFO] ✓ SPI max speed set to %u Hz\n", speed);

    // Configure SPI Bits per Word
    uint8_t bits = 8;
    if (ioctl(fd_spi, SPI_IOC_WR_BITS_PER_WORD, &bits) < 0) {
        fprintf(stderr, "[ERROR] Cannot set SPI bits per word %d: %s\n", bits, strerror(errno));
        close(fd_spi);
        return 1;
    }
    print_timestamp();
    fprintf(stdout, "[INFO] ✓ SPI bits per word set to %d\n", bits);

    // Configure RESET GPIO
    print_timestamp();
    fprintf(stdout, "[INFO] 4. Configuring GPIO %d (RESET)...\n", RESET_GPIO);
    // Export GPIO first
    if (access(RESET_GPIO_PATH, F_OK) == -1) {
        if (gpio_export(RESET_GPIO) != 0) {
            fprintf(stderr, "[ERROR] Failed to export GPIO %d\n", RESET_GPIO);
            close(fd_spi);
            return 1;
        }
        print_timestamp();
        fprintf(stdout, "[INFO] ✓ GPIO %d exported\n", RESET_GPIO);
    } else {
         print_timestamp();
         fprintf(stdout, "[INFO] ✓ GPIO %d already exported\n", RESET_GPIO);
    }

    // Set GPIO direction to output
    if (gpio_set_direction(RESET_GPIO, "out") != 0) {
        fprintf(stderr, "[ERROR] Failed to set GPIO %d direction to out\n", RESET_GPIO);
        close(fd_spi);
        return 1;
    }
    print_timestamp();
    fprintf(stdout, "[INFO] ✓ GPIO %d direction set to out\n", RESET_GPIO);

    // Read initial RESET state (optional, for info)
    int fd_reset_val = open(RESET_GPIO_PATH, O_RDONLY);
    if (fd_reset_val >= 0) {
        char val_char;
        if (read(fd_reset_val, &val_char, 1) > 0) {
            print_timestamp();
            fprintf(stdout, "[INFO] Initial RESET pin value: %c\n", val_char);
        }
        close(fd_reset_val);
    }

    // Perform RESET sequence: HIGH -> LOW -> HIGH
    print_timestamp();
    fprintf(stdout, "[INFO] 5. Performing RESET sequence (HIGH -> LOW -> HIGH)...\n");

    // Ensure reset pin starts HIGH (inactive)
    if (gpio_write_value(RESET_GPIO, 1) != 0) {
        fprintf(stderr, "[ERROR] Failed to set GPIO %d HIGH before reset\n", RESET_GPIO);
        close(fd_spi);
        return 1;
    }
    print_timestamp();
    fprintf(stdout, "[INFO] ✓ RESET pin set HIGH (inactive)\n");
    usleep(100000); // 100 ms delay

    // Set reset pin LOW (active reset)
    if (gpio_write_value(RESET_GPIO, 0) != 0) {
        fprintf(stderr, "[ERROR] Failed to set GPIO %d LOW for reset\n", RESET_GPIO);
        close(fd_spi);
        return 1;
    }
    print_timestamp();
    fprintf(stdout, "[INFO] ✓ RESET pin set LOW (active reset)\n");
    usleep(100000); // 100 ms delay

    // Set reset pin HIGH (release reset)
    if (gpio_write_value(RESET_GPIO, 1) != 0) {
        fprintf(stderr, "[ERROR] Failed to set GPIO %d HIGH after reset\n", RESET_GPIO);
        close(fd_spi);
        return 1;
    }
    print_timestamp();
    fprintf(stdout, "[INFO] ✓ RESET pin set HIGH (released reset)\n");

    // Wait for SX1302 internal initialization after reset
    print_timestamp();
    fprintf(stdout, "[INFO] 5a. Waiting 500ms for SX1302 internal initialization after reset...\n");
    usleep(500000); // 500 ms delay
    print_timestamp();
    fprintf(stdout, "[INFO] ✓ Initialization delay completed\n");

    // Read initial RESET state again (optional, for info)
    fd_reset_val = open(RESET_GPIO_PATH, O_RDONLY);
    if (fd_reset_val >= 0) {
        char val_char;
        if (read(fd_reset_val, &val_char, 1) > 0) {
            print_timestamp();
            fprintf(stdout, "[INFO] RESET pin value after delay: %c\n", val_char);
        }
        close(fd_reset_val);
    }


    // Attempt to read Chip Version Register multiple times
    const int num_attempts = 3;
    bool chip_responded = false;
    uint8_t chip_version = 0x00;

    for (int attempt = 1; attempt <= num_attempts; attempt++) {
        print_timestamp();
        fprintf(stdout, "[INFO] 6. Attempt %d: Reading Chip Version Register (0x%04X) via SPI...\n", attempt, SX1302_VERSION_REG_ADDR);

        if (spi_read_register(fd_spi, SX1302_VERSION_REG_ADDR, &chip_version) != 0) {
             fprintf(stderr, "[ERROR] Failed to read register 0x%04X on attempt %d\n", SX1302_VERSION_REG_ADDR, attempt);
             continue; // Try next attempt
        }

        print_timestamp();
        fprintf(stdout, "[INFO] ✓ SPI transfer succeeded on attempt %d\n", attempt);
        print_timestamp();
        fprintf(stdout, "[INFO] Chip Version Register (0x%04X) read on attempt %d = 0x%02X\n", SX1302_VERSION_REG_ADDR, attempt, chip_version);

        if (chip_version != 0x00 && chip_version != 0xFF) {
            chip_responded = true;
            break; // Found a potentially valid response
        }
    }

    // Interpret final result
    print_timestamp();
    fprintf(stdout, "[RESULT] Final Chip Version Register Value = 0x%02X\n", chip_version);

    if (chip_responded) {
        if (chip_version == 0x01) { // Expected value for SX1302 rev1
            print_timestamp();
            fprintf(stdout, "[SUCCESS] ✓ Chip Version 0x%02X matches expected SX1302 rev1 ID!\n", chip_version);
            print_timestamp();
            fprintf(stdout, "[INFO] → HAL connection sequence (lgw_connect) should succeed if AGC/ARB firmware loads correctly.\n");
        } else {
             print_timestamp();
             fprintf(stdout, "[WARNING] Chip Version 0x%02X is non-zero but unexpected.\n", chip_version);
             print_timestamp();
             fprintf(stdout, "[INFO] → This might indicate a different chip revision or an issue.\n");
             print_timestamp();
             fprintf(stdout, "[INFO] → SX1302 might need firmware loading (AGC/ARB) before full functionality, but version check failed.\n");
        }
    } else {
        // Both 0x00 and 0xFF are considered "not responding" for the main logic
        print_timestamp();
        fprintf(stderr, "[FAILURE] Chip Version Register indicates SX1302 is NOT RESPONDING CORRECTLY!\n");
        print_timestamp();
        fprintf(stdout, "[INFO] Possible causes (matching lgw_connect failure):\n");
        print_timestamp();
        fprintf(stdout, "  - MISO wire not connected/broken (most common cause of 0x00)\n");
        print_timestamp();
        fprintf(stdout, "  - CS wire not connected to correct pin (PH3/Pin 33) (common cause of 0x00)\n");
        print_timestamp();
        fprintf(stdout, "  - RESET wire not connected or sequence failed (can cause 0x00)\n");
        print_timestamp();
        fprintf(stdout, "  - Power supply unstable or insufficient (VCC must be 4.8V-5.5V under load)\n");
        print_timestamp();
        fprintf(stdout, "  - SPI mode/speed incorrect (unlikely if default mode used)\n");
        print_timestamp();
        fprintf(stdout, "  - MISO wire floating (can cause 0xFF if CS disconnected, but CS was active here)\n");
        print_timestamp();
        fprintf(stdout, "  - Defective LoRaWAN1302 module\n");
        print_timestamp();
        fprintf(stdout, "  - Internal chip failure after reset\n");
    }

    close(fd_spi);
    print_timestamp();
    fprintf(stdout, "[INFO] Diagnostic tool finished.\n");

    return (chip_responded && chip_version == 0x01) ? 0 : 1; // Return 0 only on full success
}
