#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/spi/spidev.h>
#include <stdint.h>
#include <errno.h>
#include <string.h> 

int main() {
    int fd = open("/dev/spidev1.0", O_RDWR);
    if (fd < 0) {
        perror("open /dev/spidev1.0");
        return 1;
    }
    printf("Opened /dev/spidev1.0\n");

    // Попробуем прочитать один байт через SPI (простой обмен)
    uint8_t tx[] = {0x00};  // команда чтения регистра 0x00
    uint8_t rx[1] = {0};
    struct spi_ioc_transfer tr = {
        .tx_buf = (unsigned long)tx,
        .rx_buf = (unsigned long)rx,
        .len = 1,
        .speed_hz = 500000,
        .bits_per_word = 8,
    };

    int ret = ioctl(fd, SPI_IOC_MESSAGE(1), &tr);
    if (ret < 0) {
        perror("ioctl SPI_IOC_MESSAGE");
        printf("errno = %d\n", errno);
        close(fd);
        return 1;
    }

    printf("SPI read: 0x%02X\n", rx[0]);
    close(fd);
    return 0;
}
