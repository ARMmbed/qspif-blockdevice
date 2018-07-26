# Quad SPI (QSPI) Flash Block Device

Block device driver for NOR based QSPI flash devices that support SFDP.

NOR based QSPI flash supports up to 4bits per cycle of instruction, address and data.
SFDP based QSPI Flash supports variable bus modes (single, dual, quad), several sector erase size types, multiple regions of sector size types and more.

SFDP JEDEC standard can be found in:
https://www.jedec.org/system/files/docs/JESD216B.pdf

``` cpp
// QSPI SFDP Flash - Block Device example
#include "mbed.h"
#include "QSPIFBlockDevice.h"

QSPIFBlockDevice blockD(MBED_CONF_QSPIF_QSPI_IO0,MBED_CONF_QSPIF_QSPI_IO1,MBED_CONF_QSPIF_QSPI_IO2,MBED_CONF_QSPIF_QSPI_IO3,
		MBED_CONF_QSPIF_QSPI_CLK,MBED_CONF_QSPIF_QSPI_CS,0,MBED_CONF_QSPIF_QSPI_FREQ);


int main() {
    printf("QSPI SFDP Flash Block Device example\n");

    // Initialize the SPI flash device and print the memory layout
    blockD.init();
    bd_size_t sector_size_at_address_0 = blockD.get_erase_size(0);

    printf("QSPIF BD size: %llu\n",         blockD.size());
    printf("QSPIF BD read size: %llu\n",    blockD.get_read_size());
    printf("QSPIF BD program size: %llu\n", blockD.get_program_size());

    printf("QSPIF BD erase size (at address 0): %llu\n", sector_size_at_address_0);

    // Write "Hello World!" to the first block
    char *buffer = (char*) malloc(sector_size_at_address_0);
    sprintf(buffer, "Hello World!\n");
    blockD.erase(0, sector_size_at_address_0);
    blockD.program(buffer, 0, sector_size_at_address_0);

    // Read back what was stored
    blockD.read(buffer, 0, sector_size_at_address_0);
    printf("%s", buffer);

    // Deinitialize the device
    blockD.deinit();
}
```
