# OpeniCELink

Open source firmware for iCELink. The FPGA Bitstream flasher on iCESugar nano FPGA dev board.

# Requirement

1. ST Link programmer.
2. Some cable for programming
3. STM32CubeIDE

# How it works ?

The APM32F103TBU6 (STM32 clone) use the USB MSC Library (provided by STMicro) and enumerate itself as USB disk. With the USB<->Internal Flash interface in **USB_DEVICE/App/usbd_storage_if.c** allow PC to access APM32's internal Flash as a disk which has 128 sector and 512 bytes per sector. At first time I formatted this disk. The total disk size (capacity) is too little for FAT16 or FAT32, Windows decided to format as **FAT12** Filesystem. Everytime the iCELink plugged to the PC. The code that I wrote called a function in **main.c** called **ice_mkfs()**. This function write the Boot Sector data to the Flash address of **0x0800FC00** (Boot sector data is copy and modified from the Windows formatted disk) and also write the volumn label to the Flash.

After internal Flash was formatted. **There's 2 while loops** under main.c's while loop. First while loop locate the Bitstream byte on internal flash by using "pattern match" algorithm (to match with **0x7EAA997E** iCE40 Bitstream preamble). In return we have the offset address of where to look for the Bitstream on internal Flash. Sencond while loop locate the end of Bitstream (to match with **0x010600**). After we know where to look for the Bitstream and the end of Bitstream. By taking the Delta and plus 1 we will get the size of Bitstream.

When user **Drag N Drop** the Bitstream file to the disk. Those 2 while loops I mentioned earlier will take care of finding the address of the Bitstream and the size. the next stage is to put iCE40 under reset by pulling down the CRESET pin and flash Bitstream to the on-board SPI NOR flash. In this case it is W25Q16. 2Mbytes NOR flash used to store the Bitstream of iCE40. After Bitstream was flashed to the SPI NOR Flash. iCELink will release the FPGA CRESET pin to let the iCE40 read the Bitstream from the SPI NOR Flash. Then iCELink will reformat its internal Flash with new and fresh FAT12 filesystem, ready for new programming. 