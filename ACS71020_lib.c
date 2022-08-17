/*
 * ACS71020_lib.c
 *
 *  Created on: Aug 17, 2022
 *      Author: James (james@spacewizard.com.tw)
 */
#include <stdint.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <getopt.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/types.h>
#include <linux/spi/spidev.h>
#include <math.h>

#define VRMS_IRMS_ADDRESS 0x20 // Vrms, Irms
#define PACTIVE 0x21           // Active Power
#define PAPPARANT 0x22         // Apparant Power
#define PREACTIVE 0x23         // Reactive Power
#define PFACTOR 0x24           // Power Factor
#define NUMPSTOUT 0x25         // Number of Samples of Current and Voltage used for calculation
#define VCODES 0x2A            // Instantaneous Voltage Measurement
#define ICODES 0x2B            // Instantaneous Current Measurement


#define TRIMMING_REGISTER 0x1B // Offset Adjustment
#define RMS_AVG_LEN 0x1C       // Number of averages
#define CUSTOMER_ACCESS 0x2F   // Customer Access


#define READ_COMMAND_ACS71020  0x80
#define WRITE_COMMAND_ACS71020 0x00

#define RMS_CALCULATION_NUMBER_AVERAGES 130686 // In Binary : 00000000 00000001 11111110 01111110 In Decimal : 126 for first stage, 1020 for second stage
#define OFFSET_REGISTER_VALUE 1053448          //
#define CUSTOMER_ACCESS_CODE 0x4F70656E        // For writing on Offsets and Trim Registers
#define DELTA_V_IN_MAX 275          	       // As per the datasheet(Page No 6) - Comment just for testing purpose

static const unsigned char BitRT[] = 
{
  0x00, 0x80, 0x40, 0xC0, 0x20, 0xA0, 0x60, 0xE0, 0x10, 0x90, 0x50, 0xD0, 0x30, 0xB0, 0x70, 0xF0, 
  0x08, 0x88, 0x48, 0xC8, 0x28, 0xA8, 0x68, 0xE8, 0x18, 0x98, 0x58, 0xD8, 0x38, 0xB8, 0x78, 0xF8, 
  0x04, 0x84, 0x44, 0xC4, 0x24, 0xA4, 0x64, 0xE4, 0x14, 0x94, 0x54, 0xD4, 0x34, 0xB4, 0x74, 0xF4, 
  0x0C, 0x8C, 0x4C, 0xCC, 0x2C, 0xAC, 0x6C, 0xEC, 0x1C, 0x9C, 0x5C, 0xDC, 0x3C, 0xBC, 0x7C, 0xFC, 
  0x02, 0x82, 0x42, 0xC2, 0x22, 0xA2, 0x62, 0xE2, 0x12, 0x92, 0x52, 0xD2, 0x32, 0xB2, 0x72, 0xF2, 
  0x0A, 0x8A, 0x4A, 0xCA, 0x2A, 0xAA, 0x6A, 0xEA, 0x1A, 0x9A, 0x5A, 0xDA, 0x3A, 0xBA, 0x7A, 0xFA,
  0x06, 0x86, 0x46, 0xC6, 0x26, 0xA6, 0x66, 0xE6, 0x16, 0x96, 0x56, 0xD6, 0x36, 0xB6, 0x76, 0xF6, 
  0x0E, 0x8E, 0x4E, 0xCE, 0x2E, 0xAE, 0x6E, 0xEE, 0x1E, 0x9E, 0x5E, 0xDE, 0x3E, 0xBE, 0x7E, 0xFE,
  0x01, 0x81, 0x41, 0xC1, 0x21, 0xA1, 0x61, 0xE1, 0x11, 0x91, 0x51, 0xD1, 0x31, 0xB1, 0x71, 0xF1,
  0x09, 0x89, 0x49, 0xC9, 0x29, 0xA9, 0x69, 0xE9, 0x19, 0x99, 0x59, 0xD9, 0x39, 0xB9, 0x79, 0xF9, 
  0x05, 0x85, 0x45, 0xC5, 0x25, 0xA5, 0x65, 0xE5, 0x15, 0x95, 0x55, 0xD5, 0x35, 0xB5, 0x75, 0xF5,
  0x0D, 0x8D, 0x4D, 0xCD, 0x2D, 0xAD, 0x6D, 0xED, 0x1D, 0x9D, 0x5D, 0xDD, 0x3D, 0xBD, 0x7D, 0xFD,
  0x03, 0x83, 0x43, 0xC3, 0x23, 0xA3, 0x63, 0xE3, 0x13, 0x93, 0x53, 0xD3, 0x33, 0xB3, 0x73, 0xF3, 
  0x0B, 0x8B, 0x4B, 0xCB, 0x2B, 0xAB, 0x6B, 0xEB, 0x1B, 0x9B, 0x5B, 0xDB, 0x3B, 0xBB, 0x7B, 0xFB,
  0x07, 0x87, 0x47, 0xC7, 0x27, 0xA7, 0x67, 0xE7, 0x17, 0x97, 0x57, 0xD7, 0x37, 0xB7, 0x77, 0xF7, 
  0x0F, 0x8F, 0x4F, 0xCF, 0x2F, 0xAF, 0x6F, 0xEF, 0x1F, 0x9F, 0x5F, 0xDF, 0x3F, 0xBF, 0x7F, 0xFF
};
static const char *device = "/dev/spidev1.0";
static uint8_t mode=0;
static uint8_t bits = 8;
static uint32_t speed = 5000000;
static uint16_t delay;

int fd=0;
//
static float Vmax, Imax;

static void pabort(const char *s)
{
        perror(s);
        abort();
}
static void acs71020_transfer_ss0(unsigned char *wbuf,unsigned char *rbuf) 
{
        int i,ret;
        unsigned char tx[5],rx[5];
        for(i=0;i<5;i++) tx[i]=BitRT[wbuf[i]];
        struct spi_ioc_transfer tr = {
                .tx_buf = (unsigned long)tx,
                .rx_buf = (unsigned long)rx,
                .len = 5,
                .delay_usecs = delay,
                .speed_hz = speed,
                .bits_per_word = bits,
        };
        ret = ioctl(fd, SPI_IOC_MESSAGE(1), &tr);
        if (ret < 1) pabort("can't send spi message");
        for(i=0;i<5;i++) rbuf[i]=BitRT[rx[i]];
}
static void acs71020_transfer_ss1(unsigned char *wbuf,unsigned char *rbuf) {
        int i,ret;
        unsigned char tx[5],rx[5];
        for(i=0;i<5;i++) tx[i]=BitRT[wbuf[i]];
        struct spi_ioc_transfer tr = {
                .tx_buf = (unsigned long)tx,
                .rx_buf = (unsigned long)rx,
                .len = 5,
                .delay_usecs = delay,
                .speed_hz = speed,
                .bits_per_word = bits,
        };
        system("gpioset gpiochip2 25=0");
        ret = ioctl(fd, SPI_IOC_MESSAGE(1), &tr);
        system("gpioset gpiochip2 25=1");
        if (ret < 1) pabort("can't send spi message");
        for(i=0;i<5;i++) rbuf[i]=BitRT[rx[i]];
}

static void ACS71020_SPI_init() {
        int ret;
        mode|=SPI_CPOL;
        mode|=SPI_CPHA;
//        mode|=SPI_NO_CS;
        fd = open(device, O_RDWR);
        if (fd < 0) pabort("can't open device");
        ret = ioctl(fd, SPI_IOC_WR_MODE, &mode);
        if (ret == -1) pabort("can't set spi mode");
        ret = ioctl(fd, SPI_IOC_RD_MODE, &mode);
        if (ret == -1) pabort("can't get spi mode");
        ret = ioctl(fd, SPI_IOC_WR_BITS_PER_WORD, &bits);
        if (ret == -1) pabort("can't set bits per word");
        ret = ioctl(fd, SPI_IOC_WR_MAX_SPEED_HZ, &speed);
        if (ret == -1) pabort("can't set max speed hz");
        ret = ioctl(fd, SPI_IOC_RD_MAX_SPEED_HZ, &speed);
        Imax = 15.00;
        Vmax=110.00;
}



/*************************************************************
 * @fn          ACS71020_getIrms()
 *
 * @brief       Fetches Effective Current
 *
 * @param       None
 *
 * @return      Float
 */
float ACS71020_getIrms()
{
    float register_val;
  uint16_t tempRegister_val;
  uint8_t normalizing_Number = 0b01111111;
  unsigned char tx[5]={0x00,0x00,0x00,0x00,0x00};
  unsigned char rx[5]={0x00,0x00,0x00,0x00,0x00};
  tx[0] = VRMS_IRMS_ADDRESS | READ_COMMAND_ACS71020;
  acs71020_transfer_ss1(tx,rx);
  tempRegister_val = rx[4] & normalizing_Number;
  tempRegister_val = tempRegister_val << 8 | rx[3];
  register_val = tempRegister_val;
  register_val = (register_val/pow(2,14)) * Imax;  
  return (register_val);
}
/*************************************************************
 * @fn          ACS71020_getVrms()
 *
 * @brief       Fetches Effective Voltage
 *
 * @param       None
 *
 * @return      Float
 */
float ACS71020_getVrms()
{
    float register_val;
    uint16_t tempRegister_val;
    uint8_t normalizing_Number = 0b01111111;
    unsigned char tx[5]={0x00,0x00,0x00,0x00,0x00};
    unsigned char rx[5]={0x00,0x00,0x00,0x00,0x00};
    tx[0] = VRMS_IRMS_ADDRESS | READ_COMMAND_ACS71020;
    acs71020_transfer_ss1(tx,rx);
    tempRegister_val = rx[2] & normalizing_Number;
    tempRegister_val = tempRegister_val << 8 | rx[1];
    register_val = tempRegister_val * pow(2,-15) * DELTA_V_IN_MAX;
    return (register_val);
}

/*********************************************************************
 * @fn          ACS71020_getVcodes()
 *
 * @brief       Instantaneous Voltage Measurement
 *
 * @param       None
 *
 * @returns    float
 */

float ACS71020_getVcodes()
{
    float register_val;
    uint16_t tempRegister_val;
    uint8_t normalizing_Number = 0b01111111;
    unsigned char tx[5]={0x00,0x00,0x00,0x00,0x00};
    unsigned char rx[5]={0x00,0x00,0x00,0x00,0x00};
    tx[0] = VCODES | READ_COMMAND_ACS71020;
    acs71020_transfer_ss1(tx,rx);
    tempRegister_val = rx[3] & normalizing_Number;
    /*
         * If bit no. 17 = 0b1(1 in decimal), then output is negative
    */
    if (tempRegister_val == 1)
    {
      tempRegister_val = rx[2] << 8 | rx[1];
      register_val = (tempRegister_val / pow(2, 16)) * Vmax * -1;
    }
    else
    {
      tempRegister_val = rx[2] << 8 | rx[1];
      register_val = (tempRegister_val / pow(2, 16)) * Vmax;
    }
    return (register_val);
}
/*********************************************************************
 * @fn          ACS71020_getIcodes()
 *
 * @brief       Instantaneous Current Measurement
 *
 * @param       None
 *
 * @returns     float
 */

float ACS71020_getIcodes()
{
    float register_val;
    uint16_t tempRegister_val;
    uint8_t normalizing_Number = 0b01111111;
    unsigned char tx[5]={0x00,0x00,0x00,0x00,0x00};
    unsigned char rx[5]={0x00,0x00,0x00,0x00,0x00};
    tx[0] = ICODES | READ_COMMAND_ACS71020;
    acs71020_transfer_ss1(tx,rx);
    tempRegister_val = rx[3] & normalizing_Number;
    /*
         * If bit no. 17 = 0b1(1 in decimal), then output is negative
    */
    if (tempRegister_val == 1)
    {
      tempRegister_val = rx[2] << 8 | rx[1];
      register_val = (tempRegister_val / pow(2, 16)) * Imax * -1;
    }
    else
    {
      tempRegister_val = rx[2] << 8 | rx[1];
      register_val = (tempRegister_val / pow(2, 16)) * Imax;
    }
    return (register_val);
}



/***********************************************************
 * @fn          Initialize_Trim_register
 *
 * @brief       Sets the offset
 *
 * @param       SPI_handle -> SPI Handle which is returned when SPI_open() is called
 *
 * @return      None
 *
 */
void Initialize_Trim_Register()
{
    unsigned char tx[5]={0x00,0x00,0x00,0x00,0x00};
    unsigned char rx[5]={0x00,0x00,0x00,0x00,0x00};
    /*
     * Access to the Shadow Memory
     */
    tx[0] = CUSTOMER_ACCESS | WRITE_COMMAND_ACS71020;
    tx[1] = CUSTOMER_ACCESS_CODE & 0xFF;
    tx[2] = (CUSTOMER_ACCESS_CODE >> 8) & 0xFF;
    tx[3] = (CUSTOMER_ACCESS_CODE >> 16) & 0xFF;
    tx[4] = (CUSTOMER_ACCESS_CODE >>24) & 0xFF;
    acs71020_transfer_ss1(tx,rx);

    /*
     * Trimming Register Setting
     */
    tx[0] = TRIMMING_REGISTER | WRITE_COMMAND_ACS71020;
    tx[1] = OFFSET_REGISTER_VALUE & 0xFF;
    tx[2] = (OFFSET_REGISTER_VALUE >> 8) & 0xFF;
    tx[3] = (OFFSET_REGISTER_VALUE >> 16) & 0xFF;
    tx[4] = (OFFSET_REGISTER_VALUE >>24) & 0xFF;
    acs71020_transfer_ss1(tx,rx);

    /*
     * Setting Number of averages
     */
    tx[0] = RMS_AVG_LEN | WRITE_COMMAND_ACS71020;
    tx[1] = RMS_CALCULATION_NUMBER_AVERAGES & 0xFF;
    tx[2] = (RMS_CALCULATION_NUMBER_AVERAGES >> 8) & 0xFF;
    tx[3] = (RMS_CALCULATION_NUMBER_AVERAGES >> 16) & 0xFF;
    tx[4] = (RMS_CALCULATION_NUMBER_AVERAGES >>24) & 0xFF;
    acs71020_transfer_ss1(tx,rx);

}

/********************************************************
 * @fn          ACS710210_SPI_Test()
 *
 * @brief       Checks if communication/transaction is executing.
 *
 * @param       None
 *
 * @return     bool
 */
unsigned char ACS71020_SPI_Check()
{
    unsigned char tx[5]={0x00,0x00,0x00,0x00,0x00};
    unsigned char rx[5]={0x00,0x00,0x00,0x00,0x00};
    Initialize_Trim_Register();
    tx[0] = TRIMMING_REGISTER | READ_COMMAND_ACS71020;
    acs71020_transfer_ss1(tx,rx);
    if(rx[0] == (OFFSET_REGISTER_VALUE & 0xFF)) return 1;
    return 0;
    
}
void main() {
    ACS71020_SPI_init();
    if(ACS71020_SPI_Check()) {
      printf("ACS71020 working!!!\r\n");
      printf("Instantaneous Voltage:%2.1f V\r\n",ACS71020_getVcodes());
      printf("Instantaneous Current:%2.1f A\r\n",ACS71020_getIcodes());
      printf("Fetches Effective Voltage:%2.1f V\r\n",ACS71020_getVrms());
      printf("Fetches Effective Current:%2.1f A\r\n",ACS71020_getIrms());
    } else {
      printf("ACS71020 not working!!!\r\n");
    }

}
