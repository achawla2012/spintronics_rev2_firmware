/*
 * spiTx.c
 *
 * designed and written
 * by Michael Reinhart Sandstedt
 *
 * First release: September 10th, 2013
 *
 * questions or support:
 * michael.sandstedt@gmail.com
 */

#include "p33exxxx.h"
#include "spintronics.h"
#include "spiTx.h"

#define CS1 PORTBbits.RB0
#define CS2 PORTBbits.RB1
#define CS3 PORTBbits.RB2
#define ALL_CS_MASK = 0x0007

#define SPI_TX_BUF_SIZE 32

/*
 * top four bits of each SPI_TX_BUF 16-bit word are the address
 */
#define SPI_TX_ADDR_MASK 0xF000
/*
 * nextfour bits are the size of the payload
 * values of 1-8 are valid (up to 8 bytes
 * may be transmitted at a time)
 */
#define SPI_TX_PL_SIZE_MASK 0x0F00

//function prototypes
static void spiTxWorker(void);

//static variables
static uint8_t spiTxCur, spiTxEnd;
static uint16_t spiTxBuf [SPI_TX_BUF_SIZE];

void spiInit(void)
{
    spiTxCur = 0;
    spiTxEnd = 0;
    
    // RE2, RE4 to be digital pins
    ANSELEbits.ANSE2 = 0;
    ANSELEbits.ANSE4 = 0;
    
    // RB0, RB1, RB2 to be digital pins
    ANSELBbits.ANSB0 = 0;
    ANSELBbits.ANSB1 = 0;
    ANSELBbits.ANSB2 = 0;

    // Enable internal pullups for RB0, RB1, RB2
    CNPUBbits.CNPUB0 = 1;
    CNPUBbits.CNPUB1 = 1;
    CNPUBbits.CNPUB2 = 1;

    RPOR5bits.RP82R = 0b000101;// SDO1 (board designator SDI)
    RPOR5bits.RP84R = 0b000110;// SCK1 (board designator CLK)
    TRISBbits.TRISB0 = 0;// set as output (board designator CS1)
    TRISBbits.TRISB1 = 0;// set as output (board designator CS2)
    TRISBbits.TRISB2 = 0;// set as output (board designator CS3)

    //Disable all chip select lines
    CS1 = 1;// PORTBbits.RB0 = 1
    CS2 = 1;// PORTBbits.RB1 = 1
    CS3 = 1;// PORTBbits.RB2 = 1

    IFS0bits.SPI1IF = 0;// Clear the Interrupt flag
    IEC0bits.SPI1IE = 0;// Disable the interrupt
    SPI1STATbits.SISEL = 0b101;// Interrupt when the last bit is shifted out of SPIxSR
    IPC2bits.SPI1IP = 4;// set SPI1 interrupt priority to 4
    SPI1CON1bits.MSTEN = 1;// Master Mode
    SPI1CON1bits.DISSCK = 0;// Internal SPI1 clock is enabled
    SPI1CON1bits.DISSDO = 0;// SDO1 pin is controlled by the module
    SPI1CON1bits.MODE16 = 0;// Communication is byte-wide (8 bits)
    SPI1CON1bits.CKP = 0;// Idle state for clock is low level
    SPI1CON1bits.CKE = 1;// Serial output data changes on transition from active clock state to idle clock state
    SPI1CON1bits.SSEN = 0;// SS1 pin is not used by the module, pin is controlled by port function
    //set SCK to 70MHz / (1 * 16) = 4.375MHz
    SPI1CON1bits.SPRE = 0x7;    //Sec. prescale = 1:1
    SPI1CON1bits.PPRE = 0x1;    //Prim. prscale = 16:1
    SPI1CON2bits.FRMEN = 0;// Framed support is disabled
    SPI1STATbits.SPIROV = 0;// Clear the SPIROV bit
    SPI1CON2bits.SPIBEN = 1;// Enhanced Buffer is enabled
    SPI1STATbits.SPISIDL = 0;// Continue the module operation in Idle mode
    SPI1STATbits.SPIEN = 1;// enable SPI module
    IFS0bits.SPI1IF = 0;// Clear the Interrupt flag
    IEC0bits.SPI1IE = 1;// Enable the interrupt
}

void __attribute__((__interrupt__, no_auto_psv)) _SPI1Interrupt(void)
{
    uint8_t junk;
    
    IFS0bits.SPI1IF = 0;// Clear the Interrupt flag
    
    //Disable all chip select lines
    CS1 = 1;
    CS2 = 1;
    CS3 = 1;
    
    SPI1STATbits.SPIROV = 0;// Clear the SPIROV bit
    
    while (0 == SPI1STATbits.SRXMPT)
    {
        //empty the input buffer
        junk = SPI1BUF;
    }
    
    //AD5160 requires > 40ns for CS low-high setup before latching input
    __asm__ volatile ("nop");
    __asm__ volatile ("nop");
    __asm__ volatile ("nop");
    
    spiTxWorker();//continue transmission
}

static void spiTxWorker(void)
{
    uint16_t addr;
    uint16_t numBytes;
    uint8_t i;
    
    __asm__ volatile ("disi #0x3FFF");//begin critical section; must be atomic!
    if (spiTxEnd != spiTxCur)//test to see if there is still data to transmit!
    {
        numBytes = (spiTxBuf[spiTxCur] && SPI_TX_PL_SIZE_MASK) >> 8;
        
        if (numBytes <= 8)
        {
            addr = (spiTxBuf[spiTxCur] && SPI_TX_ADDR_MASK) >> 12;
            
            switch (addr)
            {
                case U20:
                    CS1 = 0;
                    break;
                case U23:
                    CS2 = 0;
                    break;
                case U24:
                    CS3 = 0;
                    break;
                default:
                    break;
            }
            
            //AD5160 requires > 15ns for CS high-low setup before sck starts
            __asm__ volatile ("nop");
            __asm__ volatile ("nop");
            
            for (i = 0; i < numBytes; ++i)
            {
                SPI1BUF = (uint8_t)spiTxBuf[spiTxCur];//only grab the lower 8 bits
            }
        }//else discard packet; cannot transmit more than 8 bytes at a time
        
        spiTxCur += numBytes;
        if (spiTxCur >= SPI_TX_BUF_SIZE)
        {
            spiTxCur -= SPI_TX_BUF_SIZE;//circular buffer; wrap around
        }
    }
    
    DISICNT = 0;//end critical section
}

void spiTx(uint8_t addr, uint8_t numBytes, __eds__ uint8_t *pl)
{
    bool spawnTxThread = false;
    uint8_t bufSpaceAvl, header, i;
    
    __asm__ volatile ("disi #0x3FFF");//begin critical section; must be atomic!

    if (spiTxCur == spiTxEnd)
    {
        spawnTxThread = true;
        bufSpaceAvl = SPI_TX_BUF_SIZE - 1;
    }
    else if (spiTxEnd > spiTxCur)
    {
        bufSpaceAvl = SPI_TX_BUF_SIZE - 1 - (spiTxEnd - spiTxCur);
    }
    else
    {
        bufSpaceAvl = spiTxCur - spiTxEnd - 1;
    }

    if (bufSpaceAvl < numBytes)
    {
        spawnTxThread = false;
    }
    else {
        header = numBytes + (addr << 4);
        //assign to the uppper byte of spiTxBuf[spiTxEnd]
        *((uint8_t *)(spiTxBuf + spiTxEnd) + 1) = header;
        for (i = 0; i < numBytes; ++i)
        {
            spiTxBuf[spiTxEnd] = pl[i];
            ++spiTxEnd;
            if (spiTxEnd == SPI_TX_BUF_SIZE)
            {
                spiTxEnd = 0;//this is a circular buffer; wrap back around
            }
        }
    }
 
    DISICNT = 0;//end critical section
    
    if (spawnTxThread)
    {
        spiTxWorker();
    }
}

