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
#include "spintronicsIncludes.h"
#include "spiTx.h"
#include "utility.h"

#define CS1 PORTBbits.RB0
#define CS2 PORTBbits.RB1
#define CS3 PORTBbits.RB2
#define ALL_CS_MASK = 0x0007

#define SPI_TX_BUF_SIZE 32

/*
 * top 4 bits of each SPI_TX_BUF 16-bit word are the address,
 * this mask is just to mask off bytes though
 */
#define SPI_TX_ADDR_MASK 0xF0
/*
 * bottom 12 bits of each SPI_TX_BUF 16-bit word are the payload
 */
#define SPI_TX_PL_MASK 0x0FFF

//function prototypes
static void spiTxWorker(void);

//static variables
static uint8_t spiTxCur, spiTxEnd;
static uint16_t spiTxBuf [SPI_TX_BUF_SIZE];

void spiInit(void)
{
    uint16_t junk;
    
    spiTxCur = 0;
    spiTxEnd = 0;
    
    // RE2, RE4 to be digital pins
    ANSELEbits.ANSE2 = 0;
    ANSELEbits.ANSE4 = 0;
    
    // RB0, RB1, RB2 to be digital pins
    ANSELBbits.ANSB0 = 0;
    ANSELBbits.ANSB1 = 0;
    ANSELBbits.ANSB2 = 0;

    // Disable internal pullups for RB0, RB1, RB2
    CNPUBbits.CNPUB0 = 0;
    CNPUBbits.CNPUB1 = 0;
    CNPUBbits.CNPUB2 = 0;

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
    SPI1CON1bits.MODE16 = 1;// Communication is word-wide (16 bits)//must shift at least 10 bits at a time to AD8400
    SPI1CON1bits.CKP = 0;// Idle state for clock is low level
    SPI1CON1bits.CKE = 1;// Serial output data changes on transition from active clock state to idle clock state
    SPI1CON1bits.SSEN = 0;// SS1 pin is not used by the module, pin is controlled by port function
    //set SCK to 70MHz / (1 * 16) = 4.375MHz
    SPI1CON1bits.SPRE = 0x7;    //Sec. prescale = 1:1
    SPI1CON1bits.PPRE = 0x1;    //Prim. prscale = 16:1
    SPI1CON2bits.FRMEN = 0;// Framed support is disabled
    SPI1STATbits.SPIROV = 0;// Clear the SPIROV bit
    SPI1CON2bits.SPIBEN = 0;// Enhanced Buffer is disabled
    SPI1STATbits.SPISIDL = 0;// Continue the module operation in Idle mode
    SPI1STATbits.SPIEN = 1;// enable SPI module
    IFS0bits.SPI1IF = 0;// Clear the Interrupt flag
    junk = SPI1BUF;// clear the buffer of all input
}

void __attribute__((__interrupt__, no_auto_psv)) _SPI1Interrupt(void)
{
    uint16_t junk;
    
    IFS0bits.SPI1IF = 0;// Clear the Interrupt flag
    IEC0bits.SPI1IE = 0;// Disable the interrupt to handle CS and more transmission

    //Disable all chip select lines
    CS1 = 1;
    CS2 = 1;
    CS3 = 1;
    
    SPI1STATbits.SPIROV = 0;// Clear the SPIROV bit
    
    //clear the buffer of all input
    junk = SPI1BUF;
    
    //AD5160 requires > 40ns for CS low-high setup before latching input
    NOP();
    NOP();
    NOP();
    
    START_ATOMIC();//begin critical section; must be atomic!
    ++spiTxCur;
    if (spiTxCur == SPI_TX_BUF_SIZE) {
        spiTxCur = 0;
    }
    if (spiTxCur != spiTxEnd) {
        spiTxWorker();
    }
    END_ATOMIC();//end critical section
}

static void spiTxWorker(void)
{
    uint8_t addr;
    
    START_ATOMIC();//begin critical section; must be atomic!
    addr = *((uint8_t *)(spiTxBuf + spiTxCur) + 1) & SPI_TX_ADDR_MASK;
            
    switch (addr) {
        case U20:
            CS1 = 0;
            //AD5160 requires > 15ns for CS high-low setup before sck starts
            NOP();
            NOP();
            break;
        case U23:
            CS2 = 0;
            //AD8400 requires > 10ns for CS high-low setup before sck starts
            NOP();
            break;
        case U24:
            CS3 = 0;
            //AD8400 requires > 10ns for CS high-low setup before sck starts
            NOP();
            break;
        default:
            break;
    }
            
    SPI1BUF = spiTxBuf[spiTxCur] & SPI_TX_PL_MASK;//only grab the lowest 12 bits
    END_ATOMIC();//end critical section

    IEC0bits.SPI1IE = 1;// Enable the interrupt to handle CS and more transmission

}

void spiTx(uint8_t addr, uint8_t pl)
{
    bool spawnTxThread = false;
    uint8_t bufSpaceAvl;
    
    START_ATOMIC();//begin critical section; must be atomic!

    if (spiTxCur == spiTxEnd) {
        spawnTxThread = true;
        bufSpaceAvl = SPI_TX_BUF_SIZE - 1;
    } else if (spiTxEnd > spiTxCur) {
        bufSpaceAvl = SPI_TX_BUF_SIZE - 1 - (spiTxEnd - spiTxCur);
    }
    else {
        bufSpaceAvl = spiTxCur - spiTxEnd - 1;
    }

    if (bufSpaceAvl > 0) {
        /*
         * We will use the top 4 bits of the payload as the address for CS;
         * this allows us to shift up to 12 bits per transfer.
         * The AD8400 digipot requires a 10 bit code, where the two MSB are 0.
         * Thus, more than 8 bits are needed.  
         *
         * Make sure to keep the lowest 2 bits of addr clear for AD8400!
         */
        *(uint8_t *)(spiTxBuf + spiTxEnd) = pl;
        *((uint8_t *)(spiTxBuf + spiTxEnd) + 1) = addr;//bits 8 and 9 must be 0 for AD8400!
        ++spiTxEnd;
        if (spiTxEnd == SPI_TX_BUF_SIZE)
        {
            spiTxEnd = 0;//this is a circular buffer; wrap back around
        }
    } else {
        //else discard to avoid overflow
        spawnTxThread = false;
    }
 
    END_ATOMIC;//end critical section
    
    if (spawnTxThread)
    {
        spiTxWorker();
    }
}

