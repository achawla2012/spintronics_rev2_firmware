/*
 * i2sDrv.c
 *
 * designed and written
 * by Michael Reinhart Sandstedt
 * and Samiha Sultana
 *
 * First release: May 7th, 2013
 *
 * questions or support:
 * michael.sandstedt@gmail.com
 */

#include "p33exxxx.h"
#include "spintronics.h"
#include "delay.h"
#include "uartDrv.h"

#define DEFAULT_SENSOR_ADDRESS 0x00//this is the bit pattern to disable both MUXs
#define USB_TX_BUF_SIZE 256
#define BT_TX_BUF_SIZE 256
#define UART_RX_BUF_SZ 512

#define FTDI_RST_BAR PORTFbits.RF5

//function prototypes
static float setVolume(uint8_t channel, float voltage);
static void send(uint8_t* array, uint8_t numBytes);
static void receive (uint8_t *array, uint16_t rxPointer, uint8_t sizeOfPayload);
static void float_to_bytes(float myFloat, uint8_t *array);
static void decodeStartCommand(uint8_t startpayload[]);
static void usbTxWorker(void);
static void btTxWorker(void);
static bool copyToUSBTxBuf(uint8_t *array, uint16_t numBytes);
static bool copyToBTTxBuf(uint8_t *array, uint16_t numBytes);

//global variables
uint8_t global_state;
bool GUIRequestingRun;
bool resetStateMachine;
uint32_t measurementTime;//units are samples
_Q15 f1;//units are Q15 half-cycles per sample-period
_Q15 f2;//units are Q15 half-cycles per sample-period
_Q15 fdiff;//units are Q15 half-cycles per sample-period
_Q15 fsum;//units are Q15 half-cycles per sample-period
_Q15 a1;
_Q15 a2;
uint8_t sensorAddressTable[256];
uint8_t numberOfSensors;
uint8_t bridgeADCGainFactor;//this can be 0, 1, 2, 3, 4, 5, 6, 7, 8; gain = 2^bridgeADCGainFactor;//in practice, only 0, 1, 2, 3, 4 offer any advantage due to the noise floor of the ADC (~114dB for CS4272)

//static variables
static bool f1PlusF2OutOfRangeFlag;
static uint8_t numBytesInPayload=0, findEndOfPayload=0;
static bool USBMode;
static uint8_t usbTxBuf [USB_TX_BUF_SIZE];//must be global so as to be accessible from an ISR//static means FILE SCOPE ONLY
static uint8_t btTxBuf [BT_TX_BUF_SIZE];//must be global so as to be accessible from an ISR//static means FILE SCOPE ONLY
static int16_t usbTxCur, usbTxEnd, btTxCur, btTxEnd;//must be global so as to be accessible from an ISR//static means FILE SCOPE ONLY//keep it singed so my head doesn't hurt!
static float implementedF1, implementedF2, implementedFSum, implementedFDiff;

void processStartCommand(float GUISpecifiedA1, float GUISpecifiedF1, float GUISpecifiedA2, float GUISpecifiedF2, float GUISpecifiedT, uint8_t GUISpeciedBridgeGainFactor)
{
    float implementedA1, implementedA2, implementedT, maxOutputFrequency, maxMeasurementSamples, minMeasurementSamples;
    uint8_t implementedBridgeGainFactor, startPayload_confirmToGUI[25], i;
    float tempTime;

    implementedA1 = setVolume(0, GUISpecifiedA1);
    implementedA2 = setVolume(1, GUISpecifiedA2);

    maxOutputFrequency = MAX_OUTPUT_FREQUENCY;
    maxMeasurementSamples = MAX_MEASUREMENT_SAMPLES;
    minMeasurementSamples = MIN_MEASUREMENT_SAMPLES;

    if (GUISpecifiedF1 > maxOutputFrequency)
    {
        transmitError(F1_OUT_OF_RANGE);
        f1 = _Q15ftoi(maxOutputFrequency * TWICE_SAMPLE_PERIOD);
        implementedF1 = maxOutputFrequency;
    }
    else if (GUISpecifiedF1 < 0)
    {
        transmitError(F1_OUT_OF_RANGE);
        f1 = 0;
        implementedF1 = 0.0;
    }
    else
    {
        f1 = _Q15ftoi(GUISpecifiedF1 * TWICE_SAMPLE_PERIOD);
        implementedF1 = _itofQ15(f1) * HALF_SAMPLE_RATE;
    }

    if (GUISpecifiedF2 > maxOutputFrequency)
    {
        transmitError(F2_OUT_OF_RANGE);
        f2 = _Q15ftoi(maxOutputFrequency * TWICE_SAMPLE_PERIOD);
        implementedF2 = maxOutputFrequency;
    }
    else if (GUISpecifiedF2 < 0)
    {
        transmitError(F2_OUT_OF_RANGE);
        f2 = 0;
        implementedF2 = 0.0;
    }
    else
    {
        f2 = _Q15ftoi(GUISpecifiedF2 * TWICE_SAMPLE_PERIOD);
        implementedF2 = _itofQ15(f2) * HALF_SAMPLE_RATE;
    }

    f1PlusF2OutOfRangeFlag = false;
    if (GUISpecifiedF1 + GUISpecifiedF2 > maxOutputFrequency)
    {
        f1PlusF2OutOfRangeFlag = true;
        transmitError(F1_PLUS_F2_OUT_OF_RANGE);
        fsum = 0;
        implementedFSum = 0;
    }
    else
    {
        fsum = f1 + f2;
        implementedFSum = implementedF1 + implementedF2;
    }
    fdiff = _Q15abs(f1 - f2);
    implementedFDiff = abs(implementedF1 - implementedF2);

    tempTime = GUISpecifiedT * SAMPLE_RATE;
    if (tempTime > maxMeasurementSamples)
    {
        transmitError(T_OUT_OF_RANGE);
        measurementTime = MAX_MEASUREMENT_SAMPLES;
        implementedT = maxMeasurementSamples / SAMPLE_RATE;
    }
    else if (tempTime < minMeasurementSamples)
    {
        transmitError(T_OUT_OF_RANGE);
        measurementTime = MIN_MEASUREMENT_SAMPLES;
        implementedT = minMeasurementSamples / SAMPLE_RATE;
    }
    else
    {
        measurementTime = (uint32_t)tempTime;
        implementedT = (float)measurementTime / SAMPLE_RATE;
    }

    switch(GUISpeciedBridgeGainFactor)
    {
        case 1:
        {
            bridgeADCGainFactor = 0;
            implementedBridgeGainFactor = 1;
            break;
        }
        case 2:
        {
            bridgeADCGainFactor = 1;
            implementedBridgeGainFactor = 2;
            break;
        }
        case 4:
        {
            bridgeADCGainFactor = 2;
            implementedBridgeGainFactor = 4;
            break;
        }
        case 8:
        {
            bridgeADCGainFactor = 3;
            implementedBridgeGainFactor = 8;
            break;
        }
        case 16:
        {
            bridgeADCGainFactor = 4;
            implementedBridgeGainFactor = 16;
            break;
        }
        default:
        {
            transmitError(INVALID_DIGITAL_GAIN_VALUE);
            bridgeADCGainFactor = 0;
            implementedBridgeGainFactor = 1;
            break;
        }
    }

    startPayload_confirmToGUI[0] = 0xFE;
    startPayload_confirmToGUI[1] = confirm_StartCommand;
    startPayload_confirmToGUI[2] = 0x15;
    float_to_bytes(implementedA1, &startPayload_confirmToGUI[3]);
    float_to_bytes(implementedF1, &startPayload_confirmToGUI[7]);
    float_to_bytes(implementedA2, &startPayload_confirmToGUI[11]);
    float_to_bytes(implementedF2, &startPayload_confirmToGUI[15]);
    float_to_bytes(implementedT, &startPayload_confirmToGUI[19]);
    startPayload_confirmToGUI[23] = implementedBridgeGainFactor;

    startPayload_confirmToGUI[24] =0;
    for (i = 1; i<24; i++)
    {
        startPayload_confirmToGUI[24] = startPayload_confirmToGUI[24] ^ startPayload_confirmToGUI[i];
    }
    send(startPayload_confirmToGUI,25);
    GUIRequestingRun = true;//setting this global variable to 'true' and enabling the DCI interrupt starts the state machine
    resetStateMachine = true;//set this to make sure all accumulators are cleared when measurement begins.
    IEC3bits.DCIIE = 1;//enable the interrupt to start measurement!
}

float setVolume(uint8_t channel, float voltage)
{
    if (channel == 0x00)
    {
        if (voltage > FULLSCALE_BRIDGE_DAC_VOLTAGE)
        {
            transmitError(A1_OUT_OF_RANGE);
            a1 = 0x7FFF;
            voltage = FULLSCALE_BRIDGE_DAC_VOLTAGE;
        }
        else if (voltage < 0.0)
        {
            transmitError(A1_OUT_OF_RANGE);
            a1 = 0x0000;
            voltage = 0.0;
        }
        else
        {
            a1 = _Q15ftoi(voltage / FULLSCALE_BRIDGE_DAC_VOLTAGE);
            voltage = _itofQ15(a1) * FULLSCALE_BRIDGE_DAC_VOLTAGE;
        }
    }
    else
    {
        if (voltage > FULLSCALE_COIL_DAC_VOLTAGE)
        {
            transmitError(A2_OUT_OF_RANGE);
            a2 = 0x7FFF;
            voltage = FULLSCALE_COIL_DAC_VOLTAGE;
        }
        else if (voltage < 0.0)
        {
            transmitError(A2_OUT_OF_RANGE);
            a2 = 0x0000;
            voltage = 0.0;
        }
        else
        {
            a2 = _Q15ftoi(voltage / FULLSCALE_COIL_DAC_VOLTAGE);
            voltage = _itofQ15(a2) * FULLSCALE_COIL_DAC_VOLTAGE;
        }
    }
    return voltage;
}

void uart_Init (void)
{
    uint8_t junk;

    /***************************************************************************
     * initialization of global variables
     **************************************************************************/

    GUIRequestingRun = false;
    resetStateMachine = true;
    measurementTime = SAMPLE_RATE;
    f1 = 0;
    f2 = 0;
    fdiff = 0;
    fsum = 0;
    bridgeADCGainFactor = 1;
    f1PlusF2OutOfRangeFlag = false;
    a1 = 0x7FFF;
    a2 = 0x7FFF;
    numberOfSensors = 1;
    sensorAddressTable[0] = DEFAULT_SENSOR_ADDRESS;
    usbTxCur = 0;
    usbTxEnd = 0;
    btTxCur = 0;
    btTxEnd = 0;


    /***************************************************************************
     * pin setup
     **************************************************************************/
    
    //RP101/RF5/PIN32 connects to USB_RESET via Si8442
    TRISFbits.TRISF5 = 0;//set RF5 to be an output so we can toggle the reset pin of the FTDI via the Si8442
    FTDI_RST_BAR = 0;//bring reset low (active)

    //RPI44/RB12/PIN27 connects to USB5V via Si8442
    ANSELBbits.ANSB12 = 0;//make RB12 digital
    TRISBbits.TRISB12 = 1;//set RB12 to be an input so we can monitor whether USB is plugged in or not; RB12 is connected to USB5V via the Si8442

    //RPI45/RB13/PIN28 connects to RX_BT
    ANSELBbits.ANSB13 = 0;//make RB13 digital
    // = 0x03;//route U2TX to ???
    
    //RPI46/RB14/PIN29 connects to TX_BT
    ANSELBbits.ANSB14 = 0;//make RB14 digital
    RPINR19bits.U2RXR = 46;//route RPI46 to U2RXR

    //RPI47/RB15/PIN30 connects to FTDI_TX via Si8442
    ANSELBbits.ANSB15 = 0;//make RB15 digital
    RPINR18bits.U1RXR = 47;//route RPI47 to U1RXR

    //RP100/RF4/PIN31 connects to FTDI_RX via Si8442
    RPOR9bits.RP100R = 0x01;//route U1TX to RP100


    /***************************************************************************
     * UART1 setup for connection to USB via FTDI
     **************************************************************************/

    U1MODEbits.USIDL = 0;
    U1MODEbits.IREN = 0;
    U1MODEbits.RTSMD = 1;	//U1RTS in Simplex mode
    U1MODEbits.UEN = 0;
    U1MODEbits.WAKE = 0;
    U1MODEbits.ABAUD = 0;
    U1MODEbits.URXINV = 0;
    U1MODEbits.BRGH = 0;		//Set for slow speed mode
    U1MODEbits.PDSEL = 0;	//8-bit mode, no parity
    U1MODEbits.STSEL = 0;	//1-stop bit
    U1MODEbits.UARTEN = 1;	//UART1 is enabled
    U1MODEbits.LPBACK = 0;	//LPBACK Disabled

    /****************************** Baud rate Calculations**********************
    *
    *   U1BRG= [(Fcy/(Desired_Baud_rate*16)] - 1....Provided BRGH= 0 (Slow mode)
    *   U1BRG= [Fcy/(4*Baud_rate)] - 1..............Provided BRGH= 1 (Fast mode)
    *   Fosc= 140Mhz
    *   Fcy= Fosc/2= 70MHz
    *
    ***************************************************************************/
    U1BRG= 433;                 //Set for baudrate of 10000 Baud
    //U1BRG = 37;               //Set for baudrate of 115200 baud
    //U1BRG = 454;              //Set for baudrate of 9600 baud

    IPC3bits.U1TXIP = 4;        //TX interrupt priority
    U1STAbits.UTXISEL0 = 1;     //Interrupt when the last character is shifted out of the Transmit Shift Register and all transmit operations are completed
    U1STAbits.UTXISEL1 = 0;
    IFS0bits.U1TXIF = 0;         //clear TX interrupt flag
    IEC0bits.U1TXIE = 1;        //enable UART TX Interrupt
    U1STAbits.UTXEN = 1;        //enable UART.TXEN
    
    IEC0bits.U1RXIE = 0;         //disable RX interrupt
    IPC2bits.U1RXIP = 6;	//Receive interrupt priority
    U1STAbits.URXISEL = 0;      //Interrupt is set when any character is received and transferred from the UxRSR to the receive buffer; receive buffer has one or more characters


    /***************************************************************************
     * UART2 setup for connection to blue tooth
     **************************************************************************/

    U2MODEbits.USIDL = 0;
    U2MODEbits.IREN = 0;
    U2MODEbits.RTSMD = 1;	//U2RTS in Simplex mode
    U2MODEbits.UEN = 0;
    U2MODEbits.WAKE = 0;
    U2MODEbits.ABAUD = 0;
    U2MODEbits.URXINV = 0;
    U2MODEbits.BRGH = 0;		//Set for slow speed mode
    U2MODEbits.PDSEL = 0;	//8-bit mode, no parity
    U2MODEbits.STSEL = 0;	//1-stop bit
    U2MODEbits.UARTEN = 1;	//UART2 is enabled
    U2MODEbits.LPBACK = 0;	//LPBACK Disabled

    U2BRG = 37;                 //Set for baudrate of 115200 baud

    IPC7bits.U2TXIP = 4;        //TX interrupt priority
    U2STAbits.UTXISEL0 = 1;     //Interrupt when the last character is shifted out of the Transmit Shift Register; all transmit operations are completed
    U2STAbits.UTXISEL1 = 0;
    IFS1bits.U2TXIF = 0;
    IEC1bits.U2TXIE = 1;
    U2STAbits.UTXEN = 1;        //enable UART.TXEN

    IEC1bits.U2RXIE = 0;        //disable RX interrupt
    IPC7bits.U2RXIP = 6;	//Receive interrupt priority
    U2STAbits.URXISEL = 0;


    /***************************************************************************
     * clear interrupt flags, clear buffers, enable interrupts
     **************************************************************************/

    busy_wait_ms(10);           //wait for the FTDI to reset
    FTDI_RST_BAR = 1;           //start the FTDI
    busy_wait_ms(50);           //wait for the FTDI to start

    IFS0bits.U1RXIF = 0;        //clear U1RXIF
    U1STAbits.OERR = 0;         //clear overflow error flag
    junk = U1RXREG;             //clear the input buffer
    IEC0bits.U1RXIE = 1;         //enable RX interrupt

    IFS1bits.U2RXIF = 0;        //clear U2RXIF
    U2STAbits.OERR = 0;         //clear overflow error flag
    junk = U2RXREG;             //clear the input buffer
    IEC1bits.U2RXIE = 1;        //enable RX interrupt
}

void __attribute__((__interrupt__, no_auto_psv)) _U1RXInterrupt(void)
{
    uint8_t junk;
    static uint16_t endDataRxPointer = 0;
    static uint8_t RxBuffer[UART_RX_BUF_SZ] ={0};

    IFS0bits.U1RXIF = 0;
    if(0 == PORTBbits.RB12)
    {
        U1STAbits.OERR = 0;//clear overflow error flag
        junk = U1RXREG;//clear the input buffer
        return;//RB12 is low, indicating that USB is not connected; therefore, we should ignore incoming packets from USB
    }
    USBMode = true; // USB is connected which changes the endianness of the data stream (endianness is different for bluetooth)
    RxBuffer[endDataRxPointer] = U1RXREG;

    if(U1STAbits.OERR == 1) //check for overflow of the U1RXREG buffer
    {
        //TODO: delete this packet if overflow error is discovered?  we should
        //probably find a way to flag this packet is invalid
        U1STAbits.OERR = 0;
    }

    if (RxBuffer[endDataRxPointer] == 0xFE)
    {
        findEndOfPayload = 1;
    }
    if (findEndOfPayload == 3)
    {
       numBytesInPayload =  findEndOfPayload + RxBuffer[endDataRxPointer]+1;
    }
    if (findEndOfPayload == numBytesInPayload)
    {
        if (numBytesInPayload > (endDataRxPointer + 1))
        {
            receive(RxBuffer, endDataRxPointer - numBytesInPayload + UART_RX_BUF_SZ + 1, numBytesInPayload);
        }
        else
        {
            receive(RxBuffer, endDataRxPointer - numBytesInPayload + 1, numBytesInPayload);
        }
    }
    ++findEndOfPayload;
    ++endDataRxPointer;
    if (endDataRxPointer == UART_RX_BUF_SZ)
    {
        endDataRxPointer = 0;
    }
}

void __attribute__((__interrupt__, no_auto_psv)) _U2RXInterrupt(void)
{
    uint8_t junk;
    static uint16_t endDataRxPointer=0;
    static uint8_t RxBuffer[UART_RX_BUF_SZ] ={0};

    IFS1bits.U2RXIF = 0;

    if(1 == PORTBbits.RB12)
    {
        U2STAbits.OERR = 0;//clear overflow error flag
        junk = U2RXREG;//clear the input buffer
        return;//RB12 is high, indicating that USB is connected; therefore, we should ignore incoming packets from bluetooth
    }
    USBMode = false; // bluetooth is connected which changes the endianness of the data stream (endianness is different for USB)
    RxBuffer[endDataRxPointer] = U2RXREG;
  
    if(U2STAbits.OERR == 1) //check for overflow of the U1RXREG buffer
    {
        //TODO: delete this packet if overflow error is discovered?  we should
        //probably find a way to flag this packet is invalid
        U2STAbits.OERR = 0;
    }

    if (RxBuffer[endDataRxPointer] == 0xFE)
    {
        findEndOfPayload = 1;
    }
    if (findEndOfPayload == 3)
    {
       numBytesInPayload =  findEndOfPayload + RxBuffer[endDataRxPointer]+1;
    }
    if (findEndOfPayload == numBytesInPayload)
    {
        if (numBytesInPayload > (endDataRxPointer + 1))
        {
            receive(RxBuffer, endDataRxPointer - numBytesInPayload + UART_RX_BUF_SZ + 1,numBytesInPayload );
        }
        else
        {
            receive(RxBuffer, endDataRxPointer - numBytesInPayload + 1,numBytesInPayload );
        }
    }
    ++findEndOfPayload;
    ++endDataRxPointer;
    if (endDataRxPointer == UART_RX_BUF_SZ)
    {
        endDataRxPointer = 0;
    }

}

void __attribute__((__interrupt__, no_auto_psv)) _U1TXInterrupt(void)
{
    usbTxWorker();//The shift register got full. But now it's empty again.  Let's TX!
}

void __attribute__((__interrupt__, no_auto_psv)) _U2TXInterrupt(void)
{
    btTxWorker();//The shift register got full. But now it's empty again.  Let's TX!
}

void transmitResults(uint8_t sensor, double *phaseAngle, float *amplitude, bool bridgeADCClipFlag, bool coilADCClipFlag, bool bridgeDigitalClipFlag)
{
    uint8_t txbuffer[45],i;
    float f1BridgeVolts;
    float f2BridgeVolts;
    float fdiffBridgeVolts;
    float fsumBridgeVolts;
    float f2CoilAmps;

    float f1BridgeRadians;
    float f2BridgeRadians;
    float fdiffBridgeRadians;
    float fsumBridgeRadians;
    float f2CoilRadians;

#ifdef REPORT_RECTANGULAR_VECTORS_FOR_BRIDGE
    float f1BridgeInPhaseVolts, f1BridgeQuadratureVolts, f2BridgeInPhaseVolts, f2BridgeQuadratureVolts, fDiffBridgeInPhaseVolts, fDiffBridgeQuadratureVolts, fSumBridgeInPhaseVolts, fSumBridgeQuadratureVolts;
#endif
    switch(bridgeADCGainFactor)//if digital gain was added, need to scale the reported voltage appropriately
    {
        case 0:
        {
            f1BridgeVolts = amplitude[0] * FULLSCALE_BRIDGE_ADC_BUFFER_VOLTS;
#ifdef MEASURE_F2_AT_BRIDGE
            f2BridgeVolts = amplitude[1] * FULLSCALE_BRIDGE_ADC_BUFFER_VOLTS;
#else
            f2BridgeVolts = 0.0;
#endif
            fdiffBridgeVolts = amplitude[2] * FULLSCALE_BRIDGE_ADC_BUFFER_VOLTS;
            fsumBridgeVolts;
            if (f1PlusF2OutOfRangeFlag)
            {
                fsumBridgeVolts = 0;
            }
            else
            {
                fsumBridgeVolts = amplitude[3] * FULLSCALE_BRIDGE_ADC_BUFFER_VOLTS;
            }
            break;
        }
        case 1:
        {
            f1BridgeVolts = amplitude[0] * FULLSCALE_BRIDGE_ADC_BUFFER_VOLTS_DIVIDED_BY_2;
#ifdef MEASURE_F2_AT_BRIDGE
            f2BridgeVolts = amplitude[1] * FULLSCALE_BRIDGE_ADC_BUFFER_VOLTS_DIVIDED_BY_2;
#else
            f2BridgeVolts = 0.0;
#endif
            fdiffBridgeVolts = amplitude[2] * FULLSCALE_BRIDGE_ADC_BUFFER_VOLTS_DIVIDED_BY_2;
            fsumBridgeVolts;
            if (f1PlusF2OutOfRangeFlag)
            {
                fsumBridgeVolts = 0;
            }
            else
            {
                fsumBridgeVolts = amplitude[3] * FULLSCALE_BRIDGE_ADC_BUFFER_VOLTS_DIVIDED_BY_2;
            }
            break;
        }
        case 2:
        {
            f1BridgeVolts = amplitude[0] * FULLSCALE_BRIDGE_ADC_BUFFER_VOLTS_DIVIDED_BY_4;
#ifdef MEASURE_F2_AT_BRIDGE
            f2BridgeVolts = amplitude[1] * FULLSCALE_BRIDGE_ADC_BUFFER_VOLTS_DIVIDED_BY_4;
#else
            f2BridgeVolts = 0.0;
#endif
            fdiffBridgeVolts = amplitude[2] * FULLSCALE_BRIDGE_ADC_BUFFER_VOLTS_DIVIDED_BY_4;
            fsumBridgeVolts;
            if (f1PlusF2OutOfRangeFlag)
            {
                fsumBridgeVolts = 0;
            }
            else
            {
                fsumBridgeVolts = amplitude[3] * FULLSCALE_BRIDGE_ADC_BUFFER_VOLTS_DIVIDED_BY_4;
            }
            break;
        }
        case 3:
        {
            f1BridgeVolts = amplitude[0] * FULLSCALE_BRIDGE_ADC_BUFFER_VOLTS_DIVIDED_BY_8;
#ifdef MEASURE_F2_AT_BRIDGE
            f2BridgeVolts = amplitude[1] * FULLSCALE_BRIDGE_ADC_BUFFER_VOLTS_DIVIDED_BY_8;
#else
            f2BridgeVolts = 0.0;
#endif
            fdiffBridgeVolts = amplitude[2] * FULLSCALE_BRIDGE_ADC_BUFFER_VOLTS_DIVIDED_BY_8;
            fsumBridgeVolts;
            if (f1PlusF2OutOfRangeFlag)
            {
                fsumBridgeVolts = 0;
            }
            else
            {
                fsumBridgeVolts = amplitude[3] * FULLSCALE_BRIDGE_ADC_BUFFER_VOLTS_DIVIDED_BY_8;
            }
            break;
        }
        case 4:
        {
            f1BridgeVolts = amplitude[0] * FULLSCALE_BRIDGE_ADC_BUFFER_VOLTS_DIVIDED_BY_16;
#ifdef MEASURE_F2_AT_BRIDGE
            f2BridgeVolts = amplitude[1] * FULLSCALE_BRIDGE_ADC_BUFFER_VOLTS_DIVIDED_BY_16;
#else
            f2BridgeVolts = 0.0;
#endif
            fdiffBridgeVolts = amplitude[2] * FULLSCALE_BRIDGE_ADC_BUFFER_VOLTS_DIVIDED_BY_16;
            fsumBridgeVolts;
            if (f1PlusF2OutOfRangeFlag)
            {
                fsumBridgeVolts = 0;
            }
            else
            {
                fsumBridgeVolts = amplitude[3] * FULLSCALE_BRIDGE_ADC_BUFFER_VOLTS_DIVIDED_BY_16;
            }
            break;
        }
        default:
        {
            f1BridgeVolts = amplitude[0] * FULLSCALE_BRIDGE_ADC_BUFFER_VOLTS;
#ifdef MEASURE_F2_AT_BRIDGE
            f2BridgeVolts = amplitude[1] * FULLSCALE_BRIDGE_ADC_BUFFER_VOLTS;
#else
            f2BridgeVolts = 0.0;
#endif
            fdiffBridgeVolts = amplitude[2] * FULLSCALE_BRIDGE_ADC_BUFFER_VOLTS;
            fsumBridgeVolts;
            if (f1PlusF2OutOfRangeFlag)
            {
                fsumBridgeVolts = 0;
            }
            else
            {
                fsumBridgeVolts = amplitude[3] * FULLSCALE_BRIDGE_ADC_BUFFER_VOLTS;
            }
            break;
        }
    }

    f2CoilAmps = amplitude[4] * A_COIL_CORRESPONDING_TO_ADC_FULLSCALE;

    f1BridgeRadians = phaseAngle[0] - BRIDGE_ADC_F1_PHASE_OFFSET;
#ifdef MEASURE_F2_AT_BRIDGE
    f2BridgeDegrees = phaseAngle[1];
#else
    f2BridgeRadians = 0.0;
#endif
    fdiffBridgeRadians = phaseAngle[2] - BRIDGE_ADC_FDIFF_PHASE_OFFSET;
    if (f1PlusF2OutOfRangeFlag)
    {
        fsumBridgeRadians = 0;
    }
    else
    {
        fsumBridgeRadians = phaseAngle[3] - BRIDGE_ADC_FSUM_PHASE_OFFSET;
    }
    f2CoilRadians = phaseAngle[4];

#ifdef REPORT_RECTANGULAR_VECTORS_FOR_BRIDGE
    f1BridgeInPhaseVolts = f1BridgeVolts * cosf(f1BridgeRadians);
    f1BridgeQuadratureVolts = f1BridgeVolts * sinf(f1BridgeRadians);
    f2BridgeInPhaseVolts = f2BridgeVolts * cosf(f2BridgeRadians);
    f2BridgeQuadratureVolts = f2BridgeVolts * sinf(f2BridgeRadians);
    fDiffBridgeInPhaseVolts = fdiffBridgeVolts * cosf(fdiffBridgeRadians);
    fDiffBridgeQuadratureVolts = fdiffBridgeVolts * sinf(fdiffBridgeRadians);
    fSumBridgeInPhaseVolts = fsumBridgeVolts * cosf(fsumBridgeRadians);
    fSumBridgeQuadratureVolts = fsumBridgeVolts * sinf(fsumBridgeRadians);
#endif

    txbuffer[0] = 0xFE;
    txbuffer[1] = 0x82;
    txbuffer[2] = 0x29;
    txbuffer[3] = sensor;
#ifdef REPORT_RECTANGULAR_VECTORS_FOR_BRIDGE
    float_to_bytes(f1BridgeInPhaseVolts, &txbuffer[4]);
    float_to_bytes(f1BridgeQuadratureVolts, &txbuffer[8]);
    float_to_bytes(f2BridgeInPhaseVolts, &txbuffer[12]);
    float_to_bytes(f2BridgeQuadratureVolts, &txbuffer[16]);
    float_to_bytes(fDiffBridgeInPhaseVolts, &txbuffer[20]);
    float_to_bytes(fDiffBridgeQuadratureVolts, &txbuffer[24]);
    float_to_bytes(fSumBridgeInPhaseVolts, &txbuffer[28]);
    float_to_bytes(fSumBridgeQuadratureVolts, &txbuffer[32]);
#else
    float_to_bytes(f1BridgeVolts, &txbuffer[4]);
    float_to_bytes(f1BridgeRadians, &txbuffer[8]);
    float_to_bytes(f2BridgeVolts, &txbuffer[12]);
    float_to_bytes(f2BridgeRadians, &txbuffer[16]);
    float_to_bytes(fdiffBridgeVolts, &txbuffer[20]);
    float_to_bytes(fdiffBridgeRadians, &txbuffer[24]);
    float_to_bytes(fsumBridgeVolts, &txbuffer[28]);
    float_to_bytes(fsumBridgeRadians, &txbuffer[32]);
#endif
    float_to_bytes(f2CoilAmps, &txbuffer[36]);
    float_to_bytes(f2CoilRadians, &txbuffer[40]);

    txbuffer[44] =0;
    for (i = 1; i < 44; i++)
    {
        txbuffer[44] = txbuffer[44] ^ txbuffer[i];
    }
    send (txbuffer, 45);

    if (bridgeADCClipFlag)
    {
        transmitError(BRIDGE_ADC_CLIP);
    }
    if (coilADCClipFlag)
    {
        transmitError(COIL_ADC_CLIP);
    }
    if (bridgeDigitalClipFlag)
    {
        transmitError(BRIDGE_DIGITAL_CLIP);
    }
}

void float_to_bytes(float myFloat, uint8_t *array)
{
    /* endianness of data stream is different for USB and bluetooth*/
    if (USBMode == true)
    {
    *array = *((__eds__ uint8_t*)&myFloat);
    ++array;
    *array = *((__eds__ uint8_t*)&myFloat + 1);
    ++array;
    *array =*((__eds__ uint8_t*)&myFloat + 2);
    ++array;
    *array = *((__eds__ uint8_t*)&myFloat + 3);
    }
    else    // bluetooth is connected
    {
    *array = *((__eds__ uint8_t*)&myFloat + 3);
    --array;
    *array = *((__eds__ uint8_t*)&myFloat + 2);
    --array;
    *array =*((__eds__ uint8_t*)&myFloat + 1);
    --array;
    *array = *((__eds__ uint8_t*)&myFloat);
    }
}

void transmitError(uint8_t errorCode)
{
    static uint8_t errorpayload[5];
    errorpayload[0]= 0xFE;
    errorpayload[1] = 0x83;
    errorpayload[2] = 0x01;
    errorpayload[3] = errorCode;
    errorpayload[4] = errorpayload[1] ^ errorpayload[2] ^ errorpayload[3];
    send(errorpayload,5);
}

void send(uint8_t *array, uint8_t numBytes)
{
    if (1 == PORTBbits.RB12)//USB is connected; use USB for transmission
    {
        if (true == copyToUSBTxBuf(array, numBytes))
        {
            usbTxWorker();
        }
        //else: another thread is handling transmission, or a buffer overflow occurred
    }
    else
    {
        if (true == copyToBTTxBuf(array, numBytes))
        {
            btTxWorker();
        }
        //else: another thread is handling transmission, or a buffer overflow occurred
    }
}

void usbTxWorker(void)
{
    START_ATOMIC();//begin critical section; must be atomic!
    while (usbTxEnd != usbTxCur)//test to see if there is still data to transmit!
    {
        while (U1STAbits.UTXBF == 0)
        {
            U1TXREG = usbTxBuf[usbTxCur];
            ++usbTxCur;
            if (USB_TX_BUF_SIZE == usbTxCur)
            {
                usbTxCur = 0;
            }
        }
    }
    END_ATOMIC();//end critical section
}

void btTxWorker(void)
{
    START_ATOMIC();//begin critical section; must be atomic!
    while (btTxEnd != btTxCur)//test to see if there is still data to transmit!
    {
        while (U2STAbits.UTXBF == 0)
        {
            U2TXREG = btTxBuf[btTxCur];
            ++btTxCur;
            if (BT_TX_BUF_SIZE == btTxCur)
            {
                btTxCur = 0;
            }
        }
    }
    END_ATOMIC();//end critical section
}

bool copyToUSBTxBuf(uint8_t *array, uint16_t numBytes)
{
    bool spawnTxThread = false;
    uint16_t bufSpaceAvl, i;

    START_ATOMIC();//begin critical section; must be atomic!

    if (usbTxEnd == usbTxCur)
    {
        spawnTxThread = true;
        bufSpaceAvl = USB_TX_BUF_SIZE - 1;
    }
    else if (usbTxEnd > usbTxCur)
    {
        bufSpaceAvl = USB_TX_BUF_SIZE - 1 - (usbTxEnd - usbTxCur);
    }
    else
    {
        bufSpaceAvl = usbTxCur - usbTxEnd - 1;
    }
    
    if (bufSpaceAvl < numBytes)
    {
        spawnTxThread = false;//no message transmitted; a buffer overflow has occurred
    }
    else
    {
        for (i = 0; i < numBytes; ++i)
        {
            usbTxBuf[usbTxEnd] = array[i];
            ++usbTxEnd;
            if (usbTxEnd == USB_TX_BUF_SIZE)
            {
                usbTxEnd = 0;//this is a circular buffer; wrap back around
            }
        }
    }
    END_ATOMIC();//end critical section
    return spawnTxThread;
}

bool copyToBTTxBuf(uint8_t *array, uint16_t numBytes)
{
    bool spawnTxThread = false;
    uint16_t bufSpaceAvl, i;

    START_ATOMIC();//begin critical section; must be atomic!
    
    if (btTxEnd == btTxCur)
    {
        spawnTxThread = true;
        bufSpaceAvl = BT_TX_BUF_SIZE - 1;
    }
    else if (btTxEnd > btTxCur)
    {
        bufSpaceAvl = BT_TX_BUF_SIZE - 1 - (btTxEnd - btTxCur);
    }
    else
    {
        bufSpaceAvl = btTxCur - btTxEnd - 1;
    }

    if (bufSpaceAvl < numBytes)
    {
        spawnTxThread = false;//no message transmitted; a buffer overflow has occurred
    }
    else
    {
        for (i = 0; i < numBytes; ++i)
        {
            btTxBuf[btTxEnd] = array[i];
            ++btTxEnd;
            if (btTxEnd == BT_TX_BUF_SIZE)
            {
                btTxEnd = 0;//this is a circular buffer; wrap back around
            }
        }
    }
    END_ATOMIC();//end critical section
    return spawnTxThread;
}

void receive (uint8_t *array, uint16_t rxPointer, uint8_t sizeOfPayload)
{
    uint8_t i, payload[130]={0}, xor_byte =0;
    for(i=0; i<sizeOfPayload; i++)
    {
        if (rxPointer == 1024)
        {
            rxPointer = 0;
        }
        payload[i] = array[rxPointer];
        ++rxPointer;
    }
    for (i=1; i<(sizeOfPayload - 1); i++)
    {
        xor_byte = xor_byte ^ payload[i];
    }
    if (xor_byte != payload[i])
    {
        transmitError(Bad_Packet_XOR);
    }
    else
    {
      if (payload[1] == StartCommand)
       {
         decodeStartCommand(payload);
       }
      if (payload[1] == StopCommand)
       {
        GUIRequestingRun= false;
        payload[1] = confirm_StopCommand;
        payload[2] = 0;
        payload[3] = payload[1] ^ payload[2];
        send(payload, 4);
       }
       if (payload[1] == MuxAddressing)
       {
            //transmit multiplexer addresses to state machine
            numberOfSensors = payload[2];
            if (0 == numberOfSensors)//make sure the sensor address table has at least 1 entry
            {
                numberOfSensors = 1;
                sensorAddressTable[0] = DEFAULT_SENSOR_ADDRESS;
            }
            else
            {
                for (i=0; i<numberOfSensors; i++)
                {
                  sensorAddressTable[i] = payload[3+i];
                }
            }
            payload[1] = confirm_MuxAddressing;
            payload[2] =0;
            payload[3] = payload[1] ^ payload[2];
            send(payload, 4);
        }
    }
}

void decodeStartCommand(uint8_t startpayload[])
{
    uint8_t i, k, array[4];
    /*endianness of data stream is different for USB and bluetooth*/
    
    if (USBMode == true)
    {
    /* floating point data types begin from startpayload[3], so set k=3 */
    k=3;
    for(i=0; i<4; i++)
    {
       array[i] = startpayload[k];
       ++k;
    }
    float GUISpecifiedA1 = *(__eds__ float *)&array;

    for(i=0; i<4; i++)
    {
        array[i] = startpayload[k];
        ++k;
    }
    float GUISpecifiedF1 = *(__eds__ float *)&array;

    for(i=0; i<4; i++)
    {
        array[i] = startpayload[k];
        ++k;
    }
    float GUISpecifiedA2 = *(__eds__ float *)&array;

    for(i=0; i<4; i++)
    {
        array[i] = startpayload[k];
        ++k;
    }
    float GUISpecifiedF2 = *(__eds__ float *)&array;

    for(i=0; i<4; i++)
    {
        array[i] = startpayload[k];
        ++k;
    }
    float GUISpecifiedT = *(__eds__ float *)&array;
    processStartCommand(GUISpecifiedA1,GUISpecifiedF1, GUISpecifiedA2,GUISpecifiedF2,GUISpecifiedT, startpayload[23]);
    }
    else          //bluetooth is connected
    {
    // floating point data types end at startpayload[22], so set k = 22;

    k=22;
    for(i=0; i<4; i++)
    {
       array[i] = startpayload[k];
       --k;
    }
    float GUISpecifiedT = *(__eds__ float *)&array;

    for(i=0; i<4; i++)
    {
        array[i] = startpayload[k];
        --k;
    }
    float GUISpecifiedF2 = *(__eds__ float *)&array;

    for(i=0; i<4; i++)
    {
        array[i] = startpayload[k];
        --k;
    }
    float GUISpecifiedA2 = *(__eds__ float *)&array;

    for(i=0; i<4; i++)
    {
        array[i] = startpayload[k];
        --k;
    }
    float GUISpecifiedF1 = *(__eds__ float *)&array;

    for(i=0; i<4; i++)
    {
        array[i] = startpayload[k];
        --k;
    }
    float GUISpecifiedA1 = *(__eds__ float *)&array;
    processStartCommand(GUISpecifiedA1,GUISpecifiedF1, GUISpecifiedA2,GUISpecifiedF2,GUISpecifiedT, startpayload[23]);
    }

}
