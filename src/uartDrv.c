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
#include "spintronicsIncludes.h"
#include "spintronicsConfig.h"
#include "constants.h"
#include "fsmStates.h"
#include "uartDrv.h"
#include "commsDefines.h"
#include "balanceBridge.h"
#include "uartDrv.h"
#include "spintronicsStructs.h"
#include "digiPotDrv.h"

#define DEFAULT_SENSOR_ADDRESS 0x00//bit pattern to disable both MUXs
#define USB_TX_BUF_SIZE 256
#define BT_TX_BUF_SIZE 256
#define USB_RX_BUF_SZ MAX_RX_PAYLOAD_SIZE
#define BT_RX_BUF_SZ MAX_RX_PAYLOAD_SIZE

#define FTDI_RST_BAR PORTFbits.RF5
#define USB_5V_DETECT PORTBbits.RB12

//function prototypes
static float setVolume(uint8_t channel, float voltage);
static inline void send(uint8_t* array, uint8_t numBytes);
static inline void receive(bool rxFromUSB, uint8_t *array, uint16_t rxPointer,
                           uint8_t sizeOfPayload);
static inline void float_to_bytes(float myFloat, uint8_t *array);
static inline void decodeStartCommand(uint8_t startpayload[],
                                      uint8_t sizeOfPayload);
static void usbTxWorker(void);
static void btTxWorker(void);
static inline bool copyToUSBTxBuf(uint8_t *array, uint16_t numBytes);
static inline bool copyToBTTxBuf(uint8_t *array, uint16_t numBytes);
static inline void decodeBalanceBridgeCommand(uint8_t *payload,
                                              uint8_t sizeOfPayload);
#if !defined(NO_GUI) && !defined(SIMULATION_MODE)
static inline void processStartCommand(float GUISpecifiedA1,
                                       float GUISpecifiedF1,
                                       float GUISpecifiedA2,
                                       float GUISpecifiedF2,
                                       float GUISpecifiedT,
                                       uint8_t GUISpeciedBridgeGainFactor,
                                       float GUISpecifiedBridgeAnalogGain);
#endif

//global variables
uint8_t global_state = IDLE;
uint32_t measurementTime;//units are samples
_Q15 f1;//units are Q15 half-cycles per sample-period
_Q15 f2;//units are Q15 half-cycles per sample-period
_Q15 fdiff;//units are Q15 half-cycles per sample-period
_Q15 fsum;//units are Q15 half-cycles per sample-period
_Q15 a1;//units are Q15 franctions of DAC full-scale
_Q15 a2;//units are Q15 fractions of DAC full-scale
uint8_t bridgeADCGainFactor;//this can be 0, 1, 2, 3, 4, 5, 6, 7, 8; gain = 2^bridgeADCGainFactor;//in practice, only 0, 1, 2, 3, 4 offer any advantage due to the noise floor of the ADC (~114dB for CS4272)
uint8_t u24_code;
float inverseBridgeAnalogGain;
bool f1PlusF2OutOfRange;
_Q15 bridge_balance_amplitude;
_Q15 bridge_balance_frequency;

uint8_t *sensorAddressTable;
uint8_t sensorAddressTableAllocated[MAX_MUX_ADDRESS_TABLE_SIZE];
uint8_t numberOfSensors;

static uint8_t usbTxBuf [USB_TX_BUF_SIZE];//must be global so as to be accessible from an ISR//static means FILE SCOPE ONLY
static uint8_t btTxBuf [BT_TX_BUF_SIZE];//must be global so as to be accessible from an ISR//static means FILE SCOPE ONLY
static int16_t usbTxCur, usbTxEnd, btTxCur, btTxEnd;//must be global so as to be accessible from an ISR//static means FILE SCOPE ONLY//keep it singed so my head doesn't hurt!

#ifndef NO_GUI
static
#endif
inline void
processStartCommand(float GUISpecifiedA1, float GUISpecifiedF1,
                    float GUISpecifiedA2, float GUISpecifiedF2,
                    float GUISpecifiedT, uint8_t GUISpeciedBridgeGainFactor,
                    float GUISpecifiedBridgeAnalogGain) {

    uint32_t local_T;//units are samples
    _Q15 local_f1, local_f2, local_fdiff, local_fsum, local_bridge_balance_frequency;//units are Q15 half-cycles per sample-period
    _Q15 local_a1, local_a2, local_bridge_balance_amplitude;//units are Q15 franctions of DAC full-scale
    uint8_t local_gain_factor;//this can be 0, 1, 2, 3, 4, 5, 6, 7, 8; gain = 2^bridgeADCGainFactor;//in practice, only 0, 1, 2, 3, 4 offer any advantage due to the noise floor of the ADC (~114dB for CS4272)
    uint8_t local_u24_code;
    float local_inverseBridgeAnalogGain;
    float implementedBridgeGain;
    bool local_f1PlusF2OutOfRange;

    float implementedF1, implementedF2, implementedFSum, implementedFDiff;
    float implementedA1, implementedA2, implementedT, maxMeasurementSamples, minMeasurementSamples;
    uint8_t implementedBridgeGainFactor, startPayload_confirmToGUI[25], i;
    float tempTime;

    if (GUISpecifiedA1 > FULLSCALE_BRIDGE_DAC_VOLTAGE) {

        transmitError(A1_OUT_OF_RANGE);
        local_a1 = 0x7FFF;
        implementedA1 = FULLSCALE_BRIDGE_DAC_VOLTAGE;
        local_bridge_balance_amplitude = DEFAULT_BALANCE_AMPLITUDE;//set in case we need to balance the bridge

    } else if (GUISpecifiedA1 < 0.0) {

        transmitError(A1_OUT_OF_RANGE);
        local_a1 = 0x0000;
        implementedA1 = 0.0;
        local_bridge_balance_amplitude = DEFAULT_BALANCE_AMPLITUDE;//set in case we need to balance the bridge

    } else {
        local_a1 = _Q15ftoi(GUISpecifiedA1 / FULLSCALE_BRIDGE_DAC_VOLTAGE);
        implementedA1 = _itofQ15(local_a1) * FULLSCALE_BRIDGE_DAC_VOLTAGE;
        local_bridge_balance_amplitude = local_a1;//set in case we need to balance the bridge
    }

    if (GUISpecifiedA2 > FULLSCALE_COIL_DAC_VOLTAGE) {

        transmitError(A2_OUT_OF_RANGE);
        local_a2 = 0x7FFF;
        implementedA2 = FULLSCALE_COIL_DAC_VOLTAGE;

    } else if (GUISpecifiedA2 < 0.0) {

        transmitError(A2_OUT_OF_RANGE);
        local_a2 = 0x0000;
        implementedA2 = 0.0;

    } else {

        local_a2 = _Q15ftoi(GUISpecifiedA2 / FULLSCALE_COIL_DAC_VOLTAGE);
        implementedA2 = _itofQ15(local_a2) * FULLSCALE_COIL_DAC_VOLTAGE;

    }

    maxMeasurementSamples = MAX_MEASUREMENT_SAMPLES;
    minMeasurementSamples = MIN_MEASUREMENT_SAMPLES;

    if (GUISpecifiedF1 > MAX_OUTPUT_HZ) {
        transmitError(F1_OUT_OF_RANGE);
        local_f1 = _Q15ftoi(MAX_OUTPUT_HZ * TWICE_SAMPLE_PERIOD);
        implementedF1 = MAX_OUTPUT_HZ;
        local_bridge_balance_frequency = DEFAULT_BALANCE_FREQUENCY;//set in case we need to balance the bridge

    } else if (GUISpecifiedF1 < 0) {
        transmitError(F1_OUT_OF_RANGE);
        local_f1 = 0;
        implementedF1 = 0.0;
        local_bridge_balance_frequency = DEFAULT_BALANCE_FREQUENCY;//set in case we need to balance the bridge
    } else {
        local_f1 = _Q15ftoi(GUISpecifiedF1 * TWICE_SAMPLE_PERIOD);
        implementedF1 = _itofQ15(local_f1) * HALF_SAMPLE_RATE;
        local_bridge_balance_frequency = local_f1;
    }

    if (GUISpecifiedF2 > MAX_OUTPUT_HZ) {
        transmitError(F2_OUT_OF_RANGE);
        local_f2 = _Q15ftoi(MAX_OUTPUT_HZ * TWICE_SAMPLE_PERIOD);
        /*
         * truncate the lowest 3 bits so that the coil tone hits 0rad at least
         * every 2 ^ 16 / 2 ^3 = 8192 samples
         */
        local_f2 &= 0xFFF8;
        implementedF2 = _itofQ15(local_f2) * HALF_SAMPLE_RATE;
    } else if (GUISpecifiedF2 < 0) {
        transmitError(F2_OUT_OF_RANGE);
        local_f2 = 0;
        implementedF2 = 0.0;
    } else {
        local_f2 = _Q15ftoi(GUISpecifiedF2 * TWICE_SAMPLE_PERIOD);

        /*
         * truncate the lowest 3 bits so that the coil tone hits 0rad at least
         * every 2 ^ 16 / 2 ^ 3 = 8192 samples
         */
        local_f2 &= 0xFFF8;
        implementedF2 = _itofQ15(local_f2) * HALF_SAMPLE_RATE;
    }

    local_f1PlusF2OutOfRange = false;
    if (GUISpecifiedF1 + GUISpecifiedF2 > MAX_OUTPUT_HZ) {
        local_f1PlusF2OutOfRange = true;
        transmitError(F1_PLUS_F2_OUT_OF_RANGE);
        local_fsum = 0;
        implementedFSum = 0;
    } else {
        local_fsum = local_f1 + local_f2;
        implementedFSum = implementedF1 + implementedF2;
    }
    local_fdiff = _Q15abs(local_f1 - local_f2);
    implementedFDiff = abs(implementedF1 - implementedF2);

    tempTime = GUISpecifiedT * SAMPLE_RATE;
    if (tempTime > maxMeasurementSamples) {
        transmitError(T_OUT_OF_RANGE);
        local_T = MAX_MEASUREMENT_SAMPLES;
        implementedT = maxMeasurementSamples / SAMPLE_RATE;
    } else if (tempTime < minMeasurementSamples) {
        transmitError(T_OUT_OF_RANGE);
        local_T = MIN_MEASUREMENT_SAMPLES;
        implementedT = minMeasurementSamples / SAMPLE_RATE;
    } else {
        local_T = (uint32_t)tempTime;
        implementedT = (float)local_T / SAMPLE_RATE;
    }

    switch(GUISpeciedBridgeGainFactor)
    {
        case 1:
            local_gain_factor = 0;
            implementedBridgeGainFactor = 1;
            break;
        case 2:
            local_gain_factor = 1;
            implementedBridgeGainFactor = 2;
            break;
        case 4:
            local_gain_factor = 2;
            implementedBridgeGainFactor = 4;
            break;
        case 8:
            local_gain_factor = 3;
            implementedBridgeGainFactor = 8;
            break;
        case 16:
            local_gain_factor = 4;
            implementedBridgeGainFactor = 16;
            break;
        default:
            transmitError(INVALID_DIGITAL_GAIN_VALUE);
            local_gain_factor = 0;
            implementedBridgeGainFactor = 1;
            break;
    }

    if (GUISpecifiedBridgeAnalogGain < BRIDGE_ADC_BUFFER_MIN_GAIN) {
        transmitError(ANALOG_GAIN_OUT_OF_RANGE);
        local_u24_code = 0x00;
        implementedBridgeGain = BRIDGE_ADC_BUFFER_MIN_GAIN;
        local_inverseBridgeAnalogGain = INVERSE_BRIDGE_ANALOG_MIN_GAIN;
    } else if (GUISpecifiedBridgeAnalogGain > BRIDGE_ADC_BUFFER_MAX_GAIN) {
        transmitError(ANALOG_GAIN_OUT_OF_RANGE);
        local_u24_code = 0xFF;
        implementedBridgeGain = BRIDGE_ADC_BUFFER_MAX_GAIN;
        local_inverseBridgeAnalogGain = INVERSE_BRIDGE_ANALOG_MAX_GAIN;
    } else {
        local_u24_code = getU24CodeFromBrdigeBufGain(GUISpecifiedBridgeAnalogGain);
        implementedBridgeGain = getBridgeBufGainFromU24Code(local_u24_code);
        local_inverseBridgeAnalogGain = getBridgeInverseGainFromU24Code(local_u24_code);
    }

    startPayload_confirmToGUI[0] = CONFIRM_START_COMMAND;
    startPayload_confirmToGUI[1] = 0x15;
    float_to_bytes(implementedA1, &startPayload_confirmToGUI[2]);
    float_to_bytes(implementedF1, &startPayload_confirmToGUI[6]);
    float_to_bytes(implementedA2, &startPayload_confirmToGUI[10]);
    float_to_bytes(implementedF2, &startPayload_confirmToGUI[14]);
    float_to_bytes(implementedT, &startPayload_confirmToGUI[18]);
    float_to_bytes(implementedBridgeGain, &startPayload_confirmToGUI[22]);
    startPayload_confirmToGUI[26] = implementedBridgeGainFactor;

    startPayload_confirmToGUI[27] = 0;
    for (i = 0; i < 27; i++)
    {
        startPayload_confirmToGUI[27] = startPayload_confirmToGUI[27] ^ startPayload_confirmToGUI[i];
    }
    send(startPayload_confirmToGUI, 28);

    START_ATOMIC();//begin critical section; must be atomic!
    measurementTime = local_T;
    f1 = local_f1;
    f2 = local_f2;
    fdiff = local_fdiff;
    fsum = local_fsum;
    a1 = local_a1;
    a2 = local_a2;
    bridgeADCGainFactor = local_gain_factor;
    u24_code = local_u24_code;
    inverseBridgeAnalogGain = local_inverseBridgeAnalogGain;
    f1PlusF2OutOfRange = local_f1PlusF2OutOfRange;
    bridge_balance_amplitude = local_bridge_balance_amplitude;
    bridge_balance_frequency = local_bridge_balance_frequency;
    global_state = RAMP_DOWN_COIL_RESTART;
    END_ATOMIC();//end critical section

}

void uart_Init (void)
{
    uint8_t junk;

    /***************************************************************************
     * initialization of global variables
     **************************************************************************/

    sensorAddressTable = sensorAddressTableAllocated;
    measurementTime = SAMPLE_RATE;
    f1 = 0;
    f2 = 0;
    fdiff = 0;
    fsum = 0;
    bridgeADCGainFactor = 1;
    f1PlusF2OutOfRange = false;
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

    //RP100/RF4/PIN31 connects to RX_BT
    //PORTF is digital by default
    RPOR8bits.RP99R = 0x03;//route U2TX to RP99

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
    IEC0bits.U1TXIE = 0;        //enable UART TX Interrupt
    U1STAbits.UTXEN = 1;        //enable UART.TXEN

    IEC0bits.U1RXIE = 0;         //disable RX interrupt
    IPC2bits.U1RXIP = 3;	//Receive interrupt priority
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
    IEC1bits.U2TXIE = 0;
    U2STAbits.UTXEN = 1;        //enable UART.TXEN

    IEC1bits.U2RXIE = 0;        //disable RX interrupt
    IPC7bits.U2RXIP = 3;	//Receive interrupt priority
    U2STAbits.URXISEL = 0;


    /***************************************************************************
     * clear interrupt flags, clear buffers, enable interrupts
     **************************************************************************/

#ifndef SIMULATION_MODE
    busy_wait_ms(10);           //wait for the FTDI to reset
    FTDI_RST_BAR = 1;           //start the FTDI
    busy_wait_ms(50);           //wait for the FTDI to start
#endif

    IFS0bits.U1RXIF = 0;        //clear U1RXIF
    U1STAbits.OERR = 0;         //clear overflow error flag
    while (1 == U1STAbits.URXDA) {
        junk = U1RXREG;//clear the input buffer
    }
    IEC0bits.U1RXIE = 1;         //enable RX interrupt

    IFS1bits.U2RXIF = 0;        //clear U2RXIF
    U2STAbits.OERR = 0;         //clear overflow error flag
    while (1 == U2STAbits.URXDA) {
        junk = U2RXREG;//clear the input buffer
    }
    IEC1bits.U2RXIE = 1;        //enable RX interrupt
}

void __attribute__((__interrupt__, no_auto_psv)) _U1RXInterrupt(void)
{
    static uint8_t rxState = RX_IDLE;
    static uint16_t endDataRxPointer = 0;
    static uint16_t numBytesInPayload = 0;
    static uint8_t USB_RxBuffer[USB_RX_BUF_SZ] ={0};
    uint8_t rxByte;
    
    IFS0bits.U1RXIF = 0;

    if(0 == USB_5V_DETECT) {

        if (1 == U1STAbits.OERR) {
            U1STAbits.OERR = 0;//this also clears U1RXREG
        } else {
            while (1 == U1STAbits.URXDA) {
                rxByte = U1RXREG;//clear the input buffer
            }
        }

        rxState = IDLE;
        numBytesInPayload = 0;

        /*
         * USB is not connected;
         * Return after the input buffer and counters are cleared.
         */
        return;
    }

    if (1 == U1STAbits.OERR) {

        U1STAbits.OERR = 0;//this also clears U1RXREG

        //clear counters
        //payloadBytesReceived = 0;
        //numBytesInPayload = 0xFF;

        transmitError(UART_RX_BUFFER_OVERFLOW);

        /*
         * An overflow has occurred
         * Return after the input buffer and counters are cleared.
         */
        return;
    }

    while (U1STAbits.URXDA) {
        rxByte = U1RXREG;
        
        switch (rxState) {
            case RX_IDLE:
                if (START_FLAG == rxByte) {
                    rxState = RX_MID_PAYLOAD;
                    numBytesInPayload = 0;
                }
                break;
            case RX_MID_PAYLOAD:
                if (ESCAPE_FLAG == rxByte) {
                    rxState = RX_ESCAPE;
                } else if (STOP_FLAG == rxByte) {
                    if (numBytesInPayload > (endDataRxPointer + 1)) {
                        receive(true, USB_RxBuffer, endDataRxPointer + USB_RX_BUF_SZ - numBytesInPayload, numBytesInPayload);
                    } else {
                        receive(true, USB_RxBuffer, endDataRxPointer - numBytesInPayload, numBytesInPayload);
                    }
                    rxState = RX_IDLE;
                } else if (START_FLAG == rxByte) {
                    transmitError(MALFORMED_PACKET_RECEIVED);
                    rxState = RX_MID_PAYLOAD;
                    numBytesInPayload = 0;
                } else {
                    USB_RxBuffer[endDataRxPointer] = rxByte;
                    ++endDataRxPointer;
                    ++numBytesInPayload;
                    if (USB_RX_BUF_SZ == endDataRxPointer) {
                        endDataRxPointer = 0;
                    }
                }
                break;
            case RX_ESCAPE:
                if (START_FLAG == rxByte) {
                    transmitError(MALFORMED_PACKET_RECEIVED);
                    rxState = RX_MID_PAYLOAD;
                    numBytesInPayload = 0;
                    break;
                }

                rxByte ^= ESCAPE_XOR;

                if (START_FLAG == rxByte || ESCAPE_FLAG == rxByte || STOP_FLAG == rxByte) {
                    USB_RxBuffer[endDataRxPointer] = rxByte;
                    ++endDataRxPointer;
                    ++numBytesInPayload;
                    if (USB_RX_BUF_SZ == endDataRxPointer) {
                        endDataRxPointer = 0;
                    }
                    rxState = RX_MID_PAYLOAD;
                } else {
                    transmitError(MALFORMED_PACKET_RECEIVED);
                    rxState = RX_IDLE;
                }
                break;
            default:
                rxState = RX_IDLE;
                break;
        }
    }
}

void __attribute__((__interrupt__, no_auto_psv)) _U2RXInterrupt(void)
{
    static uint8_t rxState = RX_IDLE;
    static uint16_t endDataRxPointer = 0;
    static uint16_t numBytesInPayload = 0;
    static uint8_t BT_RxBuffer[BT_RX_BUF_SZ] ={0};
    uint8_t rxByte;
    
    IFS1bits.U2RXIF = 0;

    if(1 == USB_5V_DETECT) {

        if (1 == U2STAbits.OERR) {
            U2STAbits.OERR = 0;//this also clears U1RXREG
        } else {
            while (1 == U2STAbits.URXDA) {
                rxByte = U2RXREG;//clear the input buffer
            }
        }

        rxState = IDLE;
        numBytesInPayload = 0;

        /*
         * USB is connected;
         * Return after the input buffer and counters are cleared.
         */
        return;
    }

    if (1 == U2STAbits.OERR) {

        U2STAbits.OERR = 0;//this also clears U1RXREG

        //clear counters
        //payloadBytesReceived = 0;
        //numBytesInPayload = 0xFF;

        transmitError(UART_RX_BUFFER_OVERFLOW);

        /*
         * An overflow has occurred
         * Return after the input buffer and counters are cleared.
         */
        return;
    }

    while (U2STAbits.URXDA) {
        rxByte = U2RXREG;
        
        switch (rxState) {
            case RX_IDLE:
                if (START_FLAG == rxByte) {
                    rxState = RX_MID_PAYLOAD;
                    numBytesInPayload = 0;
                }
                break;
            case RX_MID_PAYLOAD:
                if (ESCAPE_FLAG == rxByte) {
                    rxState = RX_ESCAPE;
                } else if (STOP_FLAG == rxByte) {
                    if (numBytesInPayload > (endDataRxPointer + 1)) {
                        receive(false, BT_RxBuffer, endDataRxPointer + BT_RX_BUF_SZ - numBytesInPayload, numBytesInPayload);
                    } else {
                        receive(false, BT_RxBuffer, endDataRxPointer - numBytesInPayload, numBytesInPayload);
                    }
                    rxState = RX_IDLE;
                } else if (START_FLAG == rxByte) {
                    transmitError(MALFORMED_PACKET_RECEIVED);
                    rxState = RX_MID_PAYLOAD;
                    numBytesInPayload = 0;
                } else {
                    BT_RxBuffer[endDataRxPointer] = rxByte;
                    ++endDataRxPointer;
                    ++numBytesInPayload;
                    if (BT_RX_BUF_SZ == endDataRxPointer) {
                        endDataRxPointer = 0;
                    }
                }
                break;
            case RX_ESCAPE:
                if (START_FLAG == rxByte) {
                    transmitError(MALFORMED_PACKET_RECEIVED);
                    rxState = RX_MID_PAYLOAD;
                    numBytesInPayload = 0;
                    break;
                }

                rxByte ^= ESCAPE_XOR;

                if (START_FLAG == rxByte || ESCAPE_FLAG == rxByte || STOP_FLAG == rxByte) {
                    BT_RxBuffer[endDataRxPointer] = rxByte;
                    ++endDataRxPointer;
                    ++numBytesInPayload;
                    if (BT_RX_BUF_SZ == endDataRxPointer) {
                        endDataRxPointer = 0;
                    }
                    rxState = RX_MID_PAYLOAD;
                } else {
                    transmitError(MALFORMED_PACKET_RECEIVED);
                    rxState = RX_IDLE;
                }
                break;
            default:
                rxState = RX_IDLE;
                break;
        }
    }
}

void __attribute__((__interrupt__, no_auto_psv)) _U1TXInterrupt(void)
{
    IFS0bits.U1TXIF = 0;
    IEC0bits.U1TXIE = 0;
    usbTxWorker();//The shift register got full. But now it's empty again.  Let's TX!
}

void __attribute__((__interrupt__, no_auto_psv)) _U2TXInterrupt(void)
{
    IFS1bits.U2TXIF = 0;
    IEC1bits.U2TXIE = 0;
    btTxWorker();//The shift register got full. But now it's empty again.  Let's TX!
}

inline void
transmitResults(uint8_t sensor, __eds__ float_array_t *phaseAngle,
                __eds__ float_array_t *amplitude, bool bridgeADCClip,
                bool coilADCClip, bool bridgeDigitalClip)
{
    uint8_t txbuffer[45],i;

    txbuffer[0] = REPORT_VALUES;
    txbuffer[1] = 0x29;
    txbuffer[2] = sensor;

#if !defined(MESURE_F2_AT_BRIDGE) && !defined(MESURE_F2_AT_COIL)
    float ZERO = 0.0;
#endif
    float_to_bytes(amplitude->bridge_f1, &txbuffer[3]);
    float_to_bytes(phaseAngle->bridge_f1, &txbuffer[7]);
#ifdef MEASURE_F2_AT_BRIDGE
    float_to_bytes(amplitude->bridge_f2, &txbuffer[11]);
    float_to_bytes(phaseAngle->bridge_f2, &txbuffer[15]);
#else
    float_to_bytes(ZERO, &txbuffer[11]);
    float_to_bytes(ZERO, &txbuffer[15]);
#endif
    float_to_bytes(amplitude->bridge_fdiff, &txbuffer[19]);
    float_to_bytes(phaseAngle->bridge_fdiff, &txbuffer[23]);
    float_to_bytes(amplitude->bridge_fsum, &txbuffer[27]);
    float_to_bytes(phaseAngle->bridge_fsum, &txbuffer[31]);
#ifdef MEASURE_F2_AT_COIL
    float_to_bytes(amplitude->coil_f2, &txbuffer[35]);
    float_to_bytes(phaseAngle->coil_f2, &txbuffer[39]);
#else
    float_to_bytes(ZERO, &txbuffer[35]);
    float_to_bytes(ZERO, &txbuffer[39]);
#endif

    txbuffer[43] =0;
    for (i = 1; i < 43; i++) {
        txbuffer[43] = txbuffer[43] ^ txbuffer[i];
    }
    send (txbuffer, 44);

    if (bridgeADCClip) {
        transmitError(BRIDGE_ADC_CLIP);
    }
    if (coilADCClip) {
        transmitError(COIL_ADC_CLIP);
    }
    if (bridgeDigitalClip) {
        transmitError(BRIDGE_DIGITAL_CLIP);
    }
}

static inline void
float_to_bytes(float myFloat, uint8_t *array)
{
    *array = *((__eds__ uint8_t*)&myFloat);
    ++array;
    *array = *((__eds__ uint8_t*)&myFloat + 1);
    ++array;
    *array =*((__eds__ uint8_t*)&myFloat + 2);
    ++array;
    *array = *((__eds__ uint8_t*)&myFloat + 3);
}

inline void
transmitError(uint8_t errorCode)
{
    static uint8_t errorpayload[4];
    errorpayload[0] = REPORT_ERROR;
    errorpayload[1] = 0x01;
    errorpayload[2] = errorCode;
    errorpayload[3] = errorpayload[0] ^ errorpayload[1] ^ errorpayload[2];
    send(errorpayload, 4);
}

static inline void
send(uint8_t *array, uint8_t numBytes)
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

static void
usbTxWorker(void)
{
    START_ATOMIC();//begin critical section; must be atomic!
    /* continue if there is data to transmit and the U2TXREG isn't full */
    while (usbTxEnd != usbTxCur && U1STAbits.UTXBF == 0)
    {
        U1TXREG = usbTxBuf[usbTxCur];
        ++usbTxCur;
        if (USB_TX_BUF_SIZE == usbTxCur)
        {
            usbTxCur = 0;
        }
    }
    if (usbTxEnd != usbTxCur) {
        IEC0bits.U1TXIE = 1;//enable interrupt to call the worker again
    }
    END_ATOMIC();//end critical section
}

static void
btTxWorker(void)
{
    START_ATOMIC();//begin critical section; must be atomic!
    /* continue if there is data to transmit and the U2TXREG isn't full */
    while (btTxEnd != btTxCur && U2STAbits.UTXBF == 0)
    {
        U2TXREG = btTxBuf[btTxCur];
        ++btTxCur;
        if (BT_TX_BUF_SIZE == btTxCur)
        {
            btTxCur = 0;
        }
    }
    if (usbTxEnd != usbTxCur) {
        IEC1bits.U2TXIE = 1;//enable interrupt to call the worker again
    }
    END_ATOMIC();//end critical section
}

static inline bool
copyToUSBTxBuf(uint8_t *array, uint16_t numBytes)
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

    if (bufSpaceAvl < numBytes * 2 + 2)//encoded payload is 2x the size with all escaped characters, + 2 for START and STOP flag
    {
        spawnTxThread = false;//no message transmitted; a buffer overflow may occur
    }
    else
    {
        usbTxBuf[usbTxEnd] = START_FLAG;
        ++usbTxEnd;
        if (usbTxEnd == USB_TX_BUF_SIZE)
        {
            usbTxEnd = 0;//this is a circular buffer; wrap back around
        }
        for (i = 0; i < numBytes; ++i)
        {
            if (array[i] == START_FLAG || array[i] == ESCAPE_FLAG || array[i] == STOP_FLAG) {
                usbTxBuf[usbTxEnd] = ESCAPE_FLAG;
                ++usbTxEnd;
                if (usbTxEnd == USB_TX_BUF_SIZE)
                {
                    usbTxEnd = 0;//this is a circular buffer; wrap back around
                }
                usbTxBuf[usbTxEnd] = array[i] ^ ESCAPE_XOR;
                ++usbTxEnd;
                if (usbTxEnd == USB_TX_BUF_SIZE)
                {
                    usbTxEnd = 0;//this is a circular buffer; wrap back around
                }
            } else {
                usbTxBuf[usbTxEnd] = array[i];
                ++usbTxEnd;
                if (usbTxEnd == USB_TX_BUF_SIZE)
                {
                    usbTxEnd = 0;//this is a circular buffer; wrap back around
                }
            }
        }
        usbTxBuf[usbTxEnd] = STOP_FLAG;
        ++usbTxEnd;
        if (usbTxEnd == USB_TX_BUF_SIZE)
        {
            usbTxEnd = 0;//this is a circular buffer; wrap back around
        }
    }
    END_ATOMIC();//end critical section
    return spawnTxThread;
}

static inline bool
copyToBTTxBuf(uint8_t *array, uint16_t numBytes)
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

    if (bufSpaceAvl < numBytes * 2 + 2)//encoded payload is 2x the size with all escaped characters, + 2 for START and STOP flag
    {
        spawnTxThread = false;//no message transmitted; a buffer overflow may occur
    }
    else
    {
        btTxBuf[btTxEnd] = START_FLAG;
        ++btTxEnd;
        if (btTxEnd == BT_TX_BUF_SIZE)
        {
            btTxEnd = 0;//this is a circular buffer; wrap back around
        }
        for (i = 0; i < numBytes; ++i)
        {
            if (array[i] == START_FLAG || array[i] == ESCAPE_FLAG || array[i] == STOP_FLAG) {
                btTxBuf[btTxEnd] = ESCAPE_FLAG;
                ++btTxEnd;
                if (btTxEnd == BT_TX_BUF_SIZE)
                {
                    btTxEnd = 0;//this is a circular buffer; wrap back around
                }
                btTxBuf[btTxEnd] = array[i] ^ ESCAPE_XOR;
                ++btTxEnd;
                if (btTxEnd == BT_TX_BUF_SIZE)
                {
                    btTxEnd = 0;//this is a circular buffer; wrap back around
                }
            } else {
                btTxBuf[btTxEnd] = array[i];
                ++btTxEnd;
                if (btTxEnd == BT_TX_BUF_SIZE)
                {
                    btTxEnd = 0;//this is a circular buffer; wrap back around
                }
            }
        }
        btTxBuf[btTxEnd] = STOP_FLAG;
        ++btTxEnd;
        if (btTxEnd == BT_TX_BUF_SIZE)
        {
            btTxEnd = 0;//this is a circular buffer; wrap back around
        }
    }
    END_ATOMIC();//end critical section
    return spawnTxThread;
}

static inline void
receive (bool rxFromUSB, uint8_t *array, uint16_t rxPointer,
         uint8_t sizeOfPayload)
{
    uint16_t rx_buf_sz;
    uint8_t i, payload[MAX_RX_PAYLOAD_SIZE] = {0}, xor_byte = 0;

    if (true == rxFromUSB) {
        rx_buf_sz = USB_RX_BUF_SZ;
    } else {
        rx_buf_sz = BT_RX_BUF_SZ;
    }

    for(i = 0; i < sizeOfPayload; i++) {
        if (rxPointer == rx_buf_sz) {
            rxPointer = 0;
        }
        payload[i] = array[rxPointer];
        ++rxPointer;
    }

    //check XOR
    for (i = 0; i < (sizeOfPayload - 1); i++) {

        xor_byte = xor_byte ^ payload[i];

    } if (xor_byte != payload[i]) {

        transmitError(Bad_Packet_XOR);

    } else {

        switch (payload[0]) {

            case START_COMMAND:

                decodeStartCommand(payload, sizeOfPayload);
                break;

            case STOP_COMMAND:

                if (STOP_PAYLOAD_SIZE != sizeOfPayload) {

                    transmitError(RECEIVED_PACKET_WITH_INCORRECT_SIZE);

                } else {

                    START_ATOMIC();//begin critical section; must be atomic!
                    global_state = RAMP_DOWN_COIL_QUIT;
                    END_ATOMIC();//end critical section
                }
                break;

            case CONFIG_MUX_ADDRESSING:
            {
                uint8_t newNumSensors;
                bool copyInNewTable = false;

                if(sizeOfPayload > MAX_RX_PAYLOAD_SIZE) {

                    transmitError(RECEIVED_PACKET_WITH_INCORRECT_SIZE);

                } else {

                    START_ATOMIC();//begin critical section; must be atomic!

                    /*
                     * check to see if the new table is different than the one
                     * in memory
                     */
                    newNumSensors = sizeOfPayload - 3;

                    if (newNumSensors == 0) {

                        numberOfSensors = 1;
                        sensorAddressTable[0] = DEFAULT_SENSOR_ADDRESS;
                        sensorRBridgeTableValid = false;

                    } else if (newNumSensors != numberOfSensors) {

                        copyInNewTable = true;
                        sensorRBridgeTableValid = false;

                    } else {

                        for (i = 0; i < numberOfSensors; i++) {

                            if (sensorAddressTable[i] != payload[2+i]) {

                                copyInNewTable = true;
                                sensorRBridgeTableValid = false;
                                break;
                            }

                        }
                    }

                    if (copyInNewTable) {

                        numberOfSensors = newNumSensors;

                        for (i = 0; i < numberOfSensors; i++) {

                            sensorAddressTable[i] = payload[2+i];

                        }
                    }
                    END_ATOMIC();//end critical section

                    payload[0] = CONFIRM_MUX_ADDRESSING;
                    payload[1] = 0;
                    payload[2] = payload[0] ^ payload[1];
                    send(payload, 3);
                }
                break;
            }
            case BALANCE_WHEATSTONE_BRIDGE:

                decodeBalanceBridgeCommand(payload, sizeOfPayload);
                break;

            default:

                transmitError(UNRECOGNIZED_COMMAND_RECEIVED);
                break;
        }

    }
}

static inline void
decodeBalanceBridgeCommand(uint8_t *payload, uint8_t sizeOfPayload)
{
    uint8_t confirm_payload[4];
    float GUISpecBalanceVolts;
    float GUISpecBalanceHz;

    if (BALANCE_PAYLOAD_SIZE != sizeOfPayload) {

        transmitError(RECEIVED_PACKET_WITH_INCORRECT_SIZE);

    } else {

        START_ATOMIC();//begin critical section; must be atomic!
        sensorRBridgeTableValid = false;
        *(__eds__ uint8_t *)&GUISpecBalanceVolts = payload[2];
        *((__eds__ uint8_t *)&GUISpecBalanceVolts + 1) = payload[3];
        *((__eds__ uint8_t *)&GUISpecBalanceVolts + 2) = payload[4];
        *((__eds__ uint8_t *)&GUISpecBalanceVolts + 3) = payload[5];

        *(__eds__ uint8_t *)&GUISpecBalanceHz = payload[6];
        *((__eds__ uint8_t *)&GUISpecBalanceHz + 1) = payload[7];
        *((__eds__ uint8_t *)&GUISpecBalanceHz + 2) = payload[8];
        *((__eds__ uint8_t *)&GUISpecBalanceHz + 3) = payload[9];

        if (GUISpecBalanceVolts > FULLSCALE_BRIDGE_DAC_VOLTAGE || GUISpecBalanceVolts < 0.0) {

            transmitError(BRIDGE_BALANCE_VOLTAGE_OUT_OF_RANGE);
            bridge_balance_amplitude = DEFAULT_BALANCE_AMPLITUDE;

        } else {

            bridge_balance_amplitude = _Q15ftoi(GUISpecBalanceVolts / FULLSCALE_BRIDGE_DAC_VOLTAGE);

        }

        if (GUISpecBalanceHz > MAX_OUTPUT_HZ || GUISpecBalanceHz < 0) {

            transmitError(BRIDGE_BALANCE_FREQUENCY_OUT_OF_RANGE);
            bridge_balance_frequency = DEFAULT_BALANCE_FREQUENCY;

        } else {

            bridge_balance_frequency = _Q15ftoi(GUISpecBalanceHz * TWICE_SAMPLE_PERIOD);

        }

        confirm_payload[0] = CONFIRM_BALANCE_BRIDGE;
        confirm_payload[1] = 0;
        confirm_payload[2] = confirm_payload[0] ^ confirm_payload[1];
        send(confirm_payload, 3);

        START_ATOMIC();//begin critical section; must be atomic!
        global_state = RAMP_DOWN_COIL_BALANCE_BRIDGE;
        END_ATOMIC();//end critical section

    }
}

static inline void
decodeStartCommand(uint8_t startpayload[], uint8_t sizeOfPayload)
{
    uint8_t i, k, array[4];
    float GUISpecifiedA1;
    float GUISpecifiedF1;
    float GUISpecifiedA2;
    float GUISpecifiedF2;
    float GUISpecifiedT;
    uint8_t GUISpeciedBridgeGainFactor;
    float GUISpecifiedBridgeAnalogGain = 30.0;

    if (START_PAYLOAD_SIZE != sizeOfPayload) {
        transmitError(RECEIVED_PACKET_WITH_INCORRECT_SIZE);
        return;
    }

    /* floating point data types begin from startpayload[2], so set k=2 */
    k=2;
    for(i=0; i<4; i++) {
        array[i] = startpayload[k];
        ++k;
    }

    GUISpecifiedA1 = *(__eds__ float *)&array;

    for(i=0; i<4; i++) {
        array[i] = startpayload[k];
        ++k;
    }
    GUISpecifiedF1 = *(__eds__ float *)&array;
    for(i=0; i<4; i++) {
        array[i] = startpayload[k];
        ++k;
    }
    GUISpecifiedA2 = *(__eds__ float *)&array;

    for(i=0; i<4; i++) {
    array[i] = startpayload[k];
        ++k;
    }
    GUISpecifiedF2 = *(__eds__ float *)&array;

    for(i=0; i<4; i++) {
        array[i] = startpayload[k];
        ++k;
    }
    GUISpecifiedT = *(__eds__ float *)&array;

    for(i=0; i<4; i++) {
        array[i] = startpayload[k];
        ++k;
    }
    GUISpecifiedBridgeAnalogGain = *(__eds__ float *)&array;

    GUISpeciedBridgeGainFactor = startpayload[k];

    processStartCommand(GUISpecifiedA1, GUISpecifiedF1, GUISpecifiedA2,
                        GUISpecifiedF2, GUISpecifiedT,
                        GUISpeciedBridgeGainFactor,
                        GUISpecifiedBridgeAnalogGain);

}
