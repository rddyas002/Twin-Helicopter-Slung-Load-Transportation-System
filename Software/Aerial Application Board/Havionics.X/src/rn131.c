#define RN131_H_IMPORT

#include "../inc/rn131.h"
#include "../inc/io.h"

RN131_data_struct RN131_data;
RN131_states_struct RN131_states;
RN131_packet_struct RN131_packet;
bool RN131_dataReceived = false;

DmaChannel RN131_DMA_TX_CHN = RN131_DMA_TX_CHANNEL;
DmaChannel RN131_DMA_RX_CHN = RN131_DMA_RX_CHANNEL;

INT32 RN131_offset_us = 0;
UINT32 RN131_delay_us = 0;          // Round trip delay excluding processing time
// Initial offset and delay estimate
INT32 RN131_offset_us_0 = 0;
UINT32 RN131_delay_us_0 = 0;
volatile bool RN131_sync_complete = false;
INT32 RN131_filteredOffset = 0;
bool RN131_dataAvailable_bool = false;

void RN131_setupUART(void);
void RN131_setupDMA_TX(void);
void RN131_setupDMA_RX_pattern(void);
void RN131_setupDMA_RX_fixed(void);

void RN131_setupDMA(void){
    memset(&RN131_data, 0, sizeof(RN131_data_struct));
    memset(&RN131_states, 0, sizeof(RN131_states_struct));
    memset(&RN131_packet, 0, sizeof(RN131_packet_struct));
    RN131_states.config = 0x00;
    RN131_data.DMA_MODE = true;

    RN131_setupUART();

    UARTEnable(RN131_UART, UART_ENABLE_FLAGS(UART_PERIPHERAL | UART_RX | UART_TX));
    INTClearFlag(INT_SOURCE_UART_TX(RN131_UART));
    INTClearFlag(INT_SOURCE_UART_RX(RN131_UART));

    RN131_setupDMA_TX();
    RN131_setupDMA_RX_pattern();

    // Enable the chn
    DmaChnEnable(RN131_DMA_RX_CHN);
    // Enable DMA interrupt
    INTEnable(INT_SOURCE_DMA(RN131_DMA_RX_CHN), INT_ENABLED);
}

void RN131_setupUART(void){
    // Set UART IO direction
    RN131_UART_PIN_RX = 1;
    RN131_UART_PIN_TX = 0;

    // Configure UART1 to use flow control
    //UARTConfigure(RN131_UART, UART_ENABLE_PINS_CTS_RTS | UART_RTS_WHEN_RX_NOT_FULL | UART_ENABLE_HIGH_SPEED);
    UARTConfigure(RN131_UART, UART_ENABLE_HIGH_SPEED);
    // Generate an interrupt when TX buffer is empty or when there is data to pick up from the rx buffer
    UARTSetFifoMode(RN131_UART, UART_INTERRUPT_ON_TX_DONE | UART_INTERRUPT_ON_RX_NOT_EMPTY);
    // Set line parameters
    UARTSetLineControl(RN131_UART, UART_DATA_SIZE_8_BITS | UART_PARITY_NONE | UART_STOP_BITS_1);
    // Set data rate
    UARTSetDataRate(RN131_UART, GetPeripheralClock(), RN131_BAUD_RATE);
}

void RN131_setupDMA_TX(void){
    // setup DMA channel for transmitting data
    DmaChnOpen(RN131_DMA_TX_CHN, RN131_DMA_TX_PRIORITY, DMA_OPEN_DEFAULT);
    // Set the events: set UART1 tx interrupt to begin transfer
    // Stop transfer when block complete: block complete when patternMatch
    DmaChnSetEventControl(RN131_DMA_TX_CHN, DMA_EV_START_IRQ_EN|DMA_EV_START_IRQ(_UART1_TX_IRQ));
    // Set the transfer source and dest addresses, source and dest sizes and the cell size
    DmaChnSetTxfer(RN131_DMA_TX_CHN, (void*)&(RN131_data.tx_buffer[0]), (void*)&U1TXREG, RN131_data.tx_msglen, 1, 1);
    // Enable the transfer done interrupt
    DmaChnSetEvEnableFlags(RN131_DMA_TX_CHN, DMA_EV_BLOCK_DONE);
    // Set DMA3 interrupt priority
    INTSetVectorPriority(INT_VECTOR_DMA(RN131_DMA_TX_CHN), RN131_DMATX_PRIORITY);
    // Set DMA3 interrupt sub-priority
    INTSetVectorSubPriority(INT_VECTOR_DMA(RN131_DMA_TX_CHN), RN131_DMATX_SUBPRIORITY);
}

void RN131_setupDMA_RX_pattern(void){
    // Use a DMA channel for picking up data from wifi
    DmaChnOpen(RN131_DMA_RX_CHN, RN131_DMA_RX_PRIORITY, DMA_OPEN_AUTO);
    DmaChnSetMatchPattern(RN131_DMA_RX_CHN, '\n');	// interrupt when \n is encountered
    // Set the events: set UART1 rx interrupt to begin transfer
    // Stop transfer when block complete: block complete when patternMatch
    DmaChnSetEventControl(RN131_DMA_RX_CHN, DMA_EV_START_IRQ_EN|DMA_EV_MATCH_EN|DMA_EV_START_IRQ(_UART1_RX_IRQ));
    // Set the transfer source and dest addresses, source and dest sizes and the cell size
    DmaChnSetTxfer(RN131_DMA_RX_CHN, (void*)&U1RXREG, (void*)&(RN131_data.rx_buffer[0]), 1, RN131_BUFFER_SIZE, 1);
    // Enable the transfer done interrupt
    DmaChnSetEvEnableFlags(RN131_DMA_RX_CHN, DMA_EV_BLOCK_DONE);
    // Set DMA4 interrupt priority
    INTSetVectorPriority(INT_VECTOR_DMA(RN131_DMA_RX_CHN), RN131_DMARX_PRIORITY);
    // Set DMA4 interrupt sub-priority
    INTSetVectorSubPriority(INT_VECTOR_DMA(RN131_DMA_RX_CHN), RN131_DMARX_SUBPRIORITY);
}

void RN131_setupDMA_RX_fixed(void){
    // Use a DMA channel for picking up data from wifi
    DmaChnOpen(RN131_DMA_RX_CHN, RN131_DMA_RX_PRIORITY, DMA_OPEN_AUTO);
    // Set the events: set UART1 rx interrupt to begin transfer
    // Stop transfer when block complete: block complete when transferred bytes complete
    DmaChnSetEventControl(RN131_DMA_RX_CHN, DMA_EV_START_IRQ_EN|DMA_EV_START_IRQ(_UART1_RX_IRQ));
    // Set the transfer source and dest addresses, source and dest sizes and the cell size
    DmaChnSetTxfer(RN131_DMA_RX_CHN, (void*)&U1RXREG, (void*)&(RN131_data.rx_buffer[0]), 1, RN131_BUFFER_SIZE, RN131_RX_DATA_SIZE);
    // Enable the transfer done interrupt
    DmaChnSetEvEnableFlags(RN131_DMA_RX_CHN, DMA_EV_BLOCK_DONE);
    // Set DMA4 interrupt priority
    INTSetVectorPriority(INT_VECTOR_DMA(RN131_DMA_RX_CHN), RN131_DMARX_PRIORITY);
    // Set DMA4 interrupt sub-priority
    INTSetVectorSubPriority(INT_VECTOR_DMA(RN131_DMA_RX_CHN), RN131_DMARX_SUBPRIORITY);
}

/* Return & set function */
char * RN131_getRxDataPointer(void){
    return &RN131_data.rx_buffered[0];
}

BYTE RN131_getRxDataSize(void){
    return RN131_data.rxd_msglen;
}

void RN131_setRxIndexToZero(void){
    RN131_data.rx_index = 0;
}

void RN131_setRxMsgLenToZero(void){
    RN131_data.rx_msglen = 0;
}

void RN131_setDataReceived(bool val){
    RN131_dataReceived = val;
}

bool RN131_getDataReceived(void){
    return RN131_dataReceived;
}

void RN131_DMATransmit(void){
    if (!RN131_data.DMA_transmit_active){
        mU1TXClearIntFlag();
        RN131_data.DMA_transmit_active = true;
        DmaChnSetTxfer(RN131_DMA_TX_CHN, (void*)&(RN131_data.tx_buffer[0]), (void*)&U1TXREG, RN131_data.tx_msglen, 1, 1);
        // Make sure normal tx interrupts are disabled
        DisableIntU1TX;
        // Enable DMA interrupt
        INTEnable(INT_SOURCE_DMA(RN131_DMA_TX_CHN), INT_ENABLED);
        // Enable the channel
        DmaChnEnable(RN131_DMA_TX_CHN);
    }
}

void RN131_write2wifi(const char * data){
    BYTE len = strlen(data);
    memcpy(&RN131_data.tx_buffer[0],data,len);
    RN131_data.tx_msglen = len;
    RN131_DMATransmit();
}

void RN131_logDataWireless(float a1, float a2, float a3){
    char buffer[128];
    sprintf(&buffer[0],"%5.3f,%5.3f,%5.3f\r\n", a1, a2, a3);
    RN131_write2wifi(&buffer[0]);
}

void RN131_writeBuffer(char * data, BYTE len){
    memcpy(&RN131_data.tx_buffer[0],data,len);
    RN131_data.tx_msglen = len;
    RN131_DMATransmit();
}

void RN131_addToBuffer(char * data, BYTE len){
    memcpy(&(RN131_data.tx_buffer[RN131_data.tx_index]),data,len);
    RN131_data.tx_msglen += len;
    if (!RN131_data.DMA_transmit_active){
        RN131_data.DMA_transmit_active = true;
        // if tx buffer is empty, uart_tx will interrupt
        EnableIntU1TX;
    }
}

unsigned short int RN131_getKey(void){
    return RN131_data.key;
}

unsigned short int RN131_getIncrKey(void){
    return (++RN131_data.key);
}

void RN131_SendDataPacket(unsigned short int data[], unsigned char data_length){
    unsigned char MSB, LSB, i;
    unsigned char checksum = 0;
    
    if (!RN131_data.DMA_transmit_active){
        // expected data content
        //data[1:3] gyroscope
        //data[4:6] accelerometer
        //data[7:8] ultrasonic, voltage
        //data[9:12] servo_left, servo_right, servo_back, servo_tail_ref
        //data[13] ESC input
        //data[14] speed measurement

        RN131_data.tx_index = 0;
        RN131_data.tx_msglen = 0;

        // start sequence
        RN131_data.tx_buffer[RN131_data.tx_msglen++] = '*';
        RN131_data.tx_buffer[RN131_data.tx_msglen++] = '#';

        for (i = 0; i < data_length; i++){
            LSB = (data[i] & 0xFF);
            MSB = (data[i] >> 8);
            checksum ^= LSB;
            checksum ^= MSB;
            RN131_data.tx_buffer[RN131_data.tx_msglen++] = MSB;
            RN131_data.tx_buffer[RN131_data.tx_msglen++] = LSB;
        }
        RN131_data.tx_buffer[RN131_data.tx_msglen++] = checksum;

        // end sequence
        RN131_data.tx_buffer[RN131_data.tx_msglen++] = '@';
        RN131_data.tx_buffer[RN131_data.tx_msglen++] = '!';

        RN131_DMATransmit();
        // Log time client (helicopter) sends data (request)
        RN131_data.send_uavtime = IO_updateCoreTime_us();
        // The key is associated with this absolute time
        // Hence for time synchronisation we must check if this particular key
        // is received.
    }
}

void RN131_decodeFixedData(void){
    UINT16 checksum_calc = 0x00;
    int i = 0;
    // First check format and checksum
    if ((RN131_data.rxd_msglen != 0) &&
            (RN131_data.rx_buffered[0] == '*') &&
            (RN131_data.rx_buffered[1] == '#') &&
            (RN131_data.rx_buffered[38] == '\r')){
        for (i = 0; i < RN131_RX_DATA_SIZE - 2; i++){
            checksum_calc ^= RN131_data.rx_buffered[i];
        }

        if (checksum_calc == RN131_data.rx_buffered[37]){
            // copy direct into struct
            memcpy(&RN131_packet, &RN131_data.rx_buffered[2], RN131_PACKET_SIZEOF);
        }
        else
            return;
    }
}

void RN131_decodeData(void){
    int i = 0;
    UINT16 checksum_calc = 0x00;
    char checksum_lsb = 0, checksum_msb = 0;
    UINT16 checksum_sent = 0x0000;
    char data_bin[80] = {0};
    char data_bin_index = 0;

    if ((RN131_data.rxd_msglen != 0) &&
            (RN131_data.rx_buffered[0] == '*') &&
            (RN131_data.rx_buffered[1] == '#') &&
            (RN131_data.rx_buffered[76] == '\r')){
        // Compute checksum
        for (i = 0; i < 71; i++){
            checksum_calc ^= (UINT16)((unsigned char)RN131_data.rx_buffered[i] << 8) | (RN131_data.rx_buffered[i+1]);
        }
        
        // isolate received checksum
        RN131_ASCIIHex2Byte(&RN131_data.rx_buffered[72], &checksum_lsb);
        RN131_ASCIIHex2Byte(&RN131_data.rx_buffered[74], &checksum_msb);
        checksum_sent = (UINT16)(((UCHAR)checksum_msb << 8) | checksum_lsb);
        if (checksum_calc == checksum_sent){
            // convert ASCII hex data to binary
            for (i = 2; i < 71; i+=2){
                RN131_ASCIIHex2Byte(&RN131_data.rx_buffered[i], &data_bin[data_bin_index++]);
            }

            if (data_bin_index == 35){
                RN131_decodeBinary(&data_bin[0]);
                RN131_dataReceived = true;
                // Clear timeout since valid data has arrived
                RN131_data.timeout_rx = 0;                
            }
        }

    }
}

bool RN131_syncComplete(void){
    return RN131_sync_complete;
}

void RN131_decodeBinary(const char * rx_data){
    static INT64 offset_sum = 0;
    static UINT64 delay_sum = 0;
    static int init_count = 0;
    static int sync_count = 0;

    UINT16 temp_seq = 0;
    // Check sequence number [because udp does not ensure order under load]
    memcpy(&temp_seq, (rx_data + 33), 2);
    if (temp_seq < RN131_states.received_key){
//        char buffer[128];
//        int len = sprintf(buffer,"[%lu] ERR: SEQ_ORDER\r\n",IO_get_time_ms());
//        IO_dmesgMessage(&buffer[0],len);
        return;
    }

    memcpy(&RN131_states.translation_x, (rx_data), 2);
    memcpy(&RN131_states.translation_y, (rx_data + 2), 2);
    memcpy(&RN131_states.translation_z, (rx_data + 4), 2);

    memcpy(&RN131_states.velocity_x, (rx_data + 6), 2);
    memcpy(&RN131_states.velocity_y, (rx_data + 8), 2);
    memcpy(&RN131_states.velocity_z, (rx_data + 10), 2);

    memcpy(&RN131_states.quaternion_0, (rx_data + 12), 4);
    memcpy(&RN131_states.quaternion_1, (rx_data + 16), 4);
    memcpy(&RN131_states.quaternion_2, (rx_data + 20), 4);
    memcpy(&RN131_states.quaternion_3, (rx_data + 24), 4);

    memcpy(&RN131_states.config, (rx_data + 28), 1);

    memcpy(&RN131_states.received_key_pctime, (rx_data + 29), 4);

    memcpy(&RN131_states.received_key, (rx_data + 33), 2);

    RN131_states.received_uavtime = IO_updateCoreTime_us();

    // If keys match do an offset and delay estimate
    // Delay was found on average to be 4ms. This means that the key should have
    // more than double the average delay to return before key is over-written
    if (RN131_states.received_key == RN131_data.key){
        RN131_delay_us = RN131_states.received_uavtime - RN131_data.send_uavtime - RN131_PROCESS_LATENCY;
        RN131_offset_us = RN131_states.received_key_pctime - RN131_data.send_uavtime - (RN131_delay_us >> 1);

        RN131_OffsetFilter(RN131_offset_us);
        
        if (!RN131_sync_complete){
            // take 32 samples
            if (init_count++ < 32){
                delay_sum += RN131_delay_us;
                offset_sum += RN131_offset_us;
            }
            else{
                RN131_offset_us_0 = (INT32)(offset_sum >> 5);
                RN131_delay_us_0 = (UINT32)(delay_sum >> 5);
                // Adjust main clock by offset calculated
                IO_adjIO_time_us(RN131_offset_us_0);

                delay_sum = 0;
                offset_sum = 0;
                // 3 pass system synchronise
                if (sync_count++ > 3)
                    RN131_sync_complete = true;
            }
        }
    }
}

// Propagate last received position to current time
// Return: true if no timeout
bool RN131_extrapolatePosition(float * tx, float * ty, float * tz){
    /*
        |--------|-------|-----------------|---------|
     IMU read    |   pc calc done   Heli rx position |
             pc rx data<-----------dt--------------> |

     */

    // find the time difference between the current time and the time data was
    // received on the pc (pc computation time is ~ 300us)
    float dt = (IO_updateCoreTime_us() - RN131_states.received_key_pctime)/1e6;
    *tx = (float)RN131_states.translation_x/1000 + dt*(float)RN131_states.velocity_x/1000;
    *ty = (float)RN131_states.translation_y/1000 + dt*(float)RN131_states.velocity_y/1000;
    *tz = (float)RN131_states.translation_z/1000 + dt*(float)RN131_states.velocity_z/1000;

    // if not stable or there is a timeout return false
    if (!RN131_ekfStable() || RN131_getTimeout())
        return false;
    else
        return true;
}


bool RN131_ekfStable(void){
    return (RN131_states.config == 0x0B) ? true : false;
}

UINT32 RN131_getDelay_us(void){
    return RN131_delay_us;
}

INT32 RN131_getOffset_us(void){
    return RN131_offset_us;
}

UINT32 RN131_getDelay_us_0(void){
    return RN131_delay_us_0;
}

INT32 RN131_getOffset_us_0(void){
    return RN131_offset_us_0;
}

INT32 RN131_OffsetFilter(INT32 offset){
    static float data_buffer[RN131_OFFSET_FILTER_N + 1] = {0};
    static int current_point = RN131_OFFSET_FILTER_N;
    static float prev_output = 0;
    static bool first_enter = true;

    if (first_enter){
        int i;
        // initialise array variable
        for (i = 0; i < (RN131_OFFSET_FILTER_N + 1); i++){
            data_buffer[i] = (float)offset;
        }
        prev_output = (float)offset;
        first_enter = false;
    }

    // store current data
    data_buffer[current_point] = (float)offset;

    // next point to store data
    current_point = (current_point == RN131_OFFSET_FILTER_N) ? 0 : current_point + 1;

    prev_output = prev_output + ((float)offset - data_buffer[current_point])/RN131_OFFSET_FILTER_N;
    RN131_filteredOffset = (INT32)prev_output;
    return RN131_filteredOffset;
}

INT32 * RN131_getFilteredOffset(void){
    return &RN131_filteredOffset;
}

bool RN131_timeoutInc(void){
    if (RN131_data.timeout_rx++ > RN131_RX_TIMEOUT)
        return true;
    else
        return false;
}

bool RN131_getTimeout(void){
    if (RN131_data.timeout_rx > RN131_RX_TIMEOUT)
        return true;
    else
        return false;
}

void RN131_clearTimeout(void){
    RN131_data.timeout_rx = 0;
}

void RN131_logData(void){
    static UINT16 prev_rn131_seq = 0;
    char rn131_buffer[256];

    if (prev_rn131_seq >= RN131_states.received_key)
        return;

    prev_rn131_seq = RN131_states.received_key;

    int rn_sd = sprintf(&rn131_buffer[0],"%d,%d,%d,%d,%d,%d,%.4f,%.4f,%.4f,%.4f,%d,%lu\r\n",
            RN131_states.translation_x, RN131_states.translation_y, RN131_states.translation_z,
            RN131_states.velocity_x, RN131_states.velocity_y, RN131_states.velocity_z,
            RN131_states.quaternion_0, RN131_states.quaternion_1, RN131_states.quaternion_2, RN131_states.quaternion_3,
            RN131_states.received_key,
            RN131_states.received_uavtime);

    IO_logMessage(&rn131_buffer[0], rn_sd);
}

void RN131_logSync(void){
    char rn131_buffer[256];

    int rn_sd = sprintf(&rn131_buffer[0], "%d,%u,%lu\r\n", RN131_getOffset_us(), RN131_getDelay_us(), IO_updateCoreTime_us());

    IO_logMessage(&rn131_buffer[0], rn_sd);
}

bool RN131_ASCIIHex2Nibble(const char * hex, UCHAR * nibble){
    switch (*hex){
        case '0':
            *nibble = 0;
            break;
        case '1':
            *nibble = 1;
            break;
        case '2':
            *nibble = 2;
            break;
        case '3':
            *nibble = 3;
            break;
        case '4':
            *nibble = 4;
            break;
        case '5':
            *nibble = 5;
            break;
        case '6':
            *nibble = 6;
            break;
        case '7':
            *nibble = 7;
            break;
        case '8':
            *nibble = 8;
            break;
        case '9':
            *nibble = 9;
            break;
        case 'A':
        case 'a':
            *nibble = 10;
            break;
        case 'B':
        case 'b':
            *nibble = 11;
            break;
        case 'C':
        case 'c':
            *nibble = 12;
            break;
        case 'D':
        case 'd':
            *nibble = 13;
            break;
        case 'E':
        case 'e':
            *nibble = 14;
            break;
        case 'F':
        case 'f':
            *nibble = 15;
            break;
        default:
            return false;
            break;
    }
    return true;
}

bool RN131_ASCIIHex2Byte(const char * hex, UCHAR * byte){
    UCHAR msn = 0x00, lsn = 0x00;

    if (!RN131_ASCIIHex2Nibble(hex, &msn))
        return false;
    if (!RN131_ASCIIHex2Nibble(hex+1, &lsn))
        return false;

    *byte = ((UCHAR)msn << 4) | lsn;
    return true;
}

float RN131_get_tx(void){
    return (float)RN131_states.translation_x/1000;
}

float RN131_get_ty(void){
    return (float)RN131_states.translation_y/1000;
}

float RN131_get_tz(void){
    return (float)RN131_states.translation_z/1000;
}

short int RN131_getTx(void){
    return RN131_states.translation_x;
}

short int RN131_getTy(void){
    return RN131_states.translation_y;
}

short int RN131_getTz(void){
    return RN131_states.translation_z;
}

short int * RN131_getVx(void){
    return &RN131_states.velocity_x;
}

short int * RN131_getVy(void){
    return &RN131_states.velocity_y;
}

short int * RN131_getVz(void){
    return &RN131_states.velocity_z;
}

float RN131_getQuaternion_q0(void){
    return RN131_states.quaternion_0;
}

float RN131_getQuaternion_q1(void){
    return RN131_states.quaternion_1;
}

float RN131_getQuaternion_q2(void){
    return RN131_states.quaternion_2;
}

float RN131_getQuaternion_q3(void){
    return RN131_states.quaternion_3;
}

/**************************************************************/
/* RN131 specific commands */
void RN131_enterCmdMode(void){
    RN131_write2wifi(RN131_ENTER_COMMAND);
    // wait half a second
    IO_delayms(500);
}

void RN131_showConnection(void){
    RN131_write2wifi(RN131_SHOW_CONNECTION);
    IO_delayms(100);
}

void RN131_exitCmdMode(void){
    RN131_write2wifi(RN131_EXIT);
    IO_delayms(100);
}

void RN131_setSSID(void){
    RN131_write2wifi(RN131_SSID_REDDI);
    IO_delayms(100);
}

void RN131_setPhrase(void){
    RN131_write2wifi(RN131_PASS_REDDI);
    IO_delayms(100);
}

void RN131_save(void){
    RN131_write2wifi(RN131_SAVE);
    IO_delayms(100);
}

void RN131_reboot(void){
    RN131_write2wifi(RN131_REBOOT);
    IO_delayms(100);
}

void RN131_getEverything(void){
    RN131_write2wifi(RN131_EVERYTHING);
    IO_delayms(100);
}

int RN131_strlen(char data[], int max_len, char match){
    int i = 0;

    for (i = 0; i < max_len; i++){
        if (data[i] == match)
            return (i + 1);
    }

    return -1;
}

bool RN131_dataAvailable(void){
    return RN131_dataAvailable_bool;
}

void RN131_setDataAvailable(bool val){
    RN131_dataAvailable_bool = val;
}

/**************************************************************/

// Handler for wifi data tx using DMA channel
void __ISR(_DMA3_VECTOR, RN131_DMATX_IPL) wifiTxDmaHandler(void)
{
    int	evFlags;
    // Clear interrupt flag
    INTClearFlag(INT_SOURCE_DMA(RN131_DMA_TX_CHN));

    // Get event flags
    evFlags = DmaChnGetEvFlags(RN131_DMA_TX_CHN);

    // If interrupt due to a block event (termination) then proceed
    if(evFlags & DMA_EV_BLOCK_DONE)
    {
        DmaChnClrEvFlags(RN131_DMA_TX_CHN, DMA_EV_BLOCK_DONE);
	// Disable DMA channel
        INTEnable(INT_SOURCE_DMA(RN131_DMA_TX_CHN), INT_DISABLED);
	DmaChnDisable(RN131_DMA_TX_CHN);
        RN131_data.DMA_transmit_active = false;
    }
}

// Handler for wifi data rx using DMA channel
void __ISR(_DMA_4_VECTOR, RN131_DMARX_IPL) wifiRxDmaHandler(void)
{
    // This routine runs when a complete block has been received from the wifi module
    // The termination byte is '\n'
    int evFlags;

    // Clear interrupt flag
    INTClearFlag(INT_SOURCE_DMA(RN131_DMA_RX_CHN));
    // Get event flags
    evFlags = DmaChnGetEvFlags(RN131_DMA_RX_CHN);

    // If interrupt due to a block event (termination) then proceed
    if(evFlags & DMA_EV_BLOCK_DONE){
#if defined (RN131_PATTERN_MATCH)
        RN131_data.rx_msglen = strlen(&RN131_data.rx_buffer[0]);
        RN131_data.rx_buffer[RN131_data.rx_msglen] = 0;

        memcpy(&RN131_data.rx_buffered[0],&RN131_data.rx_buffer[0], RN131_data.rx_msglen);
        RN131_data.rxd_msglen = RN131_data.rx_msglen;
        #if !defined (USE_USB)
            RN131_decodeData();
        #else
            RN131_dataAvailable_bool = true;
        #endif

#else
        memcpy(&RN131_data.rx_buffered[0],&RN131_data.rx_buffer[0], RN131_RX_DATA_SIZE);
        RN131_data.rxd_msglen = RN131_RX_DATA_SIZE;
        RN131_decodeFixedData();
#endif
	DmaChnClrEvFlags(RN131_DMA_RX_CHN, DMA_EV_BLOCK_DONE);
        // Clear UART1Rx interrupt flag
	INTClearFlag(INT_SOURCE_UART_RX(RN131_UART));
    }
}

#undef RN131_H_IMPORT