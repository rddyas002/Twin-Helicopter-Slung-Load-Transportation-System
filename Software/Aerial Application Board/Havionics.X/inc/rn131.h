
#ifndef RN131_H
#define	RN131_H

#include <plib.h>
#include <stdbool.h>
#include <string.h>
#include <stdlib.h>
#include "HardwareProfile.h"
#include "GenericTypeDefs.h"

#ifdef RN131_H_IMPORT
	#define RN131_EXTERN
#else
	#define RN131_EXTERN extern
#endif

#define RN131_UART_PIN_RX       TRISFbits.TRISF2
#define RN131_UART_PIN_TX       TRISFbits.TRISF8
#define RN131_UART              UART1
#define RN131_DisableIntUARTTX  DisableIntU1TX
#define RN131_BAUD_RATE         (500000)
#define RN131_BUFFER_SIZE       256
#define RN131_DATA_PERIOD       (20)    // millisecond
#define RN131_DATA_PERIOD_DELTA (50000)
#define RN131_RX_TIMEOUT        (200)   // called every 1ms ~ to 0.25s
#define RN131_PROCESS_LATENCY   (210)   //us
#define RN131_RX_DATA_SIZE      (39)
#define RN131_PACKET_SIZEOF     (36)
#define RN131_OFFSET_FILTER_N   (10)

#define RN131_DMA_TX_PRIORITY   DMA_CHN_PRI2
#define RN131_DMA_RX_PRIORITY   DMA_CHN_PRI3
#define RN131_DMA_TX_CHANNEL    DMA_CHANNEL3
#define RN131_DMA_RX_CHANNEL    DMA_CHANNEL4

#define RN131_PATTERN_MATCH

// Interrupt Priorities
// DMATX
#define RN131_DMATX_IPL            ipl4
#define RN131_DMATX_PRIORITY       INT_PRIORITY_LEVEL_4
#define RN131_DMATX_SUBPRIORITY    INT_SUB_PRIORITY_LEVEL_2
// DMARX
#define RN131_DMARX_IPL            ipl4
#define RN131_DMARX_PRIORITY       INT_PRIORITY_LEVEL_4
#define RN131_DMARX_SUBPRIORITY    INT_SUB_PRIORITY_LEVEL_3

#define RN131_ENTER_COMMAND "$$$"
#define RN131_EXIT "exit\r"
#define RN131_SHOW_CONNECTION "show connection\r"
#define RN131_SSID_REDDI "set wlan ssid REDDI_Y\r"
#define RN131_PASS_REDDI "set wlan phr 3268df8c168c2534\r"
#define RN131_SSID_YashDroid "YashDroid\r"
#define RN131_PASS_YashDroid "reddi206502029\r"
#define RN131_SAVE "save\r"
#define RN131_REBOOT "reboot\r"
#define RN131_EVERYTHING "get everything\r"

typedef unsigned char UCHAR;

typedef struct{
    char tx_buffer[RN131_BUFFER_SIZE];
    BYTE tx_index;
    BYTE tx_msglen;
    char rx_buffer[RN131_BUFFER_SIZE];
    char rx_buffered[RN131_BUFFER_SIZE];
    BYTE rx_msglen;
    BYTE rxd_msglen;
    BYTE rx_index;
    bool DMA_transmit_active;

    // wifi settings
    bool DMA_MODE;

    unsigned short int key;
    UINT32 send_uavtime;

    UINT32 timeout_rx;  // time since last receive
}RN131_data_struct;

typedef struct{
    INT16 translation_x;
    INT16 translation_y;
    INT16 translation_z;

    INT16 velocity_x;
    INT16 velocity_y;
    INT16 velocity_z;

    float quaternion_0;
    float quaternion_1;
    float quaternion_2;
    float quaternion_3;

    UINT8 config;

    UINT16 received_key;
    UINT32 received_key_pctime;

    UINT32 received_uavtime;
}RN131_states_struct;

typedef struct{
    INT16 translation_x;
    INT16 translation_y;
    INT16 translation_z;

    INT16 velocity_x;
    INT16 velocity_y;
    INT16 velocity_z;

    float quaternion_0;
    float quaternion_1;
    float quaternion_2;
    float quaternion_3;

    UINT8 config;

    UINT16 received_key;
    UINT32 received_key_pctime;

    UINT8 checksum;
}RN131_packet_struct;

RN131_EXTERN void RN131_setupDMA(void);
RN131_EXTERN void RN131_decodeBinary(const char * rx_data);
RN131_EXTERN void RN131_decodeFixedData(void);
RN131_EXTERN int RN131_strlen(char data[], int max_len, char match);
RN131_EXTERN char * RN131_getRxDataPointer(void);
RN131_EXTERN BYTE RN131_getRxDataSize(void);
RN131_EXTERN void RN131_setRxIndexToZero(void);
RN131_EXTERN void RN131_DMATransmit(void);
RN131_EXTERN void RN131_addToBuffer(char * data, BYTE len);
RN131_EXTERN void RN131_write2wifi(const char * data);
RN131_EXTERN void RN131_writeBuffer(char * data, BYTE len);
RN131_EXTERN void RN131_setRxMsgLenToZero(void);
RN131_EXTERN void RN131_SendDataPacket(unsigned short int data[], unsigned char data_length);
RN131_EXTERN void RN131_logData(void);
RN131_EXTERN void RN131_logDataWireless(float a1, float a2, float a3);
RN131_EXTERN float RN131_get_tx(void);
RN131_EXTERN float RN131_get_ty(void);
RN131_EXTERN float RN131_get_tz(void);
RN131_EXTERN short int RN131_getTx(void);
RN131_EXTERN short int RN131_getTy(void);
RN131_EXTERN short int RN131_getTz(void);
RN131_EXTERN short int * RN131_getVx(void);
RN131_EXTERN short int * RN131_getVy(void);
RN131_EXTERN short int * RN131_getVz(void);
RN131_EXTERN bool RN131_extrapolatePosition(float * tx, float * ty, float * tz);
RN131_EXTERN float RN131_getQuaternion_q0(void);
RN131_EXTERN float RN131_getQuaternion_q1(void);
RN131_EXTERN float RN131_getQuaternion_q2(void);
RN131_EXTERN float RN131_getQuaternion_q3(void);
RN131_EXTERN bool RN131_timeoutInc(void);
RN131_EXTERN void RN131_clearTimeout(void);
RN131_EXTERN bool RN131_getTimeout(void);
RN131_EXTERN bool RN131_ASCIIHex2Nibble(const char * hex, UCHAR * nibble);
RN131_EXTERN bool RN131_ASCIIHex2Byte(const char * hex, UCHAR * byte);
RN131_EXTERN void RN131_setDataReceived(bool val);
RN131_EXTERN bool RN131_getDataReceived(void);
RN131_EXTERN unsigned short int RN131_getKey(void);
RN131_EXTERN unsigned short int RN131_getIncrKey(void);
RN131_EXTERN UINT32 RN131_getDelay_us(void);
RN131_EXTERN INT32 RN131_getOffset_us(void);
RN131_EXTERN UINT32 RN131_getDelay_us_0(void);
RN131_EXTERN INT32 RN131_getOffset_us_0(void);
RN131_EXTERN INT32 RN131_OffsetFilter(INT32 offset);
RN131_EXTERN INT32 * RN131_getFilteredOffset(void);
RN131_EXTERN void RN131_logSync(void);
RN131_EXTERN bool RN131_ekfStable(void);
RN131_EXTERN void RN131_setDataAvailable(bool val);
RN131_EXTERN bool RN131_dataAvailable(void);

// RN131 specific commands
RN131_EXTERN void RN131_enterCmdMode(void);
RN131_EXTERN void RN131_exitCmdMode(void);
RN131_EXTERN void RN131_showConnection(void);
RN131_EXTERN void RN131_setSSID(void);
RN131_EXTERN void RN131_setPhrase(void);
RN131_EXTERN void RN131_save(void);
RN131_EXTERN void RN131_reboot(void);
RN131_EXTERN void RN131_getEverything(void);
#endif	/* RN131_H */

