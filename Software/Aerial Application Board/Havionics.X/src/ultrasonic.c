#define ULTRASONIC_H_IMPORT

#include "../inc/ultrasonic.h"
#include "../inc/io.h"

DmaChannel ULTRASONIC_DMA_RX_CHN = ULTRASONIC_DMA_RX_CHANNEL;

volatile char ULTRASONIC_buffer[ULTRASONIC_BUFFER_SIZE];
volatile unsigned short int ULTRASONIC_distance = 0;
volatile float ULTRASONIC_heave = 0.0;
volatile bool data_updated = false;

void ULTRASONIC_setup(void){
    // Enforce RX pin as input
    TRISFbits.TRISF4 = 1;

    UARTConfigure(UART2, UART_ENABLE_PINS_TX_RX_ONLY);
    UARTSetFifoMode(UART2, UART_INTERRUPT_ON_RX_NOT_EMPTY);
    UARTSetLineControl(UART2, UART_DATA_SIZE_8_BITS | UART_PARITY_NONE | UART_STOP_BITS_1);
    UARTSetDataRate(UART2, PB_FREQ, ULTRASONIC_BAUD);
    UARTEnable(UART2, UART_ENABLE_FLAGS(UART_PERIPHERAL | UART_RX));
    INTClearFlag(INT_SOURCE_UART_RX(UART2));

    // Configure the channel
    DmaChnOpen(ULTRASONIC_DMA_RX_CHN, DMA_CHN_PRI2, DMA_OPEN_AUTO);

    DmaChnSetMatchPattern(ULTRASONIC_DMA_RX_CHN, '\r');	// interrupt when \r is encountered

    // Set the events: set UART2 rx interrupt to begin transfer
    // Stop transfer when block complete: block complete when patternMatch
    DmaChnSetEventControl(ULTRASONIC_DMA_RX_CHN, DMA_EV_START_IRQ_EN|DMA_EV_MATCH_EN|DMA_EV_START_IRQ(_UART2_RX_IRQ));
    // set the transfer source and dest addresses, source and dest sizes and the cell size
    DmaChnSetTxfer(ULTRASONIC_DMA_RX_CHN, (void*)&U2RXREG, (void*)&(ULTRASONIC_buffer[0]), 1, ULTRASONIC_BUFFER_SIZE, 1);
    // Enable the transfer done interrupt
    DmaChnSetEvEnableFlags(ULTRASONIC_DMA_RX_CHN, DMA_EV_BLOCK_DONE);

    // set DMA1 interrupt priority
    INTSetVectorPriority(INT_VECTOR_DMA(ULTRASONIC_DMA_RX_CHN), ULTRASONIC_PRIORITY);
    // set DMA1 interrupt sub-priority
    INTSetVectorSubPriority(INT_VECTOR_DMA(ULTRASONIC_DMA_RX_CHN), ULTRASONIC_SUBPRIORITY);
    // Enable the ultrasonic_chn interrupt in the INT controller
    INTEnable(INT_SOURCE_DMA(ULTRASONIC_DMA_RX_CHN), INT_ENABLED);

    // enable the chn
    DmaChnEnable(ULTRASONIC_DMA_RX_CHN);
}

unsigned short int ULTRASONIC_getData(void){
    return ULTRASONIC_distance;
}

volatile float ULTRASONIC_getHeave(void){
    return ULTRASONIC_heave;
}

bool ULTRASONIC_isUpdated(void){
    return data_updated;
}

bool ULTRASONIC_setUpdatedFalse(void){
    data_updated = false;
}

void __ISR(_DMA1_VECTOR, ULTRASONIC_IPL) ultrasonicDmaHandler(void)
{
    int	evFlags;
    // Clear interrupt flag
    INTClearFlag(INT_SOURCE_DMA(ULTRASONIC_DMA_RX_CHN));
    // Get event flags
    evFlags=DmaChnGetEvFlags(ULTRASONIC_DMA_RX_CHN);
    
    // If interrupt due to a block event (termination) then proceed
    if(evFlags & DMA_EV_BLOCK_DONE)
    {
		// if a valid distance has been received, convert it to an integer and store
		if ((ULTRASONIC_buffer[4] == '\r')
			&& (ULTRASONIC_buffer[0] == 'R'))
		{
			unsigned char distance_s[4] = {0};
			unsigned char i = 0;
			for (i = 1; i < 4; i++)
			{
				distance_s[i-1] = ULTRASONIC_buffer[i];
			}
			// convert string to decimal
			ULTRASONIC_distance = atoi(distance_s);
                        
                        float Ultrasonic_distance = 0;

                        // if height changes instantly larger than 200cm treat as outlier
                        if (fabs(ULTRASONIC_heave - Ultrasonic_distance) < 2){
                            // Convert to meters
                            ULTRASONIC_heave = Ultrasonic_distance;
                        }
                        // else...leave previous heave as best estimate

                        data_updated = true;
		}
		DmaChnClrEvFlags(ULTRASONIC_DMA_RX_CHN, DMA_EV_BLOCK_DONE);
    }

    // Clear UART2Rx interrupt flag so DMA transfer can begin again
    INTClearFlag(INT_SOURCE_UART_RX(UART2));
}

#undef ULTRASONIC_H_IMPORT