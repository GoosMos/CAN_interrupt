/*
 * Copyright (c) 2021, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "ti_msp_dl_config.h"

#define LED_STATUS_ON ((uint8_t) 0x00)
#define LED_STATUS_OFF ((uint8_t) 0x01)

#define LED0_STATUS_ON ((uint8_t) 0x01)
#define LED0_STATUS_OFF ((uint8_t) 0x00)
#define LED1_STATUS_ON ((uint8_t) 0x01)
#define LED1_STATUS_OFF ((uint8_t) 0x00)

#define ID_MODE_STANDARD (0x0U)
#define ID_MODE_EXTENDED (0x1U)


volatile uint32_t gInterruptLine1Status;

volatile bool gTXMsg; // 버튼을 누를 때 인터럽트 핸들러에 의해 상태가 변경
volatile bool gRXMsg; // CAN 메시지가 발생하면 인터럽트 핸들러에 의해 상태가 변경
volatile bool error;


static void processRxMsg(DL_MCAN_RxBufElement *rxMsg);


int main(void)
{
    SYSCFG_DL_init();


    DL_MCAN_TxBufElement txMsg0;
    DL_MCAN_TxBufElement txMsg1;
    DL_MCAN_RxBufElement rxMsg;
    DL_MCAN_RxFIFOStatus rxFS;

    /* Initialize message0 to transmit. */

    txMsg0.id = ((uint32_t)(0x4)) << 18U; /* Identifier Value. */
    txMsg0.rtr = 0U;                      /* Transmit data frame. */
    txMsg0.xtd = 0U;                      /* 11-bit standard identifier. */
    txMsg0.esi = 0U;                      /* ESI bit in CAN FD format depends only on error passive flag. */
    txMsg0.dlc = 1U;                      /* Transmitting 4 bytes. */
    txMsg0.brs = 1U;                      /* CAN FD frames transmitted with bit rate switching. */
    txMsg0.fdf = 1U;                      /* Frame transmitted in CAN FD format. */
    txMsg0.efc = 1U;                      /* Store Tx events. */
    txMsg0.mm = 0xAAU;                    /* Message Marker. */
    txMsg0.data[0] = LED0_STATUS_ON;      /* Data bytes. */

    /* Initialize message1 to transmit. */

    txMsg1.id = ((uint32_t)(0x3)) << 18U; /* Identifier Value. */
    txMsg1.rtr = 0U;                      /* Transmit data frame. */
    txMsg1.xtd = 0U;                      /* 11-bit standard identifier. */
    txMsg1.esi = 0U;                      /* ESI bit in CAN FD format depends only on error passive flag. */
    txMsg1.dlc = 1U;                      /* Transmitting 4 bytes. */
    txMsg1.brs = 1U;                      /* CAN FD frames transmitted with bit rate switching. */
    txMsg1.fdf = 1U;                      /* Frame transmitted in CAN FD format. */
    txMsg1.efc = 1U;                      /* Store Tx events. */
    txMsg1.mm = 0xAAU;                    /* Message Marker. */
    txMsg1.data[0] = LED1_STATUS_ON;      /* Data bytes. */


    gRXMsg                = false;
    gTXMsg                = false;
    gInterruptLine1Status = 0;


    NVIC_EnableIRQ(GPIO_SWITCHES_INT_IRQN); // 스위치 인터럽트가 발생
    NVIC_EnableIRQ(MCAN0_INST_INT_IRQN);

    while (DL_MCAN_OPERATION_MODE_NORMAL != DL_MCAN_getOpMode(MCAN0_INST))
        ;


    while (1) {

        __WFE(); // 인터럽트 발생까지 waiting

        if (gTXMsg == true) // 인터럽트로 인해 메시지가 전송될 수 있는 경우, button interrupt에 의해 false -> true
        {
            gTXMsg = false;

            if (txMsg0.data[0] == LED0_STATUS_ON) {
                txMsg0.data[0] = LED0_STATUS_OFF;
            } else {
                txMsg0.data[0] = LED0_STATUS_ON;
            }

            if (txMsg1.data[0] == LED1_STATUS_ON) {
                txMsg1.data[0] = LED1_STATUS_OFF;
            } else {
                txMsg1.data[0] = LED1_STATUS_ON;
            }

            /* Write Tx Message to the Message RAM. */
            DL_MCAN_writeMsgRam(MCAN0_INST, DL_MCAN_MEM_TYPE_BUF, 0, &txMsg0);

            /* Add request for transmission. */
            DL_MCAN_TXBufAddReq(MCAN0_INST, 0);
        }

        if (gRXMsg == true) // CAN메세지 인터럽트로 인하여 메시지를 읽을 수 있는 경우
        {
            gRXMsg = false;

            rxFS.fillLvl = 0;
            if ((gInterruptLine1Status & MCAN_IR_RF0N_MASK) == MCAN_IR_RF0N_MASK) {
                rxFS.num = DL_MCAN_RX_FIFO_NUM_0;
                while ((rxFS.fillLvl) == 0) {
                    DL_MCAN_getRxFIFOStatus(MCAN0_INST, &rxFS);
                }

                DL_MCAN_readMsgRam(MCAN0_INST, DL_MCAN_MEM_TYPE_FIFO, 0U, rxFS.num, &rxMsg);

                DL_MCAN_writeRxFIFOAck(MCAN0_INST, rxFS.num, rxFS.getIdx);

                processRxMsg(&rxMsg);

                gInterruptLine1Status &= ~(MCAN_IR_RF0N_MASK);
            }
        }
    }
}

// 읽은 메시지를 처리하는 함수
void processRxMsg(DL_MCAN_RxBufElement *rxMsg)
{
    uint32_t idMode;
    uint32_t id;

    idMode = rxMsg->xtd;

    if (ID_MODE_EXTENDED == idMode) {
        id = rxMsg->id;
    } else {
        /* Assuming package is using 11-bit standard ID.
         * When package uses standard id, ID is stored in ID[28:18]*/
        id = ((rxMsg->id & (uint32_t) 0x1FFC0000) >> (uint32_t) 18);
    }
    switch (id) {
        case 0x3:
            if (rxMsg->data[0] == LED_STATUS_ON) {
                DL_GPIO_clearPins(GPIO_LEDS_PORT, GPIO_LEDS_USER_LED_1_PIN);
            } else {
                DL_GPIO_setPins(GPIO_LEDS_PORT, GPIO_LEDS_USER_LED_1_PIN);
            }
            break;
        case 0x4:
            if (rxMsg->data[0] == LED_STATUS_ON) {
                DL_GPIO_clearPins(GPIO_LEDS_PORT, GPIO_LEDS_USER_LED_2_PIN);
            } else {
                DL_GPIO_setPins(GPIO_LEDS_PORT, GPIO_LEDS_USER_LED_2_PIN);
            }
            break;
        default:
            /* Don't do anything */
            break;
    }
}


// CAN메시지가 수신될때 발생하는 인터럽트 핸들러
void MCAN0_INST_IRQHandler(void)
{
    switch (DL_MCAN_getPendingInterrupt(MCAN0_INST)) {
        case DL_MCAN_IIDX_LINE1:
            /* Check MCAN interrupts fired during TX/RX of CAN package */
            gInterruptLine1Status |= DL_MCAN_getIntrStatus(MCAN0_INST); // 인터럽트 상태를 비트연산을 통해 받아온다.
            DL_MCAN_clearIntrStatus(MCAN0_INST, gInterruptLine1Status, DL_MCAN_INTR_SRC_MCAN_LINE_1); // 인터럽스 상태를 클리어한다.

            gRXMsg = true;
            break;
        default:
            break;
    }
}


// button interrupt handler
void GROUP1_IRQHandler(void)
{
    switch (DL_Interrupt_getPendingGroup(DL_INTERRUPT_GROUP_1)) {
        case GPIO_SWITCHES_INT_IIDX:
            switch (DL_GPIO_getPendingInterrupt(GPIO_SWITCHES_PORT)) {
                case DL_GPIO_IIDX_DIO21:
                    gTXMsg = true;
                    break;
                default:
                    break;
            }
            break;
        default:
            break;
    }
}

