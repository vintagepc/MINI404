/*-
 * CAN module for STM32 SOCs in QEMU.
 *
 * Currently known to be common to the following chips:
 * - STM32C092
 * - STM32H503
 *
 * Copyright (c) 2026 VintagePC <github.com/vinagepc>
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */
#pragma once
#include <stdint.h>
#include "qemu/compiler.h"
#include "../utility/macros.h"
#include "stm32_shared.h"
#include "stm32_fdcan_index.h"


typedef union {
	struct {
		uint32_t DAY           : 8; // /*!<Timestamp Day                           */
		uint32_t MON           : 8; // /*!<Timestamp Month                         */
		uint32_t YEAR          : 4; // /*!<Timestamp Year                          */
		uint32_t SUBSTEP       : 4; // /*!<Sub-step of Core release                */
		uint32_t STEP          : 4; // /*!<Step of Core release                    */
		uint32_t REL           : 4; // /*!<Core release                            */
	} QEMU_PACKED bits;
	uint32_t raw;
}  REGDEF_NAME(com_fdcan,crel);
CHECK_TYPEDEF_u32(REGDEF_NAME(com_fdcan,crel),bits);

typedef union {
	struct {
		uint32_t ETV           :32; // /*!<Endianness Test Value                    */
	} QEMU_PACKED bits;
	uint32_t raw;
}  REGDEF_NAME(com_fdcan,endn);
CHECK_TYPEDEF_u32(REGDEF_NAME(com_fdcan,endn),bits);

typedef union {
	struct {
		uint32_t DSJW          : 4; // /*!<Synchronization Jump Width              */
		uint32_t DTSEG2        : 4; // /*!<Data time segment after sample point    */
		uint32_t DTSEG1        : 5; // /*!<Data time segment before sample point   */
		uint32_t _reserved13   : 3;
		uint32_t DBRP          : 5; // /*!<Data BIt Rate Prescaler                 */
		uint32_t _reserved21   : 2;
		uint32_t TDC           : 1; // /*!<Transceiver Delay Compensation          */
		uint32_t _reserved24   : 8;
	} QEMU_PACKED bits;
	uint32_t raw;
}  REGDEF_NAME(com_fdcan,dbtp);
CHECK_TYPEDEF_u32(REGDEF_NAME(com_fdcan,dbtp),bits);

typedef union {
	struct {
		uint32_t _reserved0    : 4;
		uint32_t LBCK          : 1; // /*!<Loop Back mode                           */
		uint32_t TX            : 2; // /*!<Control of Transmit Pin                  */
		uint32_t RX            : 1; // /*!<Receive Pin                              */
		uint32_t _reserved8    :24;
	} QEMU_PACKED bits;
	uint32_t raw;
}  REGDEF_NAME(com_fdcan,test);
CHECK_TYPEDEF_u32(REGDEF_NAME(com_fdcan,test),bits);

typedef union {
	struct {
		uint32_t WDC           : 8; // /*!<Watchdog configuration                   */
		uint32_t WDV           : 8; // /*!<Watchdog value                           */
		uint32_t _reserved16   :16;
	} QEMU_PACKED bits;
	uint32_t raw;
}  REGDEF_NAME(com_fdcan,rwd);
CHECK_TYPEDEF_u32(REGDEF_NAME(com_fdcan,rwd),bits);

typedef union {
	struct {
		uint32_t INIT          : 1; // /*!<Initialization                           */
		uint32_t CCE           : 1; // /*!<Configuration Change Enable              */
		uint32_t ASM           : 1; // /*!<ASM Restricted Operation Mode            */
		uint32_t CSA           : 1; // /*!<Clock Stop Acknowledge                   */
		uint32_t CSR           : 1; // /*!<Clock Stop Request                       */
		uint32_t MON           : 1; // /*!<Bus Monitoring Mode                      */
		uint32_t DAR           : 1; // /*!<Disable Automatic Retransmission         */
		uint32_t TEST          : 1; // /*!<Test Mode Enable                         */
		uint32_t FDOE          : 1; // /*!<FD Operation Enable                      */
		uint32_t BRSE          : 1; // /*!<FDCAN Bit Rate Switching                 */
		uint32_t _reserved10   : 2;
		uint32_t PXHD          : 1; // /*!<Protocol Exception Handling Disable      */
		uint32_t EFBI          : 1; // /*!<Edge Filtering during Bus Integration    */
		uint32_t TXP           : 1; // /*!<Two CAN bit times Pause                  */
		uint32_t NISO          : 1; // /*!<Non ISO Operation                        */
		uint32_t _reserved16   :16;
	} QEMU_PACKED bits;
	uint32_t raw;
}  REGDEF_NAME(com_fdcan,cccr);
CHECK_TYPEDEF_u32(REGDEF_NAME(com_fdcan,cccr),bits);

typedef union {
	struct {
		uint32_t NTSEG2        : 7; // /*!<Nominal Time segment after sample point  */
		uint32_t _reserved7    : 1;
		uint32_t NTSEG1        : 8; // /*!<Nominal Time segment before sample point */
		uint32_t NBRP          : 9; // /*!<Bit Rate Prescaler                       */
		uint32_t NSJW          : 7; // /*!<Nominal (Re)Synchronization Jump Width   */
	} QEMU_PACKED bits;
	uint32_t raw;
}  REGDEF_NAME(com_fdcan,nbtp);
CHECK_TYPEDEF_u32(REGDEF_NAME(com_fdcan,nbtp),bits);

typedef union {
	struct {
		uint32_t TSS           : 2; // /*!<Timestamp Select                         */
		uint32_t _reserved2    :14;
		uint32_t TCP           : 4; // /*!<Timestamp Counter Prescaler              */
		uint32_t _reserved20   :12;
	} QEMU_PACKED bits;
	uint32_t raw;
}  REGDEF_NAME(com_fdcan,tscc);
CHECK_TYPEDEF_u32(REGDEF_NAME(com_fdcan,tscc),bits);

typedef union {
	struct {
		uint32_t TSC           :16; // /*!<Timestamp Counter                        */
		uint32_t _reserved16   :16;
	} QEMU_PACKED bits;
	uint32_t raw;
}  REGDEF_NAME(com_fdcan,tscv);
CHECK_TYPEDEF_u32(REGDEF_NAME(com_fdcan,tscv),bits);

typedef union {
	struct {
		uint32_t ETOC          : 1; // /*!<Enable Timeout Counter                   */
		uint32_t TOS           : 2; // /*!<Timeout Select                           */
		uint32_t _reserved3    :13;
		uint32_t TOP           :16; // /*!<Timeout Period                           */
	} QEMU_PACKED bits;
	uint32_t raw;
}  REGDEF_NAME(com_fdcan,tocc);
CHECK_TYPEDEF_u32(REGDEF_NAME(com_fdcan,tocc),bits);

typedef union {
	struct {
		uint32_t TOC           :16; // /*!<Timeout Counter                          */
		uint32_t _reserved16   :16;
	} QEMU_PACKED bits;
	uint32_t raw;
}  REGDEF_NAME(com_fdcan,tocv);
CHECK_TYPEDEF_u32(REGDEF_NAME(com_fdcan,tocv),bits);

typedef union {
	struct {
		uint32_t TEC           : 8; // /*!<Transmit Error Counter                   */
		uint32_t REC           : 7; // /*!<Receive Error Counter                    */
		uint32_t RP            : 1; // /*!<Receive Error Passive                    */
		uint32_t CEL           : 8; // /*!<CAN Error Logging                        */
		uint32_t _reserved24   : 8;
	} QEMU_PACKED bits;
	uint32_t raw;
}  REGDEF_NAME(com_fdcan,ecr);
CHECK_TYPEDEF_u32(REGDEF_NAME(com_fdcan,ecr),bits);

typedef union {
	struct {
		uint32_t LEC           : 3; // /*!<Last Error Code                          */
		uint32_t ACT           : 2; // /*!<Activity                                 */
		uint32_t EP            : 1; // /*!<Error Passive                            */
		uint32_t EW            : 1; // /*!<Warning Status                           */
		uint32_t BO            : 1; // /*!<Bus_Off Status                           */
		uint32_t DLEC          : 3; // /*!<Data Last Error Code                     */
		uint32_t RESI          : 1; // /*!<ESI flag of last received FDCAN Message  */
		uint32_t RBRS          : 1; // /*!<BRS flag of last received FDCAN Message  */
		uint32_t REDL          : 1; // /*!<Received FDCAN Message                   */
		uint32_t PXE           : 1; // /*!<Protocol Exception Event                 */
		uint32_t _reserved15   : 1;
		uint32_t TDCV          : 7; // /*!<Transmitter Delay Compensation Value     */
		uint32_t _reserved23   : 9;
	} QEMU_PACKED bits;
	uint32_t raw;
}  REGDEF_NAME(com_fdcan,psr);
CHECK_TYPEDEF_u32(REGDEF_NAME(com_fdcan,psr),bits);

typedef union {
	struct {
		uint32_t TDCF          : 7; // /*!<Transmitter Delay Compensation Filter    */
		uint32_t _reserved7    : 1;
		uint32_t TDCO          : 7; // /*!<Transmitter Delay Compensation Offset    */
		uint32_t _reserved15   :17;
	} QEMU_PACKED bits;
	uint32_t raw;
}  REGDEF_NAME(com_fdcan,tdcr);
CHECK_TYPEDEF_u32(REGDEF_NAME(com_fdcan,tdcr),bits);

typedef union {
	struct {
		uint32_t RF0N          : 1; // /*!<Rx FIFO 0 New Message                    */
		uint32_t RF0F          : 1; // /*!<Rx FIFO 0 Full                           */
		uint32_t RF0L          : 1; // /*!<Rx FIFO 0 Message Lost                   */
		uint32_t RF1N          : 1; // /*!<Rx FIFO 1 New Message                    */
		uint32_t RF1F          : 1; // /*!<Rx FIFO 1 Full                           */
		uint32_t RF1L          : 1; // /*!<Rx FIFO 1 Message Lost                   */
		uint32_t HPM           : 1; // /*!<High Priority Message                    */
		uint32_t TC            : 1; // /*!<Transmission Completed                   */
		uint32_t TCF           : 1; // /*!<Transmission Cancellation Finished       */
		uint32_t TFE           : 1; // /*!<Tx FIFO Empty                            */
		uint32_t TEFN          : 1; // /*!<Tx Event FIFO New Entry                  */
		uint32_t TEFF          : 1; // /*!<Tx Event FIFO Full                       */
		uint32_t TEFL          : 1; // /*!<Tx Event FIFO Element Lost               */
		uint32_t TSW           : 1; // /*!<Timestamp Wraparound                     */
		uint32_t MRAF          : 1; // /*!<Message RAM Access Failure               */
		uint32_t TOO           : 1; // /*!<Timeout Occurred                         */
		uint32_t ELO           : 1; // /*!<Error Logging Overflow                   */
		uint32_t EP            : 1; // /*!<Error Passive                            */
		uint32_t EW            : 1; // /*!<Warning Status                           */
		uint32_t BO            : 1; // /*!<Bus_Off Status                           */
		uint32_t WDI           : 1; // /*!<Watchdog Interrupt                       */
		uint32_t PEA           : 1; // /*!<Protocol Error in Arbitration Phase      */
		uint32_t PED           : 1; // /*!<Protocol Error in Data Phase             */
		uint32_t ARA           : 1; // /*!<Access to Reserved Address               */
		uint32_t _reserved24   : 8;
	} QEMU_PACKED bits;
	uint32_t raw;
}  REGDEF_NAME(com_fdcan,ir);
CHECK_TYPEDEF_u32(REGDEF_NAME(com_fdcan,ir),bits);

typedef union {
	struct {
		uint32_t RF0NE         : 1; // /*!<Rx FIFO 0 New Message Enable             */
		uint32_t RF0FE         : 1; // /*!<Rx FIFO 0 Full Enable                    */
		uint32_t RF0LE         : 1; // /*!<Rx FIFO 0 Message Lost Enable            */
		uint32_t RF1NE         : 1; // /*!<Rx FIFO 1 New Message Enable             */
		uint32_t RF1FE         : 1; // /*!<Rx FIFO 1 Full Enable                    */
		uint32_t RF1LE         : 1; // /*!<Rx FIFO 1 Message Lost Enable            */
		uint32_t HPME          : 1; // /*!<High Priority Message Enable             */
		uint32_t TCE           : 1; // /*!<Transmission Completed Enable            */
		uint32_t TCFE          : 1; // /*!<Transmission Cancellation Finished Enable*/
		uint32_t TFEE          : 1; // /*!<Tx FIFO Empty Enable                     */
		uint32_t TEFNE         : 1; // /*!<Tx Event FIFO New Entry Enable           */
		uint32_t TEFFE         : 1; // /*!<Tx Event FIFO Full Enable                */
		uint32_t TEFLE         : 1; // /*!<Tx Event FIFO Element Lost Enable        */
		uint32_t TSWE          : 1; // /*!<Timestamp Wraparound Enable              */
		uint32_t MRAFE         : 1; // /*!<Message RAM Access Failure Enable        */
		uint32_t TOOE          : 1; // /*!<Timeout Occurred Enable                  */
		uint32_t ELOE          : 1; // /*!<Error Logging Overflow Enable            */
		uint32_t EPE           : 1; // /*!<Error Passive Enable                     */
		uint32_t EWE           : 1; // /*!<Warning Status Enable                    */
		uint32_t BOE           : 1; // /*!<Bus_Off Status Enable                    */
		uint32_t WDIE          : 1; // /*!<Watchdog Interrupt Enable                */
		uint32_t PEAE          : 1; // /*!<Protocol Error in Arbitration Phase Enable*/
		uint32_t PEDE          : 1; // /*!<Protocol Error in Data Phase Enable      */
		uint32_t ARAE          : 1; // /*!<Access to Reserved Address Enable        */
		uint32_t _reserved24   : 8;
	} QEMU_PACKED bits;
	uint32_t raw;
}  REGDEF_NAME(com_fdcan,ie);
CHECK_TYPEDEF_u32(REGDEF_NAME(com_fdcan,ie),bits);

typedef union {
	struct {
		uint32_t RXFIFO0       : 1;
		uint32_t RXFIFO1       : 1;
		uint32_t SMSG          : 1;
		uint32_t TFERR         : 1;
		uint32_t MISC          : 1;
		uint32_t BERR          : 1;
		uint32_t PERR          : 1;
		uint32_t _reserved7    :25;
	} QEMU_PACKED bits;
	uint32_t raw;
}  REGDEF_NAME(com_fdcan,ils);
CHECK_TYPEDEF_u32(REGDEF_NAME(com_fdcan,ils),bits);

typedef union {
	struct {
		uint32_t EINT0         : 1; // /*!<Enable Interrupt Line 0                  */
		uint32_t EINT1         : 1; // /*!<Enable Interrupt Line 1                  */
		uint32_t _reserved2    :30;
	} QEMU_PACKED bits;
	uint32_t raw;
}  REGDEF_NAME(com_fdcan,ile);
CHECK_TYPEDEF_u32(REGDEF_NAME(com_fdcan,ile),bits);

typedef union {
	struct {
		uint32_t RRFE          : 1; // /*!<Reject Remote Frames Extended            */
		uint32_t RRFS          : 1; // /*!<Reject Remote Frames Standard            */
		uint32_t ANFE          : 2; // /*!<Accept Non-matching Frames Extended      */
		uint32_t ANFS          : 2; // /*!<Accept Non-matching Frames Standard      */
		uint32_t _reserved6    : 2;
		uint32_t F1OM          : 1; // /*!<FIFO 1 operation mode                    */
		uint32_t F0OM          : 1; // /*!<FIFO 0 operation mode                    */
		uint32_t _reserved10   : 6;
		uint32_t LSS           : 5; // /*!<List Size Standard                       */
		uint32_t _reserved21   : 3;
		uint32_t LSE           : 4; // /*!<List Size Extended                       */
		uint32_t _reserved28   : 4;
	} QEMU_PACKED bits;
	uint32_t raw;
}  REGDEF_NAME(com_fdcan,rxgfc);
CHECK_TYPEDEF_u32(REGDEF_NAME(com_fdcan,rxgfc),bits);

typedef union {
	struct {
		uint32_t EIDM          :29; // /*!<Extended ID Mask                         */
		uint32_t _reserved29   : 3;
	} QEMU_PACKED bits;
	uint32_t raw;
}  REGDEF_NAME(com_fdcan,xidam);
CHECK_TYPEDEF_u32(REGDEF_NAME(com_fdcan,xidam),bits);

typedef union {
	struct {
		uint32_t BIDX          : 3; // /*!<Buffer Index                             */
		uint32_t _reserved3    : 3;
		uint32_t MSI           : 2; // /*!<Message Storage Indicator                */
		uint32_t FIDX          : 5; // /*!<Filter Index                             */
		uint32_t _reserved13   : 2;
		uint32_t FLST          : 1; // /*!<Filter List                              */
		uint32_t _reserved16   :16;
	} QEMU_PACKED bits;
	uint32_t raw;
}  REGDEF_NAME(com_fdcan,hpms);
CHECK_TYPEDEF_u32(REGDEF_NAME(com_fdcan,hpms),bits);

typedef union {
	struct {
		uint32_t F0FL          : 4; // /*!<Rx FIFO 0 Fill Level                     */
		uint32_t _reserved4    : 4;
		uint32_t F0GI          : 2; // /*!<Rx FIFO 0 Get Index                      */
		uint32_t _reserved10   : 6;
		uint32_t F0PI          : 2; // /*!<Rx FIFO 0 Put Index                      */
		uint32_t _reserved18   : 6;
		uint32_t F0F           : 1; // /*!<Rx FIFO 0 Full                           */
		uint32_t RF0L          : 1; // /*!<Rx FIFO 0 Message Lost                   */
		uint32_t _reserved26   : 6;
	} QEMU_PACKED bits;
	uint32_t raw;
}  REGDEF_NAME(com_fdcan,rxf0s);
CHECK_TYPEDEF_u32(REGDEF_NAME(com_fdcan,rxf0s),bits);

typedef union {
	struct {
		uint32_t F0AI          : 3; // /*!<Rx FIFO 0 Acknowledge Index              */
		uint32_t _reserved3    :29;
	} QEMU_PACKED bits;
	uint32_t raw;
}  REGDEF_NAME(com_fdcan,rxf0a);
CHECK_TYPEDEF_u32(REGDEF_NAME(com_fdcan,rxf0a),bits);

typedef union {
	struct {
		uint32_t F1FL          : 4; // /*!<Rx FIFO 1 Fill Level                     */
		uint32_t _reserved4    : 4;
		uint32_t F1GI          : 2; // /*!<Rx FIFO 1 Get Index                      */
		uint32_t _reserved10   : 6;
		uint32_t F1PI          : 2; // /*!<Rx FIFO 1 Put Index                      */
		uint32_t _reserved18   : 6;
		uint32_t F1F           : 1; // /*!<Rx FIFO 1 Full                           */
		uint32_t RF1L          : 1; // /*!<Rx FIFO 1 Message Lost                   */
		uint32_t _reserved26   : 6;
	} QEMU_PACKED bits;
	uint32_t raw;
}  REGDEF_NAME(com_fdcan,rxf1s);
CHECK_TYPEDEF_u32(REGDEF_NAME(com_fdcan,rxf1s),bits);

typedef union {
	struct {
		uint32_t F1AI          : 3; // /*!<Rx FIFO 1 Acknowledge Index              */
		uint32_t _reserved3    :29;
	} QEMU_PACKED bits;
	uint32_t raw;
}  REGDEF_NAME(com_fdcan,rxf1a);
CHECK_TYPEDEF_u32(REGDEF_NAME(com_fdcan,rxf1a),bits);

typedef union {
	struct {
		uint32_t _reserved0    :24;
		uint32_t TFQM          : 1; // /*!<Tx FIFO/Queue Mode                       */
		uint32_t _reserved25   : 7;
	} QEMU_PACKED bits;
	uint32_t raw;
}  REGDEF_NAME(com_fdcan,txbc);
CHECK_TYPEDEF_u32(REGDEF_NAME(com_fdcan,txbc),bits);

typedef union {
	struct {
		uint32_t TFFL          : 3; // /*!<Tx FIFO Free Level                       */
		uint32_t _reserved3    : 5;
		uint32_t TFGI          : 2; // /*!<Tx FIFO Get Index                        */
		uint32_t _reserved10   : 6;
		uint32_t TFQPI         : 2; // /*!<Tx FIFO/Queue Put Index                  */
		uint32_t _reserved18   : 3;
		uint32_t TFQF          : 1; // /*!<Tx FIFO/Queue Full                       */
		uint32_t _reserved22   :10;
	} QEMU_PACKED bits;
	uint32_t raw;
}  REGDEF_NAME(com_fdcan,txfqs);
CHECK_TYPEDEF_u32(REGDEF_NAME(com_fdcan,txfqs),bits);

typedef union {
	struct {
		uint32_t TRP           : 3; // /*!<Transmission Request Pending             */
		uint32_t _reserved3    :29;
	} QEMU_PACKED bits;
	uint32_t raw;
}  REGDEF_NAME(com_fdcan,txbrp);
CHECK_TYPEDEF_u32(REGDEF_NAME(com_fdcan,txbrp),bits);

typedef union {
	struct {
		uint32_t AR            : 3; // /*!<Add Request                              */
		uint32_t _reserved3    :29;
	} QEMU_PACKED bits;
	uint32_t raw;
}  REGDEF_NAME(com_fdcan,txbar);
CHECK_TYPEDEF_u32(REGDEF_NAME(com_fdcan,txbar),bits);

typedef union {
	struct {
		uint32_t CR            : 3; // /*!<Cancellation Request                     */
		uint32_t _reserved3    :29;
	} QEMU_PACKED bits;
	uint32_t raw;
}  REGDEF_NAME(com_fdcan,txbcr);
CHECK_TYPEDEF_u32(REGDEF_NAME(com_fdcan,txbcr),bits);

typedef union {
	struct {
		uint32_t TO            : 3; // /*!<Transmission Occurred                    */
		uint32_t _reserved3    :29;
	} QEMU_PACKED bits;
	uint32_t raw;
}  REGDEF_NAME(com_fdcan,txbto);
CHECK_TYPEDEF_u32(REGDEF_NAME(com_fdcan,txbto),bits);

typedef union {
	struct {
		uint32_t CF            : 3; // /*!<Cancellation Finished                    */
		uint32_t _reserved3    :29;
	} QEMU_PACKED bits;
	uint32_t raw;
}  REGDEF_NAME(com_fdcan,txbcf);
CHECK_TYPEDEF_u32(REGDEF_NAME(com_fdcan,txbcf),bits);

typedef union {
	struct {
		uint32_t TIE           : 3; // /*!<Transmission Interrupt Enable            */
		uint32_t _reserved3    :29;
	} QEMU_PACKED bits;
	uint32_t raw;
}  REGDEF_NAME(com_fdcan,txbtie);
CHECK_TYPEDEF_u32(REGDEF_NAME(com_fdcan,txbtie),bits);

typedef union {
	struct {
		uint32_t CFIE          : 3; // /*!<Cancellation Finished Interrupt Enable   */
		uint32_t _reserved3    :29;
	} QEMU_PACKED bits;
	uint32_t raw;
}  REGDEF_NAME(com_fdcan,txbcie);
CHECK_TYPEDEF_u32(REGDEF_NAME(com_fdcan,txbcie),bits);

typedef union {
	struct {
		uint32_t EFFL          : 3; // /*!<Event FIFO Fill Level                    */
		uint32_t _reserved3    : 5;
		uint32_t EFGI          : 2; // /*!<Event FIFO Get Index                     */
		uint32_t _reserved10   : 6;
		uint32_t EFPI          : 2; // /*!<Event FIFO Put Index                     */
		uint32_t _reserved18   : 6;
		uint32_t EFF           : 1; // /*!<Event FIFO Full                          */
		uint32_t TEFL          : 1; // /*!<Tx Event FIFO Element Lost               */
		uint32_t _reserved26   : 6;
	} QEMU_PACKED bits;
	uint32_t raw;
}  REGDEF_NAME(com_fdcan,txefs);
CHECK_TYPEDEF_u32(REGDEF_NAME(com_fdcan,txefs),bits);

typedef union {
	struct {
		uint32_t EFAI          : 2; // /*!<Event FIFO Acknowledge Index             */
		uint32_t _reserved2    :30;
	} QEMU_PACKED bits;
	uint32_t raw;
}  REGDEF_NAME(com_fdcan,txefa);
CHECK_TYPEDEF_u32(REGDEF_NAME(com_fdcan,txefa),bits);

typedef union {
	struct {
		uint32_t PDIV          : 4; // /*!<Input Clock Divider                      */
		uint32_t _reserved4    :28;
	} QEMU_PACKED bits;
	uint32_t raw;
}  REGDEF_NAME(com_fdcan,ckdiv);
CHECK_TYPEDEF_u32(REGDEF_NAME(com_fdcan,ckdiv),bits);

typedef union {
	struct {
		REGDEF_NAME(com_fdcan,crel) CREL;
		REGDEF_NAME(com_fdcan,endn) ENDN;
		uint32_t _reserved0x8;
		REGDEF_NAME(com_fdcan,dbtp) DBTP;
		REGDEF_NAME(com_fdcan,test) TEST;
		REGDEF_NAME(com_fdcan,rwd) RWD;
		REGDEF_NAME(com_fdcan,cccr) CCCR;
		REGDEF_NAME(com_fdcan,nbtp) NBTP;
		REGDEF_NAME(com_fdcan,tscc) TSCC;
		REGDEF_NAME(com_fdcan,tscv) TSCV;
		REGDEF_NAME(com_fdcan,tocc) TOCC;
		REGDEF_NAME(com_fdcan,tocv) TOCV;
		uint32_t _reserved0x30[ 4];
		REGDEF_NAME(com_fdcan,ecr) ECR;
		REGDEF_NAME(com_fdcan,psr) PSR;
		REGDEF_NAME(com_fdcan,tdcr) TDCR;
		uint32_t _reserved0x4C;
		REGDEF_NAME(com_fdcan,ir) IR;
		REGDEF_NAME(com_fdcan,ie) IE;
		REGDEF_NAME(com_fdcan,ils) ILS;
		REGDEF_NAME(com_fdcan,ile) ILE;
		uint32_t _reserved0x60[ 8];
		REGDEF_NAME(com_fdcan,rxgfc) RXGFC;
		REGDEF_NAME(com_fdcan,xidam) XIDAM;
		REGDEF_NAME(com_fdcan,hpms) HPMS;
		uint32_t _reserved0x8C;
		REGDEF_NAME(com_fdcan,rxf0s) RXF0S;
		REGDEF_NAME(com_fdcan,rxf0a) RXF0A;
		REGDEF_NAME(com_fdcan,rxf1s) RXF1S;
		REGDEF_NAME(com_fdcan,rxf1a) RXF1A;
		uint32_t _reserved0xA0[ 8];
		REGDEF_NAME(com_fdcan,txbc) TXBC;
		REGDEF_NAME(com_fdcan,txfqs) TXFQS;
		REGDEF_NAME(com_fdcan,txbrp) TXBRP;
		REGDEF_NAME(com_fdcan,txbar) TXBAR;
		REGDEF_NAME(com_fdcan,txbcr) TXBCR;
		REGDEF_NAME(com_fdcan,txbto) TXBTO;
		REGDEF_NAME(com_fdcan,txbcf) TXBCF;
		REGDEF_NAME(com_fdcan,txbtie) TXBTIE;
		REGDEF_NAME(com_fdcan,txbcie) TXBCIE;
		REGDEF_NAME(com_fdcan,txefs) TXEFS;
		REGDEF_NAME(com_fdcan,txefa) TXEFA;
		uint32_t _reserved0xEC[ 5];
		REGDEF_NAME(com_fdcan,ckdiv) CKDIV;
	} /*QEMU_PACKED*/;
	uint32_t raw[RI_END];
}  REGDEF_NAME(stm32com,fdcan);

QEMU_BUILD_BUG_MSG(sizeof(REGDEF_NAME(stm32com,fdcan)) != sizeof(uint32_t)*RI_END , "Structure Size mismatch - expected uint32[RI_END]");
QEMU_BUILD_BUG_MSG(offsetof(REGDEF_NAME(stm32com,fdcan), CREL) != 0, "Offset mismatch for CREL");
QEMU_BUILD_BUG_MSG(offsetof(REGDEF_NAME(stm32com,fdcan), ENDN) != 4, "Offset mismatch for ENDN");
QEMU_BUILD_BUG_MSG(offsetof(REGDEF_NAME(stm32com,fdcan), DBTP) != 12, "Offset mismatch for DBTP");
QEMU_BUILD_BUG_MSG(offsetof(REGDEF_NAME(stm32com,fdcan), TEST) != 16, "Offset mismatch for TEST");
QEMU_BUILD_BUG_MSG(offsetof(REGDEF_NAME(stm32com,fdcan), RWD) != 20, "Offset mismatch for RWD");
QEMU_BUILD_BUG_MSG(offsetof(REGDEF_NAME(stm32com,fdcan), CCCR) != 24, "Offset mismatch for CCCR");
QEMU_BUILD_BUG_MSG(offsetof(REGDEF_NAME(stm32com,fdcan), NBTP) != 28, "Offset mismatch for NBTP");
QEMU_BUILD_BUG_MSG(offsetof(REGDEF_NAME(stm32com,fdcan), TSCC) != 32, "Offset mismatch for TSCC");
QEMU_BUILD_BUG_MSG(offsetof(REGDEF_NAME(stm32com,fdcan), TSCV) != 36, "Offset mismatch for TSCV");
QEMU_BUILD_BUG_MSG(offsetof(REGDEF_NAME(stm32com,fdcan), TOCC) != 40, "Offset mismatch for TOCC");
QEMU_BUILD_BUG_MSG(offsetof(REGDEF_NAME(stm32com,fdcan), TOCV) != 44, "Offset mismatch for TOCV");
QEMU_BUILD_BUG_MSG(offsetof(REGDEF_NAME(stm32com,fdcan), ECR) != 64, "Offset mismatch for ECR");
QEMU_BUILD_BUG_MSG(offsetof(REGDEF_NAME(stm32com,fdcan), PSR) != 68, "Offset mismatch for PSR");
QEMU_BUILD_BUG_MSG(offsetof(REGDEF_NAME(stm32com,fdcan), TDCR) != 72, "Offset mismatch for TDCR");
QEMU_BUILD_BUG_MSG(offsetof(REGDEF_NAME(stm32com,fdcan), IR) != 80, "Offset mismatch for IR");
QEMU_BUILD_BUG_MSG(offsetof(REGDEF_NAME(stm32com,fdcan), IE) != 84, "Offset mismatch for IE");
QEMU_BUILD_BUG_MSG(offsetof(REGDEF_NAME(stm32com,fdcan), ILS) != 88, "Offset mismatch for ILS");
QEMU_BUILD_BUG_MSG(offsetof(REGDEF_NAME(stm32com,fdcan), ILE) != 92, "Offset mismatch for ILE");
QEMU_BUILD_BUG_MSG(offsetof(REGDEF_NAME(stm32com,fdcan), RXGFC) != 128, "Offset mismatch for RXGFC");
QEMU_BUILD_BUG_MSG(offsetof(REGDEF_NAME(stm32com,fdcan), XIDAM) != 132, "Offset mismatch for XIDAM");
QEMU_BUILD_BUG_MSG(offsetof(REGDEF_NAME(stm32com,fdcan), HPMS) != 136, "Offset mismatch for HPMS");
QEMU_BUILD_BUG_MSG(offsetof(REGDEF_NAME(stm32com,fdcan), RXF0S) != 144, "Offset mismatch for RXF0S");
QEMU_BUILD_BUG_MSG(offsetof(REGDEF_NAME(stm32com,fdcan), RXF0A) != 148, "Offset mismatch for RXF0A");
QEMU_BUILD_BUG_MSG(offsetof(REGDEF_NAME(stm32com,fdcan), RXF1S) != 152, "Offset mismatch for RXF1S");
QEMU_BUILD_BUG_MSG(offsetof(REGDEF_NAME(stm32com,fdcan), RXF1A) != 156, "Offset mismatch for RXF1A");
QEMU_BUILD_BUG_MSG(offsetof(REGDEF_NAME(stm32com,fdcan), TXBC) != 192, "Offset mismatch for TXBC");
QEMU_BUILD_BUG_MSG(offsetof(REGDEF_NAME(stm32com,fdcan), TXFQS) != 196, "Offset mismatch for TXFQS");
QEMU_BUILD_BUG_MSG(offsetof(REGDEF_NAME(stm32com,fdcan), TXBRP) != 200, "Offset mismatch for TXBRP");
QEMU_BUILD_BUG_MSG(offsetof(REGDEF_NAME(stm32com,fdcan), TXBAR) != 204, "Offset mismatch for TXBAR");
QEMU_BUILD_BUG_MSG(offsetof(REGDEF_NAME(stm32com,fdcan), TXBCR) != 208, "Offset mismatch for TXBCR");
QEMU_BUILD_BUG_MSG(offsetof(REGDEF_NAME(stm32com,fdcan), TXBTO) != 212, "Offset mismatch for TXBTO");
QEMU_BUILD_BUG_MSG(offsetof(REGDEF_NAME(stm32com,fdcan), TXBCF) != 216, "Offset mismatch for TXBCF");
QEMU_BUILD_BUG_MSG(offsetof(REGDEF_NAME(stm32com,fdcan), TXBTIE) != 220, "Offset mismatch for TXBTIE");
QEMU_BUILD_BUG_MSG(offsetof(REGDEF_NAME(stm32com,fdcan), TXBCIE) != 224, "Offset mismatch for TXBCIE");
QEMU_BUILD_BUG_MSG(offsetof(REGDEF_NAME(stm32com,fdcan), TXEFS) != 228, "Offset mismatch for TXEFS");
QEMU_BUILD_BUG_MSG(offsetof(REGDEF_NAME(stm32com,fdcan), TXEFA) != 232, "Offset mismatch for TXEFA");
QEMU_BUILD_BUG_MSG(offsetof(REGDEF_NAME(stm32com,fdcan), CKDIV) != 256, "Offset mismatch for CKDIV");
