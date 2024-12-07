#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "WE2_device_addr.h"
#include "WE2_ARMCM55.h"

#define HSC_CLK                 24000000U
#define CM55M_CLK               HSC_CLK
#define SYSTEM_CLOCK		CM55M_CLK

typedef volatile struct dw_uart_reg {
	uint32_t DATA;		/*!< data in/out and DLL */
	uint32_t IER;		/*!< Interrupt enable register and DLH */
	uint32_t IIR;		/*!< Interrupt Id register and FCR */
	uint32_t LCR;		/*!< Line control Register */
	uint32_t MCR;		/*!< Modem control register */
	uint32_t LSR;		/*!< Line Status Register */
	uint32_t MSR;		/*!< Modem status Register */
	uint32_t SCRATCHPAD;	/*!< Uart scratch pad register */
	uint32_t LPDLL;		/*!< Low Power Divisor Latch (Low) Reg */
	uint32_t LPDLH;		/*!< Low Power Divisor Latch (High) Reg */
	uint32_t RES1[2];	/*!< Reserved */
	uint32_t SHR[16];	/*!< Shadow data register(SRBR and STHR) */
	uint32_t FAR;		/*!< FIFO Access register */
	uint32_t TFR;		/*!< Transmit FIFO Read */
	uint32_t RFW;		/*!< Receive FIFO write */
	uint32_t USR;		/*!< UART status register */
	uint32_t TFL;		/*!< Transmit FIFO level */
	uint32_t RFL;		/*!< Receive FIFO level */
	uint32_t SRR;		/*!< Software reset register */
	uint32_t SRTS;		/*!< Shadow request to send */
	uint32_t SBCR;		/*!< Shadow break control */
	uint32_t SDMAM;		/*!< Shadow DMA mode */
	uint32_t SFE;		/*!< Shadow FIFO enable */
	uint32_t SRT;		/*!< Shadow RCVR Trigger */
	uint32_t STET;		/*!< Shadow TX empty register */
	uint32_t HTX;		/*!< Halt TX */
	uint32_t DMASA;		/*!< DMA Software ACK */
	uint32_t RES2[18];	/*!< Reserved */
	uint32_t CPR;		/*!< Camponent parameter register */
	uint32_t UCV;		/*!< UART Component Version */
	uint32_t CTR;		/*!< Component typw register */
} DW_UART_REG, *DW_UART_REG_PTR;

int main(void)
{
	while (1) {
		((DW_UART_REG_PTR)HX_UART0_BASE)->DATA = 'o';
		((DW_UART_REG_PTR)HX_UART0_BASE)->DATA = '\n';
	}

	return 0;
}

void *_sbrk_r() { return NULL; }
void _exit(int status) { while (1); }
int _read_r() { return 0; }
int _write_r() { return 0; }
int _close_r() { return 0; }
int _fstat_r() { return 0; }
int _isatty_() { return 0; }
int _isatty() { return 0; }
int _lseek_r() { return 0; }

extern void EPII_NVIC_SetVector(IRQn_Type IRQn, uint32_t vector)
{
	uint32_t *vectors = (uint32_t *)SCB->VTOR;
	uint32_t addr = (uint32_t)&vectors[(int32_t)IRQn + NVIC_USER_IRQ_OFFSET];

	NVIC_SetVector(IRQn, vector);

	if ((SCB->CCR & SCB_CCR_DC_Msk) != 0U) {
		SCB_CleanDCache_by_Addr((volatile void *)addr, 4);
	}

	if ((SCB->CCR & SCB_CCR_IC_Msk) != 0U) {
		SCB_InvalidateICache_by_Addr((volatile void *)addr, 4);
	}
}

extern uint32_t __INITIAL_SP;
extern uint32_t __STACK_LIMIT;

__NO_RETURN void Reset_Handler(void)
{
	__set_MSP((uint32_t)(&__INITIAL_SP));
	__set_MSPLIM((uint32_t) (&__STACK_LIMIT));

	SystemInit(); /* CMSIS System Initialization */

	SCB_DisableICache();
	SCB_DisableDCache();
	SCB_EnableICache();
	SCB_EnableDCache();

	__PROGRAM_START();
}

void Default_Handler(void)
{
}

void NMI_Handler(void) __attribute__ ((weak, alias("Default_Handler")));
void HardFault_Handler(void) __attribute__ ((weak));
void MemManage_Handler(void) __attribute__ ((weak, alias("Default_Handler")));
void BusFault_Handler(void) __attribute__ ((weak, alias("Default_Handler")));
void UsageFault_Handler(void) __attribute__ ((weak, alias("Default_Handler")));
void SecureFault_Handler(void) __attribute__ ((weak, alias("Default_Handler")));
void SVC_Handler(void) __attribute__ ((weak, alias("Default_Handler")));
void DebugMon_Handler(void) __attribute__ ((weak, alias("Default_Handler")));
void PendSV_Handler(void) __attribute__ ((weak, alias("Default_Handler")));
void SysTick_Handler(void) __attribute__ ((weak, alias("Default_Handler")));
void Interrupt0_Handler(void) __attribute__ ((weak, alias("Default_Handler")));
void Interrupt1_Handler(void) __attribute__ ((weak, alias("Default_Handler")));
void Interrupt2_Handler(void) __attribute__ ((weak, alias("Default_Handler")));
void Interrupt3_Handler(void) __attribute__ ((weak, alias("Default_Handler")));
void Interrupt4_Handler(void) __attribute__ ((weak, alias("Default_Handler")));
void Interrupt5_Handler(void) __attribute__ ((weak, alias("Default_Handler")));
void Interrupt6_Handler(void) __attribute__ ((weak, alias("Default_Handler")));
void Interrupt7_Handler(void) __attribute__ ((weak, alias("Default_Handler")));
void Interrupt8_Handler(void) __attribute__ ((weak, alias("Default_Handler")));
void Interrupt9_Handler(void) __attribute__ ((weak, alias("Default_Handler")));

const VECTOR_TABLE_Type __VECTOR_TABLE[496] __VECTOR_TABLE_ATTRIBUTE = {
	(VECTOR_TABLE_Type)(&__INITIAL_SP), /* Initial Stack Pointer */
	Reset_Handler,		/*  Reset Handler */
	NMI_Handler,		/* -14 NMI Handler */
	HardFault_Handler,	/* -13 Hard Fault Handler */
	MemManage_Handler,	/* -12 MPU Fault Handler */
	BusFault_Handler,	/* -11 Bus Fault Handler */
	UsageFault_Handler,	/* -10 Usage Fault Handler */
	SecureFault_Handler,	/* -9 Secure Fault Handler */
	0,			/* Reserved */
	0,			/* Reserved */
	0,			/* Reservedn */
	SVC_Handler,		/* -5 SVCall Handler */
	DebugMon_Handler,	/* -4 Debug Monitor Handler */
	0,			/* Reserved */
	PendSV_Handler,		/* -2 PendSV Handler */
	SysTick_Handler,	/* -1 SysTick Handler */
	Interrupt0_Handler,	/* 0 Interrupt 0 */
	Interrupt1_Handler,	/* 1 Interrupt 1 */
	Interrupt2_Handler,	/* 2 Interrupt 2 */
	Interrupt3_Handler,	/* 3 Interrupt 3 */
	Interrupt4_Handler,	/* 4 Interrupt 4 */
	Interrupt5_Handler,	/* 5 Interrupt 5 */
	Interrupt6_Handler,	/* 6 Interrupt 6 */
	Interrupt7_Handler,	/* 7 Interrupt 7 */
	Interrupt8_Handler,	/* 8 Interrupt 8 */
	Interrupt9_Handler,	/*  9 Interrupt 9 */
};

uint32_t SystemCoreClock = SYSTEM_CLOCK;

void SystemCoreClockInit(void)
{
	uint32_t val = SysTick_LOAD_RELOAD_Msk+1;
	SystemCoreClock = SYSTEM_CLOCK;

	if (SysTick_Config(val)) {
		while (1) {
			continue;
		}
	}
}

void SystemCoreClockUpdate(uint32_t clock) {
	uint32_t val = SysTick_LOAD_RELOAD_Msk+1;
	SystemCoreClock = clock;

	if (SysTick_Config(val)) {
		while (1) {
			continue;
		}
	}
}

void SystemGetTick(uint32_t *systick, uint32_t *loop_cnt)
{
	*systick = SysTick->VAL;
	*loop_cnt = 0;
}

void SystemInit(void) {
	SCB->VTOR = (uint32_t)(&__VECTOR_TABLE[0]);
	SCB->CPACR |= ((3U << 10U*2U) |           /* enable CP10 Full Access */
		       (3U << 11U*2U));           /* enable CP11 Full Access */
	SCB_NS->CPACR |= ((3U << 10U*2U) |        /* enable CP10 Full Access */
		          (3U << 11U*2U));        /* enable CP11 Full Access */
	SCB->SHCSR |= (1 << SCB_SHCSR_MEMFAULTENA_Pos);
	SCB->SHCSR |= (1 << SCB_SHCSR_BUSFAULTENA_Pos);
	SCB->SHCSR |= (1 << SCB_SHCSR_USGFAULTENA_Pos);
	SCB->SHCSR |= (1 << SCB_SHCSR_SECUREFAULTENA_Pos);

	SCB->CCR |= SCB_CCR_LOB_Msk;
	__ISB();

	for(IRQn_Type idx = 0; idx <=200; idx++) {
		NVIC_DisableIRQ(idx);
	}

	SystemCoreClockInit();
}
