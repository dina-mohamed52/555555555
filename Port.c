# include "port.h"
#include "../tm4c123gh6pm11.h"
#include "TExaS.h"

#define GPIO_PORTF_DATA_R       (*((volatile unsigned long *)0x400253FC))
#define GPIO_PORTF_DIR_R        (*((volatile unsigned long *)0x40025400))
#define GPIO_PORTF_AFSEL_R      (*((volatile unsigned long *)0x40025420))
#define GPIO_PORTF_PUR_R        (*((volatile unsigned long *)0x40025510))
#define GPIO_PORTF_DEN_R        (*((volatile unsigned long *)0x4002551C))
#define GPIO_PORTF_LOCK_R       (*((volatile unsigned long *)0x40025520))
#define GPIO_PORTF_CR_R         (*((volatile unsigned long *)0x40025524))
#define GPIO_PORTF_AMSEL_R      (*((volatile unsigned long *)0x40025528))
#define GPIO_PORTF_PCTL_R       (*((volatile unsigned long *)0x4002552C))

#define NVIC_EN0_R              (*((volatile unsigned long *)0xE000E100))  // IRQ 0 to 31 Set Enable Register
#define NVIC_PRI7_R             (*((volatile unsigned long *)0xE000E41C))  // IRQ 28 to 31 Priority Register
#define SYSCTL_RCGC2_R          (*((volatile unsigned long *)0x400FE108))
#define GPIO_PORTE_DATA_BITS_R  ((volatile unsigned long *)0x40024000)
#define GPIO_PORTE_DATA_R       (*((volatile unsigned long *)0x400243FC))
#define GPIO_PORTE_DIR_R        (*((volatile unsigned long *)0x40024400))
#define GPIO_PORTE_IS_R         (*((volatile unsigned long *)0x40024404))
#define GPIO_PORTE_IBE_R        (*((volatile unsigned long *)0x40024408))
#define GPIO_PORTE_IEV_R        (*((volatile unsigned long *)0x4002440C))
#define GPIO_PORTE_IM_R         (*((volatile unsigned long *)0x40024410))
#define GPIO_PORTE_RIS_R        (*((volatile unsigned long *)0x40024414))
#define GPIO_PORTE_MIS_R        (*((volatile unsigned long *)0x40024418))
#define GPIO_PORTE_ICR_R        (*((volatile unsigned long *)0x4002441C))
#define GPIO_PORTE_AFSEL_R      (*((volatile unsigned long *)0x40024420))
#define GPIO_PORTE_DR2R_R       (*((volatile unsigned long *)0x40024500))
#define GPIO_PORTE_DR4R_R       (*((volatile unsigned long *)0x40024504))
#define GPIO_PORTE_DR8R_R       (*((volatile unsigned long *)0x40024508))
#define GPIO_PORTE_ODR_R        (*((volatile unsigned long *)0x4002450C))
#define GPIO_PORTE_PUR_R        (*((volatile unsigned long *)0x40024510))
#define GPIO_PORTE_PDR_R        (*((volatile unsigned long *)0x40024514))
#define GPIO_PORTE_SLR_R        (*((volatile unsigned long *)0x40024518))
#define GPIO_PORTE_DEN_R        (*((volatile unsigned long *)0x4002451C))
#define GPIO_PORTE_LOCK_R       (*((volatile unsigned long *)0x40024520))
#define GPIO_PORTE_CR_R         (*((volatile unsigned long *)0x40024524))
#define GPIO_PORTE_AMSEL_R      (*((volatile unsigned long *)0x40024528))
#define GPIO_PORTE_PCTL_R       (*((volatile unsigned long *)0x4002452C))
#define GPIO_PORTE_ADCCTL_R     (*((volatile unsigned long *)0x40024530))
#define GPIO_PORTE_DMACTL_R     (*((volatile unsigned long *)0x40024534))

void EnableInterrupts(void);  // Enable interrupts
void PortE_Init(void)
{
    volatile unsigned long delay;
    SYSCTL_RCGC2_R |= 0x00000010;     // 1) E clock
    delay = SYSCTL_RCGC2_R;           // delay
    GPIO_PORTE_LOCK_R = 0x4C4F434B;   // 2) unlock PortF PE0
    GPIO_PORTE_CR_R = 0x1E;           // allow changes to PE4-0
    GPIO_PORTE_AMSEL_R = 0x00;        // 3) disable analog function
    GPIO_PORTE_PCTL_R = 0x00000000;   // 4) GPIO clear bit PCTL
    GPIO_PORTE_DIR_R = 0x02;          // 5) PE1 inpuT
    GPIO_PORTE_AFSEL_R = 0x00;        // 6) no alternate function
    GPIO_PORTE_PUR_R = 0x00;          // enable pullup resistors on PE1,PE0
    GPIO_PORTE_DEN_R = 0xFF;
    GPIO_PORTE_IS_R &=~(1<<0);	// (d) PE0 is edge-sensitive
    GPIO_PORTE_IS_R &=~(1<<2);// (d) PE2 is edge-sensitive
    GPIO_PORTE_IS_R &=~(1<<4);//(d) PE4 is edge-sensitive
    GPIO_PORTE_IBE_R &= ~(1<<0);    //     PE0 is not both edges
    GPIO_PORTE_IBE_R &= ~(1<<2);    //     PE2 is not both edges
    GPIO_PORTE_IBE_R &= ~(1<<4);    //     PE4 is not both edges
    GPIO_PORTE_IEV_R &= ~(1<<0);    //     PE0 falling edge event
    GPIO_PORTE_IEV_R &= ~(1<<2);    //     PE2 falling edge event
    GPIO_PORTE_IEV_R &= ~(1<<4);    //     PE4 falling edge event
    GPIO_PORTE_ICR_R = (1<<0);      // (e) clear flag0
    GPIO_PORTE_ICR_R = (1<<2);      // (e) clear flag2
    GPIO_PORTE_ICR_R = (1<<4);      // (e) clear flag4
    GPIO_PORTE_IM_R |= (1<<0);      // (f) arm interrupt on PF0
    GPIO_PORTE_IM_R |= (1<<2);      // (f) arm interrupt on PF2
    GPIO_PORTE_IM_R |= (1<<4);      // (f) arm interrupt on PF4
    NVIC_PRI7_R = (NVIC_PRI7_R&0xFF00FFFF)|0x00A00000; // (g) priority 5
    NVIC_EN0_R = 0x40000000;      // (h) enable interrupt 30 in NVIC
    EnableInterrupts();           // (i) Clears the I bit
}

void ISPORTE_ISPRESSED(void)
{
    unsigned SW1;
    unsigned SW2;
    unsigned SW3;
    SW1=  GPIO_PORTE_DATA_R & ( 1 << PORTE_SWITCH1_PIN);
    SW2=  GPIO_PORTE_DATA_R & ( 1 << PORTE_SWITCH2_PIN);
    SW3=  GPIO_PORTE_DATA_R & ( 1 << PORTE_SWITCH3_PIN);
}

void Delay1ms(unsigned long count)
{
    unsigned long volatile time;
    while(count>0)
    {
        time = 727240;  // 0.1sec at 80 MHz
        while(time)
        {
            time--;
        }
        count--;
    }
}

void TurnX(void)
{
    GPIO_PORTF_DATA_R |=(1<<1);
    GPIO_PORTF_DATA_R &= ~(1<<2);
}

void TurnO(void)
{
    GPIO_PORTF_DATA_R &= ~(1<<1);
    GPIO_PORTF_DATA_R |=(1<<2);
}
