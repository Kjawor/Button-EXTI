#include <inttypes.h>
#include <stdbool.h>


struct rcc {
  volatile uint32_t CR, PLLCFGR, CFGR, CIR, AHB1RSTR, AHB2RSTR, AHB3RSTR,
      RESERVED0, APB1RSTR, APB2RSTR, RESERVED1[2], AHB1ENR, AHB2ENR, AHB3ENR,
      RESERVED2, APB1ENR, APB2ENR, RESERVED3[2], AHB1LPENR, AHB2LPENR,
      AHB3LPENR, RESERVED4, APB1LPENR, APB2LPENR, RESERVED5[2], BDCR, CSR,
      RESERVED6[2], SSCGR, PLLI2SCFGR;
  //structure that holds all the RCC registers that are responsible for managing the clock and reset functionalities of peripherals
  //with some of these registers we are able to turn on peripherals that are turned off initially

};



#define RCC ((struct rcc*) 0x40023800)

struct gpio{
	volatile uint32_t MODER,OTYPER,OSPEEDR,PUPDR,IDR,ODR,BSRR,LCKR,AFR[2];
};

#define GPIOD ((struct gpio *) 0x40020C00)
#define GPIOA ((struct gpio *) 0x40020000)

struct exti{
	volatile uint32_t IMR,EMR,RTSR,FTSR,SWIER,PR;
};
//registers for external interrupts


#define EXTI ((struct exti*) 0x40013C00)

struct syscfg{
	volatile uint32_t MEMRMP,PMC,EXTICR[4],RESERVED[2], CMPCR;
};


#define SYSCFG ((struct syscfg*) 0x40013800)

struct nvic{
	volatile uint32_t ISER[8U],RESERVED0[24U],ICER[8U], RESERVED1[24U],  ISPR[8U], RESERVED2[24U],
	ICPR[8U], RESERVED3[24U], IABR[8U], RESERVED4[56U], IP[240U], RESERVED5[644U], STIR;
};
//registers to control interrupts


#define NVIC ((struct nvic*) 0xE000E100)




volatile bool status = false;
//function that will trigger each time an interrupt in EXTI0 occurs.
void EXTI0_IRQHandler(void){
	if(EXTI->PR & (1 << 0)){ //checks to see if interrupt will was triggered by checking the pending bit
		// if the pending bit is set it means a desired edge event(falling or rising) has happened
		EXTI->PR |= (1 << 0); //clears the bit
		status = !status; //change the current status of the LED
	}
}






int main(void){





	RCC->AHB1ENR |= (1U << 0); //enable GPIOA clock for button
	RCC->AHB1ENR |= (1U << 3); //enable GPIOD clock for LED
	RCC->APB2ENR |= (1U <<14); //syscfg clock so we can configure the EXTI line
	SYSCFG->EXTICR[0] = (0U << 0); //map EXTI0 to GPIOA, EXTI0 is responsible for pin 0 so if we assign GPIOA to
	//EXTI0 then GPIOA will be mapped to receive interrupts



	GPIOD->MODER &= ~(1U << 26); //clear mode bits for GPIOD LED
	GPIOD->MODER |= (1U << 26); //set GPIOD LED to output


	GPIOA->MODER &= ~(1U<<0); //set button pin to input
	GPIOA->PUPDR &= ~(3U<<0); //clear the pullup/pulldown register for the button pin
	GPIOA->PUPDR |= (2U<<0); //set the button pin to a pulldown register.





	//each bit in EXTI corresponds to an EXTI line not gpio pin
	//falling edge is button being pressed
	//rising edge is button being released.
	EXTI->IMR |=(1U<<0); //enable interrupt for EXTI line 0
	EXTI->RTSR &= ~(1U<<0); //clear any bits that could have been set on the rising edge interrupt trigger
	EXTI->FTSR |=(1U<<0); //set the bit to trigger an interrupt on falling edge for EXTI line 0



	//set priority for interrupt
	//ip[6] relates to interrupt at index 6 in the vector table
	//3 is the priority we want the interrupt to have
	//shift the wanted priority 4 bits to the most significant bits in the field so the priority can be seen
	// the 4 least significant bits are ignored even if set.
	// the AND at the end is used to check that the 4 least significant bits are not set
	NVIC->IP[6] = (uint8_t)((3 << (8U - 4U)) & (uint32_t) 0xFFUL);




	//enable interrupt in NVIC
	//multiple ISER registers each containing 32 interrupts
	//6 >>5UL divides 6 by 32 to determine which ISER register our interrupt belongs to
	// in our case this will be ISER 0
	// 6 & 0x1FUL calculates the position of where the interrupt is in the register
	// in this case 6 &0x1FUL will equal position 6
	// 1UL will enable the interrupt at the location acquired with the previously discussed statement.

	NVIC->ISER[6 >> 5UL] = (uint32_t)(1UL << (((uint32_t)6)&0x1FUL));









	while(1){
		if(status) {
			GPIOD->ODR |= (1U<<13); //set the output register at the position of the LED which will result in the LED
			//turning on
			//this can also be done using the BSRR register
		}
		else{
			GPIOD->ODR &= ~(1U<<13);
			//clear the bit at the location of the LED
		}

	}
}
