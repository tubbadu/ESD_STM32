/*
 *
 * MODER => set the pin mode (input, output, etc...)
 * ODR => set the value for the pin
 *
 */

int main(void)
{
	//PORT REGISTERS
	// MODER => MODE REGISTER
	// ODR => OUTPUT DATA REGISTER
	// IDR => INPUT DATA REGISTER

	// led pin setup
	volatile unsigned int *GPIOA_MODER = (unsigned int*) (0x40020000 + 0x00);	// select the correct location of the register (from datasheet)
	volatile unsigned int *GPIOA_ODR = (unsigned int*) (0x40020000 + 0x14);		// same as above, the offset is 0x14 to get the value (from datasheet)

	// button pin setup
	volatile unsigned int *GPIOC_MODER = (unsigned int*) (0x40020800 + 0x00);
	volatile unsigned int *GPIOC_IDR = (unsigned int*) (0x40020800 + 0x4);

	//CLOCK REGISTERS
	volatile unsigned int *RCC_AHB1ENR = (unsigned int*) (0x40023800 + 0x30);

	//ENABLE PORT CLOCK:
	// this ensure that the peripheral is enabled and connected to the AHB1 bus
	*RCC_AHB1ENR |= 0x05U; // 0x05 = 0b101 to enable port C and A


	//CONFIGURE PORT: set MODER[11:10] = 0x1
	*GPIOA_MODER = *GPIOA_MODER | 0x400;
	*GPIOC_MODER = *GPIOC_MODER | 0x00; // redundant, as the button's default state is input
	//SWITCH ON THE LED: set ODR[5] = 0x1, that is pulls PA5 high
	//*GPIOA_ODR = *GPIOA_ODR | 0x20;
	// Application code (Infinite loop)
	while (1)
	{
		if(*GPIOC_IDR & 0x1000){
			// turn on
			*GPIOA_ODR = *GPIOA_ODR | 0x20;
		} else {
			// turn off
			*GPIOA_ODR = *GPIOA_ODR & ~0x20;
		}
		//*GPIOA_ODR = *GPIOA_ODR ^ 0x20; // XOR with 1, means "toggle" the bit
	}
}
