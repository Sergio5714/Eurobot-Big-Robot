#include "stm32f4xx.h"
#include "Board.h"
#include "Communication.h"


uint16_t ahbPresc;
uint8_t apb1Presc;
uint8_t apb2Presc;

int main()
{		
 	boardInitAll();
	ahbPresc = rccGetAhbPrescaler();
	apb1Presc = rccGetApb1Prescaler();
	apb2Presc = rccGetApb1Prescaler();
	while (1)
	{
		getPackage();
		checkCommandAndExecute();
	}
}
