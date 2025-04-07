/*
 * Este codigo configura y utiliza la comunicacion UART en la EK-TM4C1294XL.
 * Recibe datos a traves de UART0, les agrega el mensaje " desde Tiva", y los reenvia.
 * Adicionalmente, cuando se presiona el boton de usuario 1, envia "motor1" por UART.
 * Cuando se presiona el boton de usuario 2, envia "motor2" por UART.
 */

 #include <stdint.h>
 #include <stdbool.h>
 #include <string.h>
 #include "inc/hw_memmap.h"
 #include "driverlib/debug.h"
 #include "driverlib/gpio.h"
 #include "driverlib/sysctl.h"
 #include "driverlib/pin_map.h"
 #include "driverlib/uart.h"
 
 #include "utils/uartstdio.c"
 
 char data[100]; // Buffer para almacenar los datos recibidos por UART
 
 int main(void)
 {
     // Configurar la frecuencia del sistema a 120 MHz
     SysCtlClockFreqSet((SYSCTL_XTAL_25MHZ | SYSCTL_OSC_MAIN | SYSCTL_USE_PLL | SYSCTL_CFG_VCO_480), 120000000);
     
     // Habilitar los perifericos UART0 y el puerto GPIOA
     SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
     SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
     SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOJ); // Habilitar GPIOJ para los botones de usuario
     
     // Configurar los pines PA0 y PA1 para comunicacion UART0
     GPIOPinConfigure(GPIO_PA0_U0RX);
     GPIOPinConfigure(GPIO_PA1_U0TX);
     GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);
     
     // Configurar botones de usuario en PJ0 y PJ1 como entradas con pull-up
     GPIOPinTypeGPIOInput(GPIO_PORTJ_BASE, GPIO_PIN_0 | GPIO_PIN_1);
     GPIOPadConfigSet(GPIO_PORTJ_BASE, GPIO_PIN_0 | GPIO_PIN_1, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);
     
     // Configurar UART0 con una velocidad de 9600 baudios
     UARTStdioConfig(0, 9600, 120000000);
     
     while (1)
     {
         // Verificar si hay caracteres disponibles en UART0
         if (UARTCharsAvail(UART0_BASE)) {
             UARTgets(data, 100); // Leer los datos recibidos
             strcat(data, " desde Tiva\n"); // Agregar mensaje al final del texto recibido
             UARTprintf(data); // Enviar el mensaje de vuelta por UART
         }
         
         // Verificar si se presiona el boton de usuario 1 (PJ0)
         if (GPIOPinRead(GPIO_PORTJ_BASE, GPIO_PIN_0) == 0) {
             UARTprintf("motor1\n"); // Enviar mensaje "motor1"
             SysCtlDelay(12000000); // Pequeña pausa para evitar rebotes
         }
         
         // Verificar si se presiona el boton de usuario 2 (PJ1)
         if (GPIOPinRead(GPIO_PORTJ_BASE, GPIO_PIN_1) == 0) {
             UARTprintf("motor2\n"); // Enviar mensaje "motor2"
             SysCtlDelay(12000000); // Pequeña pausa para evitar rebotes
         }
     }
 }