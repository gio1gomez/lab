#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_memmap.h"
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "driverlib/pwm.h"
#include "driverlib/pin_map.h"
#include "driverlib/uart.h"
#include "utils/uartstdio.c"

#define PWM_FREQ 20000

#define TRIG_PIN GPIO_PIN_2  // PB2
#define ECHO_PIN GPIO_PIN_3  // PB3

void delayMicroseconds(uint32_t us) {
    SysCtlDelay((SysCtlClockGet() / 3 / 1000000) * us);
}

void configurarUART(void) {
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
    GPIOPinConfigure(GPIO_PA0_U0RX);
    GPIOPinConfigure(GPIO_PA1_U0TX);
    GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);
    UARTStdioConfig(0, 9600, 120000000);
}

uint32_t medirDistancia() {
    uint32_t tiempo = 0;

    GPIOPinWrite(GPIO_PORTB_BASE, TRIG_PIN, 0);
    delayMicroseconds(2);
    GPIOPinWrite(GPIO_PORTB_BASE, TRIG_PIN, TRIG_PIN);
    delayMicroseconds(10);
    GPIOPinWrite(GPIO_PORTB_BASE, TRIG_PIN, 0);

    while (GPIOPinRead(GPIO_PORTB_BASE, ECHO_PIN) == 0);
    while (GPIOPinRead(GPIO_PORTB_BASE, ECHO_PIN)) {
        tiempo++;
        delayMicroseconds(1);
    }

    return (tiempo * 0.0343) / 2;  // en cm
}

void Motor_Init(void) {
    SysCtlPWMClockSet(SYSCTL_PWMDIV_64);

    SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOK);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);

    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_PWM0));
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOK));
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOA));

    GPIOPinConfigure(GPIO_PK4_M0PWM6);
    GPIOPinConfigure(GPIO_PK5_M0PWM7);
    GPIOPinTypePWM(GPIO_PORTK_BASE, GPIO_PIN_4 | GPIO_PIN_5);

    GPIOPinTypeGPIOOutput(GPIO_PORTA_BASE, GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_6 | GPIO_PIN_7);

    PWMGenConfigure(PWM0_BASE, PWM_GEN_3, PWM_GEN_MODE_DOWN | PWM_GEN_MODE_NO_SYNC);

    uint32_t pwmClock = SysCtlClockGet() / 64;
    uint32_t load = (pwmClock / PWM_FREQ) - 1;
    PWMGenPeriodSet(PWM0_BASE, PWM_GEN_3, load);

    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_6, 0);
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_7, 0);

    PWMOutputState(PWM0_BASE, PWM_OUT_6_BIT | PWM_OUT_7_BIT, true);
    PWMGenEnable(PWM0_BASE, PWM_GEN_3);
}

void Motor1_Forward(uint32_t duty) {
    GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_6 | GPIO_PIN_7, GPIO_PIN_6); // AIN1 = 1, AIN2 = 0
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_6, duty);
}

void Motor2_Forward(uint32_t duty) {
    GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_2 | GPIO_PIN_3, GPIO_PIN_2); // BIN1 = 1, BIN2 = 0
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_7, duty);
}

void Motor_Stop() {
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_6, 0);
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_7, 0);
    GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_6 | GPIO_PIN_7, 0);
}

int main(void) {
    SysCtlClockFreqSet((SYSCTL_XTAL_25MHZ | SYSCTL_OSC_MAIN |
                        SYSCTL_USE_PLL | SYSCTL_CFG_VCO_480), 120000000);

    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
    GPIOPinTypeGPIOOutput(GPIO_PORTB_BASE, TRIG_PIN);
    GPIOPinTypeGPIOInput(GPIO_PORTB_BASE, ECHO_PIN);

    configurarUART();
    Motor_Init();

    uint32_t pwmClock = SysCtlClockGet() / 64;
    uint32_t load = (pwmClock / PWM_FREQ) - 1;
    uint32_t duty = (load * 30) / 100; // 30%

    while (1) {
        uint32_t distancia = medirDistancia();
        UARTprintf("Distancia: %d cm\n", distancia);

        if (distancia <= 3) {
            Motor_Stop();
        } else {
            Motor1_Forward(duty);
            Motor2_Forward(duty);
        }

        SysCtlDelay(SysCtlClockGet() / 100); // Delay corto (~10ms)
    }
}