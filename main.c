/**
 * main.c
 *
 * ECE 3849 Lab 0 Starter Project
 * Gene Bogdanov    10/18/2017
 *
 * This version is using the new hardware for B2017: the EK-TM4C1294XL LaunchPad with BOOSTXL-EDUMKII BoosterPack.
 *
 */

#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <math.h>
#include "driverlib/fpu.h"
#include "driverlib/sysctl.h"
#include "driverlib/interrupt.h"
#include "driverlib/gpio.h"
#include "driverlib/timer.h"
#include "driverlib/adc.h"
#include "inc/tm4c1294ncpdt.h"
#include "Crystalfontz128x128_ST7735.h"
#include "buttons.h"
#include "oscilloscope.h"
#include "driverlib/timer.h"
#include "inc/hw_memmap.h"
#include "inc/hw_ints.h"
#include "sysctl_pll.h"

//......DEFINES......//
#define ADC_OFFSET 2048
#define ADC_BUFFER_SIZE 2048 // size must be a power of 2
#define ADC_BUFFER_WRAP(i) ((i) & (ADC_BUFFER_SIZE - 1)) // index wrapping macro
#define ADC_BITS 12
#define VIN_RANGE 3.3
#define PIXELS_PER_DIV 20
#define ADC_BITS 12
#define ADC_OFFSET 2048

//......GLOBALS......//
volatile uint32_t gADCBufferIndex = ADC_BUFFER_SIZE - 1;  // Latest sample index
volatile uint16_t gADCBuffer[ADC_BUFFER_SIZE];           // Circular buffer
volatile uint32_t gADCErrors;
volatile uint32_t ADCIndex;
volatile uint16_t triggerDirection = 0;
volatile uint32_t triggerVoltage = 2048;
volatile uint16_t samplingRateState = 11;
volatile float VOLTS_PER_DIV = 0.5;
uint16_t ADCDraw[128];
uint32_t gSystemClock; // [Hz] system clock frequency
volatile uint32_t gTime = 8345; // time in hundredths of a second
volatile uint16_t voltageScaleState = 2;
char string[30];
tContext sContext;
uint32_t count_unloaded = 0;
uint32_t count_loaded = 0;
float cpu_load = 0.0;

//......FUNCTION PROTOTYPES......//
void ADCInit(void);
void GetWaveform(int Direction, uint16_t Voltage);
void initMain();
void drawGrid(tContext sContext);
uint32_t cpu_load_count(void);

//......MAIN......//
int main(void)
{
    initMain(); //
    GrContextInit(&sContext, &g_sCrystalfontz128x128); // Initialize the grlib graphics context
    GrContextFontSet(&sContext, &g_sFontCmss12);     // Select font
    tRectangle rectFullScreen = { 0, 0, GrContextDpyWidthGet(&sContext) - 1,GrContextDpyHeightGet(&sContext) - 1 };

    while (true)
    {
        // Local variables to prevent shared data issues
        uint16_t voltageScaleStateLocal = voltageScaleState;
        uint16_t triggerDirectionLocal = triggerDirection;
        uint16_t samplingRateStateLocal = samplingRateState;
        float VOLTS_PER_DIV_LOCAL = VOLTS_PER_DIV;

        //Scaling factor
        float fScale = (VIN_RANGE * PIXELS_PER_DIV)
                / ((1 << ADC_BITS) * VOLTS_PER_DIV_LOCAL);

        // Fill screen with black
        GrContextForegroundSet(&sContext, ClrBlack);
        GrRectFill(&sContext, &rectFullScreen);

        // Compute CPU load
        count_loaded = cpu_load_count();
        cpu_load = 1.0f - (float) count_loaded / count_unloaded;
        snprintf(string, sizeof(string), "CPU LOAD: %.02f %%", cpu_load * 100); // Convert CPU load to string for printing

        // Draw all elements on the LCD screen
        drawScreen(sContext, triggerDirectionLocal, voltageScaleStateLocal, samplingRateStateLocal,string);

        // Draw waveform
        GrContextForegroundSet(&sContext, ClrYellow); // Yellow waveform
        getWaveform(triggerDirectionLocal, triggerVoltage); // Puts 128 samples into global frame buffer
        int i;
        for (i = 0; i < 127; i++)
        {
            int sample = ADCDraw[i];
            int nextSample = ADCDraw[i + 1];
            int currentY = LCD_VERTICAL_MAX / 2
                    - (int) roundf(fScale * (sample - ADC_OFFSET));
            int nextY = LCD_VERTICAL_MAX / 2
                    - (int) roundf(fScale * (nextSample - ADC_OFFSET));
            GrLineDraw(&sContext, i, currentY, i + 1, nextY);
        }
        GrFlush(&sContext); // Update the LCD display by flushing it
    }
}

// Initialize ADC
void ADCInit(void)
{

    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
    GPIOPinTypeADC(GPIO_PORTE_BASE, GPIO_PIN_0); // GPIO setup for analog input AIN3

    SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0); // initialize ADC peripherals
    SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC1);

    // ADC clock
    uint32_t pll_frequency = SysCtlFrequencyGet(CRYSTAL_FREQUENCY);
    uint32_t pll_divisor = (pll_frequency - 1) / (16 * ADC_SAMPLING_RATE) + 1; //round up

    ADCClockConfigSet(ADC0_BASE, ADC_CLOCK_SRC_PLL | ADC_CLOCK_RATE_FULL,
                      pll_divisor);
    ADCClockConfigSet(ADC1_BASE, ADC_CLOCK_SRC_PLL | ADC_CLOCK_RATE_FULL,
                      pll_divisor);
    ADCSequenceDisable(ADC1_BASE, 0); // choose ADC1 sequence 0; disable before configuring
    ADCSequenceConfigure(ADC1_BASE, 0, ADC_TRIGGER_ALWAYS, 0); // specify the "Always" trigger
    ADCSequenceStepConfigure(ADC1_BASE, 0, 0,
    ADC_CTL_CH3 | ADC_CTL_IE | ADC_CTL_END); // in the 0th step, sample channel 3 (AIN3)

    // Enable interrupt, and make it the end of sequence
    ADCSequenceEnable(ADC1_BASE, 0); // enable the sequence.  it is now sampling
    ADCIntEnable(ADC1_BASE, 0); // enable sequence 0 interrupt in the ADC1 peripheral
    IntPrioritySet(INT_ADC1SS0, 0);    // set ADC1 sequence 0 interrupt priority
    IntEnable(INT_ADC1SS0); // enable ADC1 sequence 0 interrupt in int. controller

    // Setup timer for ADC conversions trigger
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER2);
    TimerDisable(TIMER2_BASE, TIMER_A);
    TimerConfigure(TIMER2_BASE, TIMER_CFG_PERIODIC);
    TimerEnable(TIMER2_BASE, TIMER_A);
    TimerControlTrigger(TIMER2_BASE, TIMER_A, 1);

}

// ISR for ADC
void ADC_ISR(void)
{
    ADC1_ISC_R = ADC_ISC_IN0; // clear ADC1 sequence0 interrupt flag in the ADCISC register
    if (ADC1_OSTAT_R & ADC_OSTAT_OV0) // check for ADC FIFO overflow
    {
        gADCErrors++;                   // count errors
        ADC1_OSTAT_R = ADC_OSTAT_OV0;   // clear overflow condition
    }
    gADCBuffer[gADCBufferIndex = ADC_BUFFER_WRAP(gADCBufferIndex + 1)] =
    ADC1_SSFIFO0_R;         // read sample from the ADC1 sequence 0 FIFO
}

// Initialize all necessary components
void initMain()
{
    IntMasterDisable();
    // Enable the Floating Point Unit, and permit ISRs to use it
    FPUEnable();
    FPULazyStackingEnable();
    // Initialize the system clock to 120 MHz
    gSystemClock = SysCtlClockFreqSet(
    SYSCTL_XTAL_25MHZ | SYSCTL_OSC_MAIN | SYSCTL_USE_PLL | SYSCTL_CFG_VCO_480, 120000000);
    Crystalfontz128x128_Init(); // Initialize the LCD display driver
    Crystalfontz128x128_SetOrientation(LCD_ORIENTATION_UP); // set screen orientation
    ButtonInit(); //Initialize buttons
    ADCInit(); //Initialize ADC
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER3);
    TimerDisable(TIMER3_BASE, TIMER_BOTH);
    TimerConfigure(TIMER3_BASE, TIMER_CFG_A_PERIODIC);
    TimerLoadSet(TIMER3_BASE, TIMER_A, gSystemClock / 100 - 1); // .01 sec interval
    count_unloaded = cpu_load_count(); //measure cpu load with ints disabled
    IntMasterEnable();
}

uint32_t cpu_load_count(void)
{
    uint32_t i = 0;
    TimerIntClear(TIMER3_BASE, TIMER_TIMA_TIMEOUT);
    TimerEnable(TIMER3_BASE, TIMER_A); // start one-shot timer
    while (!(TimerIntStatus(TIMER3_BASE, false) & TIMER_TIMA_TIMEOUT))
        i++;
    return i;
}

