/*
 * oscilloscope.c
 *
 *  Created on: Mar 26, 2019
 *      Author: mhle
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
#include "driverlib/timer.h"
#include "inc/hw_memmap.h"
#include "inc/hw_ints.h"
#include "sysctl_pll.h"

#define ADC_BUFFER_WRAP(i) ((i) & (ADC_BUFFER_SIZE - 1))
#define ADC_BUFFER_SIZE 2048

extern volatile uint32_t gADCBufferIndex;  // Latest sample index
extern volatile uint16_t gADCBuffer[];     // Circular buffer
extern volatile uint32_t ADCIndex;
extern uint16_t ADCDraw[128];
const char * const gTimeScaleStr[] = { "100 ms", "50ms", "20ms", "10 ms",
                                       "5 ms", "2 ms", "1 ms", "500 us",
                                       "200 us", "100 us", "50 us", "20 us" }; // Strings to draw time scale on LCD display
const char * const gVoltageScaleStr[] =
        { "100 mV", "200 mV", "500 mV", "  1 V" };  // Strings to draw voltage scale on LCD display

// Draw oscilloscope's grid on the LCD screen
void drawGrid(tContext sContext)
{
    GrContextForegroundSet(&sContext, ClrDarkGreen); // LCD Lines green
    int count;
    // Draw vertical lines
    for (count = 0; count < 8; count++)
    {
        GrLineDrawV(&sContext, 4 + count * 20, 0, 127);
    }
    // Draw horizontal lines
    for (count = 0; count < 8; count++)
    {
        GrLineDrawH(&sContext, 0, 127, 4 + count * 20);
    }
}

// Draw the trigger icon on the LCD screen
void drawTriggerIcon(tContext sContext, uint16_t direction)
{
    if (direction == 1) // Falling edge
    {
        GrLineDrawV(&sContext, 110, 12, 1);
        GrLineDrawH(&sContext, 109, 115, 12);
        GrLineDrawH(&sContext, 105, 111, 1);
        GrLineDraw(&sContext, 108, 5, 109, 6);
        GrLineDraw(&sContext, 112, 5, 111, 6);
    }
    else //Rising edge
    {
        GrLineDrawV(&sContext, 110, 12, 1);
        GrLineDrawH(&sContext, 104, 110, 12);
        GrLineDrawH(&sContext, 110, 116, 1);
        GrLineDraw(&sContext, 108, 6, 109, 5);
        GrLineDraw(&sContext, 112, 6, 111, 5);
    }
}

// Draw all elements on the LCD screen
void drawScreen(tContext sContext, uint16_t direction, uint16_t vScaleState,
                uint16_t samplingState, char* string)
{
    // Draw grid
    drawGrid(sContext);
    // Draw trigger icon
    GrContextForegroundSet(&sContext, ClrAquamarine);
    drawTriggerIcon(sContext, direction);
    // Draw time scale string
    GrStringDraw(&sContext, gTimeScaleStr[samplingState], -1, 10, 1, false);
    // Draw voltage scale string
    GrStringDraw(&sContext, gVoltageScaleStr[vScaleState], -1, 55, 1, false);
    // Draw CPU load string
    GrStringDraw(&sContext, string, -1, 5, 115, false);
    // Draw trigger level and location
    GrContextForegroundSet(&sContext, ClrLightSalmon);
    GrLineDrawH(&sContext, 0, 128, 64);
    GrCircleDraw(&sContext, 64, 64, 3);
}

// Perform trigger search and take out 128 samples from ADC circular buffer to display on LCD screen
void getWaveform(int triggerDirection, uint16_t triggerVoltage)
{
    uint16_t prevVoltage = 0;
    ADCIndex = ADC_BUFFER_WRAP(gADCBufferIndex - 64); // Index begins at half a screen before the most recent sample
    prevVoltage = gADCBuffer[ADCIndex];               // Store the voltage at ADCIndex
    while (1)
    {
        if (triggerDirection == 1) // Falling edge trigger. Checks if voltage crosses the set threshold
        {
            if (prevVoltage < triggerVoltage
                    && gADCBuffer[ADCIndex] >= triggerVoltage) //Get out of loop if the trigger is found
            {
                break;
            }
            else                   // Go to next index if no trigger is found
            {
                prevVoltage = gADCBuffer[ADCIndex];
                ADCIndex = ADC_BUFFER_WRAP(ADCIndex - 1);
            }
        }
        else                       // Rising edge trigger. Checks if voltage crosses the set threshold
        {
            if (prevVoltage > triggerVoltage
                    && gADCBuffer[ADCIndex] <= triggerVoltage) //Get out of loop if the trigger is found
            {
                break;
            } // Go to next index if no trigger is found
            else
            {
                prevVoltage = gADCBuffer[ADCIndex];
                ADCIndex = ADC_BUFFER_WRAP(ADCIndex - 1);
            }
        }
    }
    // Once trigger is found, pull 128 samples from circular buffer (64 before index, 64 after index)
    int cnt;
    ADCIndex = ADC_BUFFER_WRAP(ADCIndex - 63);
    for (cnt = 0; cnt < 128; cnt++)
    {
        ADCDraw[cnt] = gADCBuffer[ADCIndex];
        ADCIndex = ADC_BUFFER_WRAP(ADCIndex + 1);
    }
}

