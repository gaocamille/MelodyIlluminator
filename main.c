#include <ti/devices/msp432p4xx/inc/msp.h>
#include <ti/devices/msp432p4xx/driverlib/driverlib.h>
#include <ti/grlib/grlib.h>
#include "LcdDriver/Crystalfontz128x128_ST7735.h"
#include <stdio.h>

#define ARM_MATH_CM4

#include <arm_math.h>
#include <arm_const_structs.h>

#define TEST_LENGTH_SAMPLES 512
#define SAMPLE_LENGTH 512

/* ------------------------------------------------------------------
 * Global variables for FFT Bin Example
 * ------------------------------------------------------------------- */
uint32_t fftSize = SAMPLE_LENGTH;
uint32_t ifftFlag = 0;
uint32_t doBitReverse = 1;
volatile arm_status status;

/* Graphic library context */
Graphics_Context g_sContext;

#define SMCLK_FREQUENCY     48000000
#define SAMPLE_FREQUENCY    8000
#define AMPLITUDE_MAX 2047

// Définition des indices de fréquence pour les plages
#define LOW_FREQ_START_INDEX  (20 * SAMPLE_LENGTH / SAMPLE_FREQUENCY)
#define LOW_FREQ_END_INDEX    (700 * SAMPLE_LENGTH / SAMPLE_FREQUENCY)
#define MID_FREQ_START_INDEX  (700 * SAMPLE_LENGTH / SAMPLE_FREQUENCY)
#define MID_FREQ_END_INDEX    (7000 * SAMPLE_LENGTH / SAMPLE_FREQUENCY)


// Seuils pour l'activation des LED
#define LOW_INTENSITY_THRESHOLD  300

#define HIGH_INTENSITY_THRESHOLD 300


#define LED_1 BIT4  // Port 2.4 //rouge
#define LED_2 BIT5  // Port 2.5
#define LED_3 BIT6
#define LED_4 BIT1



/* DMA Control Table */
#if defined(__TI_COMPILER_VERSION__)
#pragma DATA_ALIGN(MSP_EXP432P401RLP_DMAControlTable, 1024)
#elif defined(__IAR_SYSTEMS_ICC__)
#pragma data_alignment=1024
#elif defined(__GNUC__)
__attribute__ ((aligned (1024)))
#elif defined(__CC_ARM)
__align(1024)
#endif
static DMA_ControlTable MSP_EXP432P401RLP_DMAControlTable[32];

/* FFT data/processing buffers*/
float hann[SAMPLE_LENGTH];
int16_t data_array1[SAMPLE_LENGTH];
int16_t data_array2[SAMPLE_LENGTH];
int16_t data_input[SAMPLE_LENGTH * 2];
int16_t data_output[SAMPLE_LENGTH];

volatile int switch_data = 0;

//uint32_t color = 0;

/* Timer_A PWM Configuration Parameter */
Timer_A_PWMConfig pwmConfig =
{
    TIMER_A_CLOCKSOURCE_SMCLK,
    TIMER_A_CLOCKSOURCE_DIVIDER_1,
    (SMCLK_FREQUENCY / SAMPLE_FREQUENCY),
    TIMER_A_CAPTURECOMPARE_REGISTER_1,
    TIMER_A_OUTPUTMODE_SET_RESET,
    (SMCLK_FREQUENCY / SAMPLE_FREQUENCY) / 2
};

// Définition des états
typedef enum {
    PROGRAM_1,
    PROGRAM_2,
    PROGRAM_3
} ProgramState;




volatile ProgramState currentProgram = PROGRAM_1;

// Fonctions pour chaque programme

void runProgram1(q15_t maxValue,  uint32_t maxIndex, q15_t lowMaxValue,  q15_t midMaxValue) {




                         arm_max_q15(data_output, fftSize, &maxValue, &maxIndex);
                         if (maxValue>1000){
                          // Allumer LED1 pour valeurs trop importantes
                          P2->OUT = LED_1;
                            }
                         else {

                       P2->OUT = LED_4;
                        }



                             int i=0;
                           // Trouver la valeur maximale dans chaque plage de fréquences


                               for (i = LOW_FREQ_START_INDEX; i <= LOW_FREQ_END_INDEX; i++) {
                                   if (data_output[i] > lowMaxValue) lowMaxValue = data_output[i];
                               }
                               for (i = MID_FREQ_START_INDEX; i <= MID_FREQ_END_INDEX; i++) {
                                   if (data_output[i] > midMaxValue) midMaxValue = data_output[i];
                               }


                               //printf("Max low Value: %d, Max mid Value: %d, Max high Value: %d\n", lowMaxValue,midMaxValue,highMaxValue);
                              printf("Max low Value: %d, Max mid Value: %d\n", lowMaxValue,midMaxValue);

                               // Calculer le rapport cyclique pour chaque LED
                               uint32_t dutyCycleLow = (lowMaxValue >= 600) ? 1000 : (lowMaxValue * 1000) / 600;
                               uint32_t dutyCycleMid = (midMaxValue >= 600) ? 1000 : (midMaxValue * 100) / 600;


                               // Configurer les registres CCR pour chaque LED

                               TIMER_A0->CCR[2] = (dutyCycleMid * TIMER_A0->CCR[0]) / 1000;  // LED_2 (Mediums)
                              TIMER_A0->CCR[3] = (dutyCycleLow * TIMER_A0->CCR[0]) / 1000;  // LED_3 (Graves)


                               //_delay_cycles(500); // Délai pour observer la variation



}

void runProgram2( q15_t maxValue) {


    // Allumer différentes LEDs en fonction de la fréquence max
                           if (maxValue >= 0 && maxValue <= 600)
                           {
                               P2->OUT = LED_3;  // Allume la LED sur le port 2.4
                           }
                           else if (maxValue > 600 && maxValue <= 1200)
                           {
                               P2->OUT = LED_2;  // Allume la LED sur le port 2.5
                           }
                           else if (maxValue > 1200 && maxValue <= 5000)
                           {
                               P2->OUT = LED_1;  // Allume la LED sur le port 2.6
                           }
                           else
                           {
                               P2->OUT = 0;  // Éteint toutes les LEDs si la fréquence ne correspond à aucune plage spécifiée
                           }

                           printf("Max Value: %d\n", maxValue );

}

void runProgram3( uint32_t maxIndex) {




    /* Contrôle des LEDs basé sur la fréquence */

       float dominantFrequency = (float)maxIndex * SAMPLE_FREQUENCY / fftSize;

       if (dominantFrequency < 200.0) {
           // Allumer LED3 pour les graves
           P2->OUT = LED_3;
       } else if (dominantFrequency >= 200.0 && dominantFrequency < 2000.0) {
           // Allumer LED2 pour les mediums
           P2->OUT = LED_2;
       } else {
           // Allumer LED1 pour les aigus
           P2->OUT = LED_1;
       }

       printf("Max Fr: %f\n",dominantFrequency );
}





void configureLED_P1(){
    // Éteindre toutes les LEDs
         P2->OUT &= ~(LED_1 | LED_2 | LED_3 | LED_4 );

          P2->DIR |= BIT5;
                P2->DIR |= BIT4;
                P2->DIR |= BIT6;
                P2->DIR |= BIT1;



                // Configuration de la broche P2.5 et P2.4 pour la fonction alternative 2 (Timer_A0)

         P2->SEL0 |= BIT5; P2->SEL1 &= ~BIT5;
                //P2->SEL0 |= BIT4; P2->SEL1 &= ~BIT4;
                P2->SEL0 |= BIT6; P2->SEL1 &= ~BIT6;
}

void configureLED_P2_P3(){
    // Éteindre toutes les LEDs
       P2->OUT &= ~(LED_1 | LED_2 | LED_3 | LED_4);

       // Configurer les broches en mode GPIO standard pour les programmes 2 et 3
       P2->SEL0 &= ~(BIT5 | BIT6); // Remettre en mode GPIO
       P2->SEL1 &= ~(BIT5 | BIT6); // Remettre en mode GPIO

       P2->DIR |= (LED_1 | LED_2 | LED_3); // Configurer en sortie


}

//pour bouton

void PORT1_IRQHandler() {
    // Vérifiez si le bouton connecté à P1.1 est pressé
    if (P1->IFG & BIT1) { // BIT1 correspond à la pin P1.1
        currentProgram = (currentProgram + 1) % 3; // Changez l'état du programme

        // Changer la configuration des LEDs en fonction du programme actuel
                     if (currentProgram == 0) {
                         configureLED_P1();
                     } else if (currentProgram == 1) {
                         configureLED_P2_P3();
                     }
                     else
                     {
                         configureLED_P2_P3();
                     }

        // Debounce (si nécessaire) pour éviter les changements d'état multiples dus aux rebonds du bouton
        // Par exemple, un simple délai:
                     volatile int i = 0;
        for (i = 0; i < 10000; i++);

        P1->IFG &= ~BIT1; // Effacer le flag d'interruption pour P1.1
    }
}



int main(void)
{

    /* set P1.1 as GPIO configuration */
       P1->SEL0 &= ~BIT1;
       P1->SEL1 &= ~BIT1;

       /* set P1.1  as input pins */
       P1->DIR &= ~BIT1 ;
       /* pullup mode configuration */
       P1->OUT |=  BIT1;
       /* enable pullup resistor */
       P1->REN  |= BIT1;
/*

           P1->IES |= BIT1;

           P1->IE |= BIT1 ;

           P1->IFG = 0;

           // Activer les interruptions dans le NVIC (Nested Vector Interrupt Controller)
                    NVIC_EnableIRQ(PORT1_IRQn);

                    // Activer les interruptions globalement
                    __enable_irq();
            */

           // Activer le mode interruption pour P1.1
           P1->IES |= BIT1;  // Configurer pour déclencher l'interruption sur front descendant
           P1->IFG &= ~BIT1; // Nettoyer le flag d'interruption
           P1->IE |= BIT1;   // Activer l'interruption pour P1.1

           // Activer les interruptions dans le NVIC (Nested Vector Interrupt Controller)
           NVIC_EnableIRQ(PORT1_IRQn);

           // Activer les interruptions globalement
           __enable_irq();





         // Étape 2: Configurer Timer_A0 pour la modulation de largeur d'impulsion (PWM)
            TIMER_A0->CTL = TIMER_A_CTL_SSEL__SMCLK | // SMCLK comme source d'horloge
                            TIMER_A_CTL_MC__UP |      // Mode de comptage en montée
                            TIMER_A_CTL_CLR;          // Effacer le Timer

            TIMER_A0->CCR[0] = 1000 - 1; // Période du PWM (définir en fonction de la fréquence souhaitée)

            // Configurer les canaux pour la sortie PWM
            // Pour P2.4 (LED connectée à CCR[1])
           // TIMER_A0->CCTL[1] = TIMER_A_CCTLN_OUTMOD_7; // Mode de sortie Reset/Set pour PWM
           // TIMER_A0->CCR[1] = 500; // Valeur initiale du rapport cyclique (50%)

            // Pour P2.5 (LED connectée à CCR[2])
            TIMER_A0->CCTL[2] = TIMER_A_CCTLN_OUTMOD_7; // Mode de sortie Reset/Set pour PWM
            TIMER_A0->CCR[2] = 500; // Valeur initiale du rapport cyclique (50%)

            // Pour P2.6 (LED connectée à CCR[3])
            TIMER_A0->CCTL[3] = TIMER_A_CCTLN_OUTMOD_7; // Mode de sortie Reset/Set pour PWM
            TIMER_A0->CCR[3] = 500; // Valeur initiale du rapport cyclique (50%)




    /* Halting WDT and disabling master interrupts */
    MAP_WDT_A_holdTimer();
    MAP_Interrupt_disableMaster();

    /* Set the core voltage level to VCORE1 */
    MAP_PCM_setCoreVoltageLevel(PCM_VCORE1);

    /* Set 2 flash wait states for Flash bank 0 and 1*/
    MAP_FlashCtl_setWaitState(FLASH_BANK0, 2);
    MAP_FlashCtl_setWaitState(FLASH_BANK1, 2);

    /* Initializes Clock System */
    MAP_CS_setDCOCenteredFrequency(CS_DCO_FREQUENCY_48);
    MAP_CS_initClockSignal(CS_MCLK, CS_DCOCLK_SELECT, CS_CLOCK_DIVIDER_1);
    MAP_CS_initClockSignal(CS_HSMCLK, CS_DCOCLK_SELECT, CS_CLOCK_DIVIDER_1);
    MAP_CS_initClockSignal(CS_SMCLK, CS_DCOCLK_SELECT, CS_CLOCK_DIVIDER_1);
    MAP_CS_initClockSignal(CS_ACLK, CS_REFOCLK_SELECT, CS_CLOCK_DIVIDER_1);

    // Initialize Hann Window
    int n;
    for(n = 0; n < SAMPLE_LENGTH; n++)
    {
        hann[n] = 0.5f - 0.5f * cosf((2 * PI * n) / (SAMPLE_LENGTH - 1));
    }

    /* Configuring Timer_A to have a period of approximately 500ms and
     * an initial duty cycle of 10% of that (3200 ticks)  */
    Timer_A_generatePWM(TIMER_A0_BASE, &pwmConfig);

    /* Initializing ADC (MCLK/1/1) */
    ADC14_enableModule();
    ADC14_initModule(ADC_CLOCKSOURCE_MCLK, ADC_PREDIVIDER_1, ADC_DIVIDER_1, 0);

    ADC14_setSampleHoldTrigger(ADC_TRIGGER_SOURCE1, false);

    /* Configuring GPIOs (4.3 A10) */
    GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P4, GPIO_PIN3,
                                               GPIO_TERTIARY_MODULE_FUNCTION);

    /* Configuring ADC Memory */
    ADC14_configureSingleSampleMode(ADC_MEM0, true);
    ADC14_configureConversionMemory(ADC_MEM0, ADC_VREFPOS_AVCC_VREFNEG_VSS,
                                    ADC_INPUT_A10, false);

    /* Set ADC result format to signed binary */
    ADC14_setResultFormat(ADC_SIGNED_BINARY);

    /* Configuring DMA module */
    DMA_enableModule();
    DMA_setControlBase(MSP_EXP432P401RLP_DMAControlTable);

    DMA_disableChannelAttribute(DMA_CH7_ADC14,
                                UDMA_ATTR_ALTSELECT | UDMA_ATTR_USEBURST |
                                UDMA_ATTR_HIGH_PRIORITY |
                                UDMA_ATTR_REQMASK);

    /* Setting Control Indexes. In this case we will set the source of the
     * DMA transfer to ADC14 Memory 0
     *  and the destination to the
     * destination data array. */
    DMA_setChannelControl(
        UDMA_PRI_SELECT | DMA_CH7_ADC14,
        UDMA_SIZE_16 | UDMA_SRC_INC_NONE |
        UDMA_DST_INC_16 | UDMA_ARB_1);
    DMA_setChannelTransfer(UDMA_PRI_SELECT | DMA_CH7_ADC14,
                               UDMA_MODE_PINGPONG, (void*) &ADC14->MEM[0],
                               data_array1, SAMPLE_LENGTH);

    DMA_setChannelControl(
        UDMA_ALT_SELECT | DMA_CH7_ADC14,
        UDMA_SIZE_16 | UDMA_SRC_INC_NONE |
        UDMA_DST_INC_16 | UDMA_ARB_1);
    DMA_setChannelTransfer(UDMA_ALT_SELECT | DMA_CH7_ADC14,
                               UDMA_MODE_PINGPONG, (void*) &ADC14->MEM[0],
                               data_array2, SAMPLE_LENGTH);

    /* Assigning/Enabling Interrupts */
    DMA_assignInterrupt(DMA_INT1, 7);
    Interrupt_enableInterrupt(INT_DMA_INT1);
    DMA_assignChannel(DMA_CH7_ADC14);
    DMA_clearInterruptFlag(7);
    Interrupt_enableMaster();

    /* Now that the DMA is primed and setup, enabling the channels. The ADC14
     * hardware should take over and transfer/receive all bytes */
    DMA_enableChannel(7);
    ADC14_enableConversion();


    while(1)
    {
        PCM_gotoLPM0();

        int i = 0;

                    /* Computer real FFT using the completed data buffer */
                    if(switch_data & 1)
                    {
                        for(i = 0; i < 512; i++)
                        {
                            data_array1[i] = (int16_t)(hann[i] * data_array1[i]);
                        }
                        arm_rfft_instance_q15 instance;
                        status = arm_rfft_init_q15(&instance, fftSize, ifftFlag,
                                                   doBitReverse);

                        arm_rfft_q15(&instance, data_array1, data_input);
                    }
                    else
                    {
                        for(i = 0; i < 512; i++)
                        {
                            data_array2[i] = (int16_t)(hann[i] * data_array2[i]);
                        }
                        arm_rfft_instance_q15 instance;
                        status = arm_rfft_init_q15(&instance, fftSize, ifftFlag,
                                                   doBitReverse);

                        arm_rfft_q15(&instance, data_array2, data_input);
                    }

                    /* Calculate magnitude of FFT complex output */


                    for(i = 0; i < 1024; i += 2)
                            {
                                data_output[i /
                                            2] =
                                    (int32_t)(sqrtf((data_input[i] *
                                                     data_input[i]) +
                                                    (data_input[i + 1] * data_input[i + 1])));
                            }



                    q15_t maxValue;
                    uint32_t maxIndex = 0;
                    q15_t lowMaxValue = 0, midMaxValue = 0;
                    arm_max_q15(data_output, fftSize, &maxValue, &maxIndex);

                     switch (currentProgram) {
                                     case PROGRAM_1:
                                       runProgram1(maxValue, maxIndex,lowMaxValue, midMaxValue);
                                    break;
                                    case PROGRAM_2:
                                        runProgram2(maxValue);
                                        break;
                                    case PROGRAM_3:
                                        runProgram3(maxIndex);
                                        break;
                                }


    }
}

/* Completion interrupt for ADC14 MEM0 */
void DMA_INT1_IRQHandler(void)
 {
    /* Switch between primary and alternate bufferes with DMA's PingPong mode */
    if(DMA_getChannelAttribute(7) & UDMA_ATTR_ALTSELECT)
    {
        DMA_setChannelControl(
            UDMA_PRI_SELECT | DMA_CH7_ADC14,
            UDMA_SIZE_16 | UDMA_SRC_INC_NONE |
            UDMA_DST_INC_16 | UDMA_ARB_1);
        DMA_setChannelTransfer(UDMA_PRI_SELECT | DMA_CH7_ADC14,
                               UDMA_MODE_PINGPONG, (void*) &ADC14->MEM[0],
                               data_array1, SAMPLE_LENGTH);
        switch_data = 1;
    }
    else
    {
        DMA_setChannelControl(
            UDMA_ALT_SELECT | DMA_CH7_ADC14,
            UDMA_SIZE_16 | UDMA_SRC_INC_NONE |
            UDMA_DST_INC_16 | UDMA_ARB_1);
        DMA_setChannelTransfer(UDMA_ALT_SELECT | DMA_CH7_ADC14,
                               UDMA_MODE_PINGPONG, (void*) &ADC14->MEM[0],
                               data_array2, SAMPLE_LENGTH);
        switch_data = 0;
    }
 }

