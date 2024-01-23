

#include <ti/devices/msp432p4xx/inc/msp.h>
#include <ti/devices/msp432p4xx/driverlib/driverlib.h>
#include <ti/grlib/grlib.h>
#include "LcdDriver/Crystalfontz128x128_ST7735.h"
#include <stdio.h>



void main(void)
{
    int i;

    // Stop watchdog timer
    WDT_A->CTL = WDT_A_CTL_PW | WDT_A_CTL_HOLD;

    // Configuration de la broche P2.5  et P2.4 en tant que sorties
    P2->DIR |= BIT5;
    P2->DIR |= BIT4;
    P2->DIR |= BIT6;

    // Configuration de la broche P2.5 et P2.4 pour la fonction alternative 2 (Timer_A0)
    P2->SEL0 |= BIT5; P2->SEL1 &= ~BIT5;
    P2->SEL0 |= BIT4; P2->SEL1 &= ~BIT4;
    P2->SEL0 |= BIT6; P2->SEL1 &= ~BIT6;

    // Configuration du Timer_A0 pour la modulation de largeur d'impulsion (PWM)
    TIMER_A0->CTL = TIMER_A_CTL_SSEL__SMCLK |  // Utiliser SMCLK comme source d'horloge
                    TIMER_A_CTL_MC__UP |       // Mode de compteur ascendante
                    TIMER_A_CTL_CLR;           // Effacer le compteur

    // Configuration du canal CCR2 du Timer_A0 pour la sortie PWM
    TIMER_A0->CCR[0] = 1000 - 1;                // Période du signal PWM (1000 cycles)
    // Canal CCR2 -> (TA0.2)
    TIMER_A0->CCR[2] = 500;  // Canal CCR1     // Valeur du compteur pour contrôler le rapport cyclique (50% initial)
    TIMER_A0->CCTL[2] = TIMER_A_CCTLN_OUTMOD_7; // Mode de sortie PWM
    // Canal CCR1 -> (TA0.1)
    TIMER_A0->CCR[1] = 500;                      // Valeur du compteur pour contrôler le rapport cyclique (50% initial)
    TIMER_A0->CCTL[1] = TIMER_A_CCTLN_OUTMOD_7; // Mode de sortie PWM
    // Canal CCR2 -> (TA0.3)
        TIMER_A0->CCR[3] = 500;  // Canal CCR1     // Valeur du compteur pour contrôler le rapport cyclique (50% initial)
        TIMER_A0->CCTL[3] = TIMER_A_CCTLN_OUTMOD_7; // Mode de sortie PWM

    // Boucle infinie
    while (1)
    {
        // Variation de l'intensité lumineuse en modifiant la valeur du rapport cyclique
        for ( i = 0; i < 1000; ++i)
        {
            TIMER_A0->CCR[2] = i;
            TIMER_A0->CCR[1] = i;
            TIMER_A0->CCR[3] = i;
            __delay_cycles(1000); // Délai pour observer la variation
        }

        // Inversion de la variation de l'intensité lumineuse
        for ( i = 1000; i > 0; --i)
        {
            TIMER_A0->CCR[2] = i;
            TIMER_A0->CCR[1] = i;
            TIMER_A0->CCR[3] = i;
            __delay_cycles(1000); // Délai pour observer la variation
        }
    }
}





