#define F_CPU 14745600UL
#define UART_BAUD_RATE 115200
#define UART_BAUD_CALC (((F_CPU / (USART_BAUDRATE * 16UL))) - 1) 

#include <avr/io.h>
#include <stdlib.h>
#include <avr/pgmspace.h>
#include "avr/interrupt.h"
#include "inttypes.h"
#include "uart.h" 	// http://homepage.hispeed.ch/peterfleury/doxygen/avr-gcc-libraries/group__pfleury__uart.html#gac19a76bb7d446125734a67f9f4b68991
#include <util/delay.h>

/* Déclaration des variables */
volatile unsigned int potIn=0;              // Mesure potentiomètre par ADC
char buffer[5];     
uint16_t cptOVF;                            // Compteur d'overflow du timer
uint16_t nbOVF;                             // Valeur compteur lors du front montant
uint16_t res;                               // Résidu timer lors du  front montant
uint16_t yc; uint16_t y; int32_t e;         // Commande, mesure et erreur
uint8_t Kp=1; uint8_t Ki=3; uint8_t Kd=2;   // Gains 
int32_t ui; int32_t ui0;                    // Actuelle et ancienne consigne intégrateur
int32_t ud; int32_t ud0;
int32_t u;                                  // Consigne finale

/* Initialisations et fonctions */
void adc_init() { 
    // Single Ended Input ADC0
    ADMUX|=(0<<MUX4)|(0<<MUX3)|(0<<MUX2)|(0<<MUX1)|(0<<MUX0);
    // ADC registres de données: ajusté à gauche (8bits dans ADCH, 2 dans ADCL)
    ADMUX|=(1<<ADLAR);
    // Voltage: AVCC (broche 30) avec capacité externe sur la broche AREF
    ADMUX|=(1<<REFS0) | (0<<REFS1);
    // ADC activé, interruption autorisée, pré-diviseur = 128
    ADCSRA|=(1<<ADEN)|(1<<ADIE)|(1<<ADPS2)|(1<<ADPS1)|(1<<ADPS0);
    ADCSRA|=(1<<ADSC); // Démarre la conversion
}

void timer_init() {
    DDRD &=~ (1<<DDD6);            // Pin PD6 mis en entrée (lecture signal de l'encodeur)
    TCCR1A = 0x00;  TCCR1B = 0x00; // Initialise les 8 bits bas et haut à 0
    // Pré-diviseur=1 (CS10), front de détection montant (ICES), suppresseur de bruit (ICNC1)
    TCCR1B |= (1<<CS10) | (1<<ICES1) | (1<<ICNC1);  
    // Autorise interruption sur overflow (TOIE1) and front montant sur entrée (TICIE1)
    TIMSK |= (1<<TOIE1) | (1<<TICIE1);
}

void pwm_init() {
    DDRB |= (1<<PB3); // Mise en sortie broche de commande de la tension (PWM)
    DDRA |= (1<<PA1); // Mise en sortie broche de commande du sens de rotation du moteur
    // WGM: Fast PWM, COM: non-inversé, CS: pré-diviseur = 8 
    TCCR0 |=(1<<WGM00)|(1<<WGM01)|(0<<COM00)|(1<<COM01)|(1<<CS01); 
}

void pwmControl(uImp) {                     // Fonction de contrôle du PWM intégré
    PORTA &= (0<<PA2); PORTA |= (1<<PA1);   // Commande du sens de rotation
    OCR0 = uImp;                            // Commande de la largeur d'impulsion [0-255]
}

/* MAIN */
int main(void) {
    sei(); // Autorisation globale des interruptions
    
    uart_init(UART_BAUD_SELECT(UART_BAUD_RATE,F_CPU)); // Initialise UART and défini le Baud Rate
    pwm_init();
    timer_init();
    adc_init();
    
    while(1) {
        y = F_CPU/(500*(nbOVF*65535+res));  // Vitesse moteur mesurée [tour/s]
        yc = potIn>>10;                     // Consigne (potentiomètre) [0-255]
        e =  yc - y;                        // Erreur
        
        ui = ui0 + Ki*(e + e0);             // Consigne intégrateur
        if (Kp*ui>254){ui=255/Kp;}	
        if (Kp*ui<-254){ui=-255;}
        ud = -ud0 + Kd*(e-e0);              // Consigne dérivateur
        if (Kp*ud>254){ud=255/Kp;}	
        if (Kp*ud<-254){ud=-255;}
        u = Kp*(e+ui+ud);                   // Consigne finale
        if (u>254){u=255;}
        if (u<0){u=0;}
        
        pwmControl(u); // Appel de la fonction de contrôle avec la consigne u
        
        uart_puts("Consigne [Hz]:");    itoa(yc,buffer,10); uart_puts(buffer);
        uart_puts("   Sortie [Hz]:");   itoa(y,buffer,10); uart_puts(buffer);
        uart_puts("   Erreur [Hz]:");   itoa(e,buffer,10); uart_puts(buffer);
        uart_putc('\n');
        uart_puts("Intégrateur:");      itoa(ui,buffer,10); uart_puts(buffer);
        uart_puts("   Dérivateur:");    itoa(ud,buffer,10); uart_puts(buffer);
        uart_puts("   PWM:");           itoa(u,buffer,10); uart_puts(buffer);
        uart_putc('\n');
        
        e0 = e; ui0 = ui; ud0 = ud;  // Mise à jour des variables
        
        _delay_ms(100);
    }
}

/* Routines d'interruption */ 
ISR(ADC_vect) { // Fin de conversion de l'ADC           
    while(ADCSRA & (1<<ADSC));      // Attente que la conversion soit terminée
    unsigned char adcl = ADCL;
    unsigned char adch = ADCH;
    potIn = (adch<<8) | adcl;       // Récupère la valeur
    ADCSRA|=(1<<ADSC);              // Relance une nouvelle conversion
}

ISR(TIMER1_OVF_vect) { // Overflow du compteur        
    cptOVF++; 
    TCNT1=0;
}

ISR(TIMER1_CAPT_vect) { // Front montant du signal de l'encodeur          
    res = ICR1; 
    nbOVF = cptOVF;	
    TCNT1 = 0; 
    cptOVF = 0;
}