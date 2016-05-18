#include <avr/io.h>
#include <stdint.h>
#include <avr/interrupt.h>
#include <stdbool.h>
#include <util/delay.h>
#define F_CPU 20000000UL

/*
 * ZASADA DZIAŁANIA MOSTKA !!!
 * PWML i PWMR przekazuje sygnał PWM na piny EA odpowiednich silników.
 * Piny EA decyduja o predkosci !
 * Piny IN1,IN2,IN3,IN4 na mostku decydują o trybie pracy silnikow - FOrward, Reverse
 */
#define PWML PB1 // Łączymy z pinem EAA na mostku.
#define PWMR PB2 // Łączymy z pinem EAB

#define EAF  PB0 // Enable engine-A Forward - IN1
#define EAR  PB3 // Enable engine-A Reverse - IN2
#define EBF  PB4 // Enable engine-B Forward - IN3
#define EBR  PB5 // Enable engine-B Reverse - IN4





#define BEG_SPEED 0 // wartośc początkowa prędkości kół
uint16_t DEF_SPEED=2; // domyślna prędkośc kół
int last_state;
#define FOR_SPEED 10


#define F1          0
#define F2          1
#define F3          2
#define F4          3
#define F5          4

#define ADC_THRESHOLD   900


// Sekcja odpowiadająca za skrecanie - 11 stopni skrętu.
#define T0 0
#define T1 10
#define T2 20
#define T3 30
#define T4 40
#define T5 50
#define T6 60
#define T7 70
#define T8 80
#define T9 90
#define T10 100

///////////////////////////////////////////////////////////////////////////////////////////////
///                                                                                       /////
/// w przypadku wątpliwości : http://www.voytek.evbox.pl/programy/adc/Przetwornik_AC.html /////
///                                                                                       /////
///////////////////////////////////////////////////////////////////////////////////////////////

/*
 * Struktura zapisujaca stan czujników
 */
typedef struct{
    int TO1;
    int TO2;
    int TO3;
    int TO4;
    int TO5;
}trans_state;

/*
 * ADEN - zezwala na pracę przetwornika anologowo-cyfrowego
 * ADPS - wszystkie 3 odpowiadaja za taktowanie zegara atmegi.
 * Ustawione według wzoru 50kHz < F_atmegi/F_zegara <200kHz. W tym przypadku 20MHz/8
 * REFS0 opisane poniżej.
 * ADLAR - wybór zapisu wartości ADC z rejestrów.
 */
void Inicjalizacja_ADC ()
{
    ADMUX = (1<<REFS0) | (1<<ADLAR);
    ADCSRA= (1<<ADEN)| (1<<ADPS2)|(1<<ADPS1)|(1<<ADPS0);
}

/*
 * Odczytywanie wartości rejestru ADC(wartośc na czujniku rozróżniająca kolor)
 * Refs0 - rejest(chyba rejestr) ustawienia napięcia referencyjnego potrzebnego do porównania napięcia
 * na czujniku i na wejściu Aref atmegi. Porównujemy dla Aref z napięciem idacym przez kondensator
 * ADCSRA |= _BV(ADSC); - rozpoczynamy konwersje sygnału analogowego na digital. Pętla czeka na zakończenie konwersji.
 */
uint16_t adc_read(uint8_t adcx) {

    ADMUX   &=  0xf0;
    ADMUX   = _BV(REFS0) | adcx;

    ADCSRA |= _BV(ADSC);
    while ( (ADCSRA & _BV(ADSC)) );

    return ADC;
}

/*
 * Funkcja odpowiada ustawianie 1 lub 0 w zależności od odczytanej wartości z czujników
 * 1 dla czarnej linii, 0 dla białej
 */
int detector(uint8_t adcx){
    if(adc_read(adcx)>ADC_THRESHOLD)
        return 1;
    else
        return 0;
}

/*
 * Ustawianie stanu czujników wdg działania funckji "detector".
 * Potrzebne do wykonania odpowiedniego case w funkcji "execute_state"
 */
void state_detector(trans_state *state){
    state->TO1=detector(F1);
    state->TO2=detector(F2);
    state->TO3=detector(F3);
    state->TO4=detector(F4);
    state->TO5=detector(F5);
}

/*
 * Obliczanie odpowiedniego case którego będziemy wykonywac.
 * Zeby kazdy case byl unikalny kazdy z czujnikow ma przypisana swoja potege 2.
 * TO1 - czujnik z lewej strony, TO5 - czujnik z prawej strony.
 */
int calculate_state(trans_state *state){
    return (state->TO1*16)+(state->TO2*8)+(state->TO3*4)+(state->TO4*2)+(state->TO5);
}

/*
 * Stan startowy
 */
void start(trans_state *trans){
    trans->TO1=0;
    trans->TO2=0;
    trans->TO3=0;
    trans->TO4=0;
    trans->TO5=0;
}

/*
 * Funkcje odpowiadajace za tryb dzialania silnika. Silnik A - lewe koło, silnik B - prawe koło
 */
void forward(){
    PORTB |= _BV(EAF);
    PORTB |= _BV(EBF);
    PORTB &= ~_BV(EAR);
    PORTB &= ~_BV(EBR);
    DEF_SPEED=FOR_SPEED;
}

void reverse(){
    PORTB &= ~_BV(EAF);
    PORTB &= ~_BV(EBF);
    PORTB |= _BV(EAR);
    PORTB |= _BV(EBR);
}

void right_forward(){
    PORTB |= _BV(EAF);
    PORTB &= ~_BV(EBF);
    PORTB &= ~_BV(EAR);
    PORTB |= _BV(EBR);
    DEF_SPEED=FOR_SPEED;
}

void right_reverse(){
    PORTB &= ~_BV(EAF);
    PORTB |= _BV(EBF);
    PORTB |= _BV(EAR);
    PORTB &= ~_BV(EBR);
    DEF_SPEED=FOR_SPEED;
}

void stop(){
    PORTB &= ~_BV(EAF);
    PORTB &= ~_BV(EBF);
    PORTB &= ~_BV(EAR);
    PORTB &= ~_BV(EBR);
}


void initPWM(){
    TCCR1A |= _BV(COM1A1) | _BV(COM1B1)| _BV(WGM12) | _BV(WGM11) | _BV(WGM10); // Ustawiamy tryb pracy taki, że PWM jest kodowane na 9 bitach
    TCCR1B |= _BV(CS10); // brak preskalera - timer działa z max częstotliwością

    OCR1A = BEG_SPEED;
    OCR1B = BEG_SPEED;

    forward();
}

/*
 * Ustawiamy port B na output, port C na input.
 */
void initPIN(){
    DDRB |= _BV(PB0) | _BV(PB1) | _BV(PB2) | _BV(PB3) | _BV(PB4) |_BV(PB5);
    DDRC=0x00;

}

/*
 * Nazwa mowi wszystko ;)
 */
void change_speed(uint8_t left,uint8_t right){
    OCR1A = DEF_SPEED*right;
    OCR1B = DEF_SPEED*left;
}

/*
 * Zmiana trybu dzialania silnika w przypadku gdy robot zgubi linie.
 * last_state==1 jedziemy w prawo i na odwrót.
 */
void execute_last_state(int last_state){
    if(last_state==1){
        right_reverse();
        change_speed(T4,T4);
    }
    else if(last_state==0){
        right_forward();
        change_speed(T4,T4);
    }
    else{
    	forward();
    	change_speed(T9,T9);
    }
}

/*
 * Nieprzerwanie wykonywana funkcja sterujaca silnikiem
 * Odpowiednie case można sobie rozrysowac według zasady z funkcji "calculate_state"
 */
void execute_state(trans_state *state){

    /*
     * sposób liczenia Casów :
     * Wartosci czujnikow od lewj strony (przy stanie wysokim) :
     * PC0 - 16, PC1  - 8, PC2 - 4, PC3 - 2, PC4 - 1
     */
    int tmp=calculate_state(state);
    switch(tmp){
                case 4: //Jazda na wprost
                case 10:
                case 14:
                case 17:
                case 21:
                case 27:
                case 29:
                case 31:
                    forward();
                    change_speed(T10,T10);
                    break;
                case 2: //Delikatny zakręt w prawo
                case 6:
                case 11:
                case 18:
                case 22:
                    forward();
                    change_speed(T10,T9);
                    last_state=1;
                    break;
                case 3: // ostry zakręt w prawo
                case 7:
                    forward();
                    change_speed(T10,T8);
                    last_state=1;
                    break;
                case 15:
                    forward();
                    change_speed(T10,T5);
                    last_state=1;
                    break;
                case 24: //ostry zakret w lewo
                case 25:
                case 26:
                case 28:
                    forward();
                    change_speed(T8,T10);
                    last_state=0;
                    break;
                case 30:
                    forward();
                    change_speed(T5,T10);
                    last_state=0;
                    break;
                case 8: //delikatny zakret w lewo
                case 9:
                case 12:
                case 13:
                    forward();
                    change_speed(T9,T10);
                    last_state=0;
                    break;
                case 0: //wsteczny
                    execute_last_state(last_state);
                    break;
                    /*reverse();
                    change_speed(T7,T7);
                    _delay_ms(10);
                    break;*/
                case 1:
                    right_reverse();
                    change_speed(T6,T6);
                    last_state=1;
                    break;
                case 16:
                    right_forward();
                    change_speed(T6,T6);
                    last_state=0;
                    break;
    default :
        forward();
        change_speed(T10,T10);
    }
}

int main(void){

    Inicjalizacja_ADC();
    ADCSRA |= _BV(ADEN);

    initPIN();
    initPWM();

    trans_state state;
    trans_state last_state;

    start(&state);
    start(&last_state);


    for (;;) {
        state_detector(&state);
        execute_state(&state);
    }

}
