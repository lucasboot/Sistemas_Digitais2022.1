//Configurações
#define F_CPU 16000000UL
#define BAUD 9600
#define MYUBRR F_CPU/16/BAUD-1

//Bibliotecas
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>


//Macros
#define set_bit(Y, bit_x) (Y |= (1 << bit_x)) //ligar os LEDs
#define clr_bit(Y, bit_x) (Y &= ~(1 << bit_x)) //desligar os LEDs
#define tst_bit(Y, bit_x) (Y & (1 << bit_x)) //testar se os botões foram pressionados
#define clp_bit(Y, bit_x) (Y ^= (1 << bit_x)) //Muda o estado da porta


//Estados, so tem 2 que nao tem transicao vazia, então 
//so precisa de um bool funcionando OU mudando_temp
bool mudando_temp = false;
unsigned int temperatura = 25; //comeca setado em 25°C


const unsigned char diusplay[]= {0xC3,0xC7,0xCB,0xCF,0xD3,0xD7,0xDB,0xDF,0xE3,0xE7};



void setup(){
  	DDRB = 0x3F;
    DDRD = 0x3F;
  	PORTB = display[temperatura%10];
    PORTD = display[temperatura/10];
}


int main(){
	setup();
	while(1);
	return 0;
}
