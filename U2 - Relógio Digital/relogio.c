
#define F_CPU 160000000UL
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#define set_bit(Y, bit_x) (Y |= (1 << bit_x)) //ligar os LEDs
#define clr_bit(Y, bit_x) (Y &= ~(1 << bit_x)) //desligar os LEDs
#define tst_bit(Y, bit_x) (Y & (1 << bit_x)) //testar se os botões foram pressionados

int cont = 0;
int segUnidade = 0;
const unsigned char Tabela[] = {0x00, 0x04, 0x08, 0x0C,
                                 0x10, 0x14, 0x18,0x1C,
                                 0x20, 0x24};


void setup()
{
  DDRD = 0xFC;  //1111 1100 PD2 até PD7
  DDRB = 0xFF; // 1111 1111 PB0 até PB5
  DDRC = 0x3F;
  
  PORTB = Tabela[1];
  PORTD = Tabela[9];
  PORTC = Tabela[0];
  
  //temporizador modo normal
  TCCR0B = (1 << CS02) | (1 <<CS00); //prescaller 1024 101
  TCCR0A = 0;
  TIMSK0 = 1<<TOIE0;
  sei();

 //set_bit(PORTB, PB1);


  
}
int main(){
  setup(); 
  while (1){} //loop de exução
  return 0;
}


ISR(TIMER0_OVF_vect){
  cont++;
  if(cont >= 61){ //1000 / 16,384 = 61,03
    if(segUnidade <= 8){
    	segUnidade++;
    } else {
      segUnidade = 0;
    }
    PORTC = Tabela[segUnidade];
    cont = 0;
    
  }
}