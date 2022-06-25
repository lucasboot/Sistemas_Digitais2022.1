
#define F_CPU 160000000UL
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

#define DS 8
#define EN 10
#define CLK 11

#define D0 PD5
#define D1 PD4
#define D2 PD3
#define D3 PD2

#define set_bit(Y, bit_x) (Y |= (1 << bit_x)) //ligar os LEDs
#define clr_bit(Y, bit_x) (Y &= ~(1 << bit_x)) //desligar os LEDs
#define tst_bit(Y, bit_x) (Y & (1 << bit_x)) //testar se os botões foram pressionados


void serialInit();
void serialWrite(unsigned char data);

void ssdInit();
void ssdUpdate();
void ssdDigit(unsigned char pin, unsigned char data);

char disp[4]; // Valor a ser escrito em cada display
char atual=0; // Display a ser atualizado

void setup(){
  DDRD = 0x3C;
  PORTD = 0x00;
  DDRB = 0x0D;
  PORTB = 0x00;

  ssdDigit(0, 5); // Escreve  no display 0
  ssdDigit(1, 2);
  ssdDigit(2, 3); // Escreve 0 no display 2
  ssdDigit(3, 4); // Escreve 0 no display 3
  ssdUpdate();
  ssdUpdate();
  
    /*
  serialWrite(disp[0]);
  set_bit(PORTD, D0);

  serialWrite(disp[1]);
  //set_bit(PORTD, D1);

  serialWrite(disp[2]);
  //set_bit(PORTD, D2);

  serialWrite(disp[3]);
  //set_bit(PORTD, D3);*/

}

void ssdDigit(unsigned char pin, unsigned char data){
  static const char valor [] = {0xFC , 0x60 , 0xDA , 0xF2 , 0x66 , 0xB6 , 0xBE , 0xE0 , 0xFE , 0xF6 , 0xEE , 0x3E , 0x9C , 0x7A ,0x9E , 0x8E};
  disp[pin] = valor[data]; // Atualiza a posição referente ao display com o valor referente do número desejado
  //set_bit(PORTD, valor[data]);
}

void ssdUpdate(){
    /*
  digitalWrite(D0, LOW);
  digitalWrite(D1, LOW);
  digitalWrite(D2, LOW);
  digitalWrite(D3, LOW); */// Desliga todos os displays
  // A cada chamada da função é escrito na saída do registrador o valor correspondente a um display
  // A variável atual é atualizada de forma que na próxima chamada da função o próximo display seja atualizado
  switch (atual){
    case 0:
    serialWrite(disp[0]);
    digitalWrite(D0, HIGH);
    atual = 1;
    	break;
    case 1:
    serialWrite(disp[1]);
    digitalWrite(D1, HIGH);
    atual = 2;
    	break;
    case 2:
    serialWrite(disp[2]);
    digitalWrite(D2, HIGH);
    atual = 3;
    	break;
    case 3:
    serialWrite(disp[3]);
    digitalWrite(D3, HIGH);
    atual = 0;
    	break;
    default:
    atual = 0;
    	break;
  }
}

void serialWrite(unsigned char data){
  digitalWrite(EN, LOW); //Desabilita a transferencia de dados para a saida
  
  for (char i = 0; i < 8; i++){
    digitalWrite(CLK, LOW);
    if (data & 0x80>>i){ //0x80 = 0b1000 0000
      digitalWrite(DS, HIGH);
    }
    else{
      digitalWrite(DS, LOW);
    }
    digitalWrite(CLK, HIGH);
  }
  digitalWrite(EN, HIGH); //Habilita a transferencia de dados para a saida
  digitalWrite(DS, LOW); //Mantém o restante com nível baixo
  digitalWrite(CLK, LOW);
  
}
int main (){
    setup();
    while(1);
    return 0;
}







  /*
  ssdDigit(0, millis()/1000%10);  	// unidades de segundo
  ssdDigit(1, (millis()/10000)%10); // dezenas de segundo
  ssdDigit(2, (millis()/100000)%10); // centenas de segundo
  ssdDigit(3, (millis()/1000000)%10); // milhares de segundo
  */