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

int cont = 0;
int segUnidade = 0;
int minUnidade = 0;
int minDezena = 0;

enum estados{INICIAL=1, AJUSTE_HORA, RELOGIO, CRIAR_ALARME, TOCAR_ALARME};
estados atual = INICIAL;

const unsigned char Tabela[] = {0x00, 0x04, 0x08, 0x0C,
                                 0x10, 0x14, 0x18,0x1C,
                                 0x20, 0x24};

//Assinatura das funções de comunicação
void UART_Init(void);
uint8_t uartTxOk(void);
void uart_Transmit (uint8_t data);
void uartString (char *c);
uint8_t uartRxOk(void);
uint8_t uartRX();
void uartIntRx(uint8_t _hab);
void uartIntTx(uint8_t _hab);


void setup()
{
  DDRD = 0xFC;  //1111 1100 PD2 até PD7
  DDRB = 0xFF; // 1111 1111 PB0 até PB5
  DDRC = 0x3F;
  
  PORTB = Tabela[minDezena]; //dez minutos
  PORTD = Tabela[0]; //uni minutos
  PORTC = Tabela[0]; //uni segundos
  
  //temporizador modo normal
  TCCR0B = (1 << CS02) | (1 <<CS00); //prescaller 1024 101
  TCCR0A = 0;
  TIMSK0 = 1<<TOIE0;
  
  
  //UART INIT
  UART_Init();
  uartIntRx(1);//Habilita a interrupção de recep.

  sei();

 //set_bit(PORTB, PB1);


  
}
int main(){
  setup(); 
  while (1){} //loop de exução
  return 0;
}


ISR(TIMER0_OVF_vect){
  if(atual != INICIAL && atual != AJUSTE_HORA){
    cont++;
    if(cont >= 1){ //1000 / 16,384 = 61,03
      if(segUnidade <= 8){
        segUnidade++;
      } else {
        segUnidade = 0;
        uart_Transmit(65);
      }
      PORTC = Tabela[segUnidade];
      cont = 0;
      
    }
  }
}

ISR(USART_RX_vect){
	uint8_t dado_rx; //Variável para armazenar dado recebido;
	dado_rx = uartRX(); //Armazena o dado;
  	if(dado_rx == 60){
      minUnidade++;
      if(minUnidade > 9){ //passou 10seg
      	minUnidade = 0;
        minDezena++;
        if(minDezena > 5){ //passou 60min
        	minDezena=0;
          	uart_Transmit(70);
        }
        PORTB = Tabela[minDezena];
      }
      PORTD = Tabela[minUnidade];
  	}
    if(dado_rx < 60 && atual != INICIAL){
      int dezena = dado_rx/10;
 	    int unidade = dado_rx%10;
      PORTD = Tabela[unidade];
      PORTB = Tabela[dezena];
    }
  	

}

//Função para inicialização da USART
void UART_Init(void){
	UBRR0H = (uint8_t) (MYUBRR >>8); //Ajusta a taxa de transmissão
	UBRR0L = (uint8_t) (MYUBRR);
	UCSR0A = 0; // Desabilita velocidade dupla.
	UCSR0B = (1 << RXEN0) | (1  << TXEN0); //Habilita o transmissor e o receptor
	UCSR0C = (1 << UCSZ01) | (1 << UCSZ00); // Ajusta o formato do frame:
	//8 bits de dados e 1 bit de parada
}
//Verifica se novo dado pode ser enviado pela UART
//Retorna valor 32 se novo dado pode ser enviado ou Zero caso não
uint8_t uartTxOk(void){
	return (UCSR0A & (1 << UDRE0));
}
//Envia um byte pela porta UART
void uart_Transmit (uint8_t data){
	UDR0 = data;//Coloca o dado no registrador de transmissão e o envia
}

//Envia uma string pela porta UART.
void uartString (char *c){
	for(;*c!=0;c++){
		while(!uartTxOk());
		uart_Transmit(*c);
	}
}
/*Verifica se UART possui novo dado
Retorna valor 128 se existir novo dado recebido. Zero se não
*/
uint8_t uartRxOk(void){
	return (UCSR0A & (1<< RXC0));
}
//Ler byte recebido na porta UART
uint8_t uartRX(){
	return UDR0;
}
/*Habilita ou desabilita a interrupção de recepção da USART
x = 0, desabilita, qualquer outro valor, habilita a interrupção.
*/
void uartIntRx(uint8_t _hab){
	if(_hab){
		UCSR0B |= (1 << RXCIE0); //Habilita a interrupção de recep.
	}else{
		UCSR0B &=~(1 << RXCIE0); //Desabilita a interrupção de recep.
	}
}
/*Habilita ou desabilita a interrupção de transmissão da USART
x = 0, desabilita, qualquer outro valor, habilita a interrupção. 
*/
void uartIntTx(uint8_t _hab){
	if(_hab){
		UCSR0B |= (1 << TXCIE0); //Habilita a interrupção de trans.
		}else{
		UCSR0B &=~(1 << TXCIE0); //Desabilita a interrupção de trans.
	}
}


