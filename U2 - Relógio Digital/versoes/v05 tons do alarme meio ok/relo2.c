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


int cont = 0;
int cont2 = 0;
int segUnidade = 0;
int minUnidade = 0;
int minDezena = 0;

int minutos=0;


enum estados{INICIAL=1, AJUSTE_HORARIO, RELOGIO, CRIAR_ALARME, TOCAR_ALARME};
estados atual = INICIAL;

const unsigned char Tabela[] = {0x00, 0x04, 0x08, 0x0C,
                                 0x10, 0x14, 0x18,0x1C,
                                 0x20, 0x24};
const unsigned char TabelaHora[]= {0xC3,0xC7,0xCB,0xCF,0xD3,0xD7,0xDB,0xDF,0xE3,0xE7};


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
  DDRD = 0x00;  //1111 1100 PD2 até PD7
  DDRB = 0x00; // 1111 1111 PB0 até PB5
  DDRC = 0x00;
  
  PORTB = Tabela[minDezena]; //dez minutos
  PORTD = TabelaHora[0]; //uni minutos
  PORTC = Tabela[0]; //uni segundos
  
  //temporizador modo normal
  TCCR0B = (1 << CS02) | (1 <<CS00); //prescaller 1024 101
  TCCR0A = 0;
  TIMSK0 = 1<<TOIE0; //habilita a interrupção do temporizador
  
  
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
  if(atual != INICIAL){
    cont++;
    if(cont >= 1){ //1000 / 16,384 = 61,03
      if(segUnidade <= 8){
        segUnidade++;
      } else {
        segUnidade = 0;
        uart_Transmit(65);
      }
      if(atual == RELOGIO | atual == TOCAR_ALARME){
        PORTC = Tabela[segUnidade];
      }
      cont = 0;
    }
  } else {
    cont2++;
    if(cont2 >= 10){
      clp_bit(PORTD, PD6);
      cont2 = 0;
    }
      
    }

  }


ISR(USART_RX_vect){
	uint8_t dado_rx; //Variável para armazenar dado recebido;
	dado_rx = uartRX(); //Armazena o dado;
  if(dado_rx == 60){
      minUnidade++;
      if(minUnidade > 9){ //passou 10min
      	minUnidade = 0;
        minDezena++;
        if(minDezena > 5){ //passou 60min
        	minDezena=0;
          uart_Transmit(70);
        }
        minutos = minDezena*10 + minUnidade;
        if(atual == RELOGIO | atual == TOCAR_ALARME)
          PORTB = Tabela[minDezena];
      }
      if(atual == RELOGIO | atual == TOCAR_ALARME)
        PORTD = TabelaHora[minUnidade];
  	} else if(dado_rx >=100 && dado_rx <= 159 && (atual == AJUSTE_HORARIO | atual == CRIAR_ALARME)){ //potenciometro minutos
        if(atual == AJUSTE_HORARIO){
          dado_rx = dado_rx - 100;
          minutos = dado_rx;
          minDezena = dado_rx/10;
          minUnidade = dado_rx%10;
          PORTD = TabelaHora[minUnidade];
          PORTB = Tabela[minDezena];
        } else if (atual == CRIAR_ALARME){
          dado_rx = dado_rx - 100;
          int i = dado_rx%10;
          int j = dado_rx/10;
          PORTD = TabelaHora[i];
          PORTB = Tabela[j];
        }
    }
    else if(dado_rx == 40 && (atual == RELOGIO | atual == INICIAL)){
      atual = AJUSTE_HORARIO;
    }
    else if(dado_rx == 30){
      atual = RELOGIO;
      PORTD = TabelaHora[minUnidade];
      PORTB = Tabela[minDezena];
      
    } else if (dado_rx == 15){
      atual = CRIAR_ALARME;
    } else if(dado_rx == 10){ //enviando minutos do relógio para o arduino 1
      int envio = (10*minDezena+minUnidade) + 180;
      uart_Transmit(envio);
    } else if(dado_rx = 66){
      atual = TOCAR_ALARME;
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


