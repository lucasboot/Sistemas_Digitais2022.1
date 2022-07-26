//Configurações
#define F_CPU 16000000UL
#define BAUD 9600
#define MYUBRR F_CPU/16/BAUD-1

//Bibliotecas
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>


//Assinatura das funções de comunicação
void UART_Init(void);
uint8_t uartTxOk(void);
void uart_Transmit (uint8_t data);
void uartString (char *c);
uint8_t uartRxOk(void);
uint8_t uartRX();
void uartIntRx(uint8_t _hab);
void uartIntTx(uint8_t _hab);
void uartTx(uint8_t dado);
void uartDec2B(uint16_t valor);

//Macros
#define set_bit(Y, bit_x) (Y |= (1 << bit_x)) //ligar os LEDs
#define clr_bit(Y, bit_x) (Y &= ~(1 << bit_x)) //desligar os LEDs
#define tst_bit(Y, bit_x) (Y & (1 << bit_x)) //testar se os botões foram pressionados
#define clp_bit(Y, bit_x) (Y ^= (1 << bit_x)) //Muda o estado da porta


//Estados, so tem 2 que nao tem transicao vazia, então 
//so precisa de um bool funcionando OU mudando_temp
bool mudando_temp = false;
unsigned int temperatura = 25; //comeca setado em 25°C


const unsigned char display[]= {0xC3,0xC7,0xCB,0xCF,0xD3,0xD7,0xDB,0xDF,0xE3,0xE7};

const unsigned char Tabela[] = {0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09};

void setup(){
  	DDRB = 0x3F;
    DDRD = 0x00;
  	PORTB = Tabela[temperatura%10];
    PORTD = display[temperatura/10];

    //UART INIT
    UART_Init();
    uartIntRx(1);//Habilita a interrupção de recep.
    sei();
}


int main(){
	setup();
	while(1);
	return 0;
}

ISR(USART_RX_vect){
    //uartString("ola");
	uint8_t dado_rx; //Variável para armazenar dado recebido;
	dado_rx = uartRX(); //Armazena o dado;
    if(dado_rx >= 16 && dado_rx <=30){
        temperatura = dado_rx;
        PORTB = Tabela[temperatura%10];
        PORTD = display[temperatura/10];
    }

}







//Biblioteca UART
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
void uartTx(uint8_t dado)
{ UDR0 = dado;//envia dado
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

void uartDec2B(uint16_t valor)
{ int8_t disp;
	char digitos[5];
	int8_t conta = 0;
	do //converte o valor armazenando os algarismos no vetor digitos
	{ disp = (valor%10) + 48;//armazena o resto da divisao por 10 e soma com 48
		valor /= 10;
		digitos[conta]=disp;
		conta++;
	} while (valor!=0);
	for (disp=conta-1; disp>=0; disp-- )//envia valores do vetor digitos
	{ while (!uartTxOk());	 //aguarda último dado ser enviado
		uartTx(digitos[disp]);//envia algarismo
	}
}