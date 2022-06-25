#define F_CPU 16000000UL
#define BAUD 9600
#define MYUBRR F_CPU/16/BAUD-1
#include <avr/io.h>
#include <avr/interrupt.h>

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

int main(){
	uint8_t dado_rx; //Variável para armazernar dado recebido.
	DDRB = 0x20; // Pinos PB configurados como entrada, exceto PB5
	PORTB = 0x20; //PB5 em alto
	UART_Init();
	uartString("Digite L ou D.\r \n");
	while(1){
		if(uartRxOk()){ // Verifica se existe novo dado
			dado_rx = uartRX(); //Armazena dado
			switch(dado_rx){//Testa o valor Lido
				case 'L':
				case 'l':
				uartString("Ligar LED. \r\n");
				PORTB |= 1 << PB5; //Liga LED
				break;
				case 'D':
				case 'd':
				uartString("Desliga LED.\r\n");
				PORTB &= ~(1 << PB5); //Desliga LED
				break;				
			}
		}
	}	
}

