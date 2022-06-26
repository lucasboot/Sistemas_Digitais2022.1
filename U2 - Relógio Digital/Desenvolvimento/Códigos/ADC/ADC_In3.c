#include <avr/io.h>
#include <avr/delay.h>
#include <avr/interrupt.h>
#define F_CPU 16000000
#define AREF 0 // Tensão de referência  = Aref
#define AVCC 1 // Tensão de referência  = AVcc
#define VR11 2 // Tensão de referência  = 1,1 V
#define ADC0 0 // Seleciona a entrada ADC0
#define ADC1 1 // Seleciona a entrada ADC1
#define ADC2 2 // Seleciona a entrada ADC2
#define ADC3 3 // Seleciona a entrada ADC3
#define ADC4 4 // Seleciona a entrada ADC4
#define ADC5 5 // Seleciona a entrada ADC5
#define Vtemp 6// Seleciona o Sensor de Temperatura
#define V11 7 //  Seleciona a tensão de 1,1 V
#define Vgnd 8 // Seleciona a tensão GND (0 V)
#define BAUD 9600
#define MYUBRR F_CPU/16/BAUD-1


//---------------------------------------------------------------------------
//Configura o conversor ADC do ATmega328p
//ref = 0. Para usar a tensão de referência Aref
//ref = 1. Para usar a tensão de referência Avcc - Lembre-se do Capacitor 100nF
//ref = 2. Para usar a tensão de referência interna de 1,1 V
//did: valor para o registrador DIDR0
//---------------------------------------------------------------------------
void adcBegin(uint8_t ref, uint8_t did)
{ ADCSRA = 0;     //configuração inicial
	ADCSRB = 0;   //configuração inicial
	DIDR0  = did; //configuração DIDR0
	if (ref == 0)
	{	 ADMUX &= ~((1<<REFS1) | (1<<REFS0));//Aref
	}
	if ((ref == 1) || (ref > 2))
	{ ADMUX &= ~(1<<REFS1);   //Avcc
		ADMUX |=  (1<<REFS0); //Avcc
	}
	if (ref == 2)
	{ ADMUX |= (1<<REFS1) | (1<<REFS0);  //Tensão interna de ref (1.1V)
	}
	ADMUX &= ~(1<<ADLAR); //Alinhamento a direita
	ADCSRA |= (1<<ADEN)|(1<<ADPS2)|(1<<ADPS1)|(1<<ADPS0);//habilita AD. Prescaler de 128 (clk_AD = F_CPU/128)
}

//---------------------------------------------------------------------------
//Seleciona canal do ADC
//0 <= channel <= 5 - Leitura dos pinos AD0 a AD5
//channel = 6 - Leitura do sensor de temperatura
//channel = 7 - 1,1V
//channel > 7 - GND
//---------------------------------------------------------------------------
void adcChannel(uint8_t canal)
{ if (canal <= 5)//seleciona um canal no multiplex
	ADMUX = (ADMUX & 0xF0) | canal;
	if (canal == 6)//seleciona sensor interno de temperatura
	ADMUX = (ADMUX & 0xF0) | 0x08;
	if (canal == 7)//seleciona 1,1 V
	ADMUX = (ADMUX & 0xF0) | 0x0E;
	if (canal > 7)//seleciona GND
	ADMUX = (ADMUX & 0xF0) | 0x0F;
}

//---------------------------------------------------------------------------
//Inicia conversão
//---------------------------------------------------------------------------
void adcSample(void)
{ ADCSRA |= (1<<ADSC);//Inicia conversão
}

//---------------------------------------------------------------------------
//Verifica se conversão foi concluída
//Retorna valor 0 se conversão concluída. 64 se não.
//---------------------------------------------------------------------------
uint8_t adcOk(void)
{ return (ADCSRA & (1<<ADSC));
}

//---------------------------------------------------------------------------
//Ler o ADC e retorna o valor lido do ADC
//---------------------------------------------------------------------------
uint16_t adcReadOnly()
{ return ((ADCH<<8) |ADCL );//retorna o valor do ADC
}

//---------------------------------------------------------------------------
//Converte, aguarda, ler e retorna valor lido do ADC
//---------------------------------------------------------------------------
uint16_t adcRead()
{ adcSample();         //Inicia conversão
	while(adcOk());      //Aguarda fim da conversão (ADSC = 0)
	return adcReadOnly();//retorna o valor do ADC
}
//---------------------------------------------------------------------------
//Habilita ou desabilita interrupção do ADC
//Se x = 0, desabilita interrupção
//Caso contrário, habilita interrupção
//---------------------------------------------------------------------------
void adcIntEn(uint8_t x)
{ if (x)
	ADCSRA |= (1<<ADIE);//habilita interrupção do ADC
	else
	ADCSRA &= ~(1<<ADIE);//Desabilita interrupção do ADC
}


//---------------------------------------------------------------------------
//inicializa a porta de comunicação uart do ATmega328
//---------------------------------------------------------------------------

void UART_Init(void){
	UBRR0H = (uint8_t) (MYUBRR >>8); //Ajusta a taxa de transmissão
	UBRR0L = (uint8_t) (MYUBRR);
	UCSR0A = 0; // Desabilita velocidade dupla.
	UCSR0B = (1 << RXEN0) | (1  << TXEN0); //Habilita o transmissor e o receptor
	UCSR0C = (1 << UCSZ01) | (1 << UCSZ00); // Ajusta o formato do frame:
	//8 bits de dados e 1 bit de parada
}


//---------------------------------------------------------------------------
//verifica se novo dado pode ser enviado pela UART
//---------------------------------------------------------------------------
uint8_t uartTxOk (void)
{ return (UCSR0A & (1<<UDRE0));//retorna valor 32 se novo dado pode ser enviado. Zero se não.
}

//---------------------------------------------------------------------------
//Envia um byte pela porta uart
//---------------------------------------------------------------------------
void uartTx(uint8_t dado)
{ UDR0 = dado;//envia dado
}

//---------------------------------------------------------------------------
//verifica se UART possui novo dado
//retorna valor 128 se existir novo dado recebido. Zero se não.
//---------------------------------------------------------------------------
uint8_t uartRxOk (void)
{ return (UCSR0A & (1<<RXC0));
}

//---------------------------------------------------------------------------
//Ler byte recebido na porta uart
//---------------------------------------------------------------------------
uint8_t uartRx()
{ return UDR0; //retorna o dado recebido
}

//---------------------------------------------------------------------------
//Habilita ou desabilita interrupção de recepção da usart
//x = 0, desabilita interrupção. x = 1, habilita interrupção
//---------------------------------------------------------------------------
void uartIntRx(uint8_t x)
{ if (x)
	UCSR0B |= (1<<RXCIE0);//Habilita interrupção de recepção de dados
	else
	UCSR0B &= ~(1<<RXCIE0);//Desabilita interrupção de recepção de dados
}

//---------------------------------------------------------------------------
//Habilita ou desabilita interrupção de Trasnmissão da usart
//x = 0, desabilita interrupção. x = 1, habilita interrupção
//---------------------------------------------------------------------------
void uartIntTx(uint8_t x)
{ if (x)
	UCSR0B |= (1<<TXCIE0);//Habilita interrupção de recepção de dados
	else
	UCSR0B &= ~(1<<TXCIE0);//Desabilita interrupção de recepção de dados
}

//---------------------------------------------------------------------------
//Envia uma string pela porta uart. Ultimo valor da string deve ser 0.
//---------------------------------------------------------------------------
void uartString(char *c)
{ for (; *c!=0; c++)
	{ while (!uartTxOk());	//aguarda último dado ser enviado
		uartTx(*c);
	}
}

//---------------------------------------------------------------------------
//Envia pela uart variavel de 1 byte (8 bits) com digitos em decimal
//---------------------------------------------------------------------------
void uartDec1B(uint8_t valor)
{ int8_t disp;
	char digitos[3];
	int8_t conta = 0;
	do //converte o valor armazenando os algarismos no vetor digitos
	{ disp = (valor%10) + 48;//armazena o resto da divisao por 10 e soma com 48
		valor /= 10;
		digitos[conta]=disp;
		conta++;
	} while (valor!=0);
	for (disp=conta-1; disp>=0; disp-- )//envia valores do vetor digitos
	{ while (!uartTxOk());  //aguarda último dado ser enviado
		uartTx(digitos[disp]);//envia algarismo
	}
}

//---------------------------------------------------------------------------
//Envia pela uart variavel de 2 bytes (16 bits) com digitos em decimal
//---------------------------------------------------------------------------
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

//---------------------------------------------------------------------------
//Envia pela uart variavel de 4 bytes (32 bits) com digitos em decimal
//---------------------------------------------------------------------------
void uartDec4B(uint32_t valor)
{ int8_t disp;
	char digitos[10];
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

//---------------------------------------------------------------------------
//Envia pela uart variavel de 1 byte (8 bits) com digitos em hexadecimal
//---------------------------------------------------------------------------
void uartHex1B(uint8_t valor)
{ uint8_t disp;
	disp = (valor%16) + 48;    //armazena o resto da divisao por 16 e soma com 48
	if (disp > 57) disp += 7;  //soma 7 se algarismo for uma letra devido tabela ascii
	valor = (valor / 16) + 48; //armazena o resto da divisao por 16 e soma com 48
	if (valor > 57) valor += 7;//soma 7 se algarismo for uma letra devido tabela ascii
	while (!uartTxOk());	     //aguarda último dado ser enviado
	uartTx(valor);             //envia digito mais significativo
	while (!uartTxOk());	     //aguarda último dado ser enviado
	uartTx(disp);             //envia digito menos significativo
}

//---------------------------------------------------------------------------
//Envia pela uart variavel de 2 bytes (16 bits) com digitos em hexadecimal
//---------------------------------------------------------------------------
void uartHex2B(uint16_t valor)
{ uint8_t disp0;
	uint8_t disp1;
	disp0 = (uint8_t) (valor & 0x00FF);//armazena byte menos significativo
	disp1 = (uint8_t) (valor >> 8);    //armazena byte mais significativo
	uartHex1B(disp1);               //envia byte mais significativo
	uartHex1B(disp0);               //envia byte menos significativo
}

//---------------------------------------------------------------------------
//Envia pela uart variavel de 4 bytes (32 bits) com digitos em hexadecimal
//---------------------------------------------------------------------------
void uartHex4B(uint32_t valor)
{ uint16_t disp0;
	uint16_t disp1;
	disp0 = (uint16_t) (valor & 0x0000FFFF);//armazena dois bytes menos significativos
	disp1 = (uint16_t) (valor >> 16);       //armazena dois bytes mais significativos
	uartHex2B(disp1);                    //envia dois bytes mais significativos
	uartHex2B(disp0);                    //envia dois bytes menos significativos
}



int main(void)
{	UART_Init(); //Inicializa UART
	adcBegin(AVCC, 0x01);   //Inicializa A/D
	adcChannel(ADC0);       //Seleciona canal
	adcIntEn(1);            //Interrupção do A/D
	uartIntRx(1);           //Interrupção da uart
	sei();                  //Interrupção geral
	while (1);
}

//Tratamendo da interrupção de recepção de dados
ISR(USART_RX_vect)
{  switch (uartRx())
{
case 'c':
case 'C':
adcSample();      //Inicia conversão
	break;
}
	
}

//Tratamendo da interrupção do A/D
ISR(ADC_vect)
{ uartString("Valor: ");    //Envia string
	uartDec2B(adcReadOnly()); //Ler e envia valor do A/D
	uartString("\r\n");       //Nova linha
}
