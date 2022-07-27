//Configurações
#define F_CPU 16000000UL
#define BAUD 9600
#define MYUBRR F_CPU/16/BAUD-1

//Bibliotecas
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

//BOTÕES
#define BOTAO_CONFIRMAR PB3
#define BOTAO_MUDARTEMP PB4
#define BOTAO_MUDARMODO PB5

//Conversão ADC
#define AREF 0 // Tensão de referência  = Aref
#define AVCC 1 // Tensão de referência  = AVcc
#define VR11 2 // Tensão de referência  = 1,1 V
#define ADC0 0 // Seleciona a entrada ADC0
#define ADC1 1 //seleciona a entrada ADC1
#define Vtemp 6// Seleciona o Sensor de Temperatura
#define V11 7 //  Seleciona a tensão de 1,1 V
#define Vgnd 8 // Seleciona a tensão GND (0 V)


//Macros
#define set_bit(Y, bit_x) (Y |= (1 << bit_x)) //ligar os LEDs
#define clr_bit(Y, bit_x) (Y &= ~(1 << bit_x)) //desligar os LEDs
#define tst_bit(Y, bit_x) (Y & (1 << bit_x)) //testar se os botões foram pressionados
#define clp_bit(Y, bit_x) (Y ^= (1 << bit_x)) //Muda o estado da porta


//Estados, so tem 2 que nao tem transicao vazia, então 
//so precisa de um bool funcionando OU mudando_temp
bool mudando_temp = false;
unsigned int cont = 0;


//Modos
const unsigned char azul = 0x3A; //010
const unsigned char vermelho = 0x3C;//100
const unsigned char verde = 0x39; //001

//Variaveis importantes
unsigned int indiceModo = 0; // 0 -> COOL, 1-> FAN, 2->AUTO
const unsigned char Modos[]={azul, vermelho, verde};
//Temperatura selecionada
unsigned int temperatura = 25; //comeca setado em 25°C
unsigned int temperaturaReal;


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

//Assinatura das funções de conversão ADC
void adcBegin(uint8_t ref, uint8_t did);
void adcChannel(uint8_t canal);
void adcSample(void);
uint8_t adcOk(void);
uint16_t adcReadOnly();
uint16_t adcRead();
void adcIntEn(uint8_t x);


void setup(){
  	DDRB = 0x6F;
    
    DDRD = 0x00;
    PORTD = 0x3F; 
  	PORTB = Modos[indiceModo]; //começa azul

    //Interrupção botões
    PCICR = (1 << PCIE0) ; //habilita interrupção por mudanças de sinal no PORTB e PORTD
    PCMSK0 = (1 << PCINT3) | (1 << PCINT4) | (1 << PCINT5);


    //UART INIT
    UART_Init();
    //uartIntRx(1);//Habilita a interrupção de recep.

    //Temporizador modo normal
    TCCR0B = (1 << CS02) | (1 <<CS00); //prescaller 1024 101
    TCCR0A = 0;
    TIMSK0 = 0<<TOIE0; //temporizador inicia desligado, só é ativado no modo AUTO
  
  

    //Configs ADC
    adcBegin(AVCC, 0x01);   //Inicializa A/D
	//adcChannel(ADC0);       //Seleciona canal
	adcIntEn(1);            //Interrupção do A/D

    sei();
}


int main(){
	setup();
	while(1){
        adcSample();
    };
	return 0;
}
//menu: m/M > trocar modo | t/T > ajustar temperatura
ISR(USART_RX_vect){
	uint8_t dado_rx; //Variável para armazenar dado recebido;
	dado_rx = uartRX(); //Armazena o dado;

}

ISR(PCINT0_vect) //interrupção do botão, confirmar temperatura
{
     //uartString("oi");
    if (!(tst_bit(PINB, BOTAO_CONFIRMAR))) { 
        if(mudando_temp)
            mudando_temp=false;
    } else if (!(tst_bit(PINB, BOTAO_MUDARTEMP))) {
         mudando_temp = true;
    } else if (!(tst_bit(PINB, BOTAO_MUDARMODO))) {
        uartString("oi");
        if(!mudando_temp){
            indiceModo++;
            if(indiceModo == 2) //modo AUTO
                TIMSK0 = 1<<TOIE0;
            else
                TIMSK0 = 0<<TOIE0;

            if(indiceModo>2)
                indiceModo = 0;
            PORTB = Modos[indiceModo]; 
        }
    }

}

uint16_t parte = 68; //1023/15 = 68
//Tratamendo da interrupção do A/D
ISR(ADC_vect)
{ 
    if(mudando_temp){
        adcChannel(ADC0);       //Seleciona canal
        uint16_t valor = adcReadOnly();
        uint16_t ntemperatura = valor/parte + 16;
        if(ntemperatura > 30)
            ntemperatura = 30;
        uart_Transmit(ntemperatura);
        //Transmitir para o outro arduino o valor
        //uart_Transmit(ntemperatura);
    
    } else { //sensor de temperatura
        adcChannel(ADC1);       //Seleciona canal
        //uartString("Temperatura: ");    //Envia string
        uint16_t valor = adcReadOnly();
        uint16_t ntemperatura = (valor*(5000 / 1024.0)-500)/10; //fórmula da conversão
        temperaturaReal = ntemperatura;
      	/*
      	uint16_t dezena = temperatura/10;
 	    uint16_t unidade = temperatura%10;
        //uartDec2B(temperatura); //Ler e envia valor do A/D
        uartDec2B(dezena); //Ler e envia valor do A/D
 	    uartDec2B(unidade); //Ler e envia valor do A/D
        uartString("\n"); 
        */
    }

}

ISR(TIMER0_OVF_vect){
    cont++;
    if(cont >= 61){ //pasou 1min
        cont = 0;
        //uartString("alou");
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







//Biblioteca ADC

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
{ 
  ADCSRA |= (1<<ADSC);//Inicia conversão
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
//Habilita ou desabilita interrupção do ADC
//Se x = 0, desabilita interrupção
//Caso contrário, habilita interrupção
//---------------------------------------------------------------------------
