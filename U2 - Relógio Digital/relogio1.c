//Configurações
#define F_CPU 16000000UL
#define BAUD 9600
#define MYUBRR F_CPU/16/BAUD-1

//Bibliotecas
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

//ADC
#define AREF 0 // Tensão de referência  = Aref
#define AVCC 1 // Tensão de referência  = AVcc
#define VR11 2 // Tensão de referência  = 1,1 V
#define ADC0 0 // Seleciona a entrada ADC0
#define Vtemp 6// Seleciona o Sensor de Temperatura
#define V11 7 //  Seleciona a tensão de 1,1 V
#define Vgnd 8 // Seleciona a tensão GND (0 V)

//BOTÕES
#define BOTAO_HORA_SONECA PD6
#define BOTAO_MINUTO PD7
#define BOTAO_CONFIRMAR PB0
#define BOTAO_ALARME PB1

//MACROS
#define set_bit(Y, bit_x) (Y |= (1 << bit_x)) //ligar os LEDs
#define clr_bit(Y, bit_x) (Y &= ~(1 << bit_x)) //desligar os LEDs
#define tst_bit(Y, bit_x) (Y & (1 << bit_x)) //testar se os botões foram pressionados

//Funções ADC
void adcBegin(uint8_t ref, uint8_t did);
void adcChannel(uint8_t canal);
void adcSample(void);
uint8_t adcOk(void);
uint16_t adcReadOnly();
uint16_t adcRead();
void adcIntEn(uint8_t x);

enum estados{INICIAL=1, AJUSTE_HORA, RELOGIO, CRIAR_ALARME, TOCAR_ALARME};
estados atual = INICIAL;
int opcao = 0;


const unsigned char Tabela[] = {0x00, 0x04, 0x08, 0x0C,
                                 0x10, 0x14, 0x18,0x1C,
                                 0x20, 0x24};
int dezenaSegundos = 0;
int horaUnidade = 0;
int horaDezena = 0;
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
  DDRD = 0x3C;  //0011 1100 PD2 até PD5
  DDRB = 0x3C; // 0011 1111 PB2 até PB5
  PORTB |= Tabela[horaDezena];
  PORTD |= Tabela[horaUnidade];

  //Botões
  set_bit(PORTB, BOTAO_CONFIRMAR);
  set_bit(PORTB, BOTAO_ALARME);
  set_bit(PORTD, BOTAO_MINUTO);
  set_bit(PORTD, BOTAO_HORA_SONECA);
  PCICR = (1 << PCIE0) | (1 << PCIE2); //habilita interrupção por mudanças de sinal no PORTB e PORTD

  PCMSK0 = (1 << PCINT0) | (1 << PCINT1); //pinos com interrupção ligada
  PCMSK2 = (1 << PCINT22) | (1 << PCINT23); //pinos com interrupção ligada


  DDRC = 0x3F;
  PORTC = Tabela[0];
  
  //ADC
  adcBegin(AVCC, 0x01);   //Inicializa A/D
  adcChannel(ADC0);       //Seleciona canal
  adcIntEn(1);            //Interrupção do A/D
  
  UART_Init();
  uartIntRx(1);//Habilita a interrupção de recep.
  sei();
}

void set_inicial(){
      uartString("modo set horario");
      bool confirmar = false;
      _delay_ms(500);
      while (!confirmar)
      {
        
        if (!(tst_bit(PIND, BOTAO_MINUTO))){
            opcao = 0;
        }
        if (!(tst_bit(PIND, BOTAO_HORA_SONECA))){
            opcao = 1;
            
        }
        if(!(tst_bit(PINB, BOTAO_CONFIRMAR))){
          confirmar = true;
        }
        
      }
      opcao = -1; 
}
//interrupção de recepcao
ISR(USART_RX_vect){
	uint8_t dado_rx; //Variável para armazenar dado recebido;
	dado_rx = uartRX(); //Armazena o dado;
  	if(dado_rx == 65){
      dezenaSegundos++;
      if(dezenaSegundos == 6){ //1 minuto
        dezenaSegundos = 0;
        uart_Transmit(60); //passou 1 minuto
      }
      PORTC = Tabela[dezenaSegundos];
  	}
  	if(dado_rx == 70){
      horaUnidade++;
      if(horaUnidade > 9){ //passou 10 horas
      	horaDezena++;
      }
      if(horaUnidade == 4 && horaDezena == 2){
      	horaDezena = 0;
        horaUnidade = 0;
      }
      PORTD |= Tabela[horaUnidade];
      PORTB |= Tabela[horaDezena];
  	}
}
uint16_t parte;
uint16_t valor;
//Tratamendo da interrupção do A/D
ISR(ADC_vect)
{ 	
  uartString("chamada interup adc");
  if(opcao == 0) { // mudar minutos
    parte = 17;
    valor = adcReadOnly();
    uint16_t minutos = valor/parte;
    if(minutos >= 60){
        minutos = 59;
    }
    uart_Transmit(minutos);
  } else if (opcao == 1){ //mudar hora
    parte = 44; //    1022/23 = 44,43
    valor = adcReadOnly();
    uint16_t horas = valor/parte;
    if(horas >= 24){
        horas = 23;
    }
    PORTD |= Tabela[horas%10];
    PORTB |= Tabela[horas/10];
  }
 
 	
}

ISR(PCINT2_vect) //interrupções nos botões
{
  if (!(tst_bit(PIND, BOTAO_HORA_SONECA))) { 

  } 
  else if (!(tst_bit(PIND, BOTAO_MINUTO))){
    if(atual == INICIAL){
      set_inicial();
    }

  } 
}

ISR(PCINT0_vect) //interrupções nos botões
{/*
  if (!(tst_bit(PINB, BOTAO_CONFIRMAR))){

  } else if (!(tst_bit(PINB, BOTAO_ALARME))) {
  }*/


}

int main(){
  setup(); 
  adcSample();
  while (1); //loop de exução
  return 0;
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

//*********ADC******************

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
