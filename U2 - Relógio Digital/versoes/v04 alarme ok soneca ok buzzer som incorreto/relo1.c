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
#define xor_bit(Y,bit_x)(Y^=(1<<bit_x)) // Altera o estado da saída (positivo para negativo e vice-versa)


//Funções ADC
void adcBegin(uint8_t ref, uint8_t did);
void adcChannel(uint8_t canal);
void adcSample(void);
uint8_t adcOk(void);
uint16_t adcReadOnly();
uint16_t adcRead();
void adcIntEn(uint8_t x);

enum estados{INICIAL=1, AJUSTE_HORARIO, RELOGIO, CRIAR_ALARME, TOCAR_ALARME};
estados atual = INICIAL;
int opcao = -1;
bool habAjuste = false;
int horas = 0;
bool confirmar = false; //confirmar ajuste de horario

//ALARME
bool habAlarme = false;
int horaAlarme = 0;
int minAlarme = 0;
bool mesmoDia = true;
bool criando = false;
int tempoAteAlarme = 0;
int contAlarme = 0;
int cont = 0;


//BUZZER
#define CONST 120000
long int aux = 0; //auxiliar para o buzzer

const unsigned char Tabela[] = {0x00, 0x04, 0x08, 0x0C,
                                 0x10, 0x14, 0x18,0x1C,
                                 0x20, 0x24};

const unsigned char TabelaHora[]= {0xC3,0xC7,0xCB,0xCF,0xD3,0xD7,0xDB,0xDF,0xE3,0xE7};
//const unsigned char TabelaSeg[]= {0x81, 0x85, 0x89, 0x8D, 0x91, 0x95, 0x99, 0x9D, 0xA1, 0xA5};
const unsigned char TabelaSeg[]= {0x01, 0x05, 0x09, 0x0D, 0x11, 0x15, 0x19, 0x1D, 0x21, 0x25};

                                 
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
  DDRD = 0x00;  //0011 1100 PD2 até PD5
  DDRB = 0x00; // 0011 1111 PB2 até PB5
  DDRC = 0x00;
  PORTB = TabelaHora[horaDezena];
  PORTD = TabelaHora[horaUnidade];
  PORTC = TabelaSeg[0]; //dezena segundos

  PCICR = (1 << PCIE0) | (1 << PCIE2); //habilita interrupção por mudanças de sinal no PORTB e PORTD

  PCMSK0 = (1 << PCINT0) | (1 << PCINT1); //pinos com interrupção ligada
  PCMSK2 = (1 << PCINT22) | (1 << PCINT23); //pinos com interrupção ligada


  //Temporizador normal do Alarme
  TCCR0B = (1 << CS02) | (1 <<CS00); //prescaller 1024 101
  TCCR0A = 0;
  TIMSK0 = 0<<TOIE0; // interrupção do alarme inicialmente desabilitada


  //Temporizador do buzzer
  TCCR2B = (1 << CS22);
  TCCR2A = (1<<WGM21);
  TIMSK2 = 1 << TOIE2;

  
  
  //ADC
  adcBegin(AVCC, 0x01);   //Inicializa A/D
  adcChannel(ADC0);       //Seleciona canal
  adcIntEn(1);            //Interrupção do A/D
  
  UART_Init();
  uartIntRx(1);//Habilita a interrupção de recep.
  sei();
}


void set_inicial(){
      //uartString("modo set horario");
      confirmar = false;
      //_delay_ms(500);
      while (!confirmar)
      {
        adcSample();  
      }
      opcao = -1; 
}

ISR(TIMER0_OVF_vect){ //temporizador do alarme contAlarme e tempoAteAlarme
  if(atual != INICIAL && atual != CRIAR_ALARME){
    cont++;
    if(cont >= 1*60){ //1000 / 16,384 = 61,03 / contando 1 minuto
      uartString("passou 1min\n");
      contAlarme++;
      if(contAlarme == tempoAteAlarme){
        //uartString("atual = tocar");
        atual = TOCAR_ALARME;
        uart_Transmit(66);
      }
      cont = 0;
    }
  } 
  if(atual == TOCAR_ALARME){ //()
        if(aux<CONST)
          OCR2A=2000;
        else if(aux>=CONST && aux<2*CONST)
          OCR2A=1000;
        else if(aux>=2*CONST && aux<3*CONST)
          OCR2A=500;
        else
          aux=0;
        //_delay_ms(200);
        OCR2A=0;
    }

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
      if(atual == RELOGIO | atual == TOCAR_ALARME){
        PORTC = TabelaSeg[dezenaSegundos];
      }
  	}
  	else if(dado_rx == 70){
      horaUnidade++;
      if(horaUnidade > 9){ //passou 10 horas
      	horaDezena++;
      }
      if(horaUnidade == 4 && horaDezena == 2){
      	horaDezena = 0;
        horaUnidade = 0;
      }
      if(atual == RELOGIO | atual == TOCAR_ALARME){
        PORTD = TabelaHora[horaUnidade];
        PORTB = TabelaHora[horaDezena];
      }
      horas = horaDezena*10 + horaUnidade;
  	}
    else if(dado_rx >=180 && dado_rx <= 239 && criando == true){ //agora temos horas e minutos do relógio e alarme
      criando = false;
      dado_rx = dado_rx-180;
      if(horaAlarme < horas | (horaAlarme == horas && minAlarme < dado_rx)){
        mesmoDia = false;
      } else {
        mesmoDia = true;
      }
      if(mesmoDia){
        tempoAteAlarme = (horaAlarme - horas)*60 + (minAlarme - dado_rx);
      } else {
        tempoAteAlarme = ((24 - horas)*60 + 60-dado_rx) + horaAlarme*60+minAlarme;
      }
      uartString("Min pro alarme: ");
      uartDec2B(tempoAteAlarme);
      //uartString("\n");

      TIMSK0 = 1<<TOIE0; //habilita a interrupção do alarme

    }
}
uint16_t valor;
//Tratamendo da interrupção do A/D
void uartTx(uint8_t dado)
{ UDR0 = dado;//envia dado
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

ISR(ADC_vect)
{  
  if(atual == AJUSTE_HORARIO){
    if(opcao == 0) { // mudar minutos
      valor = adcReadOnly();
      uint16_t minutos = valor/17;
      if(minutos >= 60){
          minutos = 59;
      }
      uart_Transmit(minutos+100);
    } else if (opcao == 1){ //mudar hora
      valor = adcReadOnly();
      uint16_t horasConv = valor/44;  // 1022/23 = 44,43  
      //uartDec2B(horasConv); 
      //uartString("\r\n");  

      if(horasConv >= 24){
          horasConv = 23;
      }
      horas = horasConv;
      horaDezena = horasConv/10;
      horaUnidade = horasConv%10;
      PORTD = TabelaHora[horaUnidade];
      PORTB = TabelaHora[horaDezena];
    }
  } else if (atual == CRIAR_ALARME){
    if(opcao == 0) { // mudar minutos
      valor = adcReadOnly();
      uint16_t minutos = valor/17; //1022/59 = 17
      if(minutos >= 60){
          minutos = 59;
      }
      minAlarme = minutos;
      uart_Transmit(minutos+100);
    } else if (opcao == 1){ //mudar hora
      valor = adcReadOnly();
      uint16_t horasConv = valor/44;    //    1022/23 = 44,43
      //uartDec2B(horasConv); 
      //uartString("\r\n");  

      if(horasConv >= 24){
          horasConv = 23;
      }
      horaAlarme = horasConv;
      int i = horasConv%10;
      int j = horasConv/10;
      PORTD = TabelaHora[i];
      PORTB = TabelaHora[j];
    }
  }
 
 	
}

ISR(PCINT2_vect) //interrupções nos botões
{
  if (!(tst_bit(PIND, BOTAO_HORA_SONECA))) { 
    if(atual == AJUSTE_HORARIO | atual == CRIAR_ALARME){
        opcao = 1;
      } else if(atual == TOCAR_ALARME){
        contAlarme = 0;
        cont = 0;
        tempoAteAlarme = 5;
        atual = RELOGIO;
        aux=0;
        OCR2A=0;
        uart_Transmit(30); //voltar o modo relógio
        uartString("soneca 5min\n");

      }
  } 
  else if (!(tst_bit(PIND, BOTAO_MINUTO))){
    if(atual == INICIAL | atual == RELOGIO){
      habAjuste = true;
      atual = AJUSTE_HORARIO;
      uart_Transmit(40);
    }
    if(atual == AJUSTE_HORARIO | atual == CRIAR_ALARME){
      opcao = 0;
    }
  } 
}

ISR(PCINT0_vect) //interrupções nos botões
{
  if (!(tst_bit(PINB, BOTAO_CONFIRMAR))){
     if(atual == AJUSTE_HORARIO){
          confirmar = true;
          habAjuste = false;
          atual = RELOGIO; //terminei de editar o horário
          uart_Transmit(30);
          
    } else if(atual == TOCAR_ALARME){
          atual = RELOGIO;
          aux=0;
          OCR2A=0;
          uart_Transmit(30); //voltar o modo relógio
          TIMSK0 = 0<<TOIE0; //desabilita a interrupção do alarme

    } else if (atual == CRIAR_ALARME){
          habAlarme = true;
          atual = RELOGIO;
          uart_Transmit(30); //voltar o modo relógio
          PORTD = TabelaHora[horaUnidade];
          PORTB = TabelaHora[horaDezena];
          criando = true;
          //solicitar minutos do outro arduino
          uart_Transmit(10); 



    }
  } else if (!(tst_bit(PINB, BOTAO_ALARME))) {
        if(atual == RELOGIO){ //começa a criar alarme
          atual = CRIAR_ALARME;
          habAlarme = false;
          uart_Transmit(15);
          TIMSK0 = 0<<TOIE0; //desabilita a interrupção do alarme

        } else if(atual == CRIAR_ALARME){ //cancela criação do alarme
          atual = RELOGIO;
          uart_Transmit(30); //voltar o modo relógio
          PORTD = TabelaHora[horaUnidade];
          PORTB = TabelaHora[horaDezena];
        }

  }


}

ISR(TIMER2_OVF_vect){
    //Inverte o sinal de saida de PC1
    xor_bit(PORTC,1);
  	//Variando aux
    aux += OCR2A;
}

int main(){
  setup(); 
  while (1){
    if(atual == CRIAR_ALARME | atual == AJUSTE_HORARIO)
  		set_inicial();
  }; //loop de exução
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
