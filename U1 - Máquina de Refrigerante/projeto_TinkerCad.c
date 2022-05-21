
#define F_CPU 160000000UL
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#define set_bit(Y, bit_x) (Y |= (1 << bit_x)) //ligar os LEDs
#define clr_bit(Y, bit_x) (Y &= ~(1 << bit_x)) //desligar os LEDs
#define tst_bit(Y, bit_x) (Y & (1 << bit_x)) //testar se os botões foram pressionados

//Definir um nome significativo para cada pino utilizado
//LEDs
#define AGUA PB5
#define COCA PB4
#define FANTA PB3
#define SPRITE PB2
#define GUARANA PB1
#define TROCO PB0

//BOTÕES
#define BOTAO_PRESENCA PC2
#define BOTAO_50c PC5
#define BOTAO_1r PC4
#define BOTAO_SELECAO PC0
#define BOTAO_CONFIRMAR PC1

//Declarar o uso de interrupções PCINT1
ISR(PCINT1_vect);
//Variáveis de controle do código
bool colocandoMoeda = false; //estado de inserção de moedas
float dinheiro = 0; //valor inserido na máquina
int credito = 0; //conversão do valor na máquina em crédito: 1 crédito -> R$0,50
int selecao = 0; //indicar qual bebida foi escolhida pelo cliente

const unsigned char Bebidas[] = {AGUA, COCA, FANTA, SPRITE, GUARANA};
const unsigned char Tabela[] = {0x3F, 0x06, 0x5B, 0x4F,
                                0x66, 0x6D, 0x7D, 0x07,
                                0x7F, 0x67, 0x77, 0x7C,
                                0x39, 0x5E, 0x79, 0x71};

void piscarLed(int index)
{
  for (int i = 0; i < 3; i++)
  {
    set_bit(PORTB, Bebidas[index]);
    _delay_ms(500);
    clr_bit(PORTB, Bebidas[index]);
    _delay_ms(500);
  }
}

void recebendoTroco(int tempo){
  set_bit(PORTB, TROCO);
  _delay_ms(tempo);
  clr_bit(PORTB, TROCO);
}

void darTroco (){
  for (int i = credito; i >= 0; i--){
     PORTD = Tabela[i];
     _delay_ms(500);
  }
  credito = 0;
  recebendoTroco(3000);

}
void realizarPedido(int preco)
{
  desligarLeds();
  _delay_ms(100);
  piscarLed(selecao - 1);
  dinheiro -= preco;
  credito = dinheiro / 0.5;
  PORTD = Tabela[credito];
  selecao = 0;
}
void depositarMoeda()
{
  colocandoMoeda = true;
  while (colocandoMoeda)
  {
    if (!tst_bit(PINC, BOTAO_1r))
    {
      if (dinheiro <= 4.0) dinheiro += 1.0;
      colocandoMoeda = false;
      //_delay_ms(500);
    }
    if (!tst_bit(PINC, BOTAO_50c))
    {
      if (dinheiro <= 4.0) dinheiro += 0.5;
      colocandoMoeda = false;
      //_delay_ms(500);
    }
    if (dinheiro <= 4.0)
    {
      credito = dinheiro / 0.5;
      PORTD = Tabela[credito];
    }
    if (dinheiro == 2 || dinheiro == 2.5)
    {
      set_bit(PORTB, Bebidas[0]);
    }
    if (dinheiro == 3 || dinheiro == 3.5)
    {
      for (unsigned int i = 0; i < 5; i++)
      {
        set_bit(PORTB, Bebidas[i]);
      }
    }

  } // terminei de colocar a moeda
}
void desligarLeds()
{
  for (unsigned int i = 0; i < 5; i++)
  {
    clr_bit(PORTB, Bebidas[i]); // desligando leds
  }
}
void setup()
{
  for (unsigned int i = 0; i < 5; i++)
  {
    DDRB = 1 << Bebidas[i]; // configurando cada led como saída
  }
  DDRB = 1 << TROCO; // led do troco como saida
  PORTB = 0x00; // leds começam desligadas
  PORTC = 0xFF; // botoes de depositar moeda
  DDRC = 0x00;  // botoes em pull up selecionar bebida
  PCICR = 1 << PCIE1; //habilita interrupção por mudanças de sinal no PORTC
  PCMSK1 = (1 << PCINT8) | (1 << PCINT9) | (1 << PCINT10) | (1 << PCINT11) | 
           (1 << PCINT12) | (1 << PCINT13); //pinos com interrupção ligada
  sei(); //habilita as interrupções
  DDRD = 0x7F;  // 7 segmentos, pinos como saída
  PORTD = 0x3F; // 7 segmentos, inicialmente com 0
}
int main()
{
  setup(); //função para configurar o arduino e suas portas
  while (1){} //loop de execução
  return 0;
}
ISR(PCINT1_vect) //interrupções nos pinos definidos no setup realizam diferentes ações
{
  if (!(tst_bit(PINC, BOTAO_PRESENCA)))// Verifica quando há uma moeda sendo colocada
  { 
    depositarMoeda();
  }
  else if (!(tst_bit(PINC, BOTAO_SELECAO)) && dinheiro >= 2) //contador para o tipo de bebida
  {
    //_delay_ms(100);
    if (selecao <= 5)
    {
      selecao++;
    }
  }
  else if (!(tst_bit(PINC, BOTAO_CONFIRMAR))) //confirmar o pedido
  {
    
    if (selecao == 1 && dinheiro >= 2)
    {
      realizarPedido(2);
      _delay_ms(1000);
      if (credito > 0) darTroco();
    }
    else if (selecao > 1 && dinheiro >= 3)
    {
      realizarPedido(3);
      _delay_ms(1000);
      if (credito > 0) darTroco();
    }
  }
  _delay_ms(100);
}