#include <Servo.h>
#include <EEPROM.h>
#include <Modbus.h>
#include <SoftwareSerial.h>

int registradores[95];     /* vetor contendo os bytes a serem manipulados pelo protocolo*/
int numMaxRegs = 95;       /* numero maximo de registradores existentes na rede */
enum {MB_SLAVE = 1,};      /* endereco do arduino na rede */

#define COMSensorStartBracoPin 4
#define servoBasePin           3
#define COMBloqueiaBracoPin    13
#define servoInclinacaoPin     5
#define servoElevacaoPin       6
#define minhaSerialRX          7
#define minhaSerialTX          8
#define servoPincaPin          9
#define ledRPin                10
#define ledGPin                11
#define ledBPin                12


Servo base;
Servo inclinacao;
Servo elevacao;
Servo pinca;

SoftwareSerial MinhaSerial(minhaSerialRX,minhaSerialTX);
String command = "";

int startPosBase = 90;
int posMinBase   = 0;
int posMaxBase   = 180;
int posAtualBase;

int startPosInclinacao = 100;
int posMinInclinacao  = 80;
int posMaxInclinacao  = 180;
int posAtualInclinacao;

int startPosElevacao = 90;
int posMinElevacao  = 80;
int posMaxElevacao  = 180;
int posAtualElevacao;

int startPosPinca = 50;
int posMinPinca   = 15;
int posMaxPinca   = 55;
int posAtualPinca;

int tamProgRobo = 3;
int primeiroRegistradorProg = 13;
int cicloAtual = 0;

int programaBracoPos[30][4] =
{
  {  0, 80, 80,15},
  {180,180,180,50},
  { 90,100, 90,15},
};

int programaBracoVel[30][4] =
{
  { 10, 10, 10, 10},
  { 10, 10, 10,180},
  { 20, 20, 20, 20},
};

void setup() {
  configure_mb_slave(115200, 'n', 0);
  
  base.attach(servoBasePin);
  inclinacao.attach(servoInclinacaoPin);
  elevacao.attach(servoElevacaoPin);
  pinca.attach(servoPincaPin);
  pinMode(COMSensorStartBracoPin,INPUT_PULLUP);
  pinMode(COMBloqueiaBracoPin,INPUT_PULLUP);
  MinhaSerial.begin(115200); 
  
  for(int i; i<4; i++) registradores[i+2] = EEPROM.read(i);
  
  pinca.write(startPosPinca);
  posAtualPinca = startPosPinca;
  base.write(startPosBase);
  posAtualBase = startPosBase;
  inclinacao.write(startPosInclinacao);
  posAtualInclinacao = startPosInclinacao;
  elevacao.write(startPosElevacao);
  posAtualElevacao = startPosElevacao;

}

int velBase       = registradores[2];
int velInclinacao = registradores[3];
int velElevacao   = registradores[4];
int velPinca      = registradores[5];

int cicloBase;
bool ignoraStatusBase = 0;

int selecaoFuncao;
int comando;
int tamProg;
bool programaEmLoop = 0;

void loop()
{ 
  //tráfego de dados bluetooth e sinais de comunicação discretos
  rede();

  //controle manual do robo
  manual(); 

  //programação do robo via supervisório
  manipulaPrograma();

  //navegação na programação do robo via supervisório
  navegacao();

  //funcionamento do processo automatico do robo
  processo();

  //bloqueia o braco robotico caso o robo esteje muito próximo da parede
  interrompeProcesso();
}

void rede()
{
  update_mb_slave(MB_SLAVE, registradores, numMaxRegs);
  selecaoFuncao = registradores[0];
  comando       = registradores[1];
  velBase       = registradores[2];
  velInclinacao = registradores[3];
  velElevacao   = registradores[4];
  velPinca      = registradores[5];
  tamProg       = registradores[6];
  
  registradores[7]  = cicloAtual;
  registradores[8]  = posAtualBase;
  registradores[9]  = posAtualInclinacao;
  registradores[10] = posAtualElevacao;
  registradores[11] = posAtualPinca;
  registradores[12] = cicloBase;

  verificaBasePronta();
  while(MinhaSerial.available()>0)
  {
    cicloBase = MinhaSerial.read()-48;
  }
}

void manual()
{
  if(selecaoFuncao == 1)
  {
    if(comando == 1) moveBase(posAtualBase+1,velBase);
    if(comando == 2) moveBase(posAtualBase-1,velBase);
    if(comando == 3) moveInclinacao(posAtualInclinacao+1,velInclinacao);
    if(comando == 4) moveInclinacao(posAtualInclinacao-1,velInclinacao);
    if(comando == 5) moveElevacao(posAtualElevacao+1,velElevacao);
    if(comando == 6) moveElevacao(posAtualElevacao-1,velElevacao);
    if(comando == 7) movePinca(posAtualPinca+1,velPinca);
    if(comando == 8) movePinca(posAtualPinca-1,velPinca);
  }
}

void manipulaPrograma()
{
  if(selecaoFuncao == 2)
  {
    programaEmLoop = 0;
    if(comando == 1) enviaPrograma(tamProgRobo);
    if(comando == 2) recebePrograma(tamProg);
    if(comando == 3) executaPrograma(tamProgRobo);
  }
}

void navegacao()
{
  if(selecaoFuncao == 3)
  { 
    if(comando == 1) salvaVelocidades();
    if(comando == 2)
    {
      if(cicloAtual>0) cicloAtual--;
      registradores[1] = 0;
    }
    if(comando == 3)
    {
      if(cicloAtual<=tamProg) cicloAtual++;
      registradores[1] = 0;
    }
    if(comando == 4) cicloAtual=0;
  }
}

void processo()
{
  static bool iniciaSequencia = 0;
  if(selecaoFuncao == 4)
  {
    if(comando == 1)
    {
      programaEmLoop = 1;
      if(digitalRead(COMSensorStartBracoPin) == LOW)
      {
        iniciaSequencia = 1;
      }
      if(iniciaSequencia == 1) executaPrograma(tamProgRobo);
    }
    if(comando == 2)
    {
      programaEmLoop = 1;
      if(digitalRead(COMSensorStartBracoPin) == LOW)
      {
        iniciaSequencia = 1;
      }
      if(iniciaSequencia == 1) 
      {
        if(executaPrograma(tamProgRobo)) iniciaSequencia = 0;
      }
    }
    if(comando>2) iniciaSequencia = 0;
    if(comando == 3) 
    {
      programaEmLoop = 1;
      executaPrograma(tamProgRobo);
    }
    if(comando == 4)
    {
      programaEmLoop = 0;
      executaPrograma(tamProgRobo);
    }
  }
}

void interrompeProcesso()
{
  if(digitalRead(COMBloqueiaBracoPin == LOW))
  {
    if(selecaoFuncao != 1 or comando != 2)
    {
      comando = 0;
      selecaoFuncao = 0;
    }
  }
}

void verificaBasePronta()
{

}

void enviaPrograma(int l)
{
  int k = primeiroRegistradorProg;
  int c = 0;
  for(int j=0;j<l;j++){
    for(int i=0;i<4;i++){
      registradores[k+i]   = programaBracoPos[j][i];
      registradores[k+i+4] = programaBracoVel[j][i];
    }
    k = k + 8;
  }
  registradores[6] = tamProgRobo;
}

void recebePrograma(int l)
{
  int k = primeiroRegistradorProg;
  int c = 0;
  for(int j=0;j<l;j++){
    for(int i=0;i<4;i++){
      programaBracoPos[j][i] = registradores[k+i];
      programaBracoVel[j][i] = registradores[k+i+4] ;
    }
    k = k + 8;
  }
  tamProgRobo = registradores[6];
}

bool executaPrograma(int linhas)
{
  static int i,j;
  static int cont =0;
  static bool bracoPronto = 1;
  static int tInicial = 0;
  static bool primeiroLoop= 0;
  static bool basePronta       = 0;
  static bool inclinacaoPronta = 0;
  static bool elevacaoPronta   = 0;
  static bool pincaPronta      = 0;
  static bool segueProg = 0;
  if(i < linhas){
    basePronta       = moveBase(programaBracoPos[i][0],programaBracoVel[i][0]);
    inclinacaoPronta = moveInclinacao(programaBracoPos[i][1],programaBracoVel[i][1]);
    elevacaoPronta   = moveElevacao(programaBracoPos[i][2],programaBracoVel[i][2]);
    pincaPronta      = movePinca(programaBracoPos[i][3],programaBracoVel[i][3]);
    segueProg = 0;
    bracoPronto = 0;
    cicloAtual = i;
    if(cicloAtual == cicloBase) segueProg == 1;
    if(basePronta and inclinacaoPronta and elevacaoPronta and pincaPronta)
    {
      if(primeiroLoop == 0)
      {
        //MinhaSerial.print(cicloAtual+1);
        primeiroLoop = 1;
      }
    }
    if(basePronta and inclinacaoPronta and elevacaoPronta and pincaPronta)
    {
      i++;
      basePronta = 0; inclinacaoPronta = 0; elevacaoPronta = 0; pincaPronta = 0;
      tInicial = 0;
      bracoPronto = 1;
      primeiroLoop = 0;
      segueProg = 1;
    }
  }
  else
  {
    if(programaEmLoop != 1)
    {
      registradores[0] = 0;
      registradores[1] = 0;
      MinhaSerial.print(0);
    }
    primeiroLoop = 0;
    i = 0;
    cicloAtual = 0;
    return(1);
  }
  return(0);
}

void aprender(int comando){
  bool comandos[4];
  int  posicoes[4];
  static int l = 0;
  if(comando == 1){
    if(comandos[0] == 1){
      for(int i; i<4; i++) comandos[i] = 0;
      salvaPos(posicoes,l);
      l++;
    }
    moveBase(posAtualBase+1,velBase);
    posicoes[0] = posAtualBase;
    comandos[0] = 1;
  }
  if(comando == 2){
    if(comandos[2] == 1){
      for(int i; i<4; i++) comandos[i] = 0;
      salvaPos(posicoes,l);
      l++;
    }
    moveBase(posAtualInclinacao+1,velInclinacao);
    posicoes[1] = posAtualInclinacao;
    comandos[1] = 1;
  }
  if(comando == 3){
    if(comandos[3] == 1){
      for(int i; i<4; i++) comandos[i] = 0;
      salvaPos(posicoes,l);
      l++;
    }
    moveBase(posAtualElevacao+1,velElevacao);
    posicoes[2] = posAtualElevacao;
    comandos[2] = 1;
  }
  if(comando == 4){
    if(comandos[4] == 1){
      for(int i; i<4; i++) comandos[i] = 0;
      salvaPos(posicoes,l);
      l++;
    }
    moveBase(posAtualPinca+1,velPinca);
    posicoes[3] = posAtualPinca;
    comandos[3] = 1;
  }
}

void salvaPos(int posicoes[], int l)
{
  int velocidades[4] = {velBase,velInclinacao,velElevacao,velPinca};
  for(int i=0;i<4;i++){
    programaBracoPos[l][i] = posicoes[i];
    programaBracoVel[l][i] = velocidades[i];
  }
}

void salvaVelocidades()
{
  EEPROM.write(0,velBase);
  EEPROM.write(1,velInclinacao);
  EEPROM.write(2,velElevacao);
  EEPROM.write(3,velPinca);
}

bool moveBase(int posAlvo, int vel)
{
  int tempoMovimento = 1000/vel;
  static long tempoInicial = 0; 
  if(posAlvo > posMaxBase) posAlvo = posMaxBase;
  if(posAlvo < posMinBase) posAlvo = posMinBase;
  if(tempoInicial == 0) tempoInicial = millis();
  if(millis() - tempoInicial > tempoMovimento){
    tempoInicial = 0;
    if(posAtualBase < posAlvo)
    {
      base.write(posAtualBase + 1);
      posAtualBase++;
    }
    if(posAtualBase > posAlvo)
    {
      base.write(posAtualBase - 1);
      posAtualBase--;
    }
  }
  if((posAtualBase == posAlvo)or(vel == 0))
  {
    tempoInicial = 0;
    return(1);
  }
  else
  {
    return(0);
  }
}

bool moveInclinacao(int posAlvo, int vel)
{
  int tempoMovimento = 1000/vel;
  static long tempoInicial = 0;
  if(posAlvo > posMaxInclinacao) posAlvo = posMaxInclinacao;
  if(posAlvo < posMinInclinacao) posAlvo = posMinInclinacao;
  if(tempoInicial == 0) tempoInicial = millis();
  if(millis() - tempoInicial > tempoMovimento){
    tempoInicial = 0;
    if(posAtualInclinacao < posAlvo)
    {
      inclinacao.write(posAtualInclinacao + 1);
      posAtualInclinacao++;
    }
    if(posAtualInclinacao > posAlvo)
    {
      inclinacao.write(posAtualInclinacao - 1);
      posAtualInclinacao--;
    } 
  }
  if((posAtualInclinacao == posAlvo)or(vel == 0))
  {
    tempoInicial = 0;
    return(1);
  }
  else
  {
    return(0);
  }
}

bool moveElevacao(int posAlvo, int vel)
{
  int tempoMovimento = 1000/vel;
  static long tempoInicial = 0;
  if(posAlvo > posMaxElevacao) posAlvo = posMaxElevacao;
  if(posAlvo < posMinElevacao) posAlvo = posMinElevacao;
  if(tempoInicial == 0) tempoInicial = millis();
  if(millis() - tempoInicial > tempoMovimento){
    tempoInicial = 0;
    if(posAtualElevacao < posAlvo)
    {
      elevacao.write(posAtualElevacao + 1);
      posAtualElevacao++;
    }
    if(posAtualElevacao > posAlvo)
    {
      elevacao.write(posAtualElevacao - 1);
      posAtualElevacao--;
    }
  }
  if((posAtualElevacao == posAlvo)or(vel == 0))
  {
    tempoInicial = 0;
    return(1);
  }
  else
  {
    return(0);
  }
}

bool movePinca(int posAlvo, int vel)
{
  int tempoMovimento = 1000/vel;
  static long tempoInicial = 0;
  if(posAlvo > posMaxPinca) posAlvo = posMaxPinca;
  if(posAlvo < posMinPinca) posAlvo = posMinPinca;
  if(tempoInicial == 0) tempoInicial = millis();
  if(millis() - tempoInicial > tempoMovimento){
    tempoInicial = 0;
    if(posAtualPinca < posAlvo)
    {
      pinca.write(posAtualPinca + 1);
      posAtualPinca++;
    }
    if(posAtualPinca > posAlvo)
    {
      pinca.write(posAtualPinca - 1);
      posAtualPinca--;
    }
  }
  if((posAtualPinca == posAlvo)or(vel == 0))
  {
    tempoInicial = 0;
    return(1);
  }
  else
  {
    return(0);
  }
}


