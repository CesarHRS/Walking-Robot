#include <EEPROM.h>
#include <Modbus.h>
#include <Ultrasonic.h>
#include <SoftwareSerial.h>

int registradores[70];     /* vetor contendo os bytes a serem manipulados pelo protocolo*/
int numMaxRegs = 70;      /* numero maximo de registradores existentes na rede */
enum {MB_SLAVE = 1,};    /* endereco do arduino na rede */

#define encoderEPin            2
#define encoderDPin            3
#define MinhaSerialTX          4
#define MinhaSerialRX          5
#define COMSensorStartBracoPin 6
#define motorEFPin             7
#define motorETPin             8
#define COMBloqueiaBracoPin    9
#define motorEVPin             10
#define motorDFPin             13
#define motorDTPin             12
#define motorDVPin             11
#define sensorFTPin            14
#define sensorFEPin            15
#define sensorETPin            16
#define sensorEEPin            17
#define sensorDTPin            18
#define sensorDEPin            19

Ultrasonic sensorF(sensorFTPin, sensorFEPin);
Ultrasonic sensorE(sensorETPin, sensorEEPin);
Ultrasonic sensorD(sensorDTPin, sensorDEPin);
SoftwareSerial MinhaSerial(MinhaSerialRX,MinhaSerialTX);
String command = "";

int vStart = 0;
int monit = 0;

byte pulsosD = 0;
long motorDPos = 0;
int velControleD = vStart;
int motorDVelMedida = 0;
int motorDPot = 0;

byte pulsosE = 0;
long motorEPos = 0;
int  velControleE = vStart;
int  motorEVelMedida = 0;
int  motorEPot = 0;

int   motorDRefVel   = registradores[2];
float motorDKp       = registradores[3];
float motorDKd       = registradores[4];
int   motorERefVel   = registradores[5];
float motorEKp       = registradores[6];

unsigned int pulsosVolta = 20;

int tempoAmostragem;

int tamProgRobo = 2 ;
int primeiroRegistradorProg = 22;
int cicloAtual = 0;
bool resetVel = 0;
bool desligaSensores = 0;
bool ignoraStatusBraco = 0;
bool programaEmLoop = 0;
int selecaoFuncao;
int comando;
int tamProg;
int sensorGatilho;
int valorGatilho;
int cicloBraco;

int programaPosVel[10][4] =
{
  {  10,  180,  10,  180},
  {  20,  180,  20,  180},
  {   0,    0,   0,    0},
};

int processoPosVel[2500][6];

void setup() {
  configure_mb_slave(115200, 'n', 0);

  pinMode(encoderDPin, INPUT);
  pinMode(encoderEPin, INPUT);
  pinMode(COMSensorStartBracoPin, OUTPUT);
  pinMode(COMBloqueiaBracoPin,  OUTPUT);
  digitalWrite(COMSensorStartBracoPin,HIGH);
  digitalWrite(COMBloqueiaBracoPin,HIGH);

  pinMode(motorDFPin, OUTPUT);
  pinMode(motorDTPin, OUTPUT);
  pinMode(motorDVPin, OUTPUT);
  pinMode(motorEFPin, OUTPUT);
  pinMode(motorETPin, OUTPUT);
  pinMode(motorEVPin, OUTPUT);

  attachInterrupt(digitalPinToInterrupt(3), contadorD, FALLING);
  attachInterrupt(digitalPinToInterrupt(2), contadorE, FALLING);

  pulsosD = 0;
  pulsosE = 0;
  
  carregaParametros();
  MinhaSerial.begin(115200);
}

void loop() {
  //tráfego de dados bluetooth e sinais de comunicação discretos
  rede();
  
  //controle manual do robo
  manual();
  
  //programação do robo via supervisório
  manipulaPrograma();
  
  //controle de velocidade do robo
  controleVel();
  
  //salva os parametros do robo na EEPROM
  salvaParametros();

  //navegação na programação do robo via supervisório
  navegacao();

  //funcionamento do processo automatico do robo
  processo();

  //zera posicao dos motores via supervisorio
  zeraPosMotores();

  //para as rodas caso robo esteje muito próximo da parede
  //interrompeProcesso();

}

//funcoes e procedimentos principais do programa

void rede()
{
  update_mb_slave(MB_SLAVE, registradores, numMaxRegs);
  selecaoFuncao      = registradores[0];
  comando            = registradores[1];
  motorDRefVel       = registradores[2];
  motorDKp           = registradores[3];
  motorDKd           = registradores[4];
  motorERefVel       = registradores[5];
  motorEKp           = registradores[6];
  tempoAmostragem    = registradores[7] * 2;
  tamProg        = registradores[8];
  registradores[9]   = motorDVelMedida;
  registradores[10]  = motorDPot;
  registradores[11]  = motorDPos;
  registradores[12]  = motorEVelMedida;
  registradores[13]  = motorEPot;
  registradores[14]  = motorEPos;
  registradores[15]  = cicloBraco;
  registradores[16]  = cicloAtual;
  if(desligaSensores != 1)
  {
    registradores[17]  = leituraSensor(sensorE);
    registradores[18]  = leituraSensor(sensorF);
    registradores[19]  = leituraSensor(sensorD);
  }
  sensorGatilho  = registradores[20];
  valorGatilho   = registradores[21];
  verificaBracoPronto();

  while(MinhaSerial.available()>0)
  {
    cicloBraco = MinhaSerial.read()-48;
  }
}

void manual()
{
  if (selecaoFuncao == 1){
    desligaSensores = 1;
    if (comando == 0) ponteH(0, 0);
    if (comando == 1) ponteH( motorERefVel,  motorDRefVel);
    if (comando == 2) ponteH(-motorERefVel, -motorDRefVel);
    if (comando == 3) ponteH( motorERefVel, -motorDRefVel);
    if (comando == 4) ponteH(-motorERefVel,  motorDRefVel);
  }
  else desligaSensores = 0;
}

void manipulaPrograma()
{
  if (selecaoFuncao == 2)
  {
    programaEmLoop = 0;
    if (comando == 1) enviaPrograma(tamProgRobo);
    if (comando == 2) recebePrograma(tamProg);
    if (comando == 3) executaPrograma(tamProgRobo);
  }
}

void controleVel()
{
  static bool iniciaControle = 0;
  if (selecaoFuncao == 3)
  {
    if (iniciaControle == 0)
    {
      pulsosD = 0;
      pulsosE = 0;
    }
    iniciaControle = 1;
  }
  else if (selecaoFuncao == 0)
  {
    ponteH(0, 0);
    velControleE = vStart;
    velControleD = vStart;
    if ((comando == 1) and (motorDVelMedida == 0))
    {
      iniciaControle = 0;
      registradores[1] = 0;
    }
    if ((comando == 2) and (motorEVelMedida == 0))
    {
      iniciaControle = 0;
      registradores[1] = 0;
    }
  }
  if (iniciaControle == 1)
  {
    if (comando == 1)
    {
      int vel = controleVelD(motorDRefVel);
      if (vel > 255) vel = 255;
      if (vel < 0)   vel = 0;
      if ((selecaoFuncao == 3) and (comando == 1)) ponteH(0, vel);
    }
    if (comando == 2)
    {
      int vel = controleVelE(motorERefVel);
      if (vel > 255) vel = 255;
      if (vel < 0)   vel = 0;
      if ((selecaoFuncao == 3) and (comando == 2)) ponteH(vel, 0);
    }
  }
}

void salvaParametros()
{
  if (selecaoFuncao == 4)
  {
    for (int i = 2; i < 8; i++)
    {
      EEPROM.write(i, registradores[i]);
    }
  }
}

void navegacao()
{
  if(selecaoFuncao == 5)
  { 
    if(comando == 1)
    {
      if(cicloAtual>0) cicloAtual--;
      registradores[1] = 0;
    }
    if(comando == 2)
    {
      if(cicloAtual<=tamProg) cicloAtual++;
      registradores[1] = 0;
    }
    if(comando == 3) 
    {
      cicloAtual=0;
      resetVel = 1;
    }
  }
}

void processo()
{
  if(selecaoFuncao == 6)
  {
    int valorSensor = 0, limiteSensor = registradores[21];
    static bool iniciaSequencia = 0;
    if(sensorGatilho == 0) valorSensor = registradores[17];
    if(sensorGatilho == 1) valorSensor = registradores[18];
    if(sensorGatilho == 2) valorSensor = registradores[19];
    if(comando == 1)
    {
      programaEmLoop = 1;
      if(valorSensor < limiteSensor) 
      {
        iniciaSequencia = 1;
        digitalWrite(COMSensorStartBracoPin,LOW);
      }
      else
      {
        digitalWrite(COMSensorStartBracoPin,HIGH);
      }
      if(iniciaSequencia == 1) executaPrograma(tamProgRobo);
    }
    if(comando == 2)
    {
      programaEmLoop = 1;
      if(valorSensor < limiteSensor)
      {
        iniciaSequencia = 1;
        digitalWrite(COMSensorStartBracoPin,LOW);
      }
      else
      {
        digitalWrite(COMSensorStartBracoPin,HIGH);
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

void zeraPosMotores()
{
 if (comando == 100) motorDPos = 0;
 if (comando == 101) motorEPos = 0;
}

void interrompeProcesso()
{
  int sensorFrente = registradores[18];
  if(sensorFrente<15)
  {
    if(selecaoFuncao != 1 or comando != 2) ponteH(0,0);
    digitalWrite(COMBloqueiaBracoPin,LOW);
  }
  else
  {
    digitalWrite(COMBloqueiaBracoPin,HIGH);
  }
}

//funcoes e procedimentos secundários do programa

void verificaBracoPronto()
{

}

void enviaPrograma(int l)
{
  int k = primeiroRegistradorProg;
  int c = 0;
  for (int j = 0; j < l; j++) {
    for (int i = 0; i < 4; i++) {
      registradores[k + c] = programaPosVel[j][i];
      c++;
    }
  }
  registradores[8] = tamProgRobo;
}

void recebePrograma(int l)
{
  int k = primeiroRegistradorProg;
  int c = 0;
  for (int j = 0; j < l; j++) {
    for (int i = 0; i < 4; i++) {
      programaPosVel[j][i] = registradores[k + c];
      c++;
    }
  }
  tamProgRobo = registradores[8];
}

bool executaPrograma(int linhas)
{
  int i = cicloAtual;
  static int cont = 0;
  static bool primeiroLoop = 0;
  static bool segueProg = 0;
  static bool roboPronto = 0;
  if (i < linhas) {
     roboPronto = moveRobo(programaPosVel[i][0], programaPosVel[i][1], programaPosVel[i][2], programaPosVel[i][3]);
    if (roboPronto)
    {
      if(primeiroLoop == 0){
        //MinhaSerial.print(cicloAtual+1);
        primeiroLoop = 1;
      }
    }
    if(cicloAtual == cicloBraco) segueProg = 1;
    if (roboPronto)
    {
      cicloAtual++;
      roboPronto = 0;
      segueProg = 0;
    }else primeiroLoop = 0;
  }
  else
  {
    motorDPos = 0;
    motorEPos = 0;
    cicloAtual = 0;
    i = 0;
    if(programaEmLoop == 0)
    {
      registradores[0] = 0;
      registradores[1] = 0;
      MinhaSerial.print(0);
    }
    else
    {
      return(1);
    }
  }
  return(0);
}

bool moveMotorE(int ref, int vel)
{
  static int velCont = 0;
  static int posInicial = 0;
  int rampa = 10;
  int distRest = abs(posInicial + ref - motorEPos);
  if(resetVel == 1)
  {
    velCont = 0;
    resetVel = 0;
  }
  if(ref<=10) rampa = ref/2;
  if(velCont == 0) velCont = vel;
  if(posInicial == 0) posInicial = motorEPos;
  if(ref>0)
  {
    ligaMotorE(velCont);
  }
  if(ref<0)
  {
    ligaMotorE(-velCont);
  }
  if(ref == 0) return 1;
  if(distRest<rampa) 
  {
    velCont = 120;
    desligaSensores = 1;
  }
  if(((motorEPos - ref >= posInicial)and(ref>0)) or ((motorEPos - ref <= posInicial)and(ref<0)))
  {
    desligaSensores = 0;
    ligaMotorE(0);
    posInicial = 0;
    velCont = 0;
    return 1;
  }
  else return 0;
}

bool moveMotorD(int ref, int vel)
{
  static int velCont = 0;
  static int posInicial = 0;
  int rampa = 10;
  int distRest = abs(posInicial + ref - motorDPos);
  if(resetVel == 1)
  {
    velCont = 0;
    resetVel = 0;
  }
  if(ref<=10) rampa = ref/2;
  if(velCont == 0) velCont = vel;
  if(posInicial == 0) posInicial = motorDPos;
  if(ref>0)
  {
    ligaMotorD(velCont);
  }
  if(ref<0)
  {
    ligaMotorD(-velCont);
  }
  if(ref == 0) return 1;
  if(distRest<rampa) 
  {
    velCont = 120;
    desligaSensores = 1;
  }
  if(((motorDPos - ref >= posInicial)and(ref>0)) or ((motorDPos - ref <= posInicial)and(ref<0)))
  {
    desligaSensores = 0;
    ligaMotorD(0);
    posInicial = 0;
    velCont = 0;
    return 1;
  }
  else return 0;
}

int controleVelD(int ref)
{
  int velMedida = calculaVelD();
  static int erroA = 0;
  float erro = ref - velMedida;
  static long tempoInicial = 0;
  if (velMedida == 0)
  {
    erroA = 0;
  }
  if (tempoInicial == 0) tempoInicial = millis();
  if (millis() - tempoInicial > tempoAmostragem)
  {
    tempoInicial = 0;
    if (erro != 0)
    {
      float calculoDerivativo   = (erro - erroA) * (motorDKd / 10);
      erroA = erro;
      float calculoProporcional = erro * (motorDKp / 10);
      int correcao = (int)calculoProporcional;
      correcao = correcao + (int)calculoDerivativo;
      monit = abs(correcao);
      velControleD = velControleD + correcao;
      if (velControleD > 255) velControleD = 255;
      if (velControleD < -255) velControleD = -255;
      return (velControleD);
    }
    else
    {
      if (velControleE > 255) velControleE = 255;
      if (velControleE < 0) velControleE = 0;
      return (velControleD);
    }
  }
  else
  {
    return velControleD;
  }
}

int controleVelE(int ref)
{
  int velMedida = calculaVelE();
  float erro = ref - velMedida;
  static long tempoInicial = 0;
  if (tempoInicial == 0) tempoInicial = millis();
  if (millis() - tempoInicial > tempoAmostragem)
  {
    tempoInicial = 0;
    if (erro != 0)
    {
      float calculo = erro * (motorEKp / 10);
      int correcao = (int) calculo ;
      monit = calculo;
      velControleE = velControleE + correcao;
      if (velControleE > 255) velControleE = 255;
      if (velControleE < 0) velControleE = 0;
      return (velControleE);
    }
    else
    {
      if (velControleE > 255) velControleE = 255;
      if (velControleE < 0) velControleE = 0;
      return (velControleE);
    }
  }
  else
  {
    return velControleE;
  }
}

void contadorD()
{
  pulsosD++;
  if(motorDPot > 0) motorDPos++;
  if(motorDPot < 0) motorDPos--;
}

void contadorE()
{
  pulsosE++;
  if(motorEPot > 0) motorEPos++;
  if(motorEPot < 0) motorEPos--;
}

int calculaVelD()
{
  static long tempoAnterior = 0;
  if (millis() - tempoAnterior >= tempoAmostragem)
  {
    detachInterrupt(0);
    motorDVelMedida = (60 * 1000 / pulsosVolta ) / (millis() - tempoAnterior) * pulsosD;
    tempoAnterior = millis();
    motorDVelMedida = pulsosD;
    pulsosD = 0;
    attachInterrupt(digitalPinToInterrupt(3), contadorD, FALLING);
    return (motorDVelMedida);
  }
  if (millis() < tempoAmostragem)
  {
    return (255);
  }
}

int calculaVelE()
{
  static long tempoAnterior = 0;
  if (millis() - tempoAnterior >= tempoAmostragem)
  {
    detachInterrupt(0);
    motorEVelMedida = (60 * 1000 / pulsosVolta ) / (millis() - tempoAnterior) * pulsosE;
    tempoAnterior = millis();
    motorEVelMedida = pulsosE;
    pulsosE = 0;
    attachInterrupt(digitalPinToInterrupt(2), contadorE, FALLING);
    return (motorEVelMedida);
  }
  if (millis() < tempoAmostragem)
  {
    return (255);
  }
}

void imprimePulsos()
{
  Serial.print("Roda Esquerda:");
  Serial.print(motorDPos);
  Serial.print("-Roda Direita:");
  Serial.println(motorEPos);
}

void ponteH(int velE, int velD)
{
  float correcaoD = 1;
  float correcaoE = 1;
  if (velE > 255) velE = 255;
  if (velE < -255) velE = -255;
  if (velD > 255) velD = 255;
  if (velD < -255) velD = -255;
  velD = velD * correcaoD;
  velE = velE * correcaoE;
  
  ligaMotorD(velD);
  ligaMotorE(velE);
}

void controladorRobo(int refE, int refD)
{
  static int velD = 0;
  static int velE = 0;
  velD = controleVelD(refD);
  if (velD > 255) velD = 255;
  if (velD < 0)   velD = 0;
  velE = controleVelE(refE);
  if (velE > 255) velE = 255;
  if (velE < 0)   velE = 0;
  ponteH(velE, velD);
}

bool moveRobo(int refE, int velE, int refD, int velD)
{
  static bool statusMotorE = 0;
  static bool statusMotorD = 0;
  static bool timer = 0;
  static long tInicial = 0;
  if(statusMotorE == 0) statusMotorE = moveMotorE(refE,velE);
  if(statusMotorD == 0) statusMotorD = moveMotorD(refD,velD);
  if(statusMotorE and statusMotorD)
  {
    tInicial = 0;
    timer = 0;
    statusMotorE = 0;
    statusMotorD = 0;
    return(1); 
  }
  else return 0;
}

void ligaMotorD(int vel)
{
  if(vel>0)
  {
    digitalWrite(motorDTPin, LOW);
    digitalWrite(motorDFPin, HIGH);
    analogWrite(motorDVPin, vel);
  }
  if(vel<0)
  {
    digitalWrite(motorDTPin, HIGH);
    digitalWrite(motorDFPin, LOW);
    analogWrite(motorDVPin, -vel);
  }
  if(vel == 0)
  {
    digitalWrite(motorDTPin, LOW);
    digitalWrite(motorDFPin, LOW);
    analogWrite(motorDVPin, 0);
  }
  motorDPot = vel;
}

void ligaMotorE(int vel)
{
  if(vel>0)
  {
    digitalWrite(motorETPin, LOW);
    digitalWrite(motorEFPin, HIGH);
    analogWrite(motorEVPin, vel);
  }
  if(vel<0)
  {
    digitalWrite(motorETPin, HIGH);
    digitalWrite(motorEFPin, LOW);
    analogWrite(motorEVPin, -vel);
  }
  if(vel == 0)
  {
    digitalWrite(motorETPin, HIGH);
    digitalWrite(motorEFPin, HIGH);
    analogWrite(motorEVPin, 255);
  }
  motorEPot = vel;
}

void carregaParametros()
{
  for (int i = 2; i < 8; i++)
  {
    registradores[i] = EEPROM.read(i);
  }
}

float leituraSensor(Ultrasonic sensor)
{
  int cmMsec;
  long microsec = 0;
  microsec = sensor.timing();
  cmMsec = sensor.convert(microsec, Ultrasonic::CM);
  if(cmMsec>300) cmMsec = 300;
  return cmMsec;
}

