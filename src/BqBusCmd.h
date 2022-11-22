#ifndef BqbusCmd_h
#define BqbusCmd_h
#include <Arduino.h>

#define BqPort Serial                                                   //Porta serial utilizada no protocolo BqBusCmd. \nPode ser alterada diretamente na biblioteca para uso das portas Serial1, 2, 3, etc.

class BqBusCmd                                                          //Classe para controle do protocolo BqBusCmd.
{
public:
    BqBusCmd();
    BqBusCmd(byte _numMaxRegs);                                         //Construtor da classe\n\nParâmetros:\n_numMaxTrans: Número de varáveis compartilhadas na rede.
    void begin(long vel);                                               //Inicializa a porta serial na velocidade especificada.
    void comunicacao();                                                 //Realiza um ciclo completo da comunicação, que consiste em verificar dados da porta serial\ne agrupá-los nos registradores, e reenviar os dados atualizados para o computador.
    int  getReg(unsigned int reg);                                      //Obtém o valor de uma das variáveis compartilhadas na rede.\n\nParâmetros:\npos: endereço/posição da variável desejada.
    void setReg(unsigned int reg, unsigned int val);                    //Modifica o valor de uma das variáveis compartilhadas na rede.\n\nParâmetros:\npos: endereço da variável desejada.\nval: valor a ser escrito na variável.
    int getRegBit(unsigned int reg, byte bit);                          //Obtém um bit de um valor de uma das variáveis compartilhadas na rede.\n\nParâmetros:\npos: endereço da variável desejada.\nbit: o dígito binário desejado\n\nNOTA: Ao utilizar um registrador para leitura ou escrita de bit, o mesmo não pode ser utilizado\npara leitura ou escrita de valores inteiros.
    void setRegBit(unsigned int reg, byte bit, bool state);             //Modifica um bit de um valor de uma das variáveis compartilhadas na rede.\n\nParâmetros:\npos: endereço da variável desejada.\nbit: o dígito binário desejado\nstate: estado a ser escrito no bit.\n\nNOTA: Ao utilizar um registrador para leitura ou escrita de bit, o mesmo não pode ser utilizado\npara leitura ou escrita de valores inteiros.
private:
    void escrita(); 
    void escrita(byte num);                                             
    void varMonitor(unsigned int regAddress, unsigned int var);
    void bitMonitor(unsigned int regAddress, byte regBit, bool var);
    void leitura(char digito);
    bool escritaRealizada;
    void zeraFuncao();
    bool escritaCompleta();
    int  vel;
    int  setControle[10];
    byte numMaxTrans;
    long regToChange[8];
    unsigned int* regs;
};

#endif