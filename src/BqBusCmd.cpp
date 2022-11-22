#include "BqBusCmd.h"

BqBusCmd::BqBusCmd()
{

}

BqBusCmd::BqBusCmd(byte _numMaxTrans)
{
    numMaxTrans = _numMaxTrans;
    for(int i = 0; i<8; i++)
    {
        regToChange[i] = 0;
    }
    if(regs != 0)
    {
        delete [] regs;
    }
    regs = new int [numMaxTrans];
    for(int i=0; i<numMaxTrans; i++){
        regs[i]=0;
    }
}

void BqBusCmd::begin(long vel)
{
    BqPort.begin(vel);
}

void BqBusCmd::varMonitor(unsigned int regAddress, unsigned int var)
{
    regs[regAddress] = var;
    comunicacao();
}

void BqBusCmd::bitMonitor(unsigned int regAddress, byte regBit, bool var)
{
    bitWrite(regs[regAddress], regBit, var);
    comunicacao();
}

void BqBusCmd::comunicacao()
{
    static bool setOn = false;
    static bool leituraCompleta = false;
    if(!setOn)
    {
        setOn = true;
    }
    int digito = 0;
    static int pos = 0;
    while(BqPort.available() > 0)
    {
        digito = BqPort.read();
        leitura(digito);
        if(digito == '_')
        {
            leituraCompleta = true;
        }
    }
    if(leituraCompleta)
    {
        escrita();
        leituraCompleta = false;
    }
}

void BqBusCmd::leitura(char digito){
    static int pos = 0;
    static int aux = 0;
    if(digito != '.' and digito != '_')
    {
        aux = aux*10 + digito - 48;
    }
    if(digito == '.')
    {
        int i = pos/32;
        if(!bitRead(regToChange[i], pos))
        {
            regs[pos] = aux;
            pos++;
            aux = 0;
        }
        else
        {
            bitWrite(regToChange[i], pos, false);
            pos++;
            aux = 0;
        }
    }
    if(digito == '_')
    {
        pos = 0;
        aux = 0;
    }
}

void BqBusCmd::escrita()
{
    for(int i=0; i<numMaxTrans; i++)
    {
        BqPort.print(regs[i]);
        BqPort.print('.');
    }
    BqPort.print('_');
    escritaRealizada = 1;
}

bool BqBusCmd::escritaCompleta()
{
    bool retorno = escritaRealizada;
    escritaRealizada = 0;
    return(retorno);
}

void BqBusCmd::escrita(byte num)
{
    for(int i=0; i<num; i++)
    {
        BqPort.print(regs[i]);
        BqPort.print('.');
    }
    BqPort.println('_');
    escritaRealizada = 1;
}

int BqBusCmd::getReg(unsigned int pos)
{
    return(regs[pos]);
}

void BqBusCmd::setReg(unsigned int pos, unsigned int val)
{
    if(regs[pos] != val)
    {
        regs[pos] = val;
        int i = pos/32;
        bitWrite(regToChange[i], pos, true);
    }
}

int BqBusCmd::getRegBit(unsigned int pos, byte bitNum)
{
    return(bitRead(regs[pos], bitNum));
}

void BqBusCmd::setRegBit(unsigned int pos, byte bitNum, bool state)
{
    if(bitRead(regs[pos], bitNum) !=  state)
    {
        bitWrite(regs[pos], bitNum, state);
        int i = pos/32;
        bitWrite(regToChange[i], pos, true);
    }
}


