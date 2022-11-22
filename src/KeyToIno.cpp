//*************************************************************************************************/
//   Projeto    : Teste KeyToIno
//   Arquivo    : KeyToIno.cpp
//   Descrição  : Biblioteca para comunicação entre o software KeyToIno e o Arduino
//   Data       : 28/04/2019
//*************************************************************************************************/

#include "KeyToIno.h"

KeyToIno::KeyToIno()
{
    rede = BqBusCmd(7);
}

void KeyToIno::begin(long speed)
{
    rede.begin(speed);
}

void KeyToIno::readKeys()
{
    rede.comunicacao();
}

bool KeyToIno::isPressed(keys k)
{
    getAddress(k);
    bool state = rede.getRegBit(reg, bit);
    return state;
}

void KeyToIno::getAddress(keys k)
{
    int index = k;
    reg = index / 16;
    bit = index - reg * 16;
}