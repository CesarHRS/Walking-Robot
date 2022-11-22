//*************************************************************************************************/
//   Projeto    : Teste KeyToIno
//   Arquivo    : KeyToIno.h
//   Descrição  : Biblioteca para comunicação entre o software KeyToIno e o Arduino
//   Data       : 28/04/2019
//*************************************************************************************************/

#ifndef KeyToIno_h
#define KeyToIno_h

#include "BqBusCmd.h"

enum keys                           //Enumeração das teclas
{
    KEY_Escape,
    KEY_F1,
    KEY_F2,
    KEY_F3,
    KEY_F4,
    KEY_F5,
    KEY_F6,
    KEY_F7,
    KEY_F8,
    KEY_F9,
    KEY_F10,
    KEY_F11,
    KEY_F12,
    KEY_Scroll,
    KEY_Pause,
    KEY_Oemtilde,
    KEY_D1,
    KEY_D2,
    KEY_D3,
    KEY_D4,
    KEY_D5,
    KEY_D6,
    KEY_D7,
    KEY_D8,
    KEY_D9,
    KEY_Q,
    KEY_W,
    KEY_E,
    KEY_R,
    KEY_T,
    KEY_Y,
    KEY_U,
    KEY_I,
    KEY_O,
    KEY_P,
    KEY_A,
    KEY_S,
    KEY_D,
    KEY_F,
    KEY_G,
    KEY_H,
    KEY_J,
    KEY_K,
    KEY_L,
    KEY_Z,
    KEY_X,
    KEY_C,
    KEY_V,
    KEY_B,
    KEY_N,
    KEY_M,
    KEY_OemOpenBrackets,
    KEY_Oem6,
    KEY_Return,
    KEY_Delete,
    KEY_End,
    KEY_Next,
    KEY_Capital,
    KEY_Oem1,
    KEY_Oem7,
    KEY_Oem5,
    KEY_ShiftKey,
    KEY_OemBackslash,
    KEY_Oemcomma,
    KEY_OemPeriod,
    KEY_OemQuestion,
    KEY_ControlKey,
    KEY_LWin,
    KEY_Menu,
    KEY_Space,
    KEY_RWin,
    KEY_Apps,
    KEY_Left,
    KEY_Down,
    KEY_Right,
    KEY_Up,
    KEY_OemMinus,
    KEY_Oemplus,
    KEY_Back,
    KEY_Insert,
    KEY_Home,
    KEY_PageUp,
    KEY_NumLock,
    KEY_Divide,
    KEY_Multiply,
    KEY_Subtract,
    KEY_Add,
    KEY_RButton,
    KEY_Decimal,
    KEY_NumPad0,
    KEY_NumPad1,
    KEY_NumPad2,
    KEY_NumPad3,
    KEY_NumPad4,
    KEY_NumPad5,
    KEY_NumPad6,
    KEY_NumPad7,
    KEY_NumPad8,
    KEY_NumPad9
};

class KeyToIno                      //Classe para o gerenciamento da comunicação entre o Arduino e o software KeyToIno
{
    public:                 
    KeyToIno();                     //Construtor básico da classe KeyToIno
    void begin(long speed);         //Inicializa a rede na velocidade especificada\n\nParâmetros:\nspeed: velocidade da comunicação
    void readKeys();                //Faz a leitura das teclas pressionadas, e atualiza os registradores
    bool isPressed(keys k);         //Verifica se a tecla está pressionada ou solta\n\nParâmetros:\nk: tecla a ser verificada (chame as teclas digitando 'KEY')
    private:
    BqBusCmd rede;
    void getAddress(keys k);
    int reg;
    byte bit;
};

#endif