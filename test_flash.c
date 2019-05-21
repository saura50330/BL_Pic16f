/*
// Memory Map
//   -----------------
//   |    0x0000     |   Reset vector
//   |               |
//   |    0x0004     |   Interrupt vector
//   |               |
//   |               |
//   |  Boot Block   |   (this program)
//   |               |
//   |    0x0300     |   Re-mapped Reset Vector
//   |    0x0304     |   Re-mapped High Priority Interrupt Vector
//   |               |
//   |       |       |
//   |               |
//   |  Code Space   |   User program space
//   |               |
//   |       |       |
//   |               |
//   |    0x3FFF     |
//   -----------------
//
*/
// CONFIG1
//#pragma config FEXTOSC = OFF    // FEXTOSC External Oscillator mode Selection bits (Oscillator not enabled)
//#pragma config RSTOSC = HFINT32 // Power-up default value for COSC bits (HFINTOSC with 2x PLL (32MHz))
#pragma config CLKOUTEN = OFF   // Clock Out Enable bit (CLKOUT function is disabled; I/O or oscillator function on OSC2)
#pragma config CSWEN = ON       // Clock Switch Enable bit (Writing to NOSC and NDIV is allowed)
#pragma config FCMEN = ON       // Fail-Safe Clock Monitor Enable (Fail-Safe Clock Monitor is enabled)
#pragma config FEXTOSC = OFF    // RA5 functional as input
// CONFIG2
#pragma config MCLRE = OFF       // Master Clear Enable bit (MCLR/VPP pin function is MCLR; Weak pull-up enabled )
#pragma config PWRTE = OFF      // Power-up Timer Enable bit (PWRT disabled)
#pragma config WDTE = OFF       // Watchdog Timer Enable bits (WDT disabled; SWDTEN is ignored)
#pragma config LPBOREN = ON     // Low-power BOR enable bit (ULPBOR enabled)
#pragma config BOREN = SBOREN   // Brown-out Reset Enable bits (Brown-out Reset enabled according to SBOREN)
#pragma config BORV = LOW       // Brown-out Reset Voltage selection bit (Brown-out voltage (Vbor) set to 2.45V)
#pragma config PPS1WAY = ON     // PPSLOCK bit One-Way Set Enable bit (The PPSLOCK bit can be cleared and set only once; PPS registers remain locked after one clear/set cycle)
#pragma config STVREN = ON      // Stack Overflow/Underflow Reset Enable bit (Stack Overflow or Underflow will cause a Reset)
#pragma config DEBUG = OFF      // Debugger enable bit (Background debugger disabled)

// CONFIG3
#pragma config WRT = OFF        // User NVM self-write protection bits (Write protection off)
#pragma config LVP = OFF        // Low Voltage Programming Enable bit (HV on MCLR/VPP must be used for programming.)

// CONFIG4
#pragma config CP = OFF         // User NVM Program Memory Code Protection bit (User NVM code protection disabled)
#pragma config CPD = OFF        // Data NVM Memory Code Protection bit (Data NVM code protection disabled)

#include "pic16f18313.h"
#include "xc.h"
// The bootloader code does not use any interrupts.
// However, the downloaded code may use interrupts.
// The interrupt vector on a PIC16F1937 is located at 
// address 0x0004. The following function will be located 
// at the interrupt vector and will contain a jump to
// 0x0204
#define END_FLASH                0x7FF
#define APP_START_ADD            0x200

#define  NEW_RESET_VECTOR        APP_START_ADD
#define  NEW_INTERRUPT_VECTOR    APP_START_ADD + 4 // 0x204

#define _str(x)  #x
#define str(x)  _str(x)

#define PORT_DIR     0b00101000  // 0 : output 1: input
#define ADC_INPUT    0b00000000  // 0: Digital 1: ADC

#define WR_FLH 1
#define WR_EEP (WR_FLH+1)
#define RD_FLH 3
#define RD_EEP (RD_FLH +1)
#define PING 5
#define RSTCMD 6
#define FLASH 7  // command supported by application , to jump bootloader , amkes application invalid
#define PING_APP_RESP 8 // on PING command Application send 8 as responce while BL send 5 as responce
#define READ_RAM 0x0A
#define MAST_AUTH 0x0B   // master load encripted mac befor to configure 
#define DEF 0xFF

//LED
#define set_pin_0 LATAbits.LATA0=1
#define get_pin_0 LATAbits.LATA0
#define clear_pin_0 LATAbits.LATA0=0
#define toggle_pin_0 LATAbits.LATA0^=1

//SWTCH
#define set_pin_5 LATAbits.LATA5=1
#define get_pin_5 PORTAbits.RA5
#define clear_pin_5 LATAbits.LATA5=0
#define toggle_pin_5 LATAbits.LATA5^=1

#define WRITE_FLASH_BLOCKSIZE    32
#define ERASE_FLASH_BLOCKSIZE    32



typedef unsigned char uint8;
typedef unsigned int uint16;



// un changed EEPROM  location or updated in fectory command 
// location   meaning                  value  
// 0xFF       fectory data present     0x01   // when this bit is set application can not program keys
// 0xFE       Bl version               0x01
// 0xFD       reserved                 0xXX
// 0xFC       device type              0xXX
// 0xFB       LID  3                   0xXX
// 0xFA       LID  2                   0xXX
// 0xF9       LID  1                   0xXX
// 0xF8       LID  0                   0xXX
const unsigned char Bl_Ver1 @ 0xF0FE = 0x01; // Bl_Ver

// below is FDR data (0xF0 to F7))
// 0xF7       master_mac_3                   0xXX
// 0xF6       master_mac_2                   0xXX
// 0xF5       master_mac_1                   0xXX
// 0xF4       master_mac_0                   0xXX
// 0xF3       reserved                       0xXX
// 0xF2       app version                    0xXX
// 0xF1       app checksum                   0xXX
// 0xF0       app valid                      0xXX

//const unsigned char App_Valid @ 0xF0F0 = 0x11; // app valid

#define FDR_DAT_START_ADDRESS 0xF0
#define FDR_DAT_APP_VALID_ADDRESS 0xF0   // eep write address are 1 byte
    #define MASTER_MAC_START_ADD 0x70F4  // read addresses are alwys two bytes
    #define MASTER_MAC_END_ADD 0x70F7
#define FECT_DAT_START_ADDRESS 0xF8

struct Str_Com
{
    uint8 *ptr;
    uint8 lent;
};
struct Str_ReadMem
{
    uint16 add;
    uint16 result;
    uint8 typ;
    
};
struct Str_Flh
{
    uint16  *flashWordArray;
    uint16  writeAddr;
    uint8 result;
};
struct Str_EepWr
{
    uint8 add;
    uint8 eep_data;
    
};
union Un_Bt_Data
{
    struct Str_Com Com;
    struct Str_EepWr EepWr;
    struct Str_Flh Flh;
    struct Str_ReadMem  ReadMem;
};
volatile union Un_Bt_Data Bt_Data @ 0x68 ={0};
void   Bt_ComSendData(void);     // @  0x0140;
void   Bt_UnlockSeq(void);       // @  0x0150;
void   Bt_FlashWriteBlock(void); // @  0x00C5;

void   Bt_ReadData(void);       //  @ 0x0166;
void   Bt_WriteEep(void);       //  @ 0x012A;

void memcpy(uint8* to, const uint8* from,uint8 lnt );

void interrupt serrvice_isr()
{
	asm ("pagesel  " str (NEW_INTERRUPT_VECTOR));
    asm ("goto   " str (NEW_INTERRUPT_VECTOR));
/*
    #asm
		GOTO	0x204;
	#endasm
*/
}
uint8 frame[69]; // one for command , 2 for address , 64  for data240
uint16 count; // used for delay
//0xAA is appp valid else invalid
uint8 dat_cnt;     // stores no of bytes
uint8 data_checksum;  // checksum data
// uint8 reset_char = RSTCMD;   
uint8 mac_user = 0;  // this flag is 1 if mac user is validated

void main(void)
{
    //OSCILLATOR
    OSCCON1bits.NOSC=0;         // HFINTOSC + 2x PLL = 32MHz {MUST}
    OSCCON1bits.NDIV=0;         // no clock divider          {MUST} 
    while(!OSCCON3bits.ORDY);
   
    
    //DIO
    TRISA = PORT_DIR;       // set pin dir sen=t in systam config // 0: for output , 1 for input
    ANSELA = ADC_INPUT;     // Initialise ADC pins 
    WPUA = 0x39;            // tx pin need pullup and led and switch
    
    RXPPS = 0x03;   //PPS   // rx pps RA3
    RA4PPS = 0x14;  //PPS    //tx pps RA4
    
    //UART
    TX1STA=0x24;  //  transmitter config
    BRG16 = 0;    // set baud rate at 32 MZ internal ocillator
    
    if(get_pin_5 == 0)  // if switch is pressed on power up
    { 
        while(get_pin_5 == 0);// wait for switch to release
        SPBRG=207u;   // baud
        RC1STA=0x90;  // enable seial port
        Bt_Data.Com.lent = 9;
        Bt_Data.Com.ptr = "AT+BAUD7"; // set 57k baud
        Bt_ComSendData();
        // erase data which is reset erasable
        Bt_Data.EepWr.add = FDR_DAT_START_ADDRESS; // f0ff make application invalid
        while(Bt_Data.EepWr.add < FECT_DAT_START_ADDRESS)
        {
            Bt_Data.EepWr.eep_data = 0xFF;        
            Bt_WriteEep();
            Bt_Data.EepWr.add++;
        }
    }
    
   // Bt_Data.Com.lent = 1; // one chrecter sent out on evry reset
   // Bt_Data.Com.ptr = &reset_char; // set 57k baud
    
    SPBRG=34u;    // 34: 57.6k , 103:19.3k , 207:9600
    RC1STA=0x90;  // enable seial port
    
   // Bt_ComSendData();
    
    while(1)
    { 
        if(RCIF) //if any byte received
        {
            frame[dat_cnt] = RC1REG;
			dat_cnt++;
            RCIF=0;
            count=0;  
        }
        else
        {
            count++;
            if(count == 0xFFFF) // break detected
            { // process data 
                set_pin_0;
                Bt_Data.Com.lent = 5; // all responces are of 5 byte long
                if(mac_user)
                {
                    if(frame[0] == WR_FLH) //write flash request 
                    {
                        Bt_Data.Flh.result = 0xFF; // error incorrect address OR Length
                        if(dat_cnt==67)
                        {
                            Bt_Data.Flh.writeAddr=*((uint16*)&frame[1u]);
                            Bt_Data.Flh.flashWordArray=&frame[3u];
                            if((Bt_Data.Flh.writeAddr >= NEW_RESET_VECTOR) && (Bt_Data.Flh.writeAddr <= END_FLASH)) //prevent application to write bootloader memory
                            {
                                Bt_FlashWriteBlock();
                            }
                        }
                        frame[1] = Bt_Data.Flh.result;
                       // Bt_Data.Com.lent=2;
                    }
                    else if(frame[0] == WR_EEP) // write eeprom
                    {
                        Bt_Data.EepWr.add=frame[1u];
                        Bt_Data.EepWr.eep_data=frame[3u];        
                        Bt_WriteEep();
                       // Bt_Data.Com.lent=2;
                    }
                    else if((frame[0] == RD_FLH) || (frame[0] == RD_EEP)) // read flash  OR // read eeprom request
                    {
                        Bt_Data.ReadMem.add = (*((uint16*)&frame[1u]));
                        Bt_Data.ReadMem.typ = (frame[0]-RD_FLH);
                        Bt_ReadData();            
                        *((uint16*)(&frame[3])) = Bt_Data.ReadMem.result;
                        // Bt_Data.Com.lent=5;

                    }
                    
                    else if ((frame[0] == RSTCMD) )
                    {
                        asm("RESET"); // send back ping 
                    }
                    else if ((frame[0] == READ_RAM) )
                    {
                        Bt_Data.ReadMem.add = (*((uint16*)&frame[1u]));
                        *((uint16*)(&frame[3])) = *(uint16 *)(Bt_Data.ReadMem.add);
                        // Bt_Data.Com.lent = 5;
                    }
                }
                if ((frame[0] == PING) ) // awake signal , sand awake back
                {
                   // Bt_Data.Com.lent = 1; // send back ping 
                }
                else if ((frame[0] == MAST_AUTH) ) // awake signal , sand awake back
                {
                    
                    Bt_Data.ReadMem.add = MASTER_MAC_START_ADD;
                    Bt_Data.ReadMem.typ = 1;
                    mac_user = 1; 
                    uint8 *mac_frame = &frame[1];
                    while(Bt_Data.ReadMem.add <= MASTER_MAC_END_ADD) // verify MAC address
                    {
                        Bt_ReadData(); // read eeprom
                        Bt_Data.ReadMem.add++;
                        if((uint8)Bt_Data.ReadMem.result != *mac_frame)
                        {
                          mac_user = 0; // Not a Master MAC user , send 0xFF as MAC after FDR to grant access oxFF is default mac address
                        }
                        mac_frame++;
                    }
                   // Bt_Data.Com.lent = 5;
                }
                else if((dat_cnt == 0)) //no data received check if app valid
                {
                    Bt_Data.ReadMem.add = 0x70F0;
                    Bt_Data.ReadMem.typ = 1;
                    Bt_ReadData(); // read eeprom
                   
                    if((uint8)Bt_Data.ReadMem.result == 0xAA) //0xAA is appp valid else invalid // application valid flag is true, // NVM adress starts from 7000h to 70FFh last byte
                     {
                         // verify app
                         Bt_Data.ReadMem.add = APP_START_ADD; 
                         Bt_Data.ReadMem.typ = 0;
                         data_checksum = 0x00;
                         
                         while(Bt_Data.ReadMem.add <= END_FLASH) //0x7FF
                         {
                             Bt_ReadData();            
                             data_checksum +=  (uint8)Bt_Data.ReadMem.result;
                             data_checksum +=  (uint8)(Bt_Data.ReadMem.result>>8);
                             Bt_Data.ReadMem.add++;
                         }     
                         
                         Bt_Data.ReadMem.add = 0x70F1;
                         Bt_Data.ReadMem.typ = 1;
                         Bt_ReadData(); // read eeprom
                         
                         if((data_checksum == (uint8)Bt_Data.ReadMem.result) && data_checksum) // valid app
                         {
                             clear_pin_0;
                            // jump to application
                             //TX1REG=(0x55);
                             //while(TXIF==0);
                             STKPTR = 0x1F;
                             asm ("pagesel " str(NEW_RESET_VECTOR));
                             asm ("goto  "  str(NEW_RESET_VECTOR));
                         }
                         else
                         {
                             Bt_Data.EepWr.add = FDR_DAT_APP_VALID_ADDRESS; // f0f0 make application invalid
                             Bt_Data.EepWr.eep_data = DEF;        
                             Bt_WriteEep();
                             
                         }

                     }
                     Bt_Data.Com.lent = 0;  // cannot be removed
                }
                Bt_Data.Com.ptr = frame;   
                Bt_ComSendData();   
                count = 0;
                frame[0] = DEF; // this is to make sure on bus idle default is executed
                dat_cnt = 0;
                clear_pin_0;
            }
        }          
        
    }
}
asm("global _Bt_ComSendData"); // this will remove optimization for below function , code will be generated for ame
void Bt_ComSendData(void) //@0x013D
{
    while(Bt_Data.Com.lent) 
    {
        TX1REG = *Bt_Data.Com.ptr;
        Bt_Data.Com.ptr++;
        Bt_Data.Com.lent--;
        while(TXIF==0); // wait for buffer to empty
    }
}
asm("global _Bt_FlashWriteBlock"); // this will remove optimization for below function , code will be generated for ame
void  Bt_FlashWriteBlock(void)  //@ 0x00CF
{
    uint8  i,sum=0;
           
    NVMCON1bits.WRERR = 0;      // clear WRERR bit at power up
    NVMCON1bits.NVMREGS=0;
    
    while(WR);
    // Flash write must start at the beginning of a row
    //-------------------- Block erase sequence
     // Load lower 8 bits of erase address boundary
    NVMADRL = (Bt_Data.Flh.writeAddr & 0xFF);
    // Load upper 6 bits of erase address boundary
    NVMADRH = ((Bt_Data.Flh.writeAddr & 0xFF00) >> 8);

    // Block erase sequence

    NVMCON1bits.FREE = 1;    // Specify an erase operation
    NVMCON1bits.WREN = 1;    // Allows erase cycles

    // Start of required sequence to initiate erase
    Bt_UnlockSeq();

    //---------------------------- Block write sequence
    NVMCON1bits.LWLO = 1;    // Only load write latches

    for (i=0; i<WRITE_FLASH_BLOCKSIZE; i++)
    {
        // Load lower 8 bits of write address
        NVMADRL = (Bt_Data.Flh.writeAddr & 0xFF);
        // Load upper 6 bits of write address
        NVMADRH = ((Bt_Data.Flh.writeAddr & 0xFF00) >> 8);

    // Load data in current address
        NVMDATL = Bt_Data.Flh.flashWordArray[i];
        sum+=NVMDATL;
        NVMDATH = ((Bt_Data.Flh.flashWordArray[i] & 0xFF00) >> 8);
        sum+=NVMDATH;
        if(i == (WRITE_FLASH_BLOCKSIZE-1))
        {
            // Start Flash program memory write
            NVMCON1bits.LWLO = 0;
        }
        Bt_UnlockSeq();
        Bt_Data.Flh.writeAddr++;
    }
    sum&=0x7F;
    NVMCON1bits.WREN = 0;       // Disable writes
 
    Bt_Data.Flh.result = sum;
}

asm("global _Bt_UnlockSeq"); // this will remove optimization for below function , code will be generated for ame
void Bt_UnlockSeq(void) //@ 0x0163
{
    NVMCON2 = 0x55;
    NVMCON2 = 0xAA;
    NVMCON1bits.WR = 1;      // Set WR bit to begin erase
    NOP();
    NOP();
}
asm("global _Bt_ReadData"); // this will remove optimization for below function , code will be generated for ame
void Bt_ReadData(void) //@ 0x0122
{
    
    NVMCON1bits.NVMREGS = Bt_Data.ReadMem.typ;// 1 means eeprom   // 0is flash
    NVMADR = Bt_Data.ReadMem.add;
    NVMCON1bits.RD = 1;      // Initiate Read
    NOP();
    NOP();
    Bt_Data.ReadMem.result = NVMDAT;
   
}

asm("global _Bt_WriteEep"); // this will remove optimization for below function , code will be generated for ame
void Bt_WriteEep(void) // @ 0x0150
{       
    NVMCON1bits.WRERR = 0;      // clear WRERR bit at power up
    NVMCON1bits.NVMREGS=1;
    WREN = 1;
    NVMADRH=0x70;   // NVM adress starts from 7000h to 70FFh
    NVMADRL=Bt_Data.EepWr.add;
    NVMDATL=Bt_Data.EepWr.eep_data;
    Bt_UnlockSeq();
    while(NVMCON1bits.WR);
    WREN = 0;
}

/*
asm("global _memcpy");
void memcpy(uint8* to, const uint8* from,uint8 lnt )
{
    while(lnt)
    {
         *to = *from;
         to++;
         from++;
         lnt--;
    }
}


asm("global _memcmp");
uint8 memcmp(const uint8* to, const uint8* from, uint8 lnt )
{
    uint8 temp=0;
    while(lnt)
    {
        temp = *to - *from;
         if(temp)
         {   
             break;
         }
         to++;
         from++;
         lnt--;
    }
    return temp;
}
  */
