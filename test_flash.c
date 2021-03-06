/*
// Memory Map
//   -----------------
//   |    0x0000     |   Reset vector
//   |               |
//   |    0x0004     |   Interrupt vector  jump to 2A4
//   |               |
//   |               |
//   |  Boot Block   |   (this program)
//   |               |
//   |    0x02A0     |   Re-mapped Reset Vector
//   |    0x02A4     |   Re-mapped High Priority Interrupt Vector
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
#define APP_START_ADD            0x2A0

#define  NEW_RESET_VECTOR        APP_START_ADD
#define  NEW_INTERRUPT_VECTOR    APP_START_ADD + 4 // 0x2A4

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
#define START_WRITE_FLASH_BLOCKSIZE (WRITE_FLASH_BLOCKSIZE - 1)




typedef unsigned char uint8;
typedef short unsigned int uint16;
typedef long unsigned int uint32;



// un changed EEPROM  location or updated in fectory command 
// location   meaning                  value  
// 0xFF       fectory data present     0x01   // when this bit is set application can not program keys OR locations from 0xF8 to 0xFF
// 0xFE       Bl version               0x01
// 0xFD       reserved                 0xXX
// 0xFC       device type              0xXX   // 0x01 : DEvmo device , 0x02 : wall switch
// 0xFB       LID  3                   0xXX...// unique lock ID  UN/PW
// 0xFA       LID  2                   0xXX...// unique lock ID  UN/PW
// 0xF9       LID  1                   0xXX...// unique lock ID  UN/PW
// 0xF8       LID  0                   0xXX...// unique lock ID  UN/PW 
const unsigned char Bl_Ver1 @ 0xF0FE = 0x01; // Bl_Ver

const unsigned char LID_EEP[4] @ 0xF0F8 = {'1','2','3','5'};// default LID to be used for fectory programming


// below is FDR data (0xF0 to F7))
// 0xF7       master_mac_3                   0xXX // Only first 4 bytes from mac are considerd as MAC 
// 0xF6       master_mac_2                   0xXX
// 0xF5       master_mac_1                   0xXX
// 0xF4       master_mac_0                   0xXX
// 0xF3       Mac OK                         0xXX    // 0xFF means not written , else written, and this random value is access link key
// 0xF2       app version                    0xXX
// 0xF1       app checksum                   0xXX
// 0xF0       app valid                      0xXX    // written by OTA at end ,if matches with device type 0xFC jump to crc validation 

//const unsigned char App_Valid @ 0xF0F0 = 0x11; // app valid


#define FDR_DAT_START_ADDRESS_LSB 0xF0
#define FDR_DAT_APP_VALID_ADDRESS_LSB 0xF0   // eep write address are 1 byte
#define FDR_DAT_APP_VALID_ADDRESS 0x70F0
#define FDR_DAT_CHECKSUM_ADDRESS 0x70F1
#define FDR_DAT_CHECKSUM_ADDRESS_LSB F1
#define FDR_DAT_MASTER_FREEZ_ADD 0x70F3
#define FDR_DAT_MASTER_FREEZ_ADD_LSB 0xF3
#define FDR_DAT_MASTER_MAC_START_ADD 0x70F4  // read addresses are alwys two bytes
#define FDR_DAT_MASTER_MAC_START_ADD_LSB 0xF4
#define FDR_DAT_MASTER_MAC_END_ADD 0x70F7
#define FDR_DAT_MASTER_MAC_END_ADD_LSB 0xF7

#define FECT_DAT_START_ADDRESS 0x70F8
#define FECT_DAT_LID_START_ADDRESS 0x70F8
#define LID_LEN 4u

#define FECT_DAT_START_ADDRESS_LSB 0xF8

#define FECT_DAT_DEV_TYP_ADDRESS 0x70FC  // this data (device id) indicates app is valid and can be jmed to app after CRC validation



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

//void memcpy(uint8* to, const uint8* from,uint8 lnt );

void interrupt serrvice_isr()  // all interrupt will jump to this def address but we are making it to jump some differant address
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
uint8 mac_user = 0;  // this flag is 1 if mac user is validated
uint8 LID[LID_LEN];
uint8 lid_len_cnt = 0;
uint8 dev_type_ota = 0;
uint8 *mac_frame;

void set_delay(uint32 del) 
{
    while(del--);
}
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
    
    RC1STA=0x90;  // enable seial port
    
    if(get_pin_5 == 0)  // if switch is pressed on power up
    { 
        while(get_pin_5 == 0);// wait for switch to release
        
        SPBRG=207u;   // set baud 9600 as its basic baud of HC-06
        
        Bt_Data.Com.lent = 8;  // do not send end string
        Bt_Data.Com.ptr = "AT+BAUD7"; // set 57k baud 
        Bt_ComSendData();  // but this command is sent at 9600 baud
        
        set_delay(0x2FFFF); 

        
        // READ default BLE pin stored in EEPROM 
        Bt_Data.ReadMem.add = FECT_DAT_LID_START_ADDRESS;
        Bt_Data.ReadMem.typ = 1;
        while(lid_len_cnt < LID_LEN)
        {
            Bt_ReadData(); // read eeprom  
            LID[lid_len_cnt] = (uint8)Bt_Data.ReadMem.result;
            lid_len_cnt++;
            Bt_Data.ReadMem.add++;
        }
        
        // set the def baud
        SPBRG=34u;    // 34: 57.6k , 103:19.3k , 207:9600
       
        
        // set default UN 
        Bt_Data.Com.lent = 9;  //  do not send end of string
        Bt_Data.Com.ptr = "AT+NAMECD" ; 
        Bt_ComSendData();
        
        Bt_Data.Com.lent = 4; // 
        Bt_Data.Com.ptr = LID;
        Bt_ComSendData();
        
        set_delay(0x2FFFF); 
        
        // set Default  PW
        Bt_Data.Com.lent = 6;        
        Bt_Data.Com.ptr =  "AT+PIN";
        Bt_ComSendData();
        
        Bt_Data.Com.lent = 4; // no end of string chr
        Bt_Data.Com.ptr = LID;
        Bt_ComSendData();
       
        
        // erase data which is reset erasable
        Bt_Data.EepWr.add = FDR_DAT_START_ADDRESS_LSB; // f0ff make application invalid
        while(Bt_Data.EepWr.add < FECT_DAT_START_ADDRESS_LSB)
        {
            Bt_Data.EepWr.eep_data = 0xFF;        
            Bt_WriteEep();
            Bt_Data.EepWr.add++;
        }
    }
    
    // set the baud

    SPBRG=34u;    // 34: 57.6k , 103:19.3k , 207:9600

    
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
   
                    }
                    else if(frame[0] == WR_EEP) // write eeprom
                    {
                        // check if MAC address is already stored flag
                        Bt_Data.ReadMem.add = FDR_DAT_MASTER_FREEZ_ADD;
                        Bt_Data.ReadMem.typ = 1;
                        Bt_ReadData(); // read eeprom
                        
                        if((frame[3u] > FDR_DAT_MASTER_FREEZ_ADD_LSB) && (frame[3u] < FECT_DAT_START_ADDRESS_LSB) && ((uint8)Bt_Data.ReadMem.result != 0xFF))  // user canot wite this once written without FDR
                        {
                              // do not write MAC  FDR to re wite MAC
                              // Sequence wite the MAC
                              // set 0xF3 to random value !=0xFF
                        }
                        else // other data so can be writtan
                        {
                            Bt_Data.EepWr.add=frame[1u];
                            Bt_Data.EepWr.eep_data=frame[3u]; 
                            Bt_WriteEep();
                        }
  
                    }
                    /*
                    else if((frame[0] == RD_FLH) || (frame[0] == RD_EEP)) // read flash  OR // read eeprom request
                    {
                        Bt_Data.ReadMem.add = (*((uint16*)&frame[1u]));
                        Bt_Data.ReadMem.typ = (frame[0]-RD_FLH);
                        Bt_ReadData();            
                        *((uint16*)(&frame[3])) = Bt_Data.ReadMem.result;
                        // Bt_Data.Com.lent=5;

                    }
                    */
                    else if((frame[0] == RD_EEP))
                    {
                        Bt_Data.ReadMem.add = (*((uint16*)&frame[1u]));
                        Bt_Data.ReadMem.typ = 1;
                        Bt_ReadData();            
                        *((uint16*)(&frame[3])) = Bt_Data.ReadMem.result;
                    }
                    else if ((frame[0] == RSTCMD) )
                    {
                        asm("RESET"); // send back ping 
                    }
                    /*
                    else if ((frame[0] == READ_RAM) )
                    {
                        Bt_Data.ReadMem.add = (*((uint16*)&frame[1u]));
                        *((uint16*)(&frame[3])) = *(uint16 *)(Bt_Data.ReadMem.add);
                        // Bt_Data.Com.lent = 5;
                    }
                    */
                }
                if ((frame[0] == PING) ) // awake signal , sand awake back
                {
                   // Bt_Data.Com.lent = 1; // send back ping 
                }
                else if ((frame[0] == MAST_AUTH) && (dat_cnt == 6)) // only first MAC is compared // user canot acess as credtial length differs in APP 12 and BL 6
                {
                    // cmd_1,encp_1,mac_rand_1,mac_4,devid_4,devtype_1 // 12 bytes
                    Bt_Data.ReadMem.add = FDR_DAT_MASTER_MAC_START_ADD;
                    Bt_Data.ReadMem.typ = 1;
                    mac_user = 1; 
                    
                    mac_frame = &frame[2]; // skip encription
                    while(Bt_Data.ReadMem.add <= FDR_DAT_MASTER_MAC_END_ADD) // verify MAC address
                    {
                        Bt_ReadData(); // read eeprom
                        
                        if((uint8)Bt_Data.ReadMem.result != *mac_frame)
                        {
                          mac_user = 0; // Not a Master MAC user , send 0xFF as MAC after FDR to grant access oxFF is default mac address
                        }
                        
                        Bt_Data.ReadMem.add++;
                        mac_frame++;
                    }

                }
                else if((dat_cnt == 0)) //no data is received then check if app valid
                {
                    // Read Device Type written by OTA
                    Bt_Data.ReadMem.add = FDR_DAT_APP_VALID_ADDRESS;
                    Bt_Data.ReadMem.typ = 1;
                    Bt_ReadData(); // read eeprom
                    dev_type_ota = (uint8)Bt_Data.ReadMem.result;
                     // Read Device Type written by FECTORY
                    
                    Bt_Data.ReadMem.add = FECT_DAT_DEV_TYP_ADDRESS;
                    Bt_Data.ReadMem.typ = 1;
                    Bt_ReadData(); // read eeprom
                    
                    if(((uint8)Bt_Data.ReadMem.result == dev_type_ota) && ((uint8)Bt_Data.ReadMem.result != DEF)) //0xAA is appp valid else invalid // application valid flag is true, // NVM adress starts from 7000h to 70FFh last byte
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
                         
                         Bt_Data.ReadMem.add = FDR_DAT_CHECKSUM_ADDRESS;
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
                             Bt_Data.EepWr.add = FDR_DAT_APP_VALID_ADDRESS_LSB; // f0f0 make application invalid
                             Bt_Data.EepWr.eep_data = DEF;        
                             Bt_WriteEep();
                             
                         }

                     }
                     Bt_Data.Com.lent = 0;  // cannot be removed
                }
                else
                {
                    // un supported command received
                    Bt_Data.Com.lent = 0; // DONOT respond
                }
                Bt_Data.Com.ptr = frame;   
                Bt_ComSendData();   
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
    while(!TRMT); // wait till all bits are trasmitted
}
asm("global _Bt_FlashWriteBlock"); // this will remove optimization for below function , code will be generated for ame
void  Bt_FlashWriteBlock(void)  //@ 0x00CF
{
    uint8  i;
    Bt_Data.Flh.result = 0;       
    
    NVMCON1bits.WRERR = 0;      // clear WRERR bit at power up
    NVMCON1bits.NVMREGS=0;
    
    while(WR);
    // Flash write must start at the beginning of a row
    //-------------------- Block erase sequence
    NVMADR = Bt_Data.Flh.writeAddr;
    
    // Load lower 8 bits of erase address boundary
    // NVMADRL = (Bt_Data.Flh.writeAddr & 0xFF);
    // Load upper 6 bits of erase address boundary
    // NVMADRH = ((Bt_Data.Flh.writeAddr & 0xFF00) >> 8);

    // Block erase sequence

    NVMCON1bits.FREE = 1;    // Specify an erase operation
    NVMCON1bits.WREN = 1;    // Allows erase cycles

    // Start of required sequence to initiate erase
    Bt_UnlockSeq();

    //---------------------------- Block write sequence
    NVMCON1bits.LWLO = 1;    // Only load write latches

    for (i=0; i<WRITE_FLASH_BLOCKSIZE; i++)
    {
        NVMADR = Bt_Data.Flh.writeAddr;
        // Load lower 8 bits of write address
        //NVMADRL = (Bt_Data.Flh.writeAddr & 0xFF);
        // Load upper 6 bits of write address
       // NVMADRH = ((Bt_Data.Flh.writeAddr & 0xFF00) >> 8);

    // Load data in current address
        NVMDAT = Bt_Data.Flh.flashWordArray[i];
        
        //NVMDATL = Bt_Data.Flh.flashWordArray[i];
        Bt_Data.Flh.result += NVMDATL;
        //NVMDATH = ((Bt_Data.Flh.flashWordArray[i] & 0xFF00) >> 8);
        Bt_Data.Flh.result += NVMDATH;
        if(i == START_WRITE_FLASH_BLOCKSIZE)
        {
            // Start Flash program memory write
            NVMCON1bits.LWLO = 0;
        }
        Bt_UnlockSeq();
        Bt_Data.Flh.writeAddr++;
    }
    Bt_Data.Flh.result &= 0x7F;
    NVMCON1bits.WREN = 0;       // Disable writes
 
    //Bt_Data.Flh.result = sum;
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
    NVMCON1bits.NVMREGS = 1;
    WREN = 1;
    NVMADRH = 0x70;   // NVM adress starts from 7000h to 70FFh
    NVMADRL = Bt_Data.EepWr.add;
    NVMDATL = Bt_Data.EepWr.eep_data;
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