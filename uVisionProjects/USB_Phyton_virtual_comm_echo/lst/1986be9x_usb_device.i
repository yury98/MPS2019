#line 1 "src\\1986BE9x_usb_device.c"





















 

 
#line 1 ".\\inc\\opora.h"





















 
 





 

typedef enum IRQn
{
 
  NonMaskableInt_IRQn         = -14,     
  HardFault_IRQn              = -13,     
  SVCall_IRQn                 = -5,      
  PendSV_IRQn                 = -2,      
  SysTick_IRQn                = -1,      

 
  MIL_STD_1553B2_IRQn         = 0,       
  MIL_STD_1553B1_IRQn         = 1,       
  USB_IRQn                    = 2,       
  CAN1_IRQn					  = 3,		 
  CAN2_IRQn					  = 4,		 
  DMA_IRQn                    = 5,       
  UART1_IRQn                  = 6,       
  UART2_IRQn                  = 7,       
  SSP1_IRQn                   = 8,       
  BUSY_IRQn					  = 9,		 
  ARINC429R_IRQn              = 10,      
  POWER_IRQn                  = 11,      
  WWDG_IRQn                   = 12,      
  TIMER4_IRQn				  = 13,		 
  TIMER1_IRQn                 = 14,      
  TIMER2_IRQn                 = 15,      
  TIMER3_IRQn                 = 16,      
  ADC_IRQn                    = 17,      
  ETHERNET_IRQn        	      = 18,      
  SSP3_IRQn			          = 19,      
  SSP2_IRQn                   = 20,      
  ARINC429T1_IRQn			  = 21,		 
  ARINC429T2_IRQn			  = 22,		 
  ARINC429T3_IRQn			  = 23,		 
  ARINC429T4_IRQn			  = 24,		 
  BKP_IRQn		              = 27,      
  EXT_INT1_IRQn               = 28,      
  EXT_INT2_IRQn               = 29,      
  EXT_INT3_IRQn               = 30,      
  EXT_INT4_IRQn               = 31       
}IRQn_Type;

typedef enum {RESET = 0, SET = !RESET} FlagStatus, ITStatus;

typedef enum {DISABLE = 0, ENABLE = !DISABLE} FunctionalState;


typedef enum {ERROR = 0, SUCCESS = !ERROR} ErrorStatus;

 
#line 1 ".\\inc\\core_cm1.h"
 




















 





































 

 
 
 
 
 
 
 
 








 











#line 1 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\stdint.h"
 
 





 









     
#line 27 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\stdint.h"
     











#line 46 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\stdint.h"





 

     

     
typedef   signed          char int8_t;
typedef   signed short     int int16_t;
typedef   signed           int int32_t;
typedef   signed       __int64 int64_t;

     
typedef unsigned          char uint8_t;
typedef unsigned short     int uint16_t;
typedef unsigned           int uint32_t;
typedef unsigned       __int64 uint64_t;

     

     
     
typedef   signed          char int_least8_t;
typedef   signed short     int int_least16_t;
typedef   signed           int int_least32_t;
typedef   signed       __int64 int_least64_t;

     
typedef unsigned          char uint_least8_t;
typedef unsigned short     int uint_least16_t;
typedef unsigned           int uint_least32_t;
typedef unsigned       __int64 uint_least64_t;

     

     
typedef   signed           int int_fast8_t;
typedef   signed           int int_fast16_t;
typedef   signed           int int_fast32_t;
typedef   signed       __int64 int_fast64_t;

     
typedef unsigned           int uint_fast8_t;
typedef unsigned           int uint_fast16_t;
typedef unsigned           int uint_fast32_t;
typedef unsigned       __int64 uint_fast64_t;

     




typedef   signed           int intptr_t;
typedef unsigned           int uintptr_t;


     
typedef   signed     long long intmax_t;
typedef unsigned     long long uintmax_t;




     

     





     





     





     

     





     





     





     

     





     





     





     

     






     






     






     

     


     


     


     

     
#line 216 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\stdint.h"

     



     






     
    
 



#line 241 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\stdint.h"

     







     










     











#line 305 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\stdint.h"






 
#line 91 ".\\inc\\core_cm1.h"

















 

#line 117 ".\\inc\\core_cm1.h"





 


 





 
typedef struct
{
  volatile uint32_t ISER[8];                       
       uint32_t RESERVED0[24];
  volatile uint32_t ICER[8];                       
       uint32_t RSERVED1[24];
  volatile uint32_t ISPR[8];                       
       uint32_t RESERVED2[24];
  volatile uint32_t ICPR[8];                       
       uint32_t RESERVED3[24];
  volatile uint32_t IABR[8];                       
       uint32_t RESERVED4[56];
  volatile uint8_t  IP[240];                       
       uint32_t RESERVED5[644];
  volatile  uint32_t STIR;                          
}  NVIC_Type;
   





 
typedef struct
{
  volatile const  uint32_t CPUID;                         
  volatile uint32_t ICSR;                          
  volatile uint32_t VTOR;                          
  volatile uint32_t AIRCR;                         
  volatile uint32_t SCR;                           
  volatile uint32_t CCR;                           
  volatile uint8_t  SHP[12];                       
  volatile uint32_t SHCSR;                         
  volatile uint32_t CFSR;                          
  volatile uint32_t HFSR;                          
  volatile uint32_t DFSR;                          
  volatile uint32_t MMFAR;                         
  volatile uint32_t BFAR;                          
  volatile uint32_t AFSR;                          
  volatile const  uint32_t PFR[2];                        
  volatile const  uint32_t DFR;                           
  volatile const  uint32_t ADR;                           
  volatile const  uint32_t MMFR[4];                       
  volatile const  uint32_t ISAR[5];                       
} SCB_Type;

 












 






























 






 





















 









 


















 










































 









 









 














   





 
typedef struct
{
  volatile uint32_t CTRL;                          
  volatile uint32_t LOAD;                          
  volatile uint32_t VAL;                           
  volatile const  uint32_t CALIB;                         
} SysTick_Type;

 












 



 



 








   





 
typedef struct
{
  volatile  union
  {
    volatile  uint8_t    u8;                        
    volatile  uint16_t   u16;                       
    volatile  uint32_t   u32;                       
  }  PORT [32];                                
       uint32_t RESERVED0[864];
  volatile uint32_t TER;                           
       uint32_t RESERVED1[15];
  volatile uint32_t TPR;                           
       uint32_t RESERVED2[15];
  volatile uint32_t TCR;                           
       uint32_t RESERVED3[29];
  volatile uint32_t IWR;                           
  volatile uint32_t IRR;                           
  volatile uint32_t IMCR;                          
       uint32_t RESERVED4[43];
  volatile uint32_t LAR;                           
  volatile uint32_t LSR;                           
       uint32_t RESERVED5[6];
  volatile const  uint32_t PID4;                          
  volatile const  uint32_t PID5;                          
  volatile const  uint32_t PID6;                          
  volatile const  uint32_t PID7;                          
  volatile const  uint32_t PID0;                          
  volatile const  uint32_t PID1;                          
  volatile const  uint32_t PID2;                          
  volatile const  uint32_t PID3;                          
  volatile const  uint32_t CID0;                          
  volatile const  uint32_t CID1;                          
  volatile const  uint32_t CID2;                          
  volatile const  uint32_t CID3;                          
} ITM_Type;

 



 
























 



 



 



 








   





 
typedef struct
{
       uint32_t RESERVED0;
  volatile const  uint32_t ICTR;                          



       uint32_t RESERVED1;

} InterruptType_Type;

 



 








   


#line 614 ".\\inc\\core_cm1.h"





 
typedef struct
{
  volatile uint32_t DHCSR;                         
  volatile  uint32_t DCRSR;                         
  volatile uint32_t DCRDR;                         
  volatile uint32_t DEMCR;                         
} CoreDebug_Type;

 




































 






 






































   


 
#line 721 ".\\inc\\core_cm1.h"

#line 728 ".\\inc\\core_cm1.h"






   




 





#line 762 ".\\inc\\core_cm1.h"


 


 




#line 787 ".\\inc\\core_cm1.h"


 
 
 
 








 
extern uint32_t __get_PSP(void);








 
extern void __set_PSP(uint32_t topOfProcStack);








 
extern uint32_t __get_MSP(void);








 
extern void __set_MSP(uint32_t topOfMainStack);








 
extern uint32_t __REV16(uint16_t value);








 
extern int32_t __REVSH(int16_t value);


#line 937 ".\\inc\\core_cm1.h"





 








 






 






 
 





 







 
 





 






 
 





 






 
 





 






 
 





 






 
static __inline uint32_t __get_CONTROL(void)
{
  register uint32_t __regControl         __asm("control");
  return(__regControl);
}







 
static __inline void __set_CONTROL(uint32_t control)
{
  register uint32_t __regControl         __asm("control");
  __regControl = control;
}





#line 1464 ".\\inc\\core_cm1.h"







 
 


 











 
static __inline void NVIC_SetPriorityGrouping(uint32_t PriorityGroup)
{
  uint32_t reg_value;
  uint32_t PriorityGroupTmp = (PriorityGroup & 0x07);                          

  reg_value  =  ((SCB_Type *) ((0xE000E000) + 0x0D00))->AIRCR;                                                    
  reg_value &= ~((0xFFFFul << 16) | (7ul << 8));              
  reg_value  =  (reg_value                       |
                (0x5FA << 16) |
                (PriorityGroupTmp << 8));                                      
  ((SCB_Type *) ((0xE000E000) + 0x0D00))->AIRCR =  reg_value;
}








 
static __inline uint32_t NVIC_GetPriorityGrouping(void)
{
  return ((((SCB_Type *) ((0xE000E000) + 0x0D00))->AIRCR & (7ul << 8)) >> 8);    
}








 
static __inline void NVIC_EnableIRQ(IRQn_Type IRQn)
{
  ((NVIC_Type *) ((0xE000E000) + 0x0100))->ISER[((uint32_t)(IRQn) >> 5)] = (1 << ((uint32_t)(IRQn) & 0x1F));  
}








 
static __inline void NVIC_DisableIRQ(IRQn_Type IRQn)
{
  ((NVIC_Type *) ((0xE000E000) + 0x0100))->ICER[((uint32_t)(IRQn) >> 5)] = (1 << ((uint32_t)(IRQn) & 0x1F));  
}









 
static __inline uint32_t NVIC_GetPendingIRQ(IRQn_Type IRQn)
{
  return((uint32_t) ((((NVIC_Type *) ((0xE000E000) + 0x0100))->ISPR[(uint32_t)(IRQn) >> 5] & (1 << ((uint32_t)(IRQn) & 0x1F)))?1:0));  
}








 
static __inline void NVIC_SetPendingIRQ(IRQn_Type IRQn)
{
  ((NVIC_Type *) ((0xE000E000) + 0x0100))->ISPR[((uint32_t)(IRQn) >> 5)] = (1 << ((uint32_t)(IRQn) & 0x1F));  
}








 
static __inline void NVIC_ClearPendingIRQ(IRQn_Type IRQn)
{
  ((NVIC_Type *) ((0xE000E000) + 0x0100))->ICPR[((uint32_t)(IRQn) >> 5)] = (1 << ((uint32_t)(IRQn) & 0x1F));  
}









 
static __inline uint32_t NVIC_GetActive(IRQn_Type IRQn)
{
  return((uint32_t)((((NVIC_Type *) ((0xE000E000) + 0x0100))->IABR[(uint32_t)(IRQn) >> 5] & (1 << ((uint32_t)(IRQn) & 0x1F)))?1:0));  
}












 
static __inline void NVIC_SetPriority(IRQn_Type IRQn, uint32_t priority)
{
  if(IRQn < 0) {
    ((SCB_Type *) ((0xE000E000) + 0x0D00))->SHP[((uint32_t)(IRQn) & 0xF)-4] = ((priority << (8 - 4)) & 0xff); }  
  else {
    ((NVIC_Type *) ((0xE000E000) + 0x0100))->IP[(uint32_t)(IRQn)] = ((priority << (8 - 4)) & 0xff);    }         
}















 
static __inline uint32_t NVIC_GetPriority(IRQn_Type IRQn)
{

  if(IRQn < 0) {
    return((uint32_t)(((SCB_Type *) ((0xE000E000) + 0x0D00))->SHP[((uint32_t)(IRQn) & 0xF)-4] >> (8 - 4)));  }  
  else {
    return((uint32_t)(((NVIC_Type *) ((0xE000E000) + 0x0100))->IP[(uint32_t)(IRQn)]           >> (8 - 4)));  }  
}
















 
static __inline uint32_t NVIC_EncodePriority (uint32_t PriorityGroup, uint32_t PreemptPriority, uint32_t SubPriority)
{
  uint32_t PriorityGroupTmp = (PriorityGroup & 0x07);           
  uint32_t PreemptPriorityBits;
  uint32_t SubPriorityBits;

  PreemptPriorityBits = ((7 - PriorityGroupTmp) > 4) ? 4 : 7 - PriorityGroupTmp;
  SubPriorityBits     = ((PriorityGroupTmp + 4) < 7) ? 0 : PriorityGroupTmp - 7 + 4;

  return (
           ((PreemptPriority & ((1 << (PreemptPriorityBits)) - 1)) << SubPriorityBits) |
           ((SubPriority     & ((1 << (SubPriorityBits    )) - 1)))
         );
}
















 
static __inline void NVIC_DecodePriority (uint32_t Priority, uint32_t PriorityGroup, uint32_t* pPreemptPriority, uint32_t* pSubPriority)
{
  uint32_t PriorityGroupTmp = (PriorityGroup & 0x07);           
  uint32_t PreemptPriorityBits;
  uint32_t SubPriorityBits;

  PreemptPriorityBits = ((7 - PriorityGroupTmp) > 4) ? 4 : 7 - PriorityGroupTmp;
  SubPriorityBits     = ((PriorityGroupTmp + 4) < 7) ? 0 : PriorityGroupTmp - 7 + 4;

  *pPreemptPriority = (Priority >> SubPriorityBits) & ((1 << (PreemptPriorityBits)) - 1);
  *pSubPriority     = (Priority                   ) & ((1 << (SubPriorityBits    )) - 1);
}


 












 
static __inline uint32_t SysTick_Config(uint32_t ticks)
{
  if (ticks > (0xFFFFFFul << 0))  return (1);             

  ((SysTick_Type *) ((0xE000E000) + 0x0010))->LOAD  = (ticks & (0xFFFFFFul << 0)) - 1;       
  NVIC_SetPriority (SysTick_IRQn, (1<<4) - 1);   
  ((SysTick_Type *) ((0xE000E000) + 0x0010))->VAL   = 0;                                           
  ((SysTick_Type *) ((0xE000E000) + 0x0010))->CTRL  = (1ul << 2) |
                   (1ul << 1)   |
                   (1ul << 0);                     
  return (0);                                                   
}




 





 
static __inline void NVIC_SystemReset(void)
{
  ((SCB_Type *) ((0xE000E000) + 0x0D00))->AIRCR  = ((0x5FA << 16)      |
                 (((SCB_Type *) ((0xE000E000) + 0x0D00))->AIRCR & (7ul << 8)) |
                 (1ul << 2));                    
  __dsb(0);                                                      
  while(1);                                                     
}


   



 






 
 

extern volatile int ITM_RxBuffer;                     












 
static __inline uint32_t ITM_SendChar (uint32_t ch)
{
  if ((((CoreDebug_Type *) (0xE000EDF0))->DEMCR & (1ul << 24))  &&       
      (((ITM_Type *) (0xE0000000))->TCR & (1ul << 0))                  &&       
      (((ITM_Type *) (0xE0000000))->TER & (1ul << 0)        )                    )      
  {
    while (((ITM_Type *) (0xE0000000))->PORT[0].u32 == 0);
    ((ITM_Type *) (0xE0000000))->PORT[0].u8 = (uint8_t) ch;
  }
  return (ch);
}










 
static __inline int ITM_ReceiveChar (void) {
  int ch = -1;                                

  if (ITM_RxBuffer != 0x5AA55AA5) {
    ch = ITM_RxBuffer;
    ITM_RxBuffer = 0x5AA55AA5;        
  }

  return (ch);
}









 
static __inline int ITM_CheckChar (void) {

  if (ITM_RxBuffer == 0x5AA55AA5) {
    return (0);                                  
  } else {
    return (1);                                  
  }
}


   






   



 
#line 82 ".\\inc\\opora.h"
#line 1 ".\\inc\\opora_can_defs.h"






















 

 





 



 



 



 

typedef struct {
  volatile uint32_t ID;
  volatile uint32_t DLC;
  volatile uint32_t DATAL;
  volatile uint32_t DATAH;
} _CAN_BUF_TypeDef;



 

typedef struct {
  volatile uint32_t MASK;
  volatile uint32_t FILTER;
} _CAN_BUF_TypeDef0;



 



 

typedef struct {
  volatile uint32_t CONTROL;
  volatile uint32_t STATUS;
  volatile uint32_t BITTMNG;
       uint32_t RESERVED0;
  volatile uint32_t INT_EN;
       uint32_t RESERVED1[2];
  volatile uint32_t OVER;
  volatile uint32_t RXID;
  volatile uint32_t RXDLC;
  volatile uint32_t RXDATAL;
  volatile uint32_t RXDATAH;
  volatile uint32_t TXID;
  volatile uint32_t TXDLC;
  volatile uint32_t DATAL;
  volatile uint32_t DATAH;
  volatile uint32_t BUF_01_CON;
  volatile uint32_t BUF_02_CON;
  volatile uint32_t BUF_03_CON;
  volatile uint32_t BUF_04_CON;
  volatile uint32_t BUF_05_CON;
  volatile uint32_t BUF_06_CON;
  volatile uint32_t BUF_07_CON;
  volatile uint32_t BUF_08_CON;
  volatile uint32_t BUF_09_CON;
  volatile uint32_t BUF_10_CON;
  volatile uint32_t BUF_11_CON;
  volatile uint32_t BUF_12_CON;
  volatile uint32_t BUF_13_CON;
  volatile uint32_t BUF_14_CON;
  volatile uint32_t BUF_15_CON;
  volatile uint32_t BUF_16_CON;
  volatile uint32_t BUF_17_CON;
  volatile uint32_t BUF_18_CON;
  volatile uint32_t BUF_19_CON;
  volatile uint32_t BUF_20_CON;
  volatile uint32_t BUF_21_CON;
  volatile uint32_t BUF_22_CON;
  volatile uint32_t BUF_23_CON;
  volatile uint32_t BUF_24_CON;
  volatile uint32_t BUF_25_CON;
  volatile uint32_t BUF_26_CON;
  volatile uint32_t BUF_27_CON;
  volatile uint32_t BUF_28_CON;
  volatile uint32_t BUF_29_CON;
  volatile uint32_t BUF_30_CON;
  volatile uint32_t BUF_31_CON;
  volatile uint32_t BUF_32_CON;
  volatile uint32_t INT_RX;
  volatile uint32_t RX;
  volatile uint32_t INT_TX;
  volatile uint32_t TX;
       uint32_t RESERVED2[76];
  volatile uint32_t BUF_01_ID;
  volatile uint32_t BUF_01_DLC;
  volatile uint32_t BUF_01_DATAL;
  volatile uint32_t BUF_01_DATAH;
  volatile uint32_t BUF_02_ID;
  volatile uint32_t BUF_02_DLC;
  volatile uint32_t BUF_02_DATAL;
  volatile uint32_t BUF_02_DATAH;
  volatile uint32_t BUF_03_ID;
  volatile uint32_t BUF_03_DLC;
  volatile uint32_t BUF_03_DATAL;
  volatile uint32_t BUF_03_DATAH;
  volatile uint32_t BUF_04_ID;
  volatile uint32_t BUF_04_DLC;
  volatile uint32_t BUF_04_DATAL;
  volatile uint32_t BUF_04_DATAH;
  volatile uint32_t BUF_05_ID;
  volatile uint32_t BUF_05_DLC;
  volatile uint32_t BUF_05_DATAL;
  volatile uint32_t BUF_05_DATAH;
  volatile uint32_t BUF_06_ID;
  volatile uint32_t BUF_06_DLC;
  volatile uint32_t BUF_06_DATAL;
  volatile uint32_t BUF_06_DATAH;
  volatile uint32_t BUF_07_ID;
  volatile uint32_t BUF_07_DLC;
  volatile uint32_t BUF_07_DATAL;
  volatile uint32_t BUF_07_DATAH;
  volatile uint32_t BUF_08_ID;
  volatile uint32_t BUF_08_DLC;
  volatile uint32_t BUF_08_DATAL;
  volatile uint32_t BUF_08_DATAH;
  volatile uint32_t BUF_09_ID;
  volatile uint32_t BUF_09_DLC;
  volatile uint32_t BUF_09_DATAL;
  volatile uint32_t BUF_09_DATAH;
  volatile uint32_t BUF_10_ID;
  volatile uint32_t BUF_10_DLC;
  volatile uint32_t BUF_10_DATAL;
  volatile uint32_t BUF_10_DATAH;
  volatile uint32_t BUF_11_ID;
  volatile uint32_t BUF_11_DLC;
  volatile uint32_t BUF_11_DATAL;
  volatile uint32_t BUF_11_DATAH;
  volatile uint32_t BUF_12_ID;
  volatile uint32_t BUF_12_DLC;
  volatile uint32_t BUF_12_DATAL;
  volatile uint32_t BUF_12_DATAH;
  volatile uint32_t BUF_13_ID;
  volatile uint32_t BUF_13_DLC;
  volatile uint32_t BUF_13_DATAL;
  volatile uint32_t BUF_13_DATAH;
  volatile uint32_t BUF_14_ID;
  volatile uint32_t BUF_14_DLC;
  volatile uint32_t BUF_14_DATAL;
  volatile uint32_t BUF_14_DATAH;
  volatile uint32_t BUF_15_ID;
  volatile uint32_t BUF_15_DLC;
  volatile uint32_t BUF_15_DATAL;
  volatile uint32_t BUF_15_DATAH;
  volatile uint32_t BUF_16_ID;
  volatile uint32_t BUF_16_DLC;
  volatile uint32_t BUF_16_DATAL;
  volatile uint32_t BUF_16_DATAH;
  volatile uint32_t BUF_17_ID;
  volatile uint32_t BUF_17_DLC;
  volatile uint32_t BUF_17_DATAL;
  volatile uint32_t BUF_17_DATAH;
  volatile uint32_t BUF_18_ID;
  volatile uint32_t BUF_18_DLC;
  volatile uint32_t BUF_18_DATAL;
  volatile uint32_t BUF_18_DATAH;
  volatile uint32_t BUF_19_ID;
  volatile uint32_t BUF_19_DLC;
  volatile uint32_t BUF_19_DATAL;
  volatile uint32_t BUF_19_DATAH;
  volatile uint32_t BUF_20_ID;
  volatile uint32_t BUF_20_DLC;
  volatile uint32_t BUF_20_DATAL;
  volatile uint32_t BUF_20_DATAH;
  volatile uint32_t BUF_21_ID;
  volatile uint32_t BUF_21_DLC;
  volatile uint32_t BUF_21_DATAL;
  volatile uint32_t BUF_21_DATAH;
  volatile uint32_t BUF_22_ID;
  volatile uint32_t BUF_22_DLC;
  volatile uint32_t BUF_22_DATAL;
  volatile uint32_t BUF_22_DATAH;
  volatile uint32_t BUF_23_ID;
  volatile uint32_t BUF_23_DLC;
  volatile uint32_t BUF_23_DATAL;
  volatile uint32_t BUF_23_DATAH;
  volatile uint32_t BUF_24_ID;
  volatile uint32_t BUF_24_DLC;
  volatile uint32_t BUF_24_DATAL;
  volatile uint32_t BUF_24_DATAH;
  volatile uint32_t BUF_25_ID;
  volatile uint32_t BUF_25_DLC;
  volatile uint32_t BUF_25_DATAL;
  volatile uint32_t BUF_25_DATAH;
  volatile uint32_t BUF_26_ID;
  volatile uint32_t BUF_26_DLC;
  volatile uint32_t BUF_26_DATAL;
  volatile uint32_t BUF_26_DATAH;
  volatile uint32_t BUF_27_ID;
  volatile uint32_t BUF_27_DLC;
  volatile uint32_t BUF_27_DATAL;
  volatile uint32_t BUF_27_DATAH;
  volatile uint32_t BUF_28_ID;
  volatile uint32_t BUF_28_DLC;
  volatile uint32_t BUF_28_DATAL;
  volatile uint32_t BUF_28_DATAH;
  volatile uint32_t BUF_29_ID;
  volatile uint32_t BUF_29_DLC;
  volatile uint32_t BUF_29_DATAL;
  volatile uint32_t BUF_29_DATAH;
  volatile uint32_t BUF_30_ID;
  volatile uint32_t BUF_30_DLC;
  volatile uint32_t BUF_30_DATAL;
  volatile uint32_t BUF_30_DATAH;
  volatile uint32_t BUF_31_ID;
  volatile uint32_t BUF_31_DLC;
  volatile uint32_t BUF_31_DATAL;
  volatile uint32_t BUF_31_DATAH;
  volatile uint32_t BUF_32_ID;
  volatile uint32_t BUF_32_DLC;
  volatile uint32_t BUF_32_DATAL;
  volatile uint32_t BUF_32_DATAH;
       uint32_t RESERVED3[64];
  volatile uint32_t BUF_01_MASK;
  volatile uint32_t BUF_01_FILTER;
  volatile uint32_t BUF_02_MASK;
  volatile uint32_t BUF_02_FILTER;
  volatile uint32_t BUF_03_MASK;
  volatile uint32_t BUF_03_FILTER;
  volatile uint32_t BUF_04_MASK;
  volatile uint32_t BUF_04_FILTER;
  volatile uint32_t BUF_05_MASK;
  volatile uint32_t BUF_05_FILTER;
  volatile uint32_t BUF_06_MASK;
  volatile uint32_t BUF_06_FILTER;
  volatile uint32_t BUF_07_MASK;
  volatile uint32_t BUF_07_FILTER;
  volatile uint32_t BUF_08_MASK;
  volatile uint32_t BUF_08_FILTER;
  volatile uint32_t BUF_09_MASK;
  volatile uint32_t BUF_09_FILTER;
  volatile uint32_t BUF_10_MASK;
  volatile uint32_t BUF_10_FILTER;
  volatile uint32_t BUF_11_MASK;
  volatile uint32_t BUF_11_FILTER;
  volatile uint32_t BUF_12_MASK;
  volatile uint32_t BUF_12_FILTER;
  volatile uint32_t BUF_13_MASK;
  volatile uint32_t BUF_13_FILTER;
  volatile uint32_t BUF_14_MASK;
  volatile uint32_t BUF_14_FILTER;
  volatile uint32_t BUF_15_MASK;
  volatile uint32_t BUF_15_FILTER;
  volatile uint32_t BUF_16_MASK;
  volatile uint32_t BUF_16_FILTER;
  volatile uint32_t BUF_17_MASK;
  volatile uint32_t BUF_17_FILTER;
  volatile uint32_t BUF_18_MASK;
  volatile uint32_t BUF_18_FILTER;
  volatile uint32_t BUF_19_MASK;
  volatile uint32_t BUF_19_FILTER;
  volatile uint32_t BUF_20_MASK;
  volatile uint32_t BUF_20_FILTER;
  volatile uint32_t BUF_21_MASK;
  volatile uint32_t BUF_21_FILTER;
  volatile uint32_t BUF_22_MASK;
  volatile uint32_t BUF_22_FILTER;
  volatile uint32_t BUF_23_MASK;
  volatile uint32_t BUF_23_FILTER;
  volatile uint32_t BUF_24_MASK;
  volatile uint32_t BUF_24_FILTER;
  volatile uint32_t BUF_25_MASK;
  volatile uint32_t BUF_25_FILTER;
  volatile uint32_t BUF_26_MASK;
  volatile uint32_t BUF_26_FILTER;
  volatile uint32_t BUF_27_MASK;
  volatile uint32_t BUF_27_FILTER;
  volatile uint32_t BUF_28_MASK;
  volatile uint32_t BUF_28_FILTER;
  volatile uint32_t BUF_29_MASK;
  volatile uint32_t BUF_29_FILTER;
  volatile uint32_t BUF_30_MASK;
  volatile uint32_t BUF_30_FILTER;
  volatile uint32_t BUF_31_MASK;
  volatile uint32_t BUF_31_FILTER;
  volatile uint32_t BUF_32_MASK;
  volatile uint32_t BUF_32_FILTER;
} CAN_TypeDef;

   

   



 



 

















   

   



 



 












































   

   



 



 




















   

   



 



 

















   

   



 



 








   

   



 



 




















   

   



 



 














   

   



 



 














   

   



 



 








   

   



 



 




















   

   



 



 














   

   



 



 














   

   



 



 


























   

   



 



 








   

   



 



 




















   

   



 



 














   

   



 



 














   

   

   

   

   





 
#line 83 ".\\inc\\opora.h"
#line 1 ".\\inc\\opora_usb_defs.h"






















 

 





 



 



 



 

typedef struct {
  volatile uint32_t CTRL;
  volatile uint32_t STS;
  volatile uint32_t TS;
  volatile uint32_t NTS;
} _USB_SEP_TypeDef;



 

typedef struct {
  volatile uint32_t RXFD;
       uint32_t RESERVED0;
  volatile uint32_t RXFDC_L;
  volatile uint32_t RXFDC_H;
  volatile uint32_t RXFC;
       uint32_t RESERVED1[11];
  volatile uint32_t TXFD;
       uint32_t RESERVED2[3];
  volatile uint32_t TXFDC;
  volatile uint32_t RESERVED6[11];
} _USB_SEP_TypeDef0;



 



 

typedef struct {
  volatile uint32_t HTXC;
  volatile uint32_t HTXT;
  volatile uint32_t HTXLC;
  volatile uint32_t HTXSE;
  volatile uint32_t HTXA;
  volatile uint32_t HTXE;
  volatile uint32_t HFN_L;
  volatile uint32_t HFN_H;
  volatile uint32_t HIS;
  volatile uint32_t HIM;
  volatile uint32_t HRXS;
  volatile uint32_t HRXP;
  volatile uint32_t HRXA;
  volatile uint32_t HRXE;
  volatile uint32_t HRXCS;
  volatile uint32_t HSTM;
       uint32_t RESERVED0[16];
  volatile uint32_t HRXFD;
       uint32_t RESERVED1;
  volatile uint32_t HRXFDC_L;
  volatile uint32_t HRXFDC_H;
  volatile uint32_t HRXFC;
       uint32_t RESERVED2[11];
  volatile uint32_t HTXFD;
       uint32_t RESERVED3[3];
  volatile uint32_t HTXFDC;
       uint32_t RESERVED4[11];
  _USB_SEP_TypeDef USB_SEP[4];
  volatile uint32_t SC;
  volatile uint32_t SLS;
  volatile uint32_t SIS;
  volatile uint32_t SIM;
  volatile uint32_t SA;
  volatile uint32_t SFN_L;
  volatile uint32_t SFN_H;
       uint32_t RESERVED5[9];
  _USB_SEP_TypeDef0 USB_SEP_FIFO[4];
  volatile uint32_t HSCR;
  volatile uint32_t HSVR;
} USB_TypeDef;

   

   



 



 














   

   



 



 














   

   



 



 














   

   



 



 














   

   



 



 


























   

   



 



 

















   

   



 



 


























   

   



 



 

















   

   



 



 

















   

   



 



 

















   

   



 



 


























   

   



 



 








   

   

   

   

   





 
#line 84 ".\\inc\\opora.h"
#line 1 ".\\inc\\opora_eeprom_defs.h"






















 

 





 



 



 



 



 

typedef struct {
  volatile uint32_t CMD;
  volatile uint32_t ADR;
  volatile uint32_t DI;
  volatile uint32_t DO;
  volatile uint32_t KEY;
} EEPROM_TypeDef;

   

   



 



 






































   

   

   

   

   





 
#line 85 ".\\inc\\opora.h"
#line 1 ".\\inc\\opora_rst_clk_defs.h"






















 

 





 



 



 



 



 

typedef struct {
  volatile uint32_t CLOCK_STATUS;
  volatile uint32_t PLL_CONTROL;
  volatile uint32_t HS_CONTROL;
  volatile uint32_t CPU_CLOCK;
  volatile uint32_t USB_CLOCK;
  volatile uint32_t ADC_MCO_CLOCK;
  volatile uint32_t RTC_CLOCK;
  volatile uint32_t PER_CLOCK;
  volatile uint32_t CAN_CLOCK;
  volatile uint32_t TIM_CLOCK;
  volatile uint32_t UART_CLOCK;
  volatile uint32_t SSP_CLOCK;
       uint32_t RESERVED;
  volatile uint32_t ETH_CLOCK;
} RST_CLK_TypeDef;

   

   



 



 











   

   



 



 




















   

   



 



 








   

   



 



 














   

   



 



 














   

   



 



 

















   

   



 



 














   

   



 



 














   

   



 



 




















   

   



 



 














   

   



 



 














   

   

   

   

   





 
#line 86 ".\\inc\\opora.h"
#line 1 ".\\inc\\opora_dma_defs.h"






















 

 





 



 



 



 



 

typedef struct {
  volatile uint32_t STATUS;
  volatile uint32_t CFG;
  volatile uint32_t CTRL_BASE_PTR;
  volatile uint32_t ALT_CTRL_BASE_PTR;
  volatile uint32_t WAITONREQ_STATUS;
  volatile uint32_t CHNL_SW_REQUEST;
  volatile uint32_t CHNL_USEBURST_SET;
  volatile uint32_t CHNL_USEBURST_CLR;
  volatile uint32_t CHNL_REQ_MASK_SET;
  volatile uint32_t CHNL_REQ_MASK_CLR;
  volatile uint32_t CHNL_ENABLE_SET;
  volatile uint32_t CHNL_ENABLE_CLR;
  volatile uint32_t CHNL_PRI_ALT_SET;
  volatile uint32_t CHNL_PRI_ALT_CLR;
  volatile uint32_t CHNL_PRIORITY_SET;
  volatile uint32_t CHNL_PRIORITY_CLR;
       uint32_t RESERVED0[3];
  volatile uint32_t ERR_CLR;
} DMA_TypeDef;

   

   



 



 














   

   



 



 








   

   

   

   

   





 
#line 87 ".\\inc\\opora.h"
#line 1 ".\\inc\\opora_uart_defs.h"






















 

 





 



 



 



 



 

typedef struct {
  volatile uint32_t DR;
  volatile uint32_t RSR_ECR;
       uint32_t RESERVED0[4];
  volatile uint32_t FR;
       uint32_t RESERVED1;
  volatile uint32_t ILPR;
  volatile uint32_t IBRD;
  volatile uint32_t FBRD;
  volatile uint32_t LCR_H;
  volatile uint32_t CR;
  volatile uint32_t IFLS;
  volatile uint32_t IMSC;
  volatile uint32_t RIS;
  volatile uint32_t MIS;
  volatile uint32_t ICR;
  volatile uint32_t DMACR;
} UART_TypeDef;

   

   



 



 

















   

   



 



 














   

   



 



 





























   

   



 



 























   

   



 



 






































   

   



 



 








   

   



 



 



































   

   



 



 



































   

   



 



 



































   

   



 



 



































   

   



 



 











   

   

   

   

   





 
#line 88 ".\\inc\\opora.h"
#line 1 ".\\inc\\opora_spi_defs.h"






















 

 





 



 



 



 



 

typedef struct {
  volatile uint32_t SSPx_CR0;
  volatile uint32_t SSPx_CR1;
  volatile uint32_t SSPx_DR;
  volatile uint32_t SSPx_SR;
  volatile uint32_t SSPx_CPSR;
  volatile uint32_t SSPx_IMSC;
  volatile uint32_t SSPx_RIS;
  volatile uint32_t SSPx_MIS;
  volatile uint32_t SSPx_ICR;
  volatile uint32_t SSPx_DMACR;
} SPI_TypeDef;

   

   



 



 

















   

   



 



 














   

   



 



 

















   

   



 



 














   

   



 



 














   

   



 



 














   

   



 



 








   

   



 



 








   

   

   

   

   





 
#line 89 ".\\inc\\opora.h"
#line 1 ".\\inc\\opora_mil_std_1553_defs.h"






















 

 





 



 



 



 



 

typedef struct {
       uint32_t RESERVED0[1024];
  volatile uint32_t CONTROL;
  volatile uint32_t STATUS;
  volatile uint32_t ERROR;
  volatile uint32_t CommandWord1;
  volatile uint32_t CommandWord2;
  volatile uint32_t ModeData;
  volatile uint32_t StatusWord1;
  volatile uint32_t StatusWord2;
  volatile uint32_t INTEN;
  volatile uint32_t MSG;
} MIL_STD_1553_TypeDef;

   

   



 



 





























   

   



 



 














   

   



 



 























   

   



 



 














   

   

   

   

   





 
#line 90 ".\\inc\\opora.h"
#line 1 ".\\inc\\opora_power_defs.h"






















 

 





 



 



 



 



 

typedef struct {
  volatile uint32_t PVDCS;
} POWER_TypeDef;

   

   



 



 





























   

   

   

   

   





 
#line 91 ".\\inc\\opora.h"
#line 1 ".\\inc\\opora_wwdg_defs.h"






















 

 





 



 



 



 



 

typedef struct {
  volatile uint32_t CR;
  volatile uint32_t CFR;
  volatile uint32_t SR;
} WWDG_TypeDef;

   

   



 



 








   

   



 



 











   

   

   

   

   





 
#line 92 ".\\inc\\opora.h"
#line 1 ".\\inc\\opora_iwdg_defs.h"






















 

 





 



 



 



 



 

typedef struct {
  volatile uint32_t KR;
  volatile uint32_t PR;
  volatile uint32_t PRL;
  volatile uint32_t SR;
} IWDG_TypeDef;

   

   



 



 








   

   

   

   

   





 
#line 93 ".\\inc\\opora.h"
#line 1 ".\\inc\\opora_timer_defs.h"






















 

 





 



 



 



 



 

typedef struct {
  volatile uint32_t CNT;
  volatile uint32_t PSG;
  volatile uint32_t ARR;
  volatile uint32_t CNTRL;
  volatile uint32_t CCR1;
  volatile uint32_t CCR2;
  volatile uint32_t CCR3;
  volatile uint32_t CCR4;
  volatile uint32_t CH1_CNTRL;
  volatile uint32_t CH2_CNTRL;
  volatile uint32_t CH3_CNTRL;
  volatile uint32_t CH4_CNTRL;
  volatile uint32_t CH1_CNTRL1;
  volatile uint32_t CH2_CNTRL1;
  volatile uint32_t CH3_CNTRL1;
  volatile uint32_t CH4_CNTRL1;
  volatile uint32_t CH1_DTG;
  volatile uint32_t CH2_DTG;
  volatile uint32_t CH3_DTG;
  volatile uint32_t CH4_DTG;
  volatile uint32_t BRKETR_CNTRL;
  volatile uint32_t STATUS;
  volatile uint32_t IE;
  volatile uint32_t DMA_RE;
  volatile uint32_t CH1_CNTRL2;
  volatile uint32_t CH2_CNTRL2;
  volatile uint32_t CH3_CNTRL2;
  volatile uint32_t CH4_CNTRL2;
  volatile uint32_t CCR11;
  volatile uint32_t CCR21;
  volatile uint32_t CCR31;
  volatile uint32_t CCR41;
} TIMER_TypeDef;

   

   



 



 























   

   



 



 





























   

   



 



 




















   

   



 



 











   

   



 



 














   

   



 



 


























   

   



 



 























   

   



 



 























   

   



 



 











   

   

   

   

   





 
#line 94 ".\\inc\\opora.h"
#line 1 ".\\inc\\opora_adc_defs.h"






















 

 





 



 



 



 



 

typedef struct {
  volatile uint32_t ADC1_CFG;
  volatile uint32_t ADC2_CFG;
  volatile uint32_t ADC1_H_LEVEL;
  volatile uint32_t ADC2_H_LEVEL;
  volatile uint32_t ADC1_L_LEVEL;
  volatile uint32_t ADC2_L_LEVEL;
  volatile uint32_t ADC1_RESULT;
  volatile uint32_t ADC2_RESULT;
  volatile uint32_t ADC1_STATUS;
  volatile uint32_t ADC2_STATUS;
  volatile uint32_t ADC1_CHSEL;
  volatile uint32_t ADC2_CHSEL;
} ADC_TypeDef;

   

   



 



 





















































   

   



 



 






































   

   



 



 








   

   



 



 








   

   



 



 

















   

   



 



 

















   

   

   

   

   





 
#line 95 ".\\inc\\opora.h"
#line 1 ".\\inc\\opora_dac_defs.h"






















 

 





 



 



 



 



 

typedef struct {
  volatile uint32_t CFG;
  volatile uint32_t DAC1_DATA;
  volatile uint32_t DAC2_DATA;
} DAC_TypeDef;

   

   



 



 

















   

   



 



 








   

   



 



 








   

   

   

   

   





 
#line 96 ".\\inc\\opora.h"
#line 1 ".\\inc\\opora_port_defs.h"






















 

 





 



 



 



 



 

typedef struct {
  volatile uint32_t RXTX;
  volatile uint32_t OE;
  volatile uint32_t FUNC;
  volatile uint32_t ANALOG;
  volatile uint32_t PULL;
  volatile uint32_t PD;
  volatile uint32_t PWR;
  volatile uint32_t GFEN;
  volatile uint32_t SETTX;
  volatile uint32_t CLRTX;
  volatile uint32_t RDTX;
} PORT_TypeDef;

   

   



 



 


















































   

   



 



 


















































   

   

   

   

   





 
#line 97 ".\\inc\\opora.h"
#line 1 ".\\inc\\opora_arinc429r_defs.h"






















 

 





 



 



 



 



 

typedef struct {
  volatile uint32_t CONTROL1;
  volatile uint32_t CONTROL2;
  volatile uint32_t CONTROL3;
  volatile uint32_t STATUS1;
  volatile uint32_t STATUS2;
       uint32_t RESERVED0[2];
  volatile uint32_t CHANNEL;
  volatile uint32_t LABEL;
  volatile uint32_t DATA_R;
} ARINC429R_TypeDef;

   

   



 



 





















































   

   



 



 





















































   

   



 



 






























































   

   



 



 


















































   

   



 



 


















































   

   

   

   

   





 
#line 98 ".\\inc\\opora.h"
#line 1 ".\\inc\\opora_bkp_defs.h"






















 

 





 



 



 



 



 

typedef struct {
  volatile uint32_t REG_00;
  volatile uint32_t REG_01;
  volatile uint32_t REG_02;
  volatile uint32_t REG_03;
  volatile uint32_t REG_04;
  volatile uint32_t REG_05;
  volatile uint32_t REG_06;
  volatile uint32_t REG_07;
  volatile uint32_t REG_08;
  volatile uint32_t REG_09;
  volatile uint32_t REG_0A;
  volatile uint32_t REG_0B;
  volatile uint32_t REG_0C;
  volatile uint32_t REG_0D;
  volatile uint32_t REG_0E;
  volatile uint32_t REG_0F;
  volatile uint32_t RTC_CNT;
  volatile uint32_t RTC_DIV;
  volatile uint32_t RTC_PRL;
  volatile uint32_t RTC_ALRM;
  volatile uint32_t RTC_CS;
} BKP_TypeDef;

   

   



 



 























   

   



 



 















































   

   



 



 























   

   

   

   

   





 
#line 99 ".\\inc\\opora.h"
#line 1 ".\\inc\\opora_arinc429t_defs.h"






















 

 





 



 



 



 



 

typedef struct {
  volatile uint32_t CONTROL1;
  volatile uint32_t CONTROL2;
  volatile uint32_t STATUS;
  volatile uint32_t DATA1_T;
  volatile uint32_t DATA2_T;
  volatile uint32_t DATA3_T;
  volatile uint32_t DATA4_T;
} ARINC429T_TypeDef;

   

   



 



 















































   

   



 



 












































   

   



 



 






































   

   

   

   

   





 
#line 100 ".\\inc\\opora.h"
#line 1 ".\\inc\\opora_ext_bus_cntrl_defs.h"






















 

 





 



 



 



 



 

typedef struct {
       uint32_t RESERVED0[20];
  volatile uint32_t NAND_CYCLES;
  volatile uint32_t EXT_BUS_CONTROL;
} EXT_BUS_CNTRL_TypeDef;

   

   



 



 























   

   



 



 




















   

   

   

   

   





 
#line 101 ".\\inc\\opora.h"
#line 1 ".\\inc\\opora_ethernet_defs.h"






















 

 





 



 



 



 



 

typedef struct {
  volatile uint16_t Delimiter;	
  volatile uint16_t MAC_T;		
  volatile uint16_t MAC_M;		
  volatile uint16_t MAC_H;		
  volatile uint16_t HASH0;		
  volatile uint16_t HASH1;		
  volatile uint16_t HASH2;		
  volatile uint16_t HASH3;		
  volatile uint16_t IPG;		
  volatile uint16_t PSC;		
  volatile uint16_t BAG;		
  volatile uint16_t JitterWnd;	
  volatile uint16_t R_CFG;		
  volatile uint16_t X_CFG;		
  volatile uint32_t G_CFG;		
  volatile uint16_t IMR;		
  volatile uint16_t IFR;		
  volatile uint16_t MDIO_CTRL;	
  volatile uint16_t MDIO_DATA;	
  volatile uint16_t R_Head;		
  volatile uint16_t X_Tail;		
  volatile uint16_t R_Tail;		
  volatile uint16_t X_Head;		
  volatile uint16_t STAT;		
  volatile uint16_t RESERV;		
  volatile uint16_t PHY_CTRL;	
  volatile uint16_t PHY_STATUS;	
} ETHERNET_TypeDef;

   

   



 



 






































   

   



 



 





























   

   



 



 


























   

   



 



 












































   

   



 



 












































   

   



 



 

















   

   



 



 
































   

   

   

   

   





 
#line 102 ".\\inc\\opora.h"
#line 1 ".\\inc\\opora_sys_defs.h"






















 

 





 



 



 



 



 

typedef struct {				 
       uint32_t RESERVED0[2];	 
  volatile uint32_t ACTLR;			 
       uint32_t RESERVED1;		 
  volatile uint32_t STCSR;			 
  volatile uint32_t STRVR;			 
  volatile uint32_t STCVR;			 
  volatile uint32_t STCR;			 
       uint32_t RESERVED2[56];	 
  volatile uint32_t ISER;			 
       uint32_t RESERVED3[31];	 
  volatile uint32_t ICER;			 
       uint32_t RESERVED4[31];	 
  volatile uint32_t ISPR;			 
       uint32_t RESERVED5[31];	 
  volatile uint32_t ICPR;			 
       uint32_t RESERVED6[95];	 
  volatile uint32_t IPR0;			 
  volatile uint32_t IPR1;			 
  volatile uint32_t IPR2;			 
  volatile uint32_t IPR3;			 
  volatile uint32_t IPR4;			 
  volatile uint32_t IPR5;			 
  volatile uint32_t IPR6;			 
  volatile uint32_t IPR7;			 
       uint32_t RESERVED7[568];	 
  volatile uint32_t CPUID;			 
  volatile uint32_t ICSR;			 
       uint32_t RESERVED8;		 
  volatile uint32_t AIRCR;			 
       uint32_t RESERVED9;		 
  volatile uint32_t CCR;			 
       uint32_t RESERVED10;		 
  volatile uint32_t SHPR2;			 
  volatile uint32_t SHPR3;			 
  volatile uint32_t SHCSR;			 
} SYS_TypeDef;

   

   



 



 








   

   



 



 














   

   



 



 











   

   



 



 














   

   



 



 

















   

   



 



 





























   

   



 



 














   

   



 



 








   

   



 



 








   

   

   

   

   





 
#line 103 ".\\inc\\opora.h"



 

#line 142 ".\\inc\\opora.h"

   



 

#line 183 ".\\inc\\opora.h"

   





 
#line 26 "src\\1986BE9x_usb_device.c"
#line 1 ".\\inc\\opora_rst_clk.h"





















 

 



 
#line 30 ".\\inc\\opora_rst_clk.h"
#line 1 ".\\inc\\1986BE9x_lib.h"





















 

 



#line 48 ".\\inc\\1986BE9x_lib.h"



 






 

#line 31 ".\\inc\\opora_rst_clk.h"



 



 



 



 

typedef struct
{
  uint32_t CPU_CLK_Frequency;
  uint32_t USB_CLK_Frequency;
  uint32_t ADC_CLK_Frequency;
  uint32_t RTCHSI_Frequency;
  uint32_t RTCHSE_Frequency;
}RST_CLK_FreqTypeDef;



 

typedef struct {
     uint32_t REG_0F;
} Init_NonVolatile_RST_CLK_TypeDef;

   



 



 



 









   



 



 









   



 



 











   



 



 

#line 151 ".\\inc\\opora_rst_clk.h"



   




 



 











   



 



 

#line 201 ".\\inc\\opora_rst_clk.h"



   




 



 

#line 224 ".\\inc\\opora_rst_clk.h"

#line 234 ".\\inc\\opora_rst_clk.h"

   



 



 











   



 



 
#line 271 ".\\inc\\opora_rst_clk.h"

#line 279 ".\\inc\\opora_rst_clk.h"
   



 



 

#line 305 ".\\inc\\opora_rst_clk.h"



   




 



 



#line 353 ".\\inc\\opora_rst_clk.h"



   



 



 









#line 380 ".\\inc\\opora_rst_clk.h"

   






 



 

#line 410 ".\\inc\\opora_rst_clk.h"



   



 



 

#line 439 ".\\inc\\opora_rst_clk.h"



   

   



 

   



 

void RST_CLK_DeInit(void);
void RST_CLK_WarmDeInit(void);

void RST_CLK_HSEconfig(uint32_t RST_CLK_HSE);
ErrorStatus RST_CLK_HSEstatus(void);

void RST_CLK_LSEconfig(uint32_t RST_CLK_LSE);
ErrorStatus RST_CLK_LSEstatus(void);

void RST_CLK_HSIcmd(FunctionalState NewState);
void RST_CLK_HSIadjust(uint8_t HSItrimValue);
ErrorStatus RST_CLK_HSIstatus(void);

void RST_CLK_LSIcmd(FunctionalState NewState);
ErrorStatus RST_CLK_LSIstatus(void);

void RST_CLK_CPU_PLLconfig(uint32_t RST_CLK_CPU_PLLsource, uint32_t RST_CLK_CPU_PLLmul);
void RST_CLK_CPU_PLLuse(FunctionalState UsePLL);
void RST_CLK_CPU_PLLcmd(FunctionalState NewState);
ErrorStatus RST_CLK_CPU_PLLstatus(void);

void RST_CLK_CPUclkPrescaler(uint32_t CPUclkDivValue);
void RST_CLK_CPUclkSelection(uint32_t CPU_CLK);

void RST_CLK_USB_PLLconfig(uint32_t RST_CLK_USB_PLLsource, uint32_t RST_CLK_USB_PLLmul);
void RST_CLK_USB_PLLuse(FunctionalState UsePLL);
void RST_CLK_USB_PLLcmd(FunctionalState NewState);
ErrorStatus RST_CLK_USB_PLLstatus(void);

void RST_CLK_USBclkPrescaler(FunctionalState NewState);
void RST_CLK_USBclkEnable(FunctionalState NewState);

void RST_CLK_ADCclkSelection(uint32_t ADC_CLK);
void RST_CLK_ADCclkPrescaler(uint32_t ADCclkDivValue);
void RST_CLK_ADCclkEnable(FunctionalState NewState);

void RST_CLK_HSIclkPrescaler(uint32_t HSIclkDivValue);
void RST_CLK_RTC_HSIclkEnable(FunctionalState NewState);

void RST_CLK_HSEclkPrescaler(uint32_t HSEclkDivValue);
void RST_CLK_RTC_HSEclkEnable(FunctionalState NewState);

void RST_CLK_PCLKcmd(uint32_t RST_CLK_PCLK, FunctionalState NewState);

void RST_CLK_GetClocksFreq(RST_CLK_FreqTypeDef* RST_CLK_Clocks);

FlagStatus RST_CLK_GetFlagStatus(uint32_t RST_CLK_FLAG);

   

   

   





 

#line 27 "src\\1986BE9x_usb_device.c"
#line 1 ".\\inc\\1986BE9x_usb_handlers.h"


















 

 



 
#line 1 ".\\inc\\1986BE9x_usb_default_handlers.h"


















 

 




 
#line 1 ".\\inc\\1986BE9x_usb_cdc.h"

























 

 



 
#line 34 ".\\inc\\1986BE9x_usb_cdc.h"
#line 35 ".\\inc\\1986BE9x_usb_cdc.h"
#line 1 ".\\inc\\1986BE9x_usb_device.h"






















 

 



 
#line 31 ".\\inc\\1986BE9x_usb_device.h"
#line 32 ".\\inc\\1986BE9x_usb_device.h"
#line 1 ".\\inc\\opora_usb.h"





















 

 



 
#line 30 ".\\inc\\opora_usb.h"
#line 31 ".\\inc\\opora_usb.h"



 



 



 



 

typedef enum
{
  USB_EP0  = 0,
  USB_EP1  = 1,
  USB_EP2  = 2,
  USB_EP3  = 3,
  Num_USB_EndPoints
}USB_EP_TypeDef;





 

typedef struct
{
  uint32_t USB_USBC1_Source;       
 
  uint32_t USB_PLLUSBMUL;          
 
}USB_Clock_TypeDef;

typedef struct
{
  uint8_t USB_Version;                
  uint8_t USB_Revision;               
}USB_Version_TypeDef;

   



 



   



 








   



 

#line 120 ".\\inc\\opora_usb.h"









   



 



 

























#line 171 ".\\inc\\opora_usb.h"





   



 























   



 











   



 



























   



 







   



 














   



 























   



 

























#line 349 ".\\inc\\opora_usb.h"






   



 









   

#line 383 ".\\inc\\opora_usb.h"

   



 



 



























   



 

























#line 458 ".\\inc\\opora_usb.h"






   



 











   



 











   



 



















#line 526 ".\\inc\\opora_usb.h"






   



 









   



 



















   



 



























   

#line 611 ".\\inc\\opora_usb.h"

   




 

void USB_BRGInit(const USB_Clock_TypeDef* USB_Clock_InitStruct);
void USB_Reset(void);



 

uint32_t USB_GetHSCR(void);
void     USB_SetHSCR(uint32_t RegValue);

USB_Version_TypeDef USB_GetHSVR(void);



 

uint32_t USB_GetHTXC(void);
void     USB_SetHTXC(uint32_t RegValue);
uint32_t USB_GetHTXT(void);
void     USB_SetHTXT(uint32_t RegValue);
uint32_t USB_GetHTXLC(void);
void     USB_SetHTXLC(uint32_t RegValue);
uint32_t USB_GetHTXSE(void);
void     USB_SetHTXSE(uint32_t RegValue);
uint32_t USB_GetHTXA(void);
void     USB_SetHTXA(uint32_t RegValue);
uint32_t USB_GetHTXE(void);
void     USB_SetHTXE(uint32_t RegValue);
uint32_t USB_GetHFN(void);
uint32_t USB_GetHIS(void);
void     USB_SetHIS(uint32_t RegValue);
uint32_t USB_GetHIM(void);
void     USB_SetHIM(uint32_t RegValue);
uint32_t USB_GetHRXS(void);
uint32_t USB_GetHRXP(void);
uint32_t USB_GetHRXA(void);
uint32_t USB_GetHRXE(void);
uint32_t USB_GetHRXCS(void);
uint32_t USB_GetHSTM(void);
uint32_t USB_GetHRXFD(void);
uint32_t USB_GetHRXFDC(void);
uint32_t USB_GetHRXFC(void);
void     USB_SetHRXFC(uint32_t RegValue);
uint32_t USB_GetHTXFD(void);
void     USB_SetHTXFD(uint32_t RegValue);
uint32_t USB_GetHTXFC(void);
void     USB_SetHTXFC(uint32_t RegValue);



 

uint32_t USB_GetSEPxCTRL(USB_EP_TypeDef EndPointNumber);
void     USB_SetSEPxCTRL(USB_EP_TypeDef EndPointNumber, uint32_t RegValue);
uint32_t USB_GetSEPxSTS(USB_EP_TypeDef EndPointNumber);
uint32_t USB_GetSEPxTS(USB_EP_TypeDef EndPointNumber);
uint32_t USB_GetSEPxNTS(USB_EP_TypeDef EndPointNumber);
uint32_t USB_GetSC(void);
void     USB_SetSC(uint32_t RegValue);
uint32_t USB_GetSLS(void);
uint32_t USB_GetSIS(void);
void     USB_SetSIS(uint32_t RegValue);
uint32_t USB_GetSIM(void);
void     USB_SetSIM(uint32_t RegValue);
uint32_t USB_GetSA(void);
void     USB_SetSA(uint32_t RegValue);
uint32_t USB_GetSFN(void);
uint32_t USB_GetSEPxRXFD(USB_EP_TypeDef EndPointNumber);
uint32_t USB_GetSEPxRXFDC(USB_EP_TypeDef EndPointNumber);
uint32_t USB_GetSEPxRXFC(USB_EP_TypeDef EndPointNumber);
void     USB_SetSEPxRXFC(USB_EP_TypeDef EndPointNumber, uint32_t RegValue);
uint32_t USB_GetSEPxTXFD(USB_EP_TypeDef EndPointNumber);
void     USB_SetSEPxTXFD(USB_EP_TypeDef EndPointNumber, uint32_t RegValue);
uint32_t USB_GetSEPxTXFDC(USB_EP_TypeDef EndPointNumber);
void     USB_SetSEPxTXFDC(USB_EP_TypeDef EndPointNumber, uint32_t RegValue);
void     USB_SEPxToggleEPDATASEQ(USB_EP_TypeDef EndPointNumber);

   

   

   




 

#line 33 ".\\inc\\1986BE9x_usb_device.h"
#line 1 ".\\inc\\1986BE9x_config.h"

















 

 



#line 25 ".\\inc\\1986BE9x_config.h"
#line 26 ".\\inc\\1986BE9x_config.h"


 
 
 

 
 





 
#line 46 ".\\inc\\1986BE9x_config.h"



 
 


 


 

               

 

 


 
 
 



 


 
 
 
 
 

 
 

 



 
 


 
 

 







 

 
 








 
#line 126 ".\\inc\\1986BE9x_config.h"





 

#line 34 ".\\inc\\1986BE9x_usb_device.h"



 



 



 



 

typedef enum
{
  USB_HOST_TO_DEVICE  = 0x0,
  USB_DEVICE_TO_HOST  = 0x80
}USB_RequestTypeDT_TypeDef;



 

typedef enum
{
  USB_TYPE_STANDARD  = 0x0,
  USB_TYPE_CLASS     = 0x20,
  USB_TYPE_VENDOR    = 0x40
}USB_RequestType_TypeDef;






 

typedef enum
{
  USB_GET_STATUS = 0,
  USB_CLEAR_FEATURE,
  USB_Reserved0,
  USB_SET_FEATURE,
  USB_Reserved1,
  USB_SET_ADDRESS,
  USB_GET_DESCRIPTOR,
  USB_SET_DESCRIPTOR,
  USB_GET_CONFIGURATION,
  USB_SET_CONFIGURATION,
  USB_GET_INTERFACE,
  USB_SET_INTERFACE,
  USB_SYNCH_FRAME
} USB_Standard_Setup_TypeDef;



 

typedef enum
{
  USB_DEVICE = 1,
  USB_CONFIGURATION,
  USB_STRING,
  USB_INTERFACE,
  USB_ENDPOINT,
  USB_DEVICE_QUALIFIER,
  USB_OTHER_SPEED_CONFIGURATION,
  USB_INTERFACE_POWER
} USB_Standard_Descriptor_TypeDef;



 

typedef enum
{
  USB_ENDPOINT_HALT = 0,
  USB_DEVICE_REMOTE_WAKEUP,
  USB_TEST_MODE
} USB_Standard_Festure_Selector_TypeDef;



 

typedef enum
{
  USB_RECIPIENT_DEVICE    = 0x0,
  USB_RECIPIENT_INTERFACE = 0x1,
  USB_RECIPIENT_ENDPOINT  = 0x2,
  USB_RECIPIENT_OTHER     = 0x3
}USB_RequestRecipient_TypeDef;






 

typedef struct
{
  uint8_t  mRequestTypeData;             


 
  uint8_t  bRequest;                      
  uint16_t wValue;                        
  uint16_t wIndex;                        
  uint16_t wLength;                       
}USB_SetupPacket_TypeDef;

   



 



 



 

typedef enum
{
  USB_SUCCESS     = 0x0,              
  USB_ERROR       = 0x1,              
  USB_ERR_INV_REQ = 0x2,              
  USB_ERR_BUSY    = 0x200,            
}USB_Result;



 

typedef enum {USB_STALL_PROTO = 0x0, USB_STALL_HALT = 0x1} USB_StallType;



 

typedef USB_Result (*USB_EP_IO_Handler)(USB_EP_TypeDef EPx, uint8_t* Buffer, uint32_t Length);
typedef USB_Result (*USB_EP_Setup_Handler)(USB_EP_TypeDef EPx, const USB_SetupPacket_TypeDef* USB_SetupPacket);
typedef USB_Result (*USB_EP_Error_Handler)(USB_EP_TypeDef EPx, uint32_t STS, uint32_t TS, uint32_t CTRL);

   

   



 



   



 

   



 

USB_Result USB_EP_Init(USB_EP_TypeDef EPx, uint32_t USB_EP_Ctrl, USB_EP_Error_Handler onError);
USB_Result USB_EP_Reset(USB_EP_TypeDef EPx);
USB_Result USB_EP_Idle(USB_EP_TypeDef EPx);
USB_Result USB_EP_Stall(USB_EP_TypeDef EPx, USB_StallType bHalt);

USB_Result USB_EP_doDataIn(USB_EP_TypeDef EPx, uint8_t* Buffer, uint32_t Length, USB_EP_IO_Handler onInDone);
USB_Result USB_EP_doDataOut(USB_EP_TypeDef EPx, uint8_t* Buffer, uint32_t Length, USB_EP_IO_Handler onOutDone);

USB_Result USB_EP_setSetupHandler(USB_EP_TypeDef EPx, USB_SetupPacket_TypeDef* USB_SetupPacket, USB_EP_Setup_Handler onSetupPacket);

USB_Result USB_EP_dispatchEvent(USB_EP_TypeDef EPx, uint32_t USB_IT);

   

   



 



 



 

typedef enum
{
  USB_DEV_STATE_UNKNOWN = 0,
  USB_DEV_STATE_ATTACHED,
  USB_DEV_STATE_POWERED,
  USB_DEV_STATE_DEFAULT,
  USB_DEV_STATE_ADDRESS,
  USB_DEV_STATE_CONFIGURED,
  Num_USB_DEV_STATE
}USB_DeviceState_TypeDef;



 

typedef enum
{
  USB_DEV_SELF_POWERED_OFF = 0,
  USB_DEV_SELF_POWERED_ON  = 1
} USB_DeviceSelfPoweredState_TypeDef;



 

typedef enum
{
  USB_DEV_REMOTE_WAKEUP_DISABLED = 0,
  USB_DEV_REMOTE_WAKEUP_ENABLED = 1
}USB_DeviceRemoteWakeup_TypeDef;



 

typedef struct
{

  USB_DeviceSelfPoweredState_TypeDef SelfPowered;
#line 282 ".\\inc\\1986BE9x_usb_device.h"
}Usb_DeviceStatus_TypeDef;



 

typedef struct {
  USB_DeviceState_TypeDef  USB_DeviceState;
  Usb_DeviceStatus_TypeDef USB_DeviceStatus;
  uint32_t Address;
}USB_DeviceContext_TypeDef;



 

typedef struct {
  uint32_t PULL;                           




 
  uint32_t SPEED;                          


 
  uint32_t MODE;                           


 
}USB_DeviceBUSParam_TypeDef;












   



 

   



 



 

extern USB_SetupPacket_TypeDef USB_CurrentSetupPacket;



 

extern USB_DeviceContext_TypeDef USB_DeviceContext;

   



 

   



 

USB_Result USB_DeviceInit(const USB_Clock_TypeDef* USB_Clock_InitStruct, USB_DeviceBUSParam_TypeDef* USB_DeviceBUSParam);
USB_Result USB_DevicePowerOn(void);
USB_Result USB_DevicePowerOff(void);




USB_Result USB_DeviceReset(void);
USB_Result USB_DeviceSuspend(void);
USB_Result USB_DeviceResume(void);

USB_Result USB_DeviceSetupPacket(USB_EP_TypeDef EPx, const USB_SetupPacket_TypeDef* USB_SetupPacket);

USB_Result USB_DeviceClearFeature(USB_RequestRecipient_TypeDef Recipient, uint16_t wVALUE, uint16_t wINDEX);
USB_Result USB_DeviceSetFeature(USB_RequestRecipient_TypeDef Recipient, uint16_t wVALUE, uint16_t wINDEX);

USB_Result USB_DeviceDoStatusInAck(USB_EP_TypeDef EPx, uint8_t* Buffer, uint32_t Length);
USB_Result USB_DeviceDoStatusOutAck(USB_EP_TypeDef EPx, uint8_t* Buffer, uint32_t Length);

USB_Result USB_DeviceDispatchEvent(void);


void USB_IRQHandler(void);




 

USB_Result USB_DeviceDummyGetStatus(USB_RequestRecipient_TypeDef Recipient, uint16_t wINDEX);
USB_Result USB_DeviceDummySetAddress(uint16_t wVALUE);
USB_Result USB_DeviceDummyGetDescriptor(uint16_t wVALUE, uint16_t wINDEX, uint16_t wLENGTH);
USB_Result USB_DeviceDummySetDescriptor(uint16_t wVALUE, uint16_t wINDEX, uint16_t wLENGTH);
uint8_t    USB_DeviceDummyGetConfiguration(void);
USB_Result USB_DeviceDummySetConfiguration(uint16_t wVALUE);
uint8_t    USB_DeviceDummyGetInterface(uint16_t wINDEX);
USB_Result USB_DeviceDummySetInterface(uint16_t wVALUE, uint16_t wINDEX);
USB_Result USB_DeviceDummySyncFrame(uint16_t wINDEX, uint8_t* DATA);
USB_Result USB_DeviceDummyClassRequest(void);
USB_Result USB_DeviceDummyVendorRequest(void);
USB_Result USB_DeviceDummyDataError(USB_EP_TypeDef EPx, uint32_t STS, uint32_t TS, uint32_t CTRL);

   

   

   

   

   





 

#line 36 ".\\inc\\1986BE9x_usb_cdc.h"



 



 



 



 



 

typedef enum
{
  USB_CDC_SEND_ENCAPSULATED_COMMAND = 0x00,
  USB_CDC_GET_ENCAPSULATED_RESPONSE,
  USB_CDC_SET_COMM_FEATURE,
  USB_CDC_GET_COMM_FEATURE,
  USB_CDC_CLEAR_COMM_FEATURE,
  USB_CDC_SET_AUX_LINE_STATE        = 0x10,
  USB_CDC_SET_HOOK_STATE,
  USB_CDC_PULSE_SETUP,
  USB_CDC_SEND_PULSE,
  USB_CDC_SET_PULSE_TIME,
  USB_CDC_RING_AUX_JACK,
  USB_CDC_SET_LINE_CODING           = 0x20,
  USB_CDC_GET_LINE_CODING,
  USB_CDC_SET_CONTROL_LINE_STATE,
  USB_CDC_SEND_BREAK,
  USB_CDC_SET_RINGER_PARAMS         = 0x30,
  USB_CDC_GET_RINGER_PARAMS,
  USB_CDC_SET_OPERATION_PARAMS,
  USB_CDC_GET_OPERATION_PARAMS,
  USB_CDC_SET_LINE_PARAMS,
  USB_CDC_GET_LINE_PARAMS,
  USB_CDC_DIAL_DIGITS
} USB_CDC_Class_Setup_TypeDef;




 

typedef enum
{
  USB_CDC_bRxCarrier  = 0x1,
  USB_CDC_bTxCarrier  = 0x2,
  USB_CDC_bBreak      = 0x3,
  USB_CDC_bRingSignal = 0x4,
  USB_CDC_bFraming    = 0x5,
  USB_CDC_bParity     = 0x6,
  USB_CDC_bOverRun    = 0x7
}USB_CDCSerialState_TypeDef;



 

typedef enum
{
  USB_CDC_STOP_BITS1   = 0x0,
  USB_CDC_STOP_BITS1_5 = 0x1,
  USB_CDC_STOP_BITS2   = 0x2,
}USB_CDC_CharFormat_TypeDef;



 

typedef enum
{
  USB_CDC_PARITY_NONE  = 0x0,
  USB_CDC_PARITY_ODD   = 0x1,
  USB_CDC_PARITY_EVEN  = 0x2,
  USB_CDC_PARITY_MARK  = 0x3,
  USB_CDC_PARITY_SPACE = 0x4
}USB_CDC_ParityType_TypeDef;



 

typedef enum
{
  USB_CDC_DATE_BITS5  = 0x5,
  USB_CDC_DATE_BITS6  = 0x6,
  USB_CDC_DATE_BITS7  = 0x7,
  USB_CDC_DATE_BITS8  = 0x8,
  USB_CDC_DATE_BITS16 = 0xA
}USB_CDC_DateBits_TypeDef;



 

typedef struct
{
  uint32_t dwDTERate;
  uint8_t  bCharFormat;
  uint8_t  bParityType;
  uint8_t  bDataBits;
}USB_CDC_LineCoding_TypeDef;



 

typedef enum
{
  USB_CDC_DTR_PRESENT          = 0x1,
  USB_CDC_RTS_ACTIVATE_CARRIER = 0x2
}USB_CDC_ControlLineState_TypeDef;




 

typedef enum
{
  USB_CDC_RING_DETECT             = 0x09,
  USB_CDC_SERIAL_STATE            = 0x20,
  USB_CDC_CALL_STATE_CHANGE       = 0x28,
  USB_CDC_LINE_STATE_CHANGE       = 0x29,
  USB_CDC_CONNECTION_SPEED_CHANGE = 0x2A
}USB_CDC_LineStateReport_TypeDef;

   



 

   



 


   



 



 





   

   



 

USB_Result USB_CDC_Init(uint8_t* ReceiveBuffer, uint32_t DataPortionLength, FlagStatus StartReceiving);

USB_Result USB_CDC_SetReceiveBuffer(uint8_t* ReceiveBuffer, uint32_t DataPortionLength);
USB_Result USB_CDC_ReceiveStart(void);
USB_Result USB_CDC_ReceiveStop(void);

USB_Result USB_CDC_SendData(uint8_t* Buffer, uint32_t Length);







 

USB_Result USB_CDC_Reset(void);
USB_Result USB_CDC_GetDescriptor(uint16_t wVALUE, uint16_t wINDEX, uint16_t wLENGTH);
USB_Result USB_CDC_ClassRequest(void);

   



 

USB_Result USB_CDC_DummyDataReceive(uint8_t* Buffer, uint32_t Length);
USB_Result USB_CDC_DummyDataSent(void);













USB_Result USB_CDC_DummyGetLineCoding(uint16_t wINDEX, USB_CDC_LineCoding_TypeDef* DATA);
USB_Result USB_CDC_DummySetLineCoding(uint16_t wINDEX, const USB_CDC_LineCoding_TypeDef* DATA);










   

   

   

   

   





 

#line 28 ".\\inc\\1986BE9x_usb_default_handlers.h"

 






 



 



 



 



 





 





 





 





 





 



 






 






 





 



 





 



 





 



 





 



 






 



 





 



 






 



 





 



 





 



 





 

   



 




 



 



 





 



 





 



 





 


#line 268 ".\\inc\\1986BE9x_usb_default_handlers.h"

#line 302 ".\\inc\\1986BE9x_usb_default_handlers.h"




 





 



 





 



#line 338 ".\\inc\\1986BE9x_usb_default_handlers.h"

#line 351 ".\\inc\\1986BE9x_usb_default_handlers.h"



 





 

   

   



 











   



   

   

   

   





 

#line 27 ".\\inc\\1986BE9x_usb_handlers.h"

 







 



 



 



 









#line 64 ".\\inc\\1986BE9x_usb_handlers.h"












   




 

USB_Result USB_CDC_RecieveData(uint8_t* Buffer, uint32_t Length);


USB_Result USB_CDC_GetLineCoding(uint16_t wINDEX, USB_CDC_LineCoding_TypeDef* DATA);
USB_Result USB_CDC_SetLineCoding(uint16_t wINDEX, const USB_CDC_LineCoding_TypeDef* DATA);










   

   

   

   






 

#line 28 "src\\1986BE9x_usb_device.c"





 



 



 

   



 



 



 

typedef enum
{
  USB_EP_NAK,
  USB_EP_IDLE,
  USB_EP_IN,
  USB_EP_OUT,
  USB_EP_SETUP,
  USB_EP_STALL
}USB_EPState_TypeDef;



 

typedef struct
{
  USB_EPState_TypeDef EP_State;
  USB_StallType EP_Halt;
  struct
  {
    struct
    {
       
      uint8_t *pBuffer;
      uint32_t length, offset;
      uint32_t bytesToAck;        
 
    }IO_Buffer;
     
    USB_SetupPacket_TypeDef *pSetupPacket;
  }Buffer;
  FlagStatus EP_WasScdone;
  FlagStatus EP_WaitOut, EP_WaitSetup;
  USB_EP_IO_Handler InHandler;
  USB_EP_IO_Handler OutHandler;
  USB_EP_Setup_Handler SetupHandler;
  USB_EP_Error_Handler ErrorHandler;
}USB_EPContext_TypeDef;



 

typedef enum
{
  USB_DATA_BIT_KEEP,
  USB_DATA_BIT_TOGGLE,
  USB_DATA_BIT_DATA1,
}USB_EPData_Bit_TypeDef;

   



 



 

USB_EPContext_TypeDef USB_EPContext[Num_USB_EndPoints];

   



 

   



 





   



 

static void USB_EP_sendInDataPortion(USB_EP_TypeDef EPx, USB_EPData_Bit_TypeDef DataBitChange);
static void USB_EP_SetReady(USB_EP_TypeDef EPx, uint32_t val);

   



 













 

USB_Result USB_EP_Init(USB_EP_TypeDef EPx, uint32_t USB_EP_Ctrl, USB_EP_Error_Handler onError)
{
  USB_EPContext_TypeDef *ep = USB_EPContext + EPx;

   
  ep->Buffer.IO_Buffer.pBuffer    = 0;
  ep->Buffer.IO_Buffer.length     = 0;
  ep->Buffer.IO_Buffer.bytesToAck = 0;
  ep->Buffer.IO_Buffer.offset     = 0;
  ep->Buffer.pSetupPacket         = 0;
  ep->InHandler                   = 0;
  ep->OutHandler                  = 0;
  ep->SetupHandler                = 0;
  ep->ErrorHandler                = onError;
  ep->EP_Halt                     = USB_STALL_PROTO;
  ep->EP_State                    = USB_EP_NAK;
  ep->EP_WasScdone                = RESET;

  USB_SetSEPxCTRL(EPx, USB_EP_Ctrl);

  return USB_SUCCESS;
}









 

USB_Result USB_EP_Reset(USB_EP_TypeDef EPx)
{
  USB_EPContext_TypeDef *ep = USB_EPContext + EPx;

  ep->EP_State = USB_EP_NAK;
  ep->EP_WasScdone = RESET;

  USB_SetSEPxCTRL(EPx, (uint32_t)(1 << (4 + 16))  |                            
                       (uint32_t)(1 << (1 + 16)) |                            
                       (uint32_t)(1 << 0));                               

  return USB_SUCCESS;
}












 

USB_Result USB_EP_Idle(USB_EP_TypeDef EPx)
{
  USB_EPContext_TypeDef *ep = USB_EPContext + EPx;
  uint32_t val = (uint32_t)(1 << (3 + 16));                                

  if (ep->EP_WaitOut || ep->EP_WaitSetup)
  {
    ep->EP_State = USB_EP_IDLE;
    val |= (uint32_t)(1 << 1);                                            
  }
  else
  {
    ep->EP_State = USB_EP_NAK;
  }
  ep->EP_Halt = USB_STALL_PROTO;

  USB_EP_SetReady(EPx, val);

  return USB_SUCCESS;
}
















 

USB_Result USB_EP_Stall(USB_EP_TypeDef EPx, USB_StallType bHalt)
{
  USB_EPContext_TypeDef *ep = USB_EPContext + EPx;
  uint32_t val = 0;

  if (bHalt == USB_STALL_HALT && EPx == USB_EP0)
  {
     
    return USB_ERROR;
  }

  if (bHalt == USB_STALL_HALT || ep->EP_WaitSetup)
  {
    ep->EP_Halt  = bHalt;
    ep->EP_State = USB_EP_STALL;
    val = (uint32_t)(1 << 3) |                                           
          (uint32_t)(1 << 1);                                               
  }

  USB_EP_SetReady(EPx, val);

  return USB_SUCCESS;
}

















 

USB_Result USB_EP_doDataIn(USB_EP_TypeDef EPx, uint8_t* Buffer, uint32_t Length, USB_EP_IO_Handler onInDone)
{
  USB_EPContext_TypeDef *ep = USB_EPContext + EPx;
   
  USB_EPData_Bit_TypeDef StartInStage = (ep->EP_WaitSetup ? USB_DATA_BIT_DATA1 : USB_DATA_BIT_TOGGLE);

   
  ep->Buffer.IO_Buffer.pBuffer    = Buffer;
  ep->Buffer.IO_Buffer.length     = Length;
  ep->Buffer.IO_Buffer.bytesToAck = 32;
  ep->Buffer.IO_Buffer.offset     = 0;
  ep->InHandler                   = onInDone;
  ep->EP_State                    = USB_EP_IN;

   
  USB_EP_sendInDataPortion(EPx, StartInStage);

  return USB_SUCCESS;
}




















 

USB_Result USB_EP_doDataOut(USB_EP_TypeDef EPx, uint8_t* Buffer, uint32_t Length, USB_EP_IO_Handler onOutDone)
{
  USB_EPContext_TypeDef *ep = USB_EPContext + EPx;

   
  ep->Buffer.IO_Buffer.pBuffer    = Buffer;
  ep->Buffer.IO_Buffer.length     = Length;
  ep->Buffer.IO_Buffer.bytesToAck = 0;
  ep->Buffer.IO_Buffer.offset     = 0;
  ep->OutHandler                  = onOutDone;
  ep->EP_WaitOut                  = SET;

   
  USB_EP_Idle(EPx);

  return USB_SUCCESS;
}
















 

USB_Result USB_EP_setSetupHandler(USB_EP_TypeDef EPx, USB_SetupPacket_TypeDef* USB_SetupPacket, USB_EP_Setup_Handler onSetupPacket)
{
  USB_EPContext_TypeDef *ep = USB_EPContext + EPx;

   
  ep->Buffer.pSetupPacket         = USB_SetupPacket;
  ep->Buffer.IO_Buffer.length     = 0;
  ep->Buffer.IO_Buffer.bytesToAck = 0;
  ep->Buffer.IO_Buffer.offset     = 0;
  ep->SetupHandler                = onSetupPacket;
  ep->EP_WaitSetup                = onSetupPacket ? SET : RESET;

   
  USB_EP_Idle(EPx);

  return USB_SUCCESS;
}














 

USB_Result USB_EP_dispatchEvent(USB_EP_TypeDef EPx, uint32_t USB_IT)
{
  FlagStatus nextIteration = RESET;
  USB_Result result = USB_SUCCESS;
  uint32_t tmpSTS, tmpTS, tmpCTRL;
  uint32_t i, count;
  USB_EPContext_TypeDef *ep;

  tmpSTS = USB_GetSEPxSTS(EPx);
  tmpTS  = (USB_GetSEPxTS(EPx) & (uint32_t)(0x00000003));
  tmpCTRL = USB_GetSEPxCTRL(EPx);

  ep = USB_EPContext + EPx;

   
  if (USB_IT & ((uint32_t)0x00000001))
  {
    ep->EP_WasScdone = SET;
  }

  do
  {
    switch (ep->EP_State)
    {
       
      case USB_EP_NAK:
      {
         
        return USB_SUCCESS;
      }

       
      case USB_EP_IDLE:
      {
        if (!(tmpCTRL & ((uint32_t)0x00000002)) && ep->EP_WasScdone)
        {
           
          if ((tmpTS == (uint32_t)(0x00000002)) && ep->EP_WaitOut)
          {
             
            ep->EP_State = USB_EP_OUT;
            ep->EP_WaitOut = RESET;
            nextIteration = SET;
          }
          else if ((tmpTS == (uint32_t)(0x00000000)) && ep->EP_WaitSetup)
          {
             
            ep->EP_State = USB_EP_SETUP;
            nextIteration = SET;
          }
          else
          {
             
            result = USB_ERROR;
            USB_EP_Stall(EPx, USB_STALL_PROTO);
          }
        }
        break;
      }

       
      case USB_EP_IN:
      {
         
        if ((!(tmpCTRL & ((uint32_t)0x00000002))) && ep->EP_WasScdone &&
            (tmpTS == (uint32_t)(0x00000001)) && (tmpSTS & ((uint32_t)0x00000040)))
        {
           
          ep->Buffer.IO_Buffer.offset += ep->Buffer.IO_Buffer.bytesToAck;
          if (ep->Buffer.IO_Buffer.offset >= ep->Buffer.IO_Buffer.length)
          {
            
 
            ep->EP_State = USB_EP_NAK;
            if (ep->InHandler != 0)
            {
              result = ep->InHandler(EPx, ep->Buffer.IO_Buffer.pBuffer, ep->Buffer.IO_Buffer.length);
            }

            if (result == USB_SUCCESS)
            {
              
 
              if (ep->EP_State == USB_EP_NAK)
              {
                USB_EP_Idle(EPx);
              }
            }
            else
            {
               
              USB_EP_Stall(EPx, USB_STALL_PROTO);
            }
          }
          else
          {
             
            USB_EP_sendInDataPortion(EPx, USB_DATA_BIT_TOGGLE);
          }
        }
        else if (tmpSTS & (((uint32_t)0x00000004)
                         | ((uint32_t)0x00000001)
                         | ((uint32_t)0x00000002)
                         | ((uint32_t)0x00000008)))
        {
          
 
          ep->EP_State = USB_EP_NAK;
          if (ep->ErrorHandler != 0)
          {
            result = ep->ErrorHandler(EPx, tmpSTS, tmpTS, tmpCTRL);
          }
          if (result == USB_SUCCESS)
          {
            if (ep->EP_State == USB_EP_NAK)
            {
              
 
              ep->EP_State = USB_EP_IN;
              USB_EP_sendInDataPortion(EPx, USB_DATA_BIT_KEEP);
            }
          }
          else
          {
             
            USB_EP_Stall(EPx, USB_STALL_PROTO);
          }
        }
        break;
      }

       
      case USB_EP_OUT:
      {
         
        if (!(tmpCTRL & ((uint32_t)0x00000002)) && ep->EP_WasScdone)
        {
          nextIteration = RESET;

           
          count = USB_GetSEPxRXFDC(EPx);
          for (i = 0; i < count; i++)
          {
            ep->Buffer.IO_Buffer.pBuffer[ep->Buffer.IO_Buffer.offset + i] = USB_GetSEPxRXFD(EPx);
          }
          USB_SetSEPxRXFC(EPx, 1);
          ep->Buffer.IO_Buffer.offset += count;

          
 
          if (ep->Buffer.IO_Buffer.offset >= ep->Buffer.IO_Buffer.length)
          {
            ep->EP_State = USB_EP_NAK;
            if (ep->OutHandler != 0)
            {
              result = ep->OutHandler(EPx, ep->Buffer.IO_Buffer.pBuffer, ep->Buffer.IO_Buffer.offset);
            }

            if (result == USB_SUCCESS)
            {
              
 
              if (ep->EP_State == USB_EP_NAK)
              {
                USB_EP_Idle(EPx);
              }
            }
            else
            {
               
              USB_EP_Stall(EPx, USB_STALL_PROTO);
            }
          }
          else
          {
             
            USB_EP_SetReady(EPx, (uint32_t)(1 << 1));
          }
        }
        break;
      }

       
      case USB_EP_SETUP:
      {
         
        if (ep->EP_WasScdone)
        {
          ((void)0);

           
          count = USB_GetSEPxRXFDC(EPx);
          if (count == 8)
          {
            for (i = 0; i < count; i++)
            {
              ((uint8_t*)ep->Buffer.pSetupPacket)[i] = USB_GetSEPxRXFD(EPx);
            }
            USB_SetSEPxRXFC(EPx, 1);

             
            result = ep->SetupHandler(EPx, ep->Buffer.pSetupPacket);

            if (result == USB_SUCCESS)
            {
              
 
              if (ep->EP_State == USB_EP_SETUP)
              {
                USB_EP_Idle(EPx);
              }
            }
            else
            {
               
              USB_EP_Stall(EPx, USB_STALL_PROTO);
            }
          }
          else  
          {
            USB_SetSEPxRXFC(EPx, 1);
            result = USB_ERROR;
             
            USB_EP_Stall(EPx, USB_STALL_PROTO);
          }
        }
        nextIteration = RESET;
        break;
      }

       
      case USB_EP_STALL:
      {
        if (!(tmpCTRL & ((uint32_t)0x00000002)))
        {
           
          if (ep->EP_Halt == USB_STALL_HALT)
          {
            USB_EP_Stall(EPx, USB_STALL_HALT);
          }
          else
          {
            USB_EP_Idle(EPx);
          }
        }
        break;
      }
    }
  } while (nextIteration);

  return result;
}















 

static void USB_EP_sendInDataPortion(USB_EP_TypeDef EPx, USB_EPData_Bit_TypeDef DataBitChange)
{
  USB_EPContext_TypeDef *ep = USB_EPContext + EPx;
  uint32_t i, total;

  ((void)0);

   
  USB_SetSEPxTXFDC(EPx, 1);

   
  switch (DataBitChange)
  {
    case USB_DATA_BIT_KEEP:
      break;
    case USB_DATA_BIT_TOGGLE:
      USB_SEPxToggleEPDATASEQ(EPx);
      break;
    case USB_DATA_BIT_DATA1:
      USB_SetSEPxCTRL(EPx, (uint32_t)(1 << 2));
      break;
  }

   
  total = (ep->Buffer.IO_Buffer.offset + ep->Buffer.IO_Buffer.bytesToAck < ep->Buffer.IO_Buffer.length ?
            ep->Buffer.IO_Buffer.offset + ep->Buffer.IO_Buffer.bytesToAck : ep->Buffer.IO_Buffer.length);
  for (i = ep->Buffer.IO_Buffer.offset; i < total; i++)
  {
    USB_SetSEPxTXFD(EPx, ep->Buffer.IO_Buffer.pBuffer[i]);
  };

   
  USB_EP_SetReady(EPx, (uint32_t)(1 << 1));
}
















 

static void USB_EP_SetReady(USB_EP_TypeDef EPx, uint32_t val)
{
  USB_EPContext_TypeDef *ep = USB_EPContext + EPx;

   
  if (val & (uint32_t)(1 << 1))
  {
    USB_SetSIS(((uint32_t)0x00000001) | ((uint32_t)0x00000002) | ((uint32_t)0x00000004) | ((uint32_t)0x00000008) | ((uint32_t)0x00000010));
  }
  ep->EP_WasScdone = RESET;

   
  USB_SetSEPxCTRL(EPx, val);
}

   

   



 



 

   



 

   



 

USB_SetupPacket_TypeDef USB_CurrentSetupPacket;
USB_DeviceContext_TypeDef USB_DeviceContext;
static uint8_t SetupPacketData[2];

   



 

   



 

static USB_Result USB_Device_setAddressWork(USB_EP_TypeDef EPx, uint8_t* Buffer, uint32_t Length);

   



 











 

USB_Result USB_DeviceInit(const USB_Clock_TypeDef* USB_Clock_InitStruct, USB_DeviceBUSParam_TypeDef* USB_DeviceBUSParam)
{
   
  ((void)0);
  ((void)0);
  ((void)0);

   
  USB_BRGInit(USB_Clock_InitStruct);
  USB_Reset();
   
  USB_SetHSCR((uint32_t)(1 << (0 + 16)));
  USB_SetHSCR(USB_DeviceBUSParam->PULL);
   
  USB_SetSC(USB_DeviceBUSParam->SPEED | USB_DeviceBUSParam->MODE | (uint32_t)(1 << 0));
   
  USB_EP_Init(USB_EP0, (uint32_t)(1 << 0) | (uint32_t)(1 << 2), 0);
  USB_EP_setSetupHandler(USB_EP0, &USB_CurrentSetupPacket, USB_DeviceSetupPacket);

  USB_DeviceContext.USB_DeviceState = USB_DEV_STATE_UNKNOWN;
  USB_DeviceContext.Address = 0;

  return USB_SUCCESS;
}







 

USB_Result USB_DevicePowerOn(void)
{
  USB_SetHSCR((uint32_t)(1 << 3) | (uint32_t)(1 << 2));
  USB_DeviceContext.USB_DeviceState = USB_DEV_STATE_POWERED;

  return USB_SUCCESS;
}







 

USB_Result USB_DevicePowerOff(void)
{
  USB_SetHSCR((uint32_t)(1 << (3 + 16)));
  USB_DeviceContext.USB_DeviceState = USB_DEV_STATE_UNKNOWN;

  return USB_SUCCESS;
}

#line 880 "src\\1986BE9x_usb_device.c"



 







 

USB_Result USB_DeviceReset(void)
{
  ((USB_TypeDef *)0x40010000 )->SA=0;
  USB_DeviceContext.USB_DeviceState = USB_DEV_STATE_DEFAULT;
  return USB_SUCCESS;
}







 

USB_Result USB_DeviceSuspend(void)
{
  return USB_SUCCESS;
}







 

USB_Result USB_DeviceResume(void)
{
  return USB_SUCCESS;
}
















 

USB_Result USB_DeviceSetupPacket(USB_EP_TypeDef EPx, const USB_SetupPacket_TypeDef* USB_SetupPacket)
{
  USB_RequestType_TypeDef requestType;
  USB_RequestTypeDT_TypeDef requestDirection;

  USB_Result result = USB_SUCCESS;
  uint16_t wValue, wIndex, wLength;

   
  ((void)0);

   
  requestType = (USB_RequestType_TypeDef)(USB_SetupPacket->mRequestTypeData & 0x60);
  requestDirection = (USB_RequestTypeDT_TypeDef)(USB_SetupPacket->mRequestTypeData & 0x80);

   
  switch (requestType)
  {
     
    case USB_TYPE_STANDARD: {
      USB_RequestRecipient_TypeDef recipient;
      USB_EP_IO_Handler statusAckHandler = 0;

      wValue = USB_SetupPacket->wValue;
      wIndex = USB_SetupPacket->wIndex;
      wLength = USB_SetupPacket->wLength;

       
      recipient = (USB_RequestRecipient_TypeDef)(USB_SetupPacket->mRequestTypeData & 0x1F);
      if (!((recipient) <= USB_RECIPIENT_OTHER)) {
        result = USB_ERR_INV_REQ;
        break;
      }
       
      switch (USB_SetupPacket->bRequest)
      {
         
        case USB_GET_STATUS:
          if (requestDirection != USB_DEVICE_TO_HOST ||
             (USB_DeviceContext.USB_DeviceState == USB_DEV_STATE_ADDRESS && (recipient == USB_RECIPIENT_INTERFACE ||
              recipient == USB_RECIPIENT_ENDPOINT && wIndex >= Num_USB_EndPoints)))
          {
            result = USB_ERROR;
          }
          else
          {
             
            result = USB_SUCCESS;
            if (result == USB_SUCCESS)
            {
              switch (recipient)
              {
                 
                case USB_RECIPIENT_DEVICE:
                  SetupPacketData[0] = 0

                    | USB_DeviceContext.USB_DeviceStatus.SelfPowered




                    ;
                  break;
                 
                case USB_RECIPIENT_INTERFACE:
                  SetupPacketData[0] = 0;
                  break;
                 
                case USB_RECIPIENT_ENDPOINT:
                  SetupPacketData[0] = USB_EPContext[wIndex].EP_Halt;
                  break;
              }
              SetupPacketData[1] = 0;
              result = USB_EP_doDataIn(EPx, SetupPacketData, 2, 0);
            }
          }
          break;
         
        case USB_CLEAR_FEATURE:
          if (USB_DeviceContext.USB_DeviceState == USB_DEV_STATE_ADDRESS && (recipient == USB_RECIPIENT_INTERFACE ||
              (recipient == USB_RECIPIENT_ENDPOINT && wIndex >= Num_USB_EndPoints)))
          {
            result = USB_ERROR;
          }
          else
          {
            result = USB_DeviceClearFeature(recipient, wValue, wIndex);
          }
          break;
         
        case USB_SET_FEATURE:
          if (USB_DeviceContext.USB_DeviceState == USB_DEV_STATE_ADDRESS && (recipient == USB_RECIPIENT_INTERFACE ||
              (recipient == USB_RECIPIENT_ENDPOINT && wIndex >= Num_USB_EndPoints)))
          {
            result = USB_ERROR;
          }
          else
          {
            result = USB_DeviceSetFeature(recipient, wValue, wIndex);
          }
          break;
         
        case USB_SET_ADDRESS:
          result = USB_SUCCESS;
           
          USB_DeviceContext.Address = wValue;
          statusAckHandler = USB_Device_setAddressWork;
          break;
         
        case USB_GET_DESCRIPTOR:
          result = USB_CDC_GetDescriptor(wValue, wIndex, wLength);
          break;
         
        case USB_SET_DESCRIPTOR:
          result = USB_ERROR;
          break;
         
        case USB_GET_CONFIGURATION:
          if (USB_DeviceContext.USB_DeviceState == USB_DEV_STATE_ADDRESS)
          {
            SetupPacketData[0] = 0;
          }
          else
          {
            SetupPacketData[0] = (uint8_t)1;
          }
          result = USB_EP_doDataIn(EPx, SetupPacketData, 1, 0);
          break;
         
        case USB_SET_CONFIGURATION:
          if (wValue == 0)
          {
            USB_DeviceContext.USB_DeviceState = USB_DEV_STATE_ADDRESS;
          }
          else
          {
            result = ((wValue) == 1 ? USB_SUCCESS : USB_ERROR);
            if (result == USB_SUCCESS)
            {
              USB_DeviceContext.USB_DeviceState = USB_DEV_STATE_CONFIGURED;
            }
          }
          break;
         
        case USB_GET_INTERFACE:
          if (USB_DeviceContext.USB_DeviceState == USB_DEV_STATE_ADDRESS)
          {
            result = USB_ERROR;
          }
          else
          {
            SetupPacketData[0] = (uint8_t)0;
            result = USB_EP_doDataIn(EPx, SetupPacketData, 1, 0);
          }
          break;
         
        case USB_SET_INTERFACE:
          if (USB_DeviceContext.USB_DeviceState == USB_DEV_STATE_ADDRESS)
          {
            result = USB_ERROR;
          }
          else
          {
            result = ((wIndex) == 0 && (wValue) == 0 ? USB_SUCCESS : USB_ERROR);
          }
          break;
         
        case USB_SYNCH_FRAME:
          if (USB_DeviceContext.USB_DeviceState == USB_DEV_STATE_ADDRESS)
          {
            result = USB_ERROR;
          }
          else
          {
            result = USB_ERROR;
            if (result == USB_SUCCESS)
            {
              result = USB_EP_doDataIn(EPx, SetupPacketData, 2, 0);
            }
          }
          break;
        default:
          result = USB_ERR_INV_REQ;
      }
       
      if (result == USB_SUCCESS && wLength == 0)
      {
        result = (USB_SetupPacket->mRequestTypeData & 0x80) == USB_DEVICE_TO_HOST ?
                        USB_EP_doDataOut(EPx, 0, 0, statusAckHandler) :
                        USB_EP_doDataIn(EPx, 0, 0, statusAckHandler);
      }
      break;
    }
     
    case USB_TYPE_CLASS:
      result = USB_CDC_ClassRequest();
      break;
     
    case USB_TYPE_VENDOR:
      result = USB_ERROR;
      break;
    default:
      result = USB_ERR_INV_REQ;
  }
  return result;
}

   



 



















 

USB_Result USB_DeviceClearFeature(USB_RequestRecipient_TypeDef Recipient, uint16_t wVALUE, uint16_t wINDEX)
{
  if ((Recipient == USB_RECIPIENT_ENDPOINT) && (wVALUE == USB_ENDPOINT_HALT))
  {
    return USB_EP_Idle((USB_EP_TypeDef)wINDEX);
  }
  else
  {
    return USB_ERROR;
  }
}



















 

USB_Result USB_DeviceSetFeature(USB_RequestRecipient_TypeDef Recipient, uint16_t wVALUE, uint16_t wINDEX)
{
  if ((Recipient == USB_RECIPIENT_ENDPOINT) && (wVALUE == USB_ENDPOINT_HALT))
  {
    return USB_EP_Stall((USB_EP_TypeDef)wINDEX, USB_STALL_HALT);
  }
  else
  {
    return USB_ERROR;
  }
}

   



 


















 

USB_Result USB_DeviceDoStatusInAck(USB_EP_TypeDef EPx, uint8_t* Buffer, uint32_t Length)
{
  return USB_SUCCESS;
}

















 

USB_Result USB_DeviceDoStatusOutAck(USB_EP_TypeDef EPx, uint8_t* Buffer, uint32_t Length)
{
  return USB_EP_doDataOut(EPx, 0, 0, 0);
}



















 

static USB_Result USB_Device_setAddressWork(USB_EP_TypeDef EPx, uint8_t* Buffer, uint32_t Length)
{
   
  USB_SetSA(USB_DeviceContext.Address);
   
  USB_DeviceContext.USB_DeviceState = (USB_DeviceContext.Address ? USB_DEV_STATE_ADDRESS : USB_DEV_STATE_DEFAULT);

  return USB_SUCCESS;
}

   










 

USB_Result USB_DeviceDispatchEvent(void)
{
  uint32_t i;
  uint32_t USB_IT;
  USB_Result result = USB_SUCCESS;
  static volatile uint32_t bHandling = RESET;

   

  NVIC_DisableIRQ(USB_IRQn);


  if (!bHandling) {
    bHandling = SET;

     
    USB_IT = USB_GetSIS();
     
    if (USB_IT & ((uint32_t)0x00000004))
    {
      result = USB_DeviceReset();
    }

     
    for (i = USB_EP0; i < Num_USB_EndPoints; i++)
    {
      USB_EP_dispatchEvent((USB_EP_TypeDef)i, USB_IT);
    }

     
    USB_SetSIS(USB_IT & (~((uint32_t)0x00000001)));

    bHandling = RESET;

     

    NVIC_EnableIRQ(USB_IRQn);

  }
  return result;
}












 

void USB_IRQHandler(void)
{
  USB_DeviceDispatchEvent();
}





 

















 

USB_Result USB_DeviceDummyGetStatus(USB_RequestRecipient_TypeDef Recipient, uint16_t wINDEX)
{
  return USB_SUCCESS;
}











 

USB_Result USB_DeviceDummySetAddress(uint16_t wVALUE)
{
  return USB_SUCCESS;
}
















 

USB_Result USB_DeviceDummyGetDescriptor(uint16_t wVALUE, uint16_t wINDEX, uint16_t wLENGTH)
{
  return USB_ERROR;
}
















 

USB_Result USB_DeviceDummySetDescriptor(uint16_t wVALUE, uint16_t wINDEX, uint16_t wLENGTH)
{
  return USB_ERROR;
}










 

uint8_t USB_DeviceDummyGetConfiguration(void)
{
  return 1;
}










 

USB_Result USB_DeviceDummySetConfiguration(uint16_t wVALUE)
{
  return USB_ERROR;
}











 

uint8_t USB_DeviceDummyGetInterface(uint16_t wINDEX)
{
  return 0;
}












 

USB_Result USB_DeviceDummySetInterface(uint16_t wVALUE, uint16_t wINDEX)
{
  return USB_SUCCESS;
}














 

USB_Result USB_DeviceDummySyncFrame(uint16_t wINDEX, uint8_t* DATA)
{
  return USB_ERROR;
}












 

USB_Result USB_DeviceDummyClassRequest(void)
{
  return USB_ERROR;
}












 


USB_Result USB_DeviceDummyVendorRequest(void)
{
  return USB_ERROR;
}























 

USB_Result USB_DeviceDummyDataError(USB_EP_TypeDef EPx, uint32_t STS, uint32_t TS, uint32_t CTRL)
{
  return USB_ERROR;
}

   


   

   

   

   



 

