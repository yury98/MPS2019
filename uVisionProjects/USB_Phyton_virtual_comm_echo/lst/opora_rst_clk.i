#line 1 "src\\opora_rst_clk.c"




















 

 
#line 1 ".\\inc\\opora_rst_clk.h"





















 

 



 
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
 




















 





































 

 
 
 
 
 
 
 
 








 











#line 1 "C:\\Keil4\\ARM\\RV31\\INC\\stdint.h"
 
 





 









#line 25 "C:\\Keil4\\ARM\\RV31\\INC\\stdint.h"







 

     

     
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

     
typedef   signed       __int64 intmax_t;
typedef unsigned       __int64 uintmax_t;




     

     





     





     





     

     





     





     





     

     





     





     





     

     


     


     


     

     


     


     


     

     



     



     


     
    
 



#line 196 "C:\\Keil4\\ARM\\RV31\\INC\\stdint.h"

     







     










     











#line 260 "C:\\Keil4\\ARM\\RV31\\INC\\stdint.h"



 


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

   

   

   





 

#line 25 "src\\opora_rst_clk.c"
#line 1 ".\\inc\\1986BE9x_config.h"

















 

 



#line 25 ".\\inc\\1986BE9x_config.h"
#line 26 ".\\inc\\1986BE9x_config.h"


 
 
 

 
 





 
#line 46 ".\\inc\\1986BE9x_config.h"



 
 


 


 

               

 

 


 
 
 



 


 
 
 
 
 

 
 

 



 
 


 
 

 







 

 
 








 
#line 126 ".\\inc\\1986BE9x_config.h"





 

#line 26 "src\\opora_rst_clk.c"





 



 



 





















 













#line 96 "src\\opora_rst_clk.c"


















































#line 157 "src\\opora_rst_clk.c"

   



 





 
void RST_CLK_DeInit(void)
{
  RST_CLK_WarmDeInit();
   
  RST_CLK_PCLKcmd(((uint32_t)(1 << ((((uint32_t)0x400D8000) >> 15) & 0x1F))), ENABLE);
  ((BKP_TypeDef *)0x400D8000 )->REG_0F |= (uint32_t)((1<<22)|(1<<15));  
  ((BKP_TypeDef *)0x400D8000 )->REG_0F &= (uint32_t)((1<<22)|(1<<15));
}






 
void RST_CLK_WarmDeInit(void)
{
   
  RST_CLK_HSIcmd(ENABLE);
  RST_CLK_HSIstatus();
  RST_CLK_CPUclkSelection(((uint32_t)0x00000000));

   
  ((RST_CLK_TypeDef *)0x40020000 )->CPU_CLOCK   &= (uint32_t)0x00000000;

   
  ((RST_CLK_TypeDef *)0x40020000 )->PLL_CONTROL &= (uint32_t)0x00000000;

   
  ((RST_CLK_TypeDef *)0x40020000 )->HS_CONTROL  &= (uint32_t)0x00000000;

   
  ((RST_CLK_TypeDef *)0x40020000 )->USB_CLOCK   &= (uint32_t)0x00000000;

   
  ((RST_CLK_TypeDef *)0x40020000 )->ADC_MCO_CLOCK   &= (uint32_t)0x00000000;

   
  ((RST_CLK_TypeDef *)0x40020000 )->RTC_CLOCK   &= (uint32_t)0x00000000;

   
  ((RST_CLK_TypeDef *)0x40020000 )->PER_CLOCK   = (uint32_t)((uint32_t)(1 << ((((uint32_t)0x40020000) >> 15) & 0x1F))) | (uint32_t)((uint32_t)(1 << ((((uint32_t)0x400D8000) >> 15) & 0x1F)));
}








 
void RST_CLK_HSEconfig(uint32_t RST_CLK_HSE)
{
   
  ((void)0);
   
  ((RST_CLK_TypeDef *)0x40020000 )->HS_CONTROL &= ~((uint32_t)(((uint32_t)0x00000001) | ((uint32_t)0x00000002)));
   
  switch (RST_CLK_HSE)
  {
    case ((uint32_t)0x00000001):
       
      ((RST_CLK_TypeDef *)0x40020000 )->HS_CONTROL |= ((uint32_t)0x00000001);
      break;

    case ((uint32_t)0x00000002):
       
      ((RST_CLK_TypeDef *)0x40020000 )->HS_CONTROL |= ((uint32_t)0x00000001) | ((uint32_t)0x00000002);
      break;

    default:
      break;
  }
}





 
ErrorStatus RST_CLK_HSEstatus(void)
{
  volatile uint32_t startCounter = 0;
  ErrorStatus state;
  FlagStatus flag;

   
  do
  {
    flag = RST_CLK_GetFlagStatus(((uint32_t)(0x20 | 2)));
    startCounter++;
  } while ((startCounter < ((uint16_t)0x0600)) && (flag == RESET));

  if (RST_CLK_GetFlagStatus(((uint32_t)(0x20 | 2))) != RESET)
  {
    state = SUCCESS;
  }
  else
  {
    state = ERROR;
  }
  return state;
}








 
void RST_CLK_LSEconfig(uint32_t RST_CLK_LSE)
{
   
  ((void)0);
   
  ((BKP_TypeDef *)0x400D8000 )->REG_0F &= ~((uint32_t)(((uint32_t)0x00000001) | ((uint32_t)0x00000002)));
  switch (RST_CLK_LSE)
  {
    case ((uint32_t)0x00000001):
       
      ((BKP_TypeDef *)0x400D8000 )->REG_0F |= ((uint32_t)0x00000001);
      break;

    case ((uint32_t)0x00000002):
       
      ((BKP_TypeDef *)0x400D8000 )->REG_0F |= (((uint32_t)0x00000001) | ((uint32_t)0x00000002));
      break;

    default:
      break;
  }
}





 
ErrorStatus RST_CLK_LSEstatus(void)
{
  volatile uint32_t startCounter = 0;
  ErrorStatus state;
  FlagStatus flag;

  
 do
  {
    flag = RST_CLK_GetFlagStatus(((uint32_t)(0x00 | 13)));
    startCounter++;
  } while ((startCounter < ((uint16_t)0x0600)) && (flag == RESET));

  if (RST_CLK_GetFlagStatus(((uint32_t)(0x00 | 13))) != RESET)
  {
    state = SUCCESS;
  }
  else
  {
    state = ERROR;
  }
  return state;
}







 
void RST_CLK_HSIcmd(FunctionalState NewState)
{
   
  ((void)0);

  *(volatile uint32_t *) (0x42000000 + ((uint32_t)&((BKP_TypeDef*)(0x400D8000 - 0x40000000))->REG_0F)*32 + 22*4) = (uint32_t)NewState;
}






 
void RST_CLK_HSIadjust(uint8_t HSItrimValue)
{
  uint32_t temp;
   
  ((void)0);
  temp = ((BKP_TypeDef *)0x400D8000 )->REG_0F;
   
  temp &= ~((uint32_t)(0x3F << 24));
   
  temp |= (uint32_t)HSItrimValue << 24;
   
  ((BKP_TypeDef *)0x400D8000 )->REG_0F = temp;
}





 
ErrorStatus RST_CLK_HSIstatus(void)
{
  volatile uint32_t startCounter = 0;
  ErrorStatus state;
  FlagStatus flag;

   
  do
  {
    flag = RST_CLK_GetFlagStatus(((uint32_t)(0x00 | 23)));
    startCounter++;
  } while ((startCounter < ((uint16_t)0x0600)) && (flag == RESET));

  if (RST_CLK_GetFlagStatus(((uint32_t)(0x00 | 23))) != RESET)
  {
    state = SUCCESS;
  }
  else
  {
    state = ERROR;
  }
  return state;
}







 
void RST_CLK_LSIcmd(FunctionalState NewState)
{
   
  ((void)0);

  *(volatile uint32_t *) (0x42000000 + ((uint32_t)&((BKP_TypeDef*)(0x400D8000 - 0x40000000))->REG_0F)*32 + 15*4) = (uint32_t)NewState;
}





 
ErrorStatus RST_CLK_LSIstatus(void)
{
  volatile uint32_t startCounter = 0;
  ErrorStatus state;
  FlagStatus flag;

   
  do
  {
    flag = RST_CLK_GetFlagStatus(((uint32_t)(0x00 | 21)));
    startCounter++;
  } while ((startCounter < ((uint16_t)0x0600)) && (flag == RESET));

  if (RST_CLK_GetFlagStatus(((uint32_t)(0x00 | 21))) != RESET)
  {
    state = SUCCESS;
  }
  else
  {
    state = ERROR;
  }
  return (state);
}



























 
void RST_CLK_CPU_PLLconfig(uint32_t RST_CLK_CPU_PLLsource, uint32_t RST_CLK_CPU_PLLmul)
{
  uint32_t temp;

   
  ((void)0);
  ((void)0);

   
  temp = ((RST_CLK_TypeDef *)0x40020000 )->CPU_CLOCK;
   
  temp &= (~(uint32_t)0x03);
   
  temp |= RST_CLK_CPU_PLLsource;
   
  ((RST_CLK_TypeDef *)0x40020000 )->CPU_CLOCK = temp;

   
  temp = ((RST_CLK_TypeDef *)0x40020000 )->PLL_CONTROL;
   
  temp &= (~(uint32_t)(0x0F << 8));
   
  temp |= (RST_CLK_CPU_PLLmul<<8);
   
  ((RST_CLK_TypeDef *)0x40020000 )->PLL_CONTROL = temp;

  if ( *(volatile uint32_t *) (0x42000000 + ((uint32_t)&((RST_CLK_TypeDef*)(0x40020000 - 0x40000000))->PLL_CONTROL)*32 + 2*4))
  {
     *(volatile uint32_t *) (0x42000000 + ((uint32_t)&((RST_CLK_TypeDef*)(0x40020000 - 0x40000000))->PLL_CONTROL)*32 + 3*4) = (uint32_t)0x01;
  }
}





 
void RST_CLK_CPU_PLLuse(FunctionalState UsePLL)
{
   
  ((void)0);

 
 ((RST_CLK_TypeDef *)0x40020000 )->CPU_CLOCK|=1<<2;
}





 
void RST_CLK_CPU_PLLcmd(FunctionalState NewState)
{
  ((void)0);

 
 ((RST_CLK_TypeDef *)0x40020000 )->PLL_CONTROL|=1<<2;
}





 
ErrorStatus RST_CLK_CPU_PLLstatus(void)
{
  volatile uint32_t startCounter = 0;
  ErrorStatus state;
  FlagStatus flag;

   
  do
  {
    flag = RST_CLK_GetFlagStatus(((uint32_t)(0x20 | 1)));
    startCounter++;
  } while ((startCounter < ((uint16_t)0x0600)) && (flag == RESET));

  if (RST_CLK_GetFlagStatus(((uint32_t)(0x20 | 1))) != RESET)
  {
    state = SUCCESS;
  }
  else
  {
    state = ERROR;
  }
  return state;
}














 
void RST_CLK_CPUclkPrescaler(uint32_t CPUclkDivValue)
{
  uint32_t temp;

   
  ((void)0);

  temp = ((RST_CLK_TypeDef *)0x40020000 )->CPU_CLOCK;
   
  temp &= (~(uint32_t)0x00000F0);
   
  temp |= CPUclkDivValue;
   
  ((RST_CLK_TypeDef *)0x40020000 )->CPU_CLOCK = temp;
}









 
void RST_CLK_CPUclkSelection(uint32_t CPU_CLK)
{
  uint32_t temp;

   
  ((void)0);

  temp = ((RST_CLK_TypeDef *)0x40020000 )->CPU_CLOCK;
   
  temp &= (~(uint32_t)(0x03 << 8));
   
  temp |= CPU_CLK;
   
  ((RST_CLK_TypeDef *)0x40020000 )->CPU_CLOCK = temp;
}



























 
void RST_CLK_USB_PLLconfig(uint32_t RST_CLK_USB_PLLsource, uint32_t RST_CLK_USB_PLLmul)
{
  uint32_t temp;

   
  ((void)0);
  ((void)0);

   
  temp = ((RST_CLK_TypeDef *)0x40020000 )->USB_CLOCK;
   
  temp &= (~(uint32_t)0x03);
   
  temp |= RST_CLK_USB_PLLsource;
   
  ((RST_CLK_TypeDef *)0x40020000 )->USB_CLOCK = temp;

   
  temp = ((RST_CLK_TypeDef *)0x40020000 )->PLL_CONTROL;
   
  temp &= (~(uint32_t)(0x0F << 4));
   
  temp |= (RST_CLK_USB_PLLmul<<4);
   
  ((RST_CLK_TypeDef *)0x40020000 )->PLL_CONTROL = temp;

  if ( *(volatile uint32_t *) (0x42000000 + ((uint32_t)&((RST_CLK_TypeDef*)(0x40020000 - 0x40000000))->PLL_CONTROL)*32 + 0*4))
  {
    *(volatile uint32_t *) (0x42000000 + ((uint32_t)&((RST_CLK_TypeDef*)(0x40020000 - 0x40000000))->PLL_CONTROL)*32 + 1*4)  = (uint32_t)0x01;
  }
}





 
void RST_CLK_USB_PLLuse(FunctionalState UsePLL)
{
   
  ((void)0);

  *(volatile uint32_t *) (0x42000000 + ((uint32_t)&((RST_CLK_TypeDef*)(0x40020000 - 0x40000000))->USB_CLOCK)*32 + 2*4) = (uint32_t)UsePLL;
}





 
void RST_CLK_USB_PLLcmd(FunctionalState NewState)
{
   
  ((void)0);

  *(volatile uint32_t *) (0x42000000 + ((uint32_t)&((RST_CLK_TypeDef*)(0x40020000 - 0x40000000))->PLL_CONTROL)*32 + 0*4) = (uint32_t)NewState;
}





 
ErrorStatus RST_CLK_USB_PLLstatus(void)
{
  volatile uint32_t startCounter = 0;
  ErrorStatus state;
  FlagStatus flag;

   
  do
  {
    flag = RST_CLK_GetFlagStatus(((uint32_t)(0x20 | 0)));
    startCounter++;
  } while ((startCounter < ((uint16_t)0x0600)) && (flag == RESET));

  if (RST_CLK_GetFlagStatus(((uint32_t)(0x20 | 0))) != RESET)
  {
    state = SUCCESS;
  }
  else
  {
    state = ERROR;
  }
  return state;
}







 
void RST_CLK_USBclkPrescaler(FunctionalState NewState)
{
   
  ((void)0);

  *(volatile uint32_t *) (0x42000000 + ((uint32_t)&((RST_CLK_TypeDef*)(0x40020000 - 0x40000000))->USB_CLOCK)*32 + 8*4) = (uint32_t)NewState;
}







 
void RST_CLK_USBclkEnable(FunctionalState NewState)
{
   
  ((void)0);

  *(volatile uint32_t *) (0x42000000 + ((uint32_t)&((RST_CLK_TypeDef*)(0x40020000 - 0x40000000))->USB_CLOCK)*32 + 4*4) = (uint32_t)NewState;
}












 
void RST_CLK_ADCclkSelection(uint32_t ADC_CLK)
{
  uint32_t temp;

   
  ((void)0);

   
  temp = ((RST_CLK_TypeDef *)0x40020000 )->ADC_MCO_CLOCK;
   
  temp &= (~(uint32_t)((0x03 << 4) | 0x03));
   
  temp |= ADC_CLK;
   
  ((RST_CLK_TypeDef *)0x40020000 )->ADC_MCO_CLOCK = temp;
}






















 
void RST_CLK_ADCclkPrescaler(uint32_t ADCclkDivValue)
{
  uint32_t temp;

   
  ((void)0);

  temp = ((RST_CLK_TypeDef *)0x40020000 )->ADC_MCO_CLOCK;
   
  temp &= (~(uint32_t)(0x0F << 8));
   
  temp |= ADCclkDivValue<<8;
   
  ((RST_CLK_TypeDef *)0x40020000 )->ADC_MCO_CLOCK = temp;
}







 
void RST_CLK_ADCclkEnable(FunctionalState NewState)
{
   
  ((void)0);

  *(volatile uint32_t *) (0x42000000 + ((uint32_t)&((RST_CLK_TypeDef*)(0x40020000 - 0x40000000))->ADC_MCO_CLOCK)*32 + 13*4) = (uint32_t)NewState;
}






















 
void RST_CLK_HSIclkPrescaler(uint32_t HSIclkDivValue)
{
  uint32_t temp;

   
  ((void)0);

  temp = ((RST_CLK_TypeDef *)0x40020000 )->RTC_CLOCK;
   
  temp &= (~(uint32_t)(0x0F << 4));
   
  temp |= HSIclkDivValue<<4;
   
  ((RST_CLK_TypeDef *)0x40020000 )->RTC_CLOCK = temp;
}







 
void RST_CLK_RTC_HSIclkEnable(FunctionalState NewState)
{
   
  ((void)0);

  *(volatile uint32_t *) (0x42000000 + ((uint32_t)&((RST_CLK_TypeDef*)(0x40020000 - 0x40000000))->RTC_CLOCK)*32 + 9*4) = (uint32_t)NewState;
}






















 
void RST_CLK_HSEclkPrescaler(uint32_t HSEclkDivValue)
{
  uint32_t temp;

   
  ((void)0);

  temp = ((RST_CLK_TypeDef *)0x40020000 )->RTC_CLOCK;
   
  temp &= (~(uint32_t)(0x0F << 0));
   
  temp |= HSEclkDivValue<<0;
   
  ((RST_CLK_TypeDef *)0x40020000 )->RTC_CLOCK = temp;
}







 
void RST_CLK_RTC_HSEclkEnable(FunctionalState NewState)
{
   
  ((void)0);

  *(volatile uint32_t *) (0x42000000 + ((uint32_t)&((RST_CLK_TypeDef*)(0x40020000 - 0x40000000))->RTC_CLOCK)*32 + 8*4) = (uint32_t)NewState;
}
















 
void RST_CLK_PCLKcmd(uint32_t RST_CLK_PCLK, FunctionalState NewState)
{
   
  ((void)0);
  ((void)0);
  if (NewState != DISABLE)
  {
    ((RST_CLK_TypeDef *)0x40020000 )->PER_CLOCK |= RST_CLK_PCLK;
  }
  else
  {
    ((RST_CLK_TypeDef *)0x40020000 )->PER_CLOCK &= ~RST_CLK_PCLK;
  }
}







 
void RST_CLK_GetClocksFreq(RST_CLK_FreqTypeDef* RST_CLK_Clocks)
{
  uint32_t cpu_c1_freq, cpu_c2_freq, cpu_c3_freq;
  uint32_t usb_c1_freq, usb_c2_freq, usb_c3_freq;
  uint32_t adc_c1_freq, adc_c2_freq, adc_c3_freq;
  uint32_t hsi_c1_freq, hse_c1_freq;
  uint32_t pll_mul;

   
  ((void)0);

   

   

   
  if (*(volatile uint32_t *) (0x42000000 + ((uint32_t)&((RST_CLK_TypeDef*)(0x40020000 - 0x40000000))->CPU_CLOCK)*32 + 1*4))
  {
    cpu_c1_freq = ((uint32_t)8000000);
  }
  else
  {
    cpu_c1_freq = ((uint32_t)8000000);
  }

  if (*(volatile uint32_t *) (0x42000000 + ((uint32_t)&((RST_CLK_TypeDef*)(0x40020000 - 0x40000000))->CPU_CLOCK)*32 + 0*4))
  {
    cpu_c1_freq /= 2;
  }

   
  cpu_c2_freq = cpu_c1_freq;

  if (*(volatile uint32_t *) (0x42000000 + ((uint32_t)&((RST_CLK_TypeDef*)(0x40020000 - 0x40000000))->CPU_CLOCK)*32 + 2*4) != 0)
  {
     
    pll_mul = ((((RST_CLK_TypeDef *)0x40020000 )->PLL_CONTROL >> 8) & (uint32_t)0x0F) + 1;
    cpu_c2_freq *= pll_mul;
  }

   
  switch ((((RST_CLK_TypeDef *)0x40020000 )->CPU_CLOCK >> 8) & (uint32_t)0x03)
  {
    case 0 :
       
      RST_CLK_Clocks->CPU_CLK_Frequency = ((uint32_t)8000000);
      break;
    case 1 :
       
       
      if (( ((RST_CLK_TypeDef *)0x40020000 )->CPU_CLOCK >> 4 & (uint32_t)0x08) == 0x00)
      {
        cpu_c3_freq = cpu_c2_freq;
      }
      else
      {
        cpu_c3_freq = cpu_c2_freq/(1 << ((((RST_CLK_TypeDef *)0x40020000 )->CPU_CLOCK >> 4 & (uint32_t)0x07) + 1));
      }
      RST_CLK_Clocks->CPU_CLK_Frequency = cpu_c3_freq;
      break;
    case 2 :
       
      RST_CLK_Clocks->CPU_CLK_Frequency = ((uint32_t)32768);
      break;
    default :  
       
      RST_CLK_Clocks->CPU_CLK_Frequency = ((uint32_t)40000);
      break;
  }

   

   
  if (*(volatile uint32_t *) (0x42000000 + ((uint32_t)&((RST_CLK_TypeDef*)(0x40020000 - 0x40000000))->USB_CLOCK)*32 + 1*4))
  {
    usb_c1_freq = ((uint32_t)8000000);
  }
  else
  {
    usb_c1_freq = ((uint32_t)8000000);
  }

  if (*(volatile uint32_t *) (0x42000000 + ((uint32_t)&((RST_CLK_TypeDef*)(0x40020000 - 0x40000000))->USB_CLOCK)*32 + 0*4))
  {
    usb_c1_freq /= 2;
  }

   
  usb_c2_freq = usb_c1_freq;

  if (*(volatile uint32_t *) (0x42000000 + ((uint32_t)&((RST_CLK_TypeDef*)(0x40020000 - 0x40000000))->USB_CLOCK)*32 + 2*4) != 0)
  {
     
    pll_mul = ((((RST_CLK_TypeDef *)0x40020000 )->PLL_CONTROL >> 4) & (uint32_t)0x0F) + 1;
    usb_c2_freq *= pll_mul;
  }

   
  usb_c3_freq = usb_c2_freq;
  if (*(volatile uint32_t *) (0x42000000 + ((uint32_t)&((RST_CLK_TypeDef*)(0x40020000 - 0x40000000))->USB_CLOCK)*32 + 4*4) != 0)
  {
    usb_c3_freq /= 2;
  }

  RST_CLK_Clocks->USB_CLK_Frequency = usb_c3_freq;

   
  hsi_c1_freq = ((uint32_t)8000000)/((((RST_CLK_TypeDef *)0x40020000 )->RTC_CLOCK >> 4 & (uint32_t)0x0F) + 1);
  RST_CLK_Clocks->RTCHSI_Frequency = hsi_c1_freq;

   
  hse_c1_freq = ((uint32_t)8000000)/((((RST_CLK_TypeDef *)0x40020000 )->RTC_CLOCK >> 0 & (uint32_t)0x0F) + 1);
  RST_CLK_Clocks->RTCHSE_Frequency = hse_c1_freq;

   

   
  switch ((((RST_CLK_TypeDef *)0x40020000 )->ADC_MCO_CLOCK >> 0) & (uint32_t)0x03)
  {
    case 0 :
       
      adc_c1_freq = cpu_c1_freq;
      break;
    case 1 :
       
      adc_c1_freq = usb_c1_freq;
      break;
    case 2 :
       
      adc_c1_freq = cpu_c2_freq;
      break;
    default :  
       
      adc_c1_freq = usb_c2_freq;
      break;
  }

   
  switch ((((RST_CLK_TypeDef *)0x40020000 )->ADC_MCO_CLOCK >> 4) & (uint32_t)0x03)
  {
    case 0 :
       
      adc_c2_freq = ((uint32_t)32768);
      break;
    case 1 :
       
      adc_c2_freq = ((uint32_t)40000);
      break;
    case 2 :
       
      adc_c2_freq = adc_c1_freq;
      break;
    default :  
       
      adc_c2_freq = hsi_c1_freq;
      break;
  }

  adc_c3_freq = adc_c2_freq /((((RST_CLK_TypeDef *)0x40020000 )->ADC_MCO_CLOCK >> 8 & (uint32_t)0x0F) + 1);
  RST_CLK_Clocks->ADC_CLK_Frequency = adc_c3_freq;
}












 
FlagStatus RST_CLK_GetFlagStatus(uint32_t RST_CLK_FLAG)
{
  uint32_t statusreg;
  FlagStatus bitstatus;

   
  ((void)0);

   
  if ((((uint8_t)0xE0) & RST_CLK_FLAG) == (uint32_t)0x20)  
  {
    statusreg = ((RST_CLK_TypeDef *)0x40020000 )->CLOCK_STATUS;
  }
  else                                                   
  {
    statusreg = ((BKP_TypeDef *)0x400D8000 )->REG_0F;
  }

   
  if ((statusreg & ((uint32_t)1 << (RST_CLK_FLAG & ((uint8_t)0x1F)))) != (uint32_t)0x00)
  {
    bitstatus = SET;
  }
  else
  {
    bitstatus = RESET;
  }

   
  return bitstatus;
}

   

   

   



 

