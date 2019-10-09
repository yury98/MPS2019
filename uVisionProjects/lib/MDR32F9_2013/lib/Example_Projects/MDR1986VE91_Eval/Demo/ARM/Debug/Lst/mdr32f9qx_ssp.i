#line 1 "..\\..\\..\\..\\Libraries\\MDR32F9Qx_StdPeriph_Driver\\src\\MDR32F9Qx_ssp.c"




















 

 
#line 1 "..\\..\\Config\\MDR32F9Qx_config.h"

















 





















 

 



#line 1 "..\\..\\Config\\MDR32F9Qx_board.h"

















 

 




 
#line 51 "..\\..\\Config\\MDR32F9Qx_board.h"





 

#line 47 "..\\..\\Config\\MDR32F9Qx_config.h"
#line 1 "..\\..\\..\\..\\Libraries\\MDR32F9Qx_StdPeriph_Driver\\inc\\MDR32F9Qx_lib.h"





















 

 



#line 55 "..\\..\\..\\..\\Libraries\\MDR32F9Qx_StdPeriph_Driver\\inc\\MDR32F9Qx_lib.h"



 






 
#line 48 "..\\..\\Config\\MDR32F9Qx_config.h"
#line 1 "..\\..\\..\\..\\Libraries\\CMSIS\\CM3\\CoreSupport\\stdint.h"
 
 
 
 
 
 





   typedef   signed          char int8_t;
   typedef   signed short     int int16_t;
   typedef   signed           int int32_t;

   typedef unsigned          char uint8_t;
   typedef unsigned short     int uint16_t;
   typedef unsigned           int uint32_t;

   typedef   signed          char int_least8_t;
   typedef   signed short     int int_least16_t;
   typedef   signed           int int_least32_t;

   typedef unsigned          char uint_least8_t;
   typedef unsigned short     int uint_least16_t;
   typedef unsigned           int uint_least32_t;

   typedef   signed           int int_fast8_t;
   typedef   signed           int int_fast16_t;
   typedef   signed           int int_fast32_t;

   typedef unsigned           int uint_fast8_t;
   typedef unsigned           int uint_fast16_t;
   typedef unsigned           int uint_fast32_t;

   typedef   signed           int intptr_t;
   typedef unsigned           int uintptr_t;


































































 


#line 49 "..\\..\\Config\\MDR32F9Qx_config.h"

#line 56 "..\\..\\Config\\MDR32F9Qx_config.h"


 
#line 1 "..\\..\\..\\..\\Libraries\\CMSIS\\CM3\\DeviceSupport\\MDR32F9Qx\\inc\\MDR32Fx.h"





















 

 





 



 



 






  #pragma anon_unions


 
typedef enum IRQn
{
 
  NonMaskableInt_IRQn     = -14,   
  HardFault_IRQn          = -13,   
  MemoryManagement_IRQn   = -12,   
  BusFault_IRQn           = -11,   
  UsageFault_IRQn         = -10,   
  SVCall_IRQn             = -5,    
  PendSV_IRQn             = -2,    
  SysTick_IRQn            = -1,    

 
  CAN1_IRQn               =  0,    
  CAN2_IRQn               =  1,    
  USB_IRQn                =  2,    
  DMA_IRQn                =  5,    
  UART1_IRQn              =  6,    
  UART2_IRQn              =  7,    
  SSP1_IRQn               =  8,    
  I2C_IRQn                =  10,   
  POWER_IRQn              =  11,   
  WWDG_IRQn               =  12,   
  Timer1_IRQn             =  14,   
  Timer2_IRQn             =  15,   
  Timer3_IRQn             =  16,   
  ADC_IRQn                =  17,   
  COMPARATOR_IRQn         =  19,   
  SSP2_IRQn               =  20,   
  BACKUP_IRQn             =  27,   
  EXT_INT1_IRQn           =  28,   
  EXT_INT2_IRQn           =  29,   
  EXT_INT3_IRQn           =  30,   
  EXT_INT4_IRQn           =  31    
}IRQn_Type;



 

 





   

 
#line 1 "..\\..\\..\\..\\Libraries\\CMSIS\\CM3\\CoreSupport\\core_cm3.h"
 




















 






 



 


































 

 
 
 
 
 
 
 
 








 











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






 
#line 99 "..\\..\\..\\..\\Libraries\\CMSIS\\CM3\\CoreSupport\\core_cm3.h"

















 

#line 125 "..\\..\\..\\..\\Libraries\\CMSIS\\CM3\\CoreSupport\\core_cm3.h"





 


 





 
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

 



 








   






 
typedef struct
{
  volatile const  uint32_t TYPE;                          
  volatile uint32_t CTRL;                          
  volatile uint32_t RNR;                           
  volatile uint32_t RBAR;                          
  volatile uint32_t RASR;                          
  volatile uint32_t RBAR_A1;                       
  volatile uint32_t RASR_A1;                       
  volatile uint32_t RBAR_A2;                       
  volatile uint32_t RASR_A2;                       
  volatile uint32_t RBAR_A3;                       
  volatile uint32_t RASR_A3;                       
} MPU_Type;

 









 









 



 









 



























   






 
typedef struct
{
  volatile uint32_t DHCSR;                         
  volatile  uint32_t DCRSR;                         
  volatile uint32_t DCRDR;                         
  volatile uint32_t DEMCR;                         
} CoreDebug_Type;

 




































 






 






































   


 
#line 729 "..\\..\\..\\..\\Libraries\\CMSIS\\CM3\\CoreSupport\\core_cm3.h"

#line 736 "..\\..\\..\\..\\Libraries\\CMSIS\\CM3\\CoreSupport\\core_cm3.h"






   




 





#line 770 "..\\..\\..\\..\\Libraries\\CMSIS\\CM3\\CoreSupport\\core_cm3.h"


 


 




#line 795 "..\\..\\..\\..\\Libraries\\CMSIS\\CM3\\CoreSupport\\core_cm3.h"


 
 
 
 








 
extern uint32_t __get_PSP(void);








 
extern void __set_PSP(uint32_t topOfProcStack);








 
extern uint32_t __get_MSP(void);








 
extern void __set_MSP(uint32_t topOfMainStack);








 
extern uint32_t __REV16(uint16_t value);








 
extern int32_t __REVSH(int16_t value);


#line 945 "..\\..\\..\\..\\Libraries\\CMSIS\\CM3\\CoreSupport\\core_cm3.h"





 








 
static __inline uint32_t  __get_BASEPRI(void)
{
  register uint32_t __regBasePri         __asm("basepri");
  return(__regBasePri);
}







 
static __inline void __set_BASEPRI(uint32_t basePri)
{
  register uint32_t __regBasePri         __asm("basepri");
  __regBasePri = (basePri & 0xff);
}







 
static __inline uint32_t __get_PRIMASK(void)
{
  register uint32_t __regPriMask         __asm("primask");
  return(__regPriMask);
}







 
static __inline void __set_PRIMASK(uint32_t priMask)
{
  register uint32_t __regPriMask         __asm("primask");
  __regPriMask = (priMask);
}







 
static __inline uint32_t __get_FAULTMASK(void)
{
  register uint32_t __regFaultMask       __asm("faultmask");
  return(__regFaultMask);
}







 
static __inline void __set_FAULTMASK(uint32_t faultMask)
{
  register uint32_t __regFaultMask       __asm("faultmask");
  __regFaultMask = (faultMask & 1);
}







 
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





#line 1468 "..\\..\\..\\..\\Libraries\\CMSIS\\CM3\\CoreSupport\\core_cm3.h"







 
 


 











 
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
    ((SCB_Type *) ((0xE000E000) + 0x0D00))->SHP[((uint32_t)(IRQn) & 0xF)-4] = ((priority << (8 - 3)) & 0xff); }  
  else {
    ((NVIC_Type *) ((0xE000E000) + 0x0100))->IP[(uint32_t)(IRQn)] = ((priority << (8 - 3)) & 0xff);    }         
}















 
static __inline uint32_t NVIC_GetPriority(IRQn_Type IRQn)
{

  if(IRQn < 0) {
    return((uint32_t)(((SCB_Type *) ((0xE000E000) + 0x0D00))->SHP[((uint32_t)(IRQn) & 0xF)-4] >> (8 - 3)));  }  
  else {
    return((uint32_t)(((NVIC_Type *) ((0xE000E000) + 0x0100))->IP[(uint32_t)(IRQn)]           >> (8 - 3)));  }  
}
















 
static __inline uint32_t NVIC_EncodePriority (uint32_t PriorityGroup, uint32_t PreemptPriority, uint32_t SubPriority)
{
  uint32_t PriorityGroupTmp = (PriorityGroup & 0x07);           
  uint32_t PreemptPriorityBits;
  uint32_t SubPriorityBits;

  PreemptPriorityBits = ((7 - PriorityGroupTmp) > 3) ? 3 : 7 - PriorityGroupTmp;
  SubPriorityBits     = ((PriorityGroupTmp + 3) < 7) ? 0 : PriorityGroupTmp - 7 + 3;

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

  PreemptPriorityBits = ((7 - PriorityGroupTmp) > 3) ? 3 : 7 - PriorityGroupTmp;
  SubPriorityBits     = ((PriorityGroupTmp + 3) < 7) ? 0 : PriorityGroupTmp - 7 + 3;

  *pPreemptPriority = (Priority >> SubPriorityBits) & ((1 << (PreemptPriorityBits)) - 1);
  *pSubPriority     = (Priority                   ) & ((1 << (SubPriorityBits    )) - 1);
}


 












 
static __inline uint32_t SysTick_Config(uint32_t ticks)
{
  if (ticks > (0xFFFFFFul << 0))  return (1);             

  ((SysTick_Type *) ((0xE000E000) + 0x0010))->LOAD  = (ticks & (0xFFFFFFul << 0)) - 1;       
  NVIC_SetPriority (SysTick_IRQn, (1<<3) - 1);   
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


   






   



 

    

    

#line 99 "..\\..\\..\\..\\Libraries\\CMSIS\\CM3\\DeviceSupport\\MDR32F9Qx\\inc\\MDR32Fx.h"
#line 1 "..\\..\\..\\..\\Libraries\\CMSIS\\CM3\\DeviceSupport\\MDR32F9Qx\\startup\\arm\\system_MDR32F9Qx.h"




















 



 



 



 

 





 

extern uint32_t SystemCoreClock;          
 

   



 

extern void SystemInit(void);
extern void SystemCoreClockUpdate(void);

   



   

   

   



 
#line 100 "..\\..\\..\\..\\Libraries\\CMSIS\\CM3\\DeviceSupport\\MDR32F9Qx\\inc\\MDR32Fx.h"



 

typedef enum {RESET = 0, SET = !RESET} FlagStatus, ITStatus, BitStatus;



typedef enum {DISABLE = 0, ENABLE = !DISABLE} FunctionalState;


typedef enum {ERROR = 0, SUCCESS = !ERROR} ErrorStatus;

   



 



 



 

 
typedef struct
{
  volatile uint32_t ID;
  volatile uint32_t DLC;
  volatile uint32_t DATAL;
  volatile uint32_t DATAH;
}MDR_CAN_BUF_TypeDef;

 
typedef struct
{
  volatile uint32_t MASK;
  volatile uint32_t FILTER;
}MDR_CAN_BUF_FILTER_TypeDef;

 
typedef struct
{
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
  volatile uint32_t BUF_CON[32];
  volatile uint32_t INT_RX;
  volatile uint32_t RX;
  volatile uint32_t INT_TX;
  volatile uint32_t TX;
       uint32_t RESERVED2[76];
    MDR_CAN_BUF_TypeDef CAN_BUF[32];
       uint32_t RESERVED3[64];
    MDR_CAN_BUF_FILTER_TypeDef CAN_BUF_FILTER[32];
}MDR_CAN_TypeDef;

   



 



  

 
 






 






   



  

 
 
#line 220 "..\\..\\..\\..\\Libraries\\CMSIS\\CM3\\DeviceSupport\\MDR32F9Qx\\inc\\MDR32Fx.h"

 
#line 236 "..\\..\\..\\..\\Libraries\\CMSIS\\CM3\\DeviceSupport\\MDR32F9Qx\\inc\\MDR32Fx.h"

   



  

 
 
#line 251 "..\\..\\..\\..\\Libraries\\CMSIS\\CM3\\DeviceSupport\\MDR32F9Qx\\inc\\MDR32Fx.h"

 
#line 259 "..\\..\\..\\..\\Libraries\\CMSIS\\CM3\\DeviceSupport\\MDR32F9Qx\\inc\\MDR32Fx.h"

   



  

 
 






 






   



  

 
 
 
 



 



   



  

 
 
 
 
#line 314 "..\\..\\..\\..\\Libraries\\CMSIS\\CM3\\DeviceSupport\\MDR32F9Qx\\inc\\MDR32Fx.h"

 
#line 322 "..\\..\\..\\..\\Libraries\\CMSIS\\CM3\\DeviceSupport\\MDR32F9Qx\\inc\\MDR32Fx.h"

   



  

 
 
 
 





 





   



  

 
 
 
 





 





   



  

 
 
#line 381 "..\\..\\..\\..\\Libraries\\CMSIS\\CM3\\DeviceSupport\\MDR32F9Qx\\inc\\MDR32Fx.h"

 
#line 391 "..\\..\\..\\..\\Libraries\\CMSIS\\CM3\\DeviceSupport\\MDR32F9Qx\\inc\\MDR32Fx.h"

   

   

   



 



 

 
typedef struct
{
  volatile uint32_t CTRL;
  volatile uint32_t STS;
  volatile uint32_t TS;
  volatile uint32_t NTS;
}MDR_USB_SEP_TypeDef;

 
typedef struct
{
  volatile uint32_t RXFD;
       uint32_t RESERVED0;
  volatile uint32_t RXFDC_L;
  volatile uint32_t RXFDC_H;
  volatile uint32_t RXFC;
       uint32_t RESERVED1[11];
  volatile uint32_t TXFD;
       uint32_t RESERVED2[3];
  volatile uint32_t TXFDC;
       uint32_t RESERVED3[11];
}MDR_USB_SEP_FIFO_TypeDef;

 
typedef struct
{
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
  volatile uint32_t HTXFC;
       uint32_t RESERVED4[11];
    MDR_USB_SEP_TypeDef USB_SEP[4];
  volatile uint32_t SC;
  volatile uint32_t SLS;
  volatile uint32_t SIS;
  volatile uint32_t SIM;
  volatile uint32_t SA;
  volatile uint32_t SFN_L;
  volatile uint32_t SFN_H;
       uint32_t RESERVED5[9];
    MDR_USB_SEP_FIFO_TypeDef USB_SEP_FIFO[4];
  volatile uint32_t HSCR;
  volatile uint32_t HSVR;
}MDR_USB_TypeDef;

   



 



  

 
 





 





   



  

 
 





 





   



  

 
 





 





   



  

 
 





 





   



  

 
 
#line 570 "..\\..\\..\\..\\Libraries\\CMSIS\\CM3\\DeviceSupport\\MDR32F9Qx\\inc\\MDR32Fx.h"

 
#line 580 "..\\..\\..\\..\\Libraries\\CMSIS\\CM3\\DeviceSupport\\MDR32F9Qx\\inc\\MDR32Fx.h"

   



  

 
 






 






   



  

 
 
#line 618 "..\\..\\..\\..\\Libraries\\CMSIS\\CM3\\DeviceSupport\\MDR32F9Qx\\inc\\MDR32Fx.h"

 
#line 628 "..\\..\\..\\..\\Libraries\\CMSIS\\CM3\\DeviceSupport\\MDR32F9Qx\\inc\\MDR32Fx.h"

   



  

 
 






 






   



  

 
 






 






   



  

 
 






 






   



  

 
 
#line 708 "..\\..\\..\\..\\Libraries\\CMSIS\\CM3\\DeviceSupport\\MDR32F9Qx\\inc\\MDR32Fx.h"

 
#line 718 "..\\..\\..\\..\\Libraries\\CMSIS\\CM3\\DeviceSupport\\MDR32F9Qx\\inc\\MDR32Fx.h"

   



  

 
 



 



   

   

   



 



 

 
typedef struct
{
  volatile uint32_t CMD;
  volatile uint32_t ADR;
  volatile uint32_t DI;
  volatile uint32_t DO;
  volatile uint32_t KEY;
}MDR_EEPROM_TypeDef;

   



 



  

 
 
#line 782 "..\\..\\..\\..\\Libraries\\CMSIS\\CM3\\DeviceSupport\\MDR32F9Qx\\inc\\MDR32Fx.h"

 
#line 796 "..\\..\\..\\..\\Libraries\\CMSIS\\CM3\\DeviceSupport\\MDR32F9Qx\\inc\\MDR32Fx.h"

   

   

   



 



 

 
typedef struct
{
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
}MDR_RST_CLK_TypeDef;

   



 



  

 
 




 




   



  

 
 
#line 863 "..\\..\\..\\..\\Libraries\\CMSIS\\CM3\\DeviceSupport\\MDR32F9Qx\\inc\\MDR32Fx.h"

 
#line 871 "..\\..\\..\\..\\Libraries\\CMSIS\\CM3\\DeviceSupport\\MDR32F9Qx\\inc\\MDR32Fx.h"

   



  

 
 



 



   



  

 
 





 





   



  

 
 





 





   



  

 
 





 





   



  

 
 





 





   



  

 
 





 





   



  

 
 
#line 996 "..\\..\\..\\..\\Libraries\\CMSIS\\CM3\\DeviceSupport\\MDR32F9Qx\\inc\\MDR32Fx.h"

 
#line 1004 "..\\..\\..\\..\\Libraries\\CMSIS\\CM3\\DeviceSupport\\MDR32F9Qx\\inc\\MDR32Fx.h"

   



  

 
 





 





   



  

 
 





 





   

   

   



 



 

 
typedef struct
{
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
}MDR_DMA_TypeDef;

   



 



  

 
 





 





   



  

 
 



 



   

   

   



 



 

 
typedef struct
{
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
}MDR_UART_TypeDef;

   



 



  

 
 






 






   



  

 
 





 





   



  

 
 
#line 1214 "..\\..\\..\\..\\Libraries\\CMSIS\\CM3\\DeviceSupport\\MDR32F9Qx\\inc\\MDR32Fx.h"

 
#line 1225 "..\\..\\..\\..\\Libraries\\CMSIS\\CM3\\DeviceSupport\\MDR32F9Qx\\inc\\MDR32Fx.h"

   



  

 
 
#line 1241 "..\\..\\..\\..\\Libraries\\CMSIS\\CM3\\DeviceSupport\\MDR32F9Qx\\inc\\MDR32Fx.h"

 
#line 1250 "..\\..\\..\\..\\Libraries\\CMSIS\\CM3\\DeviceSupport\\MDR32F9Qx\\inc\\MDR32Fx.h"

   



  

 
 
#line 1271 "..\\..\\..\\..\\Libraries\\CMSIS\\CM3\\DeviceSupport\\MDR32F9Qx\\inc\\MDR32Fx.h"

 
#line 1285 "..\\..\\..\\..\\Libraries\\CMSIS\\CM3\\DeviceSupport\\MDR32F9Qx\\inc\\MDR32Fx.h"

   



  

 
 



 



   



  

 
 
#line 1320 "..\\..\\..\\..\\Libraries\\CMSIS\\CM3\\DeviceSupport\\MDR32F9Qx\\inc\\MDR32Fx.h"

 
#line 1333 "..\\..\\..\\..\\Libraries\\CMSIS\\CM3\\DeviceSupport\\MDR32F9Qx\\inc\\MDR32Fx.h"

   



  

 
 
#line 1353 "..\\..\\..\\..\\Libraries\\CMSIS\\CM3\\DeviceSupport\\MDR32F9Qx\\inc\\MDR32Fx.h"

 
#line 1366 "..\\..\\..\\..\\Libraries\\CMSIS\\CM3\\DeviceSupport\\MDR32F9Qx\\inc\\MDR32Fx.h"

   



  

 
 
#line 1386 "..\\..\\..\\..\\Libraries\\CMSIS\\CM3\\DeviceSupport\\MDR32F9Qx\\inc\\MDR32Fx.h"

 
#line 1399 "..\\..\\..\\..\\Libraries\\CMSIS\\CM3\\DeviceSupport\\MDR32F9Qx\\inc\\MDR32Fx.h"

   



  

 
 
#line 1419 "..\\..\\..\\..\\Libraries\\CMSIS\\CM3\\DeviceSupport\\MDR32F9Qx\\inc\\MDR32Fx.h"

 
#line 1432 "..\\..\\..\\..\\Libraries\\CMSIS\\CM3\\DeviceSupport\\MDR32F9Qx\\inc\\MDR32Fx.h"

   



  

 
 




 




   

   

   



 



 

 
typedef struct
{
  volatile uint32_t CR0;
  volatile uint32_t CR1;
  volatile uint32_t DR;
  volatile uint32_t SR;
  volatile uint32_t CPSR;
  volatile uint32_t IMSC;
  volatile uint32_t RIS;
  volatile uint32_t MIS;
  volatile uint32_t ICR;
  volatile uint32_t DMACR;
}MDR_SSP_TypeDef;

   



 



  

 
 






 






   



  

 
 





 





   



  

 
 






 






   



  

 
 





 





   



  

 
 





 





   



  

 
 





 





   



  

 
 



 



   



  

 
 



 



   

   

   



 



 

 
typedef struct
{
  volatile uint32_t PRL;
  volatile uint32_t PRH;
  volatile uint32_t CTR;
  volatile uint32_t RXD;
  volatile uint32_t STA;
  volatile uint32_t TXD;
  volatile uint32_t CMD;
}MDR_I2C_TypeDef;

   



 



  

 
 




 




   



  

 
 






 






   



  

 
 
#line 1713 "..\\..\\..\\..\\Libraries\\CMSIS\\CM3\\DeviceSupport\\MDR32F9Qx\\inc\\MDR32Fx.h"

 
#line 1721 "..\\..\\..\\..\\Libraries\\CMSIS\\CM3\\DeviceSupport\\MDR32F9Qx\\inc\\MDR32Fx.h"

   

   

   



 



 

 
typedef struct
{
  volatile uint32_t PVDCS;
}MDR_POWER_TypeDef;

   



 



  

 
 
#line 1763 "..\\..\\..\\..\\Libraries\\CMSIS\\CM3\\DeviceSupport\\MDR32F9Qx\\inc\\MDR32Fx.h"

 
#line 1774 "..\\..\\..\\..\\Libraries\\CMSIS\\CM3\\DeviceSupport\\MDR32F9Qx\\inc\\MDR32Fx.h"

   

   

   



 



 

 
typedef struct
{
  volatile uint32_t CR;
  volatile uint32_t CFR;
  volatile uint32_t SR;
}MDR_WWDG_TypeDef;

   



 



  

 
 



 



   



  

 
 




 




   

   

   



 



 

 
typedef struct
{
  volatile uint32_t KR;
  volatile uint32_t PR;
  volatile uint32_t RLR;
  volatile uint32_t SR;
}MDR_IWDG_TypeDef;

   



 



  

 
 



 



   

   

   



 



 

 
typedef struct
{
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
}MDR_TIMER_TypeDef;

   



 



  

 
 
#line 1945 "..\\..\\..\\..\\Libraries\\CMSIS\\CM3\\DeviceSupport\\MDR32F9Qx\\inc\\MDR32Fx.h"

 
#line 1954 "..\\..\\..\\..\\Libraries\\CMSIS\\CM3\\DeviceSupport\\MDR32F9Qx\\inc\\MDR32Fx.h"

   



  

 
 
#line 1972 "..\\..\\..\\..\\Libraries\\CMSIS\\CM3\\DeviceSupport\\MDR32F9Qx\\inc\\MDR32Fx.h"

 
#line 1983 "..\\..\\..\\..\\Libraries\\CMSIS\\CM3\\DeviceSupport\\MDR32F9Qx\\inc\\MDR32Fx.h"

   



  

 
 
#line 1998 "..\\..\\..\\..\\Libraries\\CMSIS\\CM3\\DeviceSupport\\MDR32F9Qx\\inc\\MDR32Fx.h"

 
#line 2006 "..\\..\\..\\..\\Libraries\\CMSIS\\CM3\\DeviceSupport\\MDR32F9Qx\\inc\\MDR32Fx.h"

   



  

 
 




 




   



  

 
 





 





   



  

 
 
#line 2059 "..\\..\\..\\..\\Libraries\\CMSIS\\CM3\\DeviceSupport\\MDR32F9Qx\\inc\\MDR32Fx.h"

 
#line 2069 "..\\..\\..\\..\\Libraries\\CMSIS\\CM3\\DeviceSupport\\MDR32F9Qx\\inc\\MDR32Fx.h"

   



  

 
 
#line 2086 "..\\..\\..\\..\\Libraries\\CMSIS\\CM3\\DeviceSupport\\MDR32F9Qx\\inc\\MDR32Fx.h"

 
#line 2096 "..\\..\\..\\..\\Libraries\\CMSIS\\CM3\\DeviceSupport\\MDR32F9Qx\\inc\\MDR32Fx.h"

   



  

 
 
#line 2113 "..\\..\\..\\..\\Libraries\\CMSIS\\CM3\\DeviceSupport\\MDR32F9Qx\\inc\\MDR32Fx.h"

 
#line 2123 "..\\..\\..\\..\\Libraries\\CMSIS\\CM3\\DeviceSupport\\MDR32F9Qx\\inc\\MDR32Fx.h"

   



  

 
 




 




   

   

   



 



 

 
typedef struct
{
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
}MDR_ADC_TypeDef;

   



 



  

 
 
#line 2201 "..\\..\\..\\..\\Libraries\\CMSIS\\CM3\\DeviceSupport\\MDR32F9Qx\\inc\\MDR32Fx.h"

 
#line 2220 "..\\..\\..\\..\\Libraries\\CMSIS\\CM3\\DeviceSupport\\MDR32F9Qx\\inc\\MDR32Fx.h"

   



  

 
 
#line 2241 "..\\..\\..\\..\\Libraries\\CMSIS\\CM3\\DeviceSupport\\MDR32F9Qx\\inc\\MDR32Fx.h"

 
#line 2255 "..\\..\\..\\..\\Libraries\\CMSIS\\CM3\\DeviceSupport\\MDR32F9Qx\\inc\\MDR32Fx.h"

   



  

 
 
 



 



   



  

 
 
 






 






   

   

   



 



 

 
typedef struct
{
  volatile uint32_t CFG;
  volatile uint32_t DAC1_DATA;
  volatile uint32_t DAC2_DATA;
}MDR_DAC_TypeDef;

   



 



  

 
 






 






   



  

 
 



 



   



  

 
 



 



   

   

   



 



 

 
typedef struct
{
  volatile uint32_t CFG;
  volatile uint32_t RESULT;
  volatile uint32_t RESULT_LATCH;
}MDR_COMP_TypeDef;

   



 



  

 
 
#line 2415 "..\\..\\..\\..\\Libraries\\CMSIS\\CM3\\DeviceSupport\\MDR32F9Qx\\inc\\MDR32Fx.h"

 
#line 2427 "..\\..\\..\\..\\Libraries\\CMSIS\\CM3\\DeviceSupport\\MDR32F9Qx\\inc\\MDR32Fx.h"

   



  

 
 




 




   

   

   



 



 

 
typedef struct
{
  volatile uint32_t RXTX;
  volatile uint32_t OE;
  volatile uint32_t FUNC;
  volatile uint32_t ANALOG;
  volatile uint32_t PULL;
  volatile uint32_t PD;
  volatile uint32_t PWR;
  volatile uint32_t GFEN;
}MDR_PORT_TypeDef;

   



 



  

 
 
#line 2500 "..\\..\\..\\..\\Libraries\\CMSIS\\CM3\\DeviceSupport\\MDR32F9Qx\\inc\\MDR32Fx.h"

 
#line 2518 "..\\..\\..\\..\\Libraries\\CMSIS\\CM3\\DeviceSupport\\MDR32F9Qx\\inc\\MDR32Fx.h"

   



  

 
 



 



   



  

 
 



 



   



  

 
 
#line 2573 "..\\..\\..\\..\\Libraries\\CMSIS\\CM3\\DeviceSupport\\MDR32F9Qx\\inc\\MDR32Fx.h"

 
#line 2591 "..\\..\\..\\..\\Libraries\\CMSIS\\CM3\\DeviceSupport\\MDR32F9Qx\\inc\\MDR32Fx.h"

   

   

   



 



 

 
typedef struct
{
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
}MDR_BKP_TypeDef;

   



 



  

 
 
#line 2651 "..\\..\\..\\..\\Libraries\\CMSIS\\CM3\\DeviceSupport\\MDR32F9Qx\\inc\\MDR32Fx.h"

 
#line 2660 "..\\..\\..\\..\\Libraries\\CMSIS\\CM3\\DeviceSupport\\MDR32F9Qx\\inc\\MDR32Fx.h"

   



  

 
 
#line 2684 "..\\..\\..\\..\\Libraries\\CMSIS\\CM3\\DeviceSupport\\MDR32F9Qx\\inc\\MDR32Fx.h"

 
#line 2701 "..\\..\\..\\..\\Libraries\\CMSIS\\CM3\\DeviceSupport\\MDR32F9Qx\\inc\\MDR32Fx.h"

   



  

 
 
#line 2717 "..\\..\\..\\..\\Libraries\\CMSIS\\CM3\\DeviceSupport\\MDR32F9Qx\\inc\\MDR32Fx.h"

 
#line 2726 "..\\..\\..\\..\\Libraries\\CMSIS\\CM3\\DeviceSupport\\MDR32F9Qx\\inc\\MDR32Fx.h"

   

   

   



 



 

 
typedef struct
{
       uint32_t RESERVED0[20];
  volatile uint32_t NAND_CYCLES;
  volatile uint32_t CONTROL;



}MDR_EBC_TypeDef;

   



 



  

 
 
#line 2771 "..\\..\\..\\..\\Libraries\\CMSIS\\CM3\\DeviceSupport\\MDR32F9Qx\\inc\\MDR32Fx.h"

 
#line 2780 "..\\..\\..\\..\\Libraries\\CMSIS\\CM3\\DeviceSupport\\MDR32F9Qx\\inc\\MDR32Fx.h"

   



  

 
 
#line 2799 "..\\..\\..\\..\\Libraries\\CMSIS\\CM3\\DeviceSupport\\MDR32F9Qx\\inc\\MDR32Fx.h"

 
#line 2811 "..\\..\\..\\..\\Libraries\\CMSIS\\CM3\\DeviceSupport\\MDR32F9Qx\\inc\\MDR32Fx.h"


   

#line 2838 "..\\..\\..\\..\\Libraries\\CMSIS\\CM3\\DeviceSupport\\MDR32F9Qx\\inc\\MDR32Fx.h"

   

   

   




 

#line 2878 "..\\..\\..\\..\\Libraries\\CMSIS\\CM3\\DeviceSupport\\MDR32F9Qx\\inc\\MDR32Fx.h"

   



 

#line 2913 "..\\..\\..\\..\\Libraries\\CMSIS\\CM3\\DeviceSupport\\MDR32F9Qx\\inc\\MDR32Fx.h"

   





   

   

   





 
#line 61 "..\\..\\Config\\MDR32F9Qx_config.h"







 

 



 
 






 
#line 90 "..\\..\\Config\\MDR32F9Qx_config.h"


 




#line 116 "..\\..\\Config\\MDR32F9Qx_config.h"










 



 
 


 


 
            


 

 


 
 
 
 


 


 
 
 
 
 

 
 

 



 



 


 
 


 







 

 
 








 
#line 211 "..\\..\\Config\\MDR32F9Qx_config.h"

#line 225 "..\\..\\Config\\MDR32F9Qx_config.h"





 
#line 25 "..\\..\\..\\..\\Libraries\\MDR32F9Qx_StdPeriph_Driver\\src\\MDR32F9Qx_ssp.c"
#line 1 "..\\..\\..\\..\\Libraries\\MDR32F9Qx_StdPeriph_Driver\\inc\\MDR32F9Qx_ssp.h"





















 

 



 
#line 30 "..\\..\\..\\..\\Libraries\\MDR32F9Qx_StdPeriph_Driver\\inc\\MDR32F9Qx_ssp.h"



 



 



 



 

typedef struct
{
  uint16_t SSP_SCR;                        


 
  uint16_t SSP_CPSDVSR;                    
 
  uint16_t SSP_Mode;                       
 
  uint16_t SSP_WordLength;                 
 
  uint16_t SSP_SPH;                        
 
  uint16_t SSP_SPO;                        
 
  uint16_t SSP_FRF;                        
 
  uint16_t SSP_HardwareFlowControl;        
 
}SSP_InitTypeDef;

   




 







#line 88 "..\\..\\..\\..\\Libraries\\MDR32F9Qx_StdPeriph_Driver\\inc\\MDR32F9Qx_ssp.h"











 







   



 

#line 126 "..\\..\\..\\..\\Libraries\\MDR32F9Qx_StdPeriph_Driver\\inc\\MDR32F9Qx_ssp.h"

#line 140 "..\\..\\..\\..\\Libraries\\MDR32F9Qx_StdPeriph_Driver\\inc\\MDR32F9Qx_ssp.h"

   



 







   



 







   



 









   



 











   



 













   



 
















 




   



 







   



 

#line 265 "..\\..\\..\\..\\Libraries\\MDR32F9Qx_StdPeriph_Driver\\inc\\MDR32F9Qx_ssp.h"



   

   



 

   



 

void SSP_DeInit(MDR_SSP_TypeDef* SSPx);
void SSP_Init(MDR_SSP_TypeDef* SSPx, const SSP_InitTypeDef* SSP_InitStruct);
void SSP_StructInit(SSP_InitTypeDef* SSP_InitStruct);
void SSP_Cmd(MDR_SSP_TypeDef* SSPx, FunctionalState NewState);

void SSP_ITConfig(MDR_SSP_TypeDef* SSPx, uint32_t SSP_IT, FunctionalState NewState);
ITStatus SSP_GetITStatus(MDR_SSP_TypeDef* SSPx, uint32_t SSP_IT);
ITStatus SSP_GetITStatusMasked(MDR_SSP_TypeDef* SSPx, uint32_t SSP_IT);
void SSP_ClearITPendingBit(MDR_SSP_TypeDef* SSPx, uint32_t SSP_IT);

void SSP_DMACmd(MDR_SSP_TypeDef* SSPx, uint32_t SSP_DMAReq, FunctionalState NewState);

void SSP_SendData(MDR_SSP_TypeDef* SSPx, uint16_t Data);
uint16_t SSP_ReceiveData(MDR_SSP_TypeDef* SSPx);

FlagStatus SSP_GetFlagStatus(MDR_SSP_TypeDef* SSPx, uint32_t SSP_FLAG);
void SSP_BRGInit(MDR_SSP_TypeDef* SSPx, uint32_t SSP_BRG);

   

   

   





 

#line 26 "..\\..\\..\\..\\Libraries\\MDR32F9Qx_StdPeriph_Driver\\src\\MDR32F9Qx_ssp.c"
#line 1 "..\\..\\..\\..\\Libraries\\MDR32F9Qx_StdPeriph_Driver\\inc\\MDR32F9Qx_rst_clk.h"





















 

 



 
#line 30 "..\\..\\..\\..\\Libraries\\MDR32F9Qx_StdPeriph_Driver\\inc\\MDR32F9Qx_rst_clk.h"



 



 



 



 

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

   



 



 



 









   


#line 109 "..\\..\\..\\..\\Libraries\\MDR32F9Qx_StdPeriph_Driver\\inc\\MDR32F9Qx_rst_clk.h"



 



 









   



 



 











   



 



 

#line 172 "..\\..\\..\\..\\Libraries\\MDR32F9Qx_StdPeriph_Driver\\inc\\MDR32F9Qx_rst_clk.h"



   




 



 











   



 



 

#line 222 "..\\..\\..\\..\\Libraries\\MDR32F9Qx_StdPeriph_Driver\\inc\\MDR32F9Qx_rst_clk.h"



   




 



 

#line 245 "..\\..\\..\\..\\Libraries\\MDR32F9Qx_StdPeriph_Driver\\inc\\MDR32F9Qx_rst_clk.h"

#line 255 "..\\..\\..\\..\\Libraries\\MDR32F9Qx_StdPeriph_Driver\\inc\\MDR32F9Qx_rst_clk.h"

   



 



 











   



 



 
#line 292 "..\\..\\..\\..\\Libraries\\MDR32F9Qx_StdPeriph_Driver\\inc\\MDR32F9Qx_rst_clk.h"

#line 300 "..\\..\\..\\..\\Libraries\\MDR32F9Qx_StdPeriph_Driver\\inc\\MDR32F9Qx_rst_clk.h"
   



 



 

#line 319 "..\\..\\..\\..\\Libraries\\MDR32F9Qx_StdPeriph_Driver\\inc\\MDR32F9Qx_rst_clk.h"

#line 329 "..\\..\\..\\..\\Libraries\\MDR32F9Qx_StdPeriph_Driver\\inc\\MDR32F9Qx_rst_clk.h"

   



 



 
#line 345 "..\\..\\..\\..\\Libraries\\MDR32F9Qx_StdPeriph_Driver\\inc\\MDR32F9Qx_rst_clk.h"

#line 352 "..\\..\\..\\..\\Libraries\\MDR32F9Qx_StdPeriph_Driver\\inc\\MDR32F9Qx_rst_clk.h"
   



 

 

 
#line 370 "..\\..\\..\\..\\Libraries\\MDR32F9Qx_StdPeriph_Driver\\inc\\MDR32F9Qx_rst_clk.h"

#line 380 "..\\..\\..\\..\\Libraries\\MDR32F9Qx_StdPeriph_Driver\\inc\\MDR32F9Qx_rst_clk.h"
   




 



 








#line 430 "..\\..\\..\\..\\Libraries\\MDR32F9Qx_StdPeriph_Driver\\inc\\MDR32F9Qx_rst_clk.h"







#line 473 "..\\..\\..\\..\\Libraries\\MDR32F9Qx_StdPeriph_Driver\\inc\\MDR32F9Qx_rst_clk.h"

#line 528 "..\\..\\..\\..\\Libraries\\MDR32F9Qx_StdPeriph_Driver\\inc\\MDR32F9Qx_rst_clk.h"

   



 



 

#line 546 "..\\..\\..\\..\\Libraries\\MDR32F9Qx_StdPeriph_Driver\\inc\\MDR32F9Qx_rst_clk.h"

#line 554 "..\\..\\..\\..\\Libraries\\MDR32F9Qx_StdPeriph_Driver\\inc\\MDR32F9Qx_rst_clk.h"

   






 



 

#line 577 "..\\..\\..\\..\\Libraries\\MDR32F9Qx_StdPeriph_Driver\\inc\\MDR32F9Qx_rst_clk.h"

#line 587 "..\\..\\..\\..\\Libraries\\MDR32F9Qx_StdPeriph_Driver\\inc\\MDR32F9Qx_rst_clk.h"

   



 



 
#line 606 "..\\..\\..\\..\\Libraries\\MDR32F9Qx_StdPeriph_Driver\\inc\\MDR32F9Qx_rst_clk.h"

#line 616 "..\\..\\..\\..\\Libraries\\MDR32F9Qx_StdPeriph_Driver\\inc\\MDR32F9Qx_rst_clk.h"

   

   



 

   



 

void RST_CLK_DeInit(void);
void RST_CLK_WarmDeInit(void);


void RST_CLK_HSEconfig(uint32_t RST_CLK_HSE);
ErrorStatus RST_CLK_HSEstatus(void);

void RST_CLK_LSEconfig(uint32_t RST_CLK_LSE);
ErrorStatus RST_CLK_LSEstatus(void);

void RST_CLK_HSIcmd(FunctionalState NewState);
void RST_CLK_HSIadjust(uint32_t HSItrimValue);
ErrorStatus RST_CLK_HSIstatus(void);

void RST_CLK_LSIcmd(FunctionalState NewState);
void RST_CLK_LSIadjust(uint32_t LSItrimValue);
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
#line 683 "..\\..\\..\\..\\Libraries\\MDR32F9Qx_StdPeriph_Driver\\inc\\MDR32F9Qx_rst_clk.h"
void RST_CLK_GetClocksFreq(RST_CLK_FreqTypeDef* RST_CLK_Clocks);

FlagStatus RST_CLK_GetFlagStatus(uint32_t RST_CLK_FLAG);






   

   

   





 

#line 27 "..\\..\\..\\..\\Libraries\\MDR32F9Qx_StdPeriph_Driver\\src\\MDR32F9Qx_ssp.c"





 



 



 




   



 






 
void SSP_DeInit(MDR_SSP_TypeDef* SSPx)
{
   
  ((void)0);

  SSPx->CR0   = 0;
  SSPx->CR1   = 0;
  SSPx->CPSR  = 0;
  SSPx->IMSC  = 0;
  SSPx->DMACR = 0;

   
  SSPx->ICR = ((uint32_t)0x00000002) | ((uint32_t)0x00000001);

}










 
void SSP_Init(MDR_SSP_TypeDef* SSPx, const SSP_InitTypeDef* SSP_InitStruct)
{
  uint32_t tmpreg;

   
  ((void)0);
  ((void)0);
  ((void)0);
  ((void)0);
  ((void)0);
  ((void)0);
  ((void)0);
  ((void)0);
  ((void)0);

   
  SSPx->CPSR = SSP_InitStruct->SSP_CPSDVSR;

   
  tmpreg = (SSP_InitStruct->SSP_SCR << 8)
         + SSP_InitStruct->SSP_SPH
         + SSP_InitStruct->SSP_SPO
         + SSP_InitStruct->SSP_FRF
         + SSP_InitStruct->SSP_WordLength;
  SSPx->CR0 = tmpreg;

   
  tmpreg = SSP_InitStruct->SSP_HardwareFlowControl + SSP_InitStruct->SSP_Mode;
  SSPx->CR1 = tmpreg;
}






 
void SSP_StructInit(SSP_InitTypeDef* SSP_InitStruct)
{
   
   
  SSP_InitStruct->SSP_Mode = ((uint32_t)0x0000);
   
  SSP_InitStruct->SSP_WordLength = ((uint16_t)0x0007);
   
  SSP_InitStruct->SSP_SPO = ((uint16_t)0x0000);
   
  SSP_InitStruct->SSP_SPH = ((uint16_t)0x0000);
   
  SSP_InitStruct->SSP_FRF = ((uint16_t)0x0000);
   
  SSP_InitStruct->SSP_CPSDVSR = 2;
  SSP_InitStruct->SSP_SCR = 0;

}









 
void SSP_Cmd(MDR_SSP_TypeDef* SSPx, FunctionalState NewState)
{
   
  ((void)0);
  ((void)0);

  if (NewState != DISABLE)
  {
     
    SSPx->CR1 |= ((uint16_t)0x0002);
  }
  else
  {
     
    SSPx->CR1 &= ((uint16_t)0xFFFD);
  }
}















 
void SSP_ITConfig(MDR_SSP_TypeDef* SSPx, uint32_t SSP_IT, FunctionalState NewState)
{
   
  ((void)0);
  ((void)0);
  ((void)0);

  if (NewState != DISABLE)
  {
    SSPx->IMSC |= SSP_IT;
  }
  else
  {
    SSPx->IMSC &= ~SSP_IT;
  }
}













 
ITStatus SSP_GetITStatus(MDR_SSP_TypeDef* SSPx, uint32_t SSP_IT)
{
  ITStatus bitstatus;

   
  ((void)0);
  ((void)0);

  if ((SSPx->RIS & SSP_IT) == SSP_IT)
  {
    bitstatus = SET;
  }
  else
  {
    bitstatus = RESET;
  }

  return (bitstatus);
}













 
ITStatus SSP_GetITStatusMasked(MDR_SSP_TypeDef* SSPx, uint32_t SSP_IT)
{
  ITStatus bitstatus;

   
  ((void)0);
  ((void)0);

  if (SSPx->MIS & SSP_IT)
  {
    bitstatus = SET;
  }
  else
  {
    bitstatus = RESET;
  }

  return (bitstatus);
}











 
void SSP_ClearITPendingBit(MDR_SSP_TypeDef* SSPx, uint32_t SSP_IT)
{
   
  ((void)0);
  ((void)0);

  SSPx->ICR |= SSP_IT;
}














 
void SSP_DMACmd(MDR_SSP_TypeDef* SSPx, uint32_t SSP_DMAReq, FunctionalState NewState)
{
   
  ((void)0);
  ((void)0);
  ((void)0);

  if (NewState != DISABLE)
  {
     
    SSPx->DMACR |= SSP_DMAReq;
  }
  else
  {
     
    SSPx->DMACR &= (uint16_t)~SSP_DMAReq;
  }
}








 
void SSP_SendData(MDR_SSP_TypeDef* SSPx, uint16_t Data)
{
   
  ((void)0);

   
  SSPx->DR = Data;

}







 
uint16_t SSP_ReceiveData(MDR_SSP_TypeDef* SSPx)
{
   
  ((void)0);

   
  return ((uint16_t)(SSPx->DR));
}













 
FlagStatus SSP_GetFlagStatus(MDR_SSP_TypeDef* SSPx, uint32_t SSP_FLAG)
{
  FlagStatus bitstatus;

   
  ((void)0);
  ((void)0);

  if (SSPx->SR & SSP_FLAG)
  {
    bitstatus = SET;
  }
  else
  {
    bitstatus = RESET;
  }

  return (bitstatus);
}


















 
void SSP_BRGInit ( MDR_SSP_TypeDef* SSPx, uint32_t SSP_BRG ) {
	uint32_t tmpreg;

	 
	((void)0);
	((void)0);

#line 415 "..\\..\\..\\..\\Libraries\\MDR32F9Qx_StdPeriph_Driver\\src\\MDR32F9Qx_ssp.c"
		tmpreg = ((MDR_RST_CLK_TypeDef *) (0x40020000))->SSP_CLOCK;



	if (SSPx == ((MDR_SSP_TypeDef *) (0x40040000))) {
		tmpreg |= ((uint32_t)0x01000000);
		tmpreg &= ~((uint32_t)0x000000FF);
		tmpreg |= SSP_BRG;
	}
	else{
		if (SSPx == ((MDR_SSP_TypeDef *) (0x400A0000))) {
			tmpreg |= ((uint32_t)0x02000000);
			tmpreg &= ~((uint32_t)0x0000FF00);
			tmpreg |= (SSP_BRG << 8);
		}
#line 447 "..\\..\\..\\..\\Libraries\\MDR32F9Qx_StdPeriph_Driver\\src\\MDR32F9Qx_ssp.c"
	}
#line 454 "..\\..\\..\\..\\Libraries\\MDR32F9Qx_StdPeriph_Driver\\src\\MDR32F9Qx_ssp.c"
		((MDR_RST_CLK_TypeDef *) (0x40020000))->SSP_CLOCK = tmpreg;

}

   

   

   



 

