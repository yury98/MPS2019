#line 1 "..\\..\\src\\Menu_time.c"

















 
 
#line 1 "..\\1986be9x_config.h"

















 

 



#line 1 "..\\..\\..\\Library\\Libraries\\1986BE9x_StdPeriph_Driver\\inc\\1986BE9x_lib.h"





















 

 



#line 48 "..\\..\\..\\Library\\Libraries\\1986BE9x_StdPeriph_Driver\\inc\\1986BE9x_lib.h"



 






 

#line 25 "..\\1986be9x_config.h"
#line 1 "..\\..\\..\\Library\\Libraries\\CMSIS\\CM3\\CoreSupport\\stdint.h"
 
 
 
 
 
 





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
















   



   








   



   



   






   


















   

   
 
   

#line 26 "..\\1986be9x_config.h"


 

 

 
 





 
#line 46 "..\\1986be9x_config.h"



 
 


 


 
            


 

 


 
 
 
 


 


 
 

 
 

 
 

 
 


 







 

 
 








 
#line 119 "..\\1986be9x_config.h"





 

#line 21 "..\\..\\src\\Menu_time.c"
#line 1 "..\\..\\..\\Library\\Libraries\\CMSIS\\CM3\\DeviceSupport\\1986BE9x\\inc\\1986be9x.h"





















 

 





 



 



 



 

typedef enum IRQn
{
 
  NonMaskableInt_IRQn         = -14,     
  HardFault_IRQn              = -13,     
  MemoryManagement_IRQn       = -12,     
  BusFault_IRQn               = -11,     
  UsageFault_IRQn             = -10,     
  SVCall_IRQn                 = -5,      
  PendSV_IRQn                 = -2,      
  SysTick_IRQn                = -1,      

 
  CAN1_IRQn                   = 0,       
  CAN2_IRQn                   = 1,       
  USB_IRQn                    = 2,       
  DMA_IRQn                    = 5,       
  UART1_IRQn                  = 6,       
  UART2_IRQn                  = 7,       
  SSP1_IRQn                   = 8,       
  I2C_IRQn                    = 10,      
  POWER_IRQn                  = 11,      
  WWDG_IRQn                   = 12,      
  Timer1_IRQn                 = 14,      
  Timer2_IRQn                 = 15,      
  Timer3_IRQn                 = 16,      
  ADC_IRQn                    = 17,      
  COMPARATOR_IRQn             = 19,      
  SSP2_IRQn                   = 20,      
  BACKUP_IRQn                 = 27,      
  EXT_INT1_IRQn               = 28,      
  EXT_INT2_IRQn               = 29,      
  EXT_INT3_IRQn               = 30,      
  EXT_INT4_IRQn               = 31       
}IRQn_Type;

 
#line 1 "..\\..\\..\\Library\\Libraries\\CMSIS\\CM3\\CoreSupport\\core_cm3.h"
 




















 





































 

 
 
 
 
 
 
 
 








 











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






 
#line 91 "..\\..\\..\\Library\\Libraries\\CMSIS\\CM3\\CoreSupport\\core_cm3.h"

















 

#line 117 "..\\..\\..\\Library\\Libraries\\CMSIS\\CM3\\CoreSupport\\core_cm3.h"





 


 





 
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

 



 








   


#line 614 "..\\..\\..\\Library\\Libraries\\CMSIS\\CM3\\CoreSupport\\core_cm3.h"





 
typedef struct
{
  volatile uint32_t DHCSR;                         
  volatile  uint32_t DCRSR;                         
  volatile uint32_t DCRDR;                         
  volatile uint32_t DEMCR;                         
} CoreDebug_Type;

 




































 






 






































   


 
#line 721 "..\\..\\..\\Library\\Libraries\\CMSIS\\CM3\\CoreSupport\\core_cm3.h"

#line 728 "..\\..\\..\\Library\\Libraries\\CMSIS\\CM3\\CoreSupport\\core_cm3.h"






   




 





#line 762 "..\\..\\..\\Library\\Libraries\\CMSIS\\CM3\\CoreSupport\\core_cm3.h"


 


 




#line 787 "..\\..\\..\\Library\\Libraries\\CMSIS\\CM3\\CoreSupport\\core_cm3.h"


 
 
 
 








 
extern uint32_t __get_PSP(void);








 
extern void __set_PSP(uint32_t topOfProcStack);








 
extern uint32_t __get_MSP(void);








 
extern void __set_MSP(uint32_t topOfMainStack);








 
extern uint32_t __REV16(uint16_t value);








 
extern int32_t __REVSH(int16_t value);


#line 937 "..\\..\\..\\Library\\Libraries\\CMSIS\\CM3\\CoreSupport\\core_cm3.h"





 








 
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





#line 1460 "..\\..\\..\\Library\\Libraries\\CMSIS\\CM3\\CoreSupport\\core_cm3.h"







 
 


 











 
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


   






   



 

#line 82 "..\\..\\..\\Library\\Libraries\\CMSIS\\CM3\\DeviceSupport\\1986BE9x\\inc\\1986be9x.h"

#line 1 "..\\..\\..\\Library\\Libraries\\CMSIS\\CM3\\DeviceSupport\\1986BE9x\\inc\\1986BE9x_can_defs.h"






















 

 





 



 



 



 

typedef struct
{
  volatile uint32_t ID;
  volatile uint32_t DLC;
  volatile uint32_t DATAL;
  volatile uint32_t DATAH;
}CAN_BUF_TypeDef;



 

typedef struct
{
  volatile uint32_t MASK;
  volatile uint32_t FILTER;
}CAN_BUF_FILTER_TypeDef;



 



 

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
    CAN_BUF_TypeDef CAN_BUF[32];
       uint32_t RESERVED3[64];
    CAN_BUF_FILTER_TypeDef CAN_BUF_FILTER[32];
}CAN_TypeDef;

   

   



 



 

















   

   



 



 












































   

   



 



 




















   

   



 



 

















   

   



 



 








   

   



 



 




















   

   



 



 














   

   



 



 














   

   



 



 








   

   



 



 




















   

   



 



 














   

   



 



 














   

   



 



 


























   

   



 



 








   

   



 



 




















   

   



 



 














   

   



 



 














   

   

   

   

   





 
#line 84 "..\\..\\..\\Library\\Libraries\\CMSIS\\CM3\\DeviceSupport\\1986BE9x\\inc\\1986be9x.h"
#line 1 "..\\..\\..\\Library\\Libraries\\CMSIS\\CM3\\DeviceSupport\\1986BE9x\\inc\\1986BE9x_usb_defs.h"






















 

 





 



 



 



 

typedef struct
{
  volatile uint32_t CTRL;
  volatile uint32_t STS;
  volatile uint32_t TS;
  volatile uint32_t NTS;
}USB_SEP_TypeDef;



 

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
       uint32_t RESERVED6[11];
}USB_SEP_FIFO_TypeDef;



 



 

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
    USB_SEP_TypeDef USB_SEP[4];
  volatile uint32_t SC;
  volatile uint32_t SLS;
  volatile uint32_t SIS;
  volatile uint32_t SIM;
  volatile uint32_t SA;
  volatile uint32_t SFN_L;
  volatile uint32_t SFN_H;
       uint32_t RESERVED5[9];
    USB_SEP_FIFO_TypeDef USB_SEP_FIFO[4];
  volatile uint32_t HSCR;
  volatile uint32_t HSVR;
}USB_TypeDef;

   

   



 



 














   

   



 



 














   

   



 



 














   

   



 



 














   

   



 



 


























   

   



 



 

















   

   



 



 


























   

   



 



 

















   

   



 



 

















   

   



 



 

















   

   



 



 


























   

   



 



 








   

   

   

   

   





 

#line 85 "..\\..\\..\\Library\\Libraries\\CMSIS\\CM3\\DeviceSupport\\1986BE9x\\inc\\1986be9x.h"
#line 1 "..\\..\\..\\Library\\Libraries\\CMSIS\\CM3\\DeviceSupport\\1986BE9x\\inc\\1986BE9x_eeprom_defs.h"






















 

 





 



 



 



 



 

typedef struct
{
  volatile uint32_t CMD;
  volatile uint32_t ADR;
  volatile uint32_t DI;
  volatile uint32_t DO;
  volatile uint32_t KEY;
}EEPROM_TypeDef;

   

   



 



 






































   

   

   

   

   





 
#line 86 "..\\..\\..\\Library\\Libraries\\CMSIS\\CM3\\DeviceSupport\\1986BE9x\\inc\\1986be9x.h"
#line 1 "..\\..\\..\\Library\\Libraries\\CMSIS\\CM3\\DeviceSupport\\1986BE9x\\inc\\1986BE9x_rst_clk_defs.h"






















 

 





 



 



 



 



 

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
}RST_CLK_TypeDef;

   

   



 



 











   

   



 



 




















   

   



 



 








   

   



 



 














   

   



 



 














   

   



 



 

















   

   



 



 














   

   



 



 














   

   



 



 




















   

   



 



 














   

   



 



 














   

   

   

   

   





 
#line 87 "..\\..\\..\\Library\\Libraries\\CMSIS\\CM3\\DeviceSupport\\1986BE9x\\inc\\1986be9x.h"
#line 1 "..\\..\\..\\Library\\Libraries\\CMSIS\\CM3\\DeviceSupport\\1986BE9x\\inc\\1986BE9x_dma_defs.h"






















 

 





 



 



 



 



 

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
}DMA_TypeDef;

   

   



 



 














   

   



 



 








   

   

   

   

   





 
#line 88 "..\\..\\..\\Library\\Libraries\\CMSIS\\CM3\\DeviceSupport\\1986BE9x\\inc\\1986be9x.h"
#line 1 "..\\..\\..\\Library\\Libraries\\CMSIS\\CM3\\DeviceSupport\\1986BE9x\\inc\\1986BE9x_uart_defs.h"






















 

 





 



 



 



 



 

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
}UART_TypeDef;

   

   



 



 

















   

   



 



 














   

   



 



 





























   

   



 



 























   

   



 



 






































   

   



 



 








   

   



 



 



































   

   



 



 



































   

   



 



 



































   

   



 



 



































   

   



 



 











   

   

   

   

   





 
#line 89 "..\\..\\..\\Library\\Libraries\\CMSIS\\CM3\\DeviceSupport\\1986BE9x\\inc\\1986be9x.h"
#line 1 "..\\..\\..\\Library\\Libraries\\CMSIS\\CM3\\DeviceSupport\\1986BE9x\\inc\\1986BE9x_ssp_defs.h"






















 

 





 



 



 



 



 

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
}SSP_TypeDef;

   

   



 



 

















   

   



 



 














   

   



 



 

















   

   



 



 














   

   



 



 














   

   



 



 














   

   



 



 








   

   



 



 








   

   

   

   

   





 
#line 90 "..\\..\\..\\Library\\Libraries\\CMSIS\\CM3\\DeviceSupport\\1986BE9x\\inc\\1986be9x.h"
#line 1 "..\\..\\..\\Library\\Libraries\\CMSIS\\CM3\\DeviceSupport\\1986BE9x\\inc\\1986BE9x_i2c_defs.h"






















 

 





 



 



 



 



 

typedef struct
{
  volatile uint32_t PRL;
  volatile uint32_t PRH;
  volatile uint32_t CTR;
  volatile uint32_t RXD;
  volatile uint32_t STA;
  volatile uint32_t TXD;
  volatile uint32_t CMD;
}I2C_TypeDef;

   

   



 



 











   

   



 



 

















   

   



 



 




















   

   

   

   

   





 
#line 91 "..\\..\\..\\Library\\Libraries\\CMSIS\\CM3\\DeviceSupport\\1986BE9x\\inc\\1986be9x.h"
#line 1 "..\\..\\..\\Library\\Libraries\\CMSIS\\CM3\\DeviceSupport\\1986BE9x\\inc\\1986BE9x_power_defs.h"






















 

 





 



 



 



 



 

typedef struct
{
  volatile uint32_t PVDCS;
}POWER_TypeDef;

   

   



 



 





























   

   

   

   

   





 
#line 92 "..\\..\\..\\Library\\Libraries\\CMSIS\\CM3\\DeviceSupport\\1986BE9x\\inc\\1986be9x.h"
#line 1 "..\\..\\..\\Library\\Libraries\\CMSIS\\CM3\\DeviceSupport\\1986BE9x\\inc\\1986BE9x_wwdg_defs.h"






















 

 





 



 



 



 



 

typedef struct
{
  volatile uint32_t CR;
  volatile uint32_t CFR;
  volatile uint32_t SR;
}WWDG_TypeDef;

   

   



 



 








   

   



 



 











   

   

   

   

   





 
#line 93 "..\\..\\..\\Library\\Libraries\\CMSIS\\CM3\\DeviceSupport\\1986BE9x\\inc\\1986be9x.h"
#line 1 "..\\..\\..\\Library\\Libraries\\CMSIS\\CM3\\DeviceSupport\\1986BE9x\\inc\\1986BE9x_iwdg_defs.h"






















 

 





 



 



 



 



 

typedef struct
{
  volatile uint32_t KR;
  volatile uint32_t PR;
  volatile uint32_t RLR;
  volatile uint32_t SR;
}IWDG_TypeDef;

   

   



 



 








   

   

   

   

   





 
#line 94 "..\\..\\..\\Library\\Libraries\\CMSIS\\CM3\\DeviceSupport\\1986BE9x\\inc\\1986be9x.h"
#line 1 "..\\..\\..\\Library\\Libraries\\CMSIS\\CM3\\DeviceSupport\\1986BE9x\\inc\\1986BE9x_timer_defs.h"






















 

 





 



 



 



 



 

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
}TIMER_TypeDef;

   

   



 



 























   

   



 



 





























   

   



 



 




















   

   



 



 











   

   



 



 














   

   



 



 


























   

   



 



 


























   

   



 



 


























   

   



 



 











   

   

   

   

   





 
#line 95 "..\\..\\..\\Library\\Libraries\\CMSIS\\CM3\\DeviceSupport\\1986BE9x\\inc\\1986be9x.h"
#line 1 "..\\..\\..\\Library\\Libraries\\CMSIS\\CM3\\DeviceSupport\\1986BE9x\\inc\\1986BE9x_adc_defs.h"






















 

 





 



 



 



 



 

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
}ADC_TypeDef;

   

   



 



 





















































   

   



 



 






































   

   



 



 








   

   



 



 








   

   



 



 

















   

   



 



 

















   

   

   

   

   





 
#line 96 "..\\..\\..\\Library\\Libraries\\CMSIS\\CM3\\DeviceSupport\\1986BE9x\\inc\\1986be9x.h"
#line 1 "..\\..\\..\\Library\\Libraries\\CMSIS\\CM3\\DeviceSupport\\1986BE9x\\inc\\1986BE9x_dac_defs.h"






















 

 





 



 



 



 



 

typedef struct
{
  volatile uint32_t CFG;
  volatile uint32_t DAC1_DATA;
  volatile uint32_t DAC2_DATA;
}DAC_TypeDef;

   

   



 



 

















   

   



 



 








   

   



 



 








   

   

   

   

   





 
#line 97 "..\\..\\..\\Library\\Libraries\\CMSIS\\CM3\\DeviceSupport\\1986BE9x\\inc\\1986be9x.h"
#line 1 "..\\..\\..\\Library\\Libraries\\CMSIS\\CM3\\DeviceSupport\\1986BE9x\\inc\\1986BE9x_comp_defs.h"






















 

 





 



 



 



 



 

typedef struct
{
  volatile uint32_t CFG;
  volatile uint32_t RESULT;
  volatile uint32_t RESULT_LATCH;
}COMP_TypeDef;

   

   



 



 
































   

   



 



 











   

   

   

   

   





 
#line 98 "..\\..\\..\\Library\\Libraries\\CMSIS\\CM3\\DeviceSupport\\1986BE9x\\inc\\1986be9x.h"
#line 1 "..\\..\\..\\Library\\Libraries\\CMSIS\\CM3\\DeviceSupport\\1986BE9x\\inc\\1986BE9x_port_defs.h"






















 

 





 



 



 



 



 

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
}PORT_TypeDef;

   

   



 



 


















































   

   



 



 


















































   

   

   

   

   





 
#line 99 "..\\..\\..\\Library\\Libraries\\CMSIS\\CM3\\DeviceSupport\\1986BE9x\\inc\\1986be9x.h"
#line 1 "..\\..\\..\\Library\\Libraries\\CMSIS\\CM3\\DeviceSupport\\1986BE9x\\inc\\1986BE9x_bkp_defs.h"






















 

 





 



 



 



 



 

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
}BKP_TypeDef;

   

   



 



 























   

   



 



 















































   

   



 



 























   

   

   

   

   





 
#line 100 "..\\..\\..\\Library\\Libraries\\CMSIS\\CM3\\DeviceSupport\\1986BE9x\\inc\\1986be9x.h"
#line 1 "..\\..\\..\\Library\\Libraries\\CMSIS\\CM3\\DeviceSupport\\1986BE9x\\inc\\1986BE9x_ext_bus_defs.h"






















 

 





 



 



 



 



 

typedef struct
{
       uint32_t RESERVED0[20];
  volatile uint32_t NAND_CYCLES;
  volatile uint32_t CONTROL;
}EXT_BUS_TypeDef;

   

   



 



 























   

   



 



 




















   

   

   

   

   





 
#line 101 "..\\..\\..\\Library\\Libraries\\CMSIS\\CM3\\DeviceSupport\\1986BE9x\\inc\\1986be9x.h"

typedef enum {RESET = 0, SET = !RESET} FlagStatus, ITStatus;

typedef enum {DISABLE = 0, ENABLE = !DISABLE} FunctionalState;


typedef enum {ERROR = 0, SUCCESS = !ERROR} ErrorStatus;

   




 

#line 144 "..\\..\\..\\Library\\Libraries\\CMSIS\\CM3\\DeviceSupport\\1986BE9x\\inc\\1986be9x.h"

   



 

#line 179 "..\\..\\..\\Library\\Libraries\\CMSIS\\CM3\\DeviceSupport\\1986BE9x\\inc\\1986be9x.h"

   

   

   





 
#line 22 "..\\..\\src\\Menu_time.c"
#line 1 "..\\..\\..\\Library\\Libraries\\1986BE9x_StdPeriph_Driver\\inc\\1986be9x_port.h"





















 

 



 
#line 30 "..\\..\\..\\Library\\Libraries\\1986BE9x_StdPeriph_Driver\\inc\\1986be9x_port.h"
#line 31 "..\\..\\..\\Library\\Libraries\\1986BE9x_StdPeriph_Driver\\inc\\1986be9x_port.h"
#line 32 "..\\..\\..\\Library\\Libraries\\1986BE9x_StdPeriph_Driver\\inc\\1986be9x_port.h"



 



 



 

#line 51 "..\\..\\..\\Library\\Libraries\\1986BE9x_StdPeriph_Driver\\inc\\1986be9x_port.h"



 

typedef enum
{
  PORT_OE_IN            = 0x0,
  PORT_OE_OUT           = 0x1
}PORT_OE_TypeDef;






 

typedef enum
{
  PORT_MODE_ANALOG      = 0x0,
  PORT_MODE_DIGITAL     = 0x1
}PORT_MODE_TypeDef;






 

typedef enum
{
  PORT_PULL_UP_OFF      = 0x0,
  PORT_PULL_UP_ON       = 0x1
}PORT_PULL_UP_TypeDef;






 

typedef enum
{
  PORT_PULL_DOWN_OFF    = 0x0,
  PORT_PULL_DOWN_ON     = 0x1
}PORT_PULL_DOWN_TypeDef;






 

typedef enum
{
  PORT_PD_SHM_OFF       = 0x0,
  PORT_PD_SHM_ON        = 0x1
}PORT_PD_SHM_TypeDef;







 

typedef enum
{
  PORT_PD_DRIVER        = 0x0,
  PORT_PD_OPEN          = 0x1
}PORT_PD_TypeDef;





 

typedef enum
{
  PORT_GFEN_OFF         = 0x0,
  PORT_GFEN_ON          = 0x1
}PORT_GFEN_TypeDef;






 

typedef enum
{
  PORT_FUNC_PORT        = 0x0,
  PORT_FUNC_MAIN        = 0x1,
  PORT_FUNC_ALTER       = 0x2,
  PORT_FUNC_OVERRID     = 0x3
}PORT_FUNC_TypeDef;






 

typedef enum
{
  PORT_OUTPUT_OFF       = 0x0,
  PORT_SPEED_SLOW       = 0x1,
  PORT_SPEED_FAST       = 0x2,
  PORT_SPEED_MAXFAST    = 0x3
}PORT_SPEED_TypeDef;







 

typedef struct
{
  uint16_t PORT_Pin;                     
 
  PORT_OE_TypeDef PORT_OE;               
 
  PORT_PULL_UP_TypeDef PORT_PULL_UP;     
 
  PORT_PULL_DOWN_TypeDef PORT_PULL_DOWN; 
 
  PORT_PD_SHM_TypeDef PORT_PD_SHM;       
 
  PORT_PD_TypeDef PORT_PD;               
 
  PORT_GFEN_TypeDef PORT_GFEN;           
 
  PORT_FUNC_TypeDef PORT_FUNC;           
 
  PORT_SPEED_TypeDef PORT_SPEED;         
 
  PORT_MODE_TypeDef PORT_MODE;           
 
}PORT_InitTypeDef;




 

typedef enum
{
  Bit_RESET = 0,
  Bit_SET
}BitAction;



   



 



 






















   



 

#line 269 "..\\..\\..\\Library\\Libraries\\1986BE9x_StdPeriph_Driver\\inc\\1986be9x_port.h"




#line 289 "..\\..\\..\\Library\\Libraries\\1986BE9x_StdPeriph_Driver\\inc\\1986be9x_port.h"

   

   



 

   



 

void PORT_DeInit(PORT_TypeDef* PORTx);
void PORT_Init(PORT_TypeDef* PORTx, const PORT_InitTypeDef* PORT_InitStruct);
void PORT_StructInit(PORT_InitTypeDef* PORT_InitStruct);

uint8_t PORT_ReadInputDataBit(PORT_TypeDef* PORTx, uint32_t PORT_Pin);
uint32_t PORT_ReadInputData(PORT_TypeDef* PORTx);

void PORT_SetBits(PORT_TypeDef* PORTx, uint32_t PORT_Pin);
void PORT_ResetBits(PORT_TypeDef* PORTx, uint32_t PORT_Pin);
void PORT_WriteBit(PORT_TypeDef* PORTx, uint32_t PORT_Pin, BitAction BitVal);
void PORT_Write(PORT_TypeDef* PORTx, uint32_t PortVal);

   

   

   





 

#line 23 "..\\..\\src\\Menu_time.c"
#line 1 "..\\..\\..\\Library\\Libraries\\1986BE9x_StdPeriph_Driver\\inc\\1986be9x_bkp.h"





















 

 



 
#line 30 "..\\..\\..\\Library\\Libraries\\1986BE9x_StdPeriph_Driver\\inc\\1986be9x_bkp.h"
#line 31 "..\\..\\..\\Library\\Libraries\\1986BE9x_StdPeriph_Driver\\inc\\1986be9x_bkp.h"


 




 



 



 

   



 

   



 



 

#line 80 "..\\..\\..\\Library\\Libraries\\1986BE9x_StdPeriph_Driver\\inc\\1986be9x_bkp.h"

#line 90 "..\\..\\..\\Library\\Libraries\\1986BE9x_StdPeriph_Driver\\inc\\1986be9x_bkp.h"

#line 99 "..\\..\\..\\Library\\Libraries\\1986BE9x_StdPeriph_Driver\\inc\\1986be9x_bkp.h"

   



 











   



 







   



 



   



 



   



 











   


   



 
void BKP_DeInit(void);
void BKP_JTAGA_Enable(FunctionalState NewState);
void BKP_JTAGB_Enable(FunctionalState NewState);
void BKP_RTCclkSource(uint32_t RTC_CLK);
void BKP_RTC_Enable(FunctionalState NewState);
void BKP_RTC_Calibration(uint32_t RTC_Calibration);
void BKP_RTC_Reset(FunctionalState NewState);
void BKP_RTC_ITConfig(uint32_t RTC_IT, FunctionalState NewState);
uint32_t  BKP_RTC_GetCounter(void);
void BKP_RTC_SetCounter(uint32_t CounterValue);
void BKP_RTC_SetAlarm(uint32_t AlarmValue);
uint32_t  BKP_RTC_GetDivider(void);
void BKP_RTC_SetPrescaler(uint32_t PrescalerValue);
void BKP_RTC_WaitForUpdate(void);
FlagStatus BKP_RTC_GetFlagStatus(uint32_t RTC_FLAG);

   

   

   





 

#line 24 "..\\..\\src\\Menu_time.c"
#line 1 "..\\..\\..\\Library\\Libraries\\1986BE9x_StdPeriph_Driver\\inc\\1986be9x_rst_clk.h"





















 

 



 
#line 30 "..\\..\\..\\Library\\Libraries\\1986BE9x_StdPeriph_Driver\\inc\\1986be9x_rst_clk.h"
#line 31 "..\\..\\..\\Library\\Libraries\\1986BE9x_StdPeriph_Driver\\inc\\1986be9x_rst_clk.h"



 



 



 



 

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

   



 



 



 









   



 



 









   



 



 











   



 



 

#line 151 "..\\..\\..\\Library\\Libraries\\1986BE9x_StdPeriph_Driver\\inc\\1986be9x_rst_clk.h"



   




 



 











   



 



 

#line 201 "..\\..\\..\\Library\\Libraries\\1986BE9x_StdPeriph_Driver\\inc\\1986be9x_rst_clk.h"



   




 



 

#line 224 "..\\..\\..\\Library\\Libraries\\1986BE9x_StdPeriph_Driver\\inc\\1986be9x_rst_clk.h"

#line 234 "..\\..\\..\\Library\\Libraries\\1986BE9x_StdPeriph_Driver\\inc\\1986be9x_rst_clk.h"

   



 



 











   



 



 
#line 271 "..\\..\\..\\Library\\Libraries\\1986BE9x_StdPeriph_Driver\\inc\\1986be9x_rst_clk.h"

#line 279 "..\\..\\..\\Library\\Libraries\\1986BE9x_StdPeriph_Driver\\inc\\1986be9x_rst_clk.h"
   



 



 

#line 298 "..\\..\\..\\Library\\Libraries\\1986BE9x_StdPeriph_Driver\\inc\\1986be9x_rst_clk.h"

#line 308 "..\\..\\..\\Library\\Libraries\\1986BE9x_StdPeriph_Driver\\inc\\1986be9x_rst_clk.h"

   




 



 



#line 354 "..\\..\\..\\Library\\Libraries\\1986BE9x_StdPeriph_Driver\\inc\\1986be9x_rst_clk.h"






   



 



 

#line 376 "..\\..\\..\\Library\\Libraries\\1986BE9x_StdPeriph_Driver\\inc\\1986be9x_rst_clk.h"

#line 383 "..\\..\\..\\Library\\Libraries\\1986BE9x_StdPeriph_Driver\\inc\\1986be9x_rst_clk.h"

   






 



 

#line 406 "..\\..\\..\\Library\\Libraries\\1986BE9x_StdPeriph_Driver\\inc\\1986be9x_rst_clk.h"

#line 416 "..\\..\\..\\Library\\Libraries\\1986BE9x_StdPeriph_Driver\\inc\\1986be9x_rst_clk.h"

   



 



 
#line 435 "..\\..\\..\\Library\\Libraries\\1986BE9x_StdPeriph_Driver\\inc\\1986be9x_rst_clk.h"

#line 445 "..\\..\\..\\Library\\Libraries\\1986BE9x_StdPeriph_Driver\\inc\\1986be9x_rst_clk.h"

   

   



 

   



 

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

void RST_CLK_GetClocksFreq(RST_CLK_FreqTypeDef* RST_CLK_Clocks);

FlagStatus RST_CLK_GetFlagStatus(uint32_t RST_CLK_FLAG);

   

   

   





 

#line 25 "..\\..\\src\\Menu_time.c"
#line 1 "..\\..\\Inc\\systick.h"


















 

 



 
#line 27 "..\\..\\Inc\\systick.h"
#line 1 "..\\..\\Inc\\types.h"

















 


 



 
#line 27 "..\\..\\Inc\\types.h"

typedef unsigned char const    ucint8_t;
typedef volatile unsigned char vuint8_t;
typedef volatile unsigned long vuint32_t;

typedef enum {FALSE = 0, TRUE = !FALSE} bool;





 

#line 28 "..\\..\\Inc\\systick.h"



 



 



 



 

 
void SysTickStart(uint32_t ticks);
void SysTickStop(void);

 
void SysTickDelay(uint32_t val);

   

   

   

   





 

#line 26 "..\\..\\src\\Menu_time.c"
#line 1 "..\\..\\Inc\\Menu.h"


















 

 



 
#line 27 "..\\..\\Inc\\Menu.h"



 



 



 



   



 



   



 

void Menu_Init(void);
void DisplayMenuTitle(ucint8_t *ptr);
void DisplayMenu(void);
 
void BackToMenuOnSel(void);
 
void ReadKey(void);

   

   

   





 

#line 27 "..\\..\\src\\Menu_time.c"
#line 1 "..\\..\\Inc\\Menu_items.h"

















 

 





 



 



 



 

void ElementsFunc(void);
void IndicatorsFunc(void);

   



 

void LightsOnFunc(void);

   



 

void FontFunc(void);
void StyleFunc(void);
void ShiftFunc(void);
void BookFunc(void);
void AboutFunc(void);

   



 

void UARTwFIFOFunc(void);
void UARTFunc(void);

   



 

void TSENSORFunc(void);

   



 

void TimeAdjustFunc(void);
void TimeShowFunc(void);
void DateAdjustFunc(void);
void DateShowFunc(void);
void AlarmAdjustFunc(void);
void AlarmShowFunc(void);

   



 

void VCOMFunc(void);

   



 

void STANDBYMode_WAKEUP(void);
void STANDBYMode_RTCAlarm(void);
void STOPMode_RTCAlarm(void);

   

   

   

   





 

#line 28 "..\\..\\src\\Menu_time.c"
#line 1 "..\\..\\Inc\\leds.h"


















 

 



 
#line 27 "..\\..\\Inc\\leds.h"
#line 28 "..\\..\\Inc\\leds.h"
#line 29 "..\\..\\Inc\\leds.h"
#line 1 "..\\1986be9x_board.h"

















 

 




 




 



 

 

 

 







 

#line 30 "..\\..\\Inc\\leds.h"



 



 



 

#line 50 "..\\..\\Inc\\leds.h"

#line 70 "..\\..\\Inc\\leds.h"

   



 
 


   



 

extern uint32_t CurrentLights;           

   



 

void ShiftLights(void);

   

   

   





 

#line 29 "..\\..\\src\\Menu_time.c"
#line 1 "..\\..\\Inc\\lcd.h"

















 




 
#line 25 "..\\..\\Inc\\lcd.h"
#line 26 "..\\..\\Inc\\lcd.h"
#line 27 "..\\..\\Inc\\lcd.h"
#line 28 "..\\..\\Inc\\lcd.h"
#line 29 "..\\..\\Inc\\lcd.h"



 



 



 

 
typedef enum
{
    LCD_CRYSTAL1    = 0,
    LCD_CRYSTAL2    = 1,
    NUM_LCD_CRYSTALS
}LCD_Crystal;

 
typedef enum
{
    MET_OR          = 0,
    MET_XOR         = 1,
    MET_NOT_OR      = 2,
    MET_NOT_XOR     = 3,
    MET_AND         = 4,
    NUM_LCD_METHOD
}LCD_Method;

   



 

 









 






 



   



 

 






   



 

 
extern LCD_Crystal CurrentCrystal;   
extern LCD_Method CurrentMethod;     

   

 
#line 1 "..\\..\\Inc\\lcd_1986BE91.h"



















 



 



 
#line 30 "..\\..\\Inc\\lcd_1986BE91.h"
#line 31 "..\\..\\Inc\\lcd_1986BE91.h"
#line 32 "..\\..\\Inc\\lcd_1986BE91.h"
#line 33 "..\\..\\Inc\\lcd_1986BE91.h"
#line 1 "..\\..\\Inc\\lcd.h"

















 

#line 141 "..\\..\\Inc\\lcd.h"



 

#line 34 "..\\..\\Inc\\lcd_1986BE91.h"



 



 



 



 

 
typedef struct
{
    uint32_t Data;
    uint32_t Cmd;
}LCD_Ports;

   



 

extern const LCD_Ports CrystalPorts[NUM_LCD_CRYSTALS];

   



 




   

   

   

   







 

#line 116 "..\\..\\Inc\\lcd.h"







 

void ResetLCD(void);
void SetCrystal(LCD_Crystal num);
void WriteLCD_Cmd(uint32_t val);
void WriteLCD_Data(uint32_t val);
uint32_t ReadLCD_Cmd(void);
uint32_t ReadLCD_Data(void);
void LCD_INIT(void);
void LCD_CLS(void);

   

   

   





 

#line 30 "..\\..\\src\\Menu_time.c"
#line 1 "..\\..\\Inc\\text.h"


















 

 



 
#line 1 "..\\..\\Inc\\font_defs.h"

















 

 



 
#line 26 "..\\..\\Inc\\font_defs.h"



 



 



 



 
typedef struct
{
  uint32_t  Height;         
  uint32_t  Width;          
  uint32_t  Count;          
  ucint8_t  *pData;         
}sFONT;

   



 

extern sFONT Font_6x8;            
extern sFONT Font_7x10_thin;      
extern sFONT Font_7x10_bold;      
extern sFONT Font_12x16;          

   

   

   





 
#line 27 "..\\..\\Inc\\text.h"



 



 



 




 
#line 50 "..\\..\\Inc\\text.h"

   



 

 
typedef enum
{
  StyleSimple,
  StyleBlink,
  StyleFlipFlop,
  StyleVibratory
}TextStyle;

   



 



 



   



 

 
extern sFONT *CurrentFont;

   



 

 
void LCD_PUT_BYTE(uint8_t x, uint8_t y, uint8_t data);
 
void LCD_PUTC(uint8_t x, uint8_t y, uint8_t ch);
void LCD_PUTS(uint8_t x, uint8_t y, ucint8_t* str);
void LCD_PUTS_Ex(uint8_t x, uint8_t y, ucint8_t* str, uint8_t style);

   

   

   





 

#line 31 "..\\..\\src\\Menu_time.c"
#line 1 "..\\..\\Inc\\joystick.h"

















 

 



 
#line 26 "..\\..\\Inc\\joystick.h"
#line 27 "..\\..\\Inc\\joystick.h"
#line 28 "..\\..\\Inc\\joystick.h"



 



 



 

 
typedef enum
{
  SEL        = 0,
  UP         = 1,
  DOWN       = 2,
  LEFT       = 3,
  RIGHT      = 4,
  NOKEY      = 5,
  MULTIPLE   = 6,
  NUM_KEY_CODES
}KeyCode;

   



 

 

#line 73 "..\\..\\Inc\\joystick.h"

#line 111 "..\\..\\Inc\\joystick.h"

   



 






   



 

KeyCode GetKey(void);

   

   

   





 

#line 32 "..\\..\\src\\Menu_time.c"
#line 1 "..\\..\\Inc\\time.h"


















 

 



 



 



 



 



 



   



 

void Date_Update(void);
void Calendar_Init(void);

   

   

   

   





 

#line 33 "..\\..\\src\\Menu_time.c"
#line 1 "..\\..\\Inc\\1986BE9x_it.h"

















 

 



 
#line 26 "..\\..\\Inc\\1986BE9x_it.h"
#line 27 "..\\..\\Inc\\1986BE9x_it.h"



 



 



 

typedef void (* tUARTFunc)(void);
typedef void (* tUARTLineStateFunc)(uint32_t);

   



 

 



   




 

 
extern vuint32_t TimerCounter;

 
extern vuint32_t ADC1_Value;

 
extern tUARTFunc pfUARTSenderFunc;
extern tUARTFunc pfUARTReceiverFunc;
extern tUARTLineStateFunc pfUARTLineStateFunc;

 
extern vuint32_t STOPModeStatus;

 
extern vuint32_t AlarmSetStatus;

   

   

   





 

#line 34 "..\\..\\src\\Menu_time.c"



 



 



 



 




 



 
#line 67 "..\\..\\src\\Menu_time.c"

 
#line 79 "..\\..\\src\\Menu_time.c"

   



 

 
typedef struct
{
  uint8_t sec_l;
  uint8_t sec_h;
  uint8_t min_l;
  uint8_t min_h;
  uint8_t hour_l;
  uint8_t hour_h;
}tTime;

 
typedef struct
{
  uint8_t sec_l;
  uint8_t sec_h;
  uint8_t min_l;
  uint8_t min_h;
  uint8_t hour_l;
  uint8_t hour_h;
}tAlarm;

 
typedef struct
{
  uint8_t day;
  uint8_t month;
  uint16_t year;
}tDate;

   



 

   



 

static tTime  sTime;
static tAlarm sAlarm;
static tDate  sDate;

   



 







 
void RTC_Configuration(void)
{
   
  RST_CLK_LSEconfig(((uint32_t)0x00000001));
   
  while (RST_CLK_LSEstatus() != SUCCESS)
  {
  }
   
  BKP_RTCclkSource(((uint32_t)0x0004));

   
  BKP_RTC_WaitForUpdate();
   
  BKP_RTC_SetPrescaler(32768);

   
  BKP_RTC_WaitForUpdate();

   
  BKP_RTC_Enable(ENABLE);

   
  BKP_RTC_ITConfig(((uint32_t)((uint32_t)0x00000010)), ENABLE);
  NVIC_EnableIRQ(BACKUP_IRQn);
}









 
void ShowSelDigit(uint8_t Col, uint8_t Line, uint8_t ch)
{
  LCD_Method OldMethod = CurrentMethod;

  CurrentMethod = MET_AND;
  LCD_PUTC(Col, Line, ' ');
  CurrentMethod = MET_NOT_XOR;
  LCD_PUTC(Col, Line, ch);

  CurrentMethod = OldMethod;
}











 
uint8_t ReadDigit(uint8_t ColBegin, uint8_t CountBegin, uint8_t ValueMax, uint8_t ValueMin)
{
  uint32_t tmp = CountBegin;
  KeyCode key;

   
  ShowSelDigit(ColBegin, 8 + 4 + 8 + 2 + 8 + 2 + 8 + 2 + 8 + 2, (tmp + 0x30));

  for (key = GetKey(); key != SEL; key = GetKey())
  {
     
    if(key == UP)
    {
       
      if(tmp == ValueMax)
      {
        tmp = (ValueMin - 1);
      }
       
      tmp++;
      ShowSelDigit(ColBegin, 8 + 4 + 8 + 2 + 8 + 2 + 8 + 2 + 8 + 2, (tmp + 0x30));
    }
     
    if(key == DOWN)
    {
       
      if(tmp == ValueMin)
      {
        tmp = (ValueMax + 1);
      }
       
      tmp--;
      CurrentMethod = MET_AND;
      ShowSelDigit(ColBegin, 8 + 4 + 8 + 2 + 8 + 2 + 8 + 2 + 8 + 2, (tmp + 0x30));
    }
    while((GetKey() == key)){};
  }
   
  while((GetKey() == SEL)){};
  CurrentMethod = MET_AND;
  LCD_PUTC(ColBegin, 8 + 4 + 8 + 2 + 8 + 2 + 8 + 2 + 8 + 2, (tmp + 0x30));
   
  return tmp;
}







 
void Time_Display()
{
  uint32_t TimeVar = BKP_RTC_GetCounter();

   
  sTime.hour_h = (uint8_t)(TimeVar / 3600) / 10;
  LCD_PUTC(33, 8 + 4 + 8 + 2 + 8 + 2 + 8 + 2 + 8 + 2, (sTime.hour_h + 0x30));
  sTime.hour_l = (uint8_t)(((TimeVar) / 3600) % 10);
  LCD_PUTC(33 + 6, 8 + 4 + 8 + 2 + 8 + 2 + 8 + 2 + 8 + 2, (sTime.hour_l + 0x30));

   
  LCD_PUTC(33 + 6 + 6, 8 + 4 + 8 + 2 + 8 + 2 + 8 + 2 + 8 + 2, ':');

   
  sTime.min_h = (uint8_t)(((TimeVar) % 3600) / 60) / 10;
  LCD_PUTC(33 + 6 + 6 + 6, 8 + 4 + 8 + 2 + 8 + 2 + 8 + 2 + 8 + 2, (sTime.min_h + 0x30));
  sTime.min_l = (uint8_t)(((TimeVar) % 3600) / 60) % 10;
  LCD_PUTC(33 + 6 + 6 + 6 + 6, 8 + 4 + 8 + 2 + 8 + 2 + 8 + 2 + 8 + 2, (sTime.min_l + 0x30));

   
  LCD_PUTC(33 + 6 + 6 + 6 + 6 + 6, 8 + 4 + 8 + 2 + 8 + 2 + 8 + 2 + 8 + 2, ':');

   
  sTime.sec_h = (uint8_t)(((TimeVar) % 3600) % 60) / 10;
  LCD_PUTC(33 + 6 + 6 + 6 + 6 + 6 + 6, 8 + 4 + 8 + 2 + 8 + 2 + 8 + 2 + 8 + 2, (sTime.sec_h + 0x30));
  sTime.sec_l = (uint8_t)(((TimeVar) % 3600) % 60) % 10;
  LCD_PUTC(33 + 6 + 6 + 6 + 6 + 6 + 6 + 6, 8 + 4 + 8 + 2 + 8 + 2 + 8 + 2 + 8 + 2, (sTime.sec_l + 0x30));
}







 
uint32_t Time_Regulate(void)
{
  uint32_t Tmp_HH, Tmp_MM, Tmp_SS;

   
  Tmp_HH = ReadDigit(33, sTime.hour_h, 0x2, 0x0);
  if(Tmp_HH == 2)
  {
    if(sTime.hour_l > 3)
    {
      sTime.hour_l = 0;
    }
    Tmp_HH = Tmp_HH*10 + ReadDigit(33 + 6, sTime.hour_l, 0x3, 0x0);
  }
  else
  {
    Tmp_HH = Tmp_HH*10 + ReadDigit(33 + 6, sTime.hour_l, 0x9, 0x0);
  }
   
  Tmp_MM = ReadDigit(33 + 6 + 6 + 6, sTime.min_h, 0x5, 0x0);
  Tmp_MM = Tmp_MM*10 + ReadDigit(33 + 6 + 6 + 6 + 6, sTime.min_l, 0x9, 0x0);
   
  Tmp_SS = ReadDigit(33 + 6 + 6 + 6 + 6 + 6 + 6, sTime.sec_h, 0x5, 0x0);
  Tmp_SS = Tmp_SS*10 + ReadDigit(33 + 6 + 6 + 6 + 6 + 6 + 6 + 6, sTime.sec_l, 0x9, 0x0);

   
  return((Tmp_HH*3600 + Tmp_MM*60 + Tmp_SS));
}







 
void RTCHelp(void)
{
  LCD_PUTS(0, 8 + 4, "RTC is not configured");
  LCD_PUTS(0, 8 + 4 + 8 + 2, "Please, use the ");
  LCD_PUTS(0, 8 + 4 + 8 + 2 + 8 + 2, "Time Adjust menu.    ");
  while(!(GetKey() == SEL)){};
  while((GetKey() == SEL)){};
}







 
void TimePreAdjustFunc(void)
{
  sFONT *OldFont = CurrentFont;
  LCD_Method OldMethod = CurrentMethod;

   
  LCD_CLS();
  CurrentMethod = MET_AND;
  CurrentFont = &Font_6x8;
  DisplayMenuTitle("Time adjust");
  while((GetKey() == SEL)){};

  if((((BKP_TypeDef*) 0x400D8000)->REG_00 != 0x1234) || (((BKP_TypeDef*) 0x400D8000)->RTC_PRL != 32768))
  {
    LCD_PUTS(0, 8 + 4 + 8 + 2 + 8 + 2 + 8 + 2, "RTC is configured...");
     
     
    BKP_RTC_Reset(ENABLE);
    BKP_RTC_Reset(DISABLE);
     
    RTC_Configuration();
     
    BKP_RTC_SetCounter(0);
     
    BKP_RTC_WaitForUpdate();

    LCD_PUTS(0, 8 + 4 + 8 + 2 + 8 + 2 + 8 + 2, "        TIME        ");

     
    Time_Display();

     
    BKP_RTC_SetCounter(Time_Regulate());
     
    BKP_RTC_WaitForUpdate();

    ((BKP_TypeDef*) 0x400D8000)->REG_00 = 0x1234;
     
    BKP_RTC_WaitForUpdate();
  }
  else
  {
    LCD_PUTS(0, 8 + 4 + 8 + 2 + 8 + 2 + 8 + 2, "        TIME        ");

     
    Time_Display();

     
    BKP_RTC_SetCounter(Time_Regulate());
     
    BKP_RTC_WaitForUpdate();
  }
  CurrentMethod = OldMethod;
  CurrentFont = OldFont;
}







 
void TimeAdjustFunc(void)
{
  TimePreAdjustFunc();
   
  DisplayMenu();
}







 
void TimeShowFunc(void)
{
  sFONT *OldFont = CurrentFont;
  LCD_Method OldMethod = CurrentMethod;

   
  LCD_CLS();
  CurrentMethod = MET_AND;
  CurrentFont = &Font_6x8;
  DisplayMenuTitle("Time show");
  while((GetKey() == SEL)){};

  if((((BKP_TypeDef*) 0x400D8000)->REG_00 != 0x1234) || (((BKP_TypeDef*) 0x400D8000)->RTC_PRL != 32768))
  {
    RTCHelp();
  }
  else
  {
    LCD_PUTS(0, 8 + 4 + 8 + 2 + 8 + 2 + 8 + 2, "        TIME        ");
    while (GetKey() != SEL)
    {
       
      Time_Display();
    }
    while((GetKey() == SEL)){};
  }

  CurrentMethod = OldMethod;
  CurrentFont = OldFont;

   
  DisplayMenu();

}








 
static uint32_t IsLeapYear(uint32_t nYear)
{
  if(nYear % 4 != 0) return 0;
  if(nYear % 100 != 0) return 1;
  return (uint8_t)(nYear % 400 == 0);
}







 
void Date_Update(void)
{
  if(sDate.month == 1 || sDate.month == 3 || sDate.month == 5 || sDate.month == 7 ||
     sDate.month == 8 || sDate.month == 10 || sDate.month == 12)
  {
    if(sDate.day < 31)
    {
      sDate.day++;
    }
     
    else
    {
      if(sDate.month != 12)
      {
        sDate.month++;
        sDate.day = 1;
      }
       
      else
      {
        sDate.month = 1;
        sDate.day = 1;
        sDate.year++;
      }
    }
  }
  else if(sDate.month == 4 || sDate.month == 6 || sDate.month == 9 ||
          sDate.month == 11)
  {
    if(sDate.day < 30)
    {
      sDate.day++;
    }
     
    else
    {
      sDate.month++;
      sDate.day = 1;
    }
  }
  else if(sDate.month == 2)
  {
    if(sDate.day < 28)
    {
      sDate.day++;
    }
    else if(sDate.day == 28)
    {
       
      if(IsLeapYear(sDate.year))
      {
        sDate.day++;
      }
      else
      {
        sDate.month++;
        sDate.day = 1;
      }
    }
    else if(sDate.day == 29)
    {
      sDate.month++;
      sDate.day = 1;
    }
  }

  ((BKP_TypeDef*) 0x400D8000)->REG_01 = sDate.year + (sDate.month << 16) + (sDate.day << 24);
   
  BKP_RTC_WaitForUpdate();
}










 
ErrorStatus IsValidDate(uint32_t Day, uint32_t Month, uint32_t Year)
{
  if(Day == 0 || Month == 0 || Year == 0)
  {
    return ERROR;
  }
  if((Month == 1 || Month == 3 || Month == 5 || Month == 7 ||
     Month == 8 || Month == 10 || Month == 12) && (Day > 31))
  {
    return ERROR;
  }
  if((Month == 4 || Month == 6 || Month == 9 || Month == 11) && (Day > 30))
  {
    return ERROR;
  }
  if(Month == 2)
  {
    if (IsLeapYear(Year))
    {
      if (Day > 29)
      return ERROR;
    }
    else
    {
      if (Day > 28)
      return ERROR;
    }
  }
  return SUCCESS;
}







 
void Date_Display(void)
{
  uint8_t tmp;
  uint32_t temp_date;

   
  temp_date   = ((BKP_TypeDef*) 0x400D8000)->REG_01;
  sDate.day   = temp_date >> 24;
  sDate.month = (temp_date >> 16) & 0xFF;
  sDate.year  = temp_date & 0xFFFF;

   
  tmp = (uint8_t)(sDate.day / 10);
  LCD_PUTC(37, 8 + 4 + 8 + 2 + 8 + 2 + 8 + 2 + 8 + 2, (tmp + 0x30));
  tmp = (uint8_t)(sDate.day % 10);
  LCD_PUTC(37 + 6, 8 + 4 + 8 + 2 + 8 + 2 + 8 + 2 + 8 + 2, (tmp + 0x30));

   
  LCD_PUTC(37 + 6 + 6, 8 + 4 + 8 + 2 + 8 + 2 + 8 + 2 + 8 + 2, '.');

   
  tmp = (uint8_t)(sDate.month / 10);
  LCD_PUTC(37 + 6 + 6 + 6, 8 + 4 + 8 + 2 + 8 + 2 + 8 + 2 + 8 + 2, (tmp + 0x30));
  tmp = (uint8_t)(sDate.month % 10);
  LCD_PUTC(37 + 6 + 6 + 6 + 6, 8 + 4 + 8 + 2 + 8 + 2 + 8 + 2 + 8 + 2, (tmp + 0x30));

   
  LCD_PUTC(37 + 6 + 6 + 6 + 6 + 6, 8 + 4 + 8 + 2 + 8 + 2 + 8 + 2 + 8 + 2, '.');

   
  tmp = (uint8_t)(sDate.year / 1000);
  LCD_PUTC(37 + 6 + 6 + 6 + 6 + 6 + 6, 8 + 4 + 8 + 2 + 8 + 2 + 8 + 2 + 8 + 2, (tmp + 0x30));
  tmp = (uint8_t)((sDate.year / 100) % 10);
  LCD_PUTC(37 + 6 + 6 + 6 + 6 + 6 + 6 + 6, 8 + 4 + 8 + 2 + 8 + 2 + 8 + 2 + 8 + 2, (tmp + 0x30));
  tmp = (uint8_t)((sDate.year / 10) % 10);
  LCD_PUTC(37 + 6 + 6 + 6 + 6 + 6 + 6 + 6 + 6, 8 + 4 + 8 + 2 + 8 + 2 + 8 + 2 + 8 + 2, (tmp + 0x30));
  tmp = (uint8_t)(sDate.year % 10);
  LCD_PUTC(37 + 6 + 6 + 6 + 6 + 6 + 6 + 6 + 6 + 6, 8 + 4 + 8 + 2 + 8 + 2 + 8 + 2 + 8 + 2, (tmp + 0x30));
}








 
void Date_Regulate(void)
{
  uint32_t Tmp_day, Tmp_month, Tmp_year;
  FlagStatus ValidDateFlag;

  do
  {
    ValidDateFlag = SET;
     
    Tmp_day  = ReadDigit(37, (sDate.day / 10), 0x3, 0x0);
    Tmp_day  = Tmp_day * 10 + ReadDigit(37 + 6, (sDate.day % 10), 0x9, 0x0);
     
    Tmp_month = ReadDigit(37 + 6 + 6 + 6, (sDate.month / 10), 0x1, 0x0);
    Tmp_month = Tmp_month * 10 + ReadDigit(37 + 6 + 6 + 6 + 6, (sDate.month % 10), 0x2, 0x0);
     
    Tmp_year = ReadDigit(37 + 6 + 6 + 6 + 6 + 6 + 6, (sDate.year / 1000), 0x2, 0x1) * 1000;
    Tmp_year = Tmp_year + (ReadDigit(37 + 6 + 6 + 6 + 6 + 6 + 6 + 6, ((sDate.year / 100) % 10), 0x9, 0x0) * 100);
    Tmp_year = Tmp_year + (ReadDigit(37 + 6 + 6 + 6 + 6 + 6 + 6 + 6 + 6, ((sDate.year / 10) % 10), 0x9, 0x0) * 10);
    Tmp_year = Tmp_year + ReadDigit(37 + 6 + 6 + 6 + 6 + 6 + 6 + 6 + 6 + 6, (sDate.year % 10), 0x9, 0x0);
    if (!IsValidDate(Tmp_day, Tmp_month, Tmp_year))
    {
      ValidDateFlag = RESET;
      LCD_PUTS(0, 8 + 4, "   Invalid date." );
      LCD_PUTS(0, 8 + 4 + 8 + 2, "   Repeat, please.");
      SysTickDelay(2500);
      LCD_PUTS(0, 8 + 4, "                " );
      LCD_PUTS(0, 8 + 4 + 8 + 2, "                  ");
      sDate.day   = Tmp_day;
      sDate.month = Tmp_month;
      sDate.year  = Tmp_year;
    }
  } while (ValidDateFlag == RESET);
  ((BKP_TypeDef*) 0x400D8000)->REG_01 = Tmp_year + (Tmp_month << 16) + (Tmp_day << 24);
   
  BKP_RTC_WaitForUpdate();
}







 
void DatePreAdjustFunc(void)
{
  sFONT *OldFont = CurrentFont;
  LCD_Method OldMethod = CurrentMethod;

   
  LCD_CLS();
  CurrentMethod = MET_AND;
  CurrentFont = &Font_6x8;
  DisplayMenuTitle("Date adjust");
  while((GetKey() == SEL)){};

  if((((BKP_TypeDef*) 0x400D8000)->REG_00 != 0x1234) || (((BKP_TypeDef*) 0x400D8000)->RTC_PRL != 32768))
  {
    RTCHelp();
  }
  else
  {
    LCD_PUTS(0, 8 + 4 + 8 + 2 + 8 + 2 + 8 + 2, "DATE  DD.MM.YYYY");

     
    Date_Display();
     
    Date_Regulate();
  }

  CurrentMethod = OldMethod;
  CurrentFont = OldFont;
}







 
void DateAdjustFunc(void)
{
  DatePreAdjustFunc();
   
  DisplayMenu();
}







 
void DateShowFunc(void)
{
  sFONT *OldFont = CurrentFont;
  LCD_Method OldMethod = CurrentMethod;

   
  LCD_CLS();
  CurrentMethod = MET_AND;
  CurrentFont = &Font_6x8;
  DisplayMenuTitle("Date show");
  while((GetKey() == SEL)){};

  if((((BKP_TypeDef*) 0x400D8000)->REG_00 != 0x1234) || (((BKP_TypeDef*) 0x400D8000)->RTC_PRL != 32768))
  {
    RTCHelp();
  }
  else
  {
    LCD_PUTS(0, 8 + 4 + 8 + 2 + 8 + 2 + 8 + 2, "DATE  DD.MM.YYYY");

    while (GetKey() != SEL)
    {
       
      Date_Display();
    }
    while((GetKey() == SEL)){};
  }

  CurrentMethod = OldMethod;
  CurrentFont = OldFont;

   
  DisplayMenu();
}







 
void Alarm_Display()
{
  uint32_t AlarmVar = ((BKP_TypeDef*) 0x400D8000)->RTC_ALRM;

   
  LCD_PUTS(33, 8 + 4 + 8 + 2 + 8 + 2 + 8 + 2 + 8 + 2, "  :  :");

   
  sAlarm.hour_h = (uint8_t)(AlarmVar / 3600) / 10;
  LCD_PUTC(33, 8 + 4 + 8 + 2 + 8 + 2 + 8 + 2 + 8 + 2, (sAlarm.hour_h + 0x30));
  sAlarm.hour_l = (uint8_t)(((AlarmVar) / 3600) % 10);
  LCD_PUTC(33 + 6, 8 + 4 + 8 + 2 + 8 + 2 + 8 + 2 + 8 + 2, (sAlarm.hour_l + 0x30));

   
  sAlarm.min_h = (uint8_t)(((AlarmVar) % 3600) / 60) / 10;
  LCD_PUTC(33 + 6 + 6 + 6, 8 + 4 + 8 + 2 + 8 + 2 + 8 + 2 + 8 + 2, (sAlarm.min_h + 0x30));
  sAlarm.min_l = (uint8_t)(((AlarmVar) % 3600) / 60) % 10;
  LCD_PUTC(33 + 6 + 6 + 6 + 6, 8 + 4 + 8 + 2 + 8 + 2 + 8 + 2 + 8 + 2, (sAlarm.min_l + 0x30));

   
  sAlarm.sec_h = (uint8_t)(((AlarmVar) % 3600) % 60) / 10;
  LCD_PUTC(33 + 6 + 6 + 6 + 6 + 6 + 6, 8 + 4 + 8 + 2 + 8 + 2 + 8 + 2 + 8 + 2, (sAlarm.sec_h + 0x30));
  sAlarm.sec_l = (uint8_t)(((AlarmVar) % 3600) % 60) % 10;
  LCD_PUTC(33 + 6 + 6 + 6 + 6 + 6 + 6 + 6, 8 + 4 + 8 + 2 + 8 + 2 + 8 + 2 + 8 + 2, (sAlarm.sec_l + 0x30));
}







 
uint32_t Alarm_Regulate(void)
{
  uint32_t Alarm_HH, Alarm_MM, Alarm_SS;

   
  Alarm_HH = ReadDigit(33, sAlarm.hour_h, 0x2, 0x0);
  if(Alarm_HH == 2)
  {
    if(sAlarm.hour_l > 3)
    {
      sAlarm.hour_l = 0;
    }
    Alarm_HH = Alarm_HH*10 + ReadDigit(33 + 6, sAlarm.hour_l, 0x3, 0x0);
  }
  else
  {
    Alarm_HH = Alarm_HH*10 + ReadDigit(33 + 6, sAlarm.hour_l, 0x9, 0x0);
  }
   
  Alarm_MM = ReadDigit(33 + 6 + 6 + 6, sAlarm.min_h, 0x5, 0x0);
  Alarm_MM = Alarm_MM*10 + ReadDigit(33 + 6 + 6 + 6 + 6, sAlarm.min_l, 0x9, 0x0);
   
  Alarm_SS = ReadDigit(33 + 6 + 6 + 6 + 6 + 6 + 6, sAlarm.sec_h, 0x5, 0x0);
  Alarm_SS = Alarm_SS*10 + ReadDigit(33 + 6 + 6 + 6 + 6 + 6 + 6 + 6, sAlarm.sec_l, 0x9, 0x0);

   
  ((BKP_TypeDef*) 0x400D8000)->REG_02 = Alarm_HH*3600 + Alarm_MM*60 + Alarm_SS;

   
  return((Alarm_HH*3600 + Alarm_MM*60 + Alarm_SS));
}







 
void AlarmAdjustFunc(void)
{
  sFONT *OldFont = CurrentFont;
  LCD_Method OldMethod = CurrentMethod;

   
  LCD_CLS();
  CurrentMethod = MET_AND;
  CurrentFont = &Font_6x8;
  DisplayMenuTitle("Alarm adjust");
  while((GetKey() == SEL)){};

  if((((BKP_TypeDef*) 0x400D8000)->REG_00 != 0x1234) || (((BKP_TypeDef*) 0x400D8000)->RTC_PRL != 32768))
  {
    RTCHelp();
  }
  else
  {
    LCD_PUTS(33 - 6, 8 + 4 + 8 + 2 + 8 + 2 + 8 + 2, "ALARM TIME");

     
    BKP_RTC_SetAlarm(((BKP_TypeDef*) 0x400D8000)->REG_02);

     
    Alarm_Display();

     
    BKP_RTC_SetAlarm(Alarm_Regulate());
     
    BKP_RTC_WaitForUpdate();
     
    BKP_RTC_ITConfig(((uint32_t)((uint32_t)0x00000020)), ENABLE);
    AlarmSetStatus = 1;
  }
  CurrentMethod = OldMethod;
  CurrentFont = OldFont;

   
  DisplayMenu();
}







 
void AlarmShowFunc(void)
{
  sFONT *OldFont = CurrentFont;
  LCD_Method OldMethod = CurrentMethod;

   
  LCD_CLS();
  CurrentMethod = MET_AND;
  CurrentFont = &Font_6x8;
  DisplayMenuTitle("Alarm show");
  while((GetKey() == SEL)){};

  if((((BKP_TypeDef*) 0x400D8000)->REG_00 != 0x1234) || (((BKP_TypeDef*) 0x400D8000)->RTC_PRL != 32768))
  {
    RTCHelp();
  }
  else
  {
    LCD_PUTS(33 - 6, 8 + 4 + 8 + 2 + 8 + 2 + 8 + 2, "ALARM TIME");
     
    Alarm_Display();
    while(!(GetKey() == SEL)){};
    while((GetKey() == SEL)){};
  }

  CurrentMethod = OldMethod;
  CurrentFont = OldFont;

   
  DisplayMenu();
}







 
void Calendar_Init(void)
{
  uint8_t tmp, i;
  KeyCode key;

   
  RST_CLK_PCLKcmd(((uint32_t)(1 << ((((uint32_t)0x400D8000) >> 15) & 0x1F))),ENABLE);

  RTC_Configuration();

  if(((BKP_TypeDef*) 0x400D8000)->REG_00 != 0x1234)
  {
    LCD_CLS();
    LCD_PUTS(0, 8 + 4, " To configuring Time ");
    LCD_PUTS(0, 8 + 4 + 8 + 2, " and Date press SEL, ");
    LCD_PUTS(0, 8 + 4 + 8 + 2 + 8 + 2, " else press any key. ");

    for (key = NOKEY; key == NOKEY; key = GetKey())
    {
    }
    if (key == SEL)
    {
      while((GetKey() == key)){};
      TimePreAdjustFunc();
      DatePreAdjustFunc();
    }
    else
    {
       
      sDate.day   = 1;
      sDate.month = 1;
      sDate.year  = 2010;
      ((BKP_TypeDef*) 0x400D8000)->REG_01 = sDate.year + (sDate.month << 16) + (sDate.day << 24);
       
      BKP_RTC_WaitForUpdate();
       
      ((BKP_TypeDef*) 0x400D8000)->REG_02 = 0;
       
      BKP_RTC_WaitForUpdate();
    }
    while((GetKey() == key)){};
  }
  else
  {
     
    tmp = ((BKP_TypeDef*) 0x400D8000)->REG_01;
    sDate.day   = tmp >> 24;
    sDate.month = (tmp >> 16) & 0xFF;
    sDate.year  = tmp & 0xFFFF;

    tmp = BKP_RTC_GetCounter();
    if(tmp / 86399 != 0)
    {
      for(i = 0; i < (tmp / 86399); i++)
      {
        Date_Update();
      }
      BKP_RTC_SetCounter(tmp % 86399);
    }
  }
}

   

   

   

   



 
