#line 1 "..\\..\\src\\Menu_VCOM.c"

















 
 
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





 

#line 21 "..\\..\\src\\Menu_VCOM.c"
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

   

   

   





 
#line 22 "..\\..\\src\\Menu_VCOM.c"
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

   

   

   





 

#line 23 "..\\..\\src\\Menu_VCOM.c"
#line 1 "..\\..\\..\\Library\\Libraries\\1986BE9x_StdPeriph_Driver\\inc\\1986BE9x_rst_clk.h"





















 

 



 
#line 30 "..\\..\\..\\Library\\Libraries\\1986BE9x_StdPeriph_Driver\\inc\\1986BE9x_rst_clk.h"
#line 31 "..\\..\\..\\Library\\Libraries\\1986BE9x_StdPeriph_Driver\\inc\\1986BE9x_rst_clk.h"



 



 



 



 

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

   



 



 



 









   



 



 









   



 



 











   



 



 

#line 151 "..\\..\\..\\Library\\Libraries\\1986BE9x_StdPeriph_Driver\\inc\\1986BE9x_rst_clk.h"



   




 



 











   



 



 

#line 201 "..\\..\\..\\Library\\Libraries\\1986BE9x_StdPeriph_Driver\\inc\\1986BE9x_rst_clk.h"



   




 



 

#line 224 "..\\..\\..\\Library\\Libraries\\1986BE9x_StdPeriph_Driver\\inc\\1986BE9x_rst_clk.h"

#line 234 "..\\..\\..\\Library\\Libraries\\1986BE9x_StdPeriph_Driver\\inc\\1986BE9x_rst_clk.h"

   



 



 











   



 



 
#line 271 "..\\..\\..\\Library\\Libraries\\1986BE9x_StdPeriph_Driver\\inc\\1986BE9x_rst_clk.h"

#line 279 "..\\..\\..\\Library\\Libraries\\1986BE9x_StdPeriph_Driver\\inc\\1986BE9x_rst_clk.h"
   



 



 

#line 298 "..\\..\\..\\Library\\Libraries\\1986BE9x_StdPeriph_Driver\\inc\\1986BE9x_rst_clk.h"

#line 308 "..\\..\\..\\Library\\Libraries\\1986BE9x_StdPeriph_Driver\\inc\\1986BE9x_rst_clk.h"

   




 



 



#line 354 "..\\..\\..\\Library\\Libraries\\1986BE9x_StdPeriph_Driver\\inc\\1986BE9x_rst_clk.h"






   



 



 

#line 376 "..\\..\\..\\Library\\Libraries\\1986BE9x_StdPeriph_Driver\\inc\\1986BE9x_rst_clk.h"

#line 383 "..\\..\\..\\Library\\Libraries\\1986BE9x_StdPeriph_Driver\\inc\\1986BE9x_rst_clk.h"

   






 



 

#line 406 "..\\..\\..\\Library\\Libraries\\1986BE9x_StdPeriph_Driver\\inc\\1986BE9x_rst_clk.h"

#line 416 "..\\..\\..\\Library\\Libraries\\1986BE9x_StdPeriph_Driver\\inc\\1986BE9x_rst_clk.h"

   



 



 
#line 435 "..\\..\\..\\Library\\Libraries\\1986BE9x_StdPeriph_Driver\\inc\\1986BE9x_rst_clk.h"

#line 445 "..\\..\\..\\Library\\Libraries\\1986BE9x_StdPeriph_Driver\\inc\\1986BE9x_rst_clk.h"

   

   



 

   



 

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

   

   

   





 

#line 24 "..\\..\\src\\Menu_VCOM.c"
#line 1 "..\\..\\..\\Library\\Libraries\\1986BE9x_StdPeriph_Driver\\inc\\1986be9x_uart.h"





















 

 



 
#line 30 "..\\..\\..\\Library\\Libraries\\1986BE9x_StdPeriph_Driver\\inc\\1986be9x_uart.h"
#line 31 "..\\..\\..\\Library\\Libraries\\1986BE9x_StdPeriph_Driver\\inc\\1986be9x_uart.h"



 



 



 

typedef enum {BaudRateInvalid = 0, BaudRateValid = !BaudRateInvalid} BaudRateStatus;



 

typedef struct
{
  uint32_t UART_BaudRate;            


 
  uint16_t UART_WordLength;          
 
  uint16_t UART_StopBits;            
 
  uint16_t UART_Parity;              
 
  uint16_t UART_FIFOMode;            
 
  uint16_t UART_HardwareFlowControl; 
 
}UART_InitTypeDef;

   




 






 











   



 







   



 













   



 







   



 
#line 149 "..\\..\\..\\Library\\Libraries\\1986BE9x_StdPeriph_Driver\\inc\\1986be9x_uart.h"

#line 161 "..\\..\\..\\Library\\Libraries\\1986BE9x_StdPeriph_Driver\\inc\\1986be9x_uart.h"

   



 

#line 179 "..\\..\\..\\Library\\Libraries\\1986BE9x_StdPeriph_Driver\\inc\\1986be9x_uart.h"

#line 186 "..\\..\\..\\Library\\Libraries\\1986BE9x_StdPeriph_Driver\\inc\\1986be9x_uart.h"

   



 













   



 









   



 







   



 

#line 246 "..\\..\\..\\Library\\Libraries\\1986BE9x_StdPeriph_Driver\\inc\\1986be9x_uart.h"









   



 

















   



 

#line 291 "..\\..\\..\\Library\\Libraries\\1986BE9x_StdPeriph_Driver\\inc\\1986be9x_uart.h"



   

   



 

   



 

void UART_DeInit(UART_TypeDef* UARTx);
BaudRateStatus UART_Init(UART_TypeDef* UARTx, UART_InitTypeDef* UART_InitStruct);
void UART_StructInit(UART_InitTypeDef* UART_InitStruct);

void UART_Cmd(UART_TypeDef* UARTx, FunctionalState NewState);

void UART_ITConfig(UART_TypeDef* UARTx, uint32_t UART_IT, FunctionalState NewState);
ITStatus UART_GetITStatus(UART_TypeDef* UARTx, uint32_t UART_IT);
ITStatus UART_GetITStatusMasked(UART_TypeDef* UARTx, uint32_t UART_IT);
void UART_ClearITPendingBit(UART_TypeDef* UARTx, uint32_t UART_IT);

void UART_DMAConfig(UART_TypeDef* UARTx, uint32_t UART_IT_RB_LVL, uint32_t UART_IT_TB_LVL);
void UART_DMACmd(UART_TypeDef* UARTx, uint32_t UART_DMAReq, FunctionalState NewState);

void UART_SendData(UART_TypeDef* UARTx, uint16_t Data);
uint16_t UART_ReceiveData(UART_TypeDef* UARTx);
void UART_BreakLine(UART_TypeDef* UARTx, FunctionalState NewState);

void UART_IrDAConfig(UART_TypeDef* UARTx, uint32_t UART_IrDAMode);
void UART_IrDACmd(UART_TypeDef* UARTx, FunctionalState NewState);

FlagStatus UART_GetFlagStatus(UART_TypeDef* UARTx, uint32_t UART_FLAG);
void UART_BRGInit(UART_TypeDef* UARTx, uint32_t UART_BRG);

   

   

   





 

#line 25 "..\\..\\src\\Menu_VCOM.c"
#line 1 "..\\..\\Inc\\1986be9x_usb_handlers.h"


















 

 



 
#line 1 "..\\..\\..\\Library\\Libraries\\1986BE9x_StdPeriph_Driver\\inc\\USB_Library\\1986BE9x_usb_default_handlers.h"


















 

 




 
#line 1 "..\\..\\..\\Library\\Libraries\\1986BE9x_StdPeriph_Driver\\inc\\USB_Library\\1986BE9x_usb_cdc.h"

























 

 



 
#line 34 "..\\..\\..\\Library\\Libraries\\1986BE9x_StdPeriph_Driver\\inc\\USB_Library\\1986BE9x_usb_cdc.h"
#line 35 "..\\..\\..\\Library\\Libraries\\1986BE9x_StdPeriph_Driver\\inc\\USB_Library\\1986BE9x_usb_cdc.h"
#line 1 "..\\..\\..\\Library\\Libraries\\1986BE9x_StdPeriph_Driver\\inc\\USB_Library\\1986BE9x_usb_device.h"






















 

 



 
#line 31 "..\\..\\..\\Library\\Libraries\\1986BE9x_StdPeriph_Driver\\inc\\USB_Library\\1986BE9x_usb_device.h"
#line 32 "..\\..\\..\\Library\\Libraries\\1986BE9x_StdPeriph_Driver\\inc\\USB_Library\\1986BE9x_usb_device.h"
#line 1 "..\\..\\..\\Library\\Libraries\\1986BE9x_StdPeriph_Driver\\inc\\1986BE9x_usb.h"





















 

 



 
#line 30 "..\\..\\..\\Library\\Libraries\\1986BE9x_StdPeriph_Driver\\inc\\1986BE9x_usb.h"
#line 31 "..\\..\\..\\Library\\Libraries\\1986BE9x_StdPeriph_Driver\\inc\\1986BE9x_usb.h"



 



 



 



 

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

   



 



   



 








   



 

#line 120 "..\\..\\..\\Library\\Libraries\\1986BE9x_StdPeriph_Driver\\inc\\1986BE9x_usb.h"









   



 



 

























#line 171 "..\\..\\..\\Library\\Libraries\\1986BE9x_StdPeriph_Driver\\inc\\1986BE9x_usb.h"





   



 























   



 











   



 



























   



 







   



 














   



 























   



 

























#line 349 "..\\..\\..\\Library\\Libraries\\1986BE9x_StdPeriph_Driver\\inc\\1986BE9x_usb.h"






   



 









   

#line 383 "..\\..\\..\\Library\\Libraries\\1986BE9x_StdPeriph_Driver\\inc\\1986BE9x_usb.h"

   



 



 



























   



 

























#line 458 "..\\..\\..\\Library\\Libraries\\1986BE9x_StdPeriph_Driver\\inc\\1986BE9x_usb.h"






   



 











   



 











   



 



















#line 526 "..\\..\\..\\Library\\Libraries\\1986BE9x_StdPeriph_Driver\\inc\\1986BE9x_usb.h"






   



 









   



 



















   



 



























   

#line 611 "..\\..\\..\\Library\\Libraries\\1986BE9x_StdPeriph_Driver\\inc\\1986BE9x_usb.h"

   




 

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

   

   

   





 

#line 33 "..\\..\\..\\Library\\Libraries\\1986BE9x_StdPeriph_Driver\\inc\\USB_Library\\1986BE9x_usb_device.h"
#line 34 "..\\..\\..\\Library\\Libraries\\1986BE9x_StdPeriph_Driver\\inc\\USB_Library\\1986BE9x_usb_device.h"



 



 



 



 

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
#line 280 "..\\..\\..\\Library\\Libraries\\1986BE9x_StdPeriph_Driver\\inc\\USB_Library\\1986BE9x_usb_device.h"
  uint32_t                           Reserved;

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

   

   

   

   

   





 

#line 36 "..\\..\\..\\Library\\Libraries\\1986BE9x_StdPeriph_Driver\\inc\\USB_Library\\1986BE9x_usb_cdc.h"



 



 



 



 



 

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
  USB_CDC_bRxCarrier  = 0x01,
  USB_CDC_bTxCarrier  = 0x02,
  USB_CDC_bBreak      = 0x04,
  USB_CDC_bRingSignal = 0x08,
  USB_CDC_bFraming    = 0x10,
  USB_CDC_bParity     = 0x20,
  USB_CDC_bOverRun    = 0x40
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
  USB_CDC_DATA_BITS5  = 0x5,
  USB_CDC_DATA_BITS6  = 0x6,
  USB_CDC_DATA_BITS7  = 0x7,
  USB_CDC_DATA_BITS8  = 0x8,
  USB_CDC_DATA_BITS16 = 0xA
}USB_CDC_DataBits_TypeDef;



 

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


USB_Result USB_CDC_ReportState(uint16_t LineState);




 

USB_Result USB_CDC_Reset(void);
USB_Result USB_CDC_GetDescriptor(uint16_t wVALUE, uint16_t wINDEX, uint16_t wLENGTH);
USB_Result USB_CDC_ClassRequest(void);

   



 

USB_Result USB_CDC_DummyDataReceive(uint8_t* Buffer, uint32_t Length);
USB_Result USB_CDC_DummyDataSent(void);













USB_Result USB_CDC_DummyGetLineCoding(uint16_t wINDEX, USB_CDC_LineCoding_TypeDef* DATA);
USB_Result USB_CDC_DummySetLineCoding(uint16_t wINDEX, const USB_CDC_LineCoding_TypeDef* DATA);










   

   

   

   

   





 

#line 28 "..\\..\\..\\Library\\Libraries\\1986BE9x_StdPeriph_Driver\\inc\\USB_Library\\1986BE9x_usb_default_handlers.h"

 






 



 



 



 



 





 





 





 





 





 



 






 






 





 



 





 



 





 



 





 



 






 



 





 



 






 



 





 



 





 



 





 

   



 




 



 



 





 



 





 



 





 


#line 268 "..\\..\\..\\Library\\Libraries\\1986BE9x_StdPeriph_Driver\\inc\\USB_Library\\1986BE9x_usb_default_handlers.h"

#line 302 "..\\..\\..\\Library\\Libraries\\1986BE9x_StdPeriph_Driver\\inc\\USB_Library\\1986BE9x_usb_default_handlers.h"




 





 



 





 



#line 338 "..\\..\\..\\Library\\Libraries\\1986BE9x_StdPeriph_Driver\\inc\\USB_Library\\1986BE9x_usb_default_handlers.h"

#line 351 "..\\..\\..\\Library\\Libraries\\1986BE9x_StdPeriph_Driver\\inc\\USB_Library\\1986BE9x_usb_default_handlers.h"



 





 

   

   



 











   



   

   

   

   





 

#line 27 "..\\..\\Inc\\1986be9x_usb_handlers.h"

 







 



 



 



 




#line 59 "..\\..\\Inc\\1986be9x_usb_handlers.h"






   




 

USB_Result USB_CDC_RecieveData(uint8_t* Buffer, uint32_t Length);


USB_Result USB_CDC_GetLineCoding(uint16_t wINDEX, USB_CDC_LineCoding_TypeDef* DATA);
USB_Result USB_CDC_SetLineCoding(uint16_t wINDEX, const USB_CDC_LineCoding_TypeDef* DATA);


   

   

   

   





 

#line 26 "..\\..\\src\\Menu_VCOM.c"
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

   

   

   

   





 

#line 27 "..\\..\\src\\Menu_VCOM.c"
#line 1 "..\\..\\Inc\\Menu.h"


















 

 



 
#line 27 "..\\..\\Inc\\Menu.h"



 



 



 



   



 



   



 

void Menu_Init(void);
void DisplayMenuTitle(ucint8_t *ptr);
void DisplayMenu(void);
 
void BackToMenuOnSel(void);
 
void ReadKey(void);

   

   

   





 

#line 28 "..\\..\\src\\Menu_VCOM.c"
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

   

   

   

   





 

#line 29 "..\\..\\src\\Menu_VCOM.c"
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

   

   

   





 

#line 30 "..\\..\\src\\Menu_VCOM.c"
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

   

   

   





 

#line 31 "..\\..\\src\\Menu_VCOM.c"
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

   

   

   





 

#line 32 "..\\..\\src\\Menu_VCOM.c"
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

   

   

   





 

#line 33 "..\\..\\src\\Menu_VCOM.c"
#line 1 "..\\..\\Inc\\demo_init.h"

















 

 





 



 



 

static PORT_InitTypeDef PortInitStructure;

   



 

void Demo_Init(void);

   

   

   



#line 34 "..\\..\\src\\Menu_VCOM.c"
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

   

   

   





 

#line 35 "..\\..\\src\\Menu_VCOM.c"



 



 



 



 



#line 63 "..\\..\\src\\Menu_VCOM.c"

   



 

   



 

   



 

static uint8_t SendBuffer[100];
static uint8_t ReceiveBuffer[100];

static vuint32_t send_data_length = 0;
static vuint32_t send_data_pos = 0;
static vuint32_t receive_data_length = 0;

USB_Clock_TypeDef USB_Clock_InitStruct;
USB_DeviceBUSParam_TypeDef USB_DeviceBUSParam;

static UART_InitTypeDef UARTInitStructure;

static uint32_t temp_CPU_CLOCK, temp_PLL_CONTROL;


static USB_CDC_LineCoding_TypeDef LineCoding;






   



 

 
static void Setup_CPU_Clock(void);
static void Setup_USB(void);
static void UARTConfiguration(void);
static void SenderFunc(void);
static void ReceiverFunc(void);

static void LineStateFunc(uint32_t events);


 






 
static void Setup_CPU_Clock(void)
{
   
  RST_CLK_HSEconfig(((uint32_t)0x00000001));
  if (RST_CLK_HSEstatus() != SUCCESS)
  {
     
    while (1)
    {
    }
  }

   
  RST_CLK_CPU_PLLconfig(((uint32_t)0x00000002), ((uint32_t)0x00000009));
  RST_CLK_CPU_PLLcmd(ENABLE);
  if (RST_CLK_CPU_PLLstatus() != SUCCESS)
  {
     
    while (1)
    {
    }
  }

   
  RST_CLK_CPUclkPrescaler(((uint32_t)0x00000000));
   
  RST_CLK_CPU_PLLuse(ENABLE);
   
  RST_CLK_CPUclkSelection(((uint32_t)0x00000100));
}






 
static void Setup_USB(void)
{
   
  RST_CLK_PCLKcmd(((uint32_t)(1 << ((((uint32_t)0x40010000) >> 15) & 0x1F))), ENABLE);

   
  USB_Clock_InitStruct.USB_USBC1_Source = ((uint32_t)0x00000003);
  USB_Clock_InitStruct.USB_PLLUSBMUL    = ((uint32_t)0x0000000B);

  USB_DeviceBUSParam.MODE  = (uint32_t)(1 << 4);
  USB_DeviceBUSParam.SPEED = (uint32_t)(1 << 5);
  USB_DeviceBUSParam.PULL  = (uint32_t)(1 << 4);

  USB_DeviceInit(&USB_Clock_InitStruct, &USB_DeviceBUSParam);

   
  USB_SetSIM(((uint32_t)0x00000001) | ((uint32_t)0x00000002) | ((uint32_t)0x00000004) | ((uint32_t)0x00000008) | ((uint32_t)0x00000010));

  USB_DevicePowerOn();

   

  NVIC_EnableIRQ(USB_IRQn);


  USB_CDC_Reset();
}








 
USB_Result USB_CDC_RecieveData(uint8_t* Buffer, uint32_t Length)
{
   
  send_data_length = Length;
  send_data_pos = 0;

  SenderFunc();

  return USB_SUCCESS;
}










 
USB_Result USB_CDC_GetLineCoding(uint16_t wINDEX, USB_CDC_LineCoding_TypeDef* DATA)
{
  ((void)0);
  if (wINDEX != 0)
  {
     
    return USB_ERR_INV_REQ;
  }

   
  *DATA = LineCoding;

  return USB_SUCCESS;
}








 
USB_Result USB_CDC_SetLineCoding(uint16_t wINDEX, const USB_CDC_LineCoding_TypeDef* DATA)
{
  ((void)0);
  if (wINDEX != 0)
  {
     
    return USB_ERR_INV_REQ;
  }

   

   
  UARTInitStructure.UART_BaudRate = DATA->dwDTERate;

   
  switch (DATA->bCharFormat)
  {
    case USB_CDC_STOP_BITS1:
      UARTInitStructure.UART_StopBits = ((uint16_t)0x0000);
      break;
    case USB_CDC_STOP_BITS2:
      UARTInitStructure.UART_StopBits = ((uint16_t)0x0008);
      break;
    default :
      return USB_ERR_INV_REQ;
  }

   
  switch (DATA->bParityType)
  {
    case USB_CDC_PARITY_NONE:
      UARTInitStructure.UART_Parity = ((uint16_t)0x0000);
      break;
    case USB_CDC_PARITY_ODD:
      UARTInitStructure.UART_Parity = ((uint16_t)0x0002);
      break;
    case USB_CDC_PARITY_EVEN:
      UARTInitStructure.UART_Parity = ((uint16_t)0x0006);
      break;
    case USB_CDC_PARITY_MARK:
      UARTInitStructure.UART_Parity = ((uint16_t)0x0086);
      break;
    case USB_CDC_PARITY_SPACE:
      UARTInitStructure.UART_Parity = ((uint16_t)0x0082);
      break;
    default :
      return USB_ERR_INV_REQ;
  }

   
  switch (DATA->bDataBits)
  {
    case USB_CDC_DATA_BITS5:
      UARTInitStructure.UART_WordLength = ((uint16_t)0x0000);
      break;
    case USB_CDC_DATA_BITS6:
      UARTInitStructure.UART_WordLength = ((uint16_t)0x0020);
      break;
    case USB_CDC_DATA_BITS7:
      UARTInitStructure.UART_WordLength = ((uint16_t)0x0040);
      break;
    case USB_CDC_DATA_BITS8:
      UARTInitStructure.UART_WordLength = ((uint16_t)0x0060);
      break;
    default :
      return USB_ERR_INV_REQ;
  }

   
  UART_Cmd(((UART_TypeDef*) 0x40038000), DISABLE);
  UART_DeInit(((UART_TypeDef*) 0x40038000));
  UART_Init(((UART_TypeDef*) 0x40038000), &UARTInitStructure);
   
  UART_ITConfig (((UART_TypeDef*) 0x40038000), ((uint32_t)0x00000020) | ((uint32_t)0x00000010)

              | (((uint32_t)0x00000400) | ((uint32_t)0x00000200) | ((uint32_t)0x00000100) | ((uint32_t)0x00000080) | ((uint32_t)0x00000008) | ((uint32_t)0x00000004) | ((uint32_t)0x00000001))

              , ENABLE);
  UART_Cmd(((UART_TypeDef*) 0x40038000), ENABLE);

   
  LineCoding = *DATA;

  return USB_SUCCESS;
}



#line 368 "..\\..\\src\\Menu_VCOM.c"








 
static void UARTConfiguration(void)
{
   
  RST_CLK_PCLKcmd(((uint32_t)(1 << ((((uint32_t)0x40038000) >> 15) & 0x1F))), ENABLE);

   
  UART_BRGInit(((UART_TypeDef*) 0x40038000), ((uint32_t)0x00000004));

  UART_DeInit(((UART_TypeDef*) 0x40038000));


   
  PortInitStructure.PORT_Pin   = 0x0001U | 0x0002U;
  PortInitStructure.PORT_MODE  = PORT_MODE_DIGITAL;
  PortInitStructure.PORT_FUNC  = PORT_FUNC_OVERRID;
  PortInitStructure.PORT_SPEED = PORT_SPEED_MAXFAST;
  PORT_Init(((PORT_TypeDef*) 0x400E8000), &PortInitStructure);
#line 405 "..\\..\\src\\Menu_VCOM.c"

   
  NVIC_EnableIRQ(UART2_IRQn);
}







 
void VCOMHelp(void)
{
  sFONT *OldFont = CurrentFont;
  LCD_Method OldMethod = CurrentMethod;

  LCD_CLS();
  CurrentMethod = MET_AND;
  CurrentFont = &Font_6x8;
  DisplayMenuTitle("USB. Virtual COM");
  while((GetKey() == SEL)){};
  LCD_PUTS(0, CurrentFont->Height * 2 + 2,     "  To stop Press SEL");

  CurrentMethod = OldMethod;
  CurrentFont = OldFont;
}







 
static void SenderFunc(void)
{
  for (; (send_data_pos < send_data_length && (UART_GetFlagStatus (((UART_TypeDef*) 0x40038000), ((uint32_t)0x00000020)) == RESET)); send_data_pos++)
  {
    UART_SendData(((UART_TypeDef*) 0x40038000), SendBuffer[send_data_pos]);
  }
}







 
static void ReceiverFunc(void)
{
  uint16_t receive_data;

   
  for (receive_data_length = 0; ((UART_GetFlagStatus(((UART_TypeDef*) 0x40038000), ((uint32_t)0x00000010)) == RESET) && (receive_data_length < 100)); )
  {
    receive_data = UART_ReceiveData(((UART_TypeDef*) 0x40038000));

    if (((uint8_t)(receive_data >> 8)) == 0)
    {
      ReceiveBuffer[receive_data_length++] = ((uint8_t)receive_data);
    }
  }
   
  USB_CDC_SendData(ReceiveBuffer, receive_data_length);
}








 
static void LineStateFunc(uint32_t events)
{
  uint16_t lineState = 0;
  if (events & ((uint32_t)0x00000400))
  {
  lineState |= USB_CDC_bOverRun;
  }
  if (events & ((uint32_t)0x00000200))
  {
  lineState |= USB_CDC_bBreak;
  }
  if (events & ((uint32_t)0x00000100))
  {
  lineState |= USB_CDC_bParity;
  }
  if (events & ((uint32_t)0x00000080))
  {
  lineState |= USB_CDC_bFraming;
  }
  if (events & ((uint32_t)0x00000008))
  {
  lineState |= USB_CDC_bTxCarrier;
  }
  if (events & ((uint32_t)0x00000004))
  {
  lineState |= USB_CDC_bRxCarrier;
  }
  if (events & ((uint32_t)0x00000001))
  {
  lineState |= USB_CDC_bRingSignal;
  }

  if (lineState)
  {
    USB_CDC_ReportState(lineState);
  }
}











 
void VCOMFunc(void) {
  uint32_t key;

   
  VCOMHelp();

   
  temp_CPU_CLOCK   = ((RST_CLK_TypeDef*) 0x40020000)->CPU_CLOCK;
  temp_PLL_CONTROL = ((RST_CLK_TypeDef*) 0x40020000)->PLL_CONTROL;

  Setup_CPU_Clock();

  UARTConfiguration();

   
  USB_CDC_Init(SendBuffer, 1, SET);

   
  UARTInitStructure.UART_BaudRate                = 14400;
  UARTInitStructure.UART_WordLength              = ((uint16_t)0x0060);
  UARTInitStructure.UART_StopBits                = ((uint16_t)0x0000);
  UARTInitStructure.UART_Parity                  = ((uint16_t)0x0000);
  UARTInitStructure.UART_FIFOMode                = ((uint16_t)0x0000);
  UARTInitStructure.UART_HardwareFlowControl     = ((uint16_t)0x0200) |                                                    ((uint16_t)0x0100);

  

 
 

  UART_Init (((UART_TypeDef*) 0x40038000),&UARTInitStructure);


   
  LineCoding.dwDTERate = UARTInitStructure.UART_BaudRate;
  LineCoding.bCharFormat = USB_CDC_STOP_BITS1;
  LineCoding.bParityType = USB_CDC_PARITY_NONE;
  LineCoding.bDataBits = USB_CDC_DATA_BITS8;


   
  pfUARTReceiverFunc = ReceiverFunc;
  pfUARTSenderFunc = SenderFunc;

  pfUARTLineStateFunc = LineStateFunc;


   
  UART_ITConfig (((UART_TypeDef*) 0x40038000), ((uint32_t)0x00000020) | ((uint32_t)0x00000010)

              | (((uint32_t)0x00000400) | ((uint32_t)0x00000200) | ((uint32_t)0x00000100) | ((uint32_t)0x00000080) | ((uint32_t)0x00000008) | ((uint32_t)0x00000004) | ((uint32_t)0x00000001))

              , ENABLE);

   
  UART_Cmd(((UART_TypeDef*) 0x40038000), ENABLE);
  Setup_USB();

   
  key = GetKey();
  for (; key != SEL; key = GetKey())
  {
  }
  while((GetKey() == SEL)){};

   
  UART_Cmd(((UART_TypeDef*) 0x40038000), DISABLE);



  USB_CDC_ReceiveStop();
  USB_DevicePowerOff();

   
  ((RST_CLK_TypeDef*) 0x40020000)->CPU_CLOCK   = temp_CPU_CLOCK;
  ((RST_CLK_TypeDef*) 0x40020000)->PLL_CONTROL = temp_PLL_CONTROL;

  DisplayMenu();
}


   

   

   

   



 

