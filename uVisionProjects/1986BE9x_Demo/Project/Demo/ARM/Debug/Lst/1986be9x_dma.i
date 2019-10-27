#line 1 "..\\..\\..\\Library\\Libraries\\1986BE9x_StdPeriph_Driver\\src\\1986be9x_dma.c"




















 

 
#line 1 "..\\..\\..\\Library\\Libraries\\CMSIS\\CM3\\DeviceSupport\\1986BE9x\\inc\\1986BE9x.h"





















 

 





 



 



 



 

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


   






   



 

#line 82 "..\\..\\..\\Library\\Libraries\\CMSIS\\CM3\\DeviceSupport\\1986BE9x\\inc\\1986BE9x.h"

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

   

   



 



 

















   

   



 



 












































   

   



 



 




















   

   



 



 

















   

   



 



 








   

   



 



 




















   

   



 



 














   

   



 



 














   

   



 



 








   

   



 



 




















   

   



 



 














   

   



 



 














   

   



 



 


























   

   



 



 








   

   



 



 




















   

   



 



 














   

   



 



 














   

   

   

   

   





 
#line 84 "..\\..\\..\\Library\\Libraries\\CMSIS\\CM3\\DeviceSupport\\1986BE9x\\inc\\1986BE9x.h"
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

   

   



 



 














   

   



 



 














   

   



 



 














   

   



 



 














   

   



 



 


























   

   



 



 

















   

   



 



 


























   

   



 



 

















   

   



 



 

















   

   



 



 

















   

   



 



 


























   

   



 



 








   

   

   

   

   





 

#line 85 "..\\..\\..\\Library\\Libraries\\CMSIS\\CM3\\DeviceSupport\\1986BE9x\\inc\\1986BE9x.h"
#line 1 "..\\..\\..\\Library\\Libraries\\CMSIS\\CM3\\DeviceSupport\\1986BE9x\\inc\\1986BE9x_eeprom_defs.h"






















 

 





 



 



 



 



 

typedef struct
{
  volatile uint32_t CMD;
  volatile uint32_t ADR;
  volatile uint32_t DI;
  volatile uint32_t DO;
  volatile uint32_t KEY;
}EEPROM_TypeDef;

   

   



 



 






































   

   

   

   

   





 
#line 86 "..\\..\\..\\Library\\Libraries\\CMSIS\\CM3\\DeviceSupport\\1986BE9x\\inc\\1986BE9x.h"
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

   

   



 



 











   

   



 



 




















   

   



 



 








   

   



 



 














   

   



 



 














   

   



 



 

















   

   



 



 














   

   



 



 














   

   



 



 




















   

   



 



 














   

   



 



 














   

   

   

   

   





 
#line 87 "..\\..\\..\\Library\\Libraries\\CMSIS\\CM3\\DeviceSupport\\1986BE9x\\inc\\1986BE9x.h"
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

   

   



 



 














   

   



 



 








   

   

   

   

   





 
#line 88 "..\\..\\..\\Library\\Libraries\\CMSIS\\CM3\\DeviceSupport\\1986BE9x\\inc\\1986BE9x.h"
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

   

   



 



 

















   

   



 



 














   

   



 



 





























   

   



 



 























   

   



 



 






































   

   



 



 








   

   



 



 



































   

   



 



 



































   

   



 



 



































   

   



 



 



































   

   



 



 











   

   

   

   

   





 
#line 89 "..\\..\\..\\Library\\Libraries\\CMSIS\\CM3\\DeviceSupport\\1986BE9x\\inc\\1986BE9x.h"
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

   

   



 



 

















   

   



 



 














   

   



 



 

















   

   



 



 














   

   



 



 














   

   



 



 














   

   



 



 








   

   



 



 








   

   

   

   

   





 
#line 90 "..\\..\\..\\Library\\Libraries\\CMSIS\\CM3\\DeviceSupport\\1986BE9x\\inc\\1986BE9x.h"
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

   

   



 



 











   

   



 



 

















   

   



 



 




















   

   

   

   

   





 
#line 91 "..\\..\\..\\Library\\Libraries\\CMSIS\\CM3\\DeviceSupport\\1986BE9x\\inc\\1986BE9x.h"
#line 1 "..\\..\\..\\Library\\Libraries\\CMSIS\\CM3\\DeviceSupport\\1986BE9x\\inc\\1986BE9x_power_defs.h"






















 

 





 



 



 



 



 

typedef struct
{
  volatile uint32_t PVDCS;
}POWER_TypeDef;

   

   



 



 





























   

   

   

   

   





 
#line 92 "..\\..\\..\\Library\\Libraries\\CMSIS\\CM3\\DeviceSupport\\1986BE9x\\inc\\1986BE9x.h"
#line 1 "..\\..\\..\\Library\\Libraries\\CMSIS\\CM3\\DeviceSupport\\1986BE9x\\inc\\1986BE9x_wwdg_defs.h"






















 

 





 



 



 



 



 

typedef struct
{
  volatile uint32_t CR;
  volatile uint32_t CFR;
  volatile uint32_t SR;
}WWDG_TypeDef;

   

   



 



 








   

   



 



 











   

   

   

   

   





 
#line 93 "..\\..\\..\\Library\\Libraries\\CMSIS\\CM3\\DeviceSupport\\1986BE9x\\inc\\1986BE9x.h"
#line 1 "..\\..\\..\\Library\\Libraries\\CMSIS\\CM3\\DeviceSupport\\1986BE9x\\inc\\1986BE9x_iwdg_defs.h"






















 

 





 



 



 



 



 

typedef struct
{
  volatile uint32_t KR;
  volatile uint32_t PR;
  volatile uint32_t RLR;
  volatile uint32_t SR;
}IWDG_TypeDef;

   

   



 



 








   

   

   

   

   





 
#line 94 "..\\..\\..\\Library\\Libraries\\CMSIS\\CM3\\DeviceSupport\\1986BE9x\\inc\\1986BE9x.h"
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

   

   



 



 























   

   



 



 





























   

   



 



 




















   

   



 



 











   

   



 



 














   

   



 



 


























   

   



 



 


























   

   



 



 


























   

   



 



 











   

   

   

   

   





 
#line 95 "..\\..\\..\\Library\\Libraries\\CMSIS\\CM3\\DeviceSupport\\1986BE9x\\inc\\1986BE9x.h"
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

   

   



 



 





















































   

   



 



 






































   

   



 



 








   

   



 



 








   

   



 



 

















   

   



 



 

















   

   

   

   

   





 
#line 96 "..\\..\\..\\Library\\Libraries\\CMSIS\\CM3\\DeviceSupport\\1986BE9x\\inc\\1986BE9x.h"
#line 1 "..\\..\\..\\Library\\Libraries\\CMSIS\\CM3\\DeviceSupport\\1986BE9x\\inc\\1986BE9x_dac_defs.h"






















 

 





 



 



 



 



 

typedef struct
{
  volatile uint32_t CFG;
  volatile uint32_t DAC1_DATA;
  volatile uint32_t DAC2_DATA;
}DAC_TypeDef;

   

   



 



 

















   

   



 



 








   

   



 



 








   

   

   

   

   





 
#line 97 "..\\..\\..\\Library\\Libraries\\CMSIS\\CM3\\DeviceSupport\\1986BE9x\\inc\\1986BE9x.h"
#line 1 "..\\..\\..\\Library\\Libraries\\CMSIS\\CM3\\DeviceSupport\\1986BE9x\\inc\\1986BE9x_comp_defs.h"






















 

 





 



 



 



 



 

typedef struct
{
  volatile uint32_t CFG;
  volatile uint32_t RESULT;
  volatile uint32_t RESULT_LATCH;
}COMP_TypeDef;

   

   



 



 
































   

   



 



 











   

   

   

   

   





 
#line 98 "..\\..\\..\\Library\\Libraries\\CMSIS\\CM3\\DeviceSupport\\1986BE9x\\inc\\1986BE9x.h"
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

   

   



 



 


















































   

   



 



 


















































   

   

   

   

   





 
#line 99 "..\\..\\..\\Library\\Libraries\\CMSIS\\CM3\\DeviceSupport\\1986BE9x\\inc\\1986BE9x.h"
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

   

   



 



 























   

   



 



 















































   

   



 



 























   

   

   

   

   





 
#line 100 "..\\..\\..\\Library\\Libraries\\CMSIS\\CM3\\DeviceSupport\\1986BE9x\\inc\\1986BE9x.h"
#line 1 "..\\..\\..\\Library\\Libraries\\CMSIS\\CM3\\DeviceSupport\\1986BE9x\\inc\\1986BE9x_ext_bus_defs.h"






















 

 





 



 



 



 



 

typedef struct
{
       uint32_t RESERVED0[20];
  volatile uint32_t NAND_CYCLES;
  volatile uint32_t CONTROL;
}EXT_BUS_TypeDef;

   

   



 



 























   

   



 



 




















   

   

   

   

   





 
#line 101 "..\\..\\..\\Library\\Libraries\\CMSIS\\CM3\\DeviceSupport\\1986BE9x\\inc\\1986BE9x.h"

typedef enum {RESET = 0, SET = !RESET} FlagStatus, ITStatus;

typedef enum {DISABLE = 0, ENABLE = !DISABLE} FunctionalState;


typedef enum {ERROR = 0, SUCCESS = !ERROR} ErrorStatus;

   




 

#line 144 "..\\..\\..\\Library\\Libraries\\CMSIS\\CM3\\DeviceSupport\\1986BE9x\\inc\\1986BE9x.h"

   



 

#line 179 "..\\..\\..\\Library\\Libraries\\CMSIS\\CM3\\DeviceSupport\\1986BE9x\\inc\\1986BE9x.h"

   

   

   





 
#line 25 "..\\..\\..\\Library\\Libraries\\1986BE9x_StdPeriph_Driver\\src\\1986be9x_dma.c"
#line 1 "..\\1986BE9x_config.h"

















 

 



#line 1 "..\\..\\..\\Library\\Libraries\\1986BE9x_StdPeriph_Driver\\inc\\1986BE9x_lib.h"





















 

 



#line 48 "..\\..\\..\\Library\\Libraries\\1986BE9x_StdPeriph_Driver\\inc\\1986BE9x_lib.h"



 






 

#line 25 "..\\1986BE9x_config.h"
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
















   



   








   



   



   






   


















   

   
 
   

#line 26 "..\\1986BE9x_config.h"


 

 

 
 





 
#line 46 "..\\1986BE9x_config.h"



 
 


 


 
            


 

 


 
 
 
 


 


 
 

 
 

 
 

 
 


 







 

 
 








 
#line 119 "..\\1986BE9x_config.h"





 

#line 26 "..\\..\\..\\Library\\Libraries\\1986BE9x_StdPeriph_Driver\\src\\1986be9x_dma.c"
#line 1 "..\\..\\..\\Library\\Libraries\\1986BE9x_StdPeriph_Driver\\inc\\1986BE9x_dma.h"





















 

 



 
#line 30 "..\\..\\..\\Library\\Libraries\\1986BE9x_StdPeriph_Driver\\inc\\1986BE9x_dma.h"



 



 



 



 

typedef struct
{
  uint32_t DMA_SourceBaseAddr;       

  uint32_t DMA_DestBaseAddr;         

  uint32_t DMA_SourceIncSize;       
 

  uint32_t DMA_DestIncSize;         
 

  uint32_t DMA_MemoryDataSize;      
 

  uint32_t DMA_Mode;                
 

  uint32_t DMA_CycleSize;           
 

  uint32_t DMA_NumContinuous;       
 

  uint32_t DMA_SourceProtCtrl;      
 

  uint32_t DMA_DestProtCtrl;        
 
} DMA_CtrlDataInitTypeDef;



 

typedef struct
{
  uint32_t DMA_SourceEndAddr;        

  uint32_t DMA_DestEndAddr;          

  uint32_t DMA_Control;              

  uint32_t DMA_Unused;               

}DMA_CtrlDataTypeDef;



 

typedef struct
{
  DMA_CtrlDataInitTypeDef *DMA_PriCtrlData;  
 

  DMA_CtrlDataInitTypeDef *DMA_AltCtrlData;  
 

  uint32_t DMA_ProtCtrl;            
 

  uint8_t DMA_Priority;             
 

  uint8_t DMA_UseBurst;              
 

  uint8_t DMA_SelectDataStructure;  
 

} DMA_ChannelInitTypeDef;



 

typedef struct
{
  DMA_CtrlDataTypeDef *DMA_SG_TaskArray;  




 

  uint32_t DMA_SG_TaskNumber;        


  uint32_t DMA_SourceProtCtrl;      
 

  uint32_t DMA_DestProtCtrl;        
 

  uint32_t DMA_ProtCtrl;            
 

  uint8_t DMA_Priority;             
 

  uint8_t DMA_UseBurst;             
 

} DMA_Channel_SG_InitTypeDef;


   



 



 



   



 







   



 
#line 216 "..\\..\\..\\Library\\Libraries\\1986BE9x_StdPeriph_Driver\\inc\\1986BE9x_dma.h"

   



 

#line 231 "..\\..\\..\\Library\\Libraries\\1986BE9x_StdPeriph_Driver\\inc\\1986BE9x_dma.h"

   



 

#line 246 "..\\..\\..\\Library\\Libraries\\1986BE9x_StdPeriph_Driver\\inc\\1986BE9x_dma.h"

   



 

#line 259 "..\\..\\..\\Library\\Libraries\\1986BE9x_StdPeriph_Driver\\inc\\1986BE9x_dma.h"

   



 

#line 275 "..\\..\\..\\Library\\Libraries\\1986BE9x_StdPeriph_Driver\\inc\\1986BE9x_dma.h"

#line 285 "..\\..\\..\\Library\\Libraries\\1986BE9x_StdPeriph_Driver\\inc\\1986BE9x_dma.h"

   



 



   



 






   



 






   




 







   



 







   



 







   



 

#line 370 "..\\..\\..\\Library\\Libraries\\1986BE9x_StdPeriph_Driver\\inc\\1986BE9x_dma.h"

#line 382 "..\\..\\..\\Library\\Libraries\\1986BE9x_StdPeriph_Driver\\inc\\1986BE9x_dma.h"

   



 













   



 

#line 415 "..\\..\\..\\Library\\Libraries\\1986BE9x_StdPeriph_Driver\\inc\\1986BE9x_dma.h"

#line 424 "..\\..\\..\\Library\\Libraries\\1986BE9x_StdPeriph_Driver\\inc\\1986BE9x_dma.h"

   

   



 

   



 

void DMA_DeInit(void);
void DMA_Init(uint8_t DMA_Channel, DMA_ChannelInitTypeDef* DMA_InitStruct);
void DMA_SG_Init( uint8_t DMA_Channel, DMA_Channel_SG_InitTypeDef *DMA_SG_InitStruct);
void DMA_StructInit(DMA_ChannelInitTypeDef* DMA_InitStruct);
void DMA_SG_StructInit(DMA_Channel_SG_InitTypeDef* DMA_InitStruct);
void DMA_CtrlInit(uint8_t DMA_Channel, uint8_t DMA_CtrlDataType,  DMA_CtrlDataInitTypeDef* DMA_CtrlStruct);
void DMA_SG_CtrlInit(uint32_t DMA_Task, DMA_CtrlDataTypeDef *DMA_SG_TaskArray,  DMA_CtrlDataInitTypeDef* DMA_CtrlStruct);
void DMA_Cmd(uint8_t DMA_Channel, FunctionalState NewState);
void DMA_Request(uint8_t DMA_Channel);
void DMA_ClearError(void);
uint32_t DMA_GetCurrTransferCounter(uint8_t DMA_Channel, uint8_t DMA_CtrlData);
FlagStatus DMA_GetFlagStatus(uint8_t DMA_Channel, uint8_t DMA_Flag);

   

   

   





 

#line 27 "..\\..\\..\\Library\\Libraries\\1986BE9x_StdPeriph_Driver\\src\\1986be9x_dma.c"





 



 



 



   



 



 

 
#line 68 "..\\..\\..\\Library\\Libraries\\1986BE9x_StdPeriph_Driver\\src\\1986be9x_dma.c"

#line 76 "..\\..\\..\\Library\\Libraries\\1986BE9x_StdPeriph_Driver\\src\\1986be9x_dma.c"
	DMA_CtrlDataTypeDef DMA_ControlTable[32 * (1 + 1)] __attribute__ ((aligned (1024)));



   



 

void DMA_CtrlDataInit(DMA_CtrlDataInitTypeDef *DMA_ctrl_data_ptr, DMA_CtrlDataTypeDef *DMA_ctrl_table_ptr);

   



 









 
void DMA_CtrlDataInit(DMA_CtrlDataInitTypeDef *DMA_ctrl_data_ptr, DMA_CtrlDataTypeDef *DMA_ctrl_table_ptr)
{
   
  ((void)0);
  ((void)0);

   
  ((void)0);
  ((void)0);
  ((void)0);
  ((void)0);
  ((void)0);
  ((void)0);
  ((void)0);
  ((void)0);

   
  if (DMA_ctrl_data_ptr->DMA_SourceIncSize == ((uint32_t)(3 << 26)))
  {
    DMA_ctrl_table_ptr->DMA_SourceEndAddr = DMA_ctrl_data_ptr->DMA_SourceBaseAddr;
  }
  else
  {
    DMA_ctrl_table_ptr->DMA_SourceEndAddr = ( DMA_ctrl_data_ptr->DMA_SourceBaseAddr +
        ((DMA_ctrl_data_ptr->DMA_CycleSize - 1) << (DMA_ctrl_data_ptr->DMA_SourceIncSize >> 26)));
  }

   
  if (DMA_ctrl_data_ptr->DMA_Mode == ((uint32_t)0x4))
  {
     
    DMA_ctrl_table_ptr->DMA_DestEndAddr = (DMA_ctrl_data_ptr->DMA_DestBaseAddr + 12);
  }
  else
  {
     
    if (DMA_ctrl_data_ptr->DMA_DestIncSize == ((uint32_t)(0x3UL << 30)))
    {
      DMA_ctrl_table_ptr->DMA_DestEndAddr = DMA_ctrl_data_ptr->DMA_DestBaseAddr;
    }
    else
    {
      DMA_ctrl_table_ptr->DMA_DestEndAddr = ( DMA_ctrl_data_ptr->DMA_DestBaseAddr +
        ((DMA_ctrl_data_ptr->DMA_CycleSize - 1) << (DMA_ctrl_data_ptr->DMA_DestIncSize >> 30)));
    }
  }

   
  DMA_ctrl_table_ptr->DMA_Control = (DMA_ctrl_data_ptr->DMA_DestIncSize     |
                                     DMA_ctrl_data_ptr->DMA_MemoryDataSize  |
                                     DMA_ctrl_data_ptr->DMA_SourceIncSize   |
                                     DMA_ctrl_data_ptr->DMA_DestProtCtrl    |
                                     DMA_ctrl_data_ptr->DMA_SourceProtCtrl  |
                                     DMA_ctrl_data_ptr->DMA_NumContinuous   |
                                     ((DMA_ctrl_data_ptr->DMA_CycleSize - 1) << 4)  |
                                     DMA_ctrl_data_ptr->DMA_Mode);
}





 
void DMA_DeInit(void)
{
  ((DMA_TypeDef*) 0x40028000)->CFG = 0;                            
  ((DMA_TypeDef*) 0x40028000)->CTRL_BASE_PTR = 0;                  
  ((DMA_TypeDef*) 0x40028000)->CHNL_SW_REQUEST = 0;                
  ((DMA_TypeDef*) 0x40028000)->CHNL_USEBURST_CLR = 0xFFFFFFFF;     
  ((DMA_TypeDef*) 0x40028000)->CHNL_REQ_MASK_CLR = 0xFFFFFFFF;     
  ((DMA_TypeDef*) 0x40028000)->CHNL_ENABLE_CLR = 0xFFFFFFFF;       
  ((DMA_TypeDef*) 0x40028000)->CHNL_PRI_ALT_CLR = 0xFFFFFFFF;      
  ((DMA_TypeDef*) 0x40028000)->CHNL_PRIORITY_CLR = 0xFFFFFFFF;     
  ((DMA_TypeDef*) 0x40028000)->ERR_CLR = 0x01;                     
}










 
void DMA_CtrlInit(uint8_t DMA_Channel, uint8_t DMA_CtrlDataType,  DMA_CtrlDataInitTypeDef* DMA_CtrlStruct)
{
   
  if (DMA_CtrlDataType == ((uint8_t)0x00))
  {
    DMA_CtrlDataInit(DMA_CtrlStruct, &DMA_ControlTable[DMA_Channel]);
  }

   
  else
  {
    uint32_t ptr = (((DMA_TypeDef*) 0x40028000)->ALT_CTRL_BASE_PTR + (DMA_Channel * sizeof(DMA_CtrlDataTypeDef)));
    DMA_CtrlDataInit(DMA_CtrlStruct, (DMA_CtrlDataTypeDef *)ptr);
  }

}










 
void DMA_SG_CtrlInit(uint32_t DMA_Task, DMA_CtrlDataTypeDef *DMA_SG_TaskArray,  DMA_CtrlDataInitTypeDef* DMA_CtrlStruct)
{
  DMA_CtrlDataInit(DMA_CtrlStruct, &DMA_SG_TaskArray[DMA_Task]);
}







 
void DMA_SG_Init( uint8_t DMA_Channel, DMA_Channel_SG_InitTypeDef *DMA_SG_InitStruct)
{
  DMA_CtrlDataInitTypeDef DMA_PriCtrlData;

   
  ((void)0);
  ((void)0);
  ((void)0);
  ((void)0);
  ((void)0);
  ((void)0);
  ((void)0);
  ((void)0);
  ((void)0);

   
  ((DMA_TypeDef*) 0x40028000)->CTRL_BASE_PTR = (uint32_t)DMA_ControlTable;

   
  DMA_PriCtrlData.DMA_SourceBaseAddr = (uint32_t)(DMA_SG_InitStruct->DMA_SG_TaskArray);
  DMA_PriCtrlData.DMA_DestBaseAddr = (((DMA_TypeDef*) 0x40028000)->ALT_CTRL_BASE_PTR + (DMA_Channel * sizeof(DMA_CtrlDataTypeDef)));
  DMA_PriCtrlData.DMA_SourceIncSize = ((uint32_t)(2 << 26));
  DMA_PriCtrlData.DMA_DestIncSize = ((uint32_t)(0x2UL << 30));
  DMA_PriCtrlData.DMA_MemoryDataSize = ((uint32_t)(0x22 << 24));
  DMA_PriCtrlData.DMA_Mode = ((uint32_t)0x4);
  DMA_PriCtrlData.DMA_CycleSize = DMA_SG_InitStruct->DMA_SG_TaskNumber << 2;
  DMA_PriCtrlData.DMA_NumContinuous = ((uint32_t)(0x02 << 14));
  DMA_PriCtrlData.DMA_SourceProtCtrl = DMA_SG_InitStruct->DMA_SourceProtCtrl;
  DMA_PriCtrlData.DMA_DestProtCtrl = DMA_SG_InitStruct->DMA_DestProtCtrl;

  DMA_CtrlDataInit(&DMA_PriCtrlData, &DMA_ControlTable[DMA_Channel]);

   
  ((DMA_TypeDef*) 0x40028000)->CFG = ((uint32_t)0x00000001) || DMA_SG_InitStruct->DMA_ProtCtrl;

   
  if (DMA_SG_InitStruct->DMA_UseBurst == ((uint8_t)0x01))
  {
    ((DMA_TypeDef*) 0x40028000)->CHNL_USEBURST_SET = (1 << DMA_Channel);
  }
  else
  {
    ((DMA_TypeDef*) 0x40028000)->CHNL_USEBURST_CLR = (1 << DMA_Channel);
  }

   
  ((DMA_TypeDef*) 0x40028000)->CHNL_REQ_MASK_CLR = (1 << DMA_Channel);

   
  ((DMA_TypeDef*) 0x40028000)->CHNL_ENABLE_SET = (1 << DMA_Channel);

   
  ((DMA_TypeDef*) 0x40028000)->CHNL_PRI_ALT_CLR = (1 << DMA_Channel);        

   
  if (DMA_SG_InitStruct->DMA_Priority == ((uint8_t)0x01))
  {
    ((DMA_TypeDef*) 0x40028000)->CHNL_PRIORITY_SET = (1 << DMA_Channel);       
  }
  else
  {
    ((DMA_TypeDef*) 0x40028000)->CHNL_PRIORITY_CLR = (1 << DMA_Channel);       
  }
}








 
void DMA_Init(uint8_t DMA_Channel, DMA_ChannelInitTypeDef* DMA_InitStruct)
{
   
  ((void)0);
  ((void)0);
  ((void)0);
  ((void)0);
  ((void)0);

   
  ((DMA_TypeDef*) 0x40028000)->CTRL_BASE_PTR = (uint32_t)DMA_ControlTable;

   
  if (DMA_InitStruct->DMA_PriCtrlData)
  {
    DMA_CtrlDataInit(DMA_InitStruct->DMA_PriCtrlData, &DMA_ControlTable[DMA_Channel]);
  }


   
  if (DMA_InitStruct->DMA_AltCtrlData)
  {
    uint32_t ptr = (((DMA_TypeDef*) 0x40028000)->ALT_CTRL_BASE_PTR + (DMA_Channel * sizeof(DMA_CtrlDataTypeDef)));
    DMA_CtrlDataInit(DMA_InitStruct->DMA_AltCtrlData, (DMA_CtrlDataTypeDef *)ptr);
  }


   
  ((DMA_TypeDef*) 0x40028000)->CFG = ((uint32_t)0x00000001) || DMA_InitStruct->DMA_ProtCtrl;

   
  if (DMA_InitStruct->DMA_UseBurst == ((uint8_t)0x01))
  {
    ((DMA_TypeDef*) 0x40028000)->CHNL_USEBURST_SET = (1 << DMA_Channel);
  }
  else
  {
    ((DMA_TypeDef*) 0x40028000)->CHNL_USEBURST_CLR = (1 << DMA_Channel);
  }

   
  ((DMA_TypeDef*) 0x40028000)->CHNL_REQ_MASK_CLR = (1 << DMA_Channel);

   
  ((DMA_TypeDef*) 0x40028000)->CHNL_ENABLE_SET = (1 << DMA_Channel);

   
  if (DMA_InitStruct->DMA_SelectDataStructure == ((uint8_t)0x00))
  {
    ((DMA_TypeDef*) 0x40028000)->CHNL_PRI_ALT_CLR = (1 << DMA_Channel);        
  }
  else
  {
    ((DMA_TypeDef*) 0x40028000)->CHNL_PRI_ALT_SET = (1 << DMA_Channel);        
  }

   
  if (DMA_InitStruct->DMA_Priority == ((uint8_t)0x01))
  {
    ((DMA_TypeDef*) 0x40028000)->CHNL_PRIORITY_SET = (1 << DMA_Channel);       
  }
  else
  {
    ((DMA_TypeDef*) 0x40028000)->CHNL_PRIORITY_CLR = (1 << DMA_Channel);       
  }
}






 
void DMA_StructInit(DMA_ChannelInitTypeDef* DMA_InitStruct)
{
  DMA_InitStruct->DMA_PriCtrlData = 0;
  DMA_InitStruct->DMA_AltCtrlData = 0;
  DMA_InitStruct->DMA_ProtCtrl = 0;
  DMA_InitStruct->DMA_Priority = 0;
  DMA_InitStruct->DMA_UseBurst = 0;
  DMA_InitStruct->DMA_SelectDataStructure = 0;
}






 
void DMA_SG_StructInit(DMA_Channel_SG_InitTypeDef* DMA_SG_InitStruct)
{
  DMA_SG_InitStruct->DMA_SG_TaskArray = 0;
  DMA_SG_InitStruct->DMA_SG_TaskNumber = 0;
  DMA_SG_InitStruct->DMA_SourceProtCtrl = 0;
  DMA_SG_InitStruct->DMA_DestProtCtrl = 0;
  DMA_SG_InitStruct->DMA_ProtCtrl = 0;
  DMA_SG_InitStruct->DMA_Priority = 0;
  DMA_SG_InitStruct->DMA_UseBurst = 0;
}







 
void DMA_Cmd(uint8_t DMA_Channel, FunctionalState NewState)
{
   
  ((void)0);

   
  if ( NewState == ENABLE)
  {
    ((DMA_TypeDef*) 0x40028000)->CHNL_ENABLE_SET = (1 << DMA_Channel);
  }
  else
  {
    ((DMA_TypeDef*) 0x40028000)->CHNL_ENABLE_CLR = (1 << DMA_Channel);
  }
}





 
void DMA_Request(uint8_t DMA_Channel)
{
   
  ((void)0);

   
  ((DMA_TypeDef*) 0x40028000)->CHNL_SW_REQUEST = (1 << DMA_Channel);
}





 
void DMA_ClearError(void)
{
  ((DMA_TypeDef*) 0x40028000)->ERR_CLR = 0x01;             
}









 
uint32_t DMA_GetCurrTransferCounter(uint8_t DMA_Channel, uint8_t DMA_CtrlData)
{
  DMA_CtrlDataTypeDef *ptr;      

   
  ((void)0);
  ((void)0);

   
  if (DMA_CtrlData == ((uint8_t)0x00))
  {
    ptr = (DMA_CtrlDataTypeDef *)(((DMA_TypeDef*) 0x40028000)->CTRL_BASE_PTR + (DMA_Channel * sizeof(DMA_CtrlDataTypeDef)));
  }
  else
  {
    ptr = (DMA_CtrlDataTypeDef *)(((DMA_TypeDef*) 0x40028000)->ALT_CTRL_BASE_PTR + (DMA_Channel * sizeof(DMA_CtrlDataTypeDef)));
  }

   
  return ((ptr->DMA_Control & ((uint32_t)0x3FF0)) + 1);
}















 
FlagStatus DMA_GetFlagStatus(uint8_t DMA_Channel, uint8_t DMA_Flag)
{
   
  ((void)0);
  ((void)0);

   
  switch(DMA_Flag)
  {
    case ((uint8_t)0x01):
      return (FlagStatus)(((DMA_TypeDef*) 0x40028000)->STATUS & ((uint32_t)0x00000001));
    case ((uint8_t)0x02):
      return (FlagStatus)(((DMA_TypeDef*) 0x40028000)->ERR_CLR & 0x01);
    case ((uint8_t)0x03):
      return (FlagStatus)(((DMA_TypeDef*) 0x40028000)->CHNL_ENABLE_SET & (1 << DMA_Channel));
    case ((uint8_t)0x04):
      return (FlagStatus)(((DMA_TypeDef*) 0x40028000)->CHNL_REQ_MASK_SET & (1 << DMA_Channel));
    case ((uint8_t)0x05):
      return (FlagStatus)(((DMA_TypeDef*) 0x40028000)->WAITONREQ_STATUS & (1 << DMA_Channel));
    case ((uint8_t)0x06):
      return (FlagStatus)(((DMA_TypeDef*) 0x40028000)->CHNL_USEBURST_SET & (1 << DMA_Channel));
    case ((uint8_t)0x07):
      return (FlagStatus)(((DMA_TypeDef*) 0x40028000)->CHNL_PRI_ALT_SET & (1 << DMA_Channel));
    case ((uint8_t)0x08):
      return (FlagStatus)(((DMA_TypeDef*) 0x40028000)->CHNL_PRIORITY_SET & (1 << DMA_Channel));
    default:
      return (FlagStatus)0;
  }
}

   

   

   



 
