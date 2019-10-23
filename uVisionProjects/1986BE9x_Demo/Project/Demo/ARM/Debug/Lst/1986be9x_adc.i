#line 1 "..\\..\\..\\Library\\Libraries\\1986BE9x_StdPeriph_Driver\\src\\1986be9x_adc.c"




















 

 
#line 1 "..\\..\\..\\Library\\Libraries\\1986BE9x_StdPeriph_Driver\\inc\\1986BE9x_adc.h"





















 

 



 
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

   

   

   





 
#line 30 "..\\..\\..\\Library\\Libraries\\1986BE9x_StdPeriph_Driver\\inc\\1986BE9x_adc.h"
#line 1 "..\\..\\..\\Library\\Libraries\\1986BE9x_StdPeriph_Driver\\inc\\1986BE9x_lib.h"





















 

 



#line 48 "..\\..\\..\\Library\\Libraries\\1986BE9x_StdPeriph_Driver\\inc\\1986BE9x_lib.h"



 






 

#line 31 "..\\..\\..\\Library\\Libraries\\1986BE9x_StdPeriph_Driver\\inc\\1986BE9x_adc.h"



 



 



 



 

typedef struct
{
  uint32_t ADC_SynchronousMode;         
 

  uint32_t ADC_StartDelay;              
 

  uint32_t ADC_TempSensor;              
 

  uint32_t ADC_TempSensorAmplifier;     

 

  uint32_t ADC_TempSensorConversion;    
 

  uint32_t ADC_IntVRefConversion;       
 

  uint32_t ADC_IntVRefTrimming;         
 
}ADC_InitTypeDef;



 

typedef struct
{
  uint32_t ADC_ClockSource;             
 

  uint32_t ADC_SamplingMode;            
 

  uint32_t ADC_ChannelSwitching;        
 

  uint32_t ADC_ChannelNumber;           
 

  uint32_t ADC_Channels;                
 

  uint32_t ADC_LevelControl;            
 

  uint16_t ADC_LowLevel;                
 

  uint16_t ADC_HighLevel;                
 

  uint32_t ADC_VRefSource;              
 

  uint32_t ADC_IntVRefSource;           
 

  uint32_t ADC_Prescaler;               
 

  uint32_t ADC_DelayGo;                 
 
}ADCx_InitTypeDef;

   



 



 







   



 



   



 







   



 







   



 







   



 







   



 



   



 







   



 







   



 







   




 

#line 257 "..\\..\\..\\Library\\Libraries\\1986BE9x_StdPeriph_Driver\\inc\\1986BE9x_adc.h"

#line 274 "..\\..\\..\\Library\\Libraries\\1986BE9x_StdPeriph_Driver\\inc\\1986BE9x_adc.h"





   



 

#line 303 "..\\..\\..\\Library\\Libraries\\1986BE9x_StdPeriph_Driver\\inc\\1986BE9x_adc.h"

#line 320 "..\\..\\..\\Library\\Libraries\\1986BE9x_StdPeriph_Driver\\inc\\1986BE9x_adc.h"











   



 









   



 







   



 







   



 

#line 391 "..\\..\\..\\Library\\Libraries\\1986BE9x_StdPeriph_Driver\\inc\\1986BE9x_adc.h"

#line 408 "..\\..\\..\\Library\\Libraries\\1986BE9x_StdPeriph_Driver\\inc\\1986BE9x_adc.h"

   



 



   



 









#line 437 "..\\..\\..\\Library\\Libraries\\1986BE9x_StdPeriph_Driver\\inc\\1986BE9x_adc.h"

#line 444 "..\\..\\..\\Library\\Libraries\\1986BE9x_StdPeriph_Driver\\inc\\1986BE9x_adc.h"

   



 

















   

   



 

   



 

void ADC_DeInit(void);

void ADC_Init(const ADC_InitTypeDef* ADC_InitStruct);
void ADC_StructInit(ADC_InitTypeDef* ADC_InitStruct);

void ADC_SetTrim(uint32_t Trim);

void ADC1_Init(const ADCx_InitTypeDef* ADCx_InitStruct);
void ADC2_Init(const ADCx_InitTypeDef* ADCx_InitStruct);
void ADCx_StructInit(ADCx_InitTypeDef* ADCx_InitStruct);

void ADC1_Cmd(FunctionalState NewState);
void ADC2_Cmd(FunctionalState NewState);

void ADC1_SetChannel(uint32_t Channel);
void ADC2_SetChannel(uint32_t Channel);
void ADC1_SetChannels(uint32_t ChannelMask);
void ADC2_SetChannels(uint32_t ChannelMask);

void ADC1_OperationModeConfig(uint32_t SamplingMode, uint32_t SwitchingMode);
void ADC2_OperationModeConfig(uint32_t SamplingMode, uint32_t SwitchingMode);
void ADC1_SamplingModeConfig(uint32_t SamplingMode);
void ADC2_SamplingModeConfig(uint32_t SamplingMode);
void ADC1_ChannelSwithingConfig(uint32_t SwitchingMode);
void ADC2_ChannelSwithingConfig(uint32_t SwitchingMode);

void ADC1_LevelsConfig(uint32_t LowLevel, uint32_t HighLevel, uint32_t NewState);
void ADC2_LevelsConfig(uint32_t LowLevel, uint32_t HighLevel, uint32_t NewState);
void ADC1_SetLowLevel(uint32_t LowLevel);
void ADC2_SetLowLevel(uint32_t LowLevel);
void ADC1_SetHighLevel(uint32_t HighLevel);
void ADC2_SetHighLevel(uint32_t HighLevel);

void ADC1_Start(void);
void ADC2_Start(void);

uint32_t ADC1_GetResult(void);
uint32_t ADC2_GetResult(void);

uint32_t ADC_GetStatus(void);
uint32_t ADC1_GetStatus(void);
uint32_t ADC2_GetStatus(void);
FlagStatus ADC_GetFlagStatus(uint32_t Flag);
FlagStatus ADC1_GetFlagStatus(uint32_t Flag);
FlagStatus ADC2_GetFlagStatus(uint32_t Flag);
void ADC1_ClearOverwriteFlag(void);
void ADC2_ClearOverwriteFlag(void);
void ADC1_ClearOutOfRangeFlag(void);
void ADC2_ClearOutOfRangeFlag(void);

void ADC_ITConfig(uint32_t ADC_IT, FunctionalState NewState);
void ADC1_ITConfig(uint32_t ADC_IT, FunctionalState NewState);
void ADC2_ITConfig(uint32_t ADC_IT, FunctionalState NewState);
ITStatus ADC_GetITStatus(uint32_t ADC_IT);
ITStatus ADC1_GetITStatus(uint32_t ADC_IT);
ITStatus ADC2_GetITStatus(uint32_t ADC_IT);

   

   

   





 

#line 25 "..\\..\\..\\Library\\Libraries\\1986BE9x_StdPeriph_Driver\\src\\1986be9x_adc.c"
#line 1 "..\\1986BE9x_config.h"

















 

 



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





 

#line 26 "..\\..\\..\\Library\\Libraries\\1986BE9x_StdPeriph_Driver\\src\\1986be9x_adc.c"





 



 



 





 
void ADC_DeInit(void)
{
  ((ADC_TypeDef*) 0x40088000)->ADC1_CFG = 0;
  ((ADC_TypeDef*) 0x40088000)->ADC2_CFG = 0;
  ((ADC_TypeDef*) 0x40088000)->ADC1_H_LEVEL = 0;
  ((ADC_TypeDef*) 0x40088000)->ADC2_H_LEVEL = 0;
  ((ADC_TypeDef*) 0x40088000)->ADC1_L_LEVEL = 0;
  ((ADC_TypeDef*) 0x40088000)->ADC2_L_LEVEL = 0;
  ((ADC_TypeDef*) 0x40088000)->ADC1_RESULT;
  ((ADC_TypeDef*) 0x40088000)->ADC2_RESULT;
  ((ADC_TypeDef*) 0x40088000)->ADC1_STATUS = 0;
  ((ADC_TypeDef*) 0x40088000)->ADC2_STATUS = 0;
  ((ADC_TypeDef*) 0x40088000)->ADC1_CHSEL = 0;
  ((ADC_TypeDef*) 0x40088000)->ADC2_CHSEL = 0;
}








 
void ADC_Init(const ADC_InitTypeDef* ADC_InitStruct)
{
  uint32_t tmpreg_CFG;
  uint32_t tmpreg_MSK;

   
  ((void)0);
  ((void)0);
  ((void)0);
  ((void)0);
  ((void)0);
  ((void)0);
  ((void)0);

  tmpreg_CFG = ADC_InitStruct->ADC_SynchronousMode
             + (ADC_InitStruct->ADC_StartDelay << 28)
             + ADC_InitStruct->ADC_TempSensor
             + ADC_InitStruct->ADC_TempSensorAmplifier
             + ADC_InitStruct->ADC_TempSensorConversion
             + ADC_InitStruct->ADC_IntVRefConversion
             + (ADC_InitStruct->ADC_IntVRefTrimming << 21);

  tmpreg_MSK = ((uint32_t)0x00010000)
             | ((uint32_t)0xF0000000)
             | ((uint32_t)0x00020000)
             | ((uint32_t)0x00040000)
             | ((uint32_t)0x00080000)
             | ((uint32_t)0x00100000)
             | ((uint32_t)0x01E00000);

  ((ADC_TypeDef*) 0x40088000)->ADC1_CFG = (((ADC_TypeDef*) 0x40088000)->ADC1_CFG & ~tmpreg_MSK) + tmpreg_CFG;
}






 
void ADC_StructInit(ADC_InitTypeDef* ADC_InitStruct)
{
  ADC_InitStruct->ADC_SynchronousMode      = (((uint32_t)0x0) << 16);
  ADC_InitStruct->ADC_StartDelay           = 0;
  ADC_InitStruct->ADC_TempSensor           = (((uint32_t)0x0) << 17);
  ADC_InitStruct->ADC_TempSensorAmplifier  = (((uint32_t)0x0) << 18);
  ADC_InitStruct->ADC_TempSensorConversion = (((uint32_t)0x0) << 19);
  ADC_InitStruct->ADC_IntVRefConversion    = (((uint32_t)0x0) << 20);
  ADC_InitStruct->ADC_IntVRefTrimming      = 0;
}





 
void ADC_SetTrim(uint32_t Trim)
{
  uint32_t tmpreg_CFG;

   
  ((void)0);

  tmpreg_CFG = ((ADC_TypeDef*) 0x40088000)->ADC1_CFG & ~((uint32_t)0x01E00000);
  ((ADC_TypeDef*) 0x40088000)->ADC1_CFG = tmpreg_CFG + (Trim << 21);
}








 
void ADC1_Init(const ADCx_InitTypeDef* ADCx_InitStruct)
{
  uint32_t tmpreg_CFG1;
  uint32_t tmpreg_CFG2;

   
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
  ((void)0);
  ((void)0);

  tmpreg_CFG1 = ((ADC_TypeDef*) 0x40088000)->ADC1_CFG;

  tmpreg_CFG1 &= ~(((uint32_t)0x00000004)
                 | ((uint32_t)0x00000008)
                 | ((uint32_t)0x00000200)
                 | ((uint32_t)0x000001F0)
                 | ((uint32_t)0x00000400)
                 | ((uint32_t)0x00000800)
                 | ((uint32_t)0x0000F000)
                 | ((uint32_t)0x0E000000));

  tmpreg_CFG1 += ADCx_InitStruct->ADC_ClockSource
               + ADCx_InitStruct->ADC_SamplingMode
               + ADCx_InitStruct->ADC_ChannelSwitching
               + (ADCx_InitStruct->ADC_ChannelNumber << 4)
               + ADCx_InitStruct->ADC_LevelControl
               + ADCx_InitStruct->ADC_VRefSource
               + ADCx_InitStruct->ADC_Prescaler
               + (ADCx_InitStruct->ADC_DelayGo << 25);

  tmpreg_CFG2 = ((ADC_TypeDef*) 0x40088000)->ADC2_CFG;
  tmpreg_CFG2 &= ~((uint32_t)0x00020000);
  tmpreg_CFG2 += ADCx_InitStruct->ADC_IntVRefSource << 17;

  ((ADC_TypeDef*) 0x40088000)->ADC1_CFG = tmpreg_CFG1;
  ((ADC_TypeDef*) 0x40088000)->ADC2_CFG = tmpreg_CFG2;

  ((ADC_TypeDef*) 0x40088000)->ADC1_L_LEVEL = ADCx_InitStruct->ADC_LowLevel;
  ((ADC_TypeDef*) 0x40088000)->ADC1_H_LEVEL = ADCx_InitStruct->ADC_HighLevel;
  ((ADC_TypeDef*) 0x40088000)->ADC1_CHSEL   = ADCx_InitStruct->ADC_Channels;
}








 
void ADC2_Init(const ADCx_InitTypeDef* ADCx_InitStruct)
{
  uint32_t tmpreg_CFG2;

   
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
  ((void)0);
  ((void)0);

  tmpreg_CFG2 = ((ADC_TypeDef*) 0x40088000)->ADC2_CFG;

  tmpreg_CFG2 &= ~(((uint32_t)0x00000004)
                 | ((uint32_t)0x00000008)
                 | ((uint32_t)0x00000200)
                 | ((uint32_t)0x000001F0)
                 | ((uint32_t)0x00000400)
                 | ((uint32_t)0x00000800)
                 | ((uint32_t)0x00040000)
                 | ((uint32_t)0x0000F000)
                 | ((uint32_t)0x0E000000));

  tmpreg_CFG2 += ADCx_InitStruct->ADC_ClockSource
               + ADCx_InitStruct->ADC_SamplingMode
               + ADCx_InitStruct->ADC_ChannelSwitching
               + (ADCx_InitStruct->ADC_ChannelNumber << 4)
               + ADCx_InitStruct->ADC_LevelControl
               + ADCx_InitStruct->ADC_VRefSource
               + (ADCx_InitStruct->ADC_IntVRefSource << 18)
               + ADCx_InitStruct->ADC_Prescaler
               + (ADCx_InitStruct->ADC_DelayGo << 25);

  ((ADC_TypeDef*) 0x40088000)->ADC2_CFG = tmpreg_CFG2;
  ((ADC_TypeDef*) 0x40088000)->ADC2_L_LEVEL = ADCx_InitStruct->ADC_LowLevel;
  ((ADC_TypeDef*) 0x40088000)->ADC2_H_LEVEL = ADCx_InitStruct->ADC_HighLevel;
  ((ADC_TypeDef*) 0x40088000)->ADC2_CHSEL   = ADCx_InitStruct->ADC_Channels;
}






 
void ADCx_StructInit(ADCx_InitTypeDef* ADCx_InitStruct)
{
  ADCx_InitStruct->ADC_ClockSource      = (((uint32_t)0x0) << 2);
  ADCx_InitStruct->ADC_SamplingMode     = (((uint32_t)0x0) << 3);
  ADCx_InitStruct->ADC_ChannelSwitching = (((uint32_t)0x0) << 9);
  ADCx_InitStruct->ADC_ChannelNumber    = ((uint32_t)0x00);
  ADCx_InitStruct->ADC_Channels         = 0;
  ADCx_InitStruct->ADC_LevelControl     = (((uint32_t)0x0) << 10);
  ADCx_InitStruct->ADC_LowLevel         = 0;
  ADCx_InitStruct->ADC_HighLevel        = 0;
  ADCx_InitStruct->ADC_VRefSource       = (((uint32_t)0x0) << 11);
  ADCx_InitStruct->ADC_IntVRefSource    = ((uint32_t)0x0);
  ADCx_InitStruct->ADC_Prescaler        = (((uint32_t)0x0) << 12);
  ADCx_InitStruct->ADC_DelayGo          = 0;
}






 
void ADC1_Cmd(FunctionalState NewState)
{
  uint32_t tmpreg_CFG;

   
  ((void)0);

  tmpreg_CFG = ((ADC_TypeDef*) 0x40088000)->ADC1_CFG;

   
  if (NewState != DISABLE)
  {
     
    tmpreg_CFG |= ((uint32_t)0x00000001);
  }
  else
  {
     
    tmpreg_CFG &= ~((uint32_t)0x00000001);
  }

   
  ((ADC_TypeDef*) 0x40088000)->ADC1_CFG = tmpreg_CFG;
}






 
void ADC2_Cmd(FunctionalState NewState)
{
  uint32_t tmpreg_CFG;

   
  ((void)0);

  tmpreg_CFG = ((ADC_TypeDef*) 0x40088000)->ADC2_CFG;

   
  if (NewState != DISABLE)
  {
     
    tmpreg_CFG |= ((uint32_t)0x00000001);
  }
  else
  {
     
    tmpreg_CFG &= ~((uint32_t)0x00000001);
  }

   
  ((ADC_TypeDef*) 0x40088000)->ADC2_CFG = tmpreg_CFG;
}





 
void ADC1_SetChannel(uint32_t Channel)
{
  uint32_t tmpreg_CFG;

   
  ((void)0);

  tmpreg_CFG = ((ADC_TypeDef*) 0x40088000)->ADC1_CFG;
  tmpreg_CFG &= ~((uint32_t)0x000001F0);
  tmpreg_CFG += Channel << 4;
  ((ADC_TypeDef*) 0x40088000)->ADC1_CFG = tmpreg_CFG;
}





 
void ADC2_SetChannel(uint32_t Channel)
{
  uint32_t tmpreg_CFG;

   
  ((void)0);

  tmpreg_CFG = ((ADC_TypeDef*) 0x40088000)->ADC2_CFG;
  tmpreg_CFG &= ~((uint32_t)0x000001F0);
  tmpreg_CFG += Channel << 4;
  ((ADC_TypeDef*) 0x40088000)->ADC2_CFG = tmpreg_CFG;
}





 
void ADC1_SetChannels(uint32_t ChannelMask)
{
   
  ((void)0);

  ((ADC_TypeDef*) 0x40088000)->ADC1_CHSEL = ChannelMask;
}





 
void ADC2_SetChannels(uint32_t ChannelMask)
{
   
  ((void)0);

  ((ADC_TypeDef*) 0x40088000)->ADC2_CHSEL = ChannelMask;
}






 
void ADC1_OperationModeConfig(uint32_t SamplingMode, uint32_t SwitchingMode)
{
  uint32_t tmpreg_CFG;

   
  ((void)0);
  ((void)0);

  tmpreg_CFG = ((ADC_TypeDef*) 0x40088000)->ADC1_CFG;
  tmpreg_CFG &= ~(((uint32_t)0x00000008) | ((uint32_t)0x00000200));
  tmpreg_CFG += SamplingMode + SwitchingMode;
  ((ADC_TypeDef*) 0x40088000)->ADC1_CFG = tmpreg_CFG;
}






 
void ADC2_OperationModeConfig(uint32_t SamplingMode, uint32_t SwitchingMode)
{
  uint32_t tmpreg_CFG;

   
  ((void)0);
  ((void)0);

  tmpreg_CFG = ((ADC_TypeDef*) 0x40088000)->ADC2_CFG;
  tmpreg_CFG &= ~(((uint32_t)0x00000008) | ((uint32_t)0x00000200));
  tmpreg_CFG += SamplingMode + SwitchingMode;
  ((ADC_TypeDef*) 0x40088000)->ADC2_CFG = tmpreg_CFG;
}





 
void ADC1_SamplingModeConfig(uint32_t SamplingMode)
{
  uint32_t tmpreg_CFG;

   
  ((void)0);

  tmpreg_CFG = ((ADC_TypeDef*) 0x40088000)->ADC1_CFG;
  tmpreg_CFG &= ~((uint32_t)0x00000008);
  tmpreg_CFG += SamplingMode;
  ((ADC_TypeDef*) 0x40088000)->ADC1_CFG = tmpreg_CFG;
}





 
void ADC2_SamplingModeConfig(uint32_t SamplingMode)
{
  uint32_t tmpreg_CFG;

   
  ((void)0);

  tmpreg_CFG = ((ADC_TypeDef*) 0x40088000)->ADC2_CFG;
  tmpreg_CFG &= ~((uint32_t)0x00000008);
  tmpreg_CFG += SamplingMode;
  ((ADC_TypeDef*) 0x40088000)->ADC2_CFG = tmpreg_CFG;
}





 
void ADC1_ChannelSwithingConfig(uint32_t SwitchingMode)
{
  uint32_t tmpreg_CFG;

   
  ((void)0);

  tmpreg_CFG = ((ADC_TypeDef*) 0x40088000)->ADC1_CFG;
  tmpreg_CFG &= ~((uint32_t)0x00000200);
  tmpreg_CFG += SwitchingMode;
  ((ADC_TypeDef*) 0x40088000)->ADC1_CFG = tmpreg_CFG;
}





 
void ADC2_ChannelSwithingConfig(uint32_t SwitchingMode)
{
  uint32_t tmpreg_CFG;

   
  ((void)0);

  tmpreg_CFG = ((ADC_TypeDef*) 0x40088000)->ADC2_CFG;
  tmpreg_CFG &= ~((uint32_t)0x00000200);
  tmpreg_CFG += SwitchingMode;
  ((ADC_TypeDef*) 0x40088000)->ADC2_CFG = tmpreg_CFG;
}







 
void ADC1_LevelsConfig(uint32_t LowLevel, uint32_t HighLevel, uint32_t NewState)
{
  uint32_t tmpreg_CFG;

   
  ((void)0);
  ((void)0);
  ((void)0);

  tmpreg_CFG = ((ADC_TypeDef*) 0x40088000)->ADC1_CFG;
  tmpreg_CFG &= ~((uint32_t)0x00000400);
  tmpreg_CFG += NewState;
  ((ADC_TypeDef*) 0x40088000)->ADC1_CFG = tmpreg_CFG;

  ((ADC_TypeDef*) 0x40088000)->ADC1_L_LEVEL = LowLevel;
  ((ADC_TypeDef*) 0x40088000)->ADC1_H_LEVEL = HighLevel;
}







 
void ADC2_LevelsConfig(uint32_t LowLevel, uint32_t HighLevel, uint32_t NewState)
{
  uint32_t tmpreg_CFG;

   
  ((void)0);
  ((void)0);
  ((void)0);

  tmpreg_CFG = ((ADC_TypeDef*) 0x40088000)->ADC2_CFG;
  tmpreg_CFG &= ~((uint32_t)0x00000400);
  tmpreg_CFG += NewState;
  ((ADC_TypeDef*) 0x40088000)->ADC2_CFG = tmpreg_CFG;

  ((ADC_TypeDef*) 0x40088000)->ADC2_L_LEVEL = LowLevel;
  ((ADC_TypeDef*) 0x40088000)->ADC2_H_LEVEL = HighLevel;
}





 
void ADC1_SetLowLevel(uint32_t LowLevel)
{
   
  ((void)0);

  ((ADC_TypeDef*) 0x40088000)->ADC1_L_LEVEL = LowLevel;
}





 
void ADC2_SetLowLevel(uint32_t LowLevel)
{
   
  ((void)0);

  ((ADC_TypeDef*) 0x40088000)->ADC2_L_LEVEL = LowLevel;
}





 
void ADC1_SetHighLevel(uint32_t HighLevel)
{
   
  ((void)0);

  ((ADC_TypeDef*) 0x40088000)->ADC1_H_LEVEL = HighLevel;
}





 
void ADC2_SetHighLevel(uint32_t HighLevel)
{
   
  ((void)0);

  ((ADC_TypeDef*) 0x40088000)->ADC2_H_LEVEL = HighLevel;
}





 
void ADC1_Start(void)
{
  ((ADC_TypeDef*) 0x40088000)->ADC1_CFG |= ((uint32_t)0x00000002);
}





 
void ADC2_Start(void)
{
  ((ADC_TypeDef*) 0x40088000)->ADC2_CFG |= ((uint32_t)0x00000002);
}





 
uint32_t ADC1_GetResult(void)
{
  return ((ADC_TypeDef*) 0x40088000)->ADC1_RESULT;
}





 
uint32_t ADC2_GetResult(void)
{
  return ((ADC_TypeDef*) 0x40088000)->ADC2_RESULT;
}





 
uint32_t ADC_GetStatus(void)
{
  return ((ADC_TypeDef*) 0x40088000)->ADC1_STATUS + (((ADC_TypeDef*) 0x40088000)->ADC2_STATUS << 16);
}





 
uint32_t ADC1_GetStatus(void)
{
  return ((ADC_TypeDef*) 0x40088000)->ADC1_STATUS;
}





 
uint32_t ADC2_GetStatus(void)
{
  return ((ADC_TypeDef*) 0x40088000)->ADC2_STATUS;
}












 
FlagStatus ADC_GetFlagStatus(uint32_t Flag)
{
  FlagStatus bitstatus;

   
  ((void)0);

  if ((ADC_GetStatus() & Flag) == 0)
  {
    bitstatus = RESET;
  }
  else
  {
    bitstatus = SET;
  }

  return bitstatus;
}









 
FlagStatus ADC1_GetFlagStatus(uint32_t Flag)
{
  FlagStatus bitstatus;

   
  ((void)0);

  if ((((ADC_TypeDef*) 0x40088000)->ADC1_STATUS & Flag) == 0)
  {
    bitstatus = RESET;
  }
  else
  {
    bitstatus = SET;
  }

  return bitstatus;
}









 
FlagStatus ADC2_GetFlagStatus(uint32_t Flag)
{
  FlagStatus bitstatus;

   
  ((void)0);

  if ((((ADC_TypeDef*) 0x40088000)->ADC2_STATUS & Flag) == 0)
  {
    bitstatus = RESET;
  }
  else
  {
    bitstatus = SET;
  }

  return bitstatus;
}





 
void ADC1_ClearOverwriteFlag(void)
{
  ((ADC_TypeDef*) 0x40088000)->ADC1_STATUS &= ~(((uint32_t)0x1) << 0);
}





 
void ADC2_ClearOverwriteFlag(void)
{
  ((ADC_TypeDef*) 0x40088000)->ADC2_STATUS &= ~(((uint32_t)0x1) << 0);
}





 
void ADC1_ClearOutOfRangeFlag(void)
{
  ((ADC_TypeDef*) 0x40088000)->ADC1_STATUS &= ~(((uint32_t)0x1) << 1);
}





 
void ADC2_ClearOutOfRangeFlag(void)
{
  ((ADC_TypeDef*) 0x40088000)->ADC2_STATUS &= ~(((uint32_t)0x1) << 1);
}












 
void ADC_ITConfig(uint32_t ADC_IT, FunctionalState NewState)
{
  uint32_t tmpreg_ADC1_IE;
  uint32_t tmpreg_ADC2_IE;
  uint32_t tmpreg_ADC_IT;

   
  ((void)0);
  ((void)0);

  tmpreg_ADC1_IE = ((ADC_TypeDef*) 0x40088000)->ADC1_STATUS;
  tmpreg_ADC2_IE = ((ADC_TypeDef*) 0x40088000)->ADC2_STATUS;
  tmpreg_ADC_IT = ADC_IT << 2;

   
  if (NewState != DISABLE)
  {
     
    tmpreg_ADC1_IE |= tmpreg_ADC_IT & 0xFFFF;
    tmpreg_ADC2_IE |= tmpreg_ADC_IT >> 16;
  }
  else
  {
     
    tmpreg_ADC1_IE &= ~(tmpreg_ADC_IT & 0xFFFF);
    tmpreg_ADC2_IE &= ~(tmpreg_ADC_IT >> 16);
  }

   
  ((ADC_TypeDef*) 0x40088000)->ADC1_STATUS = tmpreg_ADC1_IE;
  ((ADC_TypeDef*) 0x40088000)->ADC2_STATUS = tmpreg_ADC2_IE;
}










 
void ADC1_ITConfig(uint32_t ADC_IT, FunctionalState NewState)
{
  uint32_t tmpreg_ADC1_IE;

   
  ((void)0);
  ((void)0);

  tmpreg_ADC1_IE = ((ADC_TypeDef*) 0x40088000)->ADC1_STATUS;

   
  if (NewState != DISABLE)
  {
     
    tmpreg_ADC1_IE |= (ADC_IT << 2);
  }
  else
  {
     
    tmpreg_ADC1_IE &= ~(ADC_IT << 2);
  }

   
  ((ADC_TypeDef*) 0x40088000)->ADC1_STATUS = tmpreg_ADC1_IE;
}










 
void ADC2_ITConfig(uint32_t ADC_IT, FunctionalState NewState)
{
  uint32_t tmpreg_ADC2_IE;

   
  ((void)0);
  ((void)0);

  tmpreg_ADC2_IE = ((ADC_TypeDef*) 0x40088000)->ADC2_STATUS;

   
  if (NewState != DISABLE)
  {
     
    tmpreg_ADC2_IE |= (ADC_IT << 2);
  }
  else
  {
     
    tmpreg_ADC2_IE &= ~(ADC_IT << 2);
  }

   
  ((ADC_TypeDef*) 0x40088000)->ADC2_STATUS = tmpreg_ADC2_IE;
}










 
ITStatus ADC_GetITStatus(uint32_t ADC_IT)
{
  ITStatus bitstatus;
  uint32_t tmpreg;

   
  ((void)0);

  tmpreg = ADC_GetStatus();
  tmpreg &= (tmpreg >> 2) & ADC_IT;

  if (tmpreg == 0)
  {
    bitstatus = RESET;
  }
  else
  {
    bitstatus = SET;
  }

  return bitstatus;
}








 
ITStatus ADC1_GetITStatus(uint32_t ADC_IT)
{
  ITStatus bitstatus;
  uint32_t tmpreg;

   
  ((void)0);

  tmpreg = ((ADC_TypeDef*) 0x40088000)->ADC1_STATUS;
  tmpreg &= (tmpreg >> 2) & ADC_IT;

  if (tmpreg == 0)
  {
    bitstatus = RESET;
  }
  else
  {
    bitstatus = SET;
  }

  return bitstatus;
}








 
ITStatus ADC2_GetITStatus(uint32_t ADC_IT)
{
  ITStatus bitstatus;
  uint32_t tmpreg;

   
  ((void)0);

  tmpreg = ((ADC_TypeDef*) 0x40088000)->ADC2_STATUS;
  tmpreg &= (tmpreg >> 2) & ADC_IT;

  if (tmpreg == 0)
  {
    bitstatus = RESET;
  }
  else
  {
    bitstatus = SET;
  }

  return bitstatus;
}

   

   

   



 

