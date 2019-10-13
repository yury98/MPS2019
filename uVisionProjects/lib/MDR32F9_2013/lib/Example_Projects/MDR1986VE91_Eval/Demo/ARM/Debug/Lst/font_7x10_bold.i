#line 1 "..\\..\\fonts\\Font_7x10_bold.c"


















 



 
#line 1 "..\\..\\inc\\font_defs.h"

















 

 



 
#line 1 "..\\..\\inc\\types.h"

















 


 



 
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






 
#line 27 "..\\..\\inc\\types.h"

typedef unsigned char const    ucint8_t;
typedef volatile unsigned char vuint8_t;
typedef volatile unsigned long vuint32_t;

typedef enum {FALSE = 0, TRUE = !FALSE} bool;





 

#line 26 "..\\..\\inc\\font_defs.h"



 



 



 



 
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

   

   

   





 
#line 25 "..\\..\\fonts\\Font_7x10_bold.c"



 



 



 

 
 
 
 
 
 

static ucint8_t Font_7x10_bold_Data[] = {
   
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,

   
  0x78, 0xa4, 0x4a, 0x42, 0x4a, 0xa4, 0x78, 0x00, 0x00, 0x01, 0x01, 0x01, 0x00, 0x00,

   
  0x78, 0xdc, 0xb6, 0xbe, 0xb6, 0xdc, 0x78, 0x00, 0x00, 0x01, 0x01, 0x01, 0x00, 0x00,

   
  0x1c, 0x3e, 0x7e, 0xfc, 0x7e, 0x3e, 0x1c, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,

   
  0x10, 0x38, 0x7c, 0xfe, 0x7c, 0x38, 0x10, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,

   
  0x30, 0x78, 0x77, 0xbf, 0x77, 0x78, 0x30, 0x00, 0x01, 0x01, 0x01, 0x01, 0x01, 0x00,

   
  0x38, 0x7c, 0x7e, 0xbf, 0x7e, 0x7c, 0x38, 0x00, 0x01, 0x01, 0x01, 0x01, 0x01, 0x00,

   
  0x00, 0x70, 0xf8, 0xf8, 0xf8, 0x70, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,

   
  0xff, 0x8f, 0x07, 0x07, 0x07, 0x8f, 0xff, 0x0f, 0x0f, 0x0f, 0x0f, 0x0f, 0x0f, 0x0f,

   
  0x00, 0x70, 0x88, 0x88, 0x88, 0x70, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,

   
  0xff, 0x8f, 0x77, 0x77, 0x77, 0x8f, 0xff, 0x0f, 0x0f, 0x0f, 0x0f, 0x0f, 0x0f, 0x0f,

   
  0xe0, 0x10, 0x10, 0x1a, 0xe6, 0x0e, 0x00, 0x00, 0x01, 0x01, 0x01, 0x00, 0x00, 0x00,

   
  0x00, 0x4e, 0x51, 0xf1, 0x51, 0x4e, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00,

   
  0x00, 0x80, 0x80, 0xfe, 0x04, 0x38, 0x00, 0x00, 0x01, 0x01, 0x00, 0x00, 0x00, 0x00,

   
  0x80, 0x80, 0xfe, 0x0a, 0xc5, 0xc5, 0x7f, 0x01, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00,

   
  0x10, 0xba, 0x44, 0xc7, 0x44, 0xba, 0x10, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00,

   
  0x00, 0x00, 0xfc, 0xf8, 0x70, 0x20, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00,

   
  0x00, 0x20, 0x70, 0xf8, 0xfc, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00,

   
  0x00, 0x44, 0xc6, 0xff, 0xc6, 0x44, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00,

   
  0x00, 0xbf, 0xbf, 0x00, 0xbf, 0xbf, 0x00, 0x00, 0x01, 0x01, 0x00, 0x01, 0x01, 0x00,

   
  0x00, 0x0e, 0x1f, 0x11, 0xff, 0x01, 0xff, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x01,

   
  0x00, 0x8e, 0xbf, 0x29, 0xfb, 0xe3, 0x00, 0x00, 0x01, 0x01, 0x01, 0x01, 0x00, 0x00,

   
  0x00, 0xc0, 0xc0, 0xc0, 0xc0, 0xc0, 0x00, 0x00, 0x01, 0x01, 0x01, 0x01, 0x01, 0x00,

   
  0x00, 0x24, 0x66, 0xff, 0x66, 0x24, 0x00, 0x00, 0x01, 0x01, 0x01, 0x01, 0x01, 0x00,

   
  0x00, 0x04, 0x06, 0xff, 0x06, 0x04, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00,

   
  0x00, 0x40, 0xc0, 0xff, 0xc0, 0x40, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00,

   
  0x20, 0x20, 0x20, 0xf8, 0x70, 0x20, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,

   
  0x20, 0x70, 0xf8, 0x20, 0x20, 0x20, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,

   
  0x00, 0xf0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x01, 0x01, 0x01, 0x01, 0x00,

   
  0x20, 0x70, 0xf8, 0x20, 0xf8, 0x70, 0x20, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,

   
  0xc0, 0xf0, 0xfc, 0xff, 0xfc, 0xf0, 0xc0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,

   
  0x06, 0x1e, 0x7e, 0xfe, 0x7e, 0x1e, 0x06, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00,

   
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,

   
  0x00, 0x00, 0xbf, 0xbf, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x01, 0x00, 0x00, 0x00,

   
  0x00, 0x07, 0x07, 0x00, 0x07, 0x07, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,

   
  0x00, 0x48, 0xfe, 0x48, 0xfe, 0x48, 0x00, 0x00, 0x00, 0x01, 0x00, 0x01, 0x00, 0x00,

   
  0x00, 0x4c, 0x9e, 0x93, 0xf2, 0x64, 0x00, 0x00, 0x00, 0x00, 0x03, 0x00, 0x00, 0x00,

   
  0x00, 0xc6, 0xe9, 0x36, 0xd8, 0x2f, 0xc7, 0x00, 0x01, 0x01, 0x00, 0x00, 0x01, 0x00,

   
  0x00, 0xe6, 0xff, 0x19, 0xff, 0xe6, 0x30, 0x00, 0x00, 0x01, 0x01, 0x00, 0x01, 0x01,

   
  0x00, 0x00, 0x04, 0x07, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,

   
  0x00, 0x7c, 0xfe, 0x83, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x01, 0x00, 0x00,

   
  0x00, 0x01, 0x83, 0xfe, 0x7c, 0x00, 0x00, 0x00, 0x01, 0x01, 0x00, 0x00, 0x00, 0x00,

   
  0x10, 0x54, 0x7c, 0x38, 0x7c, 0x54, 0x10, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,

   
  0x00, 0x20, 0x20, 0xf8, 0xf8, 0x20, 0x20, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,

   
  0x00, 0x00, 0x80, 0x80, 0x80, 0x00, 0x00, 0x00, 0x00, 0x05, 0x07, 0x03, 0x00, 0x00,

   
  0x00, 0x20, 0x20, 0x20, 0x20, 0x20, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,

   
  0x00, 0x00, 0x80, 0x80, 0x80, 0x00, 0x00, 0x00, 0x00, 0x01, 0x01, 0x01, 0x00, 0x00,

   
  0x00, 0xc0, 0xf0, 0x3c, 0x0f, 0x03, 0x00, 0x01, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00,

   
  0x00, 0xfe, 0xff, 0x01, 0xff, 0xff, 0x00, 0x00, 0x01, 0x01, 0x01, 0x01, 0x00, 0x00,

   
  0x00, 0x04, 0x06, 0xff, 0xff, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x01, 0x00, 0x00,

   
  0x00, 0x86, 0xc7, 0x71, 0x3f, 0x0e, 0x00, 0x00, 0x01, 0x01, 0x01, 0x01, 0x01, 0x00,

   
  0x00, 0x82, 0x93, 0x11, 0xff, 0xee, 0x00, 0x00, 0x00, 0x01, 0x01, 0x01, 0x00, 0x00,

   
  0x00, 0x78, 0x7f, 0x47, 0xf8, 0xf8, 0x40, 0x00, 0x00, 0x00, 0x00, 0x01, 0x01, 0x00,

   
  0x00, 0x9f, 0x9f, 0x11, 0xf1, 0xe1, 0x00, 0x00, 0x00, 0x01, 0x01, 0x01, 0x00, 0x00,

   
  0x00, 0xf8, 0xfe, 0x17, 0xf1, 0xe0, 0x00, 0x00, 0x00, 0x01, 0x01, 0x01, 0x00, 0x00,

   
  0x00, 0x01, 0xc1, 0xf9, 0x3f, 0x07, 0x00, 0x00, 0x00, 0x01, 0x01, 0x00, 0x00, 0x00,

   
  0x00, 0xee, 0xff, 0x11, 0xff, 0xee, 0x00, 0x00, 0x00, 0x01, 0x01, 0x01, 0x00, 0x00,

   
  0x00, 0x0e, 0x9f, 0xf1, 0x7f, 0x1e, 0x00, 0x00, 0x01, 0x01, 0x00, 0x00, 0x00, 0x00,

   
  0x00, 0x00, 0x8c, 0x8c, 0x8c, 0x00, 0x00, 0x00, 0x00, 0x01, 0x01, 0x01, 0x00, 0x00,

   
  0x00, 0x00, 0x8c, 0x8c, 0x8c, 0x00, 0x00, 0x00, 0x00, 0x05, 0x07, 0x03, 0x00, 0x00,

   
  0x00, 0x10, 0x38, 0x6c, 0xc6, 0x83, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00,

   
  0x00, 0x48, 0x48, 0x48, 0x48, 0x48, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,

   
  0x00, 0x83, 0xc6, 0x6c, 0x38, 0x10, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00,

   
  0x00, 0x06, 0xa7, 0xb1, 0x1f, 0x0e, 0x00, 0x00, 0x00, 0x01, 0x01, 0x00, 0x00, 0x00,

   
  0x00, 0x7c, 0xfe, 0x83, 0x39, 0x45, 0x3e, 0x00, 0x00, 0x00, 0x01, 0x01, 0x01, 0x01,

   
  0x00, 0xfc, 0xfe, 0x23, 0xfe, 0xfc, 0x00, 0x00, 0x01, 0x01, 0x00, 0x01, 0x01, 0x00,

   
  0x00, 0xff, 0xff, 0x11, 0xff, 0xee, 0x00, 0x00, 0x01, 0x01, 0x01, 0x01, 0x00, 0x00,

   
  0x00, 0xfe, 0xff, 0x01, 0xc7, 0xc6, 0x00, 0x00, 0x00, 0x01, 0x01, 0x01, 0x00, 0x00,

   
  0x00, 0xff, 0xff, 0x01, 0xff, 0xfe, 0x00, 0x00, 0x01, 0x01, 0x01, 0x01, 0x00, 0x00,

   
  0x00, 0xff, 0xff, 0x11, 0x11, 0x01, 0x00, 0x00, 0x01, 0x01, 0x01, 0x01, 0x01, 0x00,

   
  0x00, 0xff, 0xff, 0x11, 0x11, 0x01, 0x00, 0x00, 0x01, 0x01, 0x00, 0x00, 0x00, 0x00,

   
  0x00, 0xfe, 0xff, 0x01, 0xf3, 0xf2, 0x00, 0x00, 0x00, 0x01, 0x01, 0x01, 0x00, 0x00,

   
  0x00, 0xff, 0xff, 0x10, 0xff, 0xff, 0x00, 0x00, 0x01, 0x01, 0x00, 0x01, 0x01, 0x00,

   
  0x00, 0x00, 0x01, 0xff, 0xff, 0x01, 0x00, 0x00, 0x00, 0x01, 0x01, 0x01, 0x01, 0x00,

   
  0x00, 0xc0, 0xc0, 0x00, 0xff, 0xff, 0x00, 0x00, 0x00, 0x01, 0x01, 0x01, 0x00, 0x00,

   
  0x00, 0xff, 0xff, 0x38, 0xef, 0xc7, 0x00, 0x00, 0x01, 0x01, 0x00, 0x01, 0x01, 0x00,

   
  0x00, 0xff, 0xff, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x01, 0x01, 0x01, 0x01, 0x00,

   
  0xff, 0xff, 0x1c, 0x70, 0x1c, 0xff, 0xff, 0x01, 0x01, 0x00, 0x00, 0x00, 0x01, 0x01,

   
  0x00, 0xff, 0xff, 0x1c, 0x30, 0xff, 0xff, 0x00, 0x01, 0x01, 0x00, 0x00, 0x01, 0x01,

   
  0x00, 0xfe, 0xff, 0x01, 0xff, 0xfe, 0x00, 0x00, 0x00, 0x01, 0x01, 0x01, 0x00, 0x00,

   
  0x00, 0xff, 0xff, 0x11, 0x1f, 0x0e, 0x00, 0x00, 0x01, 0x01, 0x00, 0x00, 0x00, 0x00,

   
  0x00, 0xfe, 0xff, 0x01, 0xff, 0xfe, 0x00, 0x00, 0x00, 0x01, 0x03, 0x07, 0x04, 0x00,

   
  0x00, 0xff, 0xff, 0x11, 0xff, 0xee, 0x00, 0x00, 0x01, 0x01, 0x00, 0x01, 0x01, 0x00,

   
  0x00, 0xce, 0x9f, 0x39, 0xf3, 0xe6, 0x00, 0x00, 0x00, 0x01, 0x01, 0x01, 0x00, 0x00,

   
  0x00, 0x01, 0x01, 0xff, 0xff, 0x01, 0x01, 0x00, 0x00, 0x00, 0x01, 0x01, 0x00, 0x00,

   
  0x00, 0xff, 0xff, 0x00, 0xff, 0xff, 0x00, 0x00, 0x00, 0x01, 0x01, 0x01, 0x00, 0x00,

   
  0x00, 0x7f, 0xff, 0x80, 0xff, 0x7f, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00,

   
  0x3f, 0xff, 0xc0, 0x3c, 0xc0, 0xff, 0x3f, 0x00, 0x01, 0x01, 0x00, 0x01, 0x01, 0x00,

   
  0x00, 0xc7, 0xef, 0x38, 0xef, 0xc7, 0x00, 0x00, 0x01, 0x01, 0x00, 0x01, 0x01, 0x00,

   
  0x00, 0x0f, 0x1f, 0xf0, 0xf0, 0x1f, 0x0f, 0x00, 0x00, 0x00, 0x01, 0x01, 0x00, 0x00,

   
  0x00, 0xc1, 0xf1, 0x3d, 0x0f, 0x03, 0x00, 0x00, 0x01, 0x01, 0x01, 0x01, 0x01, 0x00,

   
  0x00, 0x00, 0xff, 0xff, 0x01, 0x01, 0x00, 0x00, 0x00, 0x03, 0x03, 0x02, 0x02, 0x00,

   
  0x00, 0x03, 0x0f, 0x3c, 0xf0, 0xc0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x01,

   
  0x00, 0x01, 0x01, 0xff, 0xff, 0x00, 0x00, 0x00, 0x02, 0x02, 0x03, 0x03, 0x00, 0x00,

   
  0x00, 0x04, 0x06, 0x03, 0x06, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,

   
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x02, 0x02, 0x02, 0x02, 0x02, 0x02,

   
  0x00, 0x00, 0x03, 0x07, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,

   
  0x00, 0xc0, 0xe8, 0x28, 0xf8, 0xf0, 0x00, 0x00, 0x00, 0x01, 0x01, 0x01, 0x01, 0x00,

   
  0x00, 0xff, 0xff, 0x08, 0xf8, 0xf0, 0x00, 0x00, 0x01, 0x01, 0x01, 0x01, 0x00, 0x00,

   
  0x00, 0xf0, 0xf8, 0x08, 0x98, 0x90, 0x00, 0x00, 0x00, 0x01, 0x01, 0x01, 0x00, 0x00,

   
  0x00, 0xf0, 0xf8, 0x08, 0xff, 0xff, 0x00, 0x00, 0x00, 0x01, 0x01, 0x01, 0x01, 0x00,

   
  0x00, 0xf0, 0xf8, 0x28, 0x38, 0xb0, 0x00, 0x00, 0x00, 0x01, 0x01, 0x01, 0x00, 0x00,

   
  0x00, 0x08, 0xfe, 0xff, 0x09, 0x09, 0x00, 0x00, 0x00, 0x01, 0x01, 0x00, 0x00, 0x00,

   
  0x00, 0x38, 0x7c, 0x44, 0xfc, 0xfc, 0x00, 0x00, 0x01, 0x02, 0x02, 0x03, 0x01, 0x00,

   
  0x00, 0xff, 0xff, 0x08, 0xf8, 0xf0, 0x00, 0x00, 0x01, 0x01, 0x00, 0x01, 0x01, 0x00,

   
  0x00, 0x00, 0x08, 0xf9, 0xf9, 0x00, 0x00, 0x00, 0x00, 0x01, 0x01, 0x01, 0x01, 0x00,

   
  0x00, 0x00, 0x08, 0xf9, 0xf9, 0x00, 0x00, 0x00, 0x02, 0x02, 0x03, 0x01, 0x00, 0x00,

   
  0x00, 0xff, 0xff, 0x70, 0xd8, 0x88, 0x00, 0x00, 0x01, 0x01, 0x00, 0x01, 0x01, 0x00,

   
  0x00, 0x00, 0x01, 0xff, 0xff, 0x00, 0x00, 0x00, 0x00, 0x01, 0x01, 0x01, 0x01, 0x00,

   
  0xf8, 0xf8, 0x18, 0xf0, 0x18, 0xf8, 0xf0, 0x01, 0x01, 0x00, 0x00, 0x00, 0x01, 0x01,

   
  0x00, 0xf8, 0xf8, 0x08, 0xf8, 0xf0, 0x00, 0x00, 0x01, 0x01, 0x00, 0x01, 0x01, 0x00,

   
  0x00, 0xf0, 0xf8, 0x08, 0xf8, 0xf0, 0x00, 0x00, 0x00, 0x01, 0x01, 0x01, 0x00, 0x00,

   
  0x00, 0xf8, 0xf8, 0x08, 0xf8, 0xf0, 0x00, 0x00, 0x07, 0x07, 0x01, 0x01, 0x00, 0x00,

   
  0x00, 0xf0, 0xf8, 0x08, 0xf8, 0xf8, 0x00, 0x00, 0x00, 0x01, 0x01, 0x07, 0x07, 0x00,

   
  0x00, 0xf8, 0xf8, 0x10, 0x18, 0x18, 0x00, 0x00, 0x01, 0x01, 0x00, 0x00, 0x00, 0x00,

   
  0x00, 0x30, 0x78, 0x68, 0xe8, 0xc8, 0x00, 0x00, 0x01, 0x01, 0x01, 0x01, 0x00, 0x00,

   
  0x00, 0x08, 0xfe, 0xfe, 0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x01, 0x01, 0x00,

   
  0x00, 0xf8, 0xf8, 0x00, 0xf8, 0xf8, 0x00, 0x00, 0x00, 0x01, 0x01, 0x01, 0x01, 0x00,

   
  0x00, 0x78, 0xf8, 0x80, 0xf8, 0x78, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00,

   
  0x78, 0xf8, 0x80, 0xf0, 0x80, 0xf8, 0x78, 0x00, 0x01, 0x01, 0x00, 0x01, 0x01, 0x00,

   
  0x00, 0x98, 0xf8, 0x60, 0xf8, 0x98, 0x00, 0x00, 0x01, 0x01, 0x00, 0x01, 0x01, 0x00,

   
  0x00, 0x78, 0xf8, 0x80, 0xf8, 0x78, 0x00, 0x00, 0x04, 0x06, 0x03, 0x01, 0x00, 0x00,

   
  0x00, 0x88, 0xc8, 0x68, 0x38, 0x18, 0x00, 0x00, 0x01, 0x01, 0x01, 0x01, 0x01, 0x00,

   
  0x00, 0x10, 0xfe, 0xef, 0x01, 0x01, 0x00, 0x00, 0x00, 0x00, 0x01, 0x01, 0x01, 0x00,

   
  0x00, 0x00, 0xdf, 0xdf, 0x00, 0x00, 0x00, 0x00, 0x00, 0x07, 0x07, 0x00, 0x00, 0x00,

   
  0x00, 0x01, 0xef, 0xfe, 0x10, 0x00, 0x00, 0x00, 0x01, 0x01, 0x00, 0x00, 0x00, 0x00,

   
  0x00, 0x06, 0x03, 0x07, 0x06, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,

   
  0x00, 0xf0, 0x98, 0x8c, 0x98, 0xf0, 0x00, 0x00, 0x01, 0x01, 0x01, 0x01, 0x01, 0x00,

   
  0x44, 0x00, 0x11, 0x44, 0x00, 0x11, 0x00, 0x04, 0x00, 0x01, 0x04, 0x00, 0x01, 0x00,

   
  0xaa, 0x00, 0x55, 0xaa, 0x00, 0x55, 0x00, 0x0a, 0x00, 0x05, 0x0a, 0x00, 0x05, 0x00,

   
  0xaa, 0x55, 0x55, 0xaa, 0x55, 0xaa, 0x55, 0x0a, 0x05, 0x05, 0x0a, 0x05, 0x0a, 0x05,

   
  0x00, 0x00, 0x00, 0xff, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0f, 0x00, 0x00, 0x00,

   
  0x20, 0x20, 0x20, 0xff, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0f, 0x00, 0x00, 0x00,

   
  0x50, 0x50, 0x50, 0xff, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0f, 0x00, 0x00, 0x00,

   
  0x20, 0x20, 0xff, 0x00, 0xff, 0x00, 0x00, 0x00, 0x00, 0x0f, 0x00, 0x0f, 0x00, 0x00,

   
  0x20, 0x20, 0xe0, 0x20, 0xe0, 0x00, 0x00, 0x00, 0x00, 0x0f, 0x00, 0x0f, 0x00, 0x00,

   
  0x50, 0x50, 0x50, 0xf0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0f, 0x00, 0x00, 0x00,

   
  0x50, 0x50, 0xdf, 0x00, 0xff, 0x00, 0x00, 0x00, 0x00, 0x0f, 0x00, 0x0f, 0x00, 0x00,

   
  0x00, 0x00, 0xff, 0x00, 0xff, 0x00, 0x00, 0x00, 0x00, 0x0f, 0x00, 0x0f, 0x00, 0x00,

   
  0x50, 0x50, 0xd0, 0x10, 0xf0, 0x00, 0x00, 0x00, 0x00, 0x0f, 0x00, 0x0f, 0x00, 0x00,

   
  0x50, 0x50, 0x5f, 0x40, 0x7f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,

   
  0x20, 0x20, 0x3f, 0x20, 0x3f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,

   
  0x50, 0x50, 0x50, 0x7f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,

   
  0x20, 0x20, 0x20, 0xe0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0f, 0x00, 0x00, 0x00,

   
  0x00, 0x00, 0x00, 0x3f, 0x20, 0x20, 0x20, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,

   
  0x20, 0x20, 0x20, 0x3f, 0x20, 0x20, 0x20, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,

   
  0x20, 0x20, 0x20, 0xe0, 0x20, 0x20, 0x20, 0x00, 0x00, 0x00, 0x0f, 0x00, 0x00, 0x00,

   
  0x00, 0x00, 0x00, 0xff, 0x20, 0x20, 0x20, 0x00, 0x00, 0x00, 0x0f, 0x00, 0x00, 0x00,

   
  0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,

   
  0x20, 0x20, 0x20, 0xff, 0x20, 0x20, 0x20, 0x00, 0x00, 0x00, 0x0f, 0x00, 0x00, 0x00,

   
  0x00, 0x00, 0x00, 0xff, 0x50, 0x50, 0x50, 0x00, 0x00, 0x00, 0x0f, 0x00, 0x00, 0x00,

   
  0x00, 0x00, 0xff, 0x00, 0xff, 0x20, 0x20, 0x00, 0x00, 0x0f, 0x00, 0x0f, 0x00, 0x00,

   
  0x00, 0x00, 0x7f, 0x40, 0x5f, 0x50, 0x50, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,

   
  0x00, 0x00, 0xf0, 0x10, 0xd0, 0x50, 0x50, 0x00, 0x00, 0x0f, 0x00, 0x0f, 0x00, 0x00,

   
  0x50, 0x50, 0x5f, 0x40, 0x5f, 0x50, 0x50, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,

   
  0x50, 0x50, 0xd0, 0x10, 0xd0, 0x50, 0x50, 0x00, 0x00, 0x0f, 0x00, 0x0f, 0x00, 0x00,

   
  0x00, 0x00, 0xff, 0x00, 0xdf, 0x50, 0x50, 0x00, 0x00, 0x0f, 0x00, 0x0f, 0x00, 0x00,

   
  0x50, 0x50, 0x50, 0x50, 0x50, 0x50, 0x50, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,

   
  0x50, 0x50, 0xdf, 0x00, 0xdf, 0x50, 0x50, 0x00, 0x00, 0x0f, 0x00, 0x0f, 0x00, 0x00,

   
  0x50, 0x50, 0x50, 0x5f, 0x50, 0x50, 0x50, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,

   
  0x20, 0x20, 0x3f, 0x20, 0x3f, 0x20, 0x20, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,

   
  0x50, 0x50, 0x50, 0xd0, 0x50, 0x50, 0x50, 0x00, 0x00, 0x00, 0x0f, 0x00, 0x00, 0x00,

   
  0x20, 0x20, 0xe0, 0x20, 0xe0, 0x20, 0x20, 0x00, 0x00, 0x0f, 0x00, 0x0f, 0x00, 0x00,

   
  0x00, 0x00, 0x3f, 0x20, 0x3f, 0x20, 0x20, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,

   
  0x00, 0x00, 0x00, 0x7f, 0x50, 0x50, 0x50, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,

   
  0x00, 0x00, 0x00, 0xf0, 0x50, 0x50, 0x50, 0x00, 0x00, 0x00, 0x0f, 0x00, 0x00, 0x00,

   
  0x00, 0x00, 0xe0, 0x20, 0xe0, 0x20, 0x20, 0x00, 0x00, 0x0f, 0x00, 0x0f, 0x00, 0x00,

   
  0x20, 0x20, 0xff, 0x00, 0xff, 0x20, 0x20, 0x00, 0x00, 0x0f, 0x00, 0x0f, 0x00, 0x00,

   
  0x00, 0xff, 0xff, 0x12, 0x13, 0x03, 0x00, 0x00, 0x01, 0x01, 0x01, 0x01, 0x01, 0x00,

   
  0x50, 0x50, 0x50, 0xdf, 0x50, 0x50, 0x50, 0x00, 0x00, 0x00, 0x0f, 0x00, 0x00, 0x00,

   
  0x20, 0x20, 0x20, 0x3f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,

   
  0x00, 0x00, 0x00, 0xe0, 0x20, 0x20, 0x20, 0x00, 0x00, 0x00, 0x0f, 0x00, 0x00, 0x00,

   
  0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x0f, 0x0f, 0x0f, 0x0f, 0x0f, 0x0f, 0x0f,

   
  0xc0, 0xc0, 0xc0, 0xc0, 0xc0, 0xc0, 0xc0, 0x0f, 0x0f, 0x0f, 0x0f, 0x0f, 0x0f, 0x0f,

   
  0xff, 0xff, 0xff, 0xff, 0x00, 0x00, 0x00, 0x0f, 0x0f, 0x0f, 0x0f, 0x00, 0x00, 0x00,

   
  0x00, 0x00, 0x00, 0x00, 0xff, 0xff, 0xff, 0x00, 0x00, 0x00, 0x00, 0x0f, 0x0f, 0x0f,

   
  0x3f, 0x3f, 0x3f, 0x3f, 0x3f, 0x3f, 0x3f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,

   
  0x00, 0xfe, 0xff, 0x11, 0x93, 0x82, 0x00, 0x00, 0x00, 0x01, 0x01, 0x01, 0x00, 0x00,

   
  0x00, 0xf0, 0xf8, 0x28, 0x28, 0x08, 0x90, 0x00, 0x00, 0x01, 0x01, 0x01, 0x01, 0x00,

   
  0x00, 0x01, 0x01, 0xfc, 0xfc, 0x01, 0x01, 0x00, 0x00, 0x01, 0x01, 0x01, 0x01, 0x00,

   
  0x00, 0x03, 0x0b, 0xf8, 0xfb, 0x03, 0x00, 0x00, 0x00, 0x01, 0x01, 0x01, 0x01, 0x00,

   
 

 
0x00, 0x78, 0x84, 0x30, 0x30, 0x84, 0x78,    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,

   
  0x00, 0x78, 0xfa, 0x84, 0xfa, 0x78, 0x00, 0x00, 0x04, 0x06, 0x03, 0x01, 0x00, 0x00,

   
  0x00, 0x0e, 0x1f, 0x11, 0x1f, 0x0e, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,

   
  0x00, 0xf3, 0xfb, 0x28, 0x3b, 0xb3, 0x00, 0x00, 0x00, 0x01, 0x01, 0x01, 0x00, 0x00,

   
  0x00, 0x00, 0x38, 0x7c, 0x7c, 0x38, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,

   
  0x00, 0x00, 0x00, 0x30, 0x30, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,

   
  0x00, 0x40, 0x80, 0x00, 0xfe, 0x02, 0x02, 0x00, 0x00, 0x00, 0x01, 0x01, 0x00, 0x00,

   
  0xff, 0xff, 0x1c, 0xff, 0xff, 0x27, 0x27, 0x01, 0x01, 0x00, 0x01, 0x01, 0x00, 0x00,

   
  0x74, 0x88, 0x04, 0x04, 0x88, 0x74, 0x00, 0x01, 0x00, 0x01, 0x01, 0x00, 0x01, 0x00,

   
  0x00, 0x00, 0x78, 0x78, 0x78, 0x78, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,

   
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,

   
   
  0x00, 0xfc, 0xfe, 0x23, 0xfe, 0xfc, 0x00, 0x00, 0x01, 0x01, 0x00, 0x01, 0x01, 0x00,

   
  0x00, 0xff, 0xff, 0x11, 0xf1, 0xe1, 0x00, 0x00, 0x01, 0x01, 0x01, 0x01, 0x00, 0x00,

   
  0x00, 0xff, 0xff, 0x11, 0xff, 0xee, 0x00, 0x00, 0x01, 0x01, 0x01, 0x01, 0x00, 0x00,

   
  0x00, 0xff, 0xff, 0x01, 0x01, 0x01, 0x00, 0x00, 0x01, 0x01, 0x00, 0x00, 0x00, 0x00,

   
  0x00, 0xfe, 0xff, 0x01, 0xff, 0xff, 0x00, 0x03, 0x03, 0x01, 0x01, 0x01, 0x03, 0x03,

   
  0x00, 0xff, 0xff, 0x11, 0x11, 0x01, 0x00, 0x00, 0x01, 0x01, 0x01, 0x01, 0x01, 0x00,

   
  0x00, 0xef, 0x10, 0xff, 0xff, 0x10, 0xef, 0x00, 0x01, 0x00, 0x01, 0x01, 0x00, 0x01,

   
  0x00, 0x82, 0x11, 0x11, 0xff, 0xee, 0x00, 0x00, 0x00, 0x01, 0x01, 0x01, 0x00, 0x00,

   
  0x00, 0xff, 0xff, 0x70, 0x1c, 0xff, 0xff, 0x00, 0x01, 0x01, 0x00, 0x00, 0x01, 0x01,

   
  0x00, 0xff, 0xff, 0x71, 0x1d, 0xff, 0xff, 0x00, 0x01, 0x01, 0x00, 0x00, 0x01, 0x01,

   
  0x00, 0xff, 0xff, 0x38, 0xef, 0xc7, 0x00, 0x00, 0x01, 0x01, 0x00, 0x01, 0x01, 0x00,

   
  0x00, 0xfe, 0xff, 0x01, 0xff, 0xff, 0x00, 0x00, 0x01, 0x01, 0x00, 0x01, 0x01, 0x00,

   
  0xff, 0xff, 0x1c, 0x70, 0x1c, 0xff, 0xff, 0x01, 0x01, 0x00, 0x00, 0x00, 0x01, 0x01,

   
  0x00, 0xff, 0xff, 0x10, 0xff, 0xff, 0x00, 0x00, 0x01, 0x01, 0x00, 0x01, 0x01, 0x00,

   
  0x00, 0xfe, 0xff, 0x01, 0xff, 0xfe, 0x00, 0x00, 0x00, 0x01, 0x01, 0x01, 0x00, 0x00,

   
  0x00, 0xff, 0xff, 0x01, 0xff, 0xff, 0x00, 0x00, 0x01, 0x01, 0x00, 0x01, 0x01, 0x00,

   
  0x00, 0xff, 0xff, 0x11, 0x1f, 0x0e, 0x00, 0x00, 0x01, 0x01, 0x00, 0x00, 0x00, 0x00,

   
  0x00, 0xfe, 0xff, 0x01, 0xc7, 0xc6, 0x00, 0x00, 0x00, 0x01, 0x01, 0x01, 0x00, 0x00,

   
  0x00, 0x01, 0x01, 0xff, 0xff, 0x01, 0x01, 0x00, 0x00, 0x00, 0x01, 0x01, 0x00, 0x00,

   
  0x00, 0x8f, 0x9f, 0x10, 0xff, 0xff, 0x00, 0x00, 0x00, 0x01, 0x01, 0x01, 0x00, 0x00,

   
  0x00, 0x3c, 0x42, 0xff, 0xff, 0x42, 0x3c, 0x00, 0x00, 0x00, 0x01, 0x01, 0x00, 0x00,

   
  0x00, 0xc7, 0xef, 0x38, 0xef, 0xc7, 0x00, 0x00, 0x01, 0x01, 0x00, 0x01, 0x01, 0x00,

   
  0x00, 0xff, 0xff, 0x00, 0xff, 0xff, 0x00, 0x00, 0x01, 0x01, 0x01, 0x01, 0x03, 0x03,

   
  0x00, 0x0f, 0x1f, 0x10, 0xff, 0xff, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x01, 0x00,

   
  0xff, 0xff, 0x00, 0xff, 0x00, 0xff, 0xff, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01,

   
  0xff, 0xff, 0x00, 0xff, 0x00, 0xff, 0xff, 0x01, 0x01, 0x01, 0x01, 0x01, 0x03, 0x03,

   
  0x07, 0x01, 0xff, 0xff, 0x10, 0xf0, 0xe0, 0x00, 0x00, 0x01, 0x01, 0x01, 0x01, 0x00,

   
  0xff, 0xff, 0x10, 0xf0, 0xe0, 0xff, 0xff, 0x01, 0x01, 0x01, 0x01, 0x00, 0x01, 0x01,

   
  0x00, 0xff, 0xff, 0x10, 0xf0, 0xe0, 0x00, 0x00, 0x01, 0x01, 0x01, 0x01, 0x00, 0x00,

   
  0x00, 0x82, 0x93, 0x11, 0xff, 0xfe, 0x00, 0x00, 0x00, 0x01, 0x01, 0x01, 0x00, 0x00,

   
  0xff, 0xff, 0x18, 0xff, 0x01, 0xff, 0xfe, 0x01, 0x01, 0x00, 0x01, 0x01, 0x01, 0x00,

   
  0x00, 0xee, 0xff, 0x11, 0xff, 0xff, 0x00, 0x00, 0x01, 0x01, 0x00, 0x01, 0x01, 0x00,

   
   
  0x00, 0xc0, 0xe8, 0x28, 0xf8, 0xf0, 0x00, 0x00, 0x00, 0x01, 0x01, 0x01, 0x01, 0x00,

   
  0x00, 0xf0, 0xf8, 0x28, 0xe8, 0xc4, 0x00, 0x00, 0x00, 0x01, 0x01, 0x01, 0x00, 0x00,

   
  0x00, 0xf8, 0xf8, 0x28, 0xf8, 0xd0, 0x00, 0x00, 0x01, 0x01, 0x01, 0x01, 0x00, 0x00,

   
  0x00, 0xf8, 0xf8, 0x08, 0x08, 0x08, 0x00, 0x00, 0x01, 0x01, 0x00, 0x00, 0x00, 0x00,

   
  0x00, 0xf0, 0xf8, 0x08, 0xf8, 0xf8, 0x00, 0x03, 0x03, 0x01, 0x01, 0x01, 0x03, 0x03,

   
  0x00, 0xf0, 0xf8, 0x28, 0x38, 0xb0, 0x00, 0x00, 0x00, 0x01, 0x01, 0x01, 0x00, 0x00,

   
  0x00, 0xd8, 0x20, 0xf8, 0xf8, 0x20, 0xd8, 0x00, 0x01, 0x00, 0x01, 0x01, 0x00, 0x01,

   
  0x00, 0x90, 0x08, 0x28, 0xf8, 0xd0, 0x00, 0x00, 0x00, 0x01, 0x01, 0x01, 0x00, 0x00,

   
  0x00, 0xf8, 0xf8, 0xc0, 0x60, 0xf8, 0xf8, 0x00, 0x01, 0x01, 0x00, 0x00, 0x01, 0x01,

   
  0x00, 0xf8, 0xf8, 0xc0, 0x66, 0xfb, 0xf8, 0x00, 0x01, 0x01, 0x00, 0x00, 0x01, 0x01,

   
  0x00, 0xf8, 0xf8, 0x70, 0xd8, 0x88, 0x00, 0x00, 0x01, 0x01, 0x00, 0x01, 0x01, 0x00,

   
  0x00, 0xf0, 0xf8, 0x08, 0xf8, 0xf8, 0x00, 0x00, 0x01, 0x01, 0x00, 0x01, 0x01, 0x00,

   
  0xf8, 0xf8, 0x18, 0x30, 0x18, 0xf8, 0xf8, 0x01, 0x01, 0x00, 0x00, 0x00, 0x01, 0x01,

   
  0x00, 0xf8, 0xf8, 0x20, 0xf8, 0xf8, 0x00, 0x00, 0x01, 0x01, 0x00, 0x01, 0x01, 0x00,

   
  0x00, 0xf0, 0xf8, 0x08, 0xf8, 0xf0, 0x00, 0x00, 0x00, 0x01, 0x01, 0x01, 0x00, 0x00,

   
  0x00, 0xf8, 0xf8, 0x08, 0xf8, 0xf8, 0x00, 0x00, 0x01, 0x01, 0x00, 0x01, 0x01, 0x00,

   
  0x00, 0xf8, 0xf8, 0x08, 0xf8, 0xf0, 0x00, 0x00, 0x07, 0x07, 0x01, 0x01, 0x00, 0x00,

   
  0x00, 0xf0, 0xf8, 0x08, 0x98, 0x90, 0x00, 0x00, 0x00, 0x01, 0x01, 0x01, 0x00, 0x00,

   
  0x00, 0x08, 0x08, 0xf8, 0xf8, 0x08, 0x08, 0x00, 0x00, 0x00, 0x01, 0x01, 0x00, 0x00,

   
  0x00, 0x78, 0xf8, 0x80, 0xf8, 0x78, 0x00, 0x00, 0x04, 0x06, 0x03, 0x01, 0x00, 0x00,

   
  0x00, 0xf0, 0x08, 0xf8, 0xf8, 0x08, 0xf0, 0x00, 0x00, 0x01, 0x07, 0x07, 0x01, 0x00,

   
  0x00, 0x98, 0xf8, 0x60, 0xf8, 0x98, 0x00, 0x00, 0x01, 0x01, 0x00, 0x01, 0x01, 0x00,

   
  0x00, 0xf8, 0xf8, 0x00, 0xf8, 0xf8, 0x00, 0x00, 0x01, 0x01, 0x01, 0x01, 0x03, 0x03,

   
  0x00, 0x38, 0x78, 0x40, 0xf8, 0xf8, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x01, 0x00,

   
  0xf8, 0xf8, 0x00, 0xf8, 0x00, 0xf8, 0xf8, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01,

   
  0xf8, 0xf8, 0x00, 0xf8, 0x00, 0xf8, 0xf8, 0x01, 0x01, 0x01, 0x01, 0x01, 0x03, 0x03,

   
  0x18, 0x18, 0xf8, 0xf8, 0x20, 0xe0, 0xc0, 0x00, 0x00, 0x01, 0x01, 0x01, 0x01, 0x00,

   
  0xf8, 0xf8, 0x20, 0xe0, 0xc0, 0xf8, 0xf8, 0x01, 0x01, 0x01, 0x01, 0x00, 0x01, 0x01,

   
  0x00, 0xf8, 0xf8, 0x20, 0xe0, 0xc0, 0x00, 0x00, 0x01, 0x01, 0x01, 0x01, 0x00, 0x00,

   
  0x00, 0x90, 0x08, 0x28, 0x28, 0xf8, 0xf0, 0x00, 0x00, 0x01, 0x01, 0x01, 0x01, 0x00,

   
  0xf8, 0xf8, 0x60, 0xf8, 0x08, 0xf8, 0xf0, 0x01, 0x01, 0x00, 0x01, 0x01, 0x01, 0x00,

   
  0x00, 0xb0, 0xf8, 0x48, 0xf8, 0xf8, 0x00, 0x00, 0x01, 0x01, 0x00, 0x01, 0x01, 0x00
};

sFONT Font_7x10_bold = {
  10,                        
  7,                         
  255,                       
  &Font_7x10_bold_Data[0]    
};

   

   

   









 
