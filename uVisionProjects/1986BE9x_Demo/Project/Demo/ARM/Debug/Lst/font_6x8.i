#line 1 "..\\..\\fonts\\Font_6x8.c"

















 

 
#line 1 "..\\..\\Inc\\font_defs.h"

















 

 



 
#line 1 "..\\..\\Inc\\types.h"

















 


 



 
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






 
#line 27 "..\\..\\Inc\\types.h"

typedef unsigned char const    ucint8_t;
typedef volatile unsigned char vuint8_t;
typedef volatile unsigned long vuint32_t;

typedef enum {FALSE = 0, TRUE = !FALSE} bool;





 

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

   

   

   





 
#line 22 "..\\..\\fonts\\Font_6x8.c"



 



 



 

 
 
 
 
 
 

static ucint8_t Font_6x8_Data[] = {
   
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00,

   
  0x00, 0x3e, 0x45, 0x51, 0x45, 0x3e,

   
  0x00, 0x3e, 0x6b, 0x6f, 0x6b, 0x3e,

   
  0x00, 0x1c, 0x3e, 0x7c, 0x3e, 0x1c,

   
  0x00, 0x18, 0x3c, 0x7e, 0x3c, 0x18,

   
  0x00, 0x30, 0x36, 0x7f, 0x36, 0x30,

   
  0x00, 0x18, 0x5c, 0x7e, 0x5c, 0x18,

   
  0x00, 0x00, 0x18, 0x18, 0x00, 0x00,

   
  0xff, 0xff, 0xe7, 0xe7, 0xff, 0xff,

   
  0x00, 0x3c, 0x24, 0x24, 0x3c, 0x00,

   
  0xff, 0xc3, 0xdb, 0xdb, 0xc3, 0xff,

   
  0x00, 0x30, 0x48, 0x4a, 0x36, 0x0e,

   
  0x00, 0x06, 0x29, 0x79, 0x29, 0x06,

   
  0x00, 0x60, 0x70, 0x3f, 0x02, 0x04,

   
  0x00, 0x60, 0x7e, 0x0a, 0x35, 0x3f,

   
  0x00, 0x2a, 0x1c, 0x36, 0x1c, 0x2a,

   
  0x00, 0x00, 0x7f, 0x3e, 0x1c, 0x08,

   
  0x00, 0x08, 0x1c, 0x3e, 0x7f, 0x00,

   
  0x00, 0x14, 0x36, 0x7f, 0x36, 0x14,

   
  0x00, 0x00, 0x5f, 0x00, 0x5f, 0x00,

   
  0x00, 0x06, 0x09, 0x7f, 0x01, 0x7f,

   
  0x00, 0x22, 0x4d, 0x55, 0x59, 0x22,

   
  0x00, 0x60, 0x60, 0x60, 0x60, 0x00,

   
  0x00, 0x14, 0xb6, 0xff, 0xb6, 0x14,

   
  0x00, 0x04, 0x06, 0x7f, 0x06, 0x04,

   
  0x00, 0x10, 0x30, 0x7f, 0x30, 0x10,

   
  0x00, 0x08, 0x08, 0x3e, 0x1c, 0x08,

   
  0x00, 0x08, 0x1c, 0x3e, 0x08, 0x08,

   
  0x00, 0x78, 0x40, 0x40, 0x40, 0x40,

   
  0x00, 0x08, 0x3e, 0x08, 0x3e, 0x08,

   
  0x00, 0x30, 0x3c, 0x3f, 0x3c, 0x30,

   
  0x00, 0x03, 0x0f, 0x3f, 0x0f, 0x03,

   
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00,

   
  0x00, 0x00, 0x06, 0x5f, 0x06, 0x00,

   
  0x00, 0x07, 0x03, 0x00, 0x07, 0x03,

   
  0x00, 0x24, 0x7e, 0x24, 0x7e, 0x24,

   
  0x00, 0x24, 0x2b, 0x6a, 0x12, 0x00,

   
  0x00, 0x63, 0x13, 0x08, 0x64, 0x63,

   
  0x00, 0x36, 0x49, 0x56, 0x20, 0x50,

   
  0x00, 0x00, 0x07, 0x03, 0x00, 0x00,

   
  0x00, 0x00, 0x3e, 0x41, 0x00, 0x00,

   
  0x00, 0x00, 0x41, 0x3e, 0x00, 0x00,

   
  0x00, 0x08, 0x3e, 0x1c, 0x3e, 0x08,

   
  0x00, 0x08, 0x08, 0x3e, 0x08, 0x08,

   
  0x00, 0x00, 0xe0, 0x60, 0x00, 0x00,

   
  0x00, 0x08, 0x08, 0x08, 0x08, 0x08,

   
  0x00, 0x00, 0x60, 0x60, 0x00, 0x00,

   
  0x00, 0x20, 0x10, 0x08, 0x04, 0x02,

   
  0x00, 0x3e, 0x41, 0x41, 0x41, 0x3e,

   
  0x00, 0x00, 0x42, 0x7f, 0x40, 0x00,

   
  0x00, 0x62, 0x51, 0x49, 0x49, 0x46,

   
  0x00, 0x22, 0x49, 0x49, 0x49, 0x36,

   
  0x00, 0x18, 0x14, 0x12, 0x7f, 0x10,

   
  0x00, 0x2f, 0x49, 0x49, 0x49, 0x31,

   
  0x00, 0x3c, 0x4a, 0x49, 0x49, 0x30,

   
  0x00, 0x01, 0x71, 0x09, 0x05, 0x03,

   
  0x00, 0x36, 0x49, 0x49, 0x49, 0x36,

   
  0x00, 0x06, 0x49, 0x49, 0x29, 0x1e,

   
  0x00, 0x00, 0x6c, 0x6c, 0x00, 0x00,

   
  0x00, 0x00, 0xec, 0x6c, 0x00, 0x00,

   
  0x00, 0x08, 0x14, 0x22, 0x41, 0x00,

   
  0x00, 0x24, 0x24, 0x24, 0x24, 0x24,

   
  0x00, 0x00, 0x41, 0x22, 0x14, 0x08,

   
  0x00, 0x02, 0x01, 0x59, 0x09, 0x06,

   
  0x00, 0x3e, 0x41, 0x5d, 0x55, 0x1e,

   
  0x00, 0x7e, 0x11, 0x11, 0x11, 0x7e,

   
  0x00, 0x7f, 0x49, 0x49, 0x49, 0x36,

   
  0x00, 0x3e, 0x41, 0x41, 0x41, 0x22,

   
  0x00, 0x7f, 0x41, 0x41, 0x41, 0x3e,

   
  0x00, 0x7f, 0x49, 0x49, 0x49, 0x41,

   
  0x00, 0x7f, 0x09, 0x09, 0x09, 0x01,

   
  0x00, 0x3e, 0x41, 0x49, 0x49, 0x7a,

   
  0x00, 0x7f, 0x08, 0x08, 0x08, 0x7f,

   
  0x00, 0x00, 0x41, 0x7f, 0x41, 0x00,

   
  0x00, 0x30, 0x40, 0x40, 0x40, 0x3f,

   
  0x00, 0x7f, 0x08, 0x14, 0x22, 0x41,

   
  0x00, 0x7f, 0x40, 0x40, 0x40, 0x40,

   
  0x00, 0x7f, 0x02, 0x04, 0x02, 0x7f,

   
  0x00, 0x7f, 0x02, 0x04, 0x08, 0x7f,

   
  0x00, 0x3e, 0x41, 0x41, 0x41, 0x3e,

   
  0x00, 0x7f, 0x09, 0x09, 0x09, 0x06,

   
  0x00, 0x3e, 0x41, 0x51, 0x21, 0x5e,

   
  0x00, 0x7f, 0x09, 0x09, 0x19, 0x66,

   
  0x00, 0x26, 0x49, 0x49, 0x49, 0x32,

   
  0x00, 0x01, 0x01, 0x7f, 0x01, 0x01,

   
  0x00, 0x3f, 0x40, 0x40, 0x40, 0x3f,

   
  0x00, 0x1f, 0x20, 0x40, 0x20, 0x1f,

   
  0x00, 0x3f, 0x40, 0x3c, 0x40, 0x3f,

   
  0x00, 0x63, 0x14, 0x08, 0x14, 0x63,

   
  0x00, 0x07, 0x08, 0x70, 0x08, 0x07,

   
  0x00, 0x71, 0x49, 0x45, 0x43, 0x00,

   
  0x00, 0x00, 0x7f, 0x41, 0x41, 0x00,

   
  0x00, 0x02, 0x04, 0x08, 0x10, 0x20,

   
  0x00, 0x00, 0x41, 0x41, 0x7f, 0x00,

   
  0x00, 0x04, 0x02, 0x01, 0x02, 0x04,

   
  0x80, 0x80, 0x80, 0x80, 0x80, 0x80,

   
  0x00, 0x00, 0x03, 0x07, 0x00, 0x00,

   
  0x00, 0x20, 0x54, 0x54, 0x54, 0x78,

   
  0x00, 0x7f, 0x44, 0x44, 0x44, 0x38,

   
  0x00, 0x38, 0x44, 0x44, 0x44, 0x28,

   
  0x00, 0x38, 0x44, 0x44, 0x44, 0x7f,

   
  0x00, 0x38, 0x54, 0x54, 0x54, 0x08,

   
  0x00, 0x08, 0x7e, 0x09, 0x09, 0x00,

   
  0x00, 0x18, 0xa4, 0xa4, 0xa4, 0x7c,

   
  0x00, 0x7f, 0x04, 0x04, 0x78, 0x00,

   
  0x00, 0x00, 0x00, 0x7d, 0x40, 0x00,

   
  0x00, 0x40, 0x80, 0x84, 0x7d, 0x00,

   
  0x00, 0x7f, 0x10, 0x28, 0x44, 0x00,

   
  0x00, 0x00, 0x00, 0x7f, 0x40, 0x00,

   
  0x00, 0x7c, 0x04, 0x18, 0x04, 0x78,

   
  0x00, 0x7c, 0x04, 0x04, 0x78, 0x00,

   
  0x00, 0x38, 0x44, 0x44, 0x44, 0x38,

   
  0x00, 0xfc, 0x44, 0x44, 0x44, 0x38,

   
  0x00, 0x38, 0x44, 0x44, 0x44, 0xfc,

   
  0x00, 0x44, 0x78, 0x44, 0x04, 0x08,

   
  0x00, 0x08, 0x54, 0x54, 0x54, 0x20,

   
  0x00, 0x04, 0x3e, 0x44, 0x24, 0x00,

   
  0x00, 0x3c, 0x40, 0x20, 0x7c, 0x00,

   
  0x00, 0x1c, 0x20, 0x40, 0x20, 0x1c,

   
  0x00, 0x3c, 0x60, 0x30, 0x60, 0x3c,

   
  0x00, 0x6c, 0x10, 0x10, 0x6c, 0x00,

   
  0x00, 0x9c, 0xa0, 0x60, 0x3c, 0x00,

   
  0x00, 0x64, 0x54, 0x54, 0x4c, 0x00,

   
  0x00, 0x08, 0x3e, 0x41, 0x41, 0x00,

   
  0x00, 0x00, 0x00, 0x77, 0x00, 0x00,

   
  0x00, 0x00, 0x41, 0x41, 0x3e, 0x08,

   
  0x00, 0x02, 0x01, 0x02, 0x01, 0x00,

   
  0x00, 0x3c, 0x26, 0x23, 0x26, 0x3c,

   
  0x44, 0x11, 0x44, 0x11, 0x44, 0x11,

   
  0xaa, 0x55, 0xaa, 0x55, 0xaa, 0x55,

   
  0xbb, 0xee, 0xbb, 0xee, 0xbb, 0xee,

   
  0x00, 0x00, 0x00, 0xff, 0x00, 0x00,

   
  0x08, 0x08, 0x08, 0xff, 0x00, 0x00,

   
  0x0a, 0x0a, 0x0a, 0xff, 0x00, 0x00,

   
  0x08, 0xff, 0x00, 0xff, 0x00, 0x00,

   
  0x08, 0xf8, 0x08, 0xf8, 0x00, 0x00,

   
  0x0a, 0x0a, 0x0a, 0xfe, 0x00, 0x00,

   
  0x0a, 0xfb, 0x00, 0xff, 0x00, 0x00,

   
  0x00, 0xff, 0x00, 0xff, 0x00, 0x00,

   
  0x0a, 0xfa, 0x02, 0xfe, 0x00, 0x00,

   
  0x0a, 0x0b, 0x08, 0x0f, 0x00, 0x00,

   
  0x08, 0x0f, 0x08, 0x0f, 0x00, 0x00,

   
  0x0a, 0x0a, 0x0a, 0x0f, 0x00, 0x00,

   
  0x08, 0x08, 0x08, 0xf8, 0x00, 0x00,

   
  0x00, 0x00, 0x00, 0x0f, 0x08, 0x08,

   
  0x08, 0x08, 0x08, 0x0f, 0x08, 0x08,

   
  0x08, 0x08, 0x08, 0xf8, 0x08, 0x08,

   
  0x00, 0x00, 0x00, 0xff, 0x08, 0x08,

   
  0x08, 0x08, 0x08, 0x08, 0x08, 0x08,

   
  0x08, 0x08, 0x08, 0xff, 0x08, 0x08,

   
  0x00, 0x00, 0x00, 0xff, 0x0a, 0x0a,

   
  0x00, 0xff, 0x00, 0xff, 0x08, 0x08,

   
  0x00, 0x0f, 0x08, 0x0b, 0x0a, 0x0a,

   
  0x00, 0xfe, 0x02, 0xfa, 0x0a, 0x0a,

   
  0x0a, 0x0b, 0x08, 0x0b, 0x0a, 0x0a,

   
  0x0a, 0xfa, 0x02, 0xfa, 0x0a, 0x0a,

   
  0x00, 0xff, 0x00, 0xfb, 0x0a, 0x0a,

   
  0x0a, 0x0a, 0x0a, 0x0a, 0x0a, 0x0a,

   
  0x0a, 0xfb, 0x00, 0xfb, 0x0a, 0x0a,

   
  0x0a, 0x0a, 0x0a, 0x0b, 0x0a, 0x0a,

   
  0x08, 0x0f, 0x08, 0x0f, 0x08, 0x08,

   
  0x0a, 0x0a, 0x0a, 0xfa, 0x0a, 0x0a,

   
  0x08, 0xf8, 0x08, 0xf8, 0x08, 0x08,

   
  0x00, 0x0f, 0x08, 0x0f, 0x08, 0x08,

   
  0x00, 0x00, 0x00, 0x0f, 0x0a, 0x0a,

   
  0x00, 0x00, 0x00, 0xfe, 0x0a, 0x0a,

   
  0x00, 0xf8, 0x08, 0xf8, 0x08, 0x08,

   
  0x08, 0xff, 0x08, 0xff, 0x08, 0x08,

   
  0x00, 0x7e, 0x4b, 0x4a, 0x4b, 0x42,

   
  0x0a, 0x0a, 0x0a, 0xff, 0x0a, 0x0a,

   
  0x08, 0x08, 0x08, 0x0f, 0x00, 0x00,

   
  0x00, 0x00, 0x00, 0xf8, 0x08, 0x08,

   
  0xff, 0xff, 0xff, 0xff, 0xff, 0xff,

   
  0xf0, 0xf0, 0xf0, 0xf0, 0xf0, 0xf0,

   
  0xff, 0xff, 0xff, 0x00, 0x00, 0x00,

   
  0x00, 0x00, 0x00, 0xff, 0xff, 0xff,

   
  0x0f, 0x0f, 0x0f, 0x0f, 0x0f, 0x0f,

   
  0x00, 0x3e, 0x49, 0x49, 0x41, 0x22,

   
  0x00, 0x38, 0x54, 0x54, 0x44, 0x28,

   
  0x00, 0x01, 0x40, 0x7e, 0x40, 0x01,

   
  0x00, 0x00, 0x01, 0x7c, 0x41, 0x00,

   
  0x00, 0x27, 0x48, 0x4b, 0x48, 0x3f,

   
  0x00, 0x9d, 0xa2, 0x62, 0x3d, 0x00,

   
  0x00, 0x06, 0x09, 0x09, 0x06, 0x00,

   
  0x00, 0x38, 0x55, 0x54, 0x55, 0x08,

   
  0x00, 0x00, 0x18, 0x18, 0x00, 0x00,

   
  0x00, 0x00, 0x08, 0x00, 0x00, 0x00,

   
  0x00, 0x30, 0x40, 0x3e, 0x02, 0x02,

   
  0x7f, 0x06, 0x18, 0x7f, 0x13, 0x13,

   
  0x2a, 0x3e, 0x14, 0x14, 0x3e, 0x2a,

   
  0x00, 0x3c, 0x3c, 0x3c, 0x3c, 0x00,

   
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00,

   
   
  0x00, 0x7e, 0x11, 0x11, 0x11, 0x7e,

   
  0x00, 0x7f, 0x49, 0x49, 0x49, 0x31,

   
  0x00, 0x7f, 0x49, 0x49, 0x49, 0x36,

   
  0x00, 0x7f, 0x01, 0x01, 0x01, 0x01,

   
  0xc0, 0x7e, 0x41, 0x41, 0x7f, 0xc0,

   
  0x00, 0x7f, 0x49, 0x49, 0x49, 0x41,

   
  0x00, 0x77, 0x08, 0x7f, 0x08, 0x77,

   
  0x22, 0x49, 0x49, 0x49, 0x36, 0x00,

   
  0x00, 0x7f, 0x20, 0x10, 0x08, 0x7f,

   
  0x00, 0x7e, 0x21, 0x11, 0x09, 0x7e,

   
  0x00, 0x7f, 0x08, 0x14, 0x22, 0x41,

   
  0x00, 0x40, 0x7e, 0x01, 0x01, 0x7f,

   
  0x00, 0x7f, 0x02, 0x04, 0x02, 0x7f,

   
  0x00, 0x7f, 0x08, 0x08, 0x08, 0x7f,

   
  0x00, 0x3e, 0x41, 0x41, 0x41, 0x3e,

   
  0x00, 0x7f, 0x01, 0x01, 0x01, 0x7f,

   
  0x00, 0x7f, 0x09, 0x09, 0x09, 0x06,

   
  0x00, 0x3e, 0x41, 0x41, 0x41, 0x22,

   
  0x00, 0x01, 0x01, 0x7f, 0x01, 0x01,

   
  0x00, 0x27, 0x48, 0x48, 0x48, 0x3f,

   
  0x00, 0x0e, 0x11, 0x7f, 0x11, 0x0e,

   
  0x00, 0x63, 0x14, 0x08, 0x14, 0x63,

   
  0x00, 0x7f, 0x40, 0x40, 0x7f, 0xc0,

   
  0x00, 0x07, 0x08, 0x08, 0x08, 0x7f,

   
  0x00, 0x7f, 0x40, 0x7f, 0x40, 0x7f,

   
  0x00, 0x7f, 0x40, 0x7f, 0x40, 0xff,

   
  0x03, 0x01, 0x7f, 0x48, 0x48, 0x30,

   
  0x00, 0x7f, 0x48, 0x48, 0x30, 0x7f,

   
  0x00, 0x7f, 0x48, 0x48, 0x48, 0x30,

   
  0x00, 0x22, 0x41, 0x49, 0x49, 0x3e,

   
  0x00, 0x7f, 0x08, 0x3e, 0x41, 0x3e,

   
  0x00, 0x66, 0x19, 0x09, 0x09, 0x7f,

   
   
  0x00, 0x20, 0x54, 0x54, 0x54, 0x78,

   
  0x00, 0x3c, 0x4a, 0x4a, 0x4a, 0x31,

   
  0x00, 0x7c, 0x54, 0x54, 0x54, 0x28,

   
  0x00, 0x7c, 0x04, 0x04, 0x0c, 0x00,

   
  0xc0, 0x78, 0x44, 0x44, 0x7c, 0xc0,

   
  0x00, 0x38, 0x54, 0x54, 0x54, 0x08,

   
  0x00, 0x6c, 0x10, 0x7c, 0x10, 0x6c,

   
  0x00, 0x28, 0x44, 0x54, 0x54, 0x28,

   
  0x00, 0x7c, 0x20, 0x10, 0x08, 0x7c,

   
  0x00, 0x7c, 0x20, 0x12, 0x0a, 0x7c,

   
  0x00, 0x7c, 0x10, 0x28, 0x44, 0x00,

   
  0x40, 0x38, 0x04, 0x04, 0x7c, 0x00,

   
  0x00, 0x7c, 0x08, 0x10, 0x08, 0x7c,

   
  0x00, 0x7c, 0x10, 0x10, 0x10, 0x7c,

   
  0x00, 0x38, 0x44, 0x44, 0x44, 0x38,

   
  0x00, 0x7c, 0x04, 0x04, 0x04, 0x7c,

   
  0x00, 0xfc, 0x44, 0x44, 0x44, 0x38,

   
  0x00, 0x38, 0x44, 0x44, 0x44, 0x28,

   
  0x00, 0x04, 0x04, 0x7c, 0x04, 0x04,

   
  0x00, 0x9c, 0xa0, 0x60, 0x3c, 0x00,

   
  0x00, 0x18, 0x24, 0x7c, 0x24, 0x18,

   
  0x00, 0x6c, 0x10, 0x10, 0x6c, 0x00,

   
  0x00, 0x7c, 0x40, 0x40, 0x7c, 0xc0,

   
  0x00, 0x0c, 0x10, 0x10, 0x10, 0x7c,

   
  0x00, 0x7c, 0x40, 0x7c, 0x40, 0x7c,

   
  0x00, 0x7c, 0x40, 0x7c, 0x40, 0xfc,

   
  0x0c, 0x04, 0x7c, 0x50, 0x50, 0x20,

   
  0x00, 0x7c, 0x50, 0x50, 0x20, 0x7c,

   
  0x00, 0x7c, 0x50, 0x50, 0x50, 0x20,

   
  0x00, 0x28, 0x44, 0x54, 0x54, 0x38,

   
  0x00, 0x7c, 0x10, 0x38, 0x44, 0x38,

   
  0x00, 0x48, 0x34, 0x14, 0x14, 0x7c
};

sFONT Font_6x8 = {
  8,                     
  6,                     
  255,                   
  &Font_6x8_Data[0]      
};

   

   

   



 

