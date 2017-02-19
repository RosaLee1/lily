#define nx_struct struct
#define nx_union union
#define dbg(mode, format, ...) ((void)0)
#define dbg_clear(mode, format, ...) ((void)0)
#define dbg_active(mode) 0
# 150 "/usr/bin/../lib/gcc/msp430/4.6.3/include/stddef.h" 3
typedef long int ptrdiff_t;
#line 212
typedef unsigned int size_t;
#line 324
typedef int wchar_t;
# 8 "/usr/lib/ncc/deputy_nodeputy.h"
struct __nesc_attr_nonnull {
#line 8
  int dummy;
}  ;
#line 9
struct __nesc_attr_bnd {
#line 9
  void *lo, *hi;
}  ;
#line 10
struct __nesc_attr_bnd_nok {
#line 10
  void *lo, *hi;
}  ;
#line 11
struct __nesc_attr_count {
#line 11
  int n;
}  ;
#line 12
struct __nesc_attr_count_nok {
#line 12
  int n;
}  ;
#line 13
struct __nesc_attr_one {
#line 13
  int dummy;
}  ;
#line 14
struct __nesc_attr_one_nok {
#line 14
  int dummy;
}  ;
#line 15
struct __nesc_attr_dmemset {
#line 15
  int a1, a2, a3;
}  ;
#line 16
struct __nesc_attr_dmemcpy {
#line 16
  int a1, a2, a3;
}  ;
#line 17
struct __nesc_attr_nts {
#line 17
  int dummy;
}  ;
# 38 "/usr/bin/../lib/gcc/msp430/4.6.3/../../../../msp430/include/stdint.h" 3
typedef signed char int8_t;
typedef int int16_t;
typedef long int int32_t;
__extension__ 
#line 41
typedef long long int int64_t;

typedef unsigned char uint8_t;
typedef unsigned int uint16_t;
typedef unsigned long int uint32_t;
__extension__ 
#line 46
typedef unsigned long long int uint64_t;





typedef signed char int_least8_t;
typedef int int_least16_t;
typedef long int int_least32_t;
__extension__ 
#line 55
typedef long long int int_least64_t;


typedef unsigned char uint_least8_t;
typedef unsigned int uint_least16_t;
typedef unsigned long int uint_least32_t;
__extension__ 
#line 61
typedef unsigned long long int uint_least64_t;





typedef signed char int_fast8_t;
typedef int int_fast16_t;
typedef long int int_fast32_t;
__extension__ 
#line 70
typedef long long int int_fast64_t;


typedef unsigned char uint_fast8_t;
typedef unsigned int uint_fast16_t;
typedef unsigned long int uint_fast32_t;
__extension__ 
#line 76
typedef unsigned long long int uint_fast64_t;









typedef int16_t intptr_t;
typedef uint16_t uintptr_t;





__extension__ 
#line 93
typedef long long int intmax_t;
__extension__ 
#line 94
typedef unsigned long long int uintmax_t;
# 431 "/usr/lib/ncc/nesc_nx.h"
typedef struct { unsigned char nxdata[1]; } __attribute__((packed)) nx_int8_t;typedef int8_t __nesc_nxbase_nx_int8_t  ;
typedef struct { unsigned char nxdata[2]; } __attribute__((packed)) nx_int16_t;typedef int16_t __nesc_nxbase_nx_int16_t  ;
typedef struct { unsigned char nxdata[4]; } __attribute__((packed)) nx_int32_t;typedef int32_t __nesc_nxbase_nx_int32_t  ;
typedef struct { unsigned char nxdata[8]; } __attribute__((packed)) nx_int64_t;typedef int64_t __nesc_nxbase_nx_int64_t  ;
typedef struct { unsigned char nxdata[1]; } __attribute__((packed)) nx_uint8_t;typedef uint8_t __nesc_nxbase_nx_uint8_t  ;
typedef struct { unsigned char nxdata[2]; } __attribute__((packed)) nx_uint16_t;typedef uint16_t __nesc_nxbase_nx_uint16_t  ;
typedef struct { unsigned char nxdata[4]; } __attribute__((packed)) nx_uint32_t;typedef uint32_t __nesc_nxbase_nx_uint32_t  ;
typedef struct { unsigned char nxdata[8]; } __attribute__((packed)) nx_uint64_t;typedef uint64_t __nesc_nxbase_nx_uint64_t  ;


typedef struct { unsigned char nxdata[1]; } __attribute__((packed)) nxle_int8_t;typedef int8_t __nesc_nxbase_nxle_int8_t  ;
typedef struct { unsigned char nxdata[2]; } __attribute__((packed)) nxle_int16_t;typedef int16_t __nesc_nxbase_nxle_int16_t  ;
typedef struct { unsigned char nxdata[4]; } __attribute__((packed)) nxle_int32_t;typedef int32_t __nesc_nxbase_nxle_int32_t  ;
typedef struct { unsigned char nxdata[8]; } __attribute__((packed)) nxle_int64_t;typedef int64_t __nesc_nxbase_nxle_int64_t  ;
typedef struct { unsigned char nxdata[1]; } __attribute__((packed)) nxle_uint8_t;typedef uint8_t __nesc_nxbase_nxle_uint8_t  ;
typedef struct { unsigned char nxdata[2]; } __attribute__((packed)) nxle_uint16_t;typedef uint16_t __nesc_nxbase_nxle_uint16_t  ;
typedef struct { unsigned char nxdata[4]; } __attribute__((packed)) nxle_uint32_t;typedef uint32_t __nesc_nxbase_nxle_uint32_t  ;
typedef struct { unsigned char nxdata[8]; } __attribute__((packed)) nxle_uint64_t;typedef uint64_t __nesc_nxbase_nxle_uint64_t  ;
# 48 "/usr/bin/../lib/gcc/msp430/4.6.3/../../../../msp430/include/sys/types.h" 3
typedef unsigned char u_char;
typedef unsigned short u_short;
typedef unsigned int u_int;
typedef unsigned long u_long;
typedef unsigned short ushort;
typedef unsigned int uint;

typedef uint8_t u_int8_t;
typedef uint16_t u_int16_t;
typedef uint32_t u_int32_t;
typedef uint64_t u_int64_t;

typedef u_int64_t u_quad_t;
typedef int64_t quad_t;
typedef quad_t *qaddr_t;

typedef char *caddr_t;
typedef const char *c_caddr_t;
typedef volatile char *v_caddr_t;
typedef u_int32_t fixpt_t;
typedef u_int32_t gid_t;
typedef u_int32_t in_addr_t;
typedef u_int16_t in_port_t;
typedef u_int32_t ino_t;
typedef long key_t;
typedef u_int16_t mode_t;
typedef u_int16_t nlink_t;
typedef quad_t rlim_t;
typedef int32_t segsz_t;
typedef int32_t swblk_t;
typedef int32_t ufs_daddr_t;
typedef int32_t ufs_time_t;
typedef u_int32_t uid_t;
# 44 "/usr/bin/../lib/gcc/msp430/4.6.3/../../../../msp430/include/string.h" 3
extern void *memset(void *arg_0x40305e90, int arg_0x40304010, size_t arg_0x403041a8);
#line 65
extern void *memset(void *arg_0x40310d30, int arg_0x40310e88, size_t arg_0x40315030);
# 62 "/usr/bin/../lib/gcc/msp430/4.6.3/../../../../msp430/include/stdlib.h" 3
#line 59
typedef struct __nesc_unnamed4242 {
  int quot;
  int rem;
} div_t;






#line 66
typedef struct __nesc_unnamed4243 {
  long int quot;
  long int rem;
} ldiv_t;
# 122 "/usr/bin/../lib/gcc/msp430/4.6.3/../../../../msp430/include/sys/config.h" 3
typedef long int __int32_t;
typedef unsigned long int __uint32_t;
# 12 "/usr/bin/../lib/gcc/msp430/4.6.3/../../../../msp430/include/sys/_types.h" 3
typedef long _off_t;
typedef long _ssize_t;
# 19 "/usr/bin/../lib/gcc/msp430/4.6.3/../../../../msp430/include/sys/reent.h" 3
typedef unsigned long __ULong;
#line 31
struct _glue {

  struct _glue *_next;
  int _niobs;
  struct __sFILE *_iobs;
};

struct _Bigint {

  struct _Bigint *_next;
  int _k, _maxwds, _sign, _wds;
  __ULong _x[1];
};


struct __tm {

  int __tm_sec;
  int __tm_min;
  int __tm_hour;
  int __tm_mday;
  int __tm_mon;
  int __tm_year;
  int __tm_wday;
  int __tm_yday;
  int __tm_isdst;
};







struct _atexit {
  struct _atexit *_next;
  int _ind;
  void (*_fns[32])(void );
};








struct __sbuf {
  unsigned char *_base;
  int _size;
};






typedef long _fpos_t;
#line 116
struct __sFILE {
  unsigned char *_p;
  int _r;
  int _w;
  short _flags;
  short _file;
  struct __sbuf _bf;
  int _lbfsize;


  void *_cookie;

  int (*_read)(void *_cookie, char *_buf, int _n);
  int (*_write)(void *_cookie, const char *_buf, int _n);

  _fpos_t (*_seek)(void *_cookie, _fpos_t _offset, int _whence);
  int (*_close)(void *_cookie);


  struct __sbuf _ub;
  unsigned char *_up;
  int _ur;


  unsigned char _ubuf[3];
  unsigned char _nbuf[1];


  struct __sbuf _lb;


  int _blksize;
  int _offset;

  struct _reent *_data;
};
#line 174
struct _rand48 {
  unsigned short _seed[3];
  unsigned short _mult[3];
  unsigned short _add;
};









struct _reent {


  int _errno;




  struct __sFILE *_stdin, *_stdout, *_stderr;

  int _inc;
  char _emergency[25];

  int _current_category;
  const char *_current_locale;

  int __sdidinit;

  void (*__cleanup)(struct _reent *arg_0x403353b8);


  struct _Bigint *_result;
  int _result_k;
  struct _Bigint *_p5s;
  struct _Bigint **_freelist;


  int _cvtlen;
  char *_cvtbuf;

  union __nesc_unnamed4244 {

    struct __nesc_unnamed4245 {

      unsigned int _unused_rand;
      char *_strtok_last;
      char _asctime_buf[26];
      struct __tm _localtime_buf;
      int _gamma_signgam;
      __extension__ unsigned long long _rand_next;
      struct _rand48 _r48;
    } _reent;



    struct __nesc_unnamed4246 {


      unsigned char *_nextf[30];
      unsigned int _nmalloc[30];
    } _unused;
  } _new;


  struct _atexit *_atexit;
  struct _atexit _atexit0;


  void (**_sig_func)(int arg_0x40338a70);




  struct _glue __sglue;
  struct __sFILE __sf[3];
};
#line 273
struct _reent;
# 18 "/usr/bin/../lib/gcc/msp430/4.6.3/../../../../msp430/include/math.h" 3
union __dmath {

  __uint32_t i[2];
  double d;
};




union __dmath;
#line 212
struct exception {


  int type;
  char *name;
  double arg1;
  double arg2;
  double retval;
  int err;
};
#line 265
enum __fdlibm_version {

  __fdlibm_ieee = -1, 
  __fdlibm_svid, 
  __fdlibm_xopen, 
  __fdlibm_posix
};




enum __fdlibm_version;
# 25 "/home/rgao/lily/tinyos2/tos/system/tos.h"
typedef uint8_t bool;
enum __nesc_unnamed4247 {
#line 26
  FALSE = 0, TRUE = 1
};
typedef nx_int8_t nx_bool;







struct __nesc_attr_atmostonce {
};
#line 37
struct __nesc_attr_atleastonce {
};
#line 38
struct __nesc_attr_exactlyonce {
};
# 51 "/home/rgao/lily/tinyos2/tos/types/TinyError.h"
enum __nesc_unnamed4248 {
  SUCCESS = 0, 
  FAIL = 1, 
  ESIZE = 2, 
  ECANCEL = 3, 
  EOFF = 4, 
  EBUSY = 5, 
  EINVAL = 6, 
  ERETRY = 7, 
  ERESERVE = 8, 
  EALREADY = 9, 
  ENOMEM = 10, 
  ENOACK = 11, 
  ELAST = 11
};

typedef uint8_t error_t  ;

static inline error_t ecombine(error_t r1, error_t r2)  ;
# 47 "/usr/bin/../lib/gcc/msp430/4.6.3/../../../../msp430/include/intrinsics.h" 3
void __nop(void );



void __dint(void );



void __eint(void );


unsigned int __read_status_register(void );


typedef unsigned int __istate_t;
# 168 "/usr/bin/../lib/gcc/msp430/4.6.3/../../../../msp430/include/msp430f2617.h" 3
extern volatile unsigned char IFG2 __asm ("__""IFG2");
#line 195
extern volatile unsigned int ADC12CTL0 __asm ("__""ADC12CTL0");

extern volatile unsigned int ADC12CTL1 __asm ("__""ADC12CTL1");
#line 454
extern volatile unsigned char DCOCTL __asm ("__""DCOCTL");

extern volatile unsigned char BCSCTL1 __asm ("__""BCSCTL1");
#line 1035
extern volatile unsigned int TACTL __asm ("__""TACTL");

extern volatile unsigned int TACCTL0 __asm ("__""TACCTL0");

extern volatile unsigned int TACCTL1 __asm ("__""TACCTL1");

extern volatile unsigned int TACCTL2 __asm ("__""TACCTL2");

extern volatile unsigned int TAR __asm ("__""TAR");





extern volatile unsigned int TACCR2 __asm ("__""TACCR2");
#line 1174
extern volatile unsigned int TBR __asm ("__""TBR");
#line 1293
extern volatile unsigned char UCA0CTL1 __asm ("__""UCA0CTL1");









extern const volatile unsigned char UCA0RXBUF __asm ("__""UCA0RXBUF");
#line 1318
extern volatile unsigned char UCB0CTL1 __asm ("__""UCB0CTL1");

extern volatile unsigned char UCB0BR0 __asm ("__""UCB0BR0");

extern volatile unsigned char UCB0BR1 __asm ("__""UCB0BR1");





extern const volatile unsigned char UCB0RXBUF __asm ("__""UCB0RXBUF");
#line 1340
extern volatile unsigned char UCA1CTL1 __asm ("__""UCA1CTL1");
#line 1365
extern volatile unsigned char UCB1CTL1 __asm ("__""UCB1CTL1");
#line 1556
extern volatile unsigned int WDTCTL __asm ("__""WDTCTL");
#line 1653
extern const volatile unsigned char CALDCO_8MHZ __asm ("__""CALDCO_8MHZ");

extern const volatile unsigned char CALBC1_8MHZ __asm ("__""CALBC1_8MHZ");
# 378 "/home/rgao/lily/tinyos2/tos/chips/msp430/msp430hardware.h"
typedef uint8_t mcu_power_t  ;
static inline mcu_power_t mcombine(mcu_power_t m1, mcu_power_t m2)  ;


enum __nesc_unnamed4249 {
  MSP430_POWER_ACTIVE = 0, 
  MSP430_POWER_LPM0 = 1, 
  MSP430_POWER_LPM1 = 2, 
  MSP430_POWER_LPM2 = 3, 
  MSP430_POWER_LPM3 = 4, 
  MSP430_POWER_LPM4 = 5
};

static inline void __nesc_disable_interrupt(void )  ;





static inline void __nesc_enable_interrupt(void )  ;




typedef bool __nesc_atomic_t;
__nesc_atomic_t __nesc_atomic_start(void );
void __nesc_atomic_end(__nesc_atomic_t reenable_interrupts);






__nesc_atomic_t __nesc_atomic_start(void )   ;







void __nesc_atomic_end(__nesc_atomic_t reenable_interrupts)   ;
#line 433
typedef struct { unsigned char nxdata[4]; } __attribute__((packed)) nx_float;typedef float __nesc_nxbase_nx_float  ;
#line 448
enum __nesc_unnamed4250 {
  MSP430_PORT_RESISTOR_INVALID, 
  MSP430_PORT_RESISTOR_OFF, 
  MSP430_PORT_RESISTOR_PULLDOWN, 
  MSP430_PORT_RESISTOR_PULLUP
};
# 55 "/home/rgao/lily/tinyos2/tos/platforms/z1/chips/msp430/timer/Msp430XDcoCalib.h"
static inline void Set_DCO(unsigned int Delta);
#line 108
static inline void Set_DCO(unsigned int Delta);
# 39 "/home/rgao/lily/tinyos2/tos/chips/msp430/timer/Msp430Timer.h"
enum __nesc_unnamed4251 {
  MSP430TIMER_CM_NONE = 0, 
  MSP430TIMER_CM_RISING = 1, 
  MSP430TIMER_CM_FALLING = 2, 
  MSP430TIMER_CM_BOTH = 3, 

  MSP430TIMER_STOP_MODE = 0, 
  MSP430TIMER_UP_MODE = 1, 
  MSP430TIMER_CONTINUOUS_MODE = 2, 
  MSP430TIMER_UPDOWN_MODE = 3, 

  MSP430TIMER_TACLK = 0, 
  MSP430TIMER_TBCLK = 0, 
  MSP430TIMER_ACLK = 1, 
  MSP430TIMER_SMCLK = 2, 
  MSP430TIMER_INCLK = 3, 

  MSP430TIMER_CLOCKDIV_1 = 0, 
  MSP430TIMER_CLOCKDIV_2 = 1, 
  MSP430TIMER_CLOCKDIV_4 = 2, 
  MSP430TIMER_CLOCKDIV_8 = 3
};
#line 75
#line 62
typedef struct __nesc_unnamed4252 {

  int ccifg : 1;
  int cov : 1;
  int out : 1;
  int cci : 1;
  int ccie : 1;
  int outmod : 3;
  int cap : 1;
  int clld : 2;
  int scs : 1;
  int ccis : 2;
  int cm : 2;
} msp430_compare_control_t;
#line 87
#line 77
typedef struct __nesc_unnamed4253 {

  int taifg : 1;
  int taie : 1;
  int taclr : 1;
  int _unused0 : 1;
  int mc : 2;
  int id : 2;
  int tassel : 2;
  int _unused1 : 6;
} msp430_timer_a_control_t;
#line 102
#line 89
typedef struct __nesc_unnamed4254 {

  int tbifg : 1;
  int tbie : 1;
  int tbclr : 1;
  int _unused0 : 1;
  int mc : 2;
  int id : 2;
  int tbssel : 2;
  int _unused1 : 1;
  int cntl : 2;
  int tbclgrp : 2;
  int _unused2 : 1;
} msp430_timer_b_control_t;
# 39 "/home/rgao/lily/tinyos2/tos/chips/cc2420/CC2420.h"
typedef uint8_t cc2420_status_t;
#line 93
#line 87
typedef nx_struct security_header_t {
  unsigned char __nesc_filler0[1];


  nx_uint32_t frameCounter;
  nx_uint8_t keyID[1];
} __attribute__((packed)) security_header_t;
#line 113
#line 95
typedef nx_struct cc2420_header_t {
  nxle_uint8_t length;
  nxle_uint16_t fcf;
  nxle_uint8_t dsn;
  nxle_uint16_t destpan;
  nxle_uint16_t dest;
  nxle_uint16_t src;







  nxle_uint8_t network;


  nxle_uint8_t type;
} __attribute__((packed)) cc2420_header_t;





#line 118
typedef nx_struct cc2420_footer_t {
} __attribute__((packed)) cc2420_footer_t;
#line 143
#line 128
typedef nx_struct cc2420_metadata_t {
  nx_uint8_t rssi;
  nx_uint8_t lqi;
  nx_uint8_t tx_power;
  nx_bool crc;
  nx_bool ack;
  nx_bool timesync;
  nx_uint32_t timestamp;
  nx_uint16_t rxInterval;
} __attribute__((packed)) 





cc2420_metadata_t;





#line 146
typedef nx_struct cc2420_packet_t {
  cc2420_header_t packet;
  nx_uint8_t data[];
} __attribute__((packed)) cc2420_packet_t;
#line 179
enum __nesc_unnamed4255 {

  MAC_HEADER_SIZE = sizeof(cc2420_header_t ) - 1, 

  MAC_FOOTER_SIZE = sizeof(uint16_t ), 

  MAC_PACKET_SIZE = MAC_HEADER_SIZE + 28 + MAC_FOOTER_SIZE, 

  CC2420_SIZE = MAC_HEADER_SIZE + MAC_FOOTER_SIZE
};

enum cc2420_enums {
  CC2420_TIME_ACK_TURNAROUND = 7, 
  CC2420_TIME_VREN = 20, 
  CC2420_TIME_SYMBOL = 2, 
  CC2420_BACKOFF_PERIOD = 20 / CC2420_TIME_SYMBOL, 
  CC2420_MIN_BACKOFF = 20 / CC2420_TIME_SYMBOL, 
  CC2420_ACK_WAIT_DELAY = 256
};

enum cc2420_status_enums {
  CC2420_STATUS_RSSI_VALID = 1 << 1, 
  CC2420_STATUS_LOCK = 1 << 2, 
  CC2420_STATUS_TX_ACTIVE = 1 << 3, 
  CC2420_STATUS_ENC_BUSY = 1 << 4, 
  CC2420_STATUS_TX_UNDERFLOW = 1 << 5, 
  CC2420_STATUS_XOSC16M_STABLE = 1 << 6
};

enum cc2420_config_reg_enums {
  CC2420_SNOP = 0x00, 
  CC2420_SXOSCON = 0x01, 
  CC2420_STXCAL = 0x02, 
  CC2420_SRXON = 0x03, 
  CC2420_STXON = 0x04, 
  CC2420_STXONCCA = 0x05, 
  CC2420_SRFOFF = 0x06, 
  CC2420_SXOSCOFF = 0x07, 
  CC2420_SFLUSHRX = 0x08, 
  CC2420_SFLUSHTX = 0x09, 
  CC2420_SACK = 0x0a, 
  CC2420_SACKPEND = 0x0b, 
  CC2420_SRXDEC = 0x0c, 
  CC2420_STXENC = 0x0d, 
  CC2420_SAES = 0x0e, 
  CC2420_MAIN = 0x10, 
  CC2420_MDMCTRL0 = 0x11, 
  CC2420_MDMCTRL1 = 0x12, 
  CC2420_RSSI = 0x13, 
  CC2420_SYNCWORD = 0x14, 
  CC2420_TXCTRL = 0x15, 
  CC2420_RXCTRL0 = 0x16, 
  CC2420_RXCTRL1 = 0x17, 
  CC2420_FSCTRL = 0x18, 
  CC2420_SECCTRL0 = 0x19, 
  CC2420_SECCTRL1 = 0x1a, 
  CC2420_BATTMON = 0x1b, 
  CC2420_IOCFG0 = 0x1c, 
  CC2420_IOCFG1 = 0x1d, 
  CC2420_MANFIDL = 0x1e, 
  CC2420_MANFIDH = 0x1f, 
  CC2420_FSMTC = 0x20, 
  CC2420_MANAND = 0x21, 
  CC2420_MANOR = 0x22, 
  CC2420_AGCCTRL = 0x23, 
  CC2420_AGCTST0 = 0x24, 
  CC2420_AGCTST1 = 0x25, 
  CC2420_AGCTST2 = 0x26, 
  CC2420_FSTST0 = 0x27, 
  CC2420_FSTST1 = 0x28, 
  CC2420_FSTST2 = 0x29, 
  CC2420_FSTST3 = 0x2a, 
  CC2420_RXBPFTST = 0x2b, 
  CC2420_FMSTATE = 0x2c, 
  CC2420_ADCTST = 0x2d, 
  CC2420_DACTST = 0x2e, 
  CC2420_TOPTST = 0x2f, 
  CC2420_TXFIFO = 0x3e, 
  CC2420_RXFIFO = 0x3f
};

enum cc2420_ram_addr_enums {
  CC2420_RAM_TXFIFO = 0x000, 
  CC2420_RAM_RXFIFO = 0x080, 
  CC2420_RAM_KEY0 = 0x100, 
  CC2420_RAM_RXNONCE = 0x110, 
  CC2420_RAM_SABUF = 0x120, 
  CC2420_RAM_KEY1 = 0x130, 
  CC2420_RAM_TXNONCE = 0x140, 
  CC2420_RAM_CBCSTATE = 0x150, 
  CC2420_RAM_IEEEADR = 0x160, 
  CC2420_RAM_PANID = 0x168, 
  CC2420_RAM_SHORTADR = 0x16a
};

enum cc2420_nonce_enums {
  CC2420_NONCE_BLOCK_COUNTER = 0, 
  CC2420_NONCE_KEY_SEQ_COUNTER = 2, 
  CC2420_NONCE_FRAME_COUNTER = 3, 
  CC2420_NONCE_SOURCE_ADDRESS = 7, 
  CC2420_NONCE_FLAGS = 15
};

enum cc2420_main_enums {
  CC2420_MAIN_RESETn = 15, 
  CC2420_MAIN_ENC_RESETn = 14, 
  CC2420_MAIN_DEMOD_RESETn = 13, 
  CC2420_MAIN_MOD_RESETn = 12, 
  CC2420_MAIN_FS_RESETn = 11, 
  CC2420_MAIN_XOSC16M_BYPASS = 0
};

enum cc2420_mdmctrl0_enums {
  CC2420_MDMCTRL0_RESERVED_FRAME_MODE = 13, 
  CC2420_MDMCTRL0_PAN_COORDINATOR = 12, 
  CC2420_MDMCTRL0_ADR_DECODE = 11, 
  CC2420_MDMCTRL0_CCA_HYST = 8, 
  CC2420_MDMCTRL0_CCA_MOD = 6, 
  CC2420_MDMCTRL0_AUTOCRC = 5, 
  CC2420_MDMCTRL0_AUTOACK = 4, 
  CC2420_MDMCTRL0_PREAMBLE_LENGTH = 0
};

enum cc2420_mdmctrl1_enums {
  CC2420_MDMCTRL1_CORR_THR = 6, 
  CC2420_MDMCTRL1_DEMOD_AVG_MODE = 5, 
  CC2420_MDMCTRL1_MODULATION_MODE = 4, 
  CC2420_MDMCTRL1_TX_MODE = 2, 
  CC2420_MDMCTRL1_RX_MODE = 0
};

enum cc2420_rssi_enums {
  CC2420_RSSI_CCA_THR = 8, 
  CC2420_RSSI_RSSI_VAL = 0
};

enum cc2420_syncword_enums {
  CC2420_SYNCWORD_SYNCWORD = 0
};

enum cc2420_txctrl_enums {
  CC2420_TXCTRL_TXMIXBUF_CUR = 14, 
  CC2420_TXCTRL_TX_TURNAROUND = 13, 
  CC2420_TXCTRL_TXMIX_CAP_ARRAY = 11, 
  CC2420_TXCTRL_TXMIX_CURRENT = 9, 
  CC2420_TXCTRL_PA_CURRENT = 6, 
  CC2420_TXCTRL_RESERVED = 5, 
  CC2420_TXCTRL_PA_LEVEL = 0
};

enum cc2420_rxctrl0_enums {
  CC2420_RXCTRL0_RXMIXBUF_CUR = 12, 
  CC2420_RXCTRL0_HIGH_LNA_GAIN = 10, 
  CC2420_RXCTRL0_MED_LNA_GAIN = 8, 
  CC2420_RXCTRL0_LOW_LNA_GAIN = 6, 
  CC2420_RXCTRL0_HIGH_LNA_CURRENT = 4, 
  CC2420_RXCTRL0_MED_LNA_CURRENT = 2, 
  CC2420_RXCTRL0_LOW_LNA_CURRENT = 0
};

enum cc2420_rxctrl1_enums {
  CC2420_RXCTRL1_RXBPF_LOCUR = 13, 
  CC2420_RXCTRL1_RXBPF_MIDCUR = 12, 
  CC2420_RXCTRL1_LOW_LOWGAIN = 11, 
  CC2420_RXCTRL1_MED_LOWGAIN = 10, 
  CC2420_RXCTRL1_HIGH_HGM = 9, 
  CC2420_RXCTRL1_MED_HGM = 8, 
  CC2420_RXCTRL1_LNA_CAP_ARRAY = 6, 
  CC2420_RXCTRL1_RXMIX_TAIL = 4, 
  CC2420_RXCTRL1_RXMIX_VCM = 2, 
  CC2420_RXCTRL1_RXMIX_CURRENT = 0
};

enum cc2420_rsctrl_enums {
  CC2420_FSCTRL_LOCK_THR = 14, 
  CC2420_FSCTRL_CAL_DONE = 13, 
  CC2420_FSCTRL_CAL_RUNNING = 12, 
  CC2420_FSCTRL_LOCK_LENGTH = 11, 
  CC2420_FSCTRL_LOCK_STATUS = 10, 
  CC2420_FSCTRL_FREQ = 0
};

enum cc2420_secctrl0_enums {
  CC2420_SECCTRL0_RXFIFO_PROTECTION = 9, 
  CC2420_SECCTRL0_SEC_CBC_HEAD = 8, 
  CC2420_SECCTRL0_SEC_SAKEYSEL = 7, 
  CC2420_SECCTRL0_SEC_TXKEYSEL = 6, 
  CC2420_SECCTRL0_SEC_RXKEYSEL = 5, 
  CC2420_SECCTRL0_SEC_M = 2, 
  CC2420_SECCTRL0_SEC_MODE = 0
};

enum cc2420_secctrl1_enums {
  CC2420_SECCTRL1_SEC_TXL = 8, 
  CC2420_SECCTRL1_SEC_RXL = 0
};

enum cc2420_battmon_enums {
  CC2420_BATTMON_BATT_OK = 6, 
  CC2420_BATTMON_BATTMON_EN = 5, 
  CC2420_BATTMON_BATTMON_VOLTAGE = 0
};

enum cc2420_iocfg0_enums {
  CC2420_IOCFG0_BCN_ACCEPT = 11, 
  CC2420_IOCFG0_FIFO_POLARITY = 10, 
  CC2420_IOCFG0_FIFOP_POLARITY = 9, 
  CC2420_IOCFG0_SFD_POLARITY = 8, 
  CC2420_IOCFG0_CCA_POLARITY = 7, 
  CC2420_IOCFG0_FIFOP_THR = 0
};

enum cc2420_iocfg1_enums {
  CC2420_IOCFG1_HSSD_SRC = 10, 
  CC2420_IOCFG1_SFDMUX = 5, 
  CC2420_IOCFG1_CCAMUX = 0
};

enum cc2420_manfidl_enums {
  CC2420_MANFIDL_PARTNUM = 12, 
  CC2420_MANFIDL_MANFID = 0
};

enum cc2420_manfidh_enums {
  CC2420_MANFIDH_VERSION = 12, 
  CC2420_MANFIDH_PARTNUM = 0
};

enum cc2420_fsmtc_enums {
  CC2420_FSMTC_TC_RXCHAIN2RX = 13, 
  CC2420_FSMTC_TC_SWITCH2TX = 10, 
  CC2420_FSMTC_TC_PAON2TX = 6, 
  CC2420_FSMTC_TC_TXEND2SWITCH = 3, 
  CC2420_FSMTC_TC_TXEND2PAOFF = 0
};

enum cc2420_sfdmux_enums {
  CC2420_SFDMUX_SFD = 0, 
  CC2420_SFDMUX_XOSC16M_STABLE = 24
};

enum cc2420_security_enums {
  CC2420_NO_SEC = 0, 
  CC2420_CBC_MAC = 1, 
  CC2420_CTR = 2, 
  CC2420_CCM = 3, 
  NO_SEC = 0, 
  CBC_MAC_4 = 1, 
  CBC_MAC_8 = 2, 
  CBC_MAC_16 = 3, 
  CTR = 4, 
  CCM_4 = 5, 
  CCM_8 = 6, 
  CCM_16 = 7
};


enum __nesc_unnamed4256 {

  CC2420_INVALID_TIMESTAMP = 0x80000000L
};
# 43 "/home/rgao/lily/tinyos2/tos/types/Leds.h"
enum __nesc_unnamed4257 {
  LEDS_LED0 = 1 << 0, 
  LEDS_LED1 = 1 << 1, 
  LEDS_LED2 = 1 << 2, 
  LEDS_LED3 = 1 << 3, 
  LEDS_LED4 = 1 << 4, 
  LEDS_LED5 = 1 << 5, 
  LEDS_LED6 = 1 << 6, 
  LEDS_LED7 = 1 << 7
};
# 90 "/home/rgao/lily/tinyos2/tos/chips/msp430/x2xxx/usci/msp430usci.h"
#line 85
typedef enum __nesc_unnamed4258 {
  USCI_NONE = 0, 
  USCI_UART = 1, 
  USCI_SPI = 2, 
  USCI_I2C = 3
} msp430_uscimode_t;
#line 111
#line 103
typedef struct __nesc_unnamed4259 {
  unsigned int ucsync : 1;
  unsigned int ucmode : 2;
  unsigned int ucspb : 1;
  unsigned int uc7bit : 1;
  unsigned int ucmsb : 1;
  unsigned int ucpar : 1;
  unsigned int ucpen : 1;
} __attribute((packed))  msp430_uctl0_t;
#line 126
#line 118
typedef struct __nesc_unnamed4260 {
  unsigned int ucswrst : 1;
  unsigned int uctxbrk : 1;
  unsigned int uctxaddr : 1;
  unsigned int ucdorm : 1;
  unsigned int ucbrkie : 1;
  unsigned int ucrxeie : 1;
  unsigned int ucssel : 2;
} __attribute((packed))  msp430_uctl1_t;
#line 185
#line 145
typedef enum __nesc_unnamed4261 {
  UBR_32KHZ_1200 = 0x001B, UMCTL_32KHZ_1200 = 0x04, 
  UBR_32KHZ_2400 = 0x000D, UMCTL_32KHZ_2400 = 0x0c, 
  UBR_32KHZ_4800 = 0x0006, UMCTL_32KHZ_4800 = 0x0e, 
  UBR_32KHZ_9600 = 0x0003, UMCTL_32KHZ_9600 = 0x06, 

  UBR_1048MHZ_9600 = 0x006D, UMCTL_1048MHZ_9600 = 0x04, 
  UBR_1048MHZ_19200 = 0x0036, UMCTL_1048MHZ_19200 = 0x0a, 
  UBR_1048MHZ_38400 = 0x001B, UMCTL_1048MHZ_38400 = 0x04, 
  UBR_1048MHZ_57600 = 0x0012, UMCTL_1048MHZ_57600 = 0x0c, 
  UBR_1048MHZ_115200 = 0x0009, UMCTL_1048MHZ_115200 = 0x02, 
  UBR_1048MHZ_128000 = 0x0008, UMCTL_1048MHZ_128000 = 0x02, 
  UBR_1048MHZ_256000 = 0x0004, UMCTL_1048MHZ_230400 = 0x02, 








  UBR_1MHZ_9600 = 0x6, UMCTL_1MHZ_9600 = 0x81, 
  UBR_1MHZ_19200 = 0x3, UMCTL_1MHZ_19200 = 0x41, 
  UBR_1MHZ_57600 = 0x1, UMCTL_1MHZ_57600 = 0x0F, 

  UBR_8MHZ_4800 = 0x68, UMCTL_8MHZ_4800 = 0x31, 
  UBR_8MHZ_9600 = 0x34, UMCTL_8MHZ_9600 = 0x11, 
  UBR_8MHZ_19200 = 0x1A, UMCTL_8MHZ_19200 = 0x11, 
  UBR_8MHZ_38400 = 0x0D, UMCTL_8MHZ_38400 = 0x01, 
  UBR_8MHZ_57600 = 0x08, UMCTL_8MHZ_57600 = 0xB1, 
  UBR_8MHZ_115200 = 0x04, UMCTL_8MHZ_115200 = 0x3B, 
  UBR_8MHZ_230400 = 0x02, UMCTL_8MHZ_230400 = 0x27, 

  UBR_16MHZ_4800 = 0xD0, UMCTL_16MHZ_4800 = 0x51, 
  UBR_16MHZ_9600 = 0x68, UMCTL_16MHZ_9600 = 0x31, 
  UBR_16MHZ_19200 = 0x34, UMCTL_16MHZ_19200 = 0x11, 
  UBR_16MHZ_38400 = 0x1A, UMCTL_16MHZ_38400 = 0x11, 
  UBR_16MHZ_57600 = 0x11, UMCTL_16MHZ_57600 = 0x61, 
  UBR_16MHZ_115200 = 0x8, UMCTL_16MHZ_115200 = 0xB1, 
  UBR_16MHZ_230400 = 0x4, UMCTL_16MHZ_230400 = 0x3B
} msp430_uart_rate_t;
#line 211
#line 188
typedef struct __nesc_unnamed4262 {
  unsigned int ubr : 16;
  unsigned int umctl : 8;


  unsigned int  : 1;
  unsigned int ucmode : 2;
  unsigned int ucspb : 1;
  unsigned int uc7bit : 1;
  unsigned int  : 1;
  unsigned int ucpar : 1;
  unsigned int ucpen : 1;


  unsigned int  : 5;
  unsigned int ucrxeie : 1;
  unsigned int ucssel : 2;




  unsigned int utxe : 1;
  unsigned int urxe : 1;
} msp430_uart_config_t;







#line 213
typedef struct __nesc_unnamed4263 {
  uint16_t ubr;
  uint8_t umctl;
  uint8_t uctl0;
  uint8_t uctl1;
  uint8_t ume;
} msp430_uart_registers_t;




#line 221
typedef union __nesc_unnamed4264 {
  msp430_uart_config_t uartConfig;
  msp430_uart_registers_t uartRegisters;
} msp430_uart_union_config_t;
#line 264
#line 248
typedef struct __nesc_unnamed4265 {
  unsigned int ubr : 16;


  unsigned int  : 1;
  unsigned int ucmode : 2;
  unsigned int ucmst : 1;
  unsigned int uc7bit : 1;
  unsigned int ucmsb : 1;
  unsigned int ucckpl : 1;
  unsigned int ucckph : 1;


  unsigned int  : 1;
  unsigned int  : 5;
  unsigned int ucssel : 2;
} msp430_spi_config_t;






#line 267
typedef struct __nesc_unnamed4266 {
  uint16_t ubr;
  uint8_t uctl0;
  uint8_t uctl1;
} msp430_spi_registers_t;




#line 273
typedef union __nesc_unnamed4267 {
  msp430_spi_config_t spiConfig;
  msp430_spi_registers_t spiRegisters;
} msp430_spi_union_config_t;
#line 305
#line 297
typedef struct __nesc_unnamed4268 {
  unsigned int  : 1;
  unsigned int ucmode : 2;
  unsigned int ucmst : 1;
  unsigned int  : 1;
  unsigned int ucmm : 1;
  unsigned int ucsla10 : 1;
  unsigned int uca10 : 1;
} __attribute((packed))  msp430_i2cctl0_t;
#line 320
#line 312
typedef struct __nesc_unnamed4269 {
  unsigned int ucswrst : 1;
  unsigned int uctxstt : 1;
  unsigned int uctxstp : 1;
  unsigned int uctxnack : 1;
  unsigned int uctr : 1;
  unsigned int  : 1;
  unsigned int ucssel : 2;
} __attribute((packed))  msp430_i2cctl1_t;
#line 348
#line 323
typedef struct __nesc_unnamed4270 {
  uint16_t ubr : 16;


  uint8_t  : 1;
  uint8_t ucmode : 2;
  uint8_t ucmst : 1;
  uint8_t  : 1;
  uint8_t ucmm : 1;
  uint8_t ucsla10 : 1;
  uint8_t uca10 : 1;


  uint8_t  : 1;
  uint8_t  : 1;
  uint8_t  : 1;
  uint8_t  : 1;
  uint8_t uctr : 1;
  uint8_t  : 1;
  uint8_t ucssel : 2;


  uint16_t i2coa : 10;
  uint8_t  : 5;
  uint8_t ucgcen : 1;
} msp430_i2c_config_t;






#line 350
typedef struct __nesc_unnamed4271 {
  uint16_t ubr;
  uint8_t uctl0;
  uint8_t uctl1;
  uint16_t ui2coa;
} msp430_i2c_registers_t;




#line 357
typedef union __nesc_unnamed4272 {
  msp430_i2c_config_t i2cConfig;
  msp430_i2c_registers_t i2cRegisters;
} msp430_i2c_union_config_t;
# 33 "/home/rgao/lily/tinyos2/tos/types/Resource.h"
typedef uint8_t resource_client_id_t;
enum /*TestModeAppC.spi1*/CC2420SpiC__0____nesc_unnamed4273 {
  CC2420SpiC__0__CLIENT_ID = 0U
};
enum /*CC2420SpiWireC.HplCC2420SpiC.SpiC*/Msp430SpiB0C__0____nesc_unnamed4274 {
  Msp430SpiB0C__0__CLIENT_ID = 0U
};
enum /*CC2420SpiWireC.HplCC2420SpiC.SpiC.UsciC*/Msp430UsciB0C__0____nesc_unnamed4275 {
  Msp430UsciB0C__0__CLIENT_ID = 1U
};
enum /*TestModeAppC.spi2*/CC2420SpiC__1____nesc_unnamed4276 {
  CC2420SpiC__1__CLIENT_ID = 1U
};
enum /*TestModeAppC.spi3*/CC2420SpiC__2____nesc_unnamed4277 {
  CC2420SpiC__2__CLIENT_ID = 2U
};
enum /*TestModeAppC.spi4*/CC2420SpiC__3____nesc_unnamed4278 {
  CC2420SpiC__3__CLIENT_ID = 3U
};
enum /*TestModeAppC.spi5*/CC2420SpiC__4____nesc_unnamed4279 {
  CC2420SpiC__4__CLIENT_ID = 4U
};
enum /*TestModeAppC.spi6*/CC2420SpiC__5____nesc_unnamed4280 {
  CC2420SpiC__5__CLIENT_ID = 5U
};
enum /*TestModeAppC.spi7*/CC2420SpiC__6____nesc_unnamed4281 {
  CC2420SpiC__6__CLIENT_ID = 6U
};
# 62 "/home/rgao/lily/tinyos2/tos/interfaces/Init.nc"
static error_t PlatformP__Init__init(void );
# 46 "/home/rgao/lily/tinyos2/tos/chips/msp430/timer/Msp430ClockInit.nc"
static void Msp430ClockP__Msp430ClockInit__defaultInitClocks(void );
#line 43
static void Msp430ClockP__Msp430ClockInit__default__initTimerB(void );



static void Msp430ClockP__Msp430ClockInit__defaultInitTimerA(void );
#line 42
static void Msp430ClockP__Msp430ClockInit__default__initTimerA(void );





static void Msp430ClockP__Msp430ClockInit__defaultInitTimerB(void );
#line 41
static void Msp430ClockP__Msp430ClockInit__default__initClocks(void );
# 62 "/home/rgao/lily/tinyos2/tos/interfaces/McuPowerOverride.nc"
static mcu_power_t Msp430ClockP__McuPowerOverride__lowestState(void );
# 62 "/home/rgao/lily/tinyos2/tos/interfaces/Init.nc"
static error_t Msp430ClockP__Init__init(void );
# 39 "/home/rgao/lily/tinyos2/tos/chips/msp430/timer/Msp430TimerEvent.nc"
static void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__VectorTimerX0__fired(void );
#line 39
static void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__Overflow__fired(void );
#line 39
static void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__VectorTimerX1__fired(void );
#line 39
static void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__Event__default__fired(
# 51 "/home/rgao/lily/tinyos2/tos/chips/msp430/timer/Msp430TimerP.nc"
uint8_t arg_0x4053d640);
# 39 "/home/rgao/lily/tinyos2/tos/chips/msp430/timer/Msp430TimerEvent.nc"
static void /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__VectorTimerX0__fired(void );
#line 39
static void /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__Overflow__fired(void );
#line 39
static void /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__VectorTimerX1__fired(void );
#line 39
static void /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__Event__default__fired(
# 51 "/home/rgao/lily/tinyos2/tos/chips/msp430/timer/Msp430TimerP.nc"
uint8_t arg_0x4053d640);
# 44 "/home/rgao/lily/tinyos2/tos/chips/msp430/timer/Msp430Capture.nc"
static uint16_t /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Capture__getEvent(void );
#line 86
static void /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Capture__default__captured(uint16_t time);
# 42 "/home/rgao/lily/tinyos2/tos/chips/msp430/timer/Msp430TimerControl.nc"
static msp430_compare_control_t /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Control__getControl(void );
# 39 "/home/rgao/lily/tinyos2/tos/chips/msp430/timer/Msp430TimerEvent.nc"
static void /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Event__fired(void );
# 45 "/home/rgao/lily/tinyos2/tos/chips/msp430/timer/Msp430Compare.nc"
static void /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Compare__default__fired(void );
# 48 "/home/rgao/lily/tinyos2/tos/chips/msp430/timer/Msp430Timer.nc"
static void /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Timer__overflow(void );
# 44 "/home/rgao/lily/tinyos2/tos/chips/msp430/timer/Msp430Capture.nc"
static uint16_t /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Capture__getEvent(void );
#line 86
static void /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Capture__default__captured(uint16_t time);
# 42 "/home/rgao/lily/tinyos2/tos/chips/msp430/timer/Msp430TimerControl.nc"
static msp430_compare_control_t /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Control__getControl(void );
# 39 "/home/rgao/lily/tinyos2/tos/chips/msp430/timer/Msp430TimerEvent.nc"
static void /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Event__fired(void );
# 45 "/home/rgao/lily/tinyos2/tos/chips/msp430/timer/Msp430Compare.nc"
static void /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Compare__default__fired(void );
# 48 "/home/rgao/lily/tinyos2/tos/chips/msp430/timer/Msp430Timer.nc"
static void /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Timer__overflow(void );
# 44 "/home/rgao/lily/tinyos2/tos/chips/msp430/timer/Msp430Capture.nc"
static uint16_t /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Capture__getEvent(void );
#line 86
static void /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Capture__default__captured(uint16_t time);
# 42 "/home/rgao/lily/tinyos2/tos/chips/msp430/timer/Msp430TimerControl.nc"
static msp430_compare_control_t /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Control__getControl(void );
# 39 "/home/rgao/lily/tinyos2/tos/chips/msp430/timer/Msp430TimerEvent.nc"
static void /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Event__fired(void );
# 45 "/home/rgao/lily/tinyos2/tos/chips/msp430/timer/Msp430Compare.nc"
static void /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Compare__default__fired(void );
# 48 "/home/rgao/lily/tinyos2/tos/chips/msp430/timer/Msp430Timer.nc"
static void /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Timer__overflow(void );
# 44 "/home/rgao/lily/tinyos2/tos/chips/msp430/timer/Msp430Capture.nc"
static uint16_t /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Capture__getEvent(void );
#line 86
static void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Capture__default__captured(uint16_t time);
# 42 "/home/rgao/lily/tinyos2/tos/chips/msp430/timer/Msp430TimerControl.nc"
static msp430_compare_control_t /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Control__getControl(void );
# 39 "/home/rgao/lily/tinyos2/tos/chips/msp430/timer/Msp430TimerEvent.nc"
static void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Event__fired(void );
# 45 "/home/rgao/lily/tinyos2/tos/chips/msp430/timer/Msp430Compare.nc"
static void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Compare__default__fired(void );
# 48 "/home/rgao/lily/tinyos2/tos/chips/msp430/timer/Msp430Timer.nc"
static void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Timer__overflow(void );
# 44 "/home/rgao/lily/tinyos2/tos/chips/msp430/timer/Msp430Capture.nc"
static uint16_t /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Capture__getEvent(void );
#line 86
static void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Capture__default__captured(uint16_t time);
# 42 "/home/rgao/lily/tinyos2/tos/chips/msp430/timer/Msp430TimerControl.nc"
static msp430_compare_control_t /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Control__getControl(void );
# 39 "/home/rgao/lily/tinyos2/tos/chips/msp430/timer/Msp430TimerEvent.nc"
static void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Event__fired(void );
# 45 "/home/rgao/lily/tinyos2/tos/chips/msp430/timer/Msp430Compare.nc"
static void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Compare__default__fired(void );
# 48 "/home/rgao/lily/tinyos2/tos/chips/msp430/timer/Msp430Timer.nc"
static void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Timer__overflow(void );
# 44 "/home/rgao/lily/tinyos2/tos/chips/msp430/timer/Msp430Capture.nc"
static uint16_t /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Capture__getEvent(void );
#line 86
static void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Capture__default__captured(uint16_t time);
# 42 "/home/rgao/lily/tinyos2/tos/chips/msp430/timer/Msp430TimerControl.nc"
static msp430_compare_control_t /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Control__getControl(void );
# 39 "/home/rgao/lily/tinyos2/tos/chips/msp430/timer/Msp430TimerEvent.nc"
static void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Event__fired(void );
# 45 "/home/rgao/lily/tinyos2/tos/chips/msp430/timer/Msp430Compare.nc"
static void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Compare__default__fired(void );
# 48 "/home/rgao/lily/tinyos2/tos/chips/msp430/timer/Msp430Timer.nc"
static void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Timer__overflow(void );
# 44 "/home/rgao/lily/tinyos2/tos/chips/msp430/timer/Msp430Capture.nc"
static uint16_t /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Capture__getEvent(void );
#line 86
static void /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Capture__default__captured(uint16_t time);
# 42 "/home/rgao/lily/tinyos2/tos/chips/msp430/timer/Msp430TimerControl.nc"
static msp430_compare_control_t /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Control__getControl(void );
# 39 "/home/rgao/lily/tinyos2/tos/chips/msp430/timer/Msp430TimerEvent.nc"
static void /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Event__fired(void );
# 45 "/home/rgao/lily/tinyos2/tos/chips/msp430/timer/Msp430Compare.nc"
static void /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Compare__default__fired(void );
# 48 "/home/rgao/lily/tinyos2/tos/chips/msp430/timer/Msp430Timer.nc"
static void /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Timer__overflow(void );
# 44 "/home/rgao/lily/tinyos2/tos/chips/msp430/timer/Msp430Capture.nc"
static uint16_t /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Capture__getEvent(void );
#line 86
static void /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Capture__default__captured(uint16_t time);
# 42 "/home/rgao/lily/tinyos2/tos/chips/msp430/timer/Msp430TimerControl.nc"
static msp430_compare_control_t /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Control__getControl(void );
# 39 "/home/rgao/lily/tinyos2/tos/chips/msp430/timer/Msp430TimerEvent.nc"
static void /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Event__fired(void );
# 45 "/home/rgao/lily/tinyos2/tos/chips/msp430/timer/Msp430Compare.nc"
static void /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Compare__default__fired(void );
# 48 "/home/rgao/lily/tinyos2/tos/chips/msp430/timer/Msp430Timer.nc"
static void /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Timer__overflow(void );
# 44 "/home/rgao/lily/tinyos2/tos/chips/msp430/timer/Msp430Capture.nc"
static uint16_t /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Capture__getEvent(void );
#line 86
static void /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Capture__default__captured(uint16_t time);
# 42 "/home/rgao/lily/tinyos2/tos/chips/msp430/timer/Msp430TimerControl.nc"
static msp430_compare_control_t /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Control__getControl(void );
# 39 "/home/rgao/lily/tinyos2/tos/chips/msp430/timer/Msp430TimerEvent.nc"
static void /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Event__fired(void );
# 45 "/home/rgao/lily/tinyos2/tos/chips/msp430/timer/Msp430Compare.nc"
static void /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Compare__default__fired(void );
# 48 "/home/rgao/lily/tinyos2/tos/chips/msp430/timer/Msp430Timer.nc"
static void /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Timer__overflow(void );
# 44 "/home/rgao/lily/tinyos2/tos/chips/msp430/timer/Msp430Capture.nc"
static uint16_t /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Capture__getEvent(void );
#line 86
static void /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Capture__default__captured(uint16_t time);
# 42 "/home/rgao/lily/tinyos2/tos/chips/msp430/timer/Msp430TimerControl.nc"
static msp430_compare_control_t /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Control__getControl(void );
# 39 "/home/rgao/lily/tinyos2/tos/chips/msp430/timer/Msp430TimerEvent.nc"
static void /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Event__fired(void );
# 45 "/home/rgao/lily/tinyos2/tos/chips/msp430/timer/Msp430Compare.nc"
static void /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Compare__default__fired(void );
# 48 "/home/rgao/lily/tinyos2/tos/chips/msp430/timer/Msp430Timer.nc"
static void /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Timer__overflow(void );
# 76 "/home/rgao/lily/tinyos2/tos/interfaces/McuSleep.nc"
static void McuSleepC__McuSleep__sleep(void );
# 67 "/home/rgao/lily/tinyos2/tos/interfaces/TaskBasic.nc"
static error_t SchedulerBasicP__TaskBasic__postTask(
# 56 "/home/rgao/lily/tinyos2/tos/system/SchedulerBasicP.nc"
uint8_t arg_0x404b1e10);
# 75 "/home/rgao/lily/tinyos2/tos/interfaces/TaskBasic.nc"
static void SchedulerBasicP__TaskBasic__default__runTask(
# 56 "/home/rgao/lily/tinyos2/tos/system/SchedulerBasicP.nc"
uint8_t arg_0x404b1e10);
# 57 "/home/rgao/lily/tinyos2/tos/interfaces/Scheduler.nc"
static void SchedulerBasicP__Scheduler__init(void );
#line 72
static void SchedulerBasicP__Scheduler__taskLoop(void );
#line 65
static bool SchedulerBasicP__Scheduler__runNextTask(void );
# 60 "/home/rgao/lily/tinyos2/tos/interfaces/Boot.nc"
static void TestModeC__Boot__booted(void );
# 92 "/home/rgao/lily/tinyos2/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
static void /*HplMsp430GeneralIOC.P31*/HplMsp430GeneralIORenP__17__IO__selectModuleFunc(void );
#line 92
static void /*HplMsp430GeneralIOC.P32*/HplMsp430GeneralIORenP__18__IO__selectModuleFunc(void );
#line 92
static void /*HplMsp430GeneralIOC.P33*/HplMsp430GeneralIORenP__19__IO__selectModuleFunc(void );
#line 85
static void /*HplMsp430GeneralIOC.P54*/HplMsp430GeneralIORenP__36__IO__makeOutput(void );
#line 48
static void /*HplMsp430GeneralIOC.P54*/HplMsp430GeneralIORenP__36__IO__set(void );
#line 85
static void /*HplMsp430GeneralIOC.P55*/HplMsp430GeneralIORenP__37__IO__makeOutput(void );
#line 48
static void /*HplMsp430GeneralIOC.P55*/HplMsp430GeneralIORenP__37__IO__set(void );
#line 85
static void /*HplMsp430GeneralIOC.P56*/HplMsp430GeneralIORenP__38__IO__makeOutput(void );
#line 48
static void /*HplMsp430GeneralIOC.P56*/HplMsp430GeneralIORenP__38__IO__set(void );
# 82 "/home/rgao/lily/tinyos2/tos/interfaces/SpiPacket.nc"
static void CC2420SpiP__SpiPacket__sendDone(
#line 75
uint8_t * txBuf, 
uint8_t * rxBuf, 





uint16_t len, 
error_t error);
# 91 "/home/rgao/lily/tinyos2/tos/chips/cc2420/interfaces/CC2420Fifo.nc"
static void CC2420SpiP__Fifo__default__writeDone(
# 46 "/home/rgao/lily/tinyos2/tos/chips/cc2420/spi/CC2420SpiP.nc"
uint8_t arg_0x407893f8, 
# 91 "/home/rgao/lily/tinyos2/tos/chips/cc2420/interfaces/CC2420Fifo.nc"
uint8_t * data, uint8_t length, error_t error);
#line 71
static void CC2420SpiP__Fifo__default__readDone(
# 46 "/home/rgao/lily/tinyos2/tos/chips/cc2420/spi/CC2420SpiP.nc"
uint8_t arg_0x407893f8, 
# 71 "/home/rgao/lily/tinyos2/tos/chips/cc2420/interfaces/CC2420Fifo.nc"
uint8_t * data, uint8_t length, error_t error);
# 102 "/home/rgao/lily/tinyos2/tos/interfaces/Resource.nc"
static void CC2420SpiP__SpiResource__granted(void );
# 63 "/home/rgao/lily/tinyos2/tos/chips/cc2420/interfaces/CC2420Register.nc"
static cc2420_status_t CC2420SpiP__Reg__write(
# 48 "/home/rgao/lily/tinyos2/tos/chips/cc2420/spi/CC2420SpiP.nc"
uint8_t arg_0x407875e8, 
# 63 "/home/rgao/lily/tinyos2/tos/chips/cc2420/interfaces/CC2420Register.nc"
uint16_t data);
# 102 "/home/rgao/lily/tinyos2/tos/interfaces/Resource.nc"
static void CC2420SpiP__Resource__default__granted(
# 45 "/home/rgao/lily/tinyos2/tos/chips/cc2420/spi/CC2420SpiP.nc"
uint8_t arg_0x4078a920);
# 75 "/home/rgao/lily/tinyos2/tos/interfaces/TaskBasic.nc"
static void CC2420SpiP__grant__runTask(void );
# 53 "/home/rgao/lily/tinyos2/tos/chips/cc2420/interfaces/CC2420Strobe.nc"
static cc2420_status_t CC2420SpiP__Strobe__strobe(
# 49 "/home/rgao/lily/tinyos2/tos/chips/cc2420/spi/CC2420SpiP.nc"
uint8_t arg_0x40787da0);
# 62 "/home/rgao/lily/tinyos2/tos/interfaces/Init.nc"
static error_t StateImplP__Init__init(void );
# 66 "/home/rgao/lily/tinyos2/tos/interfaces/State.nc"
static bool StateImplP__State__isState(
# 67 "/home/rgao/lily/tinyos2/tos/system/StateImplP.nc"
uint8_t arg_0x407e23b8, 
# 66 "/home/rgao/lily/tinyos2/tos/interfaces/State.nc"
uint8_t myState);
#line 61
static bool StateImplP__State__isIdle(
# 67 "/home/rgao/lily/tinyos2/tos/system/StateImplP.nc"
uint8_t arg_0x407e23b8);
# 59 "/home/rgao/lily/tinyos2/tos/interfaces/ResourceConfigure.nc"
static void /*Msp430SpiNoDmaB0P.SpiP*/Msp430SpiNoDmaBP__0__ResourceConfigure__configure(
# 44 "/home/rgao/lily/tinyos2/tos/chips/msp430/x2xxx/usci/Msp430SpiNoDmaBP.nc"
uint8_t arg_0x4082f5d0);
# 75 "/home/rgao/lily/tinyos2/tos/interfaces/TaskBasic.nc"
static void /*Msp430SpiNoDmaB0P.SpiP*/Msp430SpiNoDmaBP__0__signalDone_task__runTask(void );
# 82 "/home/rgao/lily/tinyos2/tos/interfaces/SpiPacket.nc"
static void /*Msp430SpiNoDmaB0P.SpiP*/Msp430SpiNoDmaBP__0__SpiPacket__default__sendDone(
# 46 "/home/rgao/lily/tinyos2/tos/chips/msp430/x2xxx/usci/Msp430SpiNoDmaBP.nc"
uint8_t arg_0x4082e0c0, 
# 75 "/home/rgao/lily/tinyos2/tos/interfaces/SpiPacket.nc"
uint8_t * txBuf, 
uint8_t * rxBuf, 





uint16_t len, 
error_t error);
# 45 "/home/rgao/lily/tinyos2/tos/interfaces/SpiByte.nc"
static uint8_t /*Msp430SpiNoDmaB0P.SpiP*/Msp430SpiNoDmaBP__0__SpiByte__write(uint8_t tx);
# 59 "/home/rgao/lily/tinyos2/tos/chips/msp430/x2xxx/usci/HplMsp430UsciInterrupts.nc"
static void /*Msp430SpiNoDmaB0P.SpiP*/Msp430SpiNoDmaBP__0__UsciInterrupts__rxDone(uint8_t data);
#line 54
static void /*Msp430SpiNoDmaB0P.SpiP*/Msp430SpiNoDmaBP__0__UsciInterrupts__txDone(void );
# 102 "/home/rgao/lily/tinyos2/tos/interfaces/Resource.nc"
static void /*Msp430SpiNoDmaB0P.SpiP*/Msp430SpiNoDmaBP__0__Resource__default__granted(
# 43 "/home/rgao/lily/tinyos2/tos/chips/msp430/x2xxx/usci/Msp430SpiNoDmaBP.nc"
uint8_t arg_0x40819ad8);
# 102 "/home/rgao/lily/tinyos2/tos/interfaces/Resource.nc"
static void /*Msp430SpiNoDmaB0P.SpiP*/Msp430SpiNoDmaBP__0__UsciResource__granted(
# 48 "/home/rgao/lily/tinyos2/tos/chips/msp430/x2xxx/usci/Msp430SpiNoDmaBP.nc"
uint8_t arg_0x4082e8b0);
# 45 "/home/rgao/lily/tinyos2/tos/chips/msp430/x2xxx/usci/Msp430SpiConfigure.nc"
static msp430_spi_union_config_t */*Msp430SpiNoDmaB0P.Z1UsciP*/Z1UsciP__0__Msp430SpiConfigure__getConfig(
# 41 "/home/rgao/lily/tinyos2/tos/platforms/z1/chips/msp430/usci/Z1UsciP.nc"
uint8_t arg_0x40869498);
# 58 "/home/rgao/lily/tinyos2/tos/chips/msp430/x2xxx/usci/HplMsp430UsciRawInterrupts.nc"
static void HplMsp430UsciB0P__UsciRawInterrupts__rxDone(uint8_t data);
#line 53
static void HplMsp430UsciB0P__UsciRawInterrupts__txDone(void );
# 81 "/home/rgao/lily/tinyos2/tos/chips/msp430/x2xxx/usci/HplMsp430UsciB.nc"
static void HplMsp430UsciB0P__Usci__resetUsci(bool reset);
#line 100
static void HplMsp430UsciB0P__Usci__clrRxIntr(void );
#line 92
static void HplMsp430UsciB0P__Usci__disableIntr(void );
#line 90
static void HplMsp430UsciB0P__Usci__disableRxIntr(void );










static void HplMsp430UsciB0P__Usci__clrIntr(void );
#line 64
static void HplMsp430UsciB0P__Usci__setUbr(uint16_t ubr);
#line 108
static void HplMsp430UsciB0P__Usci__tx(uint8_t data);
#line 129
static void HplMsp430UsciB0P__Usci__enableSpi(void );
#line 115
static uint8_t HplMsp430UsciB0P__Usci__rx(void );
#line 141
static void HplMsp430UsciB0P__Usci__setModeSpi(msp430_spi_union_config_t *config);
#line 98
static bool HplMsp430UsciB0P__Usci__isRxIntrPending(void );
# 58 "/home/rgao/lily/tinyos2/tos/chips/msp430/x2xxx/usci/HplMsp430UsciRawInterrupts.nc"
static void HplMsp430UsciAB0RawInterruptsP__UsciA__default__rxDone(uint8_t data);
#line 53
static void HplMsp430UsciAB0RawInterruptsP__UsciA__default__txDone(void );
# 62 "/home/rgao/lily/tinyos2/tos/interfaces/Init.nc"
static error_t LedsP__Init__init(void );
# 46 "/home/rgao/lily/tinyos2/tos/interfaces/GeneralIO.nc"
static void /*PlatformLedsC.Led0Impl*/Msp430GpioC__7__GeneralIO__makeOutput(void );
#line 40
static void /*PlatformLedsC.Led0Impl*/Msp430GpioC__7__GeneralIO__set(void );





static void /*PlatformLedsC.Led1Impl*/Msp430GpioC__8__GeneralIO__makeOutput(void );
#line 40
static void /*PlatformLedsC.Led1Impl*/Msp430GpioC__8__GeneralIO__set(void );





static void /*PlatformLedsC.Led2Impl*/Msp430GpioC__9__GeneralIO__makeOutput(void );
#line 40
static void /*PlatformLedsC.Led2Impl*/Msp430GpioC__9__GeneralIO__set(void );
# 59 "/home/rgao/lily/tinyos2/tos/chips/msp430/x2xxx/usci/HplMsp430UsciInterrupts.nc"
static void /*Msp430UsciShareB0P.UsciShareP*/Msp430UsciShareP__0__Interrupts__default__rxDone(
# 41 "/home/rgao/lily/tinyos2/tos/chips/msp430/x2xxx/usci/Msp430UsciShareP.nc"
uint8_t arg_0x40916cf8, 
# 59 "/home/rgao/lily/tinyos2/tos/chips/msp430/x2xxx/usci/HplMsp430UsciInterrupts.nc"
uint8_t data);
#line 54
static void /*Msp430UsciShareB0P.UsciShareP*/Msp430UsciShareP__0__Interrupts__default__txDone(
# 41 "/home/rgao/lily/tinyos2/tos/chips/msp430/x2xxx/usci/Msp430UsciShareP.nc"
uint8_t arg_0x40916cf8);
# 59 "/home/rgao/lily/tinyos2/tos/chips/msp430/x2xxx/usci/HplMsp430UsciInterrupts.nc"
static void /*Msp430UsciShareB0P.UsciShareP*/Msp430UsciShareP__0__RawInterrupts__rxDone(uint8_t data);
#line 54
static void /*Msp430UsciShareB0P.UsciShareP*/Msp430UsciShareP__0__RawInterrupts__txDone(void );
# 62 "/home/rgao/lily/tinyos2/tos/interfaces/Init.nc"
static error_t /*Msp430UsciShareB0P.ArbiterC.Queue*/FcfsResourceQueueC__0__Init__init(void );
# 59 "/home/rgao/lily/tinyos2/tos/interfaces/ResourceConfigure.nc"
static void /*Msp430UsciShareB0P.ArbiterC.Arbiter*/ArbiterP__0__ResourceConfigure__default__configure(
# 60 "/home/rgao/lily/tinyos2/tos/system/ArbiterP.nc"
uint8_t arg_0x4092c030);
# 102 "/home/rgao/lily/tinyos2/tos/interfaces/Resource.nc"
static void /*Msp430UsciShareB0P.ArbiterC.Arbiter*/ArbiterP__0__Resource__default__granted(
# 54 "/home/rgao/lily/tinyos2/tos/system/ArbiterP.nc"
uint8_t arg_0x4092e1b8);
# 90 "/home/rgao/lily/tinyos2/tos/interfaces/ArbiterInfo.nc"
static bool /*Msp430UsciShareB0P.ArbiterC.Arbiter*/ArbiterP__0__ArbiterInfo__inUse(void );







static uint8_t /*Msp430UsciShareB0P.ArbiterC.Arbiter*/ArbiterP__0__ArbiterInfo__userId(void );
# 75 "/home/rgao/lily/tinyos2/tos/interfaces/TaskBasic.nc"
static void /*Msp430UsciShareB0P.ArbiterC.Arbiter*/ArbiterP__0__grantedTask__runTask(void );
# 62 "/home/rgao/lily/tinyos2/tos/interfaces/Init.nc"
static error_t PlatformP__Msp430ClockInit__init(void );
#line 62
static error_t PlatformP__LedsInit__init(void );
# 50 "/home/rgao/lily/tinyos2/tos/platforms/z1/PlatformP.nc"
static inline error_t PlatformP__Init__init(void );
# 43 "/home/rgao/lily/tinyos2/tos/chips/msp430/timer/Msp430ClockInit.nc"
static void Msp430ClockP__Msp430ClockInit__initTimerB(void );
#line 42
static void Msp430ClockP__Msp430ClockInit__initTimerA(void );
#line 41
static void Msp430ClockP__Msp430ClockInit__initClocks(void );
# 54 "/home/rgao/lily/tinyos2/tos/platforms/z1/chips/msp430/timer/Msp430ClockP.nc"
static volatile uint8_t Msp430ClockP__IE1 __asm ("0x0000");
static volatile uint16_t Msp430ClockP__TACTL __asm ("0x0160");
static volatile uint16_t Msp430ClockP__TAIV __asm ("0x012E");
static volatile uint16_t Msp430ClockP__TBCTL __asm ("0x0180");
static volatile uint16_t Msp430ClockP__TBIV __asm ("0x011E");

enum Msp430ClockP____nesc_unnamed4282 {

  Msp430ClockP__ACLK_CALIB_PERIOD = 8, 
  Msp430ClockP__TARGET_DCO_DELTA = 4096 / 32 * Msp430ClockP__ACLK_CALIB_PERIOD
};

static inline mcu_power_t Msp430ClockP__McuPowerOverride__lowestState(void );
#line 82
static inline void Msp430ClockP__Msp430ClockInit__defaultInitClocks(void );
#line 117
static inline void Msp430ClockP__Msp430ClockInit__defaultInitTimerA(void );
#line 132
static inline void Msp430ClockP__Msp430ClockInit__defaultInitTimerB(void );
#line 152
static inline void Msp430ClockP__Msp430ClockInit__default__initClocks(void );




static inline void Msp430ClockP__Msp430ClockInit__default__initTimerA(void );




static inline void Msp430ClockP__Msp430ClockInit__default__initTimerB(void );





static inline void Msp430ClockP__startTimerA(void );
#line 180
static inline void Msp430ClockP__startTimerB(void );
#line 246
static inline error_t Msp430ClockP__Init__init(void );
# 39 "/home/rgao/lily/tinyos2/tos/chips/msp430/timer/Msp430TimerEvent.nc"
static void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__Event__fired(
# 51 "/home/rgao/lily/tinyos2/tos/chips/msp430/timer/Msp430TimerP.nc"
uint8_t arg_0x4053d640);
# 48 "/home/rgao/lily/tinyos2/tos/chips/msp430/timer/Msp430Timer.nc"
static void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__Timer__overflow(void );
# 126 "/home/rgao/lily/tinyos2/tos/chips/msp430/timer/Msp430TimerP.nc"
static inline void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__VectorTimerX0__fired(void );




static inline void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__VectorTimerX1__fired(void );





static inline void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__Overflow__fired(void );








static inline void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__Event__default__fired(uint8_t n);
# 39 "/home/rgao/lily/tinyos2/tos/chips/msp430/timer/Msp430TimerEvent.nc"
static void /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__Event__fired(
# 51 "/home/rgao/lily/tinyos2/tos/chips/msp430/timer/Msp430TimerP.nc"
uint8_t arg_0x4053d640);
# 48 "/home/rgao/lily/tinyos2/tos/chips/msp430/timer/Msp430Timer.nc"
static void /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__Timer__overflow(void );
# 126 "/home/rgao/lily/tinyos2/tos/chips/msp430/timer/Msp430TimerP.nc"
static inline void /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__VectorTimerX0__fired(void );




static inline void /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__VectorTimerX1__fired(void );





static inline void /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__Overflow__fired(void );








static void /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__Event__default__fired(uint8_t n);
# 86 "/home/rgao/lily/tinyos2/tos/chips/msp430/timer/Msp430Capture.nc"
static void /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Capture__captured(uint16_t time);
# 45 "/home/rgao/lily/tinyos2/tos/chips/msp430/timer/Msp430Compare.nc"
static void /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Compare__fired(void );
# 55 "/home/rgao/lily/tinyos2/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
typedef msp430_compare_control_t /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__cc_t;


static inline /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__cc_t /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__int2CC(uint16_t x)  ;
#line 85
static inline /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__cc_t /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Control__getControl(void );
#line 150
static inline uint16_t /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Capture__getEvent(void );
#line 180
static void /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Event__fired(void );







static inline void /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Capture__default__captured(uint16_t n);



static inline void /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Compare__default__fired(void );



static inline void /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Timer__overflow(void );
# 86 "/home/rgao/lily/tinyos2/tos/chips/msp430/timer/Msp430Capture.nc"
static void /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Capture__captured(uint16_t time);
# 45 "/home/rgao/lily/tinyos2/tos/chips/msp430/timer/Msp430Compare.nc"
static void /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Compare__fired(void );
# 55 "/home/rgao/lily/tinyos2/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
typedef msp430_compare_control_t /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__cc_t;


static inline /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__cc_t /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__int2CC(uint16_t x)  ;
#line 85
static inline /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__cc_t /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Control__getControl(void );
#line 150
static inline uint16_t /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Capture__getEvent(void );
#line 180
static void /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Event__fired(void );







static inline void /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Capture__default__captured(uint16_t n);



static inline void /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Compare__default__fired(void );



static inline void /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Timer__overflow(void );
# 86 "/home/rgao/lily/tinyos2/tos/chips/msp430/timer/Msp430Capture.nc"
static void /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Capture__captured(uint16_t time);
# 45 "/home/rgao/lily/tinyos2/tos/chips/msp430/timer/Msp430Compare.nc"
static void /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Compare__fired(void );
# 55 "/home/rgao/lily/tinyos2/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
typedef msp430_compare_control_t /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__cc_t;


static inline /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__cc_t /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__int2CC(uint16_t x)  ;
#line 85
static inline /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__cc_t /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Control__getControl(void );
#line 150
static inline uint16_t /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Capture__getEvent(void );
#line 180
static void /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Event__fired(void );







static inline void /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Capture__default__captured(uint16_t n);



static inline void /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Compare__default__fired(void );



static inline void /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Timer__overflow(void );
# 86 "/home/rgao/lily/tinyos2/tos/chips/msp430/timer/Msp430Capture.nc"
static void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Capture__captured(uint16_t time);
# 45 "/home/rgao/lily/tinyos2/tos/chips/msp430/timer/Msp430Compare.nc"
static void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Compare__fired(void );
# 55 "/home/rgao/lily/tinyos2/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
typedef msp430_compare_control_t /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__cc_t;


static inline /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__cc_t /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__int2CC(uint16_t x)  ;
#line 85
static inline /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__cc_t /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Control__getControl(void );
#line 150
static inline uint16_t /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Capture__getEvent(void );
#line 180
static inline void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Event__fired(void );







static inline void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Capture__default__captured(uint16_t n);



static inline void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Compare__default__fired(void );



static inline void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Timer__overflow(void );
# 86 "/home/rgao/lily/tinyos2/tos/chips/msp430/timer/Msp430Capture.nc"
static void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Capture__captured(uint16_t time);
# 45 "/home/rgao/lily/tinyos2/tos/chips/msp430/timer/Msp430Compare.nc"
static void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Compare__fired(void );
# 55 "/home/rgao/lily/tinyos2/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
typedef msp430_compare_control_t /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__cc_t;


static inline /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__cc_t /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__int2CC(uint16_t x)  ;
#line 85
static inline /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__cc_t /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Control__getControl(void );
#line 150
static inline uint16_t /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Capture__getEvent(void );
#line 180
static inline void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Event__fired(void );







static inline void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Capture__default__captured(uint16_t n);



static inline void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Compare__default__fired(void );



static inline void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Timer__overflow(void );
# 86 "/home/rgao/lily/tinyos2/tos/chips/msp430/timer/Msp430Capture.nc"
static void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Capture__captured(uint16_t time);
# 45 "/home/rgao/lily/tinyos2/tos/chips/msp430/timer/Msp430Compare.nc"
static void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Compare__fired(void );
# 55 "/home/rgao/lily/tinyos2/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
typedef msp430_compare_control_t /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__cc_t;


static inline /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__cc_t /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__int2CC(uint16_t x)  ;
#line 85
static inline /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__cc_t /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Control__getControl(void );
#line 150
static inline uint16_t /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Capture__getEvent(void );
#line 180
static inline void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Event__fired(void );







static inline void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Capture__default__captured(uint16_t n);



static inline void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Compare__default__fired(void );



static inline void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Timer__overflow(void );
# 86 "/home/rgao/lily/tinyos2/tos/chips/msp430/timer/Msp430Capture.nc"
static void /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Capture__captured(uint16_t time);
# 45 "/home/rgao/lily/tinyos2/tos/chips/msp430/timer/Msp430Compare.nc"
static void /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Compare__fired(void );
# 55 "/home/rgao/lily/tinyos2/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
typedef msp430_compare_control_t /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__cc_t;


static inline /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__cc_t /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__int2CC(uint16_t x)  ;
#line 85
static inline /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__cc_t /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Control__getControl(void );
#line 150
static inline uint16_t /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Capture__getEvent(void );
#line 180
static inline void /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Event__fired(void );







static inline void /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Capture__default__captured(uint16_t n);



static inline void /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Compare__default__fired(void );



static inline void /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Timer__overflow(void );
# 86 "/home/rgao/lily/tinyos2/tos/chips/msp430/timer/Msp430Capture.nc"
static void /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Capture__captured(uint16_t time);
# 45 "/home/rgao/lily/tinyos2/tos/chips/msp430/timer/Msp430Compare.nc"
static void /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Compare__fired(void );
# 55 "/home/rgao/lily/tinyos2/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
typedef msp430_compare_control_t /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__cc_t;


static inline /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__cc_t /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__int2CC(uint16_t x)  ;
#line 85
static inline /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__cc_t /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Control__getControl(void );
#line 150
static inline uint16_t /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Capture__getEvent(void );
#line 180
static inline void /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Event__fired(void );







static inline void /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Capture__default__captured(uint16_t n);



static inline void /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Compare__default__fired(void );



static inline void /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Timer__overflow(void );
# 86 "/home/rgao/lily/tinyos2/tos/chips/msp430/timer/Msp430Capture.nc"
static void /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Capture__captured(uint16_t time);
# 45 "/home/rgao/lily/tinyos2/tos/chips/msp430/timer/Msp430Compare.nc"
static void /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Compare__fired(void );
# 55 "/home/rgao/lily/tinyos2/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
typedef msp430_compare_control_t /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__cc_t;


static inline /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__cc_t /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__int2CC(uint16_t x)  ;
#line 85
static inline /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__cc_t /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Control__getControl(void );
#line 150
static inline uint16_t /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Capture__getEvent(void );
#line 180
static inline void /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Event__fired(void );







static inline void /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Capture__default__captured(uint16_t n);



static inline void /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Compare__default__fired(void );



static inline void /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Timer__overflow(void );
# 86 "/home/rgao/lily/tinyos2/tos/chips/msp430/timer/Msp430Capture.nc"
static void /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Capture__captured(uint16_t time);
# 45 "/home/rgao/lily/tinyos2/tos/chips/msp430/timer/Msp430Compare.nc"
static void /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Compare__fired(void );
# 55 "/home/rgao/lily/tinyos2/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
typedef msp430_compare_control_t /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__cc_t;


static inline /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__cc_t /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__int2CC(uint16_t x)  ;
#line 85
static inline /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__cc_t /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Control__getControl(void );
#line 150
static inline uint16_t /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Capture__getEvent(void );
#line 180
static inline void /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Event__fired(void );







static inline void /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Capture__default__captured(uint16_t n);



static inline void /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Compare__default__fired(void );



static inline void /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Timer__overflow(void );
# 39 "/home/rgao/lily/tinyos2/tos/chips/msp430/timer/Msp430TimerEvent.nc"
static void Msp430TimerCommonP__VectorTimerB1__fired(void );
#line 39
static void Msp430TimerCommonP__VectorTimerA0__fired(void );
#line 39
static void Msp430TimerCommonP__VectorTimerA1__fired(void );
#line 39
static void Msp430TimerCommonP__VectorTimerB0__fired(void );
# 11 "/home/rgao/lily/tinyos2/tos/chips/msp430/timer/Msp430TimerCommonP.nc"
void sig_TIMERA0_VECTOR(void ) __attribute((wakeup)) __attribute((interrupt(0x0032)))  ;
void sig_TIMERA1_VECTOR(void ) __attribute((wakeup)) __attribute((interrupt(0x0030)))  ;
void sig_TIMERB0_VECTOR(void ) __attribute((wakeup)) __attribute((interrupt(0x003A)))  ;
void sig_TIMERB1_VECTOR(void ) __attribute((wakeup)) __attribute((interrupt(0x0038)))  ;
# 62 "/home/rgao/lily/tinyos2/tos/interfaces/McuPowerOverride.nc"
static mcu_power_t McuSleepC__McuPowerOverride__lowestState(void );
# 59 "/home/rgao/lily/tinyos2/tos/platforms/z1/chips/msp430/McuSleepC.nc"
bool McuSleepC__dirty = TRUE;
mcu_power_t McuSleepC__powerState = MSP430_POWER_ACTIVE;




const uint16_t McuSleepC__msp430PowerBits[MSP430_POWER_LPM4 + 1] = { 
0, 
0x0010, 
0x0040 + 0x0010, 
0x0080 + 0x0010, 
0x0080 + 0x0040 + 0x0010, 
0x0080 + 0x0040 + 0x0020 + 0x0010 };


static inline mcu_power_t McuSleepC__getPowerState(void );
#line 107
static inline void McuSleepC__computePowerState(void );




static inline void McuSleepC__McuSleep__sleep(void );
# 62 "/home/rgao/lily/tinyos2/tos/interfaces/Init.nc"
static error_t RealMainP__SoftwareInit__init(void );
# 60 "/home/rgao/lily/tinyos2/tos/interfaces/Boot.nc"
static void RealMainP__Boot__booted(void );
# 62 "/home/rgao/lily/tinyos2/tos/interfaces/Init.nc"
static error_t RealMainP__PlatformInit__init(void );
# 57 "/home/rgao/lily/tinyos2/tos/interfaces/Scheduler.nc"
static void RealMainP__Scheduler__init(void );
#line 72
static void RealMainP__Scheduler__taskLoop(void );
#line 65
static bool RealMainP__Scheduler__runNextTask(void );
# 63 "/home/rgao/lily/tinyos2/tos/system/RealMainP.nc"
int main(void )   ;
# 75 "/home/rgao/lily/tinyos2/tos/interfaces/TaskBasic.nc"
static void SchedulerBasicP__TaskBasic__runTask(
# 56 "/home/rgao/lily/tinyos2/tos/system/SchedulerBasicP.nc"
uint8_t arg_0x404b1e10);
# 76 "/home/rgao/lily/tinyos2/tos/interfaces/McuSleep.nc"
static void SchedulerBasicP__McuSleep__sleep(void );
# 61 "/home/rgao/lily/tinyos2/tos/system/SchedulerBasicP.nc"
enum SchedulerBasicP____nesc_unnamed4283 {

  SchedulerBasicP__NUM_TASKS = 3U, 
  SchedulerBasicP__NO_TASK = 255
};

uint8_t SchedulerBasicP__m_head;
uint8_t SchedulerBasicP__m_tail;
uint8_t SchedulerBasicP__m_next[SchedulerBasicP__NUM_TASKS];








static __inline uint8_t SchedulerBasicP__popTask(void );
#line 97
static inline bool SchedulerBasicP__isWaiting(uint8_t id);




static inline bool SchedulerBasicP__pushTask(uint8_t id);
#line 124
static inline void SchedulerBasicP__Scheduler__init(void );









static bool SchedulerBasicP__Scheduler__runNextTask(void );
#line 149
static inline void SchedulerBasicP__Scheduler__taskLoop(void );
#line 170
static inline error_t SchedulerBasicP__TaskBasic__postTask(uint8_t id);




static inline void SchedulerBasicP__TaskBasic__default__runTask(uint8_t id);
# 63 "/home/rgao/lily/tinyos2/tos/chips/cc2420/interfaces/CC2420Register.nc"
static cc2420_status_t TestModeC__dactst__write(uint16_t data);
#line 63
static cc2420_status_t TestModeC__mdmctrl1__write(uint16_t data);
# 53 "/home/rgao/lily/tinyos2/tos/chips/cc2420/interfaces/CC2420Strobe.nc"
static cc2420_status_t TestModeC__SXOSCON__strobe(void );
# 63 "/home/rgao/lily/tinyos2/tos/chips/cc2420/interfaces/CC2420Register.nc"
static cc2420_status_t TestModeC__MAIN__write(uint16_t data);
# 53 "/home/rgao/lily/tinyos2/tos/chips/cc2420/interfaces/CC2420Strobe.nc"
static cc2420_status_t TestModeC__stxccaon__strobe(void );
# 63 "/home/rgao/lily/tinyos2/tos/chips/cc2420/interfaces/CC2420Register.nc"
static cc2420_status_t TestModeC__manor__write(uint16_t data);
#line 63
static cc2420_status_t TestModeC__toptst__write(uint16_t data);
# 23 "TestModeC.nc"
static inline void TestModeC__Boot__booted(void );
# 57 "/home/rgao/lily/tinyos2/tos/chips/msp430/pins/HplMsp430GeneralIORenP.nc"
static inline void /*HplMsp430GeneralIOC.P31*/HplMsp430GeneralIORenP__17__IO__selectModuleFunc(void );
#line 57
static inline void /*HplMsp430GeneralIOC.P32*/HplMsp430GeneralIORenP__18__IO__selectModuleFunc(void );
#line 57
static inline void /*HplMsp430GeneralIOC.P33*/HplMsp430GeneralIORenP__19__IO__selectModuleFunc(void );
#line 48
static inline void /*HplMsp430GeneralIOC.P54*/HplMsp430GeneralIORenP__36__IO__set(void );






static inline void /*HplMsp430GeneralIOC.P54*/HplMsp430GeneralIORenP__36__IO__makeOutput(void );
#line 48
static inline void /*HplMsp430GeneralIOC.P55*/HplMsp430GeneralIORenP__37__IO__set(void );






static inline void /*HplMsp430GeneralIOC.P55*/HplMsp430GeneralIORenP__37__IO__makeOutput(void );
#line 48
static inline void /*HplMsp430GeneralIOC.P56*/HplMsp430GeneralIORenP__38__IO__set(void );






static inline void /*HplMsp430GeneralIOC.P56*/HplMsp430GeneralIORenP__38__IO__makeOutput(void );
# 91 "/home/rgao/lily/tinyos2/tos/chips/cc2420/interfaces/CC2420Fifo.nc"
static void CC2420SpiP__Fifo__writeDone(
# 46 "/home/rgao/lily/tinyos2/tos/chips/cc2420/spi/CC2420SpiP.nc"
uint8_t arg_0x407893f8, 
# 91 "/home/rgao/lily/tinyos2/tos/chips/cc2420/interfaces/CC2420Fifo.nc"
uint8_t * data, uint8_t length, error_t error);
#line 71
static void CC2420SpiP__Fifo__readDone(
# 46 "/home/rgao/lily/tinyos2/tos/chips/cc2420/spi/CC2420SpiP.nc"
uint8_t arg_0x407893f8, 
# 71 "/home/rgao/lily/tinyos2/tos/chips/cc2420/interfaces/CC2420Fifo.nc"
uint8_t * data, uint8_t length, error_t error);
# 45 "/home/rgao/lily/tinyos2/tos/interfaces/SpiByte.nc"
static uint8_t CC2420SpiP__SpiByte__write(uint8_t tx);
# 61 "/home/rgao/lily/tinyos2/tos/interfaces/State.nc"
static bool CC2420SpiP__WorkingState__isIdle(void );
# 102 "/home/rgao/lily/tinyos2/tos/interfaces/Resource.nc"
static void CC2420SpiP__Resource__granted(
# 45 "/home/rgao/lily/tinyos2/tos/chips/cc2420/spi/CC2420SpiP.nc"
uint8_t arg_0x4078a920);
# 67 "/home/rgao/lily/tinyos2/tos/interfaces/TaskBasic.nc"
static error_t CC2420SpiP__grant__postTask(void );
# 88 "/home/rgao/lily/tinyos2/tos/chips/cc2420/spi/CC2420SpiP.nc"
enum CC2420SpiP____nesc_unnamed4284 {
#line 88
  CC2420SpiP__grant = 0U
};
#line 88
typedef int CC2420SpiP____nesc_sillytask_grant[CC2420SpiP__grant];
#line 63
enum CC2420SpiP____nesc_unnamed4285 {
  CC2420SpiP__RESOURCE_COUNT = 7U, 
  CC2420SpiP__NO_HOLDER = 0xFF
};


enum CC2420SpiP____nesc_unnamed4286 {
  CC2420SpiP__S_IDLE, 
  CC2420SpiP__S_BUSY
};


uint16_t CC2420SpiP__m_addr;





uint8_t CC2420SpiP__m_holder = CC2420SpiP__NO_HOLDER;
#line 184
static inline void CC2420SpiP__SpiResource__granted(void );
#line 305
static cc2420_status_t CC2420SpiP__Reg__write(uint8_t addr, uint16_t data);
#line 318
static cc2420_status_t CC2420SpiP__Strobe__strobe(uint8_t addr);










static void CC2420SpiP__SpiPacket__sendDone(uint8_t *tx_buf, uint8_t *rx_buf, 
uint16_t len, error_t error);
#line 358
static void CC2420SpiP__grant__runTask(void );








static inline void CC2420SpiP__Resource__default__granted(uint8_t id);


static inline void CC2420SpiP__Fifo__default__readDone(uint8_t addr, uint8_t *rx_buf, uint8_t rx_len, error_t error);


static inline void CC2420SpiP__Fifo__default__writeDone(uint8_t addr, uint8_t *tx_buf, uint8_t tx_len, error_t error);
# 74 "/home/rgao/lily/tinyos2/tos/system/StateImplP.nc"
uint8_t StateImplP__state[1U];

enum StateImplP____nesc_unnamed4287 {
  StateImplP__S_IDLE = 0
};


static inline error_t StateImplP__Init__init(void );
#line 126
static inline bool StateImplP__State__isIdle(uint8_t id);






static inline bool StateImplP__State__isState(uint8_t id, uint8_t myState);
# 82 "/home/rgao/lily/tinyos2/tos/interfaces/SpiPacket.nc"
static void /*Msp430SpiNoDmaB0P.SpiP*/Msp430SpiNoDmaBP__0__SpiPacket__sendDone(
# 46 "/home/rgao/lily/tinyos2/tos/chips/msp430/x2xxx/usci/Msp430SpiNoDmaBP.nc"
uint8_t arg_0x4082e0c0, 
# 75 "/home/rgao/lily/tinyos2/tos/interfaces/SpiPacket.nc"
uint8_t * txBuf, 
uint8_t * rxBuf, 





uint16_t len, 
error_t error);
# 45 "/home/rgao/lily/tinyos2/tos/chips/msp430/x2xxx/usci/Msp430SpiConfigure.nc"
static msp430_spi_union_config_t */*Msp430SpiNoDmaB0P.SpiP*/Msp430SpiNoDmaBP__0__Msp430SpiConfigure__getConfig(
# 49 "/home/rgao/lily/tinyos2/tos/chips/msp430/x2xxx/usci/Msp430SpiNoDmaBP.nc"
uint8_t arg_0x4082d318);
# 102 "/home/rgao/lily/tinyos2/tos/interfaces/Resource.nc"
static void /*Msp430SpiNoDmaB0P.SpiP*/Msp430SpiNoDmaBP__0__Resource__granted(
# 43 "/home/rgao/lily/tinyos2/tos/chips/msp430/x2xxx/usci/Msp430SpiNoDmaBP.nc"
uint8_t arg_0x40819ad8);
# 100 "/home/rgao/lily/tinyos2/tos/chips/msp430/x2xxx/usci/HplMsp430UsciB.nc"
static void /*Msp430SpiNoDmaB0P.SpiP*/Msp430SpiNoDmaBP__0__Usci__clrRxIntr(void );
#line 90
static void /*Msp430SpiNoDmaB0P.SpiP*/Msp430SpiNoDmaBP__0__Usci__disableRxIntr(void );
#line 108
static void /*Msp430SpiNoDmaB0P.SpiP*/Msp430SpiNoDmaBP__0__Usci__tx(uint8_t data);






static uint8_t /*Msp430SpiNoDmaB0P.SpiP*/Msp430SpiNoDmaBP__0__Usci__rx(void );
#line 141
static void /*Msp430SpiNoDmaB0P.SpiP*/Msp430SpiNoDmaBP__0__Usci__setModeSpi(msp430_spi_union_config_t *config);
#line 98
static bool /*Msp430SpiNoDmaB0P.SpiP*/Msp430SpiNoDmaBP__0__Usci__isRxIntrPending(void );
# 69 "/home/rgao/lily/tinyos2/tos/chips/msp430/x2xxx/usci/Msp430SpiNoDmaBP.nc"
enum /*Msp430SpiNoDmaB0P.SpiP*/Msp430SpiNoDmaBP__0____nesc_unnamed4288 {
#line 69
  Msp430SpiNoDmaBP__0__signalDone_task = 1U
};
#line 69
typedef int /*Msp430SpiNoDmaB0P.SpiP*/Msp430SpiNoDmaBP__0____nesc_sillytask_signalDone_task[/*Msp430SpiNoDmaB0P.SpiP*/Msp430SpiNoDmaBP__0__signalDone_task];
#line 58
enum /*Msp430SpiNoDmaB0P.SpiP*/Msp430SpiNoDmaBP__0____nesc_unnamed4289 {
  Msp430SpiNoDmaBP__0__SPI_ATOMIC_SIZE = 2
};

uint16_t /*Msp430SpiNoDmaB0P.SpiP*/Msp430SpiNoDmaBP__0__m_len;
uint8_t * /*Msp430SpiNoDmaB0P.SpiP*/Msp430SpiNoDmaBP__0__m_tx_buf;
uint8_t * /*Msp430SpiNoDmaB0P.SpiP*/Msp430SpiNoDmaBP__0__m_rx_buf;
uint16_t /*Msp430SpiNoDmaB0P.SpiP*/Msp430SpiNoDmaBP__0__m_pos;
uint8_t /*Msp430SpiNoDmaB0P.SpiP*/Msp430SpiNoDmaBP__0__m_client;

static inline void /*Msp430SpiNoDmaB0P.SpiP*/Msp430SpiNoDmaBP__0__signalDone(void );
#line 87
static inline void /*Msp430SpiNoDmaB0P.SpiP*/Msp430SpiNoDmaBP__0__ResourceConfigure__configure(uint8_t id);









static inline void /*Msp430SpiNoDmaB0P.SpiP*/Msp430SpiNoDmaBP__0__UsciResource__granted(uint8_t id);



static uint8_t /*Msp430SpiNoDmaB0P.SpiP*/Msp430SpiNoDmaBP__0__SpiByte__write(uint8_t tx);
#line 121
static inline void /*Msp430SpiNoDmaB0P.SpiP*/Msp430SpiNoDmaBP__0__Resource__default__granted(uint8_t id);

static inline void /*Msp430SpiNoDmaB0P.SpiP*/Msp430SpiNoDmaBP__0__continueOp(void );
#line 168
static inline void /*Msp430SpiNoDmaB0P.SpiP*/Msp430SpiNoDmaBP__0__signalDone_task__runTask(void );



static inline void /*Msp430SpiNoDmaB0P.SpiP*/Msp430SpiNoDmaBP__0__UsciInterrupts__rxDone(uint8_t data);
#line 185
static inline void /*Msp430SpiNoDmaB0P.SpiP*/Msp430SpiNoDmaBP__0__signalDone(void );




static inline void /*Msp430SpiNoDmaB0P.SpiP*/Msp430SpiNoDmaBP__0__UsciInterrupts__txDone(void );

static inline void /*Msp430SpiNoDmaB0P.SpiP*/Msp430SpiNoDmaBP__0__SpiPacket__default__sendDone(uint8_t id, uint8_t *tx_buf, uint8_t *rx_buf, uint16_t len, error_t error);
# 46 "/home/rgao/lily/tinyos2/tos/platforms/z1/chips/msp430/usci/Z1UsciP.nc"
msp430_spi_union_config_t /*Msp430SpiNoDmaB0P.Z1UsciP*/Z1UsciP__0__msp430_spi_z1_config = { { 
.ubr = 2, 
.ucmode = 0, 
.ucmst = 1, 
.uc7bit = 0, 
.ucmsb = 1, 
.ucckpl = 1, 
.ucckph = 0, 
.ucssel = 2 } };


static inline msp430_spi_union_config_t */*Msp430SpiNoDmaB0P.Z1UsciP*/Z1UsciP__0__Msp430SpiConfigure__getConfig(uint8_t id);
# 92 "/home/rgao/lily/tinyos2/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
static void HplMsp430UsciB0P__UCLK__selectModuleFunc(void );
# 59 "/home/rgao/lily/tinyos2/tos/chips/msp430/x2xxx/usci/HplMsp430UsciInterrupts.nc"
static void HplMsp430UsciB0P__Interrupts__rxDone(uint8_t data);
#line 54
static void HplMsp430UsciB0P__Interrupts__txDone(void );
# 92 "/home/rgao/lily/tinyos2/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
static void HplMsp430UsciB0P__SOMI__selectModuleFunc(void );
#line 92
static void HplMsp430UsciB0P__SIMO__selectModuleFunc(void );
# 73 "/home/rgao/lily/tinyos2/tos/chips/msp430/x2xxx/usci/HplMsp430UsciB0P.nc"
static volatile uint8_t HplMsp430UsciB0P__IE2 __asm ("0x0001");
static volatile uint8_t HplMsp430UsciB0P__IFG2 __asm ("0x0003");
static volatile uint8_t HplMsp430UsciB0P__UCB0CTL0 __asm ("0x0068");
static volatile uint8_t HplMsp430UsciB0P__UCB0CTL1 __asm ("0x0069");
static volatile uint8_t HplMsp430UsciB0P__UCB0RXBUF __asm ("0x006E");
static volatile uint8_t HplMsp430UsciB0P__UCB0TXBUF __asm ("0x006F");



static inline void HplMsp430UsciB0P__UsciRawInterrupts__rxDone(uint8_t temp);



static inline void HplMsp430UsciB0P__UsciRawInterrupts__txDone(void );
#line 107
static inline void HplMsp430UsciB0P__Usci__setUbr(uint16_t control);
#line 127
static inline void HplMsp430UsciB0P__Usci__resetUsci(bool reset);
#line 160
static inline void HplMsp430UsciB0P__Usci__enableSpi(void );
#line 176
static inline void HplMsp430UsciB0P__configSpi(msp430_spi_union_config_t *config);





static inline void HplMsp430UsciB0P__Usci__setModeSpi(msp430_spi_union_config_t *config);
#line 199
static inline bool HplMsp430UsciB0P__Usci__isRxIntrPending(void );









static inline void HplMsp430UsciB0P__Usci__clrRxIntr(void );



static inline void HplMsp430UsciB0P__Usci__clrIntr(void );



static inline void HplMsp430UsciB0P__Usci__disableRxIntr(void );







static inline void HplMsp430UsciB0P__Usci__disableIntr(void );
#line 250
static inline void HplMsp430UsciB0P__Usci__tx(uint8_t data);



static inline uint8_t HplMsp430UsciB0P__Usci__rx(void );
# 58 "/home/rgao/lily/tinyos2/tos/chips/msp430/x2xxx/usci/HplMsp430UsciRawInterrupts.nc"
static void HplMsp430UsciAB0RawInterruptsP__UsciA__rxDone(uint8_t data);
#line 53
static void HplMsp430UsciAB0RawInterruptsP__UsciA__txDone(void );




static void HplMsp430UsciAB0RawInterruptsP__UsciB__rxDone(uint8_t data);
#line 53
static void HplMsp430UsciAB0RawInterruptsP__UsciB__txDone(void );
# 52 "/home/rgao/lily/tinyos2/tos/chips/msp430/x2xxx/usci/HplMsp430UsciAB0RawInterruptsP.nc"
void sig_USCIAB0RX_VECTOR(void ) __attribute((wakeup)) __attribute((interrupt(0x002E)))  ;
#line 64
void sig_USCIAB0TX_VECTOR(void ) __attribute((wakeup)) __attribute((interrupt(0x002C)))  ;










static inline void HplMsp430UsciAB0RawInterruptsP__UsciA__default__txDone(void );



static inline void HplMsp430UsciAB0RawInterruptsP__UsciA__default__rxDone(uint8_t temp);
# 46 "/home/rgao/lily/tinyos2/tos/interfaces/GeneralIO.nc"
static void LedsP__Led0__makeOutput(void );
#line 40
static void LedsP__Led0__set(void );





static void LedsP__Led1__makeOutput(void );
#line 40
static void LedsP__Led1__set(void );





static void LedsP__Led2__makeOutput(void );
#line 40
static void LedsP__Led2__set(void );
# 56 "/home/rgao/lily/tinyos2/tos/system/LedsP.nc"
static inline error_t LedsP__Init__init(void );
# 85 "/home/rgao/lily/tinyos2/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
static void /*PlatformLedsC.Led0Impl*/Msp430GpioC__7__HplGeneralIO__makeOutput(void );
#line 48
static void /*PlatformLedsC.Led0Impl*/Msp430GpioC__7__HplGeneralIO__set(void );
# 48 "/home/rgao/lily/tinyos2/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline void /*PlatformLedsC.Led0Impl*/Msp430GpioC__7__GeneralIO__set(void );





static inline void /*PlatformLedsC.Led0Impl*/Msp430GpioC__7__GeneralIO__makeOutput(void );
# 85 "/home/rgao/lily/tinyos2/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
static void /*PlatformLedsC.Led1Impl*/Msp430GpioC__8__HplGeneralIO__makeOutput(void );
#line 48
static void /*PlatformLedsC.Led1Impl*/Msp430GpioC__8__HplGeneralIO__set(void );
# 48 "/home/rgao/lily/tinyos2/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline void /*PlatformLedsC.Led1Impl*/Msp430GpioC__8__GeneralIO__set(void );





static inline void /*PlatformLedsC.Led1Impl*/Msp430GpioC__8__GeneralIO__makeOutput(void );
# 85 "/home/rgao/lily/tinyos2/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
static void /*PlatformLedsC.Led2Impl*/Msp430GpioC__9__HplGeneralIO__makeOutput(void );
#line 48
static void /*PlatformLedsC.Led2Impl*/Msp430GpioC__9__HplGeneralIO__set(void );
# 48 "/home/rgao/lily/tinyos2/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline void /*PlatformLedsC.Led2Impl*/Msp430GpioC__9__GeneralIO__set(void );





static inline void /*PlatformLedsC.Led2Impl*/Msp430GpioC__9__GeneralIO__makeOutput(void );
# 59 "/home/rgao/lily/tinyos2/tos/chips/msp430/x2xxx/usci/HplMsp430UsciInterrupts.nc"
static void /*Msp430UsciShareB0P.UsciShareP*/Msp430UsciShareP__0__Interrupts__rxDone(
# 41 "/home/rgao/lily/tinyos2/tos/chips/msp430/x2xxx/usci/Msp430UsciShareP.nc"
uint8_t arg_0x40916cf8, 
# 59 "/home/rgao/lily/tinyos2/tos/chips/msp430/x2xxx/usci/HplMsp430UsciInterrupts.nc"
uint8_t data);
#line 54
static void /*Msp430UsciShareB0P.UsciShareP*/Msp430UsciShareP__0__Interrupts__txDone(
# 41 "/home/rgao/lily/tinyos2/tos/chips/msp430/x2xxx/usci/Msp430UsciShareP.nc"
uint8_t arg_0x40916cf8);
# 90 "/home/rgao/lily/tinyos2/tos/interfaces/ArbiterInfo.nc"
static bool /*Msp430UsciShareB0P.UsciShareP*/Msp430UsciShareP__0__ArbiterInfo__inUse(void );







static uint8_t /*Msp430UsciShareB0P.UsciShareP*/Msp430UsciShareP__0__ArbiterInfo__userId(void );
# 49 "/home/rgao/lily/tinyos2/tos/chips/msp430/x2xxx/usci/Msp430UsciShareP.nc"
static inline void /*Msp430UsciShareB0P.UsciShareP*/Msp430UsciShareP__0__RawInterrupts__txDone(void );




static inline void /*Msp430UsciShareB0P.UsciShareP*/Msp430UsciShareP__0__RawInterrupts__rxDone(uint8_t data);




static inline void /*Msp430UsciShareB0P.UsciShareP*/Msp430UsciShareP__0__Interrupts__default__txDone(uint8_t id);
static inline void /*Msp430UsciShareB0P.UsciShareP*/Msp430UsciShareP__0__Interrupts__default__rxDone(uint8_t id, uint8_t data);
# 49 "/home/rgao/lily/tinyos2/tos/system/FcfsResourceQueueC.nc"
enum /*Msp430UsciShareB0P.ArbiterC.Queue*/FcfsResourceQueueC__0____nesc_unnamed4290 {
#line 49
  FcfsResourceQueueC__0__NO_ENTRY = 0xFF
};
uint8_t /*Msp430UsciShareB0P.ArbiterC.Queue*/FcfsResourceQueueC__0__resQ[2U];



static inline error_t /*Msp430UsciShareB0P.ArbiterC.Queue*/FcfsResourceQueueC__0__Init__init(void );
# 59 "/home/rgao/lily/tinyos2/tos/interfaces/ResourceConfigure.nc"
static void /*Msp430UsciShareB0P.ArbiterC.Arbiter*/ArbiterP__0__ResourceConfigure__configure(
# 60 "/home/rgao/lily/tinyos2/tos/system/ArbiterP.nc"
uint8_t arg_0x4092c030);
# 102 "/home/rgao/lily/tinyos2/tos/interfaces/Resource.nc"
static void /*Msp430UsciShareB0P.ArbiterC.Arbiter*/ArbiterP__0__Resource__granted(
# 54 "/home/rgao/lily/tinyos2/tos/system/ArbiterP.nc"
uint8_t arg_0x4092e1b8);
#line 75
enum /*Msp430UsciShareB0P.ArbiterC.Arbiter*/ArbiterP__0____nesc_unnamed4291 {
#line 75
  ArbiterP__0__grantedTask = 2U
};
#line 75
typedef int /*Msp430UsciShareB0P.ArbiterC.Arbiter*/ArbiterP__0____nesc_sillytask_grantedTask[/*Msp430UsciShareB0P.ArbiterC.Arbiter*/ArbiterP__0__grantedTask];
#line 67
enum /*Msp430UsciShareB0P.ArbiterC.Arbiter*/ArbiterP__0____nesc_unnamed4292 {
#line 67
  ArbiterP__0__RES_CONTROLLED, ArbiterP__0__RES_GRANTING, ArbiterP__0__RES_IMM_GRANTING, ArbiterP__0__RES_BUSY
};
#line 68
enum /*Msp430UsciShareB0P.ArbiterC.Arbiter*/ArbiterP__0____nesc_unnamed4293 {
#line 68
  ArbiterP__0__default_owner_id = 2U
};
#line 69
enum /*Msp430UsciShareB0P.ArbiterC.Arbiter*/ArbiterP__0____nesc_unnamed4294 {
#line 69
  ArbiterP__0__NO_RES = 0xFF
};
uint8_t /*Msp430UsciShareB0P.ArbiterC.Arbiter*/ArbiterP__0__state = /*Msp430UsciShareB0P.ArbiterC.Arbiter*/ArbiterP__0__RES_CONTROLLED;
uint8_t /*Msp430UsciShareB0P.ArbiterC.Arbiter*/ArbiterP__0__resId = /*Msp430UsciShareB0P.ArbiterC.Arbiter*/ArbiterP__0__default_owner_id;
uint8_t /*Msp430UsciShareB0P.ArbiterC.Arbiter*/ArbiterP__0__reqResId;
#line 153
static bool /*Msp430UsciShareB0P.ArbiterC.Arbiter*/ArbiterP__0__ArbiterInfo__inUse(void );
#line 166
static uint8_t /*Msp430UsciShareB0P.ArbiterC.Arbiter*/ArbiterP__0__ArbiterInfo__userId(void );
#line 190
static void /*Msp430UsciShareB0P.ArbiterC.Arbiter*/ArbiterP__0__grantedTask__runTask(void );
#line 202
static inline void /*Msp430UsciShareB0P.ArbiterC.Arbiter*/ArbiterP__0__Resource__default__granted(uint8_t id);
#line 216
static inline void /*Msp430UsciShareB0P.ArbiterC.Arbiter*/ArbiterP__0__ResourceConfigure__default__configure(uint8_t id);
# 397 "/home/rgao/lily/tinyos2/tos/chips/msp430/msp430hardware.h"
static inline  void __nesc_enable_interrupt(void )
{
  __eint();
}

# 196 "/home/rgao/lily/tinyos2/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline void /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Timer__overflow(void )
{
}

#line 196
static inline void /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Timer__overflow(void )
{
}

#line 196
static inline void /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Timer__overflow(void )
{
}

# 48 "/home/rgao/lily/tinyos2/tos/chips/msp430/timer/Msp430Timer.nc"
inline static void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__Timer__overflow(void ){
#line 48
  /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Timer__overflow();
#line 48
  /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Timer__overflow();
#line 48
  /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Timer__overflow();
#line 48
}
#line 48
# 137 "/home/rgao/lily/tinyos2/tos/chips/msp430/timer/Msp430TimerP.nc"
static inline void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__Overflow__fired(void )
{
  /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__Timer__overflow();
}





static inline void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__Event__default__fired(uint8_t n)
{
}

# 39 "/home/rgao/lily/tinyos2/tos/chips/msp430/timer/Msp430TimerEvent.nc"
inline static void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__Event__fired(uint8_t arg_0x4053d640){
#line 39
  switch (arg_0x4053d640) {
#line 39
    case 0:
#line 39
      /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Event__fired();
#line 39
      break;
#line 39
    case 1:
#line 39
      /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Event__fired();
#line 39
      break;
#line 39
    case 2:
#line 39
      /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Event__fired();
#line 39
      break;
#line 39
    case 5:
#line 39
      /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__Overflow__fired();
#line 39
      break;
#line 39
    default:
#line 39
      /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__Event__default__fired(arg_0x4053d640);
#line 39
      break;
#line 39
    }
#line 39
}
#line 39
# 126 "/home/rgao/lily/tinyos2/tos/chips/msp430/timer/Msp430TimerP.nc"
static inline void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__VectorTimerX0__fired(void )
{
  /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__Event__fired(0);
}

# 39 "/home/rgao/lily/tinyos2/tos/chips/msp430/timer/Msp430TimerEvent.nc"
inline static void Msp430TimerCommonP__VectorTimerA0__fired(void ){
#line 39
  /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__VectorTimerX0__fired();
#line 39
}
#line 39
# 58 "/home/rgao/lily/tinyos2/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline  /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__cc_t /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__int2CC(uint16_t x)
#line 58
{
#line 58
  union /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0____nesc_unnamed4295 {
#line 58
    uint16_t f;
#line 58
    /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__cc_t t;
  } 
#line 58
  c = { .f = x };

#line 58
  return c.t;
}

#line 85
static inline /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__cc_t /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Control__getControl(void )
{
  return /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__int2CC(* (volatile uint16_t * )354U);
}

#line 188
static inline void /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Capture__default__captured(uint16_t n)
{
}

# 86 "/home/rgao/lily/tinyos2/tos/chips/msp430/timer/Msp430Capture.nc"
inline static void /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Capture__captured(uint16_t time){
#line 86
  /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Capture__default__captured(time);
#line 86
}
#line 86
# 150 "/home/rgao/lily/tinyos2/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline uint16_t /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Capture__getEvent(void )
{
  return * (volatile uint16_t * )370U;
}

#line 192
static inline void /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Compare__default__fired(void )
{
}

# 45 "/home/rgao/lily/tinyos2/tos/chips/msp430/timer/Msp430Compare.nc"
inline static void /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Compare__fired(void ){
#line 45
  /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Compare__default__fired();
#line 45
}
#line 45
# 58 "/home/rgao/lily/tinyos2/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline  /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__cc_t /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__int2CC(uint16_t x)
#line 58
{
#line 58
  union /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1____nesc_unnamed4296 {
#line 58
    uint16_t f;
#line 58
    /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__cc_t t;
  } 
#line 58
  c = { .f = x };

#line 58
  return c.t;
}

#line 85
static inline /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__cc_t /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Control__getControl(void )
{
  return /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__int2CC(* (volatile uint16_t * )356U);
}

#line 188
static inline void /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Capture__default__captured(uint16_t n)
{
}

# 86 "/home/rgao/lily/tinyos2/tos/chips/msp430/timer/Msp430Capture.nc"
inline static void /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Capture__captured(uint16_t time){
#line 86
  /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Capture__default__captured(time);
#line 86
}
#line 86
# 150 "/home/rgao/lily/tinyos2/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline uint16_t /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Capture__getEvent(void )
{
  return * (volatile uint16_t * )372U;
}

#line 192
static inline void /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Compare__default__fired(void )
{
}

# 45 "/home/rgao/lily/tinyos2/tos/chips/msp430/timer/Msp430Compare.nc"
inline static void /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Compare__fired(void ){
#line 45
  /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Compare__default__fired();
#line 45
}
#line 45
# 58 "/home/rgao/lily/tinyos2/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline  /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__cc_t /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__int2CC(uint16_t x)
#line 58
{
#line 58
  union /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2____nesc_unnamed4297 {
#line 58
    uint16_t f;
#line 58
    /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__cc_t t;
  } 
#line 58
  c = { .f = x };

#line 58
  return c.t;
}

#line 85
static inline /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__cc_t /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Control__getControl(void )
{
  return /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__int2CC(* (volatile uint16_t * )358U);
}

#line 188
static inline void /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Capture__default__captured(uint16_t n)
{
}

# 86 "/home/rgao/lily/tinyos2/tos/chips/msp430/timer/Msp430Capture.nc"
inline static void /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Capture__captured(uint16_t time){
#line 86
  /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Capture__default__captured(time);
#line 86
}
#line 86
# 150 "/home/rgao/lily/tinyos2/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline uint16_t /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Capture__getEvent(void )
{
  return * (volatile uint16_t * )374U;
}

#line 192
static inline void /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Compare__default__fired(void )
{
}

# 45 "/home/rgao/lily/tinyos2/tos/chips/msp430/timer/Msp430Compare.nc"
inline static void /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Compare__fired(void ){
#line 45
  /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Compare__default__fired();
#line 45
}
#line 45
# 131 "/home/rgao/lily/tinyos2/tos/chips/msp430/timer/Msp430TimerP.nc"
static inline void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__VectorTimerX1__fired(void )
{
  uint8_t n = * (volatile uint16_t * )302U;

#line 134
  /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__Event__fired(n >> 1);
}

# 39 "/home/rgao/lily/tinyos2/tos/chips/msp430/timer/Msp430TimerEvent.nc"
inline static void Msp430TimerCommonP__VectorTimerA1__fired(void ){
#line 39
  /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__VectorTimerX1__fired();
#line 39
}
#line 39
# 126 "/home/rgao/lily/tinyos2/tos/chips/msp430/timer/Msp430TimerP.nc"
static inline void /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__VectorTimerX0__fired(void )
{
  /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__Event__fired(0);
}

# 39 "/home/rgao/lily/tinyos2/tos/chips/msp430/timer/Msp430TimerEvent.nc"
inline static void Msp430TimerCommonP__VectorTimerB0__fired(void ){
#line 39
  /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__VectorTimerX0__fired();
#line 39
}
#line 39
# 196 "/home/rgao/lily/tinyos2/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline void /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Timer__overflow(void )
{
}

#line 196
static inline void /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Timer__overflow(void )
{
}

#line 196
static inline void /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Timer__overflow(void )
{
}

#line 196
static inline void /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Timer__overflow(void )
{
}

#line 196
static inline void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Timer__overflow(void )
{
}

#line 196
static inline void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Timer__overflow(void )
{
}

#line 196
static inline void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Timer__overflow(void )
{
}

# 48 "/home/rgao/lily/tinyos2/tos/chips/msp430/timer/Msp430Timer.nc"
inline static void /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__Timer__overflow(void ){
#line 48
  /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Timer__overflow();
#line 48
  /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Timer__overflow();
#line 48
  /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Timer__overflow();
#line 48
  /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Timer__overflow();
#line 48
  /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Timer__overflow();
#line 48
  /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Timer__overflow();
#line 48
  /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Timer__overflow();
#line 48
}
#line 48
# 137 "/home/rgao/lily/tinyos2/tos/chips/msp430/timer/Msp430TimerP.nc"
static inline void /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__Overflow__fired(void )
{
  /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__Timer__overflow();
}

# 192 "/home/rgao/lily/tinyos2/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Compare__default__fired(void )
{
}

# 45 "/home/rgao/lily/tinyos2/tos/chips/msp430/timer/Msp430Compare.nc"
inline static void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Compare__fired(void ){
#line 45
  /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Compare__default__fired();
#line 45
}
#line 45
# 150 "/home/rgao/lily/tinyos2/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline uint16_t /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Capture__getEvent(void )
{
  return * (volatile uint16_t * )402U;
}

#line 188
static inline void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Capture__default__captured(uint16_t n)
{
}

# 86 "/home/rgao/lily/tinyos2/tos/chips/msp430/timer/Msp430Capture.nc"
inline static void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Capture__captured(uint16_t time){
#line 86
  /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Capture__default__captured(time);
#line 86
}
#line 86
# 58 "/home/rgao/lily/tinyos2/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline  /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__cc_t /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__int2CC(uint16_t x)
#line 58
{
#line 58
  union /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3____nesc_unnamed4298 {
#line 58
    uint16_t f;
#line 58
    /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__cc_t t;
  } 
#line 58
  c = { .f = x };

#line 58
  return c.t;
}

#line 85
static inline /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__cc_t /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Control__getControl(void )
{
  return /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__int2CC(* (volatile uint16_t * )386U);
}

#line 180
static inline void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Event__fired(void )
{
  if (/*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Control__getControl().cap) {
    /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Capture__captured(/*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Capture__getEvent());
    }
  else {
#line 185
    /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Compare__fired();
    }
}




static inline void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Compare__default__fired(void )
{
}

# 45 "/home/rgao/lily/tinyos2/tos/chips/msp430/timer/Msp430Compare.nc"
inline static void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Compare__fired(void ){
#line 45
  /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Compare__default__fired();
#line 45
}
#line 45
# 150 "/home/rgao/lily/tinyos2/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline uint16_t /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Capture__getEvent(void )
{
  return * (volatile uint16_t * )404U;
}

#line 188
static inline void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Capture__default__captured(uint16_t n)
{
}

# 86 "/home/rgao/lily/tinyos2/tos/chips/msp430/timer/Msp430Capture.nc"
inline static void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Capture__captured(uint16_t time){
#line 86
  /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Capture__default__captured(time);
#line 86
}
#line 86
# 58 "/home/rgao/lily/tinyos2/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline  /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__cc_t /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__int2CC(uint16_t x)
#line 58
{
#line 58
  union /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4____nesc_unnamed4299 {
#line 58
    uint16_t f;
#line 58
    /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__cc_t t;
  } 
#line 58
  c = { .f = x };

#line 58
  return c.t;
}

#line 85
static inline /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__cc_t /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Control__getControl(void )
{
  return /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__int2CC(* (volatile uint16_t * )388U);
}

#line 180
static inline void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Event__fired(void )
{
  if (/*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Control__getControl().cap) {
    /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Capture__captured(/*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Capture__getEvent());
    }
  else {
#line 185
    /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Compare__fired();
    }
}




static inline void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Compare__default__fired(void )
{
}

# 45 "/home/rgao/lily/tinyos2/tos/chips/msp430/timer/Msp430Compare.nc"
inline static void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Compare__fired(void ){
#line 45
  /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Compare__default__fired();
#line 45
}
#line 45
# 150 "/home/rgao/lily/tinyos2/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline uint16_t /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Capture__getEvent(void )
{
  return * (volatile uint16_t * )406U;
}

#line 188
static inline void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Capture__default__captured(uint16_t n)
{
}

# 86 "/home/rgao/lily/tinyos2/tos/chips/msp430/timer/Msp430Capture.nc"
inline static void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Capture__captured(uint16_t time){
#line 86
  /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Capture__default__captured(time);
#line 86
}
#line 86
# 58 "/home/rgao/lily/tinyos2/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline  /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__cc_t /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__int2CC(uint16_t x)
#line 58
{
#line 58
  union /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5____nesc_unnamed4300 {
#line 58
    uint16_t f;
#line 58
    /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__cc_t t;
  } 
#line 58
  c = { .f = x };

#line 58
  return c.t;
}

#line 85
static inline /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__cc_t /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Control__getControl(void )
{
  return /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__int2CC(* (volatile uint16_t * )390U);
}

#line 180
static inline void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Event__fired(void )
{
  if (/*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Control__getControl().cap) {
    /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Capture__captured(/*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Capture__getEvent());
    }
  else {
#line 185
    /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Compare__fired();
    }
}




static inline void /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Compare__default__fired(void )
{
}

# 45 "/home/rgao/lily/tinyos2/tos/chips/msp430/timer/Msp430Compare.nc"
inline static void /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Compare__fired(void ){
#line 45
  /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Compare__default__fired();
#line 45
}
#line 45
# 150 "/home/rgao/lily/tinyos2/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline uint16_t /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Capture__getEvent(void )
{
  return * (volatile uint16_t * )408U;
}

#line 188
static inline void /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Capture__default__captured(uint16_t n)
{
}

# 86 "/home/rgao/lily/tinyos2/tos/chips/msp430/timer/Msp430Capture.nc"
inline static void /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Capture__captured(uint16_t time){
#line 86
  /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Capture__default__captured(time);
#line 86
}
#line 86
# 58 "/home/rgao/lily/tinyos2/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline  /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__cc_t /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__int2CC(uint16_t x)
#line 58
{
#line 58
  union /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6____nesc_unnamed4301 {
#line 58
    uint16_t f;
#line 58
    /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__cc_t t;
  } 
#line 58
  c = { .f = x };

#line 58
  return c.t;
}

#line 85
static inline /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__cc_t /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Control__getControl(void )
{
  return /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__int2CC(* (volatile uint16_t * )392U);
}

#line 180
static inline void /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Event__fired(void )
{
  if (/*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Control__getControl().cap) {
    /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Capture__captured(/*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Capture__getEvent());
    }
  else {
#line 185
    /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Compare__fired();
    }
}




static inline void /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Compare__default__fired(void )
{
}

# 45 "/home/rgao/lily/tinyos2/tos/chips/msp430/timer/Msp430Compare.nc"
inline static void /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Compare__fired(void ){
#line 45
  /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Compare__default__fired();
#line 45
}
#line 45
# 150 "/home/rgao/lily/tinyos2/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline uint16_t /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Capture__getEvent(void )
{
  return * (volatile uint16_t * )410U;
}

#line 188
static inline void /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Capture__default__captured(uint16_t n)
{
}

# 86 "/home/rgao/lily/tinyos2/tos/chips/msp430/timer/Msp430Capture.nc"
inline static void /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Capture__captured(uint16_t time){
#line 86
  /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Capture__default__captured(time);
#line 86
}
#line 86
# 58 "/home/rgao/lily/tinyos2/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline  /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__cc_t /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__int2CC(uint16_t x)
#line 58
{
#line 58
  union /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7____nesc_unnamed4302 {
#line 58
    uint16_t f;
#line 58
    /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__cc_t t;
  } 
#line 58
  c = { .f = x };

#line 58
  return c.t;
}

#line 85
static inline /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__cc_t /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Control__getControl(void )
{
  return /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__int2CC(* (volatile uint16_t * )394U);
}

#line 180
static inline void /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Event__fired(void )
{
  if (/*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Control__getControl().cap) {
    /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Capture__captured(/*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Capture__getEvent());
    }
  else {
#line 185
    /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Compare__fired();
    }
}




static inline void /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Compare__default__fired(void )
{
}

# 45 "/home/rgao/lily/tinyos2/tos/chips/msp430/timer/Msp430Compare.nc"
inline static void /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Compare__fired(void ){
#line 45
  /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Compare__default__fired();
#line 45
}
#line 45
# 150 "/home/rgao/lily/tinyos2/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline uint16_t /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Capture__getEvent(void )
{
  return * (volatile uint16_t * )412U;
}

#line 188
static inline void /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Capture__default__captured(uint16_t n)
{
}

# 86 "/home/rgao/lily/tinyos2/tos/chips/msp430/timer/Msp430Capture.nc"
inline static void /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Capture__captured(uint16_t time){
#line 86
  /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Capture__default__captured(time);
#line 86
}
#line 86
# 58 "/home/rgao/lily/tinyos2/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline  /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__cc_t /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__int2CC(uint16_t x)
#line 58
{
#line 58
  union /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8____nesc_unnamed4303 {
#line 58
    uint16_t f;
#line 58
    /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__cc_t t;
  } 
#line 58
  c = { .f = x };

#line 58
  return c.t;
}

#line 85
static inline /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__cc_t /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Control__getControl(void )
{
  return /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__int2CC(* (volatile uint16_t * )396U);
}

#line 180
static inline void /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Event__fired(void )
{
  if (/*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Control__getControl().cap) {
    /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Capture__captured(/*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Capture__getEvent());
    }
  else {
#line 185
    /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Compare__fired();
    }
}




static inline void /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Compare__default__fired(void )
{
}

# 45 "/home/rgao/lily/tinyos2/tos/chips/msp430/timer/Msp430Compare.nc"
inline static void /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Compare__fired(void ){
#line 45
  /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Compare__default__fired();
#line 45
}
#line 45
# 150 "/home/rgao/lily/tinyos2/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline uint16_t /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Capture__getEvent(void )
{
  return * (volatile uint16_t * )414U;
}

#line 188
static inline void /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Capture__default__captured(uint16_t n)
{
}

# 86 "/home/rgao/lily/tinyos2/tos/chips/msp430/timer/Msp430Capture.nc"
inline static void /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Capture__captured(uint16_t time){
#line 86
  /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Capture__default__captured(time);
#line 86
}
#line 86
# 58 "/home/rgao/lily/tinyos2/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline  /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__cc_t /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__int2CC(uint16_t x)
#line 58
{
#line 58
  union /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9____nesc_unnamed4304 {
#line 58
    uint16_t f;
#line 58
    /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__cc_t t;
  } 
#line 58
  c = { .f = x };

#line 58
  return c.t;
}

#line 85
static inline /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__cc_t /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Control__getControl(void )
{
  return /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__int2CC(* (volatile uint16_t * )398U);
}

#line 180
static inline void /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Event__fired(void )
{
  if (/*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Control__getControl().cap) {
    /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Capture__captured(/*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Capture__getEvent());
    }
  else {
#line 185
    /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Compare__fired();
    }
}

# 131 "/home/rgao/lily/tinyos2/tos/chips/msp430/timer/Msp430TimerP.nc"
static inline void /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__VectorTimerX1__fired(void )
{
  uint8_t n = * (volatile uint16_t * )286U;

#line 134
  /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__Event__fired(n >> 1);
}

# 39 "/home/rgao/lily/tinyos2/tos/chips/msp430/timer/Msp430TimerEvent.nc"
inline static void Msp430TimerCommonP__VectorTimerB1__fired(void ){
#line 39
  /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__VectorTimerX1__fired();
#line 39
}
#line 39
# 124 "/home/rgao/lily/tinyos2/tos/system/SchedulerBasicP.nc"
static inline void SchedulerBasicP__Scheduler__init(void )
{
  /* atomic removed: atomic calls only */
  {
    memset((void *)SchedulerBasicP__m_next, SchedulerBasicP__NO_TASK, sizeof SchedulerBasicP__m_next);
    SchedulerBasicP__m_head = SchedulerBasicP__NO_TASK;
    SchedulerBasicP__m_tail = SchedulerBasicP__NO_TASK;
  }
}

# 57 "/home/rgao/lily/tinyos2/tos/interfaces/Scheduler.nc"
inline static void RealMainP__Scheduler__init(void ){
#line 57
  SchedulerBasicP__Scheduler__init();
#line 57
}
#line 57
# 48 "/home/rgao/lily/tinyos2/tos/chips/msp430/pins/HplMsp430GeneralIORenP.nc"
static inline void /*HplMsp430GeneralIOC.P55*/HplMsp430GeneralIORenP__37__IO__set(void )
#line 48
{
  /* atomic removed: atomic calls only */
#line 48
  * (volatile uint8_t * )49U |= 0x01 << 5;
}

# 48 "/home/rgao/lily/tinyos2/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void /*PlatformLedsC.Led2Impl*/Msp430GpioC__9__HplGeneralIO__set(void ){
#line 48
  /*HplMsp430GeneralIOC.P55*/HplMsp430GeneralIORenP__37__IO__set();
#line 48
}
#line 48
# 48 "/home/rgao/lily/tinyos2/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline void /*PlatformLedsC.Led2Impl*/Msp430GpioC__9__GeneralIO__set(void )
#line 48
{
#line 48
  /*PlatformLedsC.Led2Impl*/Msp430GpioC__9__HplGeneralIO__set();
}

# 40 "/home/rgao/lily/tinyos2/tos/interfaces/GeneralIO.nc"
inline static void LedsP__Led2__set(void ){
#line 40
  /*PlatformLedsC.Led2Impl*/Msp430GpioC__9__GeneralIO__set();
#line 40
}
#line 40
# 48 "/home/rgao/lily/tinyos2/tos/chips/msp430/pins/HplMsp430GeneralIORenP.nc"
static inline void /*HplMsp430GeneralIOC.P56*/HplMsp430GeneralIORenP__38__IO__set(void )
#line 48
{
  /* atomic removed: atomic calls only */
#line 48
  * (volatile uint8_t * )49U |= 0x01 << 6;
}

# 48 "/home/rgao/lily/tinyos2/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void /*PlatformLedsC.Led1Impl*/Msp430GpioC__8__HplGeneralIO__set(void ){
#line 48
  /*HplMsp430GeneralIOC.P56*/HplMsp430GeneralIORenP__38__IO__set();
#line 48
}
#line 48
# 48 "/home/rgao/lily/tinyos2/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline void /*PlatformLedsC.Led1Impl*/Msp430GpioC__8__GeneralIO__set(void )
#line 48
{
#line 48
  /*PlatformLedsC.Led1Impl*/Msp430GpioC__8__HplGeneralIO__set();
}

# 40 "/home/rgao/lily/tinyos2/tos/interfaces/GeneralIO.nc"
inline static void LedsP__Led1__set(void ){
#line 40
  /*PlatformLedsC.Led1Impl*/Msp430GpioC__8__GeneralIO__set();
#line 40
}
#line 40
# 48 "/home/rgao/lily/tinyos2/tos/chips/msp430/pins/HplMsp430GeneralIORenP.nc"
static inline void /*HplMsp430GeneralIOC.P54*/HplMsp430GeneralIORenP__36__IO__set(void )
#line 48
{
  /* atomic removed: atomic calls only */
#line 48
  * (volatile uint8_t * )49U |= 0x01 << 4;
}

# 48 "/home/rgao/lily/tinyos2/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void /*PlatformLedsC.Led0Impl*/Msp430GpioC__7__HplGeneralIO__set(void ){
#line 48
  /*HplMsp430GeneralIOC.P54*/HplMsp430GeneralIORenP__36__IO__set();
#line 48
}
#line 48
# 48 "/home/rgao/lily/tinyos2/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline void /*PlatformLedsC.Led0Impl*/Msp430GpioC__7__GeneralIO__set(void )
#line 48
{
#line 48
  /*PlatformLedsC.Led0Impl*/Msp430GpioC__7__HplGeneralIO__set();
}

# 40 "/home/rgao/lily/tinyos2/tos/interfaces/GeneralIO.nc"
inline static void LedsP__Led0__set(void ){
#line 40
  /*PlatformLedsC.Led0Impl*/Msp430GpioC__7__GeneralIO__set();
#line 40
}
#line 40
# 55 "/home/rgao/lily/tinyos2/tos/chips/msp430/pins/HplMsp430GeneralIORenP.nc"
static inline void /*HplMsp430GeneralIOC.P55*/HplMsp430GeneralIORenP__37__IO__makeOutput(void )
#line 55
{
  /* atomic removed: atomic calls only */
#line 55
  * (volatile uint8_t * )50U |= 0x01 << 5;
}

# 85 "/home/rgao/lily/tinyos2/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void /*PlatformLedsC.Led2Impl*/Msp430GpioC__9__HplGeneralIO__makeOutput(void ){
#line 85
  /*HplMsp430GeneralIOC.P55*/HplMsp430GeneralIORenP__37__IO__makeOutput();
#line 85
}
#line 85
# 54 "/home/rgao/lily/tinyos2/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline void /*PlatformLedsC.Led2Impl*/Msp430GpioC__9__GeneralIO__makeOutput(void )
#line 54
{
#line 54
  /*PlatformLedsC.Led2Impl*/Msp430GpioC__9__HplGeneralIO__makeOutput();
}

# 46 "/home/rgao/lily/tinyos2/tos/interfaces/GeneralIO.nc"
inline static void LedsP__Led2__makeOutput(void ){
#line 46
  /*PlatformLedsC.Led2Impl*/Msp430GpioC__9__GeneralIO__makeOutput();
#line 46
}
#line 46
# 55 "/home/rgao/lily/tinyos2/tos/chips/msp430/pins/HplMsp430GeneralIORenP.nc"
static inline void /*HplMsp430GeneralIOC.P56*/HplMsp430GeneralIORenP__38__IO__makeOutput(void )
#line 55
{
  /* atomic removed: atomic calls only */
#line 55
  * (volatile uint8_t * )50U |= 0x01 << 6;
}

# 85 "/home/rgao/lily/tinyos2/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void /*PlatformLedsC.Led1Impl*/Msp430GpioC__8__HplGeneralIO__makeOutput(void ){
#line 85
  /*HplMsp430GeneralIOC.P56*/HplMsp430GeneralIORenP__38__IO__makeOutput();
#line 85
}
#line 85
# 54 "/home/rgao/lily/tinyos2/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline void /*PlatformLedsC.Led1Impl*/Msp430GpioC__8__GeneralIO__makeOutput(void )
#line 54
{
#line 54
  /*PlatformLedsC.Led1Impl*/Msp430GpioC__8__HplGeneralIO__makeOutput();
}

# 46 "/home/rgao/lily/tinyos2/tos/interfaces/GeneralIO.nc"
inline static void LedsP__Led1__makeOutput(void ){
#line 46
  /*PlatformLedsC.Led1Impl*/Msp430GpioC__8__GeneralIO__makeOutput();
#line 46
}
#line 46
# 55 "/home/rgao/lily/tinyos2/tos/chips/msp430/pins/HplMsp430GeneralIORenP.nc"
static inline void /*HplMsp430GeneralIOC.P54*/HplMsp430GeneralIORenP__36__IO__makeOutput(void )
#line 55
{
  /* atomic removed: atomic calls only */
#line 55
  * (volatile uint8_t * )50U |= 0x01 << 4;
}

# 85 "/home/rgao/lily/tinyos2/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void /*PlatformLedsC.Led0Impl*/Msp430GpioC__7__HplGeneralIO__makeOutput(void ){
#line 85
  /*HplMsp430GeneralIOC.P54*/HplMsp430GeneralIORenP__36__IO__makeOutput();
#line 85
}
#line 85
# 54 "/home/rgao/lily/tinyos2/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline void /*PlatformLedsC.Led0Impl*/Msp430GpioC__7__GeneralIO__makeOutput(void )
#line 54
{
#line 54
  /*PlatformLedsC.Led0Impl*/Msp430GpioC__7__HplGeneralIO__makeOutput();
}

# 46 "/home/rgao/lily/tinyos2/tos/interfaces/GeneralIO.nc"
inline static void LedsP__Led0__makeOutput(void ){
#line 46
  /*PlatformLedsC.Led0Impl*/Msp430GpioC__7__GeneralIO__makeOutput();
#line 46
}
#line 46
# 56 "/home/rgao/lily/tinyos2/tos/system/LedsP.nc"
static inline error_t LedsP__Init__init(void )
#line 56
{
  /* atomic removed: atomic calls only */
#line 57
  {
    ;
    LedsP__Led0__makeOutput();
    LedsP__Led1__makeOutput();
    LedsP__Led2__makeOutput();
    LedsP__Led0__set();
    LedsP__Led1__set();
    LedsP__Led2__set();
  }
  return SUCCESS;
}

# 62 "/home/rgao/lily/tinyos2/tos/interfaces/Init.nc"
inline static error_t PlatformP__LedsInit__init(void ){
#line 62
  unsigned char __nesc_result;
#line 62

#line 62
  __nesc_result = LedsP__Init__init();
#line 62

#line 62
  return __nesc_result;
#line 62
}
#line 62
# 108 "/home/rgao/lily/tinyos2/tos/platforms/z1/chips/msp430/timer/Msp430XDcoCalib.h"
static inline void Set_DCO(unsigned int Delta)
{
  unsigned int Compare;
#line 110
  unsigned int Oldcapture = 0;

  BCSCTL1 |= 0x30;
  TACCTL2 = 0x4000 + 0x1000 + 0x0100;
  TACTL = 0x0200 + 0x0020 + 0x0004;

  while (1) 
    {
      while (!(0x0001 & TACCTL2)) ;
      TACCTL2 &= ~0x0001;
      Compare = TACCR2;
      Compare = Compare - Oldcapture;
      Oldcapture = TACCR2;

      if (Delta == Compare) {
        break;
        }
      else {
#line 126
        if (Delta < Compare) 
          {
            DCOCTL--;
            if (DCOCTL == 0xFF) {
              if (BCSCTL1 & 0x0f) {
                BCSCTL1--;
                }
              }
          }
        else 
#line 134
          {
            DCOCTL++;
            if (DCOCTL == 0x00) {
              if ((BCSCTL1 & 0x0f) != 0x0f) {
                BCSCTL1++;
                }
              }
          }
        }
    }
#line 141
  TACCTL2 = 0;
  TACTL = 0;
  BCSCTL1 &= ~0x30;
}

# 180 "/home/rgao/lily/tinyos2/tos/platforms/z1/chips/msp430/timer/Msp430ClockP.nc"
static inline void Msp430ClockP__startTimerB(void )
{

  Msp430ClockP__TBCTL = 0x0020 | (Msp430ClockP__TBCTL & ~(0x0020 | 0x0010));
}

#line 168
static inline void Msp430ClockP__startTimerA(void )
{

  Msp430ClockP__TACTL = 0x0020 | (Msp430ClockP__TACTL & ~(0x0020 | 0x0010));
}

#line 132
static inline void Msp430ClockP__Msp430ClockInit__defaultInitTimerB(void )
{
  TBR = 0;









  Msp430ClockP__TBCTL = 0x0100 | 0x0002;
}

#line 162
static inline void Msp430ClockP__Msp430ClockInit__default__initTimerB(void )
{
  Msp430ClockP__Msp430ClockInit__defaultInitTimerB();
}

# 43 "/home/rgao/lily/tinyos2/tos/chips/msp430/timer/Msp430ClockInit.nc"
inline static void Msp430ClockP__Msp430ClockInit__initTimerB(void ){
#line 43
  Msp430ClockP__Msp430ClockInit__default__initTimerB();
#line 43
}
#line 43
# 117 "/home/rgao/lily/tinyos2/tos/platforms/z1/chips/msp430/timer/Msp430ClockP.nc"
static inline void Msp430ClockP__Msp430ClockInit__defaultInitTimerA(void )
{
  TAR = 0;









  Msp430ClockP__TACTL = 0x0200 | 0x0002;
}

#line 157
static inline void Msp430ClockP__Msp430ClockInit__default__initTimerA(void )
{
  Msp430ClockP__Msp430ClockInit__defaultInitTimerA();
}

# 42 "/home/rgao/lily/tinyos2/tos/chips/msp430/timer/Msp430ClockInit.nc"
inline static void Msp430ClockP__Msp430ClockInit__initTimerA(void ){
#line 42
  Msp430ClockP__Msp430ClockInit__default__initTimerA();
#line 42
}
#line 42
# 82 "/home/rgao/lily/tinyos2/tos/platforms/z1/chips/msp430/timer/Msp430ClockP.nc"
static inline void Msp430ClockP__Msp430ClockInit__defaultInitClocks(void )
{


  if (CALBC1_8MHZ != 0xFF) {
      DCOCTL = 0x00;
      BCSCTL1 = CALBC1_8MHZ;
      DCOCTL = CALDCO_8MHZ;
    }
  else 
#line 90
    {
      DCOCTL = 0x00;
      BCSCTL1 = 0x8D;
      DCOCTL = 0x88;
    }







  BCSCTL1 = 0x80 | BCSCTL1;
#line 114
  Msp430ClockP__IE1 &= ~0x02;
}

#line 152
static inline void Msp430ClockP__Msp430ClockInit__default__initClocks(void )
{
  Msp430ClockP__Msp430ClockInit__defaultInitClocks();
}

# 41 "/home/rgao/lily/tinyos2/tos/chips/msp430/timer/Msp430ClockInit.nc"
inline static void Msp430ClockP__Msp430ClockInit__initClocks(void ){
#line 41
  Msp430ClockP__Msp430ClockInit__default__initClocks();
#line 41
}
#line 41
# 246 "/home/rgao/lily/tinyos2/tos/platforms/z1/chips/msp430/timer/Msp430ClockP.nc"
static inline error_t Msp430ClockP__Init__init(void )
{

  Msp430ClockP__TACTL = 0x0004;
  Msp430ClockP__TAIV = 0;
  Msp430ClockP__TBCTL = 0x0004;
  Msp430ClockP__TBIV = 0;
  /* atomic removed: atomic calls only */

  {



    Msp430ClockP__Msp430ClockInit__initClocks();
    Msp430ClockP__Msp430ClockInit__initTimerA();
    Msp430ClockP__Msp430ClockInit__initTimerB();
    Msp430ClockP__startTimerA();
    Msp430ClockP__startTimerB();
  }
  Set_DCO(1953);
  return SUCCESS;
}

# 62 "/home/rgao/lily/tinyos2/tos/interfaces/Init.nc"
inline static error_t PlatformP__Msp430ClockInit__init(void ){
#line 62
  unsigned char __nesc_result;
#line 62

#line 62
  __nesc_result = Msp430ClockP__Init__init();
#line 62

#line 62
  return __nesc_result;
#line 62
}
#line 62
# 50 "/home/rgao/lily/tinyos2/tos/platforms/z1/PlatformP.nc"
static inline error_t PlatformP__Init__init(void )
#line 50
{
  WDTCTL = 0x5A00 + 0x0080;
  PlatformP__Msp430ClockInit__init();
  PlatformP__LedsInit__init();
  return SUCCESS;
}

# 62 "/home/rgao/lily/tinyos2/tos/interfaces/Init.nc"
inline static error_t RealMainP__PlatformInit__init(void ){
#line 62
  unsigned char __nesc_result;
#line 62

#line 62
  __nesc_result = PlatformP__Init__init();
#line 62

#line 62
  return __nesc_result;
#line 62
}
#line 62
# 65 "/home/rgao/lily/tinyos2/tos/interfaces/Scheduler.nc"
inline static bool RealMainP__Scheduler__runNextTask(void ){
#line 65
  unsigned char __nesc_result;
#line 65

#line 65
  __nesc_result = SchedulerBasicP__Scheduler__runNextTask();
#line 65

#line 65
  return __nesc_result;
#line 65
}
#line 65
# 57 "/home/rgao/lily/tinyos2/tos/platforms/z1/chips/msp430/usci/Z1UsciP.nc"
static inline msp430_spi_union_config_t */*Msp430SpiNoDmaB0P.Z1UsciP*/Z1UsciP__0__Msp430SpiConfigure__getConfig(uint8_t id)
#line 57
{
  return (msp430_spi_union_config_t *)&/*Msp430SpiNoDmaB0P.Z1UsciP*/Z1UsciP__0__msp430_spi_z1_config;
}

# 45 "/home/rgao/lily/tinyos2/tos/chips/msp430/x2xxx/usci/Msp430SpiConfigure.nc"
inline static msp430_spi_union_config_t */*Msp430SpiNoDmaB0P.SpiP*/Msp430SpiNoDmaBP__0__Msp430SpiConfigure__getConfig(uint8_t arg_0x4082d318){
#line 45
  union __nesc_unnamed4267 *__nesc_result;
#line 45

#line 45
  __nesc_result = /*Msp430SpiNoDmaB0P.Z1UsciP*/Z1UsciP__0__Msp430SpiConfigure__getConfig(arg_0x4082d318);
#line 45

#line 45
  return __nesc_result;
#line 45
}
#line 45
# 127 "/home/rgao/lily/tinyos2/tos/chips/msp430/x2xxx/usci/HplMsp430UsciB0P.nc"
static inline void HplMsp430UsciB0P__Usci__resetUsci(bool reset)
#line 127
{
  if (reset) {
    HplMsp430UsciB0P__UCB0CTL1 |= 0x01;
    }
  else {
#line 131
    HplMsp430UsciB0P__UCB0CTL1 &= ~0x01;
    }
}

#line 107
static inline void HplMsp430UsciB0P__Usci__setUbr(uint16_t control)
#line 107
{
  /* atomic removed: atomic calls only */
#line 108
  {
    UCB0BR0 = control & 0x00FF;
    UCB0BR1 = (control >> 8) & 0x00FF;
  }
}

#line 176
static inline void HplMsp430UsciB0P__configSpi(msp430_spi_union_config_t *config)
#line 176
{
  HplMsp430UsciB0P__UCB0CTL1 = config->spiRegisters.uctl1 | 0x01;
  HplMsp430UsciB0P__UCB0CTL0 = config->spiRegisters.uctl0 | 0x01;
  HplMsp430UsciB0P__Usci__setUbr(config->spiRegisters.ubr);
}

# 57 "/home/rgao/lily/tinyos2/tos/chips/msp430/pins/HplMsp430GeneralIORenP.nc"
static inline void /*HplMsp430GeneralIOC.P33*/HplMsp430GeneralIORenP__19__IO__selectModuleFunc(void )
#line 57
{
  /* atomic removed: atomic calls only */
#line 57
  * (volatile uint8_t * )27U |= 0x01 << 3;
}

# 92 "/home/rgao/lily/tinyos2/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void HplMsp430UsciB0P__UCLK__selectModuleFunc(void ){
#line 92
  /*HplMsp430GeneralIOC.P33*/HplMsp430GeneralIORenP__19__IO__selectModuleFunc();
#line 92
}
#line 92
# 57 "/home/rgao/lily/tinyos2/tos/chips/msp430/pins/HplMsp430GeneralIORenP.nc"
static inline void /*HplMsp430GeneralIOC.P32*/HplMsp430GeneralIORenP__18__IO__selectModuleFunc(void )
#line 57
{
  /* atomic removed: atomic calls only */
#line 57
  * (volatile uint8_t * )27U |= 0x01 << 2;
}

# 92 "/home/rgao/lily/tinyos2/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void HplMsp430UsciB0P__SOMI__selectModuleFunc(void ){
#line 92
  /*HplMsp430GeneralIOC.P32*/HplMsp430GeneralIORenP__18__IO__selectModuleFunc();
#line 92
}
#line 92
# 57 "/home/rgao/lily/tinyos2/tos/chips/msp430/pins/HplMsp430GeneralIORenP.nc"
static inline void /*HplMsp430GeneralIOC.P31*/HplMsp430GeneralIORenP__17__IO__selectModuleFunc(void )
#line 57
{
  /* atomic removed: atomic calls only */
#line 57
  * (volatile uint8_t * )27U |= 0x01 << 1;
}

# 92 "/home/rgao/lily/tinyos2/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void HplMsp430UsciB0P__SIMO__selectModuleFunc(void ){
#line 92
  /*HplMsp430GeneralIOC.P31*/HplMsp430GeneralIORenP__17__IO__selectModuleFunc();
#line 92
}
#line 92
# 160 "/home/rgao/lily/tinyos2/tos/chips/msp430/x2xxx/usci/HplMsp430UsciB0P.nc"
static inline void HplMsp430UsciB0P__Usci__enableSpi(void )
#line 160
{
  /* atomic removed: atomic calls only */
#line 161
  {
    HplMsp430UsciB0P__SIMO__selectModuleFunc();
    HplMsp430UsciB0P__SOMI__selectModuleFunc();
    HplMsp430UsciB0P__UCLK__selectModuleFunc();
  }
}

#line 213
static inline void HplMsp430UsciB0P__Usci__clrIntr(void )
#line 213
{
  HplMsp430UsciB0P__IFG2 &= ~(0x08 | 0x04);
}









static inline void HplMsp430UsciB0P__Usci__disableIntr(void )
#line 225
{
  HplMsp430UsciB0P__IE2 &= ~(0x08 | 0x04);
}

#line 182
static inline void HplMsp430UsciB0P__Usci__setModeSpi(msp430_spi_union_config_t *config)
#line 182
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 183
    {
      HplMsp430UsciB0P__Usci__disableIntr();
      HplMsp430UsciB0P__Usci__clrIntr();
      HplMsp430UsciB0P__Usci__resetUsci(TRUE);
      HplMsp430UsciB0P__Usci__enableSpi();
      HplMsp430UsciB0P__configSpi(config);
      HplMsp430UsciB0P__Usci__resetUsci(FALSE);
    }
#line 190
    __nesc_atomic_end(__nesc_atomic); }
}

# 141 "/home/rgao/lily/tinyos2/tos/chips/msp430/x2xxx/usci/HplMsp430UsciB.nc"
inline static void /*Msp430SpiNoDmaB0P.SpiP*/Msp430SpiNoDmaBP__0__Usci__setModeSpi(msp430_spi_union_config_t *config){
#line 141
  HplMsp430UsciB0P__Usci__setModeSpi(config);
#line 141
}
#line 141
# 87 "/home/rgao/lily/tinyos2/tos/chips/msp430/x2xxx/usci/Msp430SpiNoDmaBP.nc"
static inline void /*Msp430SpiNoDmaB0P.SpiP*/Msp430SpiNoDmaBP__0__ResourceConfigure__configure(uint8_t id)
#line 87
{
  /*Msp430SpiNoDmaB0P.SpiP*/Msp430SpiNoDmaBP__0__Usci__setModeSpi(/*Msp430SpiNoDmaB0P.SpiP*/Msp430SpiNoDmaBP__0__Msp430SpiConfigure__getConfig(id));
}

# 216 "/home/rgao/lily/tinyos2/tos/system/ArbiterP.nc"
static inline void /*Msp430UsciShareB0P.ArbiterC.Arbiter*/ArbiterP__0__ResourceConfigure__default__configure(uint8_t id)
#line 216
{
}

# 59 "/home/rgao/lily/tinyos2/tos/interfaces/ResourceConfigure.nc"
inline static void /*Msp430UsciShareB0P.ArbiterC.Arbiter*/ArbiterP__0__ResourceConfigure__configure(uint8_t arg_0x4092c030){
#line 59
  switch (arg_0x4092c030) {
#line 59
    case /*CC2420SpiWireC.HplCC2420SpiC.SpiC.UsciC*/Msp430UsciB0C__0__CLIENT_ID:
#line 59
      /*Msp430SpiNoDmaB0P.SpiP*/Msp430SpiNoDmaBP__0__ResourceConfigure__configure(/*CC2420SpiWireC.HplCC2420SpiC.SpiC*/Msp430SpiB0C__0__CLIENT_ID);
#line 59
      break;
#line 59
    default:
#line 59
      /*Msp430UsciShareB0P.ArbiterC.Arbiter*/ArbiterP__0__ResourceConfigure__default__configure(arg_0x4092c030);
#line 59
      break;
#line 59
    }
#line 59
}
#line 59
# 97 "/home/rgao/lily/tinyos2/tos/system/SchedulerBasicP.nc"
static inline bool SchedulerBasicP__isWaiting(uint8_t id)
{
  return SchedulerBasicP__m_next[id] != SchedulerBasicP__NO_TASK || SchedulerBasicP__m_tail == id;
}

static inline bool SchedulerBasicP__pushTask(uint8_t id)
{
  if (!SchedulerBasicP__isWaiting(id)) 
    {
      if (SchedulerBasicP__m_head == SchedulerBasicP__NO_TASK) 
        {
          SchedulerBasicP__m_head = id;
          SchedulerBasicP__m_tail = id;
        }
      else 
        {
          SchedulerBasicP__m_next[SchedulerBasicP__m_tail] = id;
          SchedulerBasicP__m_tail = id;
        }
      return TRUE;
    }
  else 
    {
      return FALSE;
    }
}

#line 170
static inline error_t SchedulerBasicP__TaskBasic__postTask(uint8_t id)
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 172
    {
#line 172
      {
        unsigned char __nesc_temp = 
#line 172
        SchedulerBasicP__pushTask(id) ? SUCCESS : EBUSY;

        {
#line 172
          __nesc_atomic_end(__nesc_atomic); 
#line 172
          return __nesc_temp;
        }
      }
    }
#line 175
    __nesc_atomic_end(__nesc_atomic); }
}

# 67 "/home/rgao/lily/tinyos2/tos/interfaces/TaskBasic.nc"
inline static error_t CC2420SpiP__grant__postTask(void ){
#line 67
  unsigned char __nesc_result;
#line 67

#line 67
  __nesc_result = SchedulerBasicP__TaskBasic__postTask(CC2420SpiP__grant);
#line 67

#line 67
  return __nesc_result;
#line 67
}
#line 67
# 184 "/home/rgao/lily/tinyos2/tos/chips/cc2420/spi/CC2420SpiP.nc"
static inline void CC2420SpiP__SpiResource__granted(void )
#line 184
{
  CC2420SpiP__grant__postTask();
}

# 121 "/home/rgao/lily/tinyos2/tos/chips/msp430/x2xxx/usci/Msp430SpiNoDmaBP.nc"
static inline void /*Msp430SpiNoDmaB0P.SpiP*/Msp430SpiNoDmaBP__0__Resource__default__granted(uint8_t id)
#line 121
{
}

# 102 "/home/rgao/lily/tinyos2/tos/interfaces/Resource.nc"
inline static void /*Msp430SpiNoDmaB0P.SpiP*/Msp430SpiNoDmaBP__0__Resource__granted(uint8_t arg_0x40819ad8){
#line 102
  switch (arg_0x40819ad8) {
#line 102
    case /*CC2420SpiWireC.HplCC2420SpiC.SpiC*/Msp430SpiB0C__0__CLIENT_ID:
#line 102
      CC2420SpiP__SpiResource__granted();
#line 102
      break;
#line 102
    default:
#line 102
      /*Msp430SpiNoDmaB0P.SpiP*/Msp430SpiNoDmaBP__0__Resource__default__granted(arg_0x40819ad8);
#line 102
      break;
#line 102
    }
#line 102
}
#line 102
# 97 "/home/rgao/lily/tinyos2/tos/chips/msp430/x2xxx/usci/Msp430SpiNoDmaBP.nc"
static inline void /*Msp430SpiNoDmaB0P.SpiP*/Msp430SpiNoDmaBP__0__UsciResource__granted(uint8_t id)
#line 97
{
  /*Msp430SpiNoDmaB0P.SpiP*/Msp430SpiNoDmaBP__0__Resource__granted(id);
}

# 202 "/home/rgao/lily/tinyos2/tos/system/ArbiterP.nc"
static inline void /*Msp430UsciShareB0P.ArbiterC.Arbiter*/ArbiterP__0__Resource__default__granted(uint8_t id)
#line 202
{
}

# 102 "/home/rgao/lily/tinyos2/tos/interfaces/Resource.nc"
inline static void /*Msp430UsciShareB0P.ArbiterC.Arbiter*/ArbiterP__0__Resource__granted(uint8_t arg_0x4092e1b8){
#line 102
  switch (arg_0x4092e1b8) {
#line 102
    case /*CC2420SpiWireC.HplCC2420SpiC.SpiC.UsciC*/Msp430UsciB0C__0__CLIENT_ID:
#line 102
      /*Msp430SpiNoDmaB0P.SpiP*/Msp430SpiNoDmaBP__0__UsciResource__granted(/*CC2420SpiWireC.HplCC2420SpiC.SpiC*/Msp430SpiB0C__0__CLIENT_ID);
#line 102
      break;
#line 102
    default:
#line 102
      /*Msp430UsciShareB0P.ArbiterC.Arbiter*/ArbiterP__0__Resource__default__granted(arg_0x4092e1b8);
#line 102
      break;
#line 102
    }
#line 102
}
#line 102
# 370 "/home/rgao/lily/tinyos2/tos/chips/cc2420/spi/CC2420SpiP.nc"
static inline void CC2420SpiP__Fifo__default__readDone(uint8_t addr, uint8_t *rx_buf, uint8_t rx_len, error_t error)
#line 370
{
}

# 71 "/home/rgao/lily/tinyos2/tos/chips/cc2420/interfaces/CC2420Fifo.nc"
inline static void CC2420SpiP__Fifo__readDone(uint8_t arg_0x407893f8, uint8_t * data, uint8_t length, error_t error){
#line 71
    CC2420SpiP__Fifo__default__readDone(arg_0x407893f8, data, length, error);
#line 71
}
#line 71
# 373 "/home/rgao/lily/tinyos2/tos/chips/cc2420/spi/CC2420SpiP.nc"
static inline void CC2420SpiP__Fifo__default__writeDone(uint8_t addr, uint8_t *tx_buf, uint8_t tx_len, error_t error)
#line 373
{
}

# 91 "/home/rgao/lily/tinyos2/tos/chips/cc2420/interfaces/CC2420Fifo.nc"
inline static void CC2420SpiP__Fifo__writeDone(uint8_t arg_0x407893f8, uint8_t * data, uint8_t length, error_t error){
#line 91
    CC2420SpiP__Fifo__default__writeDone(arg_0x407893f8, data, length, error);
#line 91
}
#line 91
# 367 "/home/rgao/lily/tinyos2/tos/chips/cc2420/spi/CC2420SpiP.nc"
static inline void CC2420SpiP__Resource__default__granted(uint8_t id)
#line 367
{
}

# 102 "/home/rgao/lily/tinyos2/tos/interfaces/Resource.nc"
inline static void CC2420SpiP__Resource__granted(uint8_t arg_0x4078a920){
#line 102
    CC2420SpiP__Resource__default__granted(arg_0x4078a920);
#line 102
}
#line 102
# 69 "/home/rgao/lily/tinyos2/tos/types/TinyError.h"
static inline  error_t ecombine(error_t r1, error_t r2)




{
  return r1 == r2 ? r1 : FAIL;
}

# 81 "/home/rgao/lily/tinyos2/tos/system/StateImplP.nc"
static inline error_t StateImplP__Init__init(void )
#line 81
{
  int i;

#line 83
  for (i = 0; i < 1U; i++) {
      StateImplP__state[i] = StateImplP__S_IDLE;
    }
  return SUCCESS;
}

# 55 "/home/rgao/lily/tinyos2/tos/system/FcfsResourceQueueC.nc"
static inline error_t /*Msp430UsciShareB0P.ArbiterC.Queue*/FcfsResourceQueueC__0__Init__init(void )
#line 55
{
  memset(/*Msp430UsciShareB0P.ArbiterC.Queue*/FcfsResourceQueueC__0__resQ, /*Msp430UsciShareB0P.ArbiterC.Queue*/FcfsResourceQueueC__0__NO_ENTRY, sizeof /*Msp430UsciShareB0P.ArbiterC.Queue*/FcfsResourceQueueC__0__resQ);
  return SUCCESS;
}

# 62 "/home/rgao/lily/tinyos2/tos/interfaces/Init.nc"
inline static error_t RealMainP__SoftwareInit__init(void ){
#line 62
  unsigned char __nesc_result;
#line 62

#line 62
  __nesc_result = /*Msp430UsciShareB0P.ArbiterC.Queue*/FcfsResourceQueueC__0__Init__init();
#line 62
  __nesc_result = ecombine(__nesc_result, StateImplP__Init__init());
#line 62

#line 62
  return __nesc_result;
#line 62
}
#line 62
# 53 "/home/rgao/lily/tinyos2/tos/chips/cc2420/interfaces/CC2420Strobe.nc"
inline static cc2420_status_t TestModeC__stxccaon__strobe(void ){
#line 53
  unsigned char __nesc_result;
#line 53

#line 53
  __nesc_result = CC2420SpiP__Strobe__strobe(CC2420_STXON);
#line 53

#line 53
  return __nesc_result;
#line 53
}
#line 53
# 63 "/home/rgao/lily/tinyos2/tos/chips/cc2420/interfaces/CC2420Register.nc"
inline static cc2420_status_t TestModeC__dactst__write(uint16_t data){
#line 63
  unsigned char __nesc_result;
#line 63

#line 63
  __nesc_result = CC2420SpiP__Reg__write(CC2420_DACTST, data);
#line 63

#line 63
  return __nesc_result;
#line 63
}
#line 63
inline static cc2420_status_t TestModeC__mdmctrl1__write(uint16_t data){
#line 63
  unsigned char __nesc_result;
#line 63

#line 63
  __nesc_result = CC2420SpiP__Reg__write(CC2420_MDMCTRL1, data);
#line 63

#line 63
  return __nesc_result;
#line 63
}
#line 63
inline static cc2420_status_t TestModeC__toptst__write(uint16_t data){
#line 63
  unsigned char __nesc_result;
#line 63

#line 63
  __nesc_result = CC2420SpiP__Reg__write(CC2420_TOPTST, data);
#line 63

#line 63
  return __nesc_result;
#line 63
}
#line 63
inline static cc2420_status_t TestModeC__manor__write(uint16_t data){
#line 63
  unsigned char __nesc_result;
#line 63

#line 63
  __nesc_result = CC2420SpiP__Reg__write(CC2420_MANOR, data);
#line 63

#line 63
  return __nesc_result;
#line 63
}
#line 63
# 53 "/home/rgao/lily/tinyos2/tos/chips/cc2420/interfaces/CC2420Strobe.nc"
inline static cc2420_status_t TestModeC__SXOSCON__strobe(void ){
#line 53
  unsigned char __nesc_result;
#line 53

#line 53
  __nesc_result = CC2420SpiP__Strobe__strobe(CC2420_SXOSCON);
#line 53

#line 53
  return __nesc_result;
#line 53
}
#line 53
# 63 "/home/rgao/lily/tinyos2/tos/chips/cc2420/interfaces/CC2420Register.nc"
inline static cc2420_status_t TestModeC__MAIN__write(uint16_t data){
#line 63
  unsigned char __nesc_result;
#line 63

#line 63
  __nesc_result = CC2420SpiP__Reg__write(CC2420_MAIN, data);
#line 63

#line 63
  return __nesc_result;
#line 63
}
#line 63
# 23 "TestModeC.nc"
static inline void TestModeC__Boot__booted(void )
{
  TestModeC__MAIN__write(0x0000);
  TestModeC__SXOSCON__strobe();
  TestModeC__manor__write(0x0100);
  TestModeC__toptst__write(0x0004);
  TestModeC__mdmctrl1__write(0x0508);
  TestModeC__dactst__write(0x1800);
  TestModeC__stxccaon__strobe();
}

# 60 "/home/rgao/lily/tinyos2/tos/interfaces/Boot.nc"
inline static void RealMainP__Boot__booted(void ){
#line 60
  TestModeC__Boot__booted();
#line 60
}
#line 60
# 133 "/home/rgao/lily/tinyos2/tos/system/StateImplP.nc"
static inline bool StateImplP__State__isState(uint8_t id, uint8_t myState)
#line 133
{
  bool isState;

  /* atomic removed: atomic calls only */
#line 135
  isState = StateImplP__state[id] == myState;
  return isState;
}

#line 126
static inline bool StateImplP__State__isIdle(uint8_t id)
#line 126
{
  return StateImplP__State__isState(id, StateImplP__S_IDLE);
}

# 61 "/home/rgao/lily/tinyos2/tos/interfaces/State.nc"
inline static bool CC2420SpiP__WorkingState__isIdle(void ){
#line 61
  unsigned char __nesc_result;
#line 61

#line 61
  __nesc_result = StateImplP__State__isIdle(0U);
#line 61

#line 61
  return __nesc_result;
#line 61
}
#line 61
# 45 "/home/rgao/lily/tinyos2/tos/interfaces/SpiByte.nc"
inline static uint8_t CC2420SpiP__SpiByte__write(uint8_t tx){
#line 45
  unsigned char __nesc_result;
#line 45

#line 45
  __nesc_result = /*Msp430SpiNoDmaB0P.SpiP*/Msp430SpiNoDmaBP__0__SpiByte__write(tx);
#line 45

#line 45
  return __nesc_result;
#line 45
}
#line 45
# 209 "/home/rgao/lily/tinyos2/tos/chips/msp430/x2xxx/usci/HplMsp430UsciB0P.nc"
static inline void HplMsp430UsciB0P__Usci__clrRxIntr(void )
#line 209
{
  HplMsp430UsciB0P__IFG2 &= ~0x04;
}

# 100 "/home/rgao/lily/tinyos2/tos/chips/msp430/x2xxx/usci/HplMsp430UsciB.nc"
inline static void /*Msp430SpiNoDmaB0P.SpiP*/Msp430SpiNoDmaBP__0__Usci__clrRxIntr(void ){
#line 100
  HplMsp430UsciB0P__Usci__clrRxIntr();
#line 100
}
#line 100
# 192 "/home/rgao/lily/tinyos2/tos/chips/msp430/x2xxx/usci/Msp430SpiNoDmaBP.nc"
static inline void /*Msp430SpiNoDmaB0P.SpiP*/Msp430SpiNoDmaBP__0__SpiPacket__default__sendDone(uint8_t id, uint8_t *tx_buf, uint8_t *rx_buf, uint16_t len, error_t error)
#line 192
{
}

# 82 "/home/rgao/lily/tinyos2/tos/interfaces/SpiPacket.nc"
inline static void /*Msp430SpiNoDmaB0P.SpiP*/Msp430SpiNoDmaBP__0__SpiPacket__sendDone(uint8_t arg_0x4082e0c0, uint8_t * txBuf, uint8_t * rxBuf, uint16_t len, error_t error){
#line 82
  switch (arg_0x4082e0c0) {
#line 82
    case /*CC2420SpiWireC.HplCC2420SpiC.SpiC*/Msp430SpiB0C__0__CLIENT_ID:
#line 82
      CC2420SpiP__SpiPacket__sendDone(txBuf, rxBuf, len, error);
#line 82
      break;
#line 82
    default:
#line 82
      /*Msp430SpiNoDmaB0P.SpiP*/Msp430SpiNoDmaBP__0__SpiPacket__default__sendDone(arg_0x4082e0c0, txBuf, rxBuf, len, error);
#line 82
      break;
#line 82
    }
#line 82
}
#line 82
# 185 "/home/rgao/lily/tinyos2/tos/chips/msp430/x2xxx/usci/Msp430SpiNoDmaBP.nc"
static inline void /*Msp430SpiNoDmaB0P.SpiP*/Msp430SpiNoDmaBP__0__signalDone(void )
#line 185
{
  /*Msp430SpiNoDmaB0P.SpiP*/Msp430SpiNoDmaBP__0__SpiPacket__sendDone(/*Msp430SpiNoDmaB0P.SpiP*/Msp430SpiNoDmaBP__0__m_client, /*Msp430SpiNoDmaB0P.SpiP*/Msp430SpiNoDmaBP__0__m_tx_buf, /*Msp430SpiNoDmaB0P.SpiP*/Msp430SpiNoDmaBP__0__m_rx_buf, /*Msp430SpiNoDmaB0P.SpiP*/Msp430SpiNoDmaBP__0__m_len, 
  SUCCESS);
}

#line 168
static inline void /*Msp430SpiNoDmaB0P.SpiP*/Msp430SpiNoDmaBP__0__signalDone_task__runTask(void )
#line 168
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 169
    /*Msp430SpiNoDmaB0P.SpiP*/Msp430SpiNoDmaBP__0__signalDone();
#line 169
    __nesc_atomic_end(__nesc_atomic); }
}

# 175 "/home/rgao/lily/tinyos2/tos/system/SchedulerBasicP.nc"
static inline void SchedulerBasicP__TaskBasic__default__runTask(uint8_t id)
{
}

# 75 "/home/rgao/lily/tinyos2/tos/interfaces/TaskBasic.nc"
inline static void SchedulerBasicP__TaskBasic__runTask(uint8_t arg_0x404b1e10){
#line 75
  switch (arg_0x404b1e10) {
#line 75
    case CC2420SpiP__grant:
#line 75
      CC2420SpiP__grant__runTask();
#line 75
      break;
#line 75
    case /*Msp430SpiNoDmaB0P.SpiP*/Msp430SpiNoDmaBP__0__signalDone_task:
#line 75
      /*Msp430SpiNoDmaB0P.SpiP*/Msp430SpiNoDmaBP__0__signalDone_task__runTask();
#line 75
      break;
#line 75
    case /*Msp430UsciShareB0P.ArbiterC.Arbiter*/ArbiterP__0__grantedTask:
#line 75
      /*Msp430UsciShareB0P.ArbiterC.Arbiter*/ArbiterP__0__grantedTask__runTask();
#line 75
      break;
#line 75
    default:
#line 75
      SchedulerBasicP__TaskBasic__default__runTask(arg_0x404b1e10);
#line 75
      break;
#line 75
    }
#line 75
}
#line 75
# 391 "/home/rgao/lily/tinyos2/tos/chips/msp430/msp430hardware.h"
static inline  void __nesc_disable_interrupt(void )
{
  __dint();
  __nop();
}

# 66 "/home/rgao/lily/tinyos2/tos/platforms/z1/chips/msp430/timer/Msp430ClockP.nc"
static inline mcu_power_t Msp430ClockP__McuPowerOverride__lowestState(void )
#line 66
{
  return MSP430_POWER_LPM3;
}

# 62 "/home/rgao/lily/tinyos2/tos/interfaces/McuPowerOverride.nc"
inline static mcu_power_t McuSleepC__McuPowerOverride__lowestState(void ){
#line 62
  unsigned char __nesc_result;
#line 62

#line 62
  __nesc_result = Msp430ClockP__McuPowerOverride__lowestState();
#line 62

#line 62
  return __nesc_result;
#line 62
}
#line 62
# 74 "/home/rgao/lily/tinyos2/tos/platforms/z1/chips/msp430/McuSleepC.nc"
static inline mcu_power_t McuSleepC__getPowerState(void )
#line 74
{
  mcu_power_t pState = MSP430_POWER_LPM3;





  if (((((((
#line 77
  TACCTL0 & 0x0010 || TACCTL1 & 0x0010) || TACCTL2 & 0x0010) && (
  TACTL & 0x0300) == 0x0200) || (
  UCA0CTL1 & 0xC0) != 0x00) || (
  UCA1CTL1 & 0xC0) != 0x00) || (
  UCB0CTL1 & 0xC0) != 0x00) || (
  UCB1CTL1 & 0xC0) != 0x00) {

    pState = MSP430_POWER_LPM1;
    }


  if (ADC12CTL0 & 0x010) {
      if (ADC12CTL1 & 0x0010) {

          if (ADC12CTL1 & 0x0008) {
            pState = MSP430_POWER_LPM1;
            }
          else {
#line 94
            pState = MSP430_POWER_ACTIVE;
            }
        }
      else {
#line 95
        if (ADC12CTL1 & 0x0400 && (TACTL & 0x0300) == 0x0200) {



            pState = MSP430_POWER_LPM1;
          }
        }
    }

  return pState;
}

# 379 "/home/rgao/lily/tinyos2/tos/chips/msp430/msp430hardware.h"
static inline  mcu_power_t mcombine(mcu_power_t m1, mcu_power_t m2)
#line 379
{
  return m1 < m2 ? m1 : m2;
}

# 107 "/home/rgao/lily/tinyos2/tos/platforms/z1/chips/msp430/McuSleepC.nc"
static inline void McuSleepC__computePowerState(void )
#line 107
{
  McuSleepC__powerState = mcombine(McuSleepC__getPowerState(), 
  McuSleepC__McuPowerOverride__lowestState());
}

static inline void McuSleepC__McuSleep__sleep(void )
#line 112
{
  uint16_t temp;

#line 114
  if (McuSleepC__dirty) {
      McuSleepC__computePowerState();
    }

  temp = McuSleepC__msp430PowerBits[McuSleepC__powerState] | 0x0008;
   __asm volatile ("bis  %0, r2" :  : "m"(temp));

   __asm volatile ("" :  :  : "memory");
  __nesc_disable_interrupt();
}

# 76 "/home/rgao/lily/tinyos2/tos/interfaces/McuSleep.nc"
inline static void SchedulerBasicP__McuSleep__sleep(void ){
#line 76
  McuSleepC__McuSleep__sleep();
#line 76
}
#line 76
# 78 "/home/rgao/lily/tinyos2/tos/system/SchedulerBasicP.nc"
static __inline uint8_t SchedulerBasicP__popTask(void )
{
  if (SchedulerBasicP__m_head != SchedulerBasicP__NO_TASK) 
    {
      uint8_t id = SchedulerBasicP__m_head;

#line 83
      SchedulerBasicP__m_head = SchedulerBasicP__m_next[SchedulerBasicP__m_head];
      if (SchedulerBasicP__m_head == SchedulerBasicP__NO_TASK) 
        {
          SchedulerBasicP__m_tail = SchedulerBasicP__NO_TASK;
        }
      SchedulerBasicP__m_next[id] = SchedulerBasicP__NO_TASK;
      return id;
    }
  else 
    {
      return SchedulerBasicP__NO_TASK;
    }
}

#line 149
static inline void SchedulerBasicP__Scheduler__taskLoop(void )
{
  for (; ; ) 
    {
      uint8_t nextTask;

      { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
        {
          while ((nextTask = SchedulerBasicP__popTask()) == SchedulerBasicP__NO_TASK) 
            {
              SchedulerBasicP__McuSleep__sleep();
            }
        }
#line 161
        __nesc_atomic_end(__nesc_atomic); }
      SchedulerBasicP__TaskBasic__runTask(nextTask);
    }
}

# 72 "/home/rgao/lily/tinyos2/tos/interfaces/Scheduler.nc"
inline static void RealMainP__Scheduler__taskLoop(void ){
#line 72
  SchedulerBasicP__Scheduler__taskLoop();
#line 72
}
#line 72
# 79 "/home/rgao/lily/tinyos2/tos/chips/msp430/x2xxx/usci/HplMsp430UsciAB0RawInterruptsP.nc"
static inline void HplMsp430UsciAB0RawInterruptsP__UsciA__default__rxDone(uint8_t temp)
#line 79
{
  return;
}

# 58 "/home/rgao/lily/tinyos2/tos/chips/msp430/x2xxx/usci/HplMsp430UsciRawInterrupts.nc"
inline static void HplMsp430UsciAB0RawInterruptsP__UsciA__rxDone(uint8_t data){
#line 58
  HplMsp430UsciAB0RawInterruptsP__UsciA__default__rxDone(data);
#line 58
}
#line 58
# 98 "/home/rgao/lily/tinyos2/tos/interfaces/ArbiterInfo.nc"
inline static uint8_t /*Msp430UsciShareB0P.UsciShareP*/Msp430UsciShareP__0__ArbiterInfo__userId(void ){
#line 98
  unsigned char __nesc_result;
#line 98

#line 98
  __nesc_result = /*Msp430UsciShareB0P.ArbiterC.Arbiter*/ArbiterP__0__ArbiterInfo__userId();
#line 98

#line 98
  return __nesc_result;
#line 98
}
#line 98
# 217 "/home/rgao/lily/tinyos2/tos/chips/msp430/x2xxx/usci/HplMsp430UsciB0P.nc"
static inline void HplMsp430UsciB0P__Usci__disableRxIntr(void )
#line 217
{
  HplMsp430UsciB0P__IE2 &= ~0x04;
}

# 90 "/home/rgao/lily/tinyos2/tos/chips/msp430/x2xxx/usci/HplMsp430UsciB.nc"
inline static void /*Msp430SpiNoDmaB0P.SpiP*/Msp430SpiNoDmaBP__0__Usci__disableRxIntr(void ){
#line 90
  HplMsp430UsciB0P__Usci__disableRxIntr();
#line 90
}
#line 90
# 250 "/home/rgao/lily/tinyos2/tos/chips/msp430/x2xxx/usci/HplMsp430UsciB0P.nc"
static inline void HplMsp430UsciB0P__Usci__tx(uint8_t data)
#line 250
{
  HplMsp430UsciB0P__UCB0TXBUF = data;
}

# 108 "/home/rgao/lily/tinyos2/tos/chips/msp430/x2xxx/usci/HplMsp430UsciB.nc"
inline static void /*Msp430SpiNoDmaB0P.SpiP*/Msp430SpiNoDmaBP__0__Usci__tx(uint8_t data){
#line 108
  HplMsp430UsciB0P__Usci__tx(data);
#line 108
}
#line 108
# 254 "/home/rgao/lily/tinyos2/tos/chips/msp430/x2xxx/usci/HplMsp430UsciB0P.nc"
static inline uint8_t HplMsp430UsciB0P__Usci__rx(void )
#line 254
{
  return HplMsp430UsciB0P__UCB0RXBUF;
}

# 115 "/home/rgao/lily/tinyos2/tos/chips/msp430/x2xxx/usci/HplMsp430UsciB.nc"
inline static uint8_t /*Msp430SpiNoDmaB0P.SpiP*/Msp430SpiNoDmaBP__0__Usci__rx(void ){
#line 115
  unsigned char __nesc_result;
#line 115

#line 115
  __nesc_result = HplMsp430UsciB0P__Usci__rx();
#line 115

#line 115
  return __nesc_result;
#line 115
}
#line 115
# 199 "/home/rgao/lily/tinyos2/tos/chips/msp430/x2xxx/usci/HplMsp430UsciB0P.nc"
static inline bool HplMsp430UsciB0P__Usci__isRxIntrPending(void )
#line 199
{
  if (HplMsp430UsciB0P__IFG2 & 0x04) {
    return TRUE;
    }
#line 202
  return FALSE;
}

# 98 "/home/rgao/lily/tinyos2/tos/chips/msp430/x2xxx/usci/HplMsp430UsciB.nc"
inline static bool /*Msp430SpiNoDmaB0P.SpiP*/Msp430SpiNoDmaBP__0__Usci__isRxIntrPending(void ){
#line 98
  unsigned char __nesc_result;
#line 98

#line 98
  __nesc_result = HplMsp430UsciB0P__Usci__isRxIntrPending();
#line 98

#line 98
  return __nesc_result;
#line 98
}
#line 98
# 123 "/home/rgao/lily/tinyos2/tos/chips/msp430/x2xxx/usci/Msp430SpiNoDmaBP.nc"
static inline void /*Msp430SpiNoDmaB0P.SpiP*/Msp430SpiNoDmaBP__0__continueOp(void )
#line 123
{

  uint8_t end;
  uint8_t tmp;

  /* atomic removed: atomic calls only */
#line 128
  {
    /*Msp430SpiNoDmaB0P.SpiP*/Msp430SpiNoDmaBP__0__Usci__tx(/*Msp430SpiNoDmaB0P.SpiP*/Msp430SpiNoDmaBP__0__m_tx_buf ? /*Msp430SpiNoDmaB0P.SpiP*/Msp430SpiNoDmaBP__0__m_tx_buf[/*Msp430SpiNoDmaB0P.SpiP*/Msp430SpiNoDmaBP__0__m_pos] : 0);

    end = /*Msp430SpiNoDmaB0P.SpiP*/Msp430SpiNoDmaBP__0__m_pos + /*Msp430SpiNoDmaB0P.SpiP*/Msp430SpiNoDmaBP__0__SPI_ATOMIC_SIZE;
    if (end > /*Msp430SpiNoDmaB0P.SpiP*/Msp430SpiNoDmaBP__0__m_len) {
      end = /*Msp430SpiNoDmaB0P.SpiP*/Msp430SpiNoDmaBP__0__m_len;
      }
    while (++/*Msp430SpiNoDmaB0P.SpiP*/Msp430SpiNoDmaBP__0__m_pos < end) {
        while (!/*Msp430SpiNoDmaB0P.SpiP*/Msp430SpiNoDmaBP__0__Usci__isRxIntrPending()) ;
        tmp = /*Msp430SpiNoDmaB0P.SpiP*/Msp430SpiNoDmaBP__0__Usci__rx();
        if (/*Msp430SpiNoDmaB0P.SpiP*/Msp430SpiNoDmaBP__0__m_rx_buf) {
          /*Msp430SpiNoDmaB0P.SpiP*/Msp430SpiNoDmaBP__0__m_rx_buf[/*Msp430SpiNoDmaB0P.SpiP*/Msp430SpiNoDmaBP__0__m_pos - 1] = tmp;
          }
#line 140
        /*Msp430SpiNoDmaB0P.SpiP*/Msp430SpiNoDmaBP__0__Usci__tx(/*Msp430SpiNoDmaB0P.SpiP*/Msp430SpiNoDmaBP__0__m_tx_buf ? /*Msp430SpiNoDmaB0P.SpiP*/Msp430SpiNoDmaBP__0__m_tx_buf[/*Msp430SpiNoDmaB0P.SpiP*/Msp430SpiNoDmaBP__0__m_pos] : 0);
      }
  }
}

#line 172
static inline void /*Msp430SpiNoDmaB0P.SpiP*/Msp430SpiNoDmaBP__0__UsciInterrupts__rxDone(uint8_t data)
#line 172
{

  if (/*Msp430SpiNoDmaB0P.SpiP*/Msp430SpiNoDmaBP__0__m_rx_buf) {
    /*Msp430SpiNoDmaB0P.SpiP*/Msp430SpiNoDmaBP__0__m_rx_buf[/*Msp430SpiNoDmaB0P.SpiP*/Msp430SpiNoDmaBP__0__m_pos - 1] = data;
    }
  if (/*Msp430SpiNoDmaB0P.SpiP*/Msp430SpiNoDmaBP__0__m_pos < /*Msp430SpiNoDmaB0P.SpiP*/Msp430SpiNoDmaBP__0__m_len) {
    /*Msp430SpiNoDmaB0P.SpiP*/Msp430SpiNoDmaBP__0__continueOp();
    }
  else 
#line 179
    {
      /*Msp430SpiNoDmaB0P.SpiP*/Msp430SpiNoDmaBP__0__Usci__disableRxIntr();
      /*Msp430SpiNoDmaB0P.SpiP*/Msp430SpiNoDmaBP__0__signalDone();
    }
}

# 60 "/home/rgao/lily/tinyos2/tos/chips/msp430/x2xxx/usci/Msp430UsciShareP.nc"
static inline void /*Msp430UsciShareB0P.UsciShareP*/Msp430UsciShareP__0__Interrupts__default__rxDone(uint8_t id, uint8_t data)
#line 60
{
}

# 59 "/home/rgao/lily/tinyos2/tos/chips/msp430/x2xxx/usci/HplMsp430UsciInterrupts.nc"
inline static void /*Msp430UsciShareB0P.UsciShareP*/Msp430UsciShareP__0__Interrupts__rxDone(uint8_t arg_0x40916cf8, uint8_t data){
#line 59
  switch (arg_0x40916cf8) {
#line 59
    case /*CC2420SpiWireC.HplCC2420SpiC.SpiC.UsciC*/Msp430UsciB0C__0__CLIENT_ID:
#line 59
      /*Msp430SpiNoDmaB0P.SpiP*/Msp430SpiNoDmaBP__0__UsciInterrupts__rxDone(data);
#line 59
      break;
#line 59
    default:
#line 59
      /*Msp430UsciShareB0P.UsciShareP*/Msp430UsciShareP__0__Interrupts__default__rxDone(arg_0x40916cf8, data);
#line 59
      break;
#line 59
    }
#line 59
}
#line 59
# 90 "/home/rgao/lily/tinyos2/tos/interfaces/ArbiterInfo.nc"
inline static bool /*Msp430UsciShareB0P.UsciShareP*/Msp430UsciShareP__0__ArbiterInfo__inUse(void ){
#line 90
  unsigned char __nesc_result;
#line 90

#line 90
  __nesc_result = /*Msp430UsciShareB0P.ArbiterC.Arbiter*/ArbiterP__0__ArbiterInfo__inUse();
#line 90

#line 90
  return __nesc_result;
#line 90
}
#line 90
# 54 "/home/rgao/lily/tinyos2/tos/chips/msp430/x2xxx/usci/Msp430UsciShareP.nc"
static inline void /*Msp430UsciShareB0P.UsciShareP*/Msp430UsciShareP__0__RawInterrupts__rxDone(uint8_t data)
#line 54
{
  if (/*Msp430UsciShareB0P.UsciShareP*/Msp430UsciShareP__0__ArbiterInfo__inUse()) {
    /*Msp430UsciShareB0P.UsciShareP*/Msp430UsciShareP__0__Interrupts__rxDone(/*Msp430UsciShareB0P.UsciShareP*/Msp430UsciShareP__0__ArbiterInfo__userId(), data);
    }
}

# 59 "/home/rgao/lily/tinyos2/tos/chips/msp430/x2xxx/usci/HplMsp430UsciInterrupts.nc"
inline static void HplMsp430UsciB0P__Interrupts__rxDone(uint8_t data){
#line 59
  /*Msp430UsciShareB0P.UsciShareP*/Msp430UsciShareP__0__RawInterrupts__rxDone(data);
#line 59
}
#line 59
# 82 "/home/rgao/lily/tinyos2/tos/chips/msp430/x2xxx/usci/HplMsp430UsciB0P.nc"
static inline void HplMsp430UsciB0P__UsciRawInterrupts__rxDone(uint8_t temp)
#line 82
{
  HplMsp430UsciB0P__Interrupts__rxDone(temp);
}

# 58 "/home/rgao/lily/tinyos2/tos/chips/msp430/x2xxx/usci/HplMsp430UsciRawInterrupts.nc"
inline static void HplMsp430UsciAB0RawInterruptsP__UsciB__rxDone(uint8_t data){
#line 58
  HplMsp430UsciB0P__UsciRawInterrupts__rxDone(data);
#line 58
}
#line 58
# 75 "/home/rgao/lily/tinyos2/tos/chips/msp430/x2xxx/usci/HplMsp430UsciAB0RawInterruptsP.nc"
static inline void HplMsp430UsciAB0RawInterruptsP__UsciA__default__txDone(void )
#line 75
{
  return;
}

# 53 "/home/rgao/lily/tinyos2/tos/chips/msp430/x2xxx/usci/HplMsp430UsciRawInterrupts.nc"
inline static void HplMsp430UsciAB0RawInterruptsP__UsciA__txDone(void ){
#line 53
  HplMsp430UsciAB0RawInterruptsP__UsciA__default__txDone();
#line 53
}
#line 53
# 190 "/home/rgao/lily/tinyos2/tos/chips/msp430/x2xxx/usci/Msp430SpiNoDmaBP.nc"
static inline void /*Msp430SpiNoDmaB0P.SpiP*/Msp430SpiNoDmaBP__0__UsciInterrupts__txDone(void )
#line 190
{
}

# 59 "/home/rgao/lily/tinyos2/tos/chips/msp430/x2xxx/usci/Msp430UsciShareP.nc"
static inline void /*Msp430UsciShareB0P.UsciShareP*/Msp430UsciShareP__0__Interrupts__default__txDone(uint8_t id)
#line 59
{
}

# 54 "/home/rgao/lily/tinyos2/tos/chips/msp430/x2xxx/usci/HplMsp430UsciInterrupts.nc"
inline static void /*Msp430UsciShareB0P.UsciShareP*/Msp430UsciShareP__0__Interrupts__txDone(uint8_t arg_0x40916cf8){
#line 54
  switch (arg_0x40916cf8) {
#line 54
    case /*CC2420SpiWireC.HplCC2420SpiC.SpiC.UsciC*/Msp430UsciB0C__0__CLIENT_ID:
#line 54
      /*Msp430SpiNoDmaB0P.SpiP*/Msp430SpiNoDmaBP__0__UsciInterrupts__txDone();
#line 54
      break;
#line 54
    default:
#line 54
      /*Msp430UsciShareB0P.UsciShareP*/Msp430UsciShareP__0__Interrupts__default__txDone(arg_0x40916cf8);
#line 54
      break;
#line 54
    }
#line 54
}
#line 54
# 49 "/home/rgao/lily/tinyos2/tos/chips/msp430/x2xxx/usci/Msp430UsciShareP.nc"
static inline void /*Msp430UsciShareB0P.UsciShareP*/Msp430UsciShareP__0__RawInterrupts__txDone(void )
#line 49
{
  if (/*Msp430UsciShareB0P.UsciShareP*/Msp430UsciShareP__0__ArbiterInfo__inUse()) {
    /*Msp430UsciShareB0P.UsciShareP*/Msp430UsciShareP__0__Interrupts__txDone(/*Msp430UsciShareB0P.UsciShareP*/Msp430UsciShareP__0__ArbiterInfo__userId());
    }
}

# 54 "/home/rgao/lily/tinyos2/tos/chips/msp430/x2xxx/usci/HplMsp430UsciInterrupts.nc"
inline static void HplMsp430UsciB0P__Interrupts__txDone(void ){
#line 54
  /*Msp430UsciShareB0P.UsciShareP*/Msp430UsciShareP__0__RawInterrupts__txDone();
#line 54
}
#line 54
# 86 "/home/rgao/lily/tinyos2/tos/chips/msp430/x2xxx/usci/HplMsp430UsciB0P.nc"
static inline void HplMsp430UsciB0P__UsciRawInterrupts__txDone(void )
#line 86
{
  HplMsp430UsciB0P__Interrupts__txDone();
}

# 53 "/home/rgao/lily/tinyos2/tos/chips/msp430/x2xxx/usci/HplMsp430UsciRawInterrupts.nc"
inline static void HplMsp430UsciAB0RawInterruptsP__UsciB__txDone(void ){
#line 53
  HplMsp430UsciB0P__UsciRawInterrupts__txDone();
#line 53
}
#line 53
# 411 "/home/rgao/lily/tinyos2/tos/chips/msp430/msp430hardware.h"
  __nesc_atomic_t __nesc_atomic_start(void )
{
  __nesc_atomic_t result = (__read_status_register() & 0x0008) != 0;

#line 414
  __nesc_disable_interrupt();
   __asm volatile ("" :  :  : "memory");
  return result;
}

  void __nesc_atomic_end(__nesc_atomic_t reenable_interrupts)
{
   __asm volatile ("" :  :  : "memory");
  if (reenable_interrupts) {
    __nesc_enable_interrupt();
    }
}

# 11 "/home/rgao/lily/tinyos2/tos/chips/msp430/timer/Msp430TimerCommonP.nc"
__attribute((wakeup)) __attribute((interrupt(0x0032)))  void sig_TIMERA0_VECTOR(void )
#line 11
{
#line 11
  Msp430TimerCommonP__VectorTimerA0__fired();
}

# 180 "/home/rgao/lily/tinyos2/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static void /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Event__fired(void )
{
  if (/*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Control__getControl().cap) {
    /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Capture__captured(/*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Capture__getEvent());
    }
  else {
#line 185
    /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Compare__fired();
    }
}

#line 180
static void /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Event__fired(void )
{
  if (/*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Control__getControl().cap) {
    /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Capture__captured(/*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Capture__getEvent());
    }
  else {
#line 185
    /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Compare__fired();
    }
}

#line 180
static void /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Event__fired(void )
{
  if (/*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Control__getControl().cap) {
    /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Capture__captured(/*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Capture__getEvent());
    }
  else {
#line 185
    /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Compare__fired();
    }
}

# 12 "/home/rgao/lily/tinyos2/tos/chips/msp430/timer/Msp430TimerCommonP.nc"
__attribute((wakeup)) __attribute((interrupt(0x0030)))  void sig_TIMERA1_VECTOR(void )
#line 12
{
#line 12
  Msp430TimerCommonP__VectorTimerA1__fired();
}

#line 13
__attribute((wakeup)) __attribute((interrupt(0x003A)))  void sig_TIMERB0_VECTOR(void )
#line 13
{
#line 13
  Msp430TimerCommonP__VectorTimerB0__fired();
}

# 146 "/home/rgao/lily/tinyos2/tos/chips/msp430/timer/Msp430TimerP.nc"
static void /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__Event__default__fired(uint8_t n)
{
}

# 39 "/home/rgao/lily/tinyos2/tos/chips/msp430/timer/Msp430TimerEvent.nc"
static void /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__Event__fired(uint8_t arg_0x4053d640){
#line 39
  switch (arg_0x4053d640) {
#line 39
    case 0:
#line 39
      /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Event__fired();
#line 39
      break;
#line 39
    case 1:
#line 39
      /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Event__fired();
#line 39
      break;
#line 39
    case 2:
#line 39
      /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Event__fired();
#line 39
      break;
#line 39
    case 3:
#line 39
      /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Event__fired();
#line 39
      break;
#line 39
    case 4:
#line 39
      /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Event__fired();
#line 39
      break;
#line 39
    case 5:
#line 39
      /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Event__fired();
#line 39
      break;
#line 39
    case 6:
#line 39
      /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Event__fired();
#line 39
      break;
#line 39
    case 7:
#line 39
      /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__Overflow__fired();
#line 39
      break;
#line 39
    default:
#line 39
      /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__Event__default__fired(arg_0x4053d640);
#line 39
      break;
#line 39
    }
#line 39
}
#line 39
# 14 "/home/rgao/lily/tinyos2/tos/chips/msp430/timer/Msp430TimerCommonP.nc"
__attribute((wakeup)) __attribute((interrupt(0x0038)))  void sig_TIMERB1_VECTOR(void )
#line 14
{
#line 14
  Msp430TimerCommonP__VectorTimerB1__fired();
}

# 63 "/home/rgao/lily/tinyos2/tos/system/RealMainP.nc"
  int main(void )
#line 63
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
    {





      {
      }
#line 71
      ;

      RealMainP__Scheduler__init();





      RealMainP__PlatformInit__init();
      while (RealMainP__Scheduler__runNextTask()) ;





      RealMainP__SoftwareInit__init();
      while (RealMainP__Scheduler__runNextTask()) ;
    }
#line 88
    __nesc_atomic_end(__nesc_atomic); }


  __nesc_enable_interrupt();

  RealMainP__Boot__booted();


  RealMainP__Scheduler__taskLoop();




  return -1;
}

# 134 "/home/rgao/lily/tinyos2/tos/system/SchedulerBasicP.nc"
static bool SchedulerBasicP__Scheduler__runNextTask(void )
{
  uint8_t nextTask;

  /* atomic removed: atomic calls only */
#line 138
  {
    nextTask = SchedulerBasicP__popTask();
    if (nextTask == SchedulerBasicP__NO_TASK) 
      {
        {
          unsigned char __nesc_temp = 
#line 142
          FALSE;

#line 142
          return __nesc_temp;
        }
      }
  }
#line 145
  SchedulerBasicP__TaskBasic__runTask(nextTask);
  return TRUE;
}

# 190 "/home/rgao/lily/tinyos2/tos/system/ArbiterP.nc"
static void /*Msp430UsciShareB0P.ArbiterC.Arbiter*/ArbiterP__0__grantedTask__runTask(void )
#line 190
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 191
    {
      /*Msp430UsciShareB0P.ArbiterC.Arbiter*/ArbiterP__0__resId = /*Msp430UsciShareB0P.ArbiterC.Arbiter*/ArbiterP__0__reqResId;
      /*Msp430UsciShareB0P.ArbiterC.Arbiter*/ArbiterP__0__state = /*Msp430UsciShareB0P.ArbiterC.Arbiter*/ArbiterP__0__RES_BUSY;
    }
#line 194
    __nesc_atomic_end(__nesc_atomic); }
  /*Msp430UsciShareB0P.ArbiterC.Arbiter*/ArbiterP__0__ResourceConfigure__configure(/*Msp430UsciShareB0P.ArbiterC.Arbiter*/ArbiterP__0__resId);
  /*Msp430UsciShareB0P.ArbiterC.Arbiter*/ArbiterP__0__Resource__granted(/*Msp430UsciShareB0P.ArbiterC.Arbiter*/ArbiterP__0__resId);
}

# 329 "/home/rgao/lily/tinyos2/tos/chips/cc2420/spi/CC2420SpiP.nc"
static void CC2420SpiP__SpiPacket__sendDone(uint8_t *tx_buf, uint8_t *rx_buf, 
uint16_t len, error_t error)
#line 330
{
  if (CC2420SpiP__m_addr & 0x40) {
      CC2420SpiP__Fifo__readDone(CC2420SpiP__m_addr & ~0x40, rx_buf, len, error);
    }
  else 
#line 333
    {
      CC2420SpiP__Fifo__writeDone(CC2420SpiP__m_addr, tx_buf, len, error);
    }
}

#line 358
static void CC2420SpiP__grant__runTask(void )
#line 358
{
  uint8_t holder;

#line 360
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 360
    {
      holder = CC2420SpiP__m_holder;
    }
#line 362
    __nesc_atomic_end(__nesc_atomic); }
  CC2420SpiP__Resource__granted(holder);
}

#line 305
static cc2420_status_t CC2420SpiP__Reg__write(uint8_t addr, uint16_t data)
#line 305
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 306
    {
      if (CC2420SpiP__WorkingState__isIdle()) {
          {
            unsigned char __nesc_temp = 
#line 308
            0;

            {
#line 308
              __nesc_atomic_end(__nesc_atomic); 
#line 308
              return __nesc_temp;
            }
          }
        }
    }
#line 312
    __nesc_atomic_end(__nesc_atomic); }
#line 311
  CC2420SpiP__SpiByte__write(addr);
  CC2420SpiP__SpiByte__write(data >> 8);
  return CC2420SpiP__SpiByte__write(data & 0xff);
}

# 101 "/home/rgao/lily/tinyos2/tos/chips/msp430/x2xxx/usci/Msp430SpiNoDmaBP.nc"
static uint8_t /*Msp430SpiNoDmaB0P.SpiP*/Msp430SpiNoDmaBP__0__SpiByte__write(uint8_t tx)
#line 101
{
  uint8_t byte;


  /*Msp430SpiNoDmaB0P.SpiP*/Msp430SpiNoDmaBP__0__Usci__tx(tx);
  while (!/*Msp430SpiNoDmaB0P.SpiP*/Msp430SpiNoDmaBP__0__Usci__isRxIntrPending()) ;
  /*Msp430SpiNoDmaB0P.SpiP*/Msp430SpiNoDmaBP__0__Usci__clrRxIntr();
  byte = /*Msp430SpiNoDmaB0P.SpiP*/Msp430SpiNoDmaBP__0__Usci__rx();

  return byte;
}

# 318 "/home/rgao/lily/tinyos2/tos/chips/cc2420/spi/CC2420SpiP.nc"
static cc2420_status_t CC2420SpiP__Strobe__strobe(uint8_t addr)
#line 318
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 319
    {
      if (CC2420SpiP__WorkingState__isIdle()) {
          {
            unsigned char __nesc_temp = 
#line 321
            0;

            {
#line 321
              __nesc_atomic_end(__nesc_atomic); 
#line 321
              return __nesc_temp;
            }
          }
        }
    }
#line 325
    __nesc_atomic_end(__nesc_atomic); }
#line 325
  return CC2420SpiP__SpiByte__write(addr);
}

# 52 "/home/rgao/lily/tinyos2/tos/chips/msp430/x2xxx/usci/HplMsp430UsciAB0RawInterruptsP.nc"
__attribute((wakeup)) __attribute((interrupt(0x002E)))  void sig_USCIAB0RX_VECTOR(void )
#line 52
{
  uint8_t temp;

#line 54
  if (IFG2 & 0x01) {
      temp = UCA0RXBUF;
      HplMsp430UsciAB0RawInterruptsP__UsciA__rxDone(temp);
    }
  if (IFG2 & 0x04) {
      temp = UCB0RXBUF;
      HplMsp430UsciAB0RawInterruptsP__UsciB__rxDone(temp);
    }
}

# 153 "/home/rgao/lily/tinyos2/tos/system/ArbiterP.nc"
static bool /*Msp430UsciShareB0P.ArbiterC.Arbiter*/ArbiterP__0__ArbiterInfo__inUse(void )
#line 153
{
  /* atomic removed: atomic calls only */
#line 154
  {
    if (/*Msp430UsciShareB0P.ArbiterC.Arbiter*/ArbiterP__0__state == /*Msp430UsciShareB0P.ArbiterC.Arbiter*/ArbiterP__0__RES_CONTROLLED) 
      {
        unsigned char __nesc_temp = 
#line 156
        FALSE;

#line 156
        return __nesc_temp;
      }
  }
#line 158
  return TRUE;
}






static uint8_t /*Msp430UsciShareB0P.ArbiterC.Arbiter*/ArbiterP__0__ArbiterInfo__userId(void )
#line 166
{
  /* atomic removed: atomic calls only */
#line 167
  {
    if (/*Msp430UsciShareB0P.ArbiterC.Arbiter*/ArbiterP__0__state != /*Msp430UsciShareB0P.ArbiterC.Arbiter*/ArbiterP__0__RES_BUSY) 
      {
        unsigned char __nesc_temp = 
#line 169
        /*Msp430UsciShareB0P.ArbiterC.Arbiter*/ArbiterP__0__NO_RES;

#line 169
        return __nesc_temp;
      }
#line 170
    {
      unsigned char __nesc_temp = 
#line 170
      /*Msp430UsciShareB0P.ArbiterC.Arbiter*/ArbiterP__0__resId;

#line 170
      return __nesc_temp;
    }
  }
}

# 64 "/home/rgao/lily/tinyos2/tos/chips/msp430/x2xxx/usci/HplMsp430UsciAB0RawInterruptsP.nc"
__attribute((wakeup)) __attribute((interrupt(0x002C)))  void sig_USCIAB0TX_VECTOR(void )
#line 64
{
  if ((IFG2 & 0x02) | (IFG2 & 0x01)) {
      HplMsp430UsciAB0RawInterruptsP__UsciA__txDone();
    }
  if ((IFG2 & 0x08) | (IFG2 & 0x04)) {
      HplMsp430UsciAB0RawInterruptsP__UsciB__txDone();
    }
}

