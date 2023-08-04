/*******************************************************************************
*
* Copyright 2004-2013 Freescale Semiconductor, Inc.
* License: NXP LA_OPT_NXP_Software_License
*
* This software is owned or controlled by Freescale Semiconductor.
* Use of this software is governed by the Freescale FreeMASTER License
* distributed with this Material.
* See the LICENSE file distributed for more details.
* 
****************************************************************************//*!
*
* @brief  FreeMASTER Driver hardware dependent stuff
*
*******************************************************************************/

#ifndef __FREEMASTER_56F8xxx_H
#define __FREEMASTER_56F8xxx_H

/******************************************************************************
* platform-specific default configuration
******************************************************************************/

/* using 16bit addressing commands */
#ifndef FMSTR_USE_NOEX_CMDS
#define FMSTR_USE_NOEX_CMDS 1
#endif

/* using 32bit addressing commands */
#ifndef FMSTR_USE_EX_CMDS
#define FMSTR_USE_EX_CMDS   1
#endif

/* using inline access to buffer memory */
#ifndef FMSTR_USE_INLINE_BUFFER_ACCESS
#define FMSTR_USE_INLINE_BUFFER_ACCESS 0
#endif

/* compiler inline statement */
#define FMSTR_INLINE inline

/*****************************************************************************
* Board configuration information 
******************************************************************************/

#define FMSTR_PROT_VER           3U      /* protocol version 3 */
#define FMSTR_CFG_FLAGS          FMSTR_CFG_REC_LARGE_MODE      /* board info flags */
#define FMSTR_CFG_BUS_WIDTH      1U      /* data bus width */
#define FMSTR_GLOB_VERSION_MAJOR 2U      /* driver version */
#define FMSTR_GLOB_VERSION_MINOR 0U
#define FMSTR_IDT_STRING "56F8xxx FreeMASTER Driver"
#define FMSTR_TSA_FLAGS FMSTR_TSA_INFO_HV2BA    /* HawkV2 TSA workaround */

/******************************************************************************
* platform-specific types
******************************************************************************/

typedef unsigned char  FMSTR_U8;        /* smallest memory entity (mostly 8bit) */
typedef unsigned short FMSTR_U16;       /* 16bit value */
typedef unsigned long  FMSTR_U32;       /* 32bit value */

typedef signed char    FMSTR_S8;        /* signed 8bit value */
typedef signed short   FMSTR_S16;       /* signed 16bit value */
typedef signed long    FMSTR_S32;       /* signed 32bit value */

#if FMSTR_REC_FLOAT_TRIG
typedef float          FMSTR_FLOAT;     /* float value */
#endif

typedef unsigned short FMSTR_FLAGS;     /* type to be union-ed with flags (at least 8 bits) */
typedef unsigned short FMSTR_SIZE8;     /* size value (at least 8 bits) */
typedef signed short   FMSTR_INDEX;     /* general for-loop index (must be signed) */

typedef unsigned char  FMSTR_BCHR;      /* type of a single character in comm.buffer */
typedef unsigned char* FMSTR_BPTR;      /* pointer within a communication buffer */

typedef unsigned short FMSTR_SCISR;     /* data type to store SCI status register */

/******************************************************************************
* communication buffer access functions (non-inline)
******************************************************************************/

void FMSTR_CopyMemory(FMSTR_ADDR nDestAddr, FMSTR_ADDR nSrcAddr, FMSTR_SIZE8 nSize);
FMSTR_BPTR FMSTR_CopyToBuffer(FMSTR_BPTR pDestBuff, FMSTR_ADDR nSrcAddr, FMSTR_SIZE8 nSize);
FMSTR_BPTR FMSTR_CopyFromBuffer(FMSTR_ADDR nDestAddr, FMSTR_BPTR pSrcBuff, FMSTR_SIZE8 nSize);
void FMSTR_CopyFromBufferWithMask(FMSTR_ADDR nDestAddr, FMSTR_BPTR pSrcBuff, FMSTR_SIZE8 nSize);
FMSTR_BPTR FMSTR_AddressFromBuffer(FMSTR_ADDR* pAddr, FMSTR_BPTR pSrc);
FMSTR_BPTR FMSTR_AddressToBuffer(FMSTR_BPTR pDest, FMSTR_ADDR nAddr);

/* FMSTR_SetExAddr is needed only if both EX and non-EX commands are used */
#if (FMSTR_USE_EX_CMDS) && (FMSTR_USE_NOEX_CMDS)
void FMSTR_SetExAddr(FMSTR_BOOL bNextAddrIsEx);
#else
/* otherwise, we know what addresses are used, (ignore FMSTR_SetExAddr) */
#define FMSTR_SetExAddr(bNextAddrIsEx) 
#endif

/******************************************************************************
* communication buffer access functions (inlines/functions - depending on cfg)
******************************************************************************/

#if FMSTR_USE_INLINE_BUFFER_ACCESS

inline FMSTR_BPTR FMSTR_ValueFromBuffer16(register FMSTR_U16* pDest, register FMSTR_BPTR pSrc)
{
    register FMSTR_U8* pd = (FMSTR_U8*) pDest;
    register FMSTR_U8 tmp;
    asm
    {
        move.bp X:(pSrc)+,tmp
        move.bp tmp,X:(pd)+
        move.bp X:(pSrc)+,tmp
        move.bp tmp,X:(pd)+
    }
    return pSrc;
}

inline FMSTR_BPTR FMSTR_ValueFromBuffer32(register FMSTR_U32* pDest, register FMSTR_BPTR pSrc)
{
    register FMSTR_U8* pd = (FMSTR_U8*) pDest;
    register FMSTR_U8 tmp;
    asm
    {
        move.bp X:(pSrc)+,tmp
        move.bp tmp,X:(pd)+
        move.bp X:(pSrc)+,tmp
        move.bp tmp,X:(pd)+
        move.bp X:(pSrc)+,tmp
        move.bp tmp,X:(pd)+
        move.bp X:(pSrc)+,tmp
        move.bp tmp,X:(pd)+
    }
    return pSrc;
}

inline FMSTR_BPTR FMSTR_ValueToBuffer16(register FMSTR_BPTR pDest, FMSTR_U16 src)
{
    register FMSTR_U8* ps = (FMSTR_U8*) & src;
    register FMSTR_U8 tmp;
    asm
    {
        move.bp X:(ps)+,tmp
        move.bp tmp,X:(pDest)+
        move.bp X:(ps)+,tmp
        move.bp tmp,X:(pDest)+
    }
    return pDest;
}

inline FMSTR_BPTR FMSTR_ValueToBuffer32(register FMSTR_BPTR pDest, FMSTR_U32 src)
{
    register FMSTR_U8* ps = (FMSTR_U8*) & src;
    register FMSTR_U8 tmp;
    asm
    {
        move.bp X:(ps)+,tmp
        move.bp tmp,X:(pDest)+
        move.bp X:(ps)+,tmp
        move.bp tmp,X:(pDest)+
        move.bp X:(ps)+,tmp
        move.bp tmp,X:(pDest)+
        move.bp X:(ps)+,tmp
        move.bp tmp,X:(pDest)+
    }
    return pDest;
}

#else /* FMSTR_USE_INLINE_BUFFER_ACCESS */

inline FMSTR_BPTR FMSTR_ValueFromBuffer16(FMSTR_U16* pDest, FMSTR_BPTR pSrc)
{
    return FMSTR_CopyFromBuffer((FMSTR_ADDR)(FMSTR_U8*)pDest, pSrc, 2);
}

inline FMSTR_BPTR FMSTR_ValueFromBuffer32(FMSTR_U32* pDest, FMSTR_BPTR pSrc)
{
    return FMSTR_CopyFromBuffer((FMSTR_ADDR)(FMSTR_U8*)pDest, pSrc, 4);
}

inline FMSTR_BPTR FMSTR_ValueToBuffer16(FMSTR_BPTR pDest, FMSTR_U16 src)
{
    return FMSTR_CopyToBuffer(pDest, (FMSTR_ADDR)(FMSTR_U8*)&src, 2);
}

inline FMSTR_BPTR FMSTR_ValueToBuffer32(FMSTR_BPTR pDest, FMSTR_U32 src)
{
    return FMSTR_CopyToBuffer(pDest, (FMSTR_ADDR)(FMSTR_U8*)&src, 4);
}

#endif /* FMSTR_USE_INLINE_BUFFER_ACCESS */

/******************************************************************************
* communication buffer access inlines
******************************************************************************/

inline FMSTR_BPTR FMSTR_SkipInBuffer(FMSTR_BPTR pDest, FMSTR_SIZE8 nSize)
{
    return pDest + nSize;
}

inline FMSTR_BPTR FMSTR_ValueFromBuffer8(FMSTR_U8* pDest, register FMSTR_BPTR pSrc)
{
    *pDest = *pSrc++;
    return pSrc;
}

inline FMSTR_BPTR FMSTR_ValueToBuffer8(FMSTR_BPTR pDest, FMSTR_U8 src)
{
    *pDest++ = src;
    return pDest;
}

inline FMSTR_BPTR FMSTR_ConstToBuffer8(FMSTR_BPTR pDest, FMSTR_U8 src) 
{
    *pDest++ = src;
    return pDest; 
}

inline FMSTR_BPTR FMSTR_ConstToBuffer16(FMSTR_BPTR pDest, FMSTR_U16 src) 
{
    return FMSTR_ConstToBuffer8(FMSTR_ConstToBuffer8(pDest, (FMSTR_U8) src), (FMSTR_U8)(src>>8));
}

/****************************************************************************************
* memory access helper macros (used in recorder trigger compare routines)
*****************************************************************************************/

inline FMSTR_S8 FMSTR_GetS8(register FMSTR_ADDR addr)
{
    if(sizeof(FMSTR_ADDR) == 4)
    {
        register void* raddr;
        register FMSTR_S8 tmp = 0;
        
        asm ( tfr addr,A );
        asm ( move.l A10, raddr );
        asm ( move.bp X:(raddr),tmp );
        return tmp;
    }
    else
    {
        return *(FMSTR_S8*)addr;
    }   
}

inline FMSTR_U8 FMSTR_GetU8(register FMSTR_ADDR addr)
{
    if(sizeof(FMSTR_ADDR) == 4)
    {
        register void* raddr;
        register FMSTR_U8 tmp = 0;
        
        asm ( tfr addr,A );
        asm ( move.l A10, raddr );
        asm ( moveu.bp X:(raddr),tmp );
        return tmp;
    }
    else
    {
        return *(FMSTR_U8*)addr;
    }   
}

inline FMSTR_S16 FMSTR_GetS16(register FMSTR_ADDR addr)
{
    if(sizeof(FMSTR_ADDR) == 4)
    {
        register void* raddr;
        register FMSTR_S16 tmp;
        
        asm ( tfr addr,A );
        asm ( move.l A10, raddr );
        asm ( asra raddr );
        asm ( move.w X:(raddr),tmp );
        return tmp;
    }
    else
    {
        return *(FMSTR_S16*)(FMSTR_U8*)addr;
    }   
}

inline FMSTR_U16 FMSTR_GetU16(register FMSTR_ADDR addr)
{
    if(sizeof(FMSTR_ADDR) == 4)
    {
        register void* raddr;
        register FMSTR_U16 tmp;
        
        asm ( tfr addr,A );
        asm ( move.l A10, raddr );
        asm ( asra raddr );
        asm ( move.w X:(raddr),tmp );
        return tmp;
    }
    else
    {
        return *(FMSTR_U16*)(FMSTR_U8*)addr;
    }   
}

inline FMSTR_S32 FMSTR_GetS32(register FMSTR_ADDR addr)
{
    if(sizeof(FMSTR_ADDR) == 4)
    {
        register void* raddr;
        register FMSTR_S32 tmp;
        
        asm ( tfr addr,A );
        asm ( move.l A10, raddr );
        asm ( asra raddr );
        asm ( move.l X:(raddr),tmp );
        return tmp;
    }
    else
    {
        return *(FMSTR_S32*)(FMSTR_U8*)addr;
    }   
}

inline FMSTR_U32 FMSTR_GetU32(register FMSTR_ADDR addr)
{
    if(sizeof(FMSTR_ADDR) == 4)
    {
        register void* raddr;
        register FMSTR_U32 tmp;
        
        asm ( tfr addr,A );
        asm ( move.l A10, raddr );
        asm ( asra raddr );
        asm ( move.l X:(raddr),tmp );
        return tmp;
    }
    else
    {
        return *(FMSTR_U32*)(FMSTR_U8*)addr;
    }   
}

#if FMSTR_REC_FLOAT_TRIG
inline FMSTR_FLOAT FMSTR_GetFloat(register FMSTR_ADDR addr)
{
    if(sizeof(FMSTR_ADDR) == 4)
    {
        register void* raddr;
        register FMSTR_U32 tmp;

        asm ( tfr addr,A );
        asm ( move.l A10, raddr );
        return *(FMSTR_FLOAT*)(FMSTR_U32*)raddr;
    }
    else
    {
        return *(FMSTR_FLOAT*)(FMSTR_U8*)addr;
    }   
}
#endif

/****************************************************************************************
* Other helper macros
*****************************************************************************************/

/* This macro assigns C pointer to FMSTR_ADDR-typed variable */
/* should be done in assembly not to trim far-space pointers in SDM */

#define FMSTR_ARR2ADDR(addrvar,arr) /*lint -e{717} */ \
    do{ /*lint -esym(529,dw,rreg) -esym(530,dw,rreg) */ \
        register FMSTR_U32 dw;          \
        register void* rreg;            \
        asm ( move.l #>>arr, rreg );    \
        asm ( move.l rreg, dw );        \
        addrvar = (FMSTR_ADDR) dw;      \
    } while(0)

#define FMSTR_PTR2ADDR(tmpAddr, ptr) ( /*lint -e{923} */ tmpAddr = (FMSTR_ADDR) (FMSTR_U8*) ptr )

/****************************************************************************************
* Platform-specific configuration
*****************************************************************************************/

/* sanity check, at least one of the modes should be enabled */
#if (!FMSTR_USE_EX_CMDS) && (!FMSTR_USE_NOEX_CMDS)
#error At least one of FMSTR_USE_EX_CMDS or FMSTR_USE_NOEX_CMDS should be set non-zero
#endif

/* JTAG base address definition */
#if FMSTR_USE_JTAG
    /* hardwired JTAG address on all HawkV2 devices */
    #ifndef FMSTR_JTAG_BASE
    #define FMSTR_JTAG_BASE 0xFFFF00 
    #endif
#endif

/* MSCAN functionality tested on 56F8xxx */
#define FMSTR_CANHW_MSCAN 1 

/* FlexCAN functionality tested on 56F8xxx */
#define FMSTR_CANHW_FLEXCAN 1 

/* FlexCAN functionality tested on 56F84xxx family */
#define FMSTR_CANHW_FLEXCAN32 1

/****************************************************************************************
* General peripheral space access macros
*****************************************************************************************/

#define FMSTR_SETBIT(base, offset, bit)     (*(volatile FMSTR_U16*)(((FMSTR_U32)(base))+(offset)) |= bit)
#define FMSTR_CLRBIT(base, offset, bit)     (*(volatile FMSTR_U16*)(((FMSTR_U32)(base))+(offset)) &= (FMSTR_U16)~((FMSTR_U16)(bit)))
#define FMSTR_TSTBIT(base, offset, bit)     (*(volatile FMSTR_U16*)(((FMSTR_U32)(base))+(offset)) & (bit))
#define FMSTR_SETREG(base, offset, value)   (*(volatile FMSTR_U16*)(((FMSTR_U32)(base))+(offset)) = value)
#define FMSTR_GETREG(base, offset)          (*(volatile FMSTR_U16*)(((FMSTR_U32)(base))+(offset)))

#define FMSTR_SETBIT32(base, offset, bit)   (*(volatile FMSTR_U32*)(((FMSTR_U32)(base))+(offset)) |= (bit))
#define FMSTR_CLRBIT32(base, offset, bit)   (*(volatile FMSTR_U32*)(((FMSTR_U32)(base))+(offset)) &= (FMSTR_U32)~((FMSTR_U32)(bit)))
#define FMSTR_TSTBIT32(base, offset, bit)   (*(volatile FMSTR_U32*)(((FMSTR_U32)(base))+(offset)) & (bit))
#define FMSTR_SETREG32(base, offset, value) (*(volatile FMSTR_U32*)(((FMSTR_U32)(base))+(offset)) = value)
#define FMSTR_GETREG32(base, offset)        (*(volatile FMSTR_U32*)(((FMSTR_U32)(base))+(offset)))

#define FMSTR_SETBITFIELD(base, offset, mask, value)     (*(volatile FMSTR_U16*)(((FMSTR_U32)(base))+(offset)) = ((*(volatile FMSTR_U16*)(((FMSTR_U32)(base))+(offset))) & ~(mask)) | (value))
#define FMSTR_SETBITFIELD32(base, offset, mask, value)   (*(volatile FMSTR_U32*)(((FMSTR_U32)(base))+(offset)) = (FMSTR_U32)(((*(volatile FMSTR_U32*)(((FMSTR_U32)(base))+(offset))) & ~(mask)) | (value)))

/****************************************************************************************
* SCI module constants
*****************************************************************************************/

/* SCI module registers */
#define FMSTR_SCIBR_OFFSET 0U
#define FMSTR_SCICR_OFFSET 1U
#define FMSTR_SCISR_OFFSET 3U
#define FMSTR_SCIDR_OFFSET 4U

/* SCI Control Register bits */
#define FMSTR_SCICR_LOOP      0x8000U
#define FMSTR_SCICR_SWAI      0x4000U
#define FMSTR_SCICR_RSRC      0x2000U
#define FMSTR_SCICR_M         0x1000U
#define FMSTR_SCICR_WAKE      0x0800U
#define FMSTR_SCICR_POL       0x0400U
#define FMSTR_SCICR_PE        0x0200U
#define FMSTR_SCICR_PT        0x0100U
#define FMSTR_SCICR_TEIE      0x0080U
#define FMSTR_SCICR_TIIE      0x0040U
#define FMSTR_SCICR_RFIE      0x0020U
#define FMSTR_SCICR_REIE      0x0010U
#define FMSTR_SCICR_TE        0x0008U
#define FMSTR_SCICR_RE        0x0004U
#define FMSTR_SCICR_RWU       0x0002U
#define FMSTR_SCICR_SBK       0x0001U

/* SCI Status registers bits */
#define FMSTR_SCISR_TDRE      0x8000U
#define FMSTR_SCISR_TIDLE     0x4000U
#define FMSTR_SCISR_RDRF      0x2000U
#define FMSTR_SCISR_RIDLE     0x1000U
#define FMSTR_SCISR_OR        0x0800U
#define FMSTR_SCISR_NF        0x0400U
#define FMSTR_SCISR_FE        0x0200U
#define FMSTR_SCISR_PF        0x0100U
#define FMSTR_SCISR_RAF       0x0001U

/*******************************************************************************************
* SCI access macros
*****************************************************************************************/

/* transmitter enable/disable */
/*lint -emacro(923,FMSTR_SCI_TE, FMSTR_SCI_TD) : casting long to pointer */
#define FMSTR_SCI_TE() FMSTR_SETBIT(FMSTR_SCI_BASE, FMSTR_SCICR_OFFSET, FMSTR_SCICR_TE)
#define FMSTR_SCI_TD() FMSTR_CLRBIT(FMSTR_SCI_BASE, FMSTR_SCICR_OFFSET, FMSTR_SCICR_TE)

/* receiver enable/disable */
/*lint -emacro(923,FMSTR_SCI_RE, FMSTR_SCI_RD) : casting long to pointer */
#define FMSTR_SCI_RE() FMSTR_SETBIT(FMSTR_SCI_BASE, FMSTR_SCICR_OFFSET, FMSTR_SCICR_RE)
#define FMSTR_SCI_RD() FMSTR_CLRBIT(FMSTR_SCI_BASE, FMSTR_SCICR_OFFSET, FMSTR_SCICR_RE)

#define FMSTR_SCI_TE_RE() FMSTR_SETBIT(FMSTR_SCI_BASE, FMSTR_SCICR_OFFSET, FMSTR_SCICR_RE | FMSTR_SCICR_TE)

/* Transmitter-empty interrupt enable/disable : casting long to pointer */
/*lint -emacro(923,FMSTR_SCI_ETXI, FMSTR_SCI_DTXI) */
#define FMSTR_SCI_ETXI() FMSTR_SETBIT(FMSTR_SCI_BASE, FMSTR_SCICR_OFFSET, FMSTR_SCICR_TEIE)
#define FMSTR_SCI_DTXI() FMSTR_CLRBIT(FMSTR_SCI_BASE, FMSTR_SCICR_OFFSET, FMSTR_SCICR_TEIE)

/* Receiver-full interrupt enable/disable : casting long to pointer */
/*lint -emacro(923,FMSTR_SCI_ERXI, FMSTR_SCI_DRXI) */
#define FMSTR_SCI_ERXI() FMSTR_SETBIT(FMSTR_SCI_BASE, FMSTR_SCICR_OFFSET, FMSTR_SCICR_RFIE)
#define FMSTR_SCI_DRXI() FMSTR_CLRBIT(FMSTR_SCI_BASE, FMSTR_SCICR_OFFSET, FMSTR_SCICR_RFIE)

/* Tranmsit character */
/*lint -emacro(923,FMSTR_SCI_PUTCHAR) : casting long to pointer */
#define FMSTR_SCI_PUTCHAR(ch) FMSTR_SETREG(FMSTR_SCI_BASE, FMSTR_SCIDR_OFFSET, ch)

/* Get received character */
/*lint -emacro(923,FMSTR_SCI_GETCHAR) : casting long to pointer */
#define FMSTR_SCI_GETCHAR() FMSTR_GETREG(FMSTR_SCI_BASE, FMSTR_SCIDR_OFFSET)

/* read status register */
/*lint -emacro(923,FMSTR_SCI_GETSR) : casting long to pointer */
#define FMSTR_SCI_GETSR()   FMSTR_GETREG(FMSTR_SCI_BASE, FMSTR_SCISR_OFFSET)

/* read & clear status register (clears error status bits only) */
/*lint -emacro(923,FMSTR_SCI_RDCLRSR) : casting long to pointer */
#define FMSTR_SCI_RDCLRSR() FMSTR_SETREG(FMSTR_SCI_BASE, FMSTR_SCISR_OFFSET, FMSTR_GETREG(FMSTR_SCI_BASE, FMSTR_SCISR_OFFSET))

/*******************************************************************************************
* JTAG access macros
*******************************************************************************************/

#define FMSTR_JTAG_OTXRXSR_OFFSET 0xfdU
#define FMSTR_JTAG_ORX_OFFSET     0xfeU
#define FMSTR_JTAG_OTX_OFFSET     0xfeU
#define FMSTR_JTAG_ORX1_OFFSET    0xffU
#define FMSTR_JTAG_OTX1_OFFSET    0xffU

/* OTXRXSR register */
#define FMSTR_JTAG_OTXRXSR_RDF 0x01U
#define FMSTR_JTAG_OTXRXSR_TDF 0x02U
#define FMSTR_JTAG_OTXRXSR_RIE 0x04U
#define FMSTR_JTAG_OTXRXSR_TIE 0x08U

#define _CHECK \
    if(FMSTR_GETREG(FMSTR_JTAG_BASE, FMSTR_JTAG_OTXRXSR_OFFSET) & FMSTR_JTAG_OTXRXSR_RIE) \
        ioctl(GPIO_A, GPIO_SET_PIN, BIT_1); \
    else \
        ioctl(GPIO_A, GPIO_CLEAR_PIN, BIT_1)

/* enable/disable JTAG Rx-full interrupt */
/*lint -emacro(923,FMSTR_JTAG_ERXI,FMSTR_JTAG_DRXI) : casting long to pointer */
#define FMSTR_JTAG_ERXI() FMSTR_SETBIT(FMSTR_JTAG_BASE, FMSTR_JTAG_OTXRXSR_OFFSET, FMSTR_JTAG_OTXRXSR_RIE)
#define FMSTR_JTAG_DRXI() FMSTR_CLRBIT(FMSTR_JTAG_BASE, FMSTR_JTAG_OTXRXSR_OFFSET, FMSTR_JTAG_OTXRXSR_RIE)

/* enable/disable JTAG Tx-empty interrupt */
/*lint -emacro(923,FMSTR_JTAG_ETXI,FMSTR_JTAG_DTXI) : casting long to pointer */
#define FMSTR_JTAG_ETXI() FMSTR_SETBIT(FMSTR_JTAG_BASE, FMSTR_JTAG_OTXRXSR_OFFSET, FMSTR_JTAG_OTXRXSR_TIE)
#define FMSTR_JTAG_DTXI() FMSTR_CLRBIT(FMSTR_JTAG_BASE, FMSTR_JTAG_OTXRXSR_OFFSET, FMSTR_JTAG_OTXRXSR_TIE)

/* this macro verifies if the RIE bit is set (note that RIE is held low by HW until JTAG init) */
/*lint -emacro(923,FMSTR_JTAG_ERXI_CHECK,FMSTR_JTAG_ETXI_CHECK) : casting long to pointer */
#define FMSTR_JTAG_ERXI_CHECK() (FMSTR_GETREG(FMSTR_JTAG_BASE, FMSTR_JTAG_OTXRXSR_OFFSET) & FMSTR_JTAG_OTXRXSR_RIE)
#define FMSTR_JTAG_ETXI_CHECK() (FMSTR_GETREG(FMSTR_JTAG_BASE, FMSTR_JTAG_OTXRXSR_OFFSET) & FMSTR_JTAG_OTXRXSR_TIE)

/* get JTAG status register */
/*lint -emacro(923,FMSTR_JTAG_GETSR) : casting long to pointer */
#define FMSTR_JTAG_GETSR() FMSTR_GETREG(FMSTR_JTAG_BASE, FMSTR_JTAG_OTXRXSR_OFFSET)

/* Tranmsit JTAG word */
/*lint -emacro(923,FMSTR_JTAG_PUTDWORD) : casting long to pointer */
#define FMSTR_JTAG_PUTDWORD(ch) FMSTR_SETREG32(FMSTR_JTAG_BASE, FMSTR_JTAG_OTX_OFFSET, ch)

/* Get received word */
/*lint -emacro(923,FMSTR_JTAG_GETDWORD) : casting long to pointer */
#define FMSTR_JTAG_GETDWORD() FMSTR_GETREG32(FMSTR_JTAG_BASE, FMSTR_JTAG_ORX_OFFSET)

/* Get upper 16bit of received word */
/*lint -emacro(923,FMSTR_JTAG_GETWORD) : casting long to pointer */
#define FMSTR_JTAG_GETWORD() FMSTR_GETREG(FMSTR_JTAG_BASE, FMSTR_JTAG_ORX1_OFFSET)

/****************************************************************************************
* MSCAN module constants
*****************************************************************************************/

/* MSCAN module registers */
#define FMSTR_MSCANRFLG_OFFSET  0x04
#define FMSTR_MSCANRIER_OFFSET  0x05
#define FMSTR_MSCANTFLG_OFFSET  0x06
#define FMSTR_MSCANTIER_OFFSET  0x07
#define FMSTR_MSCANTBSEL_OFFSET 0x0a
#define FMSTR_MSCANRXFG_OFFSET  0x20
#define FMSTR_MSCANTXFG_OFFSET  0x30

/* MSCAN MB registers offsets (must also add CANxxFG_OFFSET) */
#define FMSTR_MSCMBIDR0  0
#define FMSTR_MSCMBIDR1  1
#define FMSTR_MSCMBIDR2  2
#define FMSTR_MSCMBIDR3  3
#define FMSTR_MSCMBDSR0  4
#define FMSTR_MSCMBDSR1  5
#define FMSTR_MSCMBDSR2  6
#define FMSTR_MSCMBDSR3  7
#define FMSTR_MSCMBDSR4  8
#define FMSTR_MSCMBDSR5  9
#define FMSTR_MSCMBDSR6 10
#define FMSTR_MSCMBDSR7 11
#define FMSTR_MSCMBDLR  12
#define FMSTR_MSCMBTBPR 13
#define FMSTR_MSCMBTSRH 14
#define FMSTR_MSCMBTSRL 15

/* MSCAN CANRFLG */
#define FMSTR_MSCANRFLG_RXF     0x01

/* MSCAN CANRIER */
#define FMSTR_MSCANRFLG_RXFIE   0x01

/* MSCAN CANTFLG, TIER, TARQ, TBSEL */
#define FMSTR_MSCANTBUF_TX2     0x04
#define FMSTR_MSCANTBUF_TX1     0x02
#define FMSTR_MSCANTBUF_TX0     0x01
#define FMSTR_MSCANTBUF_ALL     0x07

/* MSCAN CANIDRx */
#define FMSTR_MSCANIDR1_IDE     0x08
#define FMSTR_MSCANIDR1_STD_RTR 0x10
#define FMSTR_MSCANIDR3_EXT_RTR 0x08

/* MSCAN: enable/disable CAN RX/TX interrupts */
#define FMSTR_MSCAN_ETXI() FMSTR_SETBIT(FMSTR_CAN_BASE, FMSTR_MSCANTIER_OFFSET, FMSTR_MSCANTBUF_ALL)
#define FMSTR_MSCAN_DTXI() FMSTR_CLRBIT(FMSTR_CAN_BASE, FMSTR_MSCANTIER_OFFSET, FMSTR_MSCANTBUF_ALL)
#define FMSTR_MSCAN_ERXI() FMSTR_SETBIT(FMSTR_CAN_BASE, FMSTR_MSCANRIER_OFFSET, FMSTR_MSCANRFLG_RXFIE)
#define FMSTR_MSCAN_DRXI() FMSTR_CLRBIT(FMSTR_CAN_BASE, FMSTR_MSCANRIER_OFFSET, FMSTR_MSCANRFLG_RXFIE)

/* MSCAN: read RX status register */
#define FMSTR_MSCAN_TEST_RXFLG()   FMSTR_TSTBIT(FMSTR_CAN_BASE, FMSTR_MSCANRFLG_OFFSET, FMSTR_MSCANRFLG_RXF)
#define FMSTR_MSCAN_CLEAR_RXFLG()  FMSTR_SETREG(FMSTR_CAN_BASE, FMSTR_MSCANRFLG_OFFSET, FMSTR_MSCANRFLG_RXF)

/* MSCAN: read TX status register */
#define FMSTR_MSCAN_TEST_TXFLG()   FMSTR_TSTBIT(FMSTR_CAN_BASE, FMSTR_MSCANTFLG_OFFSET, FMSTR_MSCANTBUF_ALL)

/* MSCAN: MB status register is not implemented */
#define FMSTR_MSCAN_GET_MBSTATUS()

/* MSCAN: id to idr translation */
#define FMSTR_MSCAN_MAKEIDR0(id) ((FMSTR_U8)(((id)&FMSTR_CAN_EXTID) ? (id)>>21 : (id)>>3))
#define FMSTR_MSCAN_MAKEIDR1(id) ((FMSTR_U8)(((id)&FMSTR_CAN_EXTID) ? ((((id)>>13) & 0xe0U) | 0x18 | (((id)>>15) & 0x07U)) : (((id)<<5)&0xe0)))
#define FMSTR_MSCAN_MAKEIDR2(id) ((FMSTR_U8)(((id)&FMSTR_CAN_EXTID) ? (id)>>7 : 0))
#define FMSTR_MSCAN_MAKEIDR3(id) ((FMSTR_U8)(((id)&FMSTR_CAN_EXTID) ? (((id)<<1) & 0xfeU) : 0))

/* MSCAN: CAN transmission */
typedef struct
{
    FMSTR_U8 nTxBufSel;
    FMSTR_U8 nDataIx;
} FMSTR_MSCAN_TCTX;

/* MSCAN: CAN transmission, configuring the buffer before each transmission */
#define FMSTR_MSCAN_TCFG(pctx) \
    FMSTR_MACROCODE_BEGIN() \
        FMSTR_SETREG(FMSTR_CAN_BASE, FMSTR_MSCANTBSEL_OFFSET, FMSTR_GETREG(FMSTR_CAN_BASE, FMSTR_MSCANTFLG_OFFSET)); \
        (pctx)->nTxBufSel = (FMSTR_U8) FMSTR_GETREG(FMSTR_CAN_BASE, FMSTR_MSCANTBSEL_OFFSET); \
        (pctx)->nDataIx = 0; \
    FMSTR_MACROCODE_END()

/* MSCAN: CAN transmission, preparing the buffer before each transmission */
#define FMSTR_MSCAN_TID(pctx, idr0, idr1, idr2, idr3) \
    FMSTR_MACROCODE_BEGIN() \
        FMSTR_SETREG(FMSTR_CAN_BASE, FMSTR_MSCANTXFG_OFFSET+FMSTR_MSCMBIDR0, idr0); \
        FMSTR_SETREG(FMSTR_CAN_BASE, FMSTR_MSCANTXFG_OFFSET+FMSTR_MSCMBIDR1, idr1); \
        FMSTR_SETREG(FMSTR_CAN_BASE, FMSTR_MSCANTXFG_OFFSET+FMSTR_MSCMBIDR2, idr2); \
        FMSTR_SETREG(FMSTR_CAN_BASE, FMSTR_MSCANTXFG_OFFSET+FMSTR_MSCMBIDR3, idr3); \
    FMSTR_MACROCODE_END()

/* CAN transmission, put one data byte into buffer */
#define FMSTR_MSCAN_TLEN(pctx, len) \
    FMSTR_SETREG(FMSTR_CAN_BASE, FMSTR_MSCANTXFG_OFFSET+FMSTR_MSCMBDLR, len)

/* CAN transmission, put one data byte into buffer */
#define FMSTR_MSCAN_PUTBYTE(pctx, dataByte) \
    FMSTR_MACROCODE_BEGIN() \
        FMSTR_SETREG(FMSTR_CAN_BASE, FMSTR_MSCANTXFG_OFFSET+FMSTR_MSCMBDSR0+(pctx)->nDataIx, dataByte); \
        (pctx)->nDataIx++; \
    FMSTR_MACROCODE_END()

/* CAN transmission, set transmit priority */
#define FMSTR_MSCAN_TPRI(pctx, txPri) \
        FMSTR_SETREG(FMSTR_CAN_BASE, FMSTR_MSCANTXFG_OFFSET+FMSTR_MSCMBTBPR, txPri)
    
/* CAN transmission, final firing of the buffer */
#define FMSTR_MSCAN_TX(pctx) \
        FMSTR_SETREG(FMSTR_CAN_BASE, FMSTR_MSCANTFLG_OFFSET, (pctx)->nTxBufSel)

/* CAN reception */
typedef struct
{
    FMSTR_U8 nDataIx;
} FMSTR_MSCAN_RCTX;

/* CAN reception, configuring the buffer, just once at the initialization phase */
#define FMSTR_MSCAN_RINIT(idr0, idr1, idr2, idr3) /* nothing needed on msCAN */

/* CAN transmission, configuring the buffer, just once at the initialization phase */
#define FMSTR_MSCAN_TINIT(idr0, idr1, idr2, idr3) /* nothing needed on msCAN *

/* CAN reception, configuring the buffer for receiving (each time receiver is re-enabled) */
#define FMSTR_MSCAN_RCFG() /* nothing needed on msCAN */

/* CAN reception, lock frame */
#define FMSTR_MSCAN_RX(pctx) \
    (pctx)->nDataIx = 0;

/* CAN reception, test if received message ID matches the one given, TRUE if matching */
#define FMSTR_MSCAN_TEST_RIDR(pctx, idr0, idr1, idr2, idr3) \
    (((idr1) & FMSTR_MSCANIDR1_IDE) ? \
        /* ext id compare */ \
        (((idr0)==FMSTR_GETREG(FMSTR_CAN_BASE, FMSTR_MSCANRXFG_OFFSET+FMSTR_MSCMBIDR0)) && \
         ((idr1)==FMSTR_GETREG(FMSTR_CAN_BASE, FMSTR_MSCANRXFG_OFFSET+FMSTR_MSCMBIDR1)) && \
         ((idr2)==FMSTR_GETREG(FMSTR_CAN_BASE, FMSTR_MSCANRXFG_OFFSET+FMSTR_MSCMBIDR2)) && \
         ((idr3)==FMSTR_GETREG(FMSTR_CAN_BASE, FMSTR_MSCANRXFG_OFFSET+FMSTR_MSCMBIDR3))) : \
        /* std id compare */ \
        (((idr0)==FMSTR_GETREG(FMSTR_CAN_BASE, FMSTR_MSCANRXFG_OFFSET+FMSTR_MSCMBIDR0)) && \
         (0==(((idr1)^FMSTR_GETREG(FMSTR_CAN_BASE, FMSTR_MSCANRXFG_OFFSET+FMSTR_MSCMBIDR1))&0xf8U))))
        

/* CAN reception, get receviced frame length */
#define FMSTR_MSCAN_RLEN(pctx) \
    FMSTR_GETREG(FMSTR_CAN_BASE, FMSTR_MSCANRXFG_OFFSET+FMSTR_MSCMBDLR)

/* CAN reception, get one received byte */
#define FMSTR_MSCAN_GETBYTE(pctx) \
    (FMSTR_U8)FMSTR_GETREG(FMSTR_CAN_BASE, FMSTR_MSCANRXFG_OFFSET+FMSTR_MSCMBDSR0+(pctx)->nDataIx++)

/* CAN reception, unlock the buffer */
#define FMSTR_MSCAN_RFINISH(pctx)  /* nothing needed on msCAN */

/****************************************************************************************
* FCAN module constants
*****************************************************************************************/

/* FlexCAN module of new 56F84xxx platform*/
#if FMSTR_USE_FLEXCAN32

/* FCAN module registers offsets */
#define FMSTR_FCANTMR_OFFSET    (0x04)
#define FMSTR_FCANIMASK_OFFSET  (0x14)
#define FMSTR_FCANIFLAG_OFFSET  (0x18)
#define FMSTR_FCANRXFG_OFFSET   (0x40 + ((FMSTR_FLEXCAN_RXMB) * 0x08)) //*0x08 because this is WORD pointer
#define FMSTR_FCANTXFG_OFFSET   (0x40 + ((FMSTR_FLEXCAN_TXMB) * 0x08))

/* FCAN MB registers offsets (must also add FCANxxFG_OFFSET) */
#define FMSTR_FCMBCSR   0x00
#define FMSTR_FCMBIDR   0x02
#define FMSTR_FCMBIDR23 0x03
#define FMSTR_FCMBDATA0 0x04
#define FMSTR_FCMBDATA1 0x06

/* FCAN32 module MB CODEs */
#define FMSTR_FCANMB_CODE_MASK      0x0f00UL   /* defines mask of codes */
#define FMSTR_FCMBCSR_CODE_SHIFT     8

#define FMSTR_FCANMB_CRXVOID        (0x0 << FMSTR_FCMBCSR_CODE_SHIFT)   /* buffer void after received data read-out */
#define FMSTR_FCANMB_CRXEMPTY       (0x4 << FMSTR_FCMBCSR_CODE_SHIFT)   /* active and empty */

#define FMSTR_FCANMB_CTXTRANS_ONCE  (0xc << FMSTR_FCMBCSR_CODE_SHIFT)   /* Initialise transmitting data from buffer */
#define FMSTR_FCANMB_CTXREADY       (0x8 << FMSTR_FCMBCSR_CODE_SHIFT)    /* Message buffer not ready for transmit */

#define FMSTR_FCMBCSR_IDE            0x0020UL
#define FMSTR_FCMBCSR_DLC_MASK       0x000FUL
#define FMSTR_FCMBCSR_DLC_SHIFT      0

/* FCAN ID flags */
#define FMSTR_FCANID0_EXT_FLG        0x80

/* FCAN: enable/disable CAN RX/TX interrupts */
#define FMSTR_FCAN_ETXI() FMSTR_SETBIT32(FMSTR_CAN_BASE, FMSTR_FCANIMASK_OFFSET, (1<<(FMSTR_FLEXCAN_TXMB)))
#define FMSTR_FCAN_DTXI() FMSTR_CLRBIT32(FMSTR_CAN_BASE, FMSTR_FCANIMASK_OFFSET, (1<<(FMSTR_FLEXCAN_TXMB)))
#define FMSTR_FCAN_ERXI() FMSTR_SETBIT32(FMSTR_CAN_BASE, FMSTR_FCANIMASK_OFFSET, (1<<(FMSTR_FLEXCAN_RXMB)))
#define FMSTR_FCAN_DRXI() FMSTR_CLRBIT32(FMSTR_CAN_BASE, FMSTR_FCANIMASK_OFFSET, (1<<(FMSTR_FLEXCAN_RXMB)))

/* FCAN: read RX status register */
#define FMSTR_FCAN_TEST_RXFLG()   FMSTR_TSTBIT32(FMSTR_CAN_BASE, FMSTR_FCANIFLAG_OFFSET, (1<<(FMSTR_FLEXCAN_RXMB)))
#define FMSTR_FCAN_CLEAR_RXFLG()  FMSTR_SETREG32(FMSTR_CAN_BASE, FMSTR_FCANIFLAG_OFFSET, (1<<(FMSTR_FLEXCAN_RXMB)))

/* FCAN: read TX status register */
#define FMSTR_FCAN_TEST_TXFLG()   FMSTR_TSTBIT32(FMSTR_CAN_BASE, FMSTR_FCANIFLAG_OFFSET, (1<<(FMSTR_FLEXCAN_TXMB)))

/* FCAN: read TX MB status register */
#define FMSTR_FCAN_GET_MBSTATUS() (FMSTR_GETREG(FMSTR_CAN_BASE, FMSTR_FCANTXFG_OFFSET + FMSTR_FCMBCSR + 1)&FMSTR_FCANMB_CODE_MASK)

/* FCAN: id to idr translation */
#define FMSTR_FCAN_MAKEIDR0(id) ((FMSTR_U8)( ((id)&FMSTR_CAN_EXTID) ? ((((id)>>24)&0x1f) | FMSTR_FCANID0_EXT_FLG) : (((id)>>6)&0x1f) ))
#define FMSTR_FCAN_MAKEIDR1(id) ( (FMSTR_U8) (((id)&FMSTR_CAN_EXTID) ? (((id)>>16) & 0xff) : (((id)<<2) & 0xfc) ) )
#define FMSTR_FCAN_MAKEIDR2(id) ( (FMSTR_U8) (((id)&FMSTR_CAN_EXTID) ? (((id)>>8)  & 0xff) : 0) )
#define FMSTR_FCAN_MAKEIDR3(id) ( (FMSTR_U8) (((id)&FMSTR_CAN_EXTID) ? ( (id)      & 0xff) : 0) )

/* FCAN reception, configuring the buffer, just once at the initialisation phase */
#define FMSTR_FCAN_RINIT(idr0, idr1, idr2, idr3) \
    FMSTR_MACROCODE_BEGIN() \
        FMSTR_SETREG(FMSTR_CAN_BASE, FMSTR_FCANRXFG_OFFSET + FMSTR_FCMBCSR + 1, (FMSTR_FCANMB_CRXVOID) | (((idr0) & 0x80) ? FMSTR_FCMBCSR_IDE : 0));\
        FMSTR_SETREG32(FMSTR_CAN_BASE, FMSTR_FCANRXFG_OFFSET + FMSTR_FCMBIDR, ((((FMSTR_U32)((idr0)&0x1f)))<<24) | (((FMSTR_U32)(idr1))<<16) | (((FMSTR_U32)(idr2))<<8) | (((FMSTR_U32)(idr3))));\
    FMSTR_MACROCODE_END()

/* CAN transmission, configuring the buffer, just once at the initialisation phase */
#define FMSTR_FCAN_TINIT(idr0, idr1, idr2, idr3) \
    FMSTR_MACROCODE_BEGIN() \
        FMSTR_SETREG(FMSTR_CAN_BASE, FMSTR_FCANTXFG_OFFSET + FMSTR_FCMBCSR + 1, (FMSTR_FCANMB_CTXREADY) | (((idr0) & 0x80) ? FMSTR_FCMBCSR_IDE : 0));\
        FMSTR_SETREG32(FMSTR_CAN_BASE, FMSTR_FCANTXFG_OFFSET + FMSTR_FCMBIDR, ((((FMSTR_U32)((idr0)&0x1f)))<<24) | (((FMSTR_U32)(idr1))<<16) | (((FMSTR_U32)(idr2))<<8) | (((FMSTR_U32)(idr3))));\
    FMSTR_MACROCODE_END()

/* CAN reception, configuring the buffer for receiving (each time receiver is re-enabled) */
#define FMSTR_FCAN_RCFG() \
        FMSTR_SETBITFIELD(FMSTR_CAN_BASE, FMSTR_FCANRXFG_OFFSET + FMSTR_FCMBCSR + 1, FMSTR_FCANMB_CODE_MASK, FMSTR_FCANMB_CRXEMPTY)

/* FCAN: CAN transmission */
typedef struct
{
    FMSTR_U8 nDataIx;
} FMSTR_FCAN_TCTX;

/* FCAN transmission, put one data byte into buffer */
#define FMSTR_FCAN_TLEN(pctx, len) \
    FMSTR_SETBITFIELD(FMSTR_CAN_BASE, FMSTR_FCANTXFG_OFFSET+FMSTR_FCMBCSR+1, FMSTR_FCMBCSR_DLC_MASK, ((len) & 0xf) << FMSTR_FCMBCSR_DLC_SHIFT)

/* FCAN transmission, put one data byte into buffer */
#define FMSTR_FCAN_PUTBYTE(pctx, dataByte) \
    FMSTR_MACROCODE_BEGIN() \
        unsigned bix = 24 - (((pctx)->nDataIx)&0x3)*8;\
        FMSTR_SETBITFIELD32(FMSTR_CAN_BASE, FMSTR_FCANTXFG_OFFSET + FMSTR_FCMBDATA0 + ((((pctx)->nDataIx)>>1)&0x0e), (FMSTR_U32)(((FMSTR_U32)0xff) << bix), (FMSTR_U32)(((FMSTR_U32)(dataByte)) << bix)); \
        (pctx)->nDataIx++;\
    FMSTR_MACROCODE_END()

/* FCAN: CAN transmission, configuring the buffer before each transmission */
#define FMSTR_FCAN_TCFG(pctx) \
    FMSTR_MACROCODE_BEGIN() \
        (pctx)->nDataIx = 0; \
    FMSTR_MACROCODE_END()

/* FCAN: CAN transmission, preparing the buffer before each transmission */
#define FMSTR_FCAN_TID(pctx, idr0, idr1, idr2, idr3) /* void, set already during TINIT */ 

/* FCAN transmission, set transmit priority */
#define FMSTR_FCAN_TPRI(pctx, txPri) /* void, not using TX priorities */

/* CAN transmission, final firing of the buffer */
#define FMSTR_FCAN_TX(pctx) \
    FMSTR_SETBITFIELD(FMSTR_CAN_BASE, FMSTR_FCANTXFG_OFFSET+FMSTR_FCMBCSR+1, FMSTR_FCANMB_CODE_MASK, FMSTR_FCANMB_CTXTRANS_ONCE)

/* FCAN reception */
typedef struct
{
    FMSTR_U8 nDataIx;
} FMSTR_FCAN_RCTX;

/* FCAN reception, lock frame */
#define FMSTR_FCAN_RX(pctx) \
    FMSTR_MACROCODE_BEGIN() \
        volatile FMSTR_U16 dummy = FMSTR_GETREG(FMSTR_CAN_BASE, FMSTR_FCANRXFG_OFFSET+FMSTR_FCMBCSR+1); \
        (pctx)->nDataIx = 0;\
    FMSTR_MACROCODE_END()

/* FCAN reception, test if received message ID matches the one given, TRUE if matching */
#define FMSTR_FCAN_TEST_RIDR(pctx, idr0, idr1, idr2, idr3) \
    ( (idr0 & FMSTR_FCANID0_EXT_FLG) ? \
        /* ext id compare */ \
        ( ((((idr0)<<24) | ((idr1)<<16) | ((idr2)<<8) | (idr3))&0x1f000000)==((FMSTR_GETREG32(FMSTR_CAN_BASE, FMSTR_FCANRXFG_OFFSET+FMSTR_FCMBIDR))&0x1f000000) ) : \
        /* std id compare */ \
        ( (((idr0)<<8) | (idr1))==((FMSTR_GETREG(FMSTR_CAN_BASE, FMSTR_FCANRXFG_OFFSET+FMSTR_FCMBIDR23))&0x1ffc) ) )

/* FCAN reception, get receviced frame length */
#define FMSTR_FCAN_RLEN(pctx) \
    (FMSTR_GETREG(FMSTR_CAN_BASE, FMSTR_FCANRXFG_OFFSET+FMSTR_FCMBCSR+1) & 0x0f)

/* FCAN reception, get one received byte */
#define FMSTR_FCAN_GETBYTE(pctx) \
        ( (FMSTR_U8)((FMSTR_U32)( (FMSTR_U32)FMSTR_GETREG32(FMSTR_CAN_BASE, FMSTR_FCANRXFG_OFFSET+FMSTR_FCMBDATA0 + ((((pctx)->nDataIx)>>1)&0x0e))) >> ( ((3-((pctx)->nDataIx))&0x03)*8 )) );\
        (pctx)->nDataIx++

/* FCAN reception, unlock the buffer */
#define FMSTR_FCAN_RFINISH(pctx)  \
    FMSTR_SETBIT(FMSTR_CAN_BASE, FMSTR_FCANTMR_OFFSET , 0)

#else /* FMSTR_CANHW_FLEXCAN32 - older FlexCAN module of 56F83xx */

/* FCAN module MB CODEs */
#define FMSTR_FCANMB_CODE_MASK      0xF0   /* defines mask of codes */

#define FMSTR_FCANMB_CRXVOID        0x00   /* buffer void after received data read-out */
#define FMSTR_FCANMB_CRXEMPTY       0x40   /* active and empty */

#define FMSTR_FCANMB_CTXTRANS_ONCE  0xC0   /* Initialize transmiting data from buffer */
#define FMSTR_FCANMB_CTXREADY       0x80    /* Message buffer not ready for transmit */


/* FCAN module registers offsets */
#define FMSTR_FCANTMR_OFFSET    0x05
#define FMSTR_FCANIER_OFFSET    0x11
#define FMSTR_FCANIFR_OFFSET    0x12
#define FMSTR_FCANRXFG_OFFSET  (0x40 + ((FMSTR_FLEXCAN_RXMB) * 0x08))
#define FMSTR_FCANTXFG_OFFSET  (0x40 + ((FMSTR_FLEXCAN_TXMB) * 0x08))

/* FCAN MB registers offsets (must also add FCANxxFG_OFFSET) */
#define FMSTR_FCMBCSR   0
#define FMSTR_FCMBIDRH  1
#define FMSTR_FCMBIDRL  2
#define FMSTR_FCMBDSR01 3
#define FMSTR_FCMBDSR23 4
#define FMSTR_FCMBDSR45 5
#define FMSTR_FCMBDSR67 6

/* FCAN CANIDRx */
#define FMSTR_FCANIDRH_IDE     0x08
#define FMSTR_FCANIDRH_SRR     0x10
#define FMSTR_FCANIDRH_STD_RTR 0x10
#define FMSTR_FCANIDRL_EXT_RTR 0x01

/* FCAN: enable/disable CAN RX/TX interrupts */
#define FMSTR_FCAN_ETXI() FMSTR_SETBIT(FMSTR_CAN_BASE, FMSTR_FCANIER_OFFSET, (1<<(FMSTR_FLEXCAN_TXMB)))
#define FMSTR_FCAN_DTXI() FMSTR_CLRBIT(FMSTR_CAN_BASE, FMSTR_FCANIER_OFFSET, (1<<(FMSTR_FLEXCAN_TXMB)))
#define FMSTR_FCAN_ERXI() FMSTR_SETBIT(FMSTR_CAN_BASE, FMSTR_FCANIER_OFFSET, (1<<(FMSTR_FLEXCAN_RXMB)))
#define FMSTR_FCAN_DRXI() FMSTR_CLRBIT(FMSTR_CAN_BASE, FMSTR_FCANIER_OFFSET, (1<<(FMSTR_FLEXCAN_RXMB)))

/* FCAN: read RX status register */
#define FMSTR_FCAN_TEST_RXFLG()   FMSTR_TSTBIT(FMSTR_CAN_BASE, FMSTR_FCANIFR_OFFSET, (1<<(FMSTR_FLEXCAN_RXMB)))
#define FMSTR_FCAN_CLEAR_RXFLG()  FMSTR_SETREG(FMSTR_CAN_BASE, FMSTR_FCANIFR_OFFSET, (1<<(FMSTR_FLEXCAN_RXMB)))

/* FCAN: read TX status register */
#define FMSTR_FCAN_TEST_TXFLG()   FMSTR_TSTBIT(FMSTR_CAN_BASE, FMSTR_FCANIFR_OFFSET, (1<<(FMSTR_FLEXCAN_TXMB)))

/* FCAN: read TX MB status register */
#define FMSTR_FCAN_GET_MBSTATUS() (FMSTR_GETREG(FMSTR_CAN_BASE, FMSTR_FCANTXFG_OFFSET+FMSTR_FCMBCSR)&FMSTR_FCANMB_CODE_MASK)

/* FSCAN: id to idr translation */
#define FMSTR_FCAN_MAKEIDR0(id) ( (FMSTR_U8) (((id)&FMSTR_CAN_EXTID) ? ((id)>>21) : (id)>>3) )
#define FMSTR_FCAN_MAKEIDR1(id) ( (FMSTR_U8) (((id)&FMSTR_CAN_EXTID) ? (((id)>>15) & 0x07) | FMSTR_FCANIDRH_IDE | FMSTR_FCANIDRH_SRR | (((id)>>13) & 0xe0) : (id)<<5) )
#define FMSTR_FCAN_MAKEIDR2(id) ( (FMSTR_U8) (((id)&FMSTR_CAN_EXTID) ? ((id)>>7) : 0) )
#define FMSTR_FCAN_MAKEIDR3(id) ( (FMSTR_U8) (((id)&FMSTR_CAN_EXTID) ? ((id)<<1) : 0) )

/* CAN reception, configuring the buffer, just once at the initialization phase */
#define FMSTR_FCAN_RINIT(idr0, idr1, idr2, idr3) \
    FMSTR_MACROCODE_BEGIN() \
        FMSTR_SETREG(FMSTR_CAN_BASE, FMSTR_FCANRXFG_OFFSET + FMSTR_FCMBCSR, FMSTR_FCANMB_CRXVOID);\
        FMSTR_SETREG(FMSTR_CAN_BASE, FMSTR_FCANRXFG_OFFSET + FMSTR_FCMBIDRH, (FMSTR_U16) ((idr0)<<8) | (idr1));\
        FMSTR_SETREG(FMSTR_CAN_BASE, FMSTR_FCANRXFG_OFFSET + FMSTR_FCMBIDRL, (FMSTR_U16) ((idr2)<<8) | (idr3));\
    FMSTR_MACROCODE_END()

/* CAN transmission, configuring the buffer, just once at the initialization phase */
#define FMSTR_FCAN_TINIT(idr0, idr1, idr2, idr3) FMSTR_SETREG(FMSTR_CAN_BASE, FMSTR_FCANTXFG_OFFSET+FMSTR_FCMBCSR, FMSTR_FCANMB_CTXREADY);

/* CAN reception, configuring the buffer for receiving (each time receiver is re-enabled) */
#define FMSTR_FCAN_RCFG() \
    FMSTR_SETREG(FMSTR_CAN_BASE, FMSTR_FCANRXFG_OFFSET + FMSTR_FCMBCSR, FMSTR_FCANMB_CRXEMPTY)
    

/* FCAN: CAN transmission */
typedef struct
{
    FMSTR_U8 nDataIx;
} FMSTR_FCAN_TCTX;


/* FCAN transmission, put one data byte into buffer */
#define FMSTR_FCAN_TLEN(pctx, len) \
    FMSTR_SETREG(FMSTR_CAN_BASE, FMSTR_FCANTXFG_OFFSET+FMSTR_FCMBCSR, \
        (FMSTR_GETREG(FMSTR_CAN_BASE, FMSTR_FCANTXFG_OFFSET+FMSTR_FCMBCSR) & 0x00f0) | (len & 0x000f))


/* FCAN transmission, put one data byte into buffer */
#define FMSTR_FCAN_PUTBYTE(pctx, dataByte) \
    FMSTR_MACROCODE_BEGIN() \
        FMSTR_SETREG(FMSTR_CAN_BASE, (FMSTR_FCANTXFG_OFFSET + FMSTR_FCMBDSR01) + (((pctx)->nDataIx)>>1), \
        ((pctx)->nDataIx) & 0x01 ? (FMSTR_GETREG(FMSTR_CAN_BASE, (FMSTR_FCANTXFG_OFFSET + FMSTR_FCMBDSR01) + (((pctx)->nDataIx)>>1)) & 0xff00) | (dataByte & 0x00ff) : \
        (FMSTR_GETREG(FMSTR_CAN_BASE, (FMSTR_FCANTXFG_OFFSET + FMSTR_FCMBDSR01) + (((pctx)->nDataIx)>>1)) & 0x00ff) | (dataByte << 8) ); \
        (pctx)->nDataIx++; \
    FMSTR_MACROCODE_END()


/* FCAN: CAN transmission, configuring the buffer before each transmission */
#define FMSTR_FCAN_TCFG(pctx) \
    FMSTR_MACROCODE_BEGIN() \
        (pctx)->nDataIx = 0; \
    FMSTR_MACROCODE_END()

/* FCAN: CAN transmission, preparing the buffer before each transmission */
#define FMSTR_FCAN_TID(pctx, idr0, idr1, idr2, idr3) \
    FMSTR_MACROCODE_BEGIN() \
        FMSTR_SETREG(FMSTR_CAN_BASE, FMSTR_FCANTXFG_OFFSET+FMSTR_FCMBIDRH, (FMSTR_U16) ((idr0)<<8) | (idr1)); \
        FMSTR_SETREG(FMSTR_CAN_BASE, FMSTR_FCANTXFG_OFFSET+FMSTR_FCMBIDRL, (FMSTR_U16) ((idr2)<<8) | (idr3)); \
    FMSTR_MACROCODE_END()


/* FCAN transmission, set transmit priority */
#define FMSTR_FCAN_TPRI(pctx, txPri) /* in FCAN module is not implemented */

/* CAN transmission, final firing of the buffer */
#define FMSTR_FCAN_TX(pctx) \
        FMSTR_SETREG(FMSTR_CAN_BASE, FMSTR_FCANTXFG_OFFSET + FMSTR_FCMBCSR, \
        (FMSTR_FCANMB_CTXTRANS_ONCE & FMSTR_FCANMB_CODE_MASK) | ((FMSTR_GETREG(FMSTR_CAN_BASE, FMSTR_FCANTXFG_OFFSET+FMSTR_FCMBCSR) & 0x000f)))

/* FCAN reception */
typedef struct
{
    FMSTR_U8 nDataIx;
} FMSTR_FCAN_RCTX;

/* FCAN reception, lock frame */
#define FMSTR_FCAN_RX(pctx) \
    (pctx)->nDataIx = 0;



/* FCAN reception, test if received message ID matches the one given, TRUE if matching */
#define FMSTR_FCAN_TEST_RIDR(pctx, idr0, idr1, idr2, idr3) \
    (((idr1) & FMSTR_FCANIDRH_IDE) ? \
        /* ext id compare */ \
        ((((((idr0)<<8) | (idr1))& 0xffe3) ==(FMSTR_GETREG(FMSTR_CAN_BASE, FMSTR_FCANRXFG_OFFSET+FMSTR_FCMBIDRH) & 0xffe3)) && \
         ((((idr2)<<8) | (idr3))==(FMSTR_GETREG(FMSTR_CAN_BASE, FMSTR_FCANRXFG_OFFSET+FMSTR_FCMBIDRL)& 0xfffe))) : \
        /* std id compare */ \
        (((((idr0)<<8) | (idr1))==(FMSTR_GETREG(FMSTR_CAN_BASE, FMSTR_FCANRXFG_OFFSET+FMSTR_FCMBIDRH) & 0xffe0))))

/* FCAN reception, get receviced frame length */
#define FMSTR_FCAN_RLEN(pctx) \
    (FMSTR_GETREG(FMSTR_CAN_BASE, FMSTR_FCANRXFG_OFFSET+FMSTR_FCMBCSR) & 0x0f)

/* FCAN reception, get one received byte */
#define FMSTR_FCAN_GETBYTE(pctx) \
     ((FMSTR_U8)((pctx)->nDataIx&0x01 ? \
        ((FMSTR_U8) (FMSTR_GETREG(FMSTR_CAN_BASE, (FMSTR_FCANRXFG_OFFSET + FMSTR_FCMBDSR01) + (((pctx)->nDataIx)>>1)) )) : \
        ((FMSTR_U8) (FMSTR_GETREG(FMSTR_CAN_BASE, (FMSTR_FCANRXFG_OFFSET + FMSTR_FCMBDSR01) + (((pctx)->nDataIx)>>1))>>8)) ));\
        (pctx)->nDataIx++

/* FCAN reception, unlock the buffer */
#define FMSTR_FCAN_RFINISH(pctx)  \
    FMSTR_SETBIT(FMSTR_CAN_BASE, FMSTR_FCANTMR_OFFSET , 0)

#endif /* FMSTR_CANHW_FLEXCAN32 */

#endif /* __FREEMASTER_56F8xxx_H */
