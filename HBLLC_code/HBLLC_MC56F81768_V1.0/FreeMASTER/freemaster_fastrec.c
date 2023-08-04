/*******************************************************************************
*
* Copyright 2004-2014 Freescale Semiconductor, Inc.
* License: NXP LA_OPT_NXP_Software_License
*
* This software is owned or controlled by Freescale Semiconductor.
* Use of this software is governed by the Freescale FreeMASTER License
* distributed with this Material.
* See the LICENSE file distributed for more details.
* 
****************************************************************************//*!
*
* @brief  FreeMASTER 56F800E-hardware dependent fast-recorder routines
*
*******************************************************************************/

#include "freemaster.h"
#include "freemaster_private.h"
#include "freemaster_protocol.h"

#if (FMSTR_USE_RECORDER) && (!(FMSTR_DISABLE)) && (FMSTR_USE_FASTREC)

#include "freemaster_fastrec.h"

/* global variables */
FMSTR_ADDR pcm_dwFastRecWritePtr;
FMSTR_ADDR pcm_dwFastRecStartBuffPtr; /* recorder buffer address */
FMSTR_ADDR pcm_dwFastRecEndBuffPtr;

/* statically allocated recorder buffer (FMSTR_REC_OWNBUFF is FALSE) */
#if FMSTR_REC_FARBUFF
#pragma section fardata begin
#endif /* FMSTR_REC_FARBUFF */
FMSTR_U16 pcm_pFastRecBuffer[FMSTR_REC_BUFF_SIZE/2];
/* end of far memory section */
#if FMSTR_REC_FARBUFF
#pragma section fardata end
#endif /* FMSTR_REC_FARBUFF */

/**************************************************************************//*!
*
* @brief    Check if recorder configuration can be used in fast-recorder mode
*
******************************************************************************/

FMSTR_BOOL FMSTR_SetUpFastRec(void)
{
    FMSTR_INDEX i;
    
    /* check all recorder variables */
    for(i=0; i<pcm_nRecVarCount; i++)
    {
        /* only word-sized variables allowed */
        if(pcm_pRecVarSize[i] != 2)
            return 0;
        
        /* byte-pointer address must be word-aligned */
        if(pcm_pRecVarAddr[i] & 1)
            return 0;
        
        /* convert byte-pointer to word-pointer */
        pcm_pRecVarAddr[i] /= 2;
    }    

    pcm_dwFastRecEndBuffPtr = pcm_dwFastRecStartBuffPtr + pcm_wRecTotalSmps * pcm_nRecVarCount;

    /* our fast-trigger implementation requires post-trigger is not zero (zero would be 0xffff) */
#if FMSTR_REC_STATIC_POSTTRIG == 0
    if(pcm_wRecPostTrigger == 0)
        pcm_wRecPostTrigger = 1;
#endif
    return 1;
}

/**************************************************************************//*!
*
* @brief    Notification from base recorder driver: start recording
*
******************************************************************************/

void FMSTR_StartFastRec(void)
{
    pcm_dwFastRecWritePtr = pcm_dwFastRecStartBuffPtr;
}

/* now follows the recorder only routines, skip that if FastRecorder is used */
#if FMSTR_USE_FASTREC

/**************************************************************************//*!
*
* @brief    Check wether given memory region is inside the recorder buffer
*
* @param    dwAddr - address of the memory to be checked
* @param    wSize  - size of the memory to be checked
*
* @return   This function returns non-zero if user space is in recorder buffer
*
* This function is called as a pert of TSA-checking process when the PC host
* is requesting memory contents
*
******************************************************************************/

FMSTR_BOOL FMSTR_IsInRecBuffer(FMSTR_ADDR dwAddr, FMSTR_SIZE8 nSize)
{
    FMSTR_BOOL bRet = 0U;
    
    /* Convert Word to Byte address */
    FMSTR_ADDR pcm_dwFastRecStartBuffBytePtr = pcm_dwFastRecStartBuffPtr * 2;
    
    if(dwAddr >= pcm_dwFastRecStartBuffBytePtr)
    {
        bRet = (FMSTR_BOOL)((dwAddr + nSize) <= (pcm_dwFastRecStartBuffBytePtr + FMSTR_GetRecBuffSize()) ? FMSTR_TRUE : FMSTR_FALSE);    
    }
    
    return bRet;
}

/**************************************************************************//*!
*
* @brief    Get recorder memory size 
*
* @return   Recorder memory size in native sizeof units (=bytes on most platforms)
*
******************************************************************************/

FMSTR_SIZE FMSTR_GetRecBuffSize()
{
    return (FMSTR_SIZE) FMSTR_REC_BUFF_SIZE;
}

/**************************************************************************//*!
*
* @brief    Handling GETRECBUFF and GETRECBUFF_EX command
*
* @param    pMessageIO - original command (in) and response buffer (out) 
*
* @return   As all command handlers, the return value should be the buffer 
*           pointer where the response output finished (except checksum)
*
* This function returns recorder buffer information
*
******************************************************************************/

FMSTR_BPTR FMSTR_GetRecBuff(FMSTR_BPTR pMessageIO)
{
    FMSTR_ADDR a;    
    
    /* must be configured */
    if(!pcm_wRecFlags.flg.bIsConfigured)
    {
        return FMSTR_ConstToBuffer8(pMessageIO, FMSTR_STC_NOTINIT);
    }
    
    /* must be stopped */
    if(pcm_wRecFlags.flg.bIsRunning)
    {
        return FMSTR_ConstToBuffer8(pMessageIO, FMSTR_STC_SERVBUSY);
    }

    /* fill the return info */
    pMessageIO = FMSTR_ConstToBuffer8(pMessageIO, FMSTR_STS_OK);

    /* word-pointer offset divided by number of (word-wide-only) variables */
    pMessageIO = FMSTR_AddressToBuffer(pMessageIO, pcm_dwFastRecStartBuffPtr * 2);

    /* calculate index of the first sample */
    a = (pcm_dwFastRecWritePtr - pcm_dwFastRecStartBuffPtr);
    /* word-pointer offset divided by number of (word-wide-only) variables */
    return FMSTR_ValueToBuffer16(pMessageIO, (FMSTR_U16)(a / (FMSTR_U16)pcm_nRecVarCount));
}

#endif /* FMSTR_USE_FASTREC */

#endif /* FMSTR_USE_RECORDER && FMSTR_USE_FASTREC */


