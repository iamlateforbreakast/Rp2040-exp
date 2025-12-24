/* CanOpenBus.c - CanOpenBus body */

/***********************************************************************/
/**
 * @file
 * @brief CanOpenBus.c - implementation of CanOpenBus module.
 *
 * This modules implements a set of functions handling the low-level
 * specificities and constraints of HWSW CAN library and provides adequate
 * functional services for CANopen Manager.
 *
 ***********************************************************************/

/**
 * @addtogroup coMgr
 * @{
 * @par CanOpenBus module
 * The CanOpenBus module in the CANopen manager is in charge of all
 * exchanges with the CAN bus. It is built on Rover OBC's HWSW CAN Bus
 * driver and takes into account any timing and use constraints of the 
 * CAN Bus controller and its driver.
 *
 * Theses services perform the operations necessary to support the
 * CANopen Manager w.r.t. CAN message handling and exchanges with CAN Bus.
 * The services are dedicated to be used by CANopen Manager via its
 * automaton's "actions, and executed in auytomaton's context.
 *
 * @}
 */

/*---------- Standard libraries includes ------------------------------*/
#ifdef PORT
#include <rtems.h>
#include <leon.h>
#include <libc.h>
#endif

/*---------- FSW includes ---------------------------------------------*/
#include "errorLib.h"

/*---------- BIOS includes ---------------------------------------------*/

/*---------- Component includes ---------------------------------------*/
#include "coMgt/CanOpenBus.h"

/*---------- Local defines & macro ------------------------------------*/
/* mask for 16-byte address alignment */ 
#define ADDR_16BYTE_BOUNDARY (0xFFFFFFF0)

/* mask for 1-Kbyte address alignment */ 
#define ADDR_1KBYTE_BOUNDARY (0xFFFFFC00)

/* bit mask of 10->4 */
#define MSB_10_4_MSK (0x7F0)

/* bit mask of 10->0 */
#define MSB_10_0_MSK (0x7FF)

/* Build the CAN Message header for CAN Controller */
#define STD_MSG_HDR_BUILD(rtr,id) ((((rtr)&1)<<30)|(((id)&MSB_10_0_MSK)<<18)) 

/* extract fields from CAN Message Header */
#define STD_MSG_HDR_TO_ID(hdr) (((hdr)>>18)&MSB_10_0_MSK)
#define STD_MSG_HDR_TO_RTR(hdr) (((hdr)>>30)&1)


/* Bit Timing definition (c.f. Cole ASIC CAN_CONF description)
 * .BPR=0 --> (sysClk/1)
 * .RSJ = 1
 * .SCALER:
 *  - Sysclk 70 Mhz scaler=4: baud rate = (70 000 000/(1*(4+1))) / 14 --> 1000000 bit/s
 * RSJ / PS1 / PS2 as per OBC EICD.  ( EXM.RM.IRD.RSE.00003 )
 */

#define CAN_BIT_TIMING_BPR (0)
#define CAN_BIT_TIMING_RSJ (1)
#define CAN_BIT_TIMING_PS1 (10)
#define CAN_BIT_TIMING_PS2 (2)
#define CAN_BIT_TIMING_SCALER_70MHZ (4)

#define CAN_IF_TX_LIST_NONE (CAN_IF_TX_LIST_NUM+1)

/* COLE ASIC register's addresses */
#define CANA_STAT_REG_ADDR (0x80000804)
#define CANB_STAT_REG_ADDR (0x80000C04)

/* constant for an impossible status value */
#define UNSET_CAN_STAT_VALUE (0x12345678)

/*---------- Local types definitions ----------------------------------*/
/** CAN Bus interface definition */

typedef struct CancanBusIf
{
  CanBus_RestartIf restartIf;                   /**< @brief restartIf function */
  CanBus_SetBusConnState setBusConnState;       /**< @brief setBusConnState function */
  CanBus_SetBitTiming setBitTiming;             /**< @brief setBitTiming function */
  CanBus_SelectBus selectBus;                   /**< @brief selectBus function */

  CanBus_StartTx startTx;                       /**< @brief startTx function */
  CanBus_StopTx  stopTx;                        /**< @brief stopTx function */

  CanBus_StartRx startRx;                       /**< @brief startRx function */
  CanBus_GetRxMsg getRxMsg;                     /**< @brief getRxMsg function */
  CanBus_StopRx  stopRx;                        /**< @brief stopRx function */

  CanBus_GetTxSts getTxSts;                     /**< @brief getTxSts function */
  CanBus_GetRxSts getRxSts;                     /**< @brief getRxSts function */

} CancanBusIf;


/* CAN Bus controller's context */
typedef struct CanBusCtx
{ 
  /* global configuration parameter/status */
  Bool       initialised;
  HdswExtCan_If_T bus;
  HdswExtCan_NomRed_T chan;
  HdswExtCan_BusConnState_T connState;
  
  /* buffer list location for Tx messages */
  HdswExtCan_MsgTx_T *pTxList[CAN_IF_TX_LIST_NUM];
  
  /* buffer for Rx messages */
  HdswExtCan_MsgRx_T *pRxBuffer;
  
  /* transmission configuration/status */
  Bool txStarted;
 
  /* transmit status */
  HdswExtCan_TxSts_T txSts;
  HdswExtCan_StsDetails_T txDetails;
  
  /* transmission message number in a list */
  Uint txListMsgs[CAN_IF_TX_LIST_NUM];
 
  /* current transmission cycle, slot and message number in the list */
  Uint curTxCycle;
  Uint curTxSlot;
  Uint curTxList;
  
  /* transmitted message of the current list */
  Uint curTxMsgs;
  
  Uint totalTxList;   /* transmitted message list */
  Uint totalTxMsgs;   /* transmitted message */
  
  /* reception configuration/status */
  Bool rxStarted;
  
  /* parameters for the error of reception while transmission is busy */
  Bool rxWhileBusyTransmission;
  Uint rxWhileBusyTxElapsedItSlots;
  
  /* receive status */
  
  HdswExtCan_RxSts_T rxSts;
  HdswExtCan_StsDetails_T rxDetails;
  
  HdswExtCan_MsgRx_T rxMsg; /* latest extracted message */
  
  Uint curRxMsgs;     /* lastly received messages */
  Uint totalExtMsgs;  /* total extracted messages */

  Uint elapsedItSlots;  /* elapsed IT Slots */
  
  Bool mstAloneOnBus;
} CanBusCtx;

/*---------- Definition of variables exported by the module -----------*/

#ifdef COMGT_UT_SUPPORT
PUBLIC void CanSimu_RestartIf (const HdswExtCan_If_T If,
  /*@out@*/ HdswExtCan_Sts_T *const Sts);

PUBLIC void CanSimu_SetBusConnState (const HdswExtCan_If_T If,
  const HdswExtCan_BusConnState_T State,
  /*@out@*/  HdswExtCan_Sts_T  *const Sts);

PUBLIC void CanSimu_SetBitTiming (          const HdswExtCan_If_T If,
/*@out@*/       const HdswExtCan_BitTiming_T *const BitTiming,
/*@out@*/       HdswExtCan_Sts_T       *const Sts);

PUBLIC HdswExtCan_Sts_T CanSimu_SelectBus (const HdswExtCan_If_T If,
  const HdswExtCan_NomRed_T NomRed);

PUBLIC void CanSimu_StartTx (const HdswExtCan_If_T If,
  const HdswExtCan_MsgTx_T     *const Msgs,
  const HdswExtCan_NofTxMsgs_T        NofMsgs,
  /*@out@*/       HdswExtCan_Sts_T       *const Sts);

PUBLIC void CanSimu_StopTx (const HdswExtCan_If_T If,
  /*@out@*/ HdswExtCan_Sts_T *const Sts);

PUBLIC void CanSimu_StartRx (   const HdswExtCan_If_T If,
  const HdswExtCan_MsgRx_T     *const Msgs,
  const HdswExtCan_NofRxMsgs_T        NofMsgs,
  /*@out@*/       HdswExtCan_Sts_T       *const Sts);


PUBLIC void CanSimu_GetRxMsg (const HdswExtCan_If_T If,
  /*@out@*/       HdswExtCan_MsgRx_T *const Msg,
  /*@out@*/       HdswExtCan_Sts_T   *const Sts);

PUBLIC void CanSimu_StopRx (const HdswExtCan_If_T If,
  /*@out@*/ HdswExtCan_Sts_T *const Sts);

PUBLIC void CanSimu_GetTxSts (   const HdswExtCan_If_T If,
  /*@out@*/       HdswNatural32_T         *const NofMsgs,
  /*@out@*/       HdswExtCan_StsDetails_T *const Details,
  /*@out@*/       HdswExtCan_TxSts_T      *const Sts);

PUBLIC void CanSimu_GetRxSts (const HdswExtCan_If_T If,
  /*@out@*/  HdswNatural32_T *const NofMsgs,
  /*@out@*/ HdswExtCan_StsDetails_T *const Details,
  /*@out@*/ HdswExtCan_RxSts_T      *const Sts);

CanSimCtx SimCtx[ROV_CANBUS_NUM] = 
{
  {
    .started = FALSE,
    .rxQ = 0,
    .txQ = 0
  },
  {
    .started = FALSE,
    .rxQ = 0,
    .txQ = 0
  }
};

#endif

static HdswExtCan_BitTiming_T bitTiming = 
{
  .BPR = CAN_BIT_TIMING_BPR,
  .RSJ = CAN_BIT_TIMING_RSJ,
  .PS1 = CAN_BIT_TIMING_PS1,
  .PS2 = CAN_BIT_TIMING_PS2,
  .SCALER = CAN_BIT_TIMING_SCALER_70MHZ
};


/*---------- Definition of local variables and constants --------------*/

/** Necessary HDSW CAN bus interface */
PRIVATE CancanBusIf canBusIf = {
#ifdef COMGT_UT_SUPPORT
  CanSimu_RestartIf,
  CanSimu_SetBusConnState,
  CanSimu_SetBitTiming,
  CanSimu_SelectBus,
  CanSimu_StartTx,
  CanSimu_StopTx,
  CanSimu_StartRx,
  CanSimu_GetRxMsg,
  CanSimu_StopRx,
  CanSimu_GetTxSts,
  CanSimu_GetRxSts
#else
  HdswExtCan_RestartIf,
  HdswExtCan_SetBusConnState,
  HdswExtCan_SetBitTiming,
  HdswExtCan_SelectBus,
  HdswExtCan_StartTx,
  HdswExtCan_StopTx,
  HdswExtCan_StartRx,
  HdswExtCan_GetRxMsg,
  HdswExtCan_StopRx,
  HdswExtCan_GetTxSts,
  HdswExtCan_GetRxSts
#endif
};

/* transmit messages buffer pool: CAN_IF_TX_LIST_NUM (2) list of 
 * CAN_IF_TX_LIST_LEGNTH message for each CAN bus interface
 *  
 * WARINING: they are not ensured being aligned 
 * at 4 32-bits word boundary, as required by CAN controller
 * So, one additional message buffer is included */

PRIVATE HdswExtCan_MsgTx_T txMsgsPool[(CAN_IF_TX_LIST_LEGNTH*CAN_IF_TX_LIST_NUM)*ROV_CANBUS_NUM+1];


/* reception messages buffer pool: 
 * CAN_IF_RX_BUFFER_MSGS message for each CAN bus interface
 *  
 * WARINING: they are not ensured being aligned 
 * at 1Kbyte boundary, as required by CAN controller
 * So, one additional 1 Kbyte buffer is included */

#define CANBUS_RX_BUFFER_SIZE (sizeof(HdswExtCan_MsgRx_T) * CAN_IF_RX_BUFFER_MSGS) /* multiple of Kbytes */
#define K_BYTE (0x400)

PRIVATE U08 rxMemPool[CANBUS_RX_BUFFER_SIZE*ROV_CANBUS_NUM + K_BYTE];


/* {{RELAX<GEN-005-M> All fields have been initialised apart the one defined with an union. The union comes from the RUAG file hdsw.h and has to be left as is. */
PRIVATE CanBusCtx busCtx[ROV_CANBUS_NUM] = 
{
  {
    /* global configuration parameter/status */
    .initialised = FALSE,
    .bus = HdswExtCan_If_B_E,
    .chan = HdswExtCan_NomRed_Nom_E, /* default value */
    .connState = HdswExtCan_BusConnState_Disconnected_E,
    
    /* transmission configuration/status */
    .txSts = HdswCommon_Sts_Success_E,
    .txStarted = FALSE,
    .txListMsgs[0] = 0,
    .txListMsgs[1] = 0,
    .curTxCycle = 0,
    .curTxSlot = 0,
    .curTxList = CAN_IF_TX_LIST_NONE,
    .curTxMsgs = 0,
    .totalTxList = 0, 
    .totalTxMsgs = 0,
    
    .rxWhileBusyTransmission = FALSE,
    .rxWhileBusyTxElapsedItSlots = 0,
  
    /* reception configuration/status */
    .rxSts = HdswCommon_Sts_Success_E,
    .rxStarted = FALSE,
    .curRxMsgs = 0, 
    .totalExtMsgs = 0,
    
    /* dummy initialisations, for rule GEN-005-M compliance */
    .pTxList[0] = NULL_VOID_PTR,
    .pTxList[1] = NULL_VOID_PTR,
    .pRxBuffer = NULL_VOID_PTR,
    .txDetails = {.Data = {0, 0, 0}},
    .rxDetails = {.Data = {0, 0, 0}},
    .rxMsg = {.MsgHdrCmn = {.Data = 0}, 
              .MsgHdrRx = {.Data = 0}},

    .elapsedItSlots = 0,
    .mstAloneOnBus = FALSE
  },
  {
    /* global configuration parameter/status */
    .initialised = FALSE,
    .bus = HdswExtCan_If_A_E,
    .chan = HdswExtCan_NomRed_Nom_E, /* default value */
    .connState = HdswExtCan_BusConnState_Disconnected_E,
    
    /* transmission configuration/status */
    .txSts = HdswCommon_Sts_Success_E,
    .txStarted = FALSE,
    .txListMsgs[0] = 0,
    .txListMsgs[1] = 0,
    .curTxCycle = 0,
    .curTxSlot = 0,
    .curTxList = CAN_IF_TX_LIST_NONE,
    .curTxMsgs = 0,
    .totalTxList = 0, 
    .totalTxMsgs = 0,
    
    .rxWhileBusyTransmission = FALSE,
    .rxWhileBusyTxElapsedItSlots = 0,
    
    /* reception configuration/status */
    .rxSts = HdswCommon_Sts_Success_E,
    .rxStarted = FALSE,
    .curRxMsgs = 0, 
    .totalExtMsgs = 0, 
    
    /* dummy initialisations, for rule GEN-005-M compliance */
    .pTxList[0] = NULL_VOID_PTR,
    .pTxList[1] = NULL_VOID_PTR,
    .pRxBuffer = NULL_VOID_PTR,
    .txDetails = {.Data = {0, 0, 0}},
    .rxDetails = {.Data = {0, 0, 0}},
    .rxMsg = {.MsgHdrCmn = {.Data = 0}, 
              .MsgHdrRx = {.Data = 0}},
        
    .elapsedItSlots = 0,
    .mstAloneOnBus = FALSE
  },
};
/* }}RELAX<GEN-005-M> */  

/*---------- Declarations of local functions --------------------------*/
#ifdef COMGT_UT_SUPPORT
PRIVATE CoStatus CanBus_checkMsg(Uint , Uint , Uint , U08 *);
#endif
PRIVATE CoStatus CanBus_updateTxCtx(Uint busId);
PRIVATE CoStatus CanBus_stop(CanBusCtx *pBusCtx, Uint busId);
/*------------------ ooOoo Inline functions (if any) ooOoo ------------*/

/*------------------ ooOoo Global functions ooOoo ---------------------*/

/***********************************************************************/
/**
 * @brief CanOpenBus_setup - setup a CAN Bus Interface
 *  
 * This function performs the some general operations for on a CAN Bus interface.
 * It is the very first function to be called and called once once
 * before using a CAN Bus. 
 * 
 * @param[in] busId   The bus identifier
 * @return 
 * - @link E_COSTATUS_OK @endlink
 * - @link E_COSTATUS_ERROR @endlink
 * 
 * 
 ***********************************************************************/ 
CoStatus CanOpenBus_setup(Uint busId)
{
  CoStatus result = E_COSTATUS_OK;
  CanBusCtx *pBusCtx;
  HdswExtCan_If_T bus;
  
  /* get Bus context */
  busId %= ROV_CANBUS_NUM;
  pBusCtx = &busCtx[busId];
 
  /* reset context variables to their default values (0 except few) */
  bus = pBusCtx->bus;
  memset(pBusCtx, 0, sizeof (CanBusCtx));  
  pBusCtx->bus = bus;
  pBusCtx->chan = HdswExtCan_NomRed_Nom_E;
  pBusCtx->connState = HdswExtCan_BusConnState_Disconnected_E;
  pBusCtx->txSts = HdswCommon_Sts_Success_E;
  pBusCtx->curTxList = CAN_IF_TX_LIST_NONE;
  pBusCtx->rxSts = HdswCommon_Sts_Success_E;
  
#ifdef COMGT_UT_SUPPORT
    /* create the message queue used by simulation once only for a bus */
    rtems_name rtemsName;
    rtems_id rtemsId;
    rtems_status_code rtSts;
    Uint msgs;
    
    /* for transmission */
    msgs = CO_TX_MSG_MAX_IN_SLOT * 2,
    rtemsName = rtems_build_name('t', 'x', busId, 0);
    rtSts = rtems_message_queue_create(rtemsName, msgs,
                                       sizeof(HdswExtCan_MsgTx_T),
                                       RTEMS_LOCAL, &rtemsId);
    /* %COVER%TRUE% Defensive programming - ERROR_REPORT macro called here. */    
    if (rtSts != RTEMS_SUCCESSFUL)
    {
        ERROR_REPORT(SW_ERROR, busId, rtemsName, rtSts);
        return (E_COSTATUS_ERROR);
    }
    SimCtx[busId].txQ = rtemsId;
    
    /* for reception */
    msgs = CAN_IF_RX_BUFFER_MSGS * 2;
    rtemsName = rtems_build_name('r', 'x', busId, 0);
    rtSts = rtems_message_queue_create(rtemsName, msgs,
                                       sizeof(HdswExtCan_MsgRx_T),
                                       RTEMS_LOCAL, &rtemsId);
    /* %COVER%TRUE% Defensive programming - ERROR_REPORT macro called here. */    
    if (rtSts != RTEMS_SUCCESSFUL)
    {
        ERROR_REPORT(SW_ERROR, busId, rtemsName, rtSts);
        return (E_COSTATUS_ERROR);
    }
    SimCtx[busId].rxQ = rtemsId;
    SimCtx[busId].started = TRUE;
  
#endif
  pBusCtx->initialised = TRUE;
 
  return result;
}

/***********************************************************************/
/**
 * @brief CanOpenBus_getCtrlStatus - Get the content of the status register
 * of the CAN Bus Interface
 *  
 * This function directly reads the status register of the CAN controller
 * and report it. .
 *
 * @param[in] busId   The bus identifier
 * @return content of the CAN controller status register
 * 
 ***********************************************************************/ 
U32 CanOpenBus_getCtrlStatus(Uint busId)
{
#ifndef COMGT_UT_SUPPORT
  static U32 *pCanStatReg[ROV_CANBUS_NUM] =
  {
    (U32 *) CANB_STAT_REG_ADDR, /* Platform bus */
    (U32 *) CANA_STAT_REG_ADDR  /* Payload bus */
  };
#else
  static U32 simulatedSts = UNSET_CAN_STAT_VALUE;
  static U32 *pCanStatReg[ROV_CANBUS_NUM] =
  {
    &simulatedSts, &simulatedSts
  };
#endif
  U32 status;
  U32 bus;
  
  
  /* get Bus context */
  bus = busId%ROV_CANBUS_NUM;
  status = *pCanStatReg[bus];

  return status;
}

/***********************************************************************/
/**
 * @brief CanOpenBus_getMstAloneOnBus - Check Master is Alone in the Bus
 *  
 * Based on the BUS id check the Master Alone in Bus sattus
 *
 * @param[in] busId   The bus identifier
 * @return mstAloneOnBus flag status
 *
 ***********************************************************************/ 
Bool CanOpenBus_getMstAloneOnBus(Uint busId)
{
  CanBusCtx *pBusCtx;
  
  pBusCtx = &busCtx[busId%ROV_CANBUS_NUM];
  
  return pBusCtx->mstAloneOnBus;
}

/***********************************************************************/
/**
 * @brief CanOpenBus_restart - Initialisation/Restart a CAN Bus Interface
 *  
 * This function performs the global initialisation or Restart operations for
 * on a CAN Bus interface. 
 * 
 * It performs a complete initialisation procedure:
 *  - restart CAN interface,
 *  - set the CAN Bus baud rate and other timing configuration
 *  - set the active channel
 *  - enable the CAN Bus  
 * 
 * @param[in] busId   The bus identifier
 * @param[in] chanId  The channel identifier
 * @return - @link E_COSTATUS_OK @endlink or error code
 *        - @link E_COSTATUS_CAN_IF_ERROR @endlink
 *        
 * @requirements
 * - SRS.DMS.CAN.FUNC.0415
 * 
 ***********************************************************************/ 
CoStatus CanOpenBus_restart(Uint busId, Uint chanId)
{
  CoStatus result = E_COSTATUS_OK;
  HdswExtCan_Sts_T canSts;
  CanBusCtx *pBusCtx;
  
  /* get Bus context */
  busId %= ROV_CANBUS_NUM;
  pBusCtx = &busCtx[busId];
  
  /* check Bus channel */
  if (chanId == CANBUS_NOM_CHAN)
  {
    pBusCtx->chan = HdswExtCan_NomRed_Nom_E;
  }
  else
  {
    pBusCtx->chan = HdswExtCan_NomRed_Red_E;
  }
  
  /* if TX or RX is ongoing, stop them */
  result = CanBus_stop(pBusCtx, busId);
  
  /* %COVER%FALSE% Defensive programming - HDSW error during previous call 
   * already processed, no need for ERROR_REPORT */
  if (result == E_COSTATUS_OK)
  { 
    /* now, apply a complete restart procedure per HDSW UM */
    pBusCtx->txStarted = FALSE;
    pBusCtx->curTxList = 0;
    pBusCtx->curTxMsgs = 0;
    pBusCtx->rxStarted = FALSE;
    pBusCtx->curRxMsgs = 0;
    pBusCtx->rxWhileBusyTxElapsedItSlots = 0;
    pBusCtx->rxWhileBusyTransmission = FALSE;
    pBusCtx->mstAloneOnBus = FALSE;

    /* 1: Disconnect CAN Bus Interface before any configuration operation */
    canBusIf.setBusConnState (pBusCtx->bus, HdswExtCan_BusConnState_Disconnected_E, &canSts);
    
    /* %COVER%TRUE% Defensive programming - ERROR_REPORT macro called here. */
    if (canSts != HdswExtCan_Sts_Success_E)
    {
      ERROR_REPORT(SW_ERROR, pBusCtx->bus, pBusCtx->chan, canSts);
      result = E_COSTATUS_CAN_IF_ERROR;
    }
    else
    {
      pBusCtx->connState = HdswExtCan_BusConnState_Disconnected_E;
      
      /* 2: Set default bit timing for the CAN Bus Interface: unexpected error  */
      canBusIf.setBitTiming (pBusCtx->bus, &bitTiming, &canSts);
      /* %COVER%TRUE% Defensive programming - ERROR_REPORT macro called here. */
      if (canSts != HdswExtCan_Sts_Success_E)
      {
        ERROR_REPORT(SW_ERROR, pBusCtx->bus, pBusCtx->chan, canSts);
        result = E_COSTATUS_CAN_IF_ERROR;
      }
      else
      {
        /* 3: Set select channel */
        canSts = canBusIf.selectBus (pBusCtx->bus, pBusCtx->chan);
        /* %COVER%TRUE% Defensive programming - ERROR_REPORT macro called here. */    
        if (canSts != HdswExtCan_Sts_Success_E)
        {
          ERROR_REPORT(SW_ERROR, pBusCtx->bus, pBusCtx->chan, canSts);
          result = E_COSTATUS_CAN_IF_ERROR;
        }
        else
        {
          /* 4: Connect CAN Bus Interface */
          canBusIf.setBusConnState (pBusCtx->bus, HdswExtCan_BusConnState_Connected_E, &canSts);
          /* %COVER%TRUE% Defensive programming - ERROR_REPORT macro called here. */
          if (canSts != HdswExtCan_Sts_Success_E)
          {
            ERROR_REPORT(SW_ERROR, pBusCtx->bus, pBusCtx->chan, canSts);
            result = E_COSTATUS_CAN_IF_ERROR;
          }
          else
          {
            pBusCtx->connState = HdswExtCan_BusConnState_Connected_E;
          }
        }
      }
    }
  }
  return result;
}

/***********************************************************************/
/**
 * @brief CanOpenBus_putMsgInList - Build a CAN message in a message list buffer
 * 
 * This function builds a transmit message in the specified transmit message list.
 * 
 * @param[in] busId     The bus identifier
 * @param[in] listId    The list number identifier
 * @param[in] msgId     The CAN message identifier
 * @param[in] msgRtr    The RTR of the message
 * @param[in] dataBytes The data size in bytes
 * @param[in] pData     The data buffer
 * @return - @link E_COSTATUS_OK @endlink or error code
 *         - @link E_COSTATUS_PARAM_ERROR @endlink
 *         - @link E_COSTATUS_MSG_LIST_FULL @endlink
 * 
 ***********************************************************************/ 
CoStatus CanOpenBus_putMsgInList(Uint busId, Uint listId, Uint msgId, Uint msgRtr, Uint dataBytes, U08 *pData)
{
  CoStatus result = E_COSTATUS_OK;
  Bool go = TRUE;
  CanBusCtx *pBusCtx;
  Uint list;
  Uint bus;
  Uint curMsg;
  HdswExtCan_MsgTx_T *pList;
  HdswExtCan_MsgTx_T *pMsg;
  
  /* get Bus context */
  bus = busId%ROV_CANBUS_NUM;
  pBusCtx = &busCtx[bus];
  
  /* check if the specified list is still in use (transmit on-going) */
  list = listId%CAN_IF_TX_LIST_NUM;
  /* %COVER%TRUE% Defensive programming - ERROR_REPORT macro called here. */  
  if ((list == pBusCtx->curTxList) && (pBusCtx->txStarted == TRUE))
  {
    ERROR_REPORT(SW_ERROR, pBusCtx->bus, pBusCtx->curTxList, listId);
    result = E_COSTATUS_PARAM_ERROR;
    go = FALSE;
  }
  
  /* check if the specified list is already full */
  /* %COVER%TRUE% Defensive programming - ERROR_REPORT macro called here. */  
  if (go && (pBusCtx->txListMsgs[list] >= CAN_IF_TX_LIST_LEGNTH))
  {
    ERROR_REPORT(SW_ERROR, pBusCtx->bus, pBusCtx->curTxList, pBusCtx->txListMsgs[list]);
    result = E_COSTATUS_MSG_LIST_FULL;
  }
  else
  {
#ifdef COMGT_UT_SUPPORT
    /* check input message format */   
    result = CanBus_checkMsg(msgId, msgRtr, dataBytes, pData);
    /* %COVER%TRUE% Defensive programming - ERROR_REPORT macro already called above for this error case. */    
    if (result != E_COSTATUS_OK)
    {
      go = FALSE;
    }
#endif
    /* put the input message format in to the specified message list */
    /* %COVER%FALSE% Defensive programming - ERROR_REPORT macro already called above for this error case. */
    if (go)
    {
      /* locate the current message in the current buffer list */ 
      curMsg = pBusCtx->txListMsgs[list];
      pList = pBusCtx->pTxList[list];
      pMsg = &pList[curMsg];
      
      /* fill in the two headers and data if any */
      dataBytes = MIN(dataBytes, CAN_MSG_SIZE); 
      memset (pMsg, 0, sizeof(HdswExtCan_MsgTx_T));
      pList[curMsg].MsgHdrCmn.Data = STD_MSG_HDR_BUILD(msgRtr,msgId); /* MACRO used for performance reason */
      pList[curMsg].MsgHdrTx.Bits.DLC = dataBytes;
      memcpy (pList[curMsg].MsgData.Data, pData, dataBytes);
     
      /* update the message counter of the list */
      pBusCtx->txListMsgs[list]++;
    }
  }
  return result;
}

/***********************************************************************/
/**
 * @brief CanOpenBus_txMsgList - transmit messages in a message list buffer
 *  
 * This function starts the transmission of message in the specified transmit message list.
 * 
 * @param[in] busId   The bus identifier
 * @param[in] listId  The list number
 * @param[in] msgNb   The messages in the list (for verification purpose)
 * @param[in] cycle   Current cycle number
 * @param[in] slot    Current slot number
 * @return - @link E_COSTATUS_OK @endlink or error code
 *         - @link E_COSTATUS_PARAM_ERROR @endlink
 *         - @link E_COSTATUS_ACTPM_ERROR @endlink
 *         - @link E_COSTATUS_EXTCAN_ERROR @endlink
 *
 ***********************************************************************/ 
CoStatus CanOpenBus_txMsgList(Uint busId, Uint listId, Uint msgNb, Uint cycle, Uint slot)
{
  CoStatus result = E_COSTATUS_OK;

  HdswExtCan_Sts_T canSts;
#ifdef COMGT_UT_SUPPORT
  static int lastMsgs[ROV_CANBUS_NUM] = {0, 0};
  static int totalMsgs[ROV_CANBUS_NUM] = {0, 0};
  Uint idx;
  CanSimCtx *pSimu;
  rtems_id txQ;
  rtems_status_code rtSts;
  Uint pendingMsg;
  Bool go = TRUE;
#endif
  CanBusCtx *pBusCtx;
  Uint list;
  Uint bus;
  HdswExtCan_MsgTx_T *pList;
  
  /* get bus context */
  bus = busId%ROV_CANBUS_NUM;
  pBusCtx = &busCtx[bus];
 
  /* update ongoing transmit context */
  pBusCtx->curTxCycle = cycle;
  pBusCtx->curTxSlot = slot;
  result = CanBus_updateTxCtx(busId);
  
  switch (result)
  {
  case E_COSTATUS_OK:
  case E_COSTATUS_BUS_BUSY:
  case E_COSTATUS_ACTPM_ERROR:
  case E_COSTATUS_EXTCAN_ERROR:
    /* expected return status: nothing to do */
    break;
    
  case E_COSTATUS_PARAM_ERROR:
    /* %COVER%STMT% Defensive programming - ERROR_REPORT macro called here. */
     /* unexpected return status: nothing to do */
  default:
    /* %COVER%STMT% Defensive programming - ERROR_REPORT macro called here. */
    /* unexpected return status: nothing to do */
    ERROR_REPORT(SW_ERROR, pBusCtx->bus, pBusCtx->chan, msgNb);
    break;
  } /* switch */
    
  /* perform the transmission only if the IF is OK */
  /* Defensive programming - error in previous HDSW operation, no ERROR_REPORT needed. */  
  if (result == E_COSTATUS_OK)
  { 
    /* check if the specified list contains requested message number */
    list = listId%CAN_IF_TX_LIST_NUM;
    
    /* %COVER%TRUE% Defensive programming - ERROR_REPORT macro called here. */  
    if (pBusCtx->txListMsgs[list] != msgNb)
    {
      ERROR_REPORT(SW_ERROR, pBusCtx->bus, msgNb, pBusCtx->txListMsgs[list]);
      result = E_COSTATUS_PARAM_ERROR;
    }
    else
    {
      pList = pBusCtx->pTxList[list];
      pBusCtx->txSts = HdswExtCan_TxSts_FcgExtCan_E;
      canBusIf.startTx (pBusCtx->bus, pList, msgNb, &canSts);

#ifdef COMGT_UT_SUPPORT
      /* simulation the message sending by the UT-dedicated queue */
      pSimu = &SimCtx[bus];
      txQ = pSimu->txQ;
      
      /* send the messages of the list to the txQ, if txQ is empty */
      pendingMsg = 0;
      rtems_message_queue_get_number_pending(txQ,
                            (rtems_unsigned32 *) &pendingMsg);
      /* %COVER%TRUE% Defensive programming - ERROR_REPORT macro called here. */
      if (pendingMsg != 0)
      {
        ERROR_REPORT(SW_ERROR, pBusCtx->bus, pendingMsg, lastMsgs[bus]);
        ERROR_REPORT(SW_ERROR, pBusCtx->bus, LEON_REG.Timer_Counter_2, 0);
        result = E_COSTATUS_ERROR;
      }
      else
      {
        for (idx = 0; go&&(idx < pBusCtx->txListMsgs[list]); idx++)
        {
          rtSts = rtems_message_queue_send(txQ, (char *) &pList[idx],
                              sizeof(HdswExtCan_MsgTx_T));
          /* %COVER%TRUE% Defensive programming - ERROR_REPORT macro called here. */
          if (rtSts != RTEMS_SUCCESSFUL)
          {
            ERROR_REPORT(SW_ERROR, pBusCtx->bus, rtSts, pendingMsg);
            ERROR_REPORT(SW_ERROR, pBusCtx->bus, lastMsgs[bus], totalMsgs[bus]);
            result = E_COSTATUS_ERROR;
            go = FALSE;
          }
        }
        /* notify the message sending from manager */
        /* %COVER%FALSE% Defensive programming - ERROR_REPORT generated */          
        if (idx != 0)
        {
          rtSts = rtems_semaphore_release(CoMgt_SlotSyncSemId[bus]);
          /* %COVER%TRUE% Defensive programming - ERROR_REPORT macro called here. */
          if (rtSts != RTEMS_SUCCESSFUL)
          {
            ERROR_REPORT(SW_ERROR, pBusCtx->bus, list, rtSts);
          }
        }
        else
        {
          ERROR_REPORT(SW_ERROR, pBusCtx->bus, 0, 0);
        }
      }
      lastMsgs[bus] = pBusCtx->txListMsgs[list];
      totalMsgs[bus] += pBusCtx->txListMsgs[list];
  #endif    
      /* %COVER%TRUE% Defensive programming - ERROR_REPORT macro called here. */
      if (canSts != HdswExtCan_Sts_Success_E)
      {
        ERROR_REPORT(SW_ERROR, pBusCtx->bus, pBusCtx->chan, canSts);
        result = E_COSTATUS_CAN_IF_ERROR;
      }
      else
      {
        pBusCtx->txStarted = TRUE;
        pBusCtx->curTxList = list;
        pBusCtx->totalTxList++;
        pBusCtx->totalTxMsgs += pBusCtx->txListMsgs[list];
        
        pBusCtx->txSts = HdswExtCan_TxSts_Success_E;
        pBusCtx->curTxMsgs = 0;
      }     
    }
  }
  return result;
}

/***********************************************************************/
/**
 * @brief CanOpenBus_getTxSts - get the status of the previous message list transmission
 * 
 * This function gets the status of messages transmitted in the previous message list.
 * 
 * @param[in] busId   The bus identifier
 * @param[out] pChan  The buffer to store channel number
 * @param[out] pList  The buffer to store list number
 * @param[out] pMsgs  The buffer to store messages in the list
 * @return - @link E_COSTATUS_OK @endlink or error code
 *         - @link E_COSTATUS_PARAM_ERROR @endlink
 *         - @link E_COSTATUS_ACTPM_ERROR @endlink
 *         - @link E_COSTATUS_EXTCAN_ERROR @endlink
 *         - @link E_COSTATUS_BUS_BUSY @endlink
 * 
 ***********************************************************************/ 
CoStatus CanOpenBus_getTxSts(Uint busId, Uint *pChan, Uint *pList, Uint *pMsgs)
{
  CoStatus result = E_COSTATUS_OK;
  CanBusCtx *pBusCtx;
  Uint list;
  Uint bus;
  Uint chan;
  T_BOOL pChanInvalid;
  T_BOOL pListInvalid;
  T_BOOL pMsgsInvalid;
  
  pChanInvalid = INVALID_U32_PTR(pChan);
  pListInvalid = INVALID_U32_PTR(pList);
  pMsgsInvalid = INVALID_U32_PTR(pMsgs);

  /* get bus context */
  bus = busId%ROV_CANBUS_NUM;
  pBusCtx = &busCtx[bus];
  list = pBusCtx->curTxList;
  if (pBusCtx->chan == HdswExtCan_NomRed_Nom_E)
  {
    chan = CANBUS_NOM_CHAN;
  }
  else
  {
    chan = CANBUS_RED_CHAN;
  }
  
  /* check the input pointer */
  /* %COVER%TRUE% Defensive programming - ERROR_REPORT macro called here. */
  if (pChanInvalid || pListInvalid
      || pMsgsInvalid)
  {
    ERROR_REPORT(SW_ERROR, (Uint)pChan, (Uint)pList, (Uint)pMsgs);
    result = E_COSTATUS_PARAM_ERROR;
  }
  else
  {
    /* update ongoing transmit context */
    result = CanBus_updateTxCtx(busId);
    
    switch (result)
    {
    case E_COSTATUS_OK:
    case E_COSTATUS_ACTPM_ERROR:
    case E_COSTATUS_EXTCAN_ERROR:
    case E_COSTATUS_BUS_BUSY:
      /* expected return status: nothing to do */
      break;
    case E_COSTATUS_PARAM_ERROR:
      /* %COVER%STMT% Defensive programming - ERROR_REPORT macro called here. */ 
    default:
      /* unexpected return status: ERROR REPORT */
      /* %COVER%STMT% Defensive programming - ERROR_REPORT macro called here. */ 
      ERROR_REPORT(SW_ERROR, pBusCtx->bus, pBusCtx->chan, result);
      break;
    } /* switch */
    
    *pChan = chan;
    *pList = list;
    *pMsgs = pBusCtx->curTxMsgs;
  }
  
  return result;
}

/***********************************************************************/
/**
 * @brief CanOpenBus_stopTxMsg - stop ongoing message transmission
 * 
 * This function stops ongoing message transmission on a CAN Bus.
 * 
 * @param[in] busId The bus identifier
 * @return - @link E_COSTATUS_OK @endlink or error code
 *         - @link E_COSTATUS_CAN_IF_ERROR @endlink
 *         

 ***********************************************************************/ 
CoStatus CanOpenBus_stopTxMsg(Uint busId)
{
  CoStatus result = E_COSTATUS_OK;
  HdswExtCan_Sts_T canSts;
  CanBusCtx *pBusCtx;
  Uint bus;
  
  /* get bus context */
  bus = busId%ROV_CANBUS_NUM;
  pBusCtx = &busCtx[bus];
  
  /* invoke CAN IF service to stop any ongoing transmission */
  canBusIf.stopTx(pBusCtx->bus, &canSts);
  /* %COVER%TRUE% Defensive programming - error returned. */
  if (canSts != HdswExtCan_Sts_Success_E)
  {
    ERROR_REPORT(SW_ERROR, pBusCtx->bus, pBusCtx->chan, canSts);
    result = E_COSTATUS_CAN_IF_ERROR;
  }
  else
  {
    pBusCtx->txStarted = FALSE;
    /* clear any message pending at stop time */
    pBusCtx->txListMsgs[pBusCtx->curTxList] = 0;
  }

  return result;
}
/***********************************************************************/
/**
 * @brief CanOpenBus_enableRxMsg - enable reception of message
 * 
 * This function enables the CAN Controller for message reception
 * 
 * @param[in] busId The bus identifier
 * @return - @link E_COSTATUS_OK @endlink or error code
 *         - @link E_COSTATUS_CAN_IF_ERROR @endlink
 *
 ***********************************************************************/ 
CoStatus CanOpenBus_enableRxMsg(Uint busId)
{
  CoStatus result = E_COSTATUS_OK;
  CanBusCtx *pBusCtx;
  HdswExtCan_Sts_T canSts;
  Uint bus;
  
  /* get bus context */
  bus = busId%ROV_CANBUS_NUM;
  pBusCtx = &busCtx[bus];
  
  /* invoke CAN IF service to enable the message reception  */
  canBusIf.startRx (pBusCtx->bus, pBusCtx->pRxBuffer, CAN_IF_RX_BUFFER_MSGS, &canSts);
  /* %COVER%TRUE% Defensive programming - ERROR_REPORT macro called here. */
  if (canSts != HdswExtCan_Sts_Success_E)
  {
    ERROR_REPORT(SW_ERROR, pBusCtx->bus, pBusCtx->chan, canSts);
    pBusCtx->rxStarted = FALSE;
    result = E_COSTATUS_CAN_IF_ERROR;
  }
  else
  {
    pBusCtx->rxStarted = TRUE;
  }
  
  return result;
}

/***********************************************************************/
/**
 * @brief CanOpenBus_getRxSts - get the status of the received message
 * 
 * This function provides the status of the received message
 * 
 * @param[in] busId   The bus identifier
 * @param[out] pChan  The buffer to store the channel number
 * @param[out] pMsgs  The buffer to store the number of messages in the list
 * @param[in] busId The bus identifier
 * @return - @link E_COSTATUS_OK @endlink or error code
 *         - @link E_COSTATUS_PARAM_ERROR @endlink
 *         - @link E_COSTATUS_EXTCAN_ERROR @endlink
 * 
 * @requirements
 * - SRS.DMS.CAN.FDIR.0100 [reset health Status if FCG_ACTIVE_PM for RX ]
 * - SRS.DMS.CAN.FDIR.0200 [reset health Status if FCG_EXTCAN for RX]
 * 
 ***********************************************************************/ 
CoStatus CanOpenBus_getRxSts(Uint busId, Uint *pChan, Uint *pMsgs)
{
  CoStatus result = E_COSTATUS_OK;
  HdswExtCan_RxSts_T canRxSts;
  CanBusCtx *pBusCtx;
  Uint bus;
  Uint msgNb;
  Uint chan;
  T_BOOL pChanInvalid;
  T_BOOL pMsgsInvalid;
  
  pChanInvalid = INVALID_U32_PTR(pChan);
  pMsgsInvalid = INVALID_U32_PTR(pMsgs);
  
  /* get bus context */
  bus = busId%ROV_CANBUS_NUM;
  pBusCtx = &busCtx[bus];
  if (pBusCtx->chan == HdswExtCan_NomRed_Nom_E)
  {
    chan = CANBUS_NOM_CHAN;
  }
  else
  {
    chan = CANBUS_RED_CHAN;
  }
  
  /* check the input pointers */
  /* %COVER%TRUE% Defensive programming - ERROR_REPORT macro called here. */
  if (pChanInvalid || pMsgsInvalid)
  {
    ERROR_REPORT(SW_ERROR, busId, (Uint)pChan, (Uint)pMsgs);
    result = E_COSTATUS_PARAM_ERROR;
  }
  else
  {  
    if (pBusCtx->rxStarted != TRUE)
    {
      /* no need to read */
      canRxSts = HdswExtCan_RxSts_Success_E;
      result = E_COSTATUS_OK;
      pBusCtx->curRxMsgs = 0;
      
#ifdef COMGT_UT_SUPPORT
    {
      rtems_status_code rtSts;
      msgNb = 0;
      /* flush any pending messages in rxQ */
      rtSts = rtems_message_queue_flush(SimCtx[bus].rxQ,
                            (rtems_unsigned32 *) &msgNb);
      /* %COVER%TRUE% Defensive programming - Simulator-supported testing feature
       * ERROR_REPORT macro called here.
       */
      if (rtSts != RTEMS_SUCCESSFUL)
      {
        ERROR_REPORT(SW_ERROR, rtSts, SimCtx[bus].rxQ, msgNb);
      }
    }
#endif
    }
    else
    {  
      msgNb = 0;
      memset (&pBusCtx->rxDetails, 0, sizeof(HdswExtCan_StsDetails_T));
      canBusIf.getRxSts (pBusCtx->bus, &msgNb, &pBusCtx->rxDetails, &canRxSts);
      switch (canRxSts)
      {
      /* if HdswExtCan_RxSts_Success_E/HdswExtCan_RxSts_Busy_E, only message number is significant */
      case HdswExtCan_RxSts_Success_E:
      case HdswExtCan_RxSts_Busy_E:
#ifdef COMGT_UT_SUPPORT
      {
        rtems_status_code rtSts;
        msgNb = 0;
        /* get the message number queued in rxQ */
        rtSts = rtems_message_queue_get_number_pending(SimCtx[bus].rxQ,
                              (rtems_unsigned32 *) &msgNb);
        /* %COVER%TRUE% Defensive programming - Simulator-supported testing feature
         * ERROR_REPORT macro called here.
         */
        if (rtSts != RTEMS_SUCCESSFUL)
        {
          ERROR_REPORT(SW_ERROR, rtSts, SimCtx[bus].rxQ, msgNb);
        }
      }
#endif
        pBusCtx->curRxMsgs = msgNb;
        result = E_COSTATUS_OK;
        break;
        
      /* if HdswExtCan_RxSts_FcgActPm_E, only Details is significant */
      case HdswExtCan_RxSts_FcgActPm_E:
        pBusCtx->curRxMsgs = 0;  /* not significant */
        result = E_COSTATUS_ACTPM_ERROR;
        break;
      
      /* if HdswExtCan_RxSts_FcgExtCan_E: both msgNb, Details are significant */
      case HdswExtCan_RxSts_FcgExtCan_E:
        pBusCtx->curRxMsgs = msgNb;
        result = E_COSTATUS_EXTCAN_ERROR;
        break;
          
      case HdswExtCan_RxSts_FcgAsw_E:
        /* %COVER%STMT% Defensive programming - ERROR_REPORT macro called here. */
      default:
        /* %COVER%STMT% Defensive programming - ERROR_REPORT macro called here. */
        pBusCtx->curRxMsgs = 0; /* not significant */
        result = E_COSTATUS_PARAM_ERROR;
        ERROR_REPORT(SW_ERROR, canRxSts, 
                     pBusCtx->rxDetails.Data[0], pBusCtx->rxDetails.Data[1]);
        break;
      }
    }
    pBusCtx->rxSts = canRxSts;
    *pMsgs = pBusCtx->curRxMsgs;
    *pChan = chan;
   }
  return result;
}

/***********************************************************************/
/**
 * @brief CanOpenBus_getNextMsg - get the next received message
 * 
 * This function extracts the next received message from CAN Controller
 * 
 * @param[in] busId       The bus identifier
 * @param[out] pId        The buffer to store the  message identifier
 * @param[out] pRtr       The buffer to store the RTR of the message
 * @param[out] pDataBytes The buffer to store data size in bytes
 * @param[out] pData      The data buffer
 * @return - @link E_COSTATUS_OK @endlink or error code
 *         - @link E_COSTATUS_PARAM_ERROR @endlink
 *         - @link E_COSTATUS_CAN_IF_ERROR @endlink
 *
 ***********************************************************************/ 
CoStatus CanOpenBus_getNextMsg(Uint busId, Uint *pId, Uint *pRtr, Uint *pDataBytes, U08 *pData)
{
  CoStatus result = E_COSTATUS_OK;
  HdswExtCan_Sts_T canSts;
  CanBusCtx *pBusCtx;
  HdswExtCan_MsgRx_T *pMsg;
  Uint bus;
  T_BOOL pIdInvalid;
  T_BOOL pRtrInvalid;
  T_BOOL pDataBytesInvalid;
  T_BOOL pDataInvalid;
  
  /* get input pointer validity */
  pIdInvalid = INVALID_U32_PTR(pId);
  pRtrInvalid = INVALID_U32_PTR(pRtr);
  pDataBytesInvalid = INVALID_U32_PTR(pDataBytes);
  pDataInvalid = INVALID_U08_PTR(pData);

  /* get bus context */
  bus = busId%ROV_CANBUS_NUM;
  pBusCtx = &busCtx[bus];
  
  /* check the input pointers */
  /* %COVER%TRUE% Defensive programming - ERROR_REPORT macro called here. */
  if (pIdInvalid || pRtrInvalid ||
      pDataBytesInvalid || pDataInvalid)
  {
    ERROR_REPORT(SW_ERROR, (Uint)pId, (Uint)pRtr, (Uint)pDataBytes);
    result = E_COSTATUS_PARAM_ERROR;
  }
  else 
  {
    /* invoke IF service */
    canBusIf.getRxMsg (pBusCtx->bus, &pBusCtx->rxMsg, &canSts);
    
#ifdef COMGT_UT_SUPPORT
    {
      CanSimCtx *pSimu;
      rtems_id rxQ;
      Uint msgSize;
      rtems_status_code rtSts; 
      pSimu = &SimCtx[bus];
      rxQ = pSimu->rxQ;
     
      /*
       * retrieve the next Bus message
       */
      msgSize = 0;
      rtSts = rtems_message_queue_receive(rxQ, (void *) &pBusCtx->rxMsg,
                (rtems_unsigned32 *) &msgSize, RTEMS_NO_WAIT, RTEMS_NO_TIMEOUT);
      /* %COVER%TRUE% Defensive programming - ERROR_REPORT macro called here. */
      if (rtSts != RTEMS_SUCCESSFUL)
      {
        ERROR_REPORT(SIMPLE_ERROR, bus, rtSts, pBusCtx->curRxMsgs);
      }
      else 
      {
        /* %COVER%TRUE% Defensive programming - ERROR_REPORT macro called here. */
        if (msgSize != sizeof(HdswExtCan_MsgRx_T))
        {
          ERROR_REPORT(SIMPLE_ERROR, bus, msgSize, pBusCtx->curRxMsgs);
        }
      }
    }
    
#endif
    /* %COVER%FALSE% Defensive programming - ERROR_REPORT macro called here. */
    if (canSts == HdswCommon_Sts_Success_E)
    {
      pMsg = &pBusCtx->rxMsg;
      
      /* extract message */
      *pId = STD_MSG_HDR_TO_ID(pMsg->MsgHdrCmn.Data);
      *pRtr = STD_MSG_HDR_TO_RTR(pMsg->MsgHdrCmn.Data);
      *pDataBytes = MIN(pMsg->MsgHdrRx.Bits.DLC, CAN_MSG_SIZE);
      
      /* copy message data */
      memcpy (pData, pMsg->MsgData.Data, *pDataBytes);
      
      pBusCtx->curRxMsgs--;
      pBusCtx->totalExtMsgs++;
    }
    else
    {
      ERROR_REPORT(SW_ERROR, bus, canSts, pBusCtx->curRxMsgs);
      result = E_COSTATUS_CAN_IF_ERROR;
    }
  }
  
  return result;
}

/***********************************************************************/
/**
 * @brief CanOpenBus_disableRxMsg - disable message reception
 * 
 * This function disables message reception on CAN controller
 * 
 * @param[in] busId The bus identifier
 * @return - @link E_COSTATUS_OK @endlink or error code
 *         - @link E_COSTATUS_CAN_IF_ERROR @endlink
 * 
 ***********************************************************************/ 
CoStatus CanOpenBus_disableRxMsg(Uint busId)
{
  CoStatus result = E_COSTATUS_OK;
  HdswExtCan_Sts_T canSts;
  CanBusCtx *pBusCtx;
  Uint bus;
  
  /* get bus context */
  bus = busId%ROV_CANBUS_NUM;
  pBusCtx = &busCtx[bus];
  
  /* invoke CAN IF API */

  canBusIf.stopRx (pBusCtx->bus, &canSts);
  /* %COVER%TRUE% Defensive programming - ERROR_REPORT macro called here. */
  if (canSts != HdswExtCan_Sts_Success_E)
  {
    ERROR_REPORT(SW_ERROR, pBusCtx->bus, pBusCtx->chan, canSts);
    result = E_COSTATUS_CAN_IF_ERROR;
  }
  else
  {
    pBusCtx->rxStarted = FALSE;
    pBusCtx->curRxMsgs = 0;
  }

  return result;
}

/***********************************************************************/
/**
 * @brief CanOpenBus_storeItFromStart - store the elapsed IT slots
 * 
 * This function stores in the bus context the input number of elapsed IT slots.
 * 
 * @param[in] busId           The bus identifier
 * @param[in] itSlotFromStart The elapsed IT slots
 * 
 ***********************************************************************/ 
void CanOpenBus_storeItFromStart(Uint busId, Uint itSlotFromStart)
{
  CanBusCtx *pBusCtx;
  Uint bus;

  /* get bus context */
  bus = busId%ROV_CANBUS_NUM;
  pBusCtx = &busCtx[bus];
  
  /* store the received elapsed IT Slots */
  pBusCtx->elapsedItSlots = itSlotFromStart;
}

/*------------------ ooOoo Local functions ooOoo ----------------------*/

#ifdef COMGT_UT_SUPPORT
/***********************************************************************/
/**
 * @brief  CanBus_checkMsg - Check CAN Bus Message format
 * 
 * @param[in] msgId     The ID of the current message
 * @param[in] msgRtZ    The RTR of the current message
 * @param[in] dataBytes The data bytes the current message
 * @param[in] pData: buffer storing the data of the current message
 * @return - @link E_COSTATUS_OK @endlink or error code
 *         - @link E_COSTATUS_PARAM_ERROR @endlink
 * 
 *
 ***********************************************************************/ 
PRIVATE CoStatus CanBus_checkMsg(Uint msgId, Uint msgRtr, Uint dataBytes, U08 *pData)
{
  CoStatus result = E_COSTATUS_OK;
  Bool go = TRUE;
  T_BOOL pDataInvalid;
  
  /* check 7 MSB in the 11-bit word: must not be all "1" */
  /* %COVER%TRUE% Defensive programming - ERROR_REPORT macro called here. */
  if ((msgId & MSB_10_4_MSK) == MSB_10_4_MSK)
  {
    ERROR_REPORT(SW_ERROR, msgId, msgRtr, dataBytes);
    result = E_COSTATUS_PARAM_ERROR;
    go = FALSE;
  }
  
  /* check RTR: data frame or 1 remote frame */
  /* %COVER%TRUE% Defensive programming - ERROR_REPORT macro called here. */
  if ((go) && ((msgRtr != CAN_MSG_RTR_DATA_FRAME) &&
               (msgRtr != CAN_MSG_RTR_REMOTE_FRAME)))
  {
    ERROR_REPORT(SW_ERROR, msgId, msgRtr, dataBytes);
    result = E_COSTATUS_PARAM_ERROR;
    go = FALSE;   
  }

  /* check data bytes according to RTR */
  /* %COVER%TRUE% Defensive programming - ERROR_REPORT macro called here. */
  if ((go) && (((msgRtr == CAN_MSG_RTR_DATA_FRAME) &&
                (dataBytes > CAN_MSG_SIZE))        ||
              ((msgRtr == CAN_MSG_RTR_REMOTE_FRAME) && 
                (dataBytes == 0))))
  {
    ERROR_REPORT(SW_ERROR, msgId, msgRtr, dataBytes);
    result = E_COSTATUS_PARAM_ERROR;
    go = FALSE;   
  }
  
  pDataInvalid = INVALID_U08_PTR(pData);
  /* check data buffer according to RTR */
  /* %COVER%TRUE% Defensive programming - ERROR_REPORT macro called here. */
  if ((go) && ((msgRtr == CAN_MSG_RTR_DATA_FRAME) && (pDataInvalid)))
  {
    ERROR_REPORT(SW_ERROR, msgId, dataBytes, (Uint)pData);
    result = E_COSTATUS_PARAM_ERROR;
  }
  return result;
}
#endif
/***********************************************************************/
/**
 * @brief  CanBus_updateTxCtx - update the message transmit context
 * 
 * @param[in,out] pBusCtx The pointer to the bus context of a Bus
 * @return - @link E_COSTATUS_OK @endlink or error code
 *         - @link E_COSTATUS_PARAM_ERROR @endlink
 *         - @link E_COSTATUS_ACTPM_ERROR @endlink
 *         - @link E_COSTATUS_EXTCAN_ERROR @endlink
 *         - @link E_COSTATUS_BUS_BUSY @endlink
 *
 * @requirements
 * - SRS.DMS.CAN.FDIR.0100 [reset health Status if FCG_ACTIVE_PM for TX ]
 * - SRS.DMS.CAN.FDIR.0200 [reset health Status if FCG_EXTCAN/BUSY for TX]
 * 
 ***********************************************************************/ 
PRIVATE CoStatus CanBus_updateTxCtx(Uint busId)
{
  CoStatus result;
  Uint list;
  Uint msgNb;
  HdswExtCan_TxSts_T canSts = HdswExtCan_TxSts_FcgAsw_E;
  Uint chan;
  Uint msgs;
  CanBusCtx *pBusCtx;
  
  pBusCtx = &busCtx[busId%ROV_CANBUS_NUM];
  
  pBusCtx->mstAloneOnBus = FALSE;
 
  /* if no transmission on-going,  nothing to do */
  if (pBusCtx->txStarted == TRUE)
  {
    list = pBusCtx->curTxList;
    canBusIf.getTxSts (pBusCtx->bus, &msgNb, &pBusCtx->txDetails, &canSts);
    
    /* There are 2 cases where we can get busy, check to differentiate */
    if (canSts == HdswExtCan_TxSts_Busy_E)
    {
      /* if a reception while busy transmission happened during the previous IT slot, mark the error */ 
      if (pBusCtx->rxWhileBusyTransmission == TRUE)
      {
        /* %COVER%FALSE% Defensive programming - There is no identified case where the busy transmission is 
         * detected twice in the same Fast Slot, but this check is left in order to cover any unforeseen 
         * case and raise false errors. */
        if (pBusCtx->elapsedItSlots > pBusCtx->rxWhileBusyTxElapsedItSlots)
        {        	
          canSts = HdswExtCan_TxSts_FcgExtCan_E;
        }
      }
      else
      {
        pBusCtx->rxWhileBusyTransmission = FALSE;
        pBusCtx->rxWhileBusyTxElapsedItSlots = 0;
        result = CanOpenBus_getRxSts(busId, &chan, &msgs);
     
        /* if messages are received and we cannot transmit => babbling idiot case of a slave, stop; 
         * also stop if reception in error. The error is marked only if this happens in two consecutive 
         * IT slots */
        if ((result == E_COSTATUS_OK) && (msgs != 0))
        {
          pBusCtx->rxWhileBusyTransmission = TRUE;
          pBusCtx->rxWhileBusyTxElapsedItSlots = pBusCtx->elapsedItSlots;
        }
        /* reported external error */
        else if (result == E_COSTATUS_EXTCAN_ERROR)
        {
          canSts = HdswExtCan_TxSts_FcgExtCan_E;
        }
        /* else no message received = nominal case master alone on bus at initialisation, nothing to do */
        else
        {
          pBusCtx->mstAloneOnBus = TRUE;
        }
      }
    }
    else
    {
      pBusCtx->rxWhileBusyTransmission = FALSE;
    }
    
#ifdef COMGT_UT_SUPPORT
    /* ALWAYS simulate right result */
    msgNb = pBusCtx->txListMsgs[list];   /* simulate right result */
#endif
    pBusCtx->txSts = canSts;
    pBusCtx->curTxMsgs = msgNb;
    
    switch (canSts)
    {
    /* if SUCCESS, consider that the list is finished */
    case HdswExtCan_TxSts_Success_E:
      /* %COVER%TRUE% Defensive programming - ERROR_REPORT macro called here. */
      if (msgNb != pBusCtx->txListMsgs[list])
      {
        /* if the messages transmitted are not as required */
        ERROR_REPORT(SW_ERROR, pBusCtx->bus, pBusCtx->txListMsgs[list], msgNb);
      }
      pBusCtx->curTxMsgs = msgNb;
      pBusCtx->txStarted = FALSE;
      result = E_COSTATUS_OK;
      break;
        
    case HdswExtCan_TxSts_Busy_E:
      pBusCtx->curTxMsgs = 0;
      result = E_COSTATUS_BUS_BUSY;
      /* keep txStarted set to TRUE */
      break;
    case HdswExtCan_TxSts_FcgExtCan_E:
      /* message number: not significant */
       pBusCtx->curTxMsgs = 0;
      result = E_COSTATUS_EXTCAN_ERROR;
      break;
     
    case HdswExtCan_TxSts_FcgActPm_E:
      /* message number unavailable */
      pBusCtx->curTxMsgs = 0;
      result = E_COSTATUS_ACTPM_ERROR;
      break;
      
    case HdswExtCan_TxSts_FcgAsw_E:
      /* no parameter is available */
      /* %COVER%STMT% Defensive programming - ERROR_REPORT macro called here. */    
      ERROR_REPORT(SW_ERROR, pBusCtx->bus, pBusCtx->chan, pBusCtx->curTxList);
      result = E_COSTATUS_PARAM_ERROR;
      break;
      
    default:
      /* %COVER%STMT% Defensive programming - ERROR_REPORT macro called here. */    
      pBusCtx->curTxMsgs = 0; /* not significant */
      ERROR_REPORT(SW_ERROR, canSts, 
                   pBusCtx->txDetails.Data[0], pBusCtx->txDetails.Data[1]);
      result = E_COSTATUS_PARAM_ERROR;
      break;
    }
    /* clear the current list */
    if (result != E_COSTATUS_BUS_BUSY)
    {
      /* only clear the list when finished or error */
      pBusCtx->txListMsgs[list] = 0;
    }
  }
  else
  {
    pBusCtx->rxWhileBusyTransmission = FALSE;
    result = E_COSTATUS_OK;
  }
  return result;
}

/***********************************************************************/
/**
 * @brief  CanBus_stop - update the message transmit context
 * 
 * @param[in,out]     pBusCtx The pointer to the bus context of a Bus
 * @param[in] busId   The bus identifier
 * @return - @link E_COSTATUS_OK @endlink or error code
 *         - @link E_COSTATUS_CAN_IF_ERROR @endlink
 *
 * 
 ***********************************************************************/ 
PRIVATE CoStatus CanBus_stop(CanBusCtx *pBusCtx, Uint busId)
{
  Bool go = TRUE;
  CoStatus result = E_COSTATUS_OK;
  Uint idx;
  Uint msgOffset;
  HdswExtCan_Sts_T canSts;
  
  /* if TX or RX is ongoing, stop them */
  if (pBusCtx->txStarted)
  {
    result = CanOpenBus_stopTxMsg (busId);
    /* %COVER%TRUE% Defensive programming - ERROR_REPORT macro called here. */    
    if (result != E_COSTATUS_OK)
    {
      ERROR_REPORT(SW_ERROR, pBusCtx->bus, pBusCtx->chan, result);
      result = E_COSTATUS_CAN_IF_ERROR;
      go = FALSE;
    }
  }
  
  if ((go) && (pBusCtx->rxStarted))
  {
    result = CanOpenBus_disableRxMsg(busId);
    /* %COVER%TRUE% Defensive programming - ERROR_REPORT macro called here. */
    if (result != E_COSTATUS_OK)
    {
      ERROR_REPORT(SW_ERROR, pBusCtx->bus, pBusCtx->chan, result);
      result = E_COSTATUS_CAN_IF_ERROR;
      go = FALSE;
    }
  }

  /* %COVER%FALSE% Defensive programming - ERROR_REPORT macro called previously. */
  if (go)
  {
    /* initialise the Tx messages flip/flop buffer addresses:
     * - allocated from global Tx message buffer pool for 2 lists
     * - aligned at 16-byte boundary
     * 
     */
    for (idx = 0; idx < CAN_IF_TX_LIST_NUM; idx++)
    {
      msgOffset = 1+(CAN_IF_TX_LIST_LEGNTH*(busId*2 + idx)); 
      pBusCtx->pTxList[idx] = (HdswExtCan_MsgTx_T *) (((Uint)&txMsgsPool[msgOffset]) & ADDR_16BYTE_BOUNDARY);
      pBusCtx->txListMsgs[idx] = 0;
    }
   
    /* initialise the Rx messages buffer address
     * - allocated from global Rx message buffer pool for the bus
     * - aligned at 1Kbyte boundary
     *
     */
    
    msgOffset = K_BYTE + (sizeof(HdswExtCan_MsgRx_T) *CAN_IF_RX_BUFFER_MSGS * busId); 
    pBusCtx->pRxBuffer = (HdswExtCan_MsgRx_T *) (((Uint)&rxMemPool[msgOffset]) & ADDR_1KBYTE_BOUNDARY);
    
    
    /* Re-initialise the CAN Bus Interface */
    canBusIf.restartIf (pBusCtx->bus, &canSts);

    /* %COVER%TRUE% Defensive programming - ERROR_REPORT macro called here. */
    if (canSts != HdswExtCan_Sts_Success_E)
    {
      ERROR_REPORT(SW_ERROR, pBusCtx->bus, pBusCtx->chan, canSts);
      result = E_COSTATUS_CAN_IF_ERROR;
    }
  }

  return result;
}
/*------------------ ooOoo End of file ooOoo --------------------------*/
