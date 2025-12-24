/* CanOpenAction.c - CanOpenAction body */

/***********************************************************************/
/**
 * @file
 * @brief CanOpenAction.c - implementation of CANopen State Machine 
 * transition actions.
 *  
 * This module implements the automaton's actions defined by the CANopen
 * manager transition tables. It includes also necessary data definitions
 * and structures to support the actions, including the House keeping
 * data area ( /link CoMgr_hkArea /endlink )for the two buses.
 *
 ***********************************************************************/

/**
 * @addtogroup coMgr
 * @{
 * @par CanOpenAction module
 * 
 * The CanOpenAction module in the CANopen manager realises the set of
 * functions corresponding to all actions defined in the transition table
 * for the bus manager. An action is invoked if the corresponding event
 * is raised when a bus manager is in a given state.
 * 
 * As an event for CANopen manager is always triggered by RTC timing event
 * (10Hz or 200 Hz), referred as cycle or slot. As result, the operations
 * performed in the actions implemented in this module defines the global
 * operation schedule of the CANopen manager.  
 * 
 * The messages received when the master is in INIT state are all discarded.
 * 
 * When the master is in PRE-POP, OP or STOP modes, the CANOpen protocols are
 * supported according to the CAN Open Standard and ROVER's tailoring, i.e
 * both received and transmitted CAN messages are handled following the involved
 * protocols.
 * 
 * Two other modules are used to support the operations of all "actions" in
 * the this module:
 * 
 * - CanOpenBus provides a set of services ensuring the CAN messages exchanges
 *   on the CAB Bus,
 * - CanOpenSdo implements a dedicated automaton for SDO protocol handling and
 *   a specific set of "actions" for this automaton.
 * 
 * Notice that excepting the while the Bus Manager is in INIT mode, all CAN messages
 * received in any 200Hz slot (slots 0 to 19) are handled, while the CAN messages
 * are not transmitted in the last time slot (slot 19). In addition, that messages
 * transmitted depend on the master's mode and time slot:
 *
 *  Slot 0: (cycle starting) 
 *    - SYNC message according to the communication cycle period configuration
 *      when the master is in PRE-OP or OP mode
 *    - Heart Beat message (including Boot Up message) according to the HB
 *      producer period,
 *    - Slave NMT commands (up to all registered commands, if transmission message
 *      list is available), 
 *    
 *  Slot 1:18
 *    - SDO transfer aborted message (not the slot 1) when the master is
 *      in PRE-OP or OP mode,
 *    - SDO Block download or SDO Block upload message when the master is in PRE-Op or OP mode,
 *        or
 *    - SDO expedited download or SDO expedited upload message when the master is in PRE-OP
 *      or OP mode when there is no SDO block transfer ongoing,
 *    - RET message to slave mode when the master is in OP mode (only in slot 14),
 *    - PDO TC message to slave mode, when the master is in OP mode (only in slot 14&16).
 *      The messages are  limited to (CO_PDO_TC_IN_CYC/2) in a slot, if transmission message
 *      list is available,
 *  Slot 19:
 *    - no message sending (with the exception of the SDO block transfer end message)
 * @}
 */

/*---------- Standard libraries includes ------------------------------*/
#include <libc.h>

/*---------- FSW includes ---------------------------------------------*/
#include "errorLib.h"

/*---------- Component includes ---------------------------------------*/
#include "coMgt/CanOpenAction.h"
#include "coMgt/CanOpenBus.h"
#include "coMgt/CanOpenSdo.h"

/*---------- Local defines & macro -----------------------
 * 
 * 
 * -------------*/

#define CO_HB_ST_UNKNOWN  (0xF0) /* invalid HB state, for test purpose */
#define CO_STATE_LIST_LENGTH (E_COSTATE_END - E_COSTATE_OFF +1)

#define CAN_ERR_STS(bit3, bit2, bit1, bit0) (((bit3)<<3) | ((bit2)<<2) | ((bit1)<<1) | (bit0))

#define CO_DUMMY_PDO_NID (96)
#define CO_DUMMY_PDO_COBID (CANOPEN_COB_RPDO1_BASE+CO_DUMMY_PDO_NID)

/*---------- Local types definitions ----------------------------------*/

/**
 * @brief CoTrans: state machine transition descriptor
 **/
 typedef struct CoTrans
{
    CoState  curState; /**< @brief current automaton state */
    CoEvt    evtIn;    /**< @brief input event */
    CoAction action;   /**< @brief corresponding 'action' to be performed */
} CoTrans;

/*---------- Definition of variables exported by the module -----------*/

/*---------- Declarations of local functions --------------------------*/

/* STATE MACHINE CALLBACK FUNCTIONS */ 
PRIVATE CoState CanOpenAction_off10Hz(CoCtx *pCtx);
PRIVATE CoState CanOpenAction_init10Hz(CoCtx *pCtx);
PRIVATE CoState CanOpenAction_stop10Hz(CoCtx *pCtx);
PRIVATE CoState CanOpenAction_preOp10Hz(CoCtx *pCtx);
PRIVATE CoState CanOpenAction_op10Hz(CoCtx *pCtx);
PRIVATE CoState CanOpenAction_init200Hz(CoCtx *pCtx);
PRIVATE CoState CanOpenAction_preOp200Hz(CoCtx *pCtx);
PRIVATE CoState CanOpenAction_op200Hz(CoCtx *pCtx);
PRIVATE CoState CanOpenAction_stop200Hz(CoCtx *pCtx);
PRIVATE CoState CanOpenAction_preOpStart(CoCtx *pCtx);
PRIVATE CoState CanOpenAction_preOpStop(CoCtx *pCtx);
PRIVATE CoState CanOpenAction_preOpReset(CoCtx *pCtx);
PRIVATE CoState CanOpenAction_preOpSwitchBus(CoCtx *pCtx);
PRIVATE CoState CanOpenAction_preOpTtoggle(CoCtx *pCtx);
PRIVATE CoState CanOpenAction_opEnterPreOp(CoCtx *pCtx);
PRIVATE CoState CanOpenAction_opStop(CoCtx *pCtx);
PRIVATE CoState CanOpenAction_opReset(CoCtx *pCtx);
PRIVATE CoState CanOpenAction_opSwitchBus(CoCtx *pCtx);
PRIVATE CoState CanOpenAction_opTtoggle(CoCtx *pCtx);
PRIVATE CoState CanOpenAction_stopEnterPreOp(CoCtx *pCtx);
PRIVATE CoState CanOpenAction_stopStart(CoCtx *pCtx);
PRIVATE CoState CanOpenAction_stopReset(CoCtx *pCtx);
PRIVATE CoState CanOpenAction_stopSwitchBus(CoCtx *pCtx);

/* Local FUNCTIONS */
PRIVATE CoTrans const *coTransGet(CoEvt event, CoState state);
PRIVATE CoEvt coEvtDecode (CoCtx *pCtx);
PRIVATE CoEvt coEvtFromNmtCs (Uint cmd);
PRIVATE void cobIdDescBuild(CoCtx *pCtx);
PRIVATE void syncWinEvtNotify(CoCtx *pCtx);
PRIVATE void busRestart(CoCtx *pCtx, Bool chanSwitch);
PRIVATE void busInMsgListDiscard(CoCtx *pCtx);
PRIVATE void cobMsgProcess(CoCtx *pCtx, CoState entryState, CoMsg *pCoMsg);
PRIVATE Uint busInMsgListProcess(CoCtx *pCtx, CoState entryState);
PRIVATE void cycleProcess(CoCtx *pCtx, CoState entryState);
PRIVATE void slotProcess(CoCtx *pCtx, CoState entryState);
PRIVATE void slaveCmdQueueHandler(CoCtx *pCtx);
PRIVATE void slaveCmdQueueClear(CoCtx *pCtx);
PRIVATE void pdoReqQueuesClear(CoCtx *pCtx);
PRIVATE void expdtSdoReqQueuesClear(CoCtx *pCtx);
PRIVATE void blockSdoReqQueuesClear(CoCtx *pCtx);
PRIVATE CoState msgsXmtCheck(CoCtx *pCtx);
PRIVATE void hbMgtHandler(CoCtx *pCtx, CoState entryState);
PRIVATE void retCycSetup(CoCtx *pCtx);
PRIVATE void retReqMsgsBuild(CoCtx *pCtx);
PRIVATE void pdoTcMsgsBuild(CoCtx *pCtx);
PRIVATE void hbMsgProcess(CoCtx *pCtx, CoState entryState, CoMsg *pCoMsg);
PRIVATE void pdoMsgProcess(CoCtx *pCtx, CoMsg *pCoMsg);
PRIVATE Uint coStateToHbState(CoState coState);
PRIVATE void getMissedHb(CoCtx *pCtx);
PRIVATE void processTxMsgList(CoCtx *pCtx, U32 list);


/*---------- Definition of local variables and constants --------------*/


/***********************************************************************/
/**
 * @brief pfNodeDesc: platform node descriptor
 
 * The contexts are create and initialised with configuration parameters
 * from CAN BUS IRD of each bus.
 * Note: the number of PDOs supported by a node are supposed as:
 * - 4 for TPDO, and
 * - 4 for RPDO
 *
 ***********************************************************************/
/* {{RELAX<GEN-005-M> False positive: all fields are correctly initialised by the macros */
PRIVATE NodeDesc pfNodeDesc[CO_PF_NODE_NUM] =
{
   PF_NODE01_DESC
   PF_NODE02_DESC
   PF_NODE03_DESC
   PF_NODE04_DESC
   PF_NODE05_DESC
   PF_NODE06_DESC
   PF_NODE07_DESC
   PF_NODE08_DESC
   PF_NODE09_DESC
   PF_NODE10_DESC
};

/* }}RELAX<GEN-005-M> */  

/***********************************************************************/
/**
 * @brief plNodeDesc: payload node descriptor
 
 * The contexts are create and initialised with configuration parameters
 * from CAN BUS IRD of each bus.
 * Note: the number of PDOs supported by a node are supposed as:
 * - 4 for TPDO, and
 * - 4 for RPDO
 *
 ***********************************************************************/
 /* {{RELAX<GEN-005-M> False positive: all fields are correctly initialised by the macros */
PRIVATE NodeDesc plNodeDesc[CO_PL_NODE_NUM] =
{
  PL_NODE01_DESC
  PL_NODE02_DESC
  PL_NODE03_DESC
  PL_NODE04_DESC
  PL_NODE05_DESC
  PL_NODE06_DESC
};
/* }}RELAX<GEN-005-M> */

/**
*/   
/* the offsets of RX (slave->master) messages */
PRIVATE U16 cobIdRxGroup[] =
{
  CANOPEN_COB_TPDO1_BASE,
  CANOPEN_COB_TPDO2_BASE,
  CANOPEN_COB_TPDO3_BASE,
  CANOPEN_COB_TPDO4_BASE,
  CANOPEN_COB_TSDO_BASE,
  CANOPEN_COB_HB_BASE
};


/* the offsets of TX (master->slave) messages */
PRIVATE U16 cobIdTxGroup[] =
{
  CANOPEN_COB_RPDO1_BASE,
  CANOPEN_COB_RPDO2_BASE,
  CANOPEN_COB_RPDO3_BASE,
  CANOPEN_COB_RPDO4_BASE,
  CANOPEN_COB_RSDO_BASE
};

#if (CO_PF_NODE_NUM > CO_SLV_NODES_MAX)
#error "Too many nodes on Platform bus"
#endif

#if (CO_PL_NODE_NUM > CO_SLV_NODES_MAX)
#error "Too many nodes on Payload bus"
#endif


/**
 * @brief Node State On platform bus
 *  
 * 
 */

/*
 * Node State On platform bus
 */
NodeStat CoMgr_pfNodeStat[CO_PF_NODE_NUM];

/**
 * @brief Node State On platform bus
 *  
 * 
 */

/*
 * Node State On payload bus
 */
NodeStat CoMgr_plNodeStat[CO_PL_NODE_NUM];


/*
 * Dummy PDO content
 */
PRIVATE U08 dummyPdo[CANOPEN_PDO_DATA_LENGTH_MAX] = {0, 0, 0, 0, 0, 0, 0, 0 };

/***********************************************************************/
/**
 * @brief pfPdoTm: platform node PDO TM Data pool
 * 
 * The storage areas for TPDO messages (PDO TM) on Platform Bus
 *
 */
/***********************************************************************/
PRIVATE PdoTmArea pfPdoTm[CO_PF_NODE_NUM*CO_PDO_TM_MAX_PER_NODE];

/***********************************************************************/
/**
 * @brief plPdoTm: payload node PDO TM Data pool
 * 
 * The storage areas for TPDO messages (PDO TM) on Payload Bus
 *
 */
/***********************************************************************/
PRIVATE PdoTmArea plPdoTm[CO_PL_NODE_NUM*CO_PDO_TM_MAX_PER_NODE];

/***********************************************************************/
/**
 * @brief pfSdoMsg: platform node SDO message Data pool
 * 
 * The storage areas for Expedited SDO messages on Platform Bus
 *
 */
/***********************************************************************/
PRIVATE SdoMsgArea pfSdoMsg[CO_PF_NODE_NUM];

/***********************************************************************/
/**
 * @brief plSdoMsg: payload node SDO message Data pool
 * 
 * The storage areas for Expedited SDO messages on Payload Bus
 *
 */
/***********************************************************************/
PRIVATE SdoMsgArea plSdoMsg[CO_PL_NODE_NUM];

/***********************************************************************/
/**
 * @brief plSdoMsg: payload node Block SDO TM Data pool
 * 
 * The storage areas for Block SDO TM on Platform bus
 *
 */
/***********************************************************************/
PRIVATE CoSdoBlkTmRecord pfSdoBlkQueue[CO_PF_SDO_BLK_TM_QUEUE_LENGTH];
PRIVATE SdoBlkTmArea pfSdoBlkTm =
{
  .written = 0,
  .read = 0,
  .blks = CO_PF_SDO_BLK_TM_QUEUE_LENGTH,
  .pBlkQueue = pfSdoBlkQueue
};

/***********************************************************************/
/**
 * @brief plSdoMsg: Payload node Block SDO TM Data pool
 * 
 * The storage areas for Block SDO TM on Payload bus
 *
 */
/***********************************************************************/

PRIVATE CoSdoBlkTmRecord plSdoBlkQueue[CO_PL_SDO_BLK_TM_QUEUE_LENGTH];
PRIVATE SdoBlkTmArea plSdoBlkTm =
{
  .written = 0,
  .read = 0,
  .blks = CO_PL_SDO_BLK_TM_QUEUE_LENGTH,
  .pBlkQueue = plSdoBlkQueue
};

/*
 * SDO Download request queues for platform & payload buses
 */
PRIVATE SdoBlkDlReq pfSdoBlkDlReq[CO_PF_BLK_SDO_DL_QUEUE_LENGTH];
PRIVATE SdoBlkDlReq plSdoBlkDlReq[CO_PL_BLK_SDO_DL_QUEUE_LENGTH];


/*---------- Definition of variables exported by the module -----------*/

/***********************************************************************/
/**
 * @brief CoMgr_defaultBus - CANopen default channel selection for two buses
 *
 * This table defines the initial active channel after a master switch-on Platform
 * and payload buses.
 * The default channel could be re-initialised before calling CanOpenMgt_Init() service
 *
 * - the selected channel should be CANBUS_NOM_CHAN (nominal) or CANBUS_RED_CHAN (redundant),
 * - the default selection is CANBUS_NOM_CHAN
 *
 * @requirements
 * - SRS.DMS.CAN.FUNC.1010 [Bdefault definition]
 * 
 ***********************************************************************/
Uint CoMgr_defaultBus[ROV_CANBUS_NUM] =
{
  CANBUS_NOM_CHAN, CANBUS_NOM_CHAN
};

/**
 * @brief canOpenTrans - CANopen State machine transition definition
 *
 * This table defines the CANopen Manager's state machine:
 * - the possible states,
 * - the possible and significant events for each state,
 * - the action for all events on all states
 * 
 * The next state of the machine at the end of a given action is provided by the action.
 *
 * This table is applicable for the 2 buses.
 *
 * 
 ***********************************************************************/
PRIVATE CoTrans const canOpenTrans[] = {
  /* E_COSTATE_OFF is transitory state, 2 events could happen
   * Event 10 Hz: CyclicSync event
   */
  {E_COSTATE_OFF, E_COEVT_10HZ, CanOpenAction_off10Hz},

  /* in Initialisation state, 2 events could happen
   * Event 10 Hz: CyclicSync event
   * Event 200 Hz: timer event
   */
  {E_COSTATE_INIT, E_COEVT_10HZ, CanOpenAction_init10Hz},
  {E_COSTATE_INIT, E_COEVT_200HZ, CanOpenAction_init200Hz}, 
  
  /* in Pre-Operational state, the following could happen
   * Event 10 Hz: CyclicSync event
   * Event 200 Hz: timer event
   * Event NMT command Enter PreOp: considered as Event 10 Hz
   * Event NMT command Start
   * Event NMT command Stop
   * Event NMT command Reset
   * Event Command SwitchBus
   * Event Ttoggle Timeout
   * TC Switch Bus
   */
  {E_COSTATE_PRE_OP, E_COEVT_10HZ, CanOpenAction_preOp10Hz},
  {E_COSTATE_PRE_OP, E_COEVT_200HZ, CanOpenAction_preOp200Hz},
  {E_COSTATE_PRE_OP, E_COEVT_ENTER_PRE_OP, CanOpenAction_preOp10Hz},
  {E_COSTATE_PRE_OP, E_COEVT_START, CanOpenAction_preOpStart},
  {E_COSTATE_PRE_OP, E_COEVT_STOP, CanOpenAction_preOpStop},
  {E_COSTATE_PRE_OP, E_COEVT_RESET, CanOpenAction_preOpReset},
  {E_COSTATE_PRE_OP, E_COEVT_SWITCH_BUS, CanOpenAction_preOpSwitchBus},
  {E_COSTATE_PRE_OP, E_COEVT_TT_TIMEOUT, CanOpenAction_preOpTtoggle},
    
  /* in Operational state, the following events could happen
   * Event 10 Hz: CyclicSync event
   * Event 200 Hz: timer event
   * Event NMT command Enter PreOp
   * Event NMT command Start: considered as Event 10 Hz
   * Event NMT command Stop
   * Event NMT command Reset
   * Event Command SwitchBus
   * Event Ttoggle Timeout
   */
  {E_COSTATE_OP, E_COEVT_10HZ, CanOpenAction_op10Hz},
  {E_COSTATE_OP, E_COEVT_200HZ, CanOpenAction_op200Hz},
  {E_COSTATE_OP, E_COEVT_ENTER_PRE_OP, CanOpenAction_opEnterPreOp},
  {E_COSTATE_OP, E_COEVT_START, CanOpenAction_op10Hz},
  {E_COSTATE_OP, E_COEVT_STOP, CanOpenAction_opStop},
  {E_COSTATE_OP, E_COEVT_RESET, CanOpenAction_opReset},
  {E_COSTATE_OP, E_COEVT_SWITCH_BUS, CanOpenAction_opSwitchBus},
  {E_COSTATE_OP, E_COEVT_TT_TIMEOUT, CanOpenAction_opTtoggle},
  
  /* in Stopped state, 6 events could happen
   * Event 10 Hz: CyclicSync event
   * Event 200 Hz: timer event
   * Event NMT command Enter PreOp
   * Event NMT command Start
   * Event NMT command Stop: considered as Event 10 Hz
   * Event NMT command Reset
   * TC Switch Bus
   */
  {E_COSTATE_STOP, E_COEVT_10HZ, CanOpenAction_stop10Hz},
  {E_COSTATE_STOP, E_COEVT_200HZ, CanOpenAction_stop200Hz},
  {E_COSTATE_STOP, E_COEVT_ENTER_PRE_OP, CanOpenAction_stopEnterPreOp},
  {E_COSTATE_STOP, E_COEVT_START, CanOpenAction_stopStart},
  {E_COSTATE_STOP, E_COEVT_STOP, CanOpenAction_stop10Hz},
  {E_COSTATE_STOP, E_COEVT_RESET, CanOpenAction_stopReset},
  {E_COSTATE_STOP, E_COEVT_SWITCH_BUS, CanOpenAction_stopSwitchBus},
     
  /** marker it as the end of transition table */
  {E_COSTATE_END, E_COEVT_END, NULL}

};

/**
 * @addtogroup coMgr
 * @{
 */

/***********************************************************************/
/**
 * @brief CoMgr_ctx platform & payload CAN Bus context descriptor
 * The contexts are created and initialised with the initial conditions
 * and configuration for the two CAN Buses
 *
 * @shared CANopen Bus context descriptor.
 * Protected by mutex @ref CoCtx.lock
 *
 *
 ***********************************************************************/
/* {{RELAX<GEN-005-M> only the specific bus configurations are initialised here; uninitialised data are initialised dynamically later at run-time in procedure CanOpenAction_setup. This is done before any access to the structure data */
CoCtx CoMgr_ctx[ROV_CANBUS_NUM] = 
{
  {
    /* manager identifier */
    .busName = "PLAT",
    .busId = CANBUS_ID_PF,
    
    /* nodes description list */
    .nodes = CO_PF_NODE_NUM,
    .pNodeList = pfNodeDesc,
    .pNodeStat = CoMgr_pfNodeStat,
    
    /* PDO TM storage area lists */
    .pPdoTmArea = pfPdoTm,

    /* SDO download request queue */
    .pSdoBlkDlReq = pfSdoBlkDlReq,
    .sdoDlQLength = CO_PF_BLK_SDO_DL_QUEUE_LENGTH,
    
    /* SDO expedited received messages queue list */
    .pSdoMsgArea = pfSdoMsg,
    
    /* SDO block TM queue */
    .pSdoBlkTmArea = &pfSdoBlkTm,
    
    /* system configuration parameters */
    .syncWinLength = CO_PF_SYNC_WIN_SLOT,
    .commSyncPeriod = CO_PF_COMM_CYCLE_PERIOD,
    .mstPrdHBPeriod = CO_PF_MST_PRD_HBT,
    .ttogglePeriod = CO_PF_MST_PRD_HBT * CO_PF_TOGGLE_CYC,
    .tresetPeriod = CO_PF_MST_PRD_HBT * (CO_PF_TOGGLE_CYC / 2 ),
    .sdoBlks = CO_PF_SDO_BLK_MAX_IN_CYCLE,
    .sdoBlkULStartTimerSlot[0] = CO_PF_SDO_START_TIMER_SLOT1,
    .sdoBlkULStartTimerSlot[1] = 0,
    .sdoBlkDLStartTimerSlot[0] = CO_PF_SDO_START_TIMER_SLOT1,
    .sdoBlkDLStartTimerSlot[1] = 0,
    .sdoBlkDlPdoTimeout = CO_PF_BLK_SDO_DL_PDO_TIMEOUT,
    
    .busChanIdSwitched = FALSE,
    .firstHbReceived = FALSE
  },
  {
    /* manager identifier */
    .busName = "PAYL",
    .busId = CANBUS_ID_PL,
    
    /* nodes description list */
    .nodes = CO_PL_NODE_NUM,
    .pNodeList = plNodeDesc,
    .pNodeStat = CoMgr_plNodeStat,
    
    /* PDO TM storage area */
    .pPdoTmArea = plPdoTm,
    
    /* SDO download request queue lists */
    .pSdoBlkDlReq = plSdoBlkDlReq,
    .sdoDlQLength = CO_PL_BLK_SDO_DL_QUEUE_LENGTH,
    
    /* SDO expedited received messages queue list */
    .pSdoMsgArea = plSdoMsg,

    /* SDO block TM queue */
    .pSdoBlkTmArea = &plSdoBlkTm,
    
    /* system configuration parameters */
    .syncWinLength = CO_PL_SYNC_WIN_SLOT,
    .commSyncPeriod = CO_PL_COMM_CYCLE_PERIOD,
    .mstPrdHBPeriod = CO_PL_MST_PRD_HBT,
    .ttogglePeriod = CO_PL_MST_PRD_HBT * CO_PL_TOGGLE_CYC,
    .tresetPeriod = CO_PL_MST_PRD_HBT * (CO_PL_TOGGLE_CYC / 2 ),
    .sdoBlks = CO_PL_SDO_BLK_MAX_IN_CYCLE,
    .sdoBlkULStartTimerSlot[0] = CO_PL_SDO_START_TIMER_SLOT1,
    .sdoBlkULStartTimerSlot[1] = CO_PL_SDO_START_TIMER_SLOT2, 
    .sdoBlkDLStartTimerSlot[0] = CO_PL_SDO_START_TIMER_SLOT1,
    .sdoBlkDLStartTimerSlot[1] = CO_PL_SDO_START_TIMER_SLOT2, 
    .sdoBlkDlPdoTimeout = CO_PL_BLK_SDO_DL_PDO_TIMEOUT,
    
    .busChanIdSwitched = FALSE,
    .firstHbReceived = FALSE
  }
};
/* }}RELAX<GEN-005-M> */

/* list of descriptor of all possible CobID for each bus */
CobIdDesc CoMgr_cobIdList[ROV_CANBUS_NUM][CANOPEN_NODE_MAX+1];

/** @db HK Data pool (HK) for the two buses */
CoBusHkArea CoMgr_hkArea[ROV_CANBUS_NUM];

/** @} */


/*------------------ ooOoo Inline functions (if any) ooOoo ------------*/

/*------------------ ooOoo Global functions ooOoo ---------------------*/

/***********************************************************************/
/**
 * @brief CanOpenAction_setup - setup automaton start-up context
 *
 * This function sets up the initial context for a Bus manager.
 * It is to be called once and only once before starting a Bus manager.
 *
 * @param[in,out] pCtx  The pointer to the context of a CAN Bus Manager.
 * @param[in] evtPid    Application Process ID to be used when generating events
 * @return      
 *  - \@link E_COSTATUS_OK \@endlink
 *  - \@link E_COSTATUS_ERROR \@endlink
 * 
 ***********************************************************************/ 
CoStatus CanOpenAction_setup(CoCtx *pCtx, U16 evtPid) 
{
  Uint chan;
  U08 idx;
  CoBusHkArea *pHk;
  rtems_name      semName;
  rtems_status_code rtStatus;
  CoState coSts;
  
  /* create context protection lock */
  (void) ResourceLock_init(&pCtx->lock, pCtx->busName);
    
  /* create cycle synchronisation semaphore */
  semName   = rtems_build_name('C', 'Y', 'C', 'S'+pCtx->busId);
  
  rtStatus = rtems_semaphore_create(semName, 0,
                               RTEMS_PRIORITY|RTEMS_SIMPLE_BINARY_SEMAPHORE,
                               0, &pCtx->cycSyncSem);
  
  /* %COVER%TRUE% Defensive programming - ERROR_REPORT macro called here. */
  if (rtStatus != RTEMS_SUCCESSFUL)
  {
    ERROR_REPORT(SW_ERROR, rtStatus, pCtx->busId, 0);
  }
  
  /* Initialise Application Process ID used for events generation */
  pCtx->evtPid = evtPid;
  
  /* initialise other context parameters */
  pCtx->curState = E_COSTATE_OFF;
  pCtx->lastState = E_COSTATE_OFF;
  pCtx->lastEvt = E_COEVT_IGNORED;
  pCtx->nTrans = 0;
  pCtx->nEvt = 0;
  pCtx->ctoggle = 0;
  
  pCtx->cycles = 0; /* i.e. not started */
  pCtx->slotInCycle = 0; 
  pCtx->slots = 0;
  pCtx->itInCycle = 0;
  
  pCtx->actPmHealth  = E_CO_PM_HEALTH_SAFE;
  pCtx->extCanHealth = E_CO_EXTCAN_HEALTH_SAFE;
  
  /* channel Switch flag set to FALSE */
  pCtx->busChanIdSwitched = FALSE;
  
  /* tm/tc counters */
  pCtx->tcCount = 0;
  pCtx->tmCount = 0;
  
  /* master/slave command indexes */
  pCtx->mstCmdIn = 0;
  pCtx->mstCmdOut = 0;
  pCtx->slvCmdIn = 0;
  pCtx->slvCmdOut = 0;
  
  pCtx->txMsgLists = 0;
  pCtx->txMsgs = 0;
  pCtx->msgsInCurList = 0;
  pCtx->rxMsgs = 0;
  
  pCtx->registeredTaskNum = 0;
  
  pCtx->hbCountUp = 0;
  pCtx->commSyncCountUp = 0;
  pCtx->commSyncEna = FALSE;
  pCtx->lastHbCycle = 0;
  
  pCtx->cucC32B = 0;
  pCtx->cucF16B = 0;
  pCtx->cucC32BNextSyncCycle = 0;
  pCtx->cucF16BNextSyncCycle = 0;
  
  /* initialise node state list */ 
  memset (pCtx->pNodeStat, 0, sizeof(NodeStat) * pCtx->nodes);

  /* set up sdo related context */
  CanOpenSdo_setup(pCtx);
  
  pCtx->pdoEna = FALSE;

  pCtx->retIn = 0;
  pCtx->retOut = 0;
  pCtx->pdoTcIn = 0;
  pCtx->pdoTcOut = 0;
  
  /* messages */
  pCtx->hbMsgsIn = 0;
  pCtx->hbMsgsOut = 0;
  pCtx->nmtMsgs = 0; 
  pCtx->syncMsgs = 0;
  pCtx->retMsgs = 0;
  pCtx->pdoMsgsIn = 0;
  pCtx->pdoMsgsOut = 0; 
  pCtx->sdoMsgsIn = 0;
  pCtx->sdoMsgsOut = 0;
  
  chan = CANBUS_NOM_CHAN;
  if (CoMgr_defaultBus[pCtx->busId] == CANBUS_RED_CHAN)
  {
    chan = CANBUS_RED_CHAN;
  }
  pCtx->chanId = chan;
  pCtx->healthStatus = E_CO_NODE_HEALTH_SAFE;
  
  /* reset Data Pool area */
  pHk = &CoMgr_hkArea[pCtx->busId];
  memset (pHk, 0, sizeof(CoBusHkArea));
  
  /* initialisation of node list dependent info in Ctx and data pool:
   * - nodeSwitchOnMsk
   * - nodeHbRegMsk
   * - pNodeStat list
   * - baseNodeId to nodeListIdx table,
   * - CobID descriptor list 
   * - DP: nodes
   * - DP->node: nodeId
   */

  pCtx->nodeHbRegMsk = 0;
  pCtx->nodeSwitchOnMsk = 0;
 
  /* set the default value before initialisation */
  for (idx = 0; idx < CO_NODE_LIST_LENGTH; idx++)
  {
    pCtx->nodeToBusNodeIdx[idx] = CO_SLV_NODES_MAX;
  }
  
  pHk->nodes = pCtx->nodes; /* HK Area */
  
  /* Build up CobId descriptor list */
  cobIdDescBuild(pCtx);
  
  /* clean-up pdoTm/SdoMsg area */ 
  memset (pCtx->pPdoTmArea, 0, sizeof(PdoTmArea)*pCtx->nodes*CO_PDO_TM_MAX_PER_NODE);
  memset (pCtx->pSdoMsgArea, 0, sizeof(SdoMsgArea)*pCtx->nodes);
  
  pCtx->ttogglePeriodInc = 0;
  pCtx->nodeHbRegMsk = 0;
  pCtx->ctoggle = 0;
  pCtx->ttoggleTimer = pCtx->ttogglePeriod;
  pCtx->tresetTimer = pCtx->tresetPeriod;
  pCtx->tresetTriggeredInOp = FALSE;
  
  pCtx->firstHbReceived = FALSE;
 
#ifdef COMGT_UT_SUPPORT
  
  for (idx = 0; idx < pCtx->nodes; idx++)
  {
    U08  baseNid;
    baseNid = pCtx->pNodeList[idx].baseNid;
    if (pCtx->busId ==0)
    {
      pCtx->pNodeList[idx].cobIdRet = baseNid | CANOPEN_COB_RPDO2_BASE;
    }
    else
    {
      pCtx->pNodeList[idx].cobIdRet = baseNid | CANOPEN_COB_RPDO3_BASE;
    }
  }
#endif
  
  /* CAN bus default setup */
  coSts = CanOpenBus_setup(pCtx->busId);
  /* %COVER%TRUE% Defensive programming - ERROR_REPORT macro called here. */  
  if (coSts != E_COSTATUS_OK)
  {
    ERROR_REPORT(SW_ERROR, pCtx->busId, coSts, 0);
  }
  
  return coSts;
}

/***********************************************************************/
/**
 * @brief CanOpenAction_off10Hz - action to perform upon a 10Hz event
 * reception when CANopen manager is in OFF state.
 * 
 * @param[in,out] pCtx  The pointer to the context of a CAN Bus Manager.
 * @return     CoState  The next state of the CAN Bus Manager
 *
 * 
 ***********************************************************************/ 
CoState CanOpenAction_off10Hz(CoCtx *pCtx) 
{
  CoState entryState = E_COSTATE_INIT;
  /* re-start the interface with pre-selected default channel */
  busRestart(pCtx, FALSE);
  
  return entryState;
}

/***********************************************************************/
/**
 * @brief CanOpenAction_init10Hz - action to perform upon a 10 Hz event
 * reception when CANopen manager is in INIT state.
 * 
 * @param[in,out] pCtx  The pointer to the context of a CAN Bus Manager.
 * @return     CoState  The next state of the CAN Bus Manager
 *
 requests */
  expdtSdoReqQueuesClear(pCtx);
  blockSdoReqQueuesClear(pCtx);

  /* Master Redundancy management context:
   * - restart Ttoggle timer,
   * - reset health status
   */
  pCtx->ttoggleTimer = pCtx->ttogglePeriod + pCtx->ttogglePeriodInc ;
  pCtx->tresetTimer = pCtx->tresetPeriod + pCtx->ttogglePeriodInc ;
  pCtx->healthStatus = E_CO_NODE_HEALTH_SAFE;
  
  /* enable message RX */
  coSts = CanOpenBus_enableRxMsg(pCtx->busId);
  
  /* %COVER%TRUE% Defensive programming - ERROR_REPORT macro called here. */  
  if (coSts != E_COSTATUS_OK)
  {
    ERROR_REPORT(SW_ERROR, pCtx->busId, coSts, 0);
  }
  
  cycleProcess(pCtx, entryState);
  
  return entryState;
}

/***********************************************************************/
/**
 * @brief CanOpenAction_init200Hz - action to perform upon a 200 Hz event
 * reception when CANopen manager is in INIT state.
 * 
 * It clears the queues of received CAN message and NMT request for slave
 * 
 * @param[in,out] pCtx  The pointer to the context of a CAN Bus Manager.
 * @return     CoState  The next state of the CAN Bus Manager
 *
 ***********************************************************************/ 
CoState CanOpenAction_init200Hz(CoCtx *pCtx) 
{
  CoState entryState = E_COSTATE_INIT;
  
  /* retrieve any incoming commands, messages then discard them */
  if(pCtx->itInCycle%2 == 0)
  {
    busInMsgListDiscard(pCtx);
    slaveCmdQueueClear(pCtx);
  }
  
  return entryState;
}

/***********************************************************************/
/**
 * @brief CanOpenAction_preOp10Hz - action to perform upon a 10 Hz event
 * reception when CANopen manager is in PRE-OPERATIONAL state.
 * 
 * @param[in,out] pCtx  The pointer to the context of a CAN Bus Manager.
 * @return     CoState  The next state of the CAN Bus Manager
 *
 ***********************************************************************/ 
CoState CanOpenAction_preOp10Hz(CoCtx *pCtx)
{
  CoState entryState = E_COSTATE_PRE_OP;
  
  cycleProcess(pCtx, entryState);
  
  return entryState; 
}

/***********************************************************************/
/**
 * @brief CanOpenAction_preOp200Hz - action to perform upon a 200 Hz event
 * reception when CANopen manager is in PRE-OPERATIONAL state.
 * 
 * @param[in,out] pCtx  The pointer to the context of a CAN Bus Manager.
 * @return     CoState  The next state of the CAN Bus Manager
 *
 ***********************************************************************/ 
CoState CanOpenAction_preOp200Hz(CoCtx *pCtx) 
{
  CoState entryState = E_COSTATE_PRE_OP;
  
  slotProcess(pCtx, entryState);

  return entryState;
}

/***********************************************************************/
/**
 * @brief CanOpenAction_preOpStart - action to perform upon a Start event
 * reception when CANopen manager is in PRE-OPERATIONAL state.
 * 
 * @param[in,out] pCtx  The pointer to the context of a CAN Bus Manager.
 * @return     CoState  The next state of the CAN Bus Manager
 *
 ***********************************************************************/ 
CoState CanOpenAction_preOpStart(CoCtx *pCtx) 
{
  CoState entryState = E_COSTATE_OP;
  CoBusHkArea *pHk;
  
  pHk = &CoMgr_hkArea[pCtx->busId];
  /* FDIR update, reset flags */
  pCtx->hasOpStateExited = FALSE;
  pHk->canHbFc = FALSE;
  pHk->canTotHbF = FALSE;
  
  /* PDO:
   * pdoEna: TRUE => Enabled
   */
  pCtx->pdoEna = TRUE;
  
  /* SDO block transfer context clean-up */
  pCtx->blkSdoUlNode = 0;
  pCtx->blkSdoDlNode = 0; 
  
  /* discard submitted PDO TC */
  pdoReqQueuesClear(pCtx);
  
  cycleProcess(pCtx, entryState);
  return entryState;
}

/***********************************************************************/
/**
 * @brief CanOpenAction_preOpStop - action to perform upon a Stop event
 * reception when CANopen manager is in PRE-OPERATIONAL state.
 * 
 * @param[in,out] pCtx  The pointer to the context of a CAN Bus Manager.
 * @return     CoState  The next state of the CAN Bus Manager
 *
 * 
 ***********************************************************************/ 
CoState CanOpenAction_preOpStop(CoCtx *pCtx) 
{
  CoState entryState = E_COSTATE_STOP;

  /* SYNC:
   * commSyncEna: FALSE ==> Disable
   */
  pCtx->commSyncEna = FALSE;

  /* SDO:
   * sdoEna: FALSE => Disabled
   */
  pCtx->sdoEna = FALSE;
 
  /* release timer */
  pCtx->ttoggleTimer = pCtx->ttogglePeriod + pCtx->ttogglePeriodInc;
  pCtx->tresetTimer = pCtx->tresetPeriod + pCtx->ttogglePeriodInc;
  
  cycleProcess(pCtx, entryState);

  return entryState;
}

/***********************************************************************/
/**
 * @brief CanOpenAction_preOpReset - action to perform upon a Reset event
 * reception when CANopen manager is in PRE-OPERATIONAL state.
 * 
 * @param[in,out] pCtx  The pointer to the context of a CAN Bus Manager.
 * @return     CoState  The next state of the CAN Bus Manager
 * 
 ***********************************************************************/ 
CoState CanOpenAction_preOpReset(CoCtx *pCtx) 
{
  CoState entryState = E_COSTATE_INIT;
  
  /* re-start the interface with current channel */
  busRestart(pCtx, FALSE);
  
  /* SYNC:
   * commSyncEna: FALSE ==> Disable
   */
  pCtx->commSyncEna = FALSE;

  /* SDO:
   * sdoEna: FALSE => Disabled
   */
  pCtx->sdoEna = FALSE;

  return entryState;
}

/***********************************************************************/
/**
 * @brief CanOpenAction_preOpSwitchBus - action to perform upon a SwitchBus
 * event reception when CANopen manager is in PRE-OPERATIONAL state.
 * 
 * @param[in,out] pCtx  The pointer to the context of a CAN Bus Manager.
 * @return     CoState  The next state of the CAN Bus Manager
 *
 ***********************************************************************/ 
CoState CanOpenAction_preOpSwitchBus(CoCtx *pCtx) 
{
  CoState entryState = E_COSTATE_INIT;
  
  /* restart the bus with opposite channel */
  busRestart(pCtx, TRUE);
  
  return entryState;
}

/***********************************************************************/
/**
 * @brief CanOpenAction_preOpTtoggle - action to perform upon a Ttoggle
 * event reception when CANopen manager is in PRE-OPERATIONAL state.
 * 
 * @param[in,out] pCtx  The pointer to the context of a CAN Bus Manager.
 * @return     CoState  The next state of the CAN Bus Manager
 * 
 * 
 ***********************************************************************/ 
CoState CanOpenAction_preOpTtoggle(CoCtx *pCtx) 
{
  CoState entryState = E_COSTATE_PRE_OP;
  Uint idx;
  U32 missedNodeMsk;
  Bool found = FALSE;
  
  /* get missed HB nodes */
  missedNodeMsk = pCtx->nodeSwitchOnMsk ^ pCtx->nodeHbRegMsk;
  for (idx = 0; (idx < CO_SLV_NODES_MAX); idx++)
  {
    if ((missedNodeMsk & (1<<idx)) != 0)
    {
      if (found == FALSE)
      {
        /* record one of missed HB node */
        pCtx->toggleNode = pCtx->pNodeList[idx].baseNid;
        pCtx->toggleCycle = pCtx->cycles;
        found = TRUE;
      }
    }
  }
  pCtx->healthStatus = E_CO_NODE_HEALTH_FAIL;
 
  cycleProcess(pCtx, entryState);
  return entryState;
}

/***********************************************************************/
/**
 * @brief CanOpenAction_op10Hz - action to perform upon a 10 Hz event
 * reception when CANopen manager is in OPERATIONAL state.
 * 
 * @param[in,out] pCtx  The pointer to the context of a CAN Bus Manager.
 * @return     CoState  The next state of the CAN Bus Manager
 *
 ***********************************************************************/ 
CoState CanOpenAction_op10Hz(CoCtx *pCtx)
{
  CoState entryState = E_COSTATE_OP;
  
  cycleProcess(pCtx, entryState);
  return entryState; 
}

/***********************************************************************/
/**
 * @brief CanOpenAction_op200Hz - action to perform upon a 200 Hz event
 * reception when CANopen manager is in OPERATIONAL state.
 * 
 * @param[in,out] pCtx  The pointer to the context of a CAN Bus Manager.
 * @return     CoState  The next state of the CAN Bus Manager
 *
 * 
 ***********************************************************************/ 
CoState CanOpenAction_op200Hz(CoCtx *pCtx) 
{
  CoState entryState = E_COSTATE_OP;
  
  /* wake up any tasks waiting for PDO Synchronisation Window */
  if ((pCtx->syncWinLength == pCtx->slotInCycle) && ((pCtx->itInCycle%2) == 0))
  {
    syncWinEvtNotify (pCtx);
  }
  
  slotProcess(pCtx, entryState);
  
  return entryState;
}

/***********************************************************************/
/**
 * @brief CanOpenAction_opEnterPreOp - action to perform upon an EnterPreOp
 * event reception when CANopen manager is in OPERATIONAL state.
 * 
 * @param[in,out] pCtx  The pointer to the context of a CAN Bus Manager.
 * @return     CoState  The next state of the CAN Bus Manager
 *
 * 
 ***********************************************************************/ 
CoState CanOpenAction_opEnterPreOp(CoCtx *pCtx) 
{
  CoState entryState = E_COSTATE_PRE_OP;
 
  /* PDO:
   * pdoEna: FALSE => Disabled
   */
  pCtx->pdoEna = FALSE;
  
  cycleProcess(pCtx, entryState);
  
  /* exiting: OP state => wake up any tasks blocked on PDO Synchronisation Window */
  syncWinEvtNotify (pCtx);
  
  /* FDIR update */
  pCtx->hasOpStateExited = TRUE;
    
  return entryState;
}

/***********************************************************************/
/**
 * @brief CanOpenAction_opStop - action to perform upon a Stop event
 * reception when CANopen manager is in OPERATIONAL state.
 * 
 * @param[in,out] pCtx  The pointer to the context of a CAN Bus Manager.
 * @return     CoState  The next state of the CAN Bus Manager
 * 
 * 
 ***********************************************************************/ 
CoState CanOpenAction_opStop(CoCtx *pCtx) 
{
  CoState entryState = E_COSTATE_STOP;
  
  /* SYNC:
   * commSyncEna: FALSE ==> Disable
   */
  pCtx->commSyncEna = FALSE;

  /* SDO:
   * sdoEna: FALSE => Disabled
   */
  pCtx->sdoEna = FALSE;

  /* PDO:
   * pdoEna: FALSE => Disabled
   */
  pCtx->pdoEna = FALSE;

  /* FDIR update */
  pCtx->hasOpStateExited = TRUE;
  
  /* release timer */
  pCtx->ttoggleTimer = pCtx->ttogglePeriod + pCtx->ttogglePeriodInc;
  pCtx->tresetTimer = pCtx->tresetPeriod + pCtx->ttogglePeriodInc;

  cycleProcess(pCtx, entryState);
  
  /* exiting: OP state => wake up any tasks blocked on PDO Synchronisation Window */
  syncWinEvtNotify (pCtx);
  
  return entryState;
}

/***********************************************************************/
/**
 * @brief CanOpenAction_opReset - action to perform upon a Reset event
 * reception when CANopen manager is in OPERATIONAL state.
 * 
 * @param[in,out] pCtx  The pointer to the context of a CAN Bus Manager.
 * @return     CoState  The next state of the CAN Bus Manager
 * 
 * 
 ***********************************************************************/ 
CoState CanOpenAction_opReset(CoCtx *pCtx) 
{
  CoState entryState = E_COSTATE_INIT;
  
  /* FDIR update */
  pCtx->hasOpStateExited = TRUE;
  
  /* re-start the interface with current channel */
  busRestart(pCtx, FALSE);
  
  /* exiting: OP state => wake up any tasks blocked on PDO Synchronisation Window */
  syncWinEvtNotify (pCtx);
  
  return entryState;
}

/***********************************************************************/
/**
 * @brief CanOpenAction_opSwitchBus - action to perform upon a SwitchBus
 * event reception when CANopen manager is in OPERATIONAL state.
 * 
 * @param[in,out] pCtx  The pointer to the context of a CAN Bus Manager.
 * @return     CoState  The next state of the CAN Bus Manager
 *
 ***********************************************************************/ 
CoState CanOpenAction_opSwitchBus(CoCtx *pCtx) 
{
  CoState entryState = E_COSTATE_INIT;
  
  /* FDIR update */
  pCtx->hasOpStateExited = TRUE;
  
  /* restart the bus with opposite channel */
  busRestart(pCtx, TRUE);
  
  /* exiting: OP state => wake up any tasks blocked on PDO Synchronisation Window */
  syncWinEvtNotify (pCtx);
  
  return entryState;
}

/***********************************************************************/
/**
 * @brief CanOpenAction_opTtoggle - action to perform upon a Ttoggle
 * event reception when CANopen manager is in OPERATIONAL state.
 * 
 * @param[in,out] pCtx  The pointer to the context of a CAN Bus Manager.
 * @return     CoState  The next state of the CAN Bus Manager
 * 
 *
 ***********************************************************************/ 
CoState CanOpenAction_opTtoggle(CoCtx *pCtx) 
{
  Uint idx;
  CoBusHkArea *pHk;
  CoNodeInfo *pNode;
  CoState entryState = E_COSTATE_PRE_OP;
  
  pHk = &CoMgr_hkArea[pCtx->busId];
  
  /* PDO:
   * pdoEna: FALSE => Disabled
   */
  pCtx->pdoEna = FALSE;
  
  /* get missed HB nodes */
  getMissedHb(pCtx);
  
  /* HK update for FDIR */
  pHk->canHbFc   = 0;
  
  for (idx = 0; idx < pCtx->nodes; idx++)
  {
    pNode = &pHk->nodeInfo[idx];
    pNode->canHbFr = ( ((pCtx->nodeSwitchOnMsk & (1<<idx)) != 0) && ((pCtx->nodeHbRegMsk & (1<<idx)) == 0) );
    if (pNode->canHbFr)
    {
      pHk->canHbFc++;
    }
  }  

  /* FDIR: check if all expected HB, i.e all nodes have failed (according to the mask), with at least one expected because on */
  pHk->canTotHbF = (((pCtx->nodeHbRegMsk & pCtx->nodeSwitchOnMsk) == 0) && (pCtx->nodeSwitchOnMsk != 0));
  
  pCtx->healthStatus = E_CO_NODE_HEALTH_FAIL;  
    
  cycleProcess(pCtx, entryState);
  
  /* exiting: OP state => wake up any tasks blocked on PDO Synchronisation Window */
  syncWinEvtNotify (pCtx);
  
  /* FDIR update */
  pCtx->hasOpStateExited = TRUE;
  
  return entryState;
}

/***********************************************************************/
/**
 * @brief CanOpenAction_stop10Hz - action to perform upon a 10 Hz event
 * reception when CANopen manager is in STOPPED state.
 * 
 * @param[in,out] pCtx  The pointer to the context of a CAN Bus Manager.
 * @return     CoState  The next state of the CAN Bus Manager
 *
 ***********************************************************************/ 
 
CoState CanOpenAction_stop10Hz(CoCtx *pCtx)
{
  CoState entryState = E_COSTATE_STOP;
  
  /* FDIR update */
  pCtx->hasOpStateExited = TRUE;
  
  /* discard submitted expedited & block SDO requests when disabled,
   * to avoid queue overflow */
  expdtSdoReqQueuesClear(pCtx);
  blockSdoReqQueuesClear(pCtx);

  cycleProcess(pCtx, entryState);
  return entryState; 
}

/***********************************************************************/
/**
 * @brief CanOpenAction_stop200Hz - action to perform upon a 200 Hz event
 * reception when CANopen manager is in STOPPED state.
 * 
 * @param[in,out] pCtx  The pointer to the context of a CAN Bus Manager.
 * @return     CoState  The next state of the CAN Bus Manager
 *
 ***********************************************************************/ 
CoState CanOpenAction_stop200Hz(CoCtx *pCtx) 
{
  CoState entryState = E_COSTATE_STOP;
  
  slotProcess(pCtx, entryState);
  return entryState;
}

/***********************************************************************/
/**
 * @brief CanOpenAction_stopEnterPreOp - action to perform upon an EnterPreOp
 * event reception when CANopen manager is in STOPPED state.
 * 
 * @param[in,out] pCtx  The pointer to the context of a CAN Bus Manager.
 * @return     CoState  The next state of the CAN Bus Manager
 *
 * 
 ***********************************************************************/ 
CoState CanOpenAction_stopEnterPreOp(CoCtx *pCtx) 
{
  CoState entryState = E_COSTATE_PRE_OP;
  
  /* SYNC:
   * commSyncEna: TRUE ==> Enabled
   * commSyncCountUp: 0 => Sending immediately
   */
  pCtx->commSyncEna = TRUE;
  pCtx->commSyncCountUp = 0;
  
  /* SDO:
   * sdoEna: TRUE => Enabled
   */
  pCtx->sdoEna = TRUE;
  pCtx->expdtSdoNode = 0;

  /* discard submitted expedited & block SDO requests */
  expdtSdoReqQueuesClear(pCtx);
  blockSdoReqQueuesClear(pCtx);

  cycleProcess(pCtx, entryState);
  return entryState;
}

/***********************************************************************/
/**
 * @brief CanOpenAction_stopStart - action to perform upon a Start event
 * reception when CANopen manager is in STOPPED state.
 * 
 * @param[in,out] pCtx  The pointer to the context of a CAN Bus Manager.
 * @return     CoState  The next state of the CAN Bus Manager
 *
 *
 ***********************************************************************/ 
CoState CanOpenAction_stopStart(CoCtx *pCtx) 
{
  CoState entryState = E_COSTATE_OP;
  CoBusHkArea *pHk;
  
  pHk = &CoMgr_hkArea[pCtx->busId];  
   
  /* SYNC:
   * commSyncEna: TRUE ==> Enabled
   * commSyncCountUp: 0 => Sending immediately
   */
  pCtx->commSyncEna = TRUE;
  pCtx->commSyncCountUp = 0;
  
  /* SDO:
   * sdoEna: TRUE => Enabled
   */
  pCtx->sdoEna = TRUE;
  pCtx->expdtSdoNode = 0;

  /* discard submitted expedited & block SDO requests */
  expdtSdoReqQueuesClear(pCtx);
  blockSdoReqQueuesClear(pCtx);
  
  /* PDO:
   * pdoEna: TRUE => Enabled
   */
  pCtx->pdoEna = TRUE;

  /* SDO block transfer context clean-up */
  pCtx->blkSdoUlNode = 0;
  pCtx->blkSdoDlNode = 0;
  
  /* discard submitted PDO TC */
  pdoReqQueuesClear(pCtx);
  
  /* FDIR update */
  pCtx->hasOpStateExited = FALSE;
  pHk->canHbFc = FALSE;
  pHk->canTotHbF = FALSE;
  
  cycleProcess(pCtx, entryState);
  return entryState;
}

/***********************************************************************/
/**
 * @brief CanOpenAction_stopReset - action to perform upon a Reset event
 * reception when CANopen manager is in STOPPED state.
 * 
 * @param[in,out] pCtx  The pointer to the context of a CAN Bus Manager.
 * @return     CoState  The next state of the CAN Bus Manager
 *
 * 
 ***********************************************************************/ 
CoState CanOpenAction_stopReset(CoCtx *pCtx) 
{
  CoState entryState = E_COSTATE_INIT;
  
  /* re-start the interface with current channel */
  busRestart(pCtx, FALSE);
  
  return entryState;
}

/***********************************************************************/
/**
 * @brief  CanOpenAction_stopSwitchBus - action to perform upon a SwitchBus
 * event reception when CANopen manager is in STOPPED state.
 * 
 * @param[in,out] pCtx  The pointer to the context of a CAN Bus Manager.
 * @return     CoState  The next state of the CAN Bus Manager
 *
 ***********************************************************************/ 
CoState CanOpenAction_stopSwitchBus(CoCtx *pCtx) 
{
  CoState entryState = E_COSTATE_INIT;
  
  /* restart the bus with opposite channel */
  busRestart(pCtx, TRUE);
  
  return entryState;
}

/***********************************************************************/
/**
 * @brief  CanOpenAction_updateHkArea - update the informations in HK area according
 * to Bus manager context, dedicated to be called at the end of cycle event
 * 
 * @param[in,out] pCtx    The pointer to the context of a CAN Bus Manager.
 * @param[in] entryState  The Bus Manager's state.
 * @return N/A
 *
 * 
 ***********************************************************************/ 
void CanOpenAction_updateHkArea(CoCtx *pCtx, CoState entryState)
{
  CoBusHkArea *pHk;
  CoNodeInfo *pNode;
  NodeStat *pStat;
  U08 idx;

  pHk = &CoMgr_hkArea[pCtx->busId];
  
  /* the following fields are updated:
   * cycle,
   * cucTime;
   * Ctoggle,
   * activeSide,
   * busState,
   * health status,
   * hbRegMsk,
   * switchMsk
   */
  
  /* cycle */
  pHk->cycle = pCtx->cycles;
  pHk->busStateTrans = pCtx->nTrans;
  
  /* cucTime */
  pHk->cucCoarse32B = pCtx->cucC32B;
  pHk->cucFine16B   = pCtx->cucF16B;

  pHk->actPmHealth  = pCtx->actPmHealth;
  pHk->extCanHealth = pCtx->extCanHealth;
  
  /* Ctoggle */
  pHk->ctoggle = pCtx->ctoggle;
  /* activeSide */
  if (pCtx->chanId == CANBUS_NOM_CHAN)
  {
    pHk->activeSide = CO_BUS_NOM;
  }
  else
  {
    pHk->activeSide = CO_BUS_RED;
  }
  /* busState */
  pHk->busState = entryState;
  /* health status */
  pHk->healthStatus = pCtx->healthStatus;
  
  /* hbRegMsk */
  pHk->hbRegMsk = pCtx->nodeHbRegMsk;
  /* switchMsk */
  pHk->switchMsk = pCtx->nodeSwitchOnMsk;
  pHk->lastHbCycle = pCtx->lastHbCycle;
  
  pHk->canStat = pCtx->canStat;
  
  pHk->toggleNode   = pCtx->toggleNode;
  pHk->uhfHailNode  = pCtx->uhfHailNode;
  pHk->unexpHbNode  = pCtx->unexpHbNode;
  pHk->toggleCycle  = pCtx->toggleCycle;
  pHk->uhfHailCycle = pCtx->uhfHailCycle;
  pHk->unexpHbCycle = pCtx->unexpHbCycle;
  
  /* traffic statistics */
  pHk->canTxMsgs  = pCtx->txMsgs;
  pHk->canRxMsgs  = pCtx->rxMsgs;
  pHk->tcCount    = pCtx->tcCount;
  pHk->tmCount    = pCtx->tmCount;
  pHk->expdtSdoUl = pCtx->expdtSdoUlOut;
  pHk->expdtSdoDl = pCtx->expdtSdoDlOut;
  pHk->expdtSdoAbt= pCtx->expdtSdoAbt;
  pHk->blkSdoUl   = pCtx->sdoUlExecIn;
  pHk->blkSdoDl   = pCtx->sdoDlExecIn;
  pHk->blkSdoAbt  = pCtx->blkSdoAbt;
  
  pHk->hbMsgsIn  = pCtx->hbMsgsIn;
  pHk->hbMsgsOut = pCtx->hbMsgsOut; 
  pHk->nmtMsgs   = pCtx->nmtMsgs;
  pHk->syncMsgs  = pCtx->syncMsgs;
  
  pHk->retMsgs   = pCtx->retMsgs;
  pHk->pdoMsgsIn = pCtx->pdoMsgsIn; 
  pHk->pdoMsgsOut= pCtx->pdoMsgsOut;
  pHk->sdoMsgsIn = pCtx->sdoMsgsIn;
  pHk->sdoMsgsOut= pCtx->sdoMsgsOut;
  
  /* for node list:
   * 
   * nodeId:        device base node ID 
   * nodeIdCnt:     device node ID count 
   * switchOn:      device state as notified by application 
   * state:         device state as announced by its last HB message 
   * lastHbCycle:   cycle of last received HB message 
   * sdoBlkCnt:     Block SDO transfer counter
   * sdoStatus:     last Block SDO transfer execution result
  */
  pNode = &pHk->nodeInfo[0];
  pStat = &pCtx->pNodeStat[0];
  for (idx = 0; idx < pCtx->nodes; idx++)
  {
    pNode->state = pStat->nodeState;
    pNode->hbTotal = pStat->hbTotal;
    pNode->lastHbCycle = pStat->lastCycle;
    if ((((1<<idx) & pCtx->nodeSwitchOnMsk)) != 0)
    {
      pNode->switchOn = TRUE;
    }
    else
    {
      pNode->switchOn = FALSE;
    }
    
    pNode->nmtCnt = pStat->nmtTotal;
    pNode->retCnt = pStat->retTotal;
    
    pNode->pdoTcCnt = pStat->pdoTcTotal;
    pNode->pdoTmCnt = pStat->pdoTmTotal;
    
    /* Expdt SDO */
    pNode->sdoExpdtUlCnt      = pStat->sdoExpdtUlTotal;
    pNode->sdoExpdtDlCnt      = pStat->sdoExpdtDlTotal;
    pNode->sdoExpdtAbtCliCnt  = pStat->sdoExpdtAbtCliCnt;
    pNode->sdoExpdtAbtSerCnt  = pStat->sdoExpdtAbtSerCnt;
    pNode->sdoExpdtAbtCliCode = pStat->sdoExpdtAbtCliCode;
    pNode->sdoExpdtAbtSerCode = pStat->sdoExpdtAbtSerCode;
        
    /* SDO Blk */
    pNode->sdoBlkDlCnt        = pStat->sdoBlkDlTotal;
    pNode->sdoBlkUlCnt        = pStat->sdoBlkUlTotal;
    pNode->sdoBlkAbtCliCnt    = pStat->sdoBlkAbtCliCnt;
    pNode->sdoBlkAbtSerCnt    = pStat->sdoBlkAbtSerCnt;
    pNode->sdoBlkAbtCliCode   = pStat->sdoBlkAbtCliCode;
    pNode->sdoBlkAbtSerCode   = pStat->sdoBlkAbtSerCode;
    pNode->sdoBlkStatus       = pStat->sdoBlkStatus;
   
    /* for the next node */
    pStat++;
    pNode++;
  }
   
  /* Update CAN Error Status from various variables */
  pHk->canErrSts = CAN_ERR_STS(pCtx->hasOpStateExited, pHk->canTotHbF, ! pCtx->extCanHealth, ! pCtx->actPmHealth);

}

/***********************************************************************/
/**
 * @brief  CanOpenAction_busMgrAutom - Engine of the CAN Bus State machine.
 *
 * This function identifies the CANopen event from inputs and its context,
 * searches for the transition descriptor corresponding to the event
 * in the current state if any, and performs the 'action' associated
 * to the transition. Finally, it updates the state of the current state
 * machine.
 *
 * This function is used to manager both platform and payload CAN Bus.
 *
 * @param[in,out] pCtx  The pointer to the context of a CAN Bus Manager.
 * @return        N/A
 *
 *
 ***********************************************************************/ 
void CanOpenAction_busMgrAutom(CoCtx *pCtx)
{
  CoEvt evt = E_COEVT_END;
  CoState state = E_COSTATE_END;
  CoTrans const *pTrans = NULL;
  
  /*
   * As the Bus Manager state machine is triggered by synchronisation event,
   * and in all state, this is transitions for synchronisation events. In case
   * a NMT command event is not allowed for a given state, the cyclic sync event
   * is always expected.  
   */

  while (pTrans == NULL)
  {
    /* retrieve the next event */
    evt = coEvtDecode (pCtx);
  
    /* identify the action */
    pTrans = coTransGet(evt, pCtx->curState);
  }
  
  /* Perform the action associated to the transition.
   * When the action has been performed, the "new" state is returned
   * by the action.
   */

  state = pTrans->action(pCtx);
  pCtx->lastEvt = evt;
  pCtx->nEvt++;

  /* state transition occurred: update context */
  if (state != pCtx->curState)
  {   
    pCtx->lastState = pCtx->curState;
    pCtx->curState = state;
    pCtx->nTrans++;    
  }
}

/*------------------ ooOoo Local functions ooOoo ----------------------*/

/***********************************************************************/
/**
 * @brief  coTransGet - get the transition corresponding to current event 
 * 
 * @param[in] event The incoming event to be handled
 * @param[in] state The urrent automaton's state
 * @return    CoTrans The pointer to the matched transition from
 *  the transition table, or Null if there is no transition associated to
 *  the couple (state, event). In such a case, no action to be taken
 *  for this event in the current state.
 *
 ***********************************************************************/ 
PRIVATE CoTrans const *coTransGet(CoEvt event, CoState state)
{
  CoTrans const *pTrans;
  Boolean found;

  /* Search what transition applies (for current event in current state) */
  pTrans = &canOpenTrans[0];
  found =  FALSE;
  
  while ((pTrans->curState != E_COSTATE_END) && (found == FALSE)) {
    /* check both event and state */
    if ((pTrans->curState == state) && (pTrans->evtIn == event))
    {
      /* Transition found */
      found = TRUE;
    }
    else
    {
      pTrans++;
    } 
  }
  
  /* %COVER%TRUE% Defensive programming - ERROR_REPORT macro called here. */      
  if (found == FALSE)
  {
    /* No action found */
    ERROR_REPORT(SW_ERROR, event, state, 0);    
    pTrans = NULL;
  }
  return pTrans;
}

/***********************************************************************/
/**
 * @brief  coEvtDecode- decode the current incoming event 
 * 
 * @param[in] pCtx  The pointer to the context of a CAN Bus Manager.
 * @return    CoEvt The current event event 
 *
 ***********************************************************************/ 
PRIVATE CoEvt coEvtDecode (CoCtx *pCtx)
{
  CoEvt coEvt;
  Uint idx;
  Uint cmd;
  
  /* upon CYCLE event */
  if ((pCtx->slotInCycle == 0) && (pCtx->itInCycle == 0))
  {
    /* upon a Cycle event, get input mstCmd if we are not in INIT */
    if ((pCtx->mstCmdIn > pCtx->mstCmdOut) && (pCtx->curState > E_COSTATE_INIT))
    {
      idx = pCtx->mstCmdOut%CO_MST_CMD_QUEUE_LENGTH;
      cmd = pCtx->mstCmdQueue[idx];
      pCtx->mstCmdOut++;
      coEvt = coEvtFromNmtCs(cmd);
    }
    /* check toggle timeout if we are in PRE_OP or OP mode */
    else if ((pCtx->ttoggleTimer < 0) && 
        ((pCtx->curState == E_COSTATE_PRE_OP)||(pCtx->curState == E_COSTATE_OP)))
    {
      coEvt = E_COEVT_TT_TIMEOUT;
    }
    /* check treset timeout if we are in OP mode */
    else if ((pCtx->tresetTimer < 0) && 
             ((pCtx->curState == E_COSTATE_PRE_OP)||(pCtx->curState == E_COSTATE_OP)))
    {
      if(pCtx->curState == E_COSTATE_OP)
      {
        pCtx->tresetTriggeredInOp = TRUE;
      }
      coEvt = E_COEVT_RESET;
    }
    else if((pCtx->tresetTriggeredInOp == TRUE) && 
            (pCtx->curState == E_COSTATE_PRE_OP))
    {
      pCtx->tresetTriggeredInOp = FALSE;
      coEvt = E_COEVT_START;
    }
    else /* otherwise, simple cycle event */
    {
      coEvt = E_COEVT_10HZ;
    }
  }
  /* upon slot event */
  else
  {
    coEvt = E_COEVT_200HZ;
  }
  
  return coEvt;
}

/***********************************************************************/
/**
 * @brief  coEvtFromNmtCs - convert MNT command specifier to
 * a CANopen manager's event
 * 
 * @param[in] cmd   The NMT command specifier.
 * @return    CoEvt The current event 
 *
 ***********************************************************************/ 
PRIVATE CoEvt coEvtFromNmtCs (Uint cmd)
{
  CoEvt coEvt;
  
  /* select event Id from command Id */
  switch (cmd)
  {
  case CANOPEN_NMT_CS_START:
    coEvt = E_COEVT_START;
    break;
  case CANOPEN_NMT_CS_STOP:
    coEvt = E_COEVT_STOP;
    break;
  case CANOPEN_NMT_CS_ENTER_PRE_OP:
    coEvt = E_COEVT_ENTER_PRE_OP;
    break; 
  case CANOPEN_NMT_CS_RESET_NODE:
  case CANOPEN_NMT_CS_RESET_COM:
    coEvt = E_COEVT_RESET;
    break;
  case CO_SWITCH_BUS_CMD:
    coEvt = E_COEVT_SWITCH_BUS;
    break;
    
  default:
    /* %COVER%STMT% Defensive programming:
     * case when an unexpected input command has been registered
     * ERROR_REPORT is invoked
     */
    coEvt = E_COEVT_IGNORED;
    ERROR_REPORT(SW_ERROR, cmd, 0, 0);
    break;
  }
  
  /* return event Id to caller */
  return coEvt;
}

/***********************************************************************/
/**
 * @brief cobIdDescBuild - Build up the CobId descriptor table
 *
 * This function Builds up CobId descriptor table of a Bus
 * 
 * @param[in] pCtx  The pointer to the context of a CAN Bus Manager.
 * @return    N/A
 *
 ***********************************************************************/ 
PRIVATE void cobIdDescBuild(CoCtx *pCtx)
{
  U08 baseNid;
  U08 nodeId;
  U08 idx;
  Uint nodeIdCnt;
  Uint pdoTmIdx;
  
  CobIdDesc *pCobIdDescList;
  CobIdDesc *pCobIdDesc;
  CoBusHkArea *pHk;
  Uint cobId;
  
  Uint groups;
  Uint group;
  Uint groupBase;
 
  pHk = &CoMgr_hkArea[pCtx->busId];
   
  /* reset CobId descriptor list */
  pCobIdDescList = CoMgr_cobIdList[pCtx->busId];
  memset (pCobIdDescList, 0, sizeof(CobIdDesc) * (CANOPEN_NODE_MAX+1));  
  
  pdoTmIdx = 0;
  
  for (idx = 0; idx < pCtx->nodes; idx++)
  {
    baseNid = pCtx->pNodeList[idx].baseNid;

    /* the table is only applicable to slave node */
    /* %COVER%FALSE% Defensive programming - ERROR_REPORT macro called here. */      
    if (CO_SLV_BASE_NODE_VALID(baseNid))
    {
      pCtx->nodeToBusNodeIdx[baseNid] = idx;
            
      /* build up cobIdDesc list: TX/nodeIdx and RX/nodeIdx */
      for (nodeIdCnt = 0; nodeIdCnt <(Uint)(CO_NMSK_NID_MAX - pCtx->pNodeList[idx].nodeMsk);
          nodeIdCnt++)
      {
        nodeId = baseNid+nodeIdCnt;
        pCtx->nodeToBusNodeIdx[nodeId] = idx;
        /* build up cobIdDesc list (TC): TX/nodeIdx */
        groups = NELEMENTS(cobIdTxGroup);
        for (group = 0; group < groups; group++)
        {
          groupBase = cobIdTxGroup[group];
          cobId = groupBase+baseNid+nodeIdCnt;
          pCobIdDesc = &pCobIdDescList[cobId];
          pCobIdDesc->txGroup = groupBase;
          pCobIdDesc->rxGroup = 0;
          pCobIdDesc->nodeIdx = idx;
          pCobIdDesc->tpdoIdx = 0; /* N/A */
        }

        /* build up cobIdDesc list (TM): RX/nodeIdx */
        groups = NELEMENTS(cobIdRxGroup);
        for (group = 0; group < groups; group++)
        {
          groupBase = cobIdRxGroup[group];
          cobId = groupBase+baseNid+nodeIdCnt;
          pCobIdDesc = &pCobIdDescList[cobId];
          pCobIdDesc->rxGroup = groupBase;
          pCobIdDesc->txGroup = 0;
          pCobIdDesc->nodeIdx = idx;
          
          if (CO_TPDO_COBID_VALID(cobId))
          {
            pCobIdDesc->tpdoIdx = pdoTmIdx;
            pdoTmIdx++;
          }
          else
          {
            pCobIdDesc->tpdoIdx = 0; /* N/A */
          }
        }
        
      } /* for nodeCnt */
      
      /* for HK Area */
      pHk->nodeInfo[idx].nodeId = baseNid;
      pHk->nodeInfo[idx].nodeIdCnt = CO_NMSK_NID_MAX - pCtx->pNodeList[idx].nodeMsk;
    }
    else
    {
      /* configuration table is incorrect */
      ERROR_REPORT(SW_ERROR, pCtx->busId, idx, baseNid);
    }
  }
}
/***********************************************************************/
/**
 * @brief syncWinEvtNotify - notify applications of Synchronisation Windows
 * event 
 *
 * This function searches in the recorded applications list the ones who
 * are pending, then release them by sending them a specific event
 * 
 * @param[in] pCtx  The pointer to the context of a CAN Bus Manager.
 * @return    N/A
 *
 ***********************************************************************/ 
PRIVATE void syncWinEvtNotify(CoCtx *pCtx)
{
  Uint idx;
  SyncWaitTask *pTask;
  
  /* search registered task list */
  pTask = pCtx->taskList;
  for (idx = 0; idx < pCtx->registeredTaskNum; idx++)
  {
    /* if a registered task is waiting, notify it */
    if (pTask->waiting == TRUE)
    {
      /* used the registered event for the task */
      /* send the event to Sync Task */ 
      (void) rtems_event_send(pTask->taskId, pTask->rtEvt);   
    }
    
    /* go to the next one */
    pTask++;
  }
}

/***********************************************************************/
/**
 * @brief busRestart - Restart the bus with selected channel
 * 
 * @param[in,out] pCtx    The pointer to the context of a CAN Bus Manager
 * @param[in] chanSwitch  Flag indicating if the Bus switch is requested
 * @return    N/A
 * 
 requested */
  if (chanSwitch)
  {
    chanId = CANBUS_OPPOSITE_CHAN(pCtx->chanId);
    pCtx->chanId = chanId;
  }
  
  /* re-start the interface with the selected channel */
  coSts = CanOpenBus_restart(pCtx->busId, pCtx->chanId);
  
  /* %COVER%TRUE% Defensive programming - ERROR_REPORT macro called here. */  
  if (coSts != E_COSTATUS_OK)
  {
    ERROR_REPORT(SW_ERROR, pCtx->busId, coSts, pCtx->chanId);
  }

  if (chanSwitch)
  { 
    /* stop the Ttoggle timer */
    /* nothing to do as the Ttoggle is not active in INIT mode, and restart involves going to INIT mode */
    
    /* update Redundancy context */
    pCtx->ctoggle++;
    
    /* 1 second additional for Ttoggle */
    pCtx->ttogglePeriodInc = CO_CYCLE_IN_SECOND;
    
    /* clear HB reg */
    pCtx->nodeHbRegMsk = 0;
    
    /* clear HB health status */
    pCtx->healthStatus = E_CO_NODE_HEALTH_SAFE;
    
    /* there has been a channel Id switch */
    pCtx->busChanIdSwitched = TRUE;
    
  }
  
  /* SYNC:
   * commSyncEna: FALSE ==> Disable
   */
  pCtx->commSyncEna = FALSE;

  /* SDO: 
   * sdoEna: FALSE ==> Disable
   */
  pCtx->sdoEna = FALSE;
  
  /* PDO:
   * pdoEna: FALSE => Disabled
   */ 
  pCtx->pdoEna = FALSE;
  
  /* TX:
   * any TX msg in the last slot is ignored
   */
  pCtx->msgsInCurList = 0;
  
  pCtx->firstHbReceived = FALSE;

  pCtx->canStat = CanOpenBus_getCtrlStatus(pCtx->busId);
  
  /* Update Data Pool */
  CanOpenAction_updateHkArea(pCtx, E_COSTATE_INIT);
}

/***********************************************************************/
/**
 * @brief busInMsgListDiscard - Discard any incoming message
 * 
 * @param[in,out] pCtx  The pointer to the context of a CAN Bus Manager.
 * @return    N/A
 *
 ***********************************************************************/ 
PRIVATE void busInMsgListDiscard(CoCtx *pCtx)
{
  CoStatus result;
  Uint chan;
  Uint msgs;
  Uint i;
  Uint msgId;
  Uint msgRtr;
  Uint dataBytes;
  U08 msgData[CAN_MSG_SIZE];
  
  /* check if there is any messages in the queue */
  result = CanOpenBus_getRxSts(pCtx->busId, &chan, &msgs);
  /* update the HK  */
  switch (result)
  {
  case E_COSTATUS_OK:
    /* nothing to do */
    break;
  case E_COSTATUS_ACTPM_ERROR:
    pCtx->actPmHealth = E_CO_PM_HEALTH_FAIL;
    break;
  case E_COSTATUS_EXTCAN_ERROR:
    pCtx->extCanHealth = E_CO_EXTCAN_HEALTH_FAIL;
    break;
  default:
    /* %COVER%STMT% Defensive programming:
     * case when an unexpected error occurred
     * ERROR_REPORT is invoked
     */
    ERROR_REPORT(SW_ERROR, pCtx->busId, result, msgs);
    break;
  }
  
  /* retrieve all available messages without using them */
  for (i = 0; ((i < msgs) && (result == E_COSTATUS_OK)); i++)
  {
    result = CanOpenBus_getNextMsg(pCtx->busId, &msgId, &msgRtr, &dataBytes, msgData);
    /* %COVER%TRUE% Defensive programming - ERROR_REPORT macro called here. */    
    if (result != E_COSTATUS_OK)
    {
      ERROR_REPORT(SW_ERROR, pCtx->busId, result, i);
    }
  }
  pCtx->rxMsgs += msgs;
  
  return;
}

/***********************************************************************/
/**
 * @brief cobMsgProcess - process a bus message
 * 
 * @param[in,out] pCtx    The pointer to the context of a CAN Bus Manager.
 * @param[in] entryState  The Bus Manager's state.
 * @param[in] pCoMsg      The pointer to CAN Bus message.
 * @return    N/A
 *
 *
 ***********************************************************************/ 
PRIVATE void cobMsgProcess(CoCtx *pCtx, CoState entryState, CoMsg *pCoMsg)
{
  U08 baseNodeId;
  U08 nodeIdx;
  U08 devNodeBaseId;
  U16 rxGroup;
  CobIdDesc *pCobIdDescList;
  T_BOOL hbStInvalid;
  
  /* according to the CObID group, different handling are performed */
  baseNodeId = CO_NODE_TO_BASE(pCoMsg->cobId);
  
  /* if the node is identified in the pre-defined list, process it */
  nodeIdx = pCtx->nodeToBusNodeIdx[baseNodeId];
  if (nodeIdx != CO_SLV_NODES_MAX) 
  {
    devNodeBaseId = pCtx->pNodeList[nodeIdx].baseNid;
    pCobIdDescList = CoMgr_cobIdList[pCtx->busId];
    rxGroup = pCobIdDescList[pCoMsg->cobId].rxGroup;
    
    switch (rxGroup)
    {
    case CANOPEN_COB_TPDO1_BASE:
    case CANOPEN_COB_TPDO2_BASE:
    case CANOPEN_COB_TPDO3_BASE:
    case CANOPEN_COB_TPDO4_BASE:
      /* messages only handled when pdo is enabled, otherwise ignored */
      if (pCtx->pdoEna)
      {
        pdoMsgProcess(pCtx, pCoMsg);
        
        /* update the HK statistics */
        pCtx->pdoMsgsIn++;
      }
      break;
    case CANOPEN_COB_TSDO_BASE:
      /* messages only handled when sdo is enabled and the base node Id is used, otherwise ignored */
      if ((pCtx->sdoEna) && (baseNodeId == devNodeBaseId))
      {       
        CanOpenSdo_submitMsg(pCtx, pCoMsg);
        
        /* update the HK statistics */
        pCtx->sdoMsgsIn++;
      }
      break;
      
    case CANOPEN_COB_HB_BASE:
      /* a minimum protocol check */
      /* - slave HB message handled only in three states, else ignored */
      hbStInvalid = CO_HB_ST_INVALID(pCoMsg->data[0]);
      if ((baseNodeId == devNodeBaseId) &&
          (pCoMsg->rtr == CANOPEN_HB_RTR) &&
          (pCoMsg->dataBytes == CANOPEN_HB_DATA_LEGNTH) &&
          (!hbStInvalid) &&
          ((entryState == E_COSTATE_PRE_OP) ||
           (entryState == E_COSTATE_OP) ||
           (entryState == E_COSTATE_STOP)))
      {
        hbMsgProcess(pCtx, entryState, pCoMsg);
        
        /* update the HK statistics */
        pCtx->hbMsgsIn++;
      }
      break;
    
    default:
      /* ignore message with invalid RX node ID: slave protocol error */
      break;
    } 
  }
}

/***********************************************************************/
/**
 * @brief   - Process incoming messages from CAN Bus interface
 * 
 * @param[in,out] pCtx    The pointer to the context of a CAN Bus Manager.
 * @param[in] entryState  The automaton's current state.
 *
 ***********************************************************************/ 
PRIVATE Uint busInMsgListProcess(CoCtx *pCtx, CoState entryState)
{
  CoStatus result;
  Uint chan;
  Uint msgs;
  Uint i;
  Uint msgId;
  Uint msgRtr;
  Uint dataBytes;
  CoMsg msg;
  
  /* check if there is any messages in the queue */
  msgs = 0;
  result = CanOpenBus_getRxSts(pCtx->busId, &chan, &msgs);
  
  /* update the HK  */
  switch (result)
  {
  case E_COSTATUS_OK:
    /* nothing to do */
    break;
  case E_COSTATUS_ACTPM_ERROR:
    pCtx->actPmHealth = E_CO_PM_HEALTH_FAIL;
    break;
  case E_COSTATUS_EXTCAN_ERROR:
    pCtx->extCanHealth = E_CO_EXTCAN_HEALTH_FAIL;
    break;
  default:
    /* %COVER%STMT% Defensive programming - ERROR_REPORT macro called here */
    pCtx->actPmHealth = E_CO_PM_HEALTH_FAIL; /* unexpected error */
    ERROR_REPORT(SW_ERROR, pCtx->busId, result, msgs);
    break;
  }  
  
  /* retrieve all available messages */
  for (i = 0; ((i < msgs) && (result == E_COSTATUS_OK)); i++)
  {
    result = CanOpenBus_getNextMsg(pCtx->busId, &msgId, &msgRtr, &dataBytes, msg.data);
    
    /* %COVER%TRUE% Defensive programming - ERROR_REPORT macro called here. */
    if (result != E_COSTATUS_OK)
    {
      ERROR_REPORT(SW_ERROR, pCtx->busId, result, i);
    }
    else
    {
      msg.cobId = msgId;
      msg.rtr = msgRtr;
      msg.dataBytes = dataBytes;
      
      /* update statistics */
      pCtx->rxMsgs++;
      
      /* processing current message */
      cobMsgProcess (pCtx, entryState, &msg);
    }
  }
  /* store the last message */
  if ((result == E_COSTATUS_OK) && (msgs > 0))
  {
    memcpy (&pCtx->lastRxMsg, &msg, sizeof(CoMsg));
  }

  return msgs;
}

/***********************************************************************/
/**
 * @brief  cycleProcess - nominal processing at the beginning of a 10Hz cycle 
 * 
 * The function is used in nominal cases to performs the following operations:
 *   - generation cycle synchronisation semaphore,
 *   - discard the received message if it is in INIT or just out INIT state,
 *   - handle the received messages, generated in the last slot of
 *     previous cycle (normally, no message is expected),
 *   - generate the SYNC message if it is in the right cycle,
 *   - generate the HB message if necessary,
 *   - discard pending expedited SDO requests if the manager is not in the right state,
 *   - generate slave NMT command on Payload bus according to the received request, 
 *   - get the CAN bus status and update the HK parameter
 *     
 * @param[in,out] pCtx    The pointer to the context of a CAN Bus Manager.
 * @param[in] entryState  The automaton's current state.
 * @return    N/A
 *
 * 
 ***********************************************************************/ 
PRIVATE void cycleProcess(CoCtx *pCtx, CoState entryState)
{
  CoStatus coSts;
  Uint list=0;
  Uint dataBytes;
  Uint msgs;
  U08 data[CAN_MSG_SIZE];
  Uint fineTime;
  
  /* flush the cycle synchronisation semaphore */
  /* %COVER%TRUE% Defensive programming - ERROR_REPORT macro called here. */
  if (rtems_semaphore_flush(pCtx->cycSyncSem) != RTEMS_SUCCESSFUL)
  {
    ERROR_REPORT(SW_ERROR, pCtx->busId, pCtx->cycles, entryState);
  }
  
  /* update the transmission list context vs previous list */
  coSts = msgsXmtCheck(pCtx);
  if ((coSts == E_COSTATUS_OK) || 
      ((coSts == E_COSTATUS_BUS_BUSY) && (pCtx->firstHbReceived == TRUE) && (CanOpenBus_getMstAloneOnBus(pCtx->busId) == TRUE)))
  {
    /* message lists used in flip/flop mode */ 
    list = (pCtx->list+1)%2;
    pCtx->list = list;
    
    /* Master Redundancy Management */
    if (((entryState == E_COSTATE_PRE_OP) || (entryState == E_COSTATE_OP))
        && (pCtx->nodeSwitchOnMsk != 0))
    { 
      pCtx->ttoggleTimer--;
      pCtx->tresetTimer--;
    }
    
    /* discard received message if it is a transition to INIT or INIT->PRE-OP  */
    if ((entryState == E_COSTATE_INIT) ||
        ((pCtx->curState == E_COSTATE_INIT) && (entryState == E_COSTATE_PRE_OP)))
    { 
      busInMsgListDiscard(pCtx);
    }
    else
    {
      /* processing received message during the last period */
      msgs = busInMsgListProcess(pCtx, entryState);
    }
    
    /* HB protocol management, needed to be done before SYNC to avoid being delayed by lot of SYNChed PDOs */
    hbMgtHandler(pCtx, entryState);
  }
  if (coSts == E_COSTATUS_OK)
  {
    /* 
     * Check SYNC transmission period for its generating 
     */
    if (pCtx->commSyncEna)
    {
      if (pCtx->commSyncCountUp == 0)
      {
        /* prepare a synchronisation message */
        dataBytes = CANOPEN_SYNC_DATA_LENGTH;
        
        /* build the message in the list */
        coSts = CanOpenBus_putMsgInList(pCtx->busId, 
                                        list, CANOPEN_COB_SYNC, CANOPEN_SYNC_RTR, dataBytes, data);
        /* %COVER%FALSE% Defensive programming - ERROR_REPORT macro called here. */
        if (coSts == E_COSTATUS_OK)
        {
          /* one more message in the list */
          pCtx->msgsInCurList++;
        }
        else
        {
          /* unexpected error: message must be sent */
          ERROR_REPORT(SW_ERROR, pCtx->busId, coSts, pCtx->chanId);
        }
        
        /* update statistics */
        pCtx->syncMsgs++;
                     
      }
      /* update the HB period counting */
      pCtx->commSyncCountUp = (pCtx->commSyncCountUp + 1) % pCtx->commSyncPeriod;
    }
    
    
    /* RET message distribution: preparation for the current cycle */
    if (pCtx->pdoEna)
    {
      /* at the last cycle of a SYNC distribution period,
       * compute the cuc time for the beginning of the next cycle
       * */
      if (pCtx->commSyncCountUp == 0)
      {
        pCtx->cucC32BNextSyncCycle = pCtx->cucC32B;
        fineTime = pCtx->cucF16B + U16_MAX/CO_CYCLE_IN_1SEC;;
        
        if (fineTime > U16_MAX)
        {
          pCtx->cucC32BNextSyncCycle++;
          fineTime -= (U16_MAX+1);
        }
        pCtx->cucF16BNextSyncCycle = fineTime & U16_MSK;
      }
      
      /* at the beginning of a SYNC distribution cycle, restart the distribution list */
      if (pCtx->commSyncCountUp == (1%pCtx->commSyncPeriod))  
      {
        retCycSetup(pCtx);
      }
    }
    else
    {
      /* clear all pending requests */
      pdoReqQueuesClear(pCtx);
      
    }
    /* incoming NMT command handling. Process only Payload commands in Slot 0 */
    if (pCtx->busId == CANBUS_ID_PL)
    {
       slaveCmdQueueHandler(pCtx);
    }

    /* request to transmit all messages in the list */
    if (pCtx->msgsInCurList > 0)
    {
      processTxMsgList(pCtx, list);
      
      /* already count msgs as transmitted for statistics even if busy or error => count them only once */
      pCtx->txMsgs += pCtx->msgsInCurList;
    }
  }
  
  pCtx->canStat = CanOpenBus_getCtrlStatus(pCtx->busId);
  
  /* update the exported HK area */
  CanOpenAction_updateHkArea(pCtx, entryState);
}

/***********************************************************************/
/**
 * @brief  slotProcess - nominal processing at the beginning of a 200Hz slot
 * 
 * The function is used in nominal cases to performs the following operations:
 *  - check the execution status of the message sent in the previous slot,
 *  - handle the received messages, generated in the last slot of
 *    previous cycle,
 *  - If in a 200Hz slot:
 *    - handle Block exchanges,
 *  - If in a 100Hz slot:
 *    - handle Expedited SDO exchanges,
 *    - generate the RET message if the manager is in the right state, cycle and slot,
 *    - generate PDO TC command if the manager is in the right state, slot,
 *    - generate slave NMT command on Platform bus according to the received request,
 *    - get the CAN bus status and update the HK parameter
 *   
 * @param[in,out] pCtx    The pointer to the context of a CAN Bus Manager.
 * @param[in] entryState  The automaton's current state.
 * 
 * @return    N/A
 * 
 ***********************************************************************/ 
PRIVATE void slotProcess(CoCtx *pCtx, CoState entryState)
{
  CoStatus coSts;
  Uint list;
  Uint msgs;
  Bool go = TRUE;
 
  /* update the transmission list context vs previous list */
  coSts = msgsXmtCheck(pCtx);
  if (coSts == E_COSTATUS_OK)
  {
    /* message lists used in flip/flop mode */ 
    list = (pCtx->list+1)%2;
    pCtx->list = list;
    
    /* processing received message during the last 200Hz period */
    msgs = busInMsgListProcess(pCtx, entryState);
    
    /* SDO protocol handling */
    if (pCtx->sdoEna)
    {
      /* SDO Block specific */
      /* if a new list is created for SDO, get and process it */
      if (pCtx->blkSdoAddList)
      {
        /* update the transmission list context */
        /* %COVER%FALSE% Defensive programming:
         * case error bus passive not anymore possible when 
         * SDO machine state in DATA2 sub-state
         * no need for ERROR_REPORT
         */
        if (msgsXmtCheck(pCtx) == E_COSTATUS_OK)
        {
          /* message lists used in flip/flop mode */ 
          list = (pCtx->list+1)%2;
          pCtx->list = list;
          
          pCtx->blkSdoAddList = FALSE;
          
          /* process additional list if necessary */
          msgs = busInMsgListProcess(pCtx, entryState);
        }
        else
        {
          go = FALSE;
        }
      }

      /* %COVER%FALSE% Defensive programming:
       * case error bus passive not anymore possible when 
       * SDO machine state in DATA2 sub-state
       * no need for ERROR_REPORT
       */      
      if (go)
      {
        /* generate REFRESH event for SDO manager */
        CanOpenSdo_updateState(pCtx);
       
        /* if a new list is created for SDO, get and process it, this will only be 
         * valid if SDO Manager is not in Ready/Idle Mode */
        if (pCtx->blkSdoAddList)
        {
          /* update the transmission list context */
          /* %COVER%FALSE% Defensive programming:
           * case error bus passive not anymore possible when 
           * SDO machine state in DATA2 sub-state
           * no need for ERROR_REPORT
           */                
          if (msgsXmtCheck(pCtx) == E_COSTATUS_OK)
          { 
            /* message lists used in flip/flop mode */ 
            list = (pCtx->list+1)%2;
            pCtx->list = list;
            
            /* process additional list if necessary */
            msgs = busInMsgListProcess(pCtx, entryState);
            
            pCtx->blkSdoAddList = FALSE;
          }
          else
          {
            go = FALSE;
          }
        }

        /* %COVER%FALSE% Defensive programming:
         * case error bus passive not anymore possible when 
         * SDO machine state in DATA2 sub-state
         * no need for ERROR_REPORT
         */              
        if (go)
        {
          /* SDO request handling */
          CanOpenSdo_handleReq(pCtx);
        }
      }
    } /* SDO enabled */
    
    /* Perform all PDO, NMT and dedicated RET sent actions at 100HZ frequency 
     * 
     * %COVER%FALSE% Defensive programming:
     * case error bus passive not anymore possible when 
     * SDO machine state in DATA2 sub-state
     * no need for ERROR_REPORT
     */          
    if ((go == TRUE) && (pCtx->itInCycle%2 == 0))
    {
      /* RET message distribution: distribution only in specific slot */
      if ((pCtx->pdoEna) &&  (pCtx->commSyncCountUp == 0) 
          && (pCtx->slotInCycle == CO_PDO_RET_DIST_SLOT))
      {
        /* process input distribution request */
        retReqMsgsBuild(pCtx);
      }
      
      /* PDO TC transmission: sending only in specific slots , and after SDO */
      if ((pCtx->pdoEna) && (pCtx->slotInCycle >= CO_PDO_TC_START_SLOT)
          && (pCtx->slotInCycle < (CO_PDO_TC_START_SLOT + CO_PDO_TC_SLOTS)))
      {
        pdoTcMsgsBuild(pCtx);
      }

      /* Process Platform NMT commands only in Slot 1 */
      if ((pCtx->busId == CANBUS_ID_PF) && (pCtx->slotInCycle == 1))
      {
        slaveCmdQueueHandler(pCtx);
      }
    }
    
    /* transmit all messages in the list */
    if ((go == TRUE) && (pCtx->msgsInCurList > 0))
    {
      processTxMsgList(pCtx, list);
            
      /* already count msgs as transmitted for statistics even if busy or error => count them only once */
      pCtx->txMsgs += pCtx->msgsInCurList;
            
    } /* if msgsInCurList > 0 */
  }  
}

/***********************************************************************/
/**
 * @brief  slaveCmdQueueHandler - handle the slave's input NMT command queue
 * 
 * @reentrant The variable @ref CoMgr_ctx is protected for exclusive
 * read/write access by a mutex @ref CoCtx.lock.
 *
 * @param[in,out] pCtx  The pointer to the context of a CAN Bus Manager.
 * @return N/A
 *
 * 
 ***********************************************************************/ 
PRIVATE void slaveCmdQueueHandler(CoCtx *pCtx)
{
  CoStatus coSts;
  Bool go = TRUE;
  Uint commands;
  Uint idx;
  Uint curMsg;
  CoMsg *pCoMsg;
  U08 baseNodeId;
  U08 nodeIdx;
  NodeStat *pNode;
  
  commands = pCtx->slvCmdIn - pCtx->slvCmdOut;
  go = TRUE;
  
  /* handler the input commands if any */
  if (commands > 0)
  {
    /* BEGIN: protect the modification of input command queues */
    ResourceLock_lock(&pCtx->lock, RESOURCELOCK_NO_TIMEOUT);
    for (idx = 0; ((idx < commands) && go); idx++)
    {
      curMsg = pCtx->slvCmdOut%CO_SLV_CMD_QUEUE_LENGTH;
      pCoMsg = &pCtx->slvCmdQueue[curMsg];
      
      /* %COVER%FALSE% Defensive programming - ERROR_REPORT macro called here. */
      if (pCoMsg->cobId == CANOPEN_COB_NMT)
      {
        /* build the NMT message in the list */
        coSts = CanOpenBus_putMsgInList(pCtx->busId, pCtx->list,
                                        pCoMsg->cobId, pCoMsg->rtr, pCoMsg->dataBytes, pCoMsg->data);
        
        /* %COVER%FALSE% Defensive programming - ERROR_REPORT macro called here.  
         * By design, there cannot be more messages to generate (limited by NMT queue size) than the 
         * max size of the send list which is currently limited to the maximum number of messages 
         * that can be transmitted in 10ms. */
        if (coSts == E_COSTATUS_OK)
        {
          /* one more message in the list */
          pCtx->msgsInCurList++;
          
          /* progress in the queue */
          pCtx->slvCmdOut++;
          
          /* update statistics */
          pCtx->nmtMsgs++;
          
          /* get its index in the node list and update its state if it is a present node
           * Note: for the TC (2,133), node can be any one, even the node not present
           */
          baseNodeId = CO_NODE_TO_BASE(pCoMsg->data[CANOPEN_NMT_DATA_NID_IDX]);
          nodeIdx = pCtx->nodeToBusNodeIdx[baseNodeId];
          
          /* the present node */
          if (nodeIdx < CO_SLV_NODES_MAX)
          {
            pNode = &pCtx->pNodeStat[nodeIdx];
            pNode->nmtTotal++;
          }
        }
        else 
        {
          /* unexpected error, or coSts == E_COSTATUS_MSG_LIST_FULL transmission list full */
          ERROR_REPORT(SW_ERROR, pCtx->busId, coSts, pCtx->chanId);
          go = FALSE;
        }
      }
      else
      {
        ERROR_REPORT(SW_ERROR, pCtx->busId, pCtx->curState, pCoMsg->cobId);
      }

    } /* for */
 
    /* END: protect the modification of input command queues */
    ResourceLock_unlock(&pCtx->lock);
  }
}

/***********************************************************************/
/**
 * @brief  slaveCmdQueueClear - clear all input commands in the queue
 * 
 * @reentrant The variable @ref CoMgr_ctx is protected for exclusive
 * read/write access by a mutex @ref CoCtx.lock.
 *
 * @param[in,out] pCtx  The pointer to the context of a CAN Bus Manager.
 * @return N/A
 * 
 ***********************************************************************/ 
PRIVATE void slaveCmdQueueClear(CoCtx *pCtx)
{
  /* BEGIN: protect the modification of input command queues */
  ResourceLock_lock(&pCtx->lock, RESOURCELOCK_NO_TIMEOUT);
  
  /* ignore all not handled commands, i.e. discard any pending input commands */
  pCtx->slvCmdIn = pCtx->slvCmdOut;
  
  /* END: protect the modification of input command queues */
  ResourceLock_unlock(&pCtx->lock);
}

/***********************************************************************/
/**
 * @brief  pdoReqQueuesClear - clear all input commands in the pdoTc and RET queue
 * 
 * @reentrant The variable @ref CoMgr_ctx is protected for exclusive
 * read/write access by a mutex @ref CoCtx.lock.
 *
 * @param[in,out] pCtx  The pointer to the context of a CAN Bus Manager.
 * @return N/A
 * 
 ***********************************************************************/ 
PRIVATE void pdoReqQueuesClear(CoCtx *pCtx)
{
  
  /* BEGIN: protect the modification of RET distribution request queues */
  ResourceLock_lock(&pCtx->lock, RESOURCELOCK_NO_TIMEOUT);
  
  /* for RET & PdoTc: ignore all not handled requests, i.e. discard any pending request */

  pCtx->retIn = pCtx->retOut;
  pCtx->pdoTcIn = pCtx->pdoTcOut;
  
  /* END: protect the modification of input command queues */
  ResourceLock_unlock(&pCtx->lock);
}

/***********************************************************************/
/**
 * @brief  expdtSdoReqQueuesClear - clear all input expedited Download & Upload commands
 * queues
 * 
 * @reentrant The variable @ref CoMgr_ctx is protected for exclusive
 * read/write access by a mutex @ref CoCtx.lock.
 *
 * @param[in,out] pCtx  The pointer to the context of a CAN Bus Manager.
 * @return N/A
 * 
 ***********************************************************************/ 
PRIVATE void expdtSdoReqQueuesClear(CoCtx *pCtx)
{
 
  /* BEGIN: protect the modification of expedited SDU Download & Upload request queues */
  ResourceLock_lock(&pCtx->lock, RESOURCELOCK_NO_TIMEOUT);
  
  pCtx->expdtSdoUlIn = pCtx->expdtSdoUlOut;
  pCtx->expdtSdoDlIn = pCtx->expdtSdoDlOut;
  
  /* END: protect the modification of input command queues */
  ResourceLock_unlock(&pCtx->lock);
}

/***********************************************************************/
/**
 * @brief  blockSdoReqQueuesClear - clear all input SDO block Download & Upload commands
 * queues
 * 
 * @reentrant The variable @ref CoMgr_ctx is protected for exclusive
 * read/write access by a mutex @ref CoCtx.lock.
 *
 * @param[in,out] pCtx  The pointer to the context of a CAN Bus Manager.
 * @return N/A
 * 
 ***********************************************************************/ 
PRIVATE void blockSdoReqQueuesClear(CoCtx *pCtx)
{
 
  /* BEGIN: protect the modification of Block SDU Download & Upload request queues */
  ResourceLock_lock(&pCtx->lock, RESOURCELOCK_NO_TIMEOUT);
  
  pCtx->sdoUlIn = pCtx->sdoUlOut;
  pCtx->sdoDlIn = pCtx->sdoDlOut;
  
  /* END: protect the modification of input command queues */
  ResourceLock_unlock(&pCtx->lock);
}
/***********************************************************************/
/**
 * @brief msgsXmtCheck - check the result of previously transmitted
 * messages list
 * 
 * @param[in,out] pCtx  The pointer to the context of a CAN Bus Manager.
 * @return current transmit status
 * - E_COSTATUS_OK
 * - E_COSTATUS_PARAM_ERROR
 * - E_COSTATUS_ACTPM_ERROR
 * - E_COSTATUS_EXTCAN_ERROR
 * - E_COSTATUS_BUS_BUSY
 *
 ***********************************************************************/ 
PRIVATE CoStatus msgsXmtCheck(CoCtx *pCtx)
{
  CoStatus coSts;
  Uint msgs;
  Uint chan;
  Uint list;
  
  /* update the transmission list context vs previous list */
  if (pCtx->msgsInCurList != 0)
  {
    /* check the last transmit list */
    coSts = CanOpenBus_getTxSts(pCtx->busId, &chan, &list, &msgs);
    
    /* update the HK parameters for active Pm & Ext Can */ 
    switch (coSts)
    {
    case E_COSTATUS_OK:
    case E_COSTATUS_BUS_BUSY:
      /* nothing to do */
      break;
    
    case E_COSTATUS_ACTPM_ERROR:
      pCtx->actPmHealth = E_CO_PM_HEALTH_FAIL;
      break;
    
    case E_COSTATUS_EXTCAN_ERROR:
      pCtx->extCanHealth = E_CO_EXTCAN_HEALTH_FAIL;
      break;
    default:
      /* %COVER%STMT% Defensive programming:
       * all error cases already processed, no need for ERROR_REPORT 
       */      
      break;
    } /* switch */
    
    /* robust checks */
    /* %COVER%TRUE% Defensive programming - ERROR_REPORT macro called here. */
    if ((coSts==E_COSTATUS_OK) &&(msgs != pCtx->msgsInCurList))
    {
      ERROR_REPORT(SW_ERROR, pCtx->busId, msgs, pCtx->msgsInCurList);
      coSts = E_COSTATUS_PARAM_ERROR; /* unexpected error */
    }
  
    if (coSts != E_COSTATUS_BUS_BUSY)
    {
      pCtx->txMsgLists++;
      pCtx->msgsInCurList = 0;
    }
  }
  else
  {
    /* no on-going transmission, remain OK */
    coSts = E_COSTATUS_OK;
  }
  return coSts;
}

/***********************************************************************/
/**
 * @brief  retCycSetup - set-up the RET distribution 1 Hz cycle context 
 * 
 * @param[in,out] pCtx  The pointer to the context of a CAN Bus Manager.
 * @return N/A
 *
 ***********************************************************************/ 
PRIVATE void retCycSetup(CoCtx *pCtx)
{
  Uint idx;
  
  for (idx = 0; idx < pCtx->nodes; idx++)
  {
    pCtx->pNodeStat[idx].retDistInSyncCyc = FALSE;
  }
}

/***********************************************************************/
/**
 * @brief  hbMgtHandler - HB Protocol management (Bootup)
 * 
 * @param[in,out] pCtx    The pointer to the context of a CAN Bus Manager.
 * @param[in] entryState  The Bus Manager's state.
 * @return N/A
 *
 * 
 ***********************************************************************/ 
PRIVATE void hbMgtHandler(CoCtx *pCtx, CoState entryState)
{
  CoStatus coSts;
  Boolean mstHbPrd = FALSE;
  U08 data;
  /* 
   * Check Heart beat transmission period
   */
  /* note: the entry state may different from the current one */
  if ((pCtx->curState == E_COSTATE_INIT) && (entryState == E_COSTATE_PRE_OP))
  {
    data = CANOPEN_HB_ST_BOOT_UP;
    mstHbPrd = TRUE;
  }
  else if (pCtx->hbCountUp == 0)
  {
    data = coStateToHbState(entryState);
    mstHbPrd = TRUE;
  }
  
  if (mstHbPrd == TRUE)
  {
    if (CanOpenBus_getMstAloneOnBus(pCtx->busId) == FALSE)
    {
      /* prepare and build a heart beat message in the list */
      coSts = CanOpenBus_putMsgInList(pCtx->busId, pCtx->list,
                      CANOPEN_HB_MASTER_ID, CANOPEN_HB_RTR,
                      CANOPEN_HB_DATA_LEGNTH, &data);
      /* %COVER%FALSE% Defensive programming - ERROR_REPORT macro called here. */
      if (coSts == E_COSTATUS_OK)
      {
        /* one more message in the list */
        pCtx->msgsInCurList++;
        
        /* update statistics */
        pCtx->hbMsgsOut++;
      }
      else
      {
        /* unexpected error: message must be sent */
        ERROR_REPORT(SW_ERROR, pCtx->busId, coSts, pCtx->chanId);
      }
    }
    
    /* Master Redundancy Management */
    
    /* - update HB cycle */
    pCtx->lastHbCycle = pCtx->cycles;
    
    if ((entryState == E_COSTATE_PRE_OP) || (entryState == E_COSTATE_OP))
    {
      if (pCtx->nodeSwitchOnMsk == 0)
      {
        /* if no switch-on node, reset the timer */
        pCtx->ttoggleTimer = pCtx->ttogglePeriod + pCtx->ttogglePeriodInc;
        pCtx->tresetTimer = pCtx->tresetPeriod + pCtx->ttogglePeriodInc;
      }
      /* if all HB of power-on nodes are received, clear HB reg and reset the timer */
      else if ((pCtx->nodeHbRegMsk & pCtx->nodeSwitchOnMsk) == pCtx->nodeSwitchOnMsk)
      {
        /* operation:
         * - clear HB_REG,
         * - reset Ttoggle timer
         * - reset Treset timer
         * - reset any its previous increment 
         * - reset healthStatus
         */
        pCtx->nodeHbRegMsk = 0;
        pCtx->ttogglePeriodInc = 0;
        pCtx->ttoggleTimer = pCtx->ttogglePeriod;
        pCtx->tresetTimer = pCtx->tresetPeriod;
        pCtx->healthStatus = E_CO_NODE_HEALTH_SAFE;
        
      }
      /* if at least one HB is received reset treset timer */
      else if(pCtx->nodeHbRegMsk > 0 )
      {
         /* operation:
         * - reset Treset timer
         */
        pCtx->tresetTimer = pCtx->tresetPeriod;
        
      }
    }
  }
  
  /* - update the HB period counting */
  pCtx->hbCountUp = (pCtx->hbCountUp + 1) % pCtx->mstPrdHBPeriod;
}
/***********************************************************************/
/**
 * @brief  retReqMsgsBuild - Build RET messages for requested nodes
 * 
 * @reentrant The variable @ref CoMgr_ctx is protected for exclusive
 * read/write access by a mutex @ref CoCtx.lock.
 *
 * @param[in,out] pCtx  The pointer to the context of a CAN Bus Manager.
 * @return N/A
 *
 ***********************************************************************/ 
PRIVATE void retReqMsgsBuild(CoCtx *pCtx)
{
  CoStatus coSts;
  Uint idx;
  Uint msgId;
  Uint msgRtr;
  Uint dataBytes;
  NodeDesc *pNodeDesc;
  NodeStat *pNodeStat;
  U08 data[CAN_MSG_SIZE];
  Uint requests;
  Uint count;
  U32 msk;

  /* BEGIN: protect the modification of RET request queues */
  ResourceLock_lock(&pCtx->lock, RESOURCELOCK_NO_TIMEOUT);
  
  requests = pCtx->retIn - pCtx->retOut;  
  for (count = 0; count < requests; count++)
  {
    /* get the cobId from its index */
    idx = pCtx->retNodeIdxQueue[pCtx->retOut%CO_RET_QUEUE_LENGTH];
    pNodeDesc = &(pCtx->pNodeList[idx]);
    pNodeStat = &(pCtx->pNodeStat[idx]);
 
    /* check if it's switched-on and no RET has been sent in the current SYNC period */ 
    msk = 1 << idx;
    
    if ((pCtx->nodeSwitchOnMsk & msk) && (pNodeStat->retDistInSyncCyc == FALSE))
    {
      /* prepare a RET message */
      msgId = pNodeDesc->cobIdRet;
      msgRtr = CO_OD_RET_DIST_RTR;
      dataBytes = CO_OD_RET_DIST_DATA_LENGTH;
      
      /* fill in the data part: coarse (32bit) + fine (16LSB) */
      memcpy (data, &pCtx->cucC32BNextSyncCycle, sizeof (Uint));
      memcpy (data+sizeof(Uint), &pCtx->cucF16BNextSyncCycle, sizeof (U16));
      /* On payload bus, padding bytes need to be sent */
      if (pCtx->busId == CANBUS_ID_PL)
      {
        data[sizeof(Uint)+sizeof(U16)] = 0;
        data[sizeof(Uint)+sizeof(U16)+1] = 0;
        dataBytes += sizeof (U16);
      }
      
      /* build the RET message in the list */
      coSts = CanOpenBus_putMsgInList(pCtx->busId, pCtx->list,
                                      msgId, msgRtr, dataBytes, data);
      /* %COVER%FALSE% Defensive programming - ERROR_REPORT macro called here. */
      if (coSts == E_COSTATUS_OK)
      {
        /* one more message in the list */
        pCtx->msgsInCurList++;
        
        /* get its index in the node list and update its state */
        pNodeStat->retTotal++;
      }
      else
      {
        /* unexpected error: messag must be sent */
        ERROR_REPORT(SW_ERROR, pCtx->busId, coSts, pCtx->chanId);
      }
      
      /* mark the node as already handled in the current SYNC period */
      pNodeStat->retDistInSyncCyc = TRUE;
      
      /* update the HK statistics */
      pCtx->retMsgs++;
    }
    pCtx->retOut++;
  } /* for */
  
  /* END: protect the modification of RET request queues */
  ResourceLock_unlock(&pCtx->lock);
}

/***********************************************************************/
/**
 * @brief  pdoTcMsgsBuild - Build up PDO TC messages
 * 
 * @reentrant The variable @ref CoMgr_ctx is protected for exclusive
 * read/write access by a mutex @ref CoCtx.lock.
 *
 * @param[in,out] pCtx  The pointer to the context of a CAN Bus Manager.
 * @return N/A
 *
 ***********************************************************************/ 
PRIVATE void pdoTcMsgsBuild(CoCtx *pCtx)
{
  CoStatus coSts;
  CoMsg *pCoMsg;
  Uint idx;
  Uint requests;
  Uint count;
  Uint built;
  NodeStat *pNode;
  Uint nodeIdx;
  Bool go = TRUE;
  Uint k;
  Uint countIncr = 0;
  
  /* BEGIN: protect the modification of PDO TC request queues */
  ResourceLock_lock(&pCtx->lock, RESOURCELOCK_NO_TIMEOUT);
  
  /* process requests up to CO_PDO_TC_IN_CYC requests */
  built = 0;
  requests = MIN((pCtx->pdoTcIn - pCtx->pdoTcOut), CO_PDO_TC_IN_CYC/2);
  for (count = 0; ((count < requests) && go); count+=(countIncr+1))
  {
    countIncr = 0;
    
    /* get the next command from requestQueue */
    idx = pCtx->pdoTcOut%CO_PDO_TC_QUEUE_LENGTH;
    pCoMsg = &pCtx->pdoTcQueue[idx];
    
    /* on more check before sending */
    /* build the message in the list */
    coSts = CanOpenBus_putMsgInList(pCtx->busId, pCtx->list,
                                    pCoMsg->cobId, pCoMsg->rtr, pCoMsg->dataBytes, pCoMsg->data);
    /* %COVER%FALSE% Defensive programming - ERROR_REPORT macro called here.  
     * By design, there cannot be more messages to generate (limited by PDO TC queue size) than the 
     * max size of the send list which is currently limited to the maximum number of messages 
     * that can be transmitted in 10ms. */
    if (coSts == E_COSTATUS_OK)
    {
      /* one more message in the list */
      pCtx->msgsInCurList++;

      /* get its index in the node list and update its state */
      nodeIdx = pCtx->nodeToBusNodeIdx[CO_NODE_TO_BASE(pCoMsg->cobId)];
      pNode = &pCtx->pNodeStat[nodeIdx];
      pNode->pdoTcTotal++;
      
      /* For nodes needing dummy PDO */
      /* take into account a dummy PDO as a request to limit the number of PDO sent in the cycle */ 
      countIncr += pCtx->pNodeList[nodeIdx].needDummyPdo; 
      requests = MIN (requests + pCtx->pNodeList[nodeIdx].needDummyPdo, CO_PDO_TC_IN_CYC/2);
      for (k=0; k<pCtx->pNodeList[nodeIdx].needDummyPdo; k++)
      {
        coSts = CanOpenBus_putMsgInList(pCtx->busId, pCtx->list,
                                        CO_DUMMY_PDO_COBID, CANOPEN_WPDO_RTR, CANOPEN_PDO_DATA_LENGTH_MAX, dummyPdo);
        /* %COVER%FALSE% Defensive programming - ERROR_REPORT macro called here.  
         * By design, there cannot be more messages to generate (limited by PDO TC queue size) than the 
         * max size of the send list which is currently limited to the maximum number of messages 
         * that can be transmitted in 10ms. */        
        if (coSts == E_COSTATUS_OK)
        {
          pCtx->msgsInCurList++;
          /* increment the number of dummy PDO sent on the bus */
          CoMgr_hkArea[pCtx->busId].dummyPdoCnt++;
        }
      }
      
      /* go to the next one */
      pCtx->pdoTcOut++;
      
      built++;
    }
    else
    {
      /* unexpected error or coSts == E_COSTATUS_MSG_LIST_FULL transmission list full */
      go = FALSE;
      ERROR_REPORT(SW_ERROR, pCtx->busId, coSts, pCtx->chanId);
    }
  } /* for */
  
  /* update HK statistics */
  pCtx->pdoMsgsOut += built;
  
  /* END: protect the modification of PDO TC request queues */
  ResourceLock_unlock(&pCtx->lock);
}

/***********************************************************************/
/**
 * @brief  hbMsgProcess - handle received heart beat message from slave
 * @param[in,out] pCtx    The pointer to the context of a CAN Bus Manager.
 * @param[in] entryState  The next state of the CAN Bus Manager.
 * @param[in] pCoMsg      The pointer to CAN Bus message.
 * @return N/A
 *
 ***********************************************************************/ 
PRIVATE void hbMsgProcess(CoCtx *pCtx, CoState entryState, CoMsg *pCoMsg)
{
  U08 baseNodeId;
  U08 nodeIdx;
  NodeStat *pNode;
  Uint nodeMsk;
  
  /* get its index in the node list and update its state */
  baseNodeId = CO_NODE_TO_BASE(pCoMsg->cobId);
  nodeIdx = pCtx->nodeToBusNodeIdx[baseNodeId];
  
  /* update states for the node */
  pNode = &pCtx->pNodeStat[nodeIdx];
  pNode->nodeState = pCoMsg->data[CANOPEN_HB_DATA_STS_IDX];
  
  pNode->lastCycle = pCtx->cycles;
  pNode->hbTotal++;

  nodeMsk = 1 << nodeIdx;
  
  /* if it is unexpected HB message */
  if ((nodeMsk & pCtx->nodeSwitchOnMsk) == 0)
  {
    if (pCtx->pNodeList[nodeIdx].isUhf != 0)
    {
      pCtx->uhfHailNode = baseNodeId;
      pCtx->uhfHailCycle = pCtx->cycles;
      pNode->uhfHail = TRUE;
      pNode->uhfHailCycle = pCtx->cycles;
    }
    else
    {
      pCtx->unexpHbNode = baseNodeId;
      pCtx->unexpHbCycle = pCtx->cycles;
    }
  }
  if ((entryState == E_COSTATE_PRE_OP) || (entryState == E_COSTATE_OP))
  {
    pCtx->nodeHbRegMsk |= nodeMsk & pCtx->nodeSwitchOnMsk;
  }
  
  pCtx->firstHbReceived = TRUE;
}
/***********************************************************************/
/**
 * @brief  pdoMsgProcess - handle received PDO TM message
 * 
 * @reentrant The variable @ref CoMgr_ctx is protected for exclusive
 * read/write access by a mutex @ref CoCtx.lock.
 * 
 * @param[in,out] pCtx  The pointer to the context of a CAN Bus Manager.
 * @param[in] pCoMsg    The pointer to CAN Bus message.
 * @return N/A
 *
 ***********************************************************************/ 
PRIVATE void pdoMsgProcess(CoCtx *pCtx, CoMsg *pCoMsg)
{
  Uint nodeIdx;
  Uint tmIdx;
  U32  bspSdoSize;
  NodeStat *pNodeStat;
  PdoTmArea *pTmCobId;
  CoPdoTmRecord *pTmRecord;
  CobIdDesc *pCobIdDescList;
  CobIdDesc *pCobIdDesc;
  
  SdoBlkUlReq *pBlkUlReq;
  
  /* identify the nodeID check if it's validated one */
  pCobIdDescList = CoMgr_cobIdList[pCtx->busId];
  pCobIdDesc = &pCobIdDescList[pCoMsg->cobId];
  
  /* %COVER%FALSE% Defensive programming - CAN slave protocol error. */
  if (pCobIdDesc->rxGroup != 0)
  {
    /* get the TM buffer list for this cobId and locate one to store the current TM */
    nodeIdx = pCobIdDesc->nodeIdx;
    tmIdx = pCobIdDesc->tpdoIdx;
    pTmCobId = &pCtx->pPdoTmArea[tmIdx];
    pTmRecord = &pTmCobId->tmQueue[pTmCobId->written % CO_PDO_TM_PER_COBID];
    
    pTmRecord->dataSize = pCoMsg->dataBytes;
    
    memcpy (pTmRecord->data, pCoMsg->data, pCoMsg->dataBytes);
    
    /* Note: SPARC Big-endian data organisation matches RET package format */
    memcpy (pTmRecord->cucCoarse, &pCtx->cucC32B, CO_CUC_COARSE_FIELD_LENGTH);
    memcpy (pTmRecord->cucFine, &pCtx->cucF16B, CO_CUC_FINE_FIELD_LENGTH);
    pTmCobId->written++;
    
    /* for a Buffer Support PDO, additional specific operation */
    pNodeStat = &pCtx->pNodeStat[nodeIdx];
    pNodeStat->pdoTmTotal++;
    
    /* only the specified cobId with correct status and packet size will be checked for Buffer Support PDO */
    if ((pNodeStat->bufSupPdoCobId == pCoMsg->cobId) && (pCoMsg->dataBytes == CO_OD_BSP_DATA_LENGTH) &&
        (CO_OD_BSP_STS_VALID(pCoMsg->data[CO_OD_BSP_BUF_STS_OFFSET])))
    {
      /* get the size field from little endian format */
      memcpy (&bspSdoSize, pCoMsg->data+CO_OD_BSP_NBYTES_OFFSET, sizeof(U32));
      /* no swap needed as in buffer support PDO, size is stored in big-endian format */
      
      /* for upload */
      if (pCoMsg->data[CO_OD_BSP_BUF_STS_OFFSET] == CO_OD_BSP_STS_READY_UL)
      {
        /* %COVER%FALSE% Defensive programming - CAN slave protocol error. */
        if ((bspSdoSize != 0) && (CO_OD_BSP_TYPE_VALID(pCoMsg->data[CO_OD_BSP_CONTENT_OFFSET])))
        {
          ResourceLock_lock(&pCtx->lock, RESOURCELOCK_NO_TIMEOUT);
          
          /* check OK, it is a valid BSP request, store the request in the queue */
          pBlkUlReq = &pCtx->sdoBlkUlReq[pCtx->sdoUlIn%CO_BLK_SDO_UL_QUEUE_LENGTH];
          
          pBlkUlReq->pdoReq = TRUE;
          pBlkUlReq->node = CO_NODE_TO_BASE(pCoMsg->cobId);
          nodeIdx = pCtx->nodeToBusNodeIdx[pBlkUlReq->node];
          pNodeStat = &pCtx->pNodeStat[nodeIdx]; 
          pBlkUlReq->objIdx = pNodeStat->bufSupPdoIdxUpl;
          pBlkUlReq->objSubIdx = pNodeStat->bufSupPdoSubIdxUpl;
        
          pBlkUlReq->nBytes = bspSdoSize;
          
          memcpy (pBlkUlReq->pdoData, pCoMsg->data, CO_OD_BSP_DATA_LENGTH);
          
          pCtx->sdoUlReqCnt++;
          pBlkUlReq->reqCnt = pCtx->sdoUlReqCnt;
          
          pCtx->sdoUlIn++;  
          ResourceLock_unlock(&pCtx->lock);
        }
      } /* Upload */
      else 
      {
        /* %COVER%FALSE% Defensive programming - CAN slave protocol error. */
        if (pCoMsg->data[CO_OD_BSP_BUF_STS_OFFSET] == CO_OD_BSP_STS_READY_DL)
        {
          /* OK, now, enable the next Block SDO download transfer to start */
          pCtx->blkSdoDlLastSlotsCnt = 0;
        }
      } /* Download */
    } /* buffer support PDO */
  }
}


/***********************************************************************/
/**
 * @brief  coStateToHbState - translate CoState to HB state
 * 
 * @param[in] coState The CO state state
 * @return corresponding HB state value
 *
 ***********************************************************************/ 
PRIVATE Uint coStateToHbState(CoState coState)
{
  /*
   * CO_HB_ST_UNKNOWN:       Off = 0, not HB state
   * CANOPEN_HB_ST_BOOT_UP:  Init = 1
   * CANOPEN_HB_ST_PRE_OP:   Pre_Op = 2
   * CANOPEN_HB_ST_OP:       Op = 3
   * CANOPEN_HB_ST_STOP:     Stop = 4
   * CO_HB_ST_UNKNOWN:       end of list, not a HB state
   */
  static Uint hbStateList[CO_STATE_LIST_LENGTH] =
  {
    CO_HB_ST_UNKNOWN,       
    CANOPEN_HB_ST_BOOT_UP,  
    CANOPEN_HB_ST_PRE_OP,   
    CANOPEN_HB_ST_OP,       
    CANOPEN_HB_ST_STOP,     
    CO_HB_ST_UNKNOWN        
  };
  return (hbStateList[MIN(coState, CO_STATE_LIST_LENGTH-1)]);
}

/***********************************************************************/
/**
 * @brief getMissedHb - get missed HB
 * Update HK for missed Hearbeats.
 * 
 * @param[in,out] pCtx  The pointer to the context of a CAN Bus Manager.
 * 
 *
 ***********************************************************************/ 
PRIVATE void getMissedHb(CoCtx *pCtx) 
{
  Uint idx;
  U32 missedNodeMsk;
  Bool found = FALSE;
  
  
  /* get missed HB nodes */
  missedNodeMsk = pCtx->nodeSwitchOnMsk ^ pCtx->nodeHbRegMsk;
  for (idx = 0; idx < CO_SLV_NODES_MAX; idx++)
  {
    if ((missedNodeMsk & (1<<idx)) != 0)
    {
      if (found == FALSE)
      {
        /* record one of missed HB node */
        pCtx->toggleNode = pCtx->pNodeList[idx].baseNid;
        pCtx->toggleCycle = pCtx->cycles;
        found = TRUE;
      }
    }
  }
}

/***********************************************************************/
/**
 * @brief processTxMsgList - Transmission at each slot
 * 
 * @param[in,out] pCtx  The pointer to the context of a CAN Bus Manager.
 * @param[in] list  The list number to transmit.
 * 
 *
 ***********************************************************************/ 
PRIVATE void processTxMsgList(CoCtx *pCtx, U32 list)
{
  CoStatus coSts;

  coSts = CanOpenBus_txMsgList(pCtx->busId, list, pCtx->msgsInCurList,
                               pCtx->cycles, pCtx->slotInCycle);

  /* update the HK parameters for active Pm & Ext Can */ 
  switch (coSts)
  {
  case E_COSTATUS_OK:
  case E_COSTATUS_BUS_BUSY:
    /* nothing to do */
    break;

  case E_COSTATUS_ACTPM_ERROR:
    pCtx->actPmHealth = E_CO_PM_HEALTH_FAIL;
    break;

  case E_COSTATUS_EXTCAN_ERROR:
    pCtx->extCanHealth = E_CO_EXTCAN_HEALTH_FAIL;
    break;

  default:
    /* %COVER%STMT% Defensive programming: all error cases already covered
     * no need for ERROR_REPORT */
    break;
  } /* switch */
}

/*------------------ ooOoo End of file ooOoo --------------------------*/
