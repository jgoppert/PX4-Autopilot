#include "fsm.h"
#include "assert.h"

static const char * stateNames[] = {
	"SAFE", "STANDBY", "ARMED"
};

// transitions
static int fsmTransitionStandby(Fsm_t * fsm);
static int fsmTransitionArmed(Fsm_t * fsm);
static int fsmTransitionSafe(Fsm_t * fsm);

// init
static int fsmInitState(Fsm_t * fsm);

// generic state event handlers
static int fsmNullHandler(Fsm_t * fsm);

// safe state event handlers
static int fsmSafeHandlerEventSafety(Fsm_t * fsm);

// standby state event handlers
static int fsmStandbyHandlerEventSafety(Fsm_t * fsm);
static int fsmStandbyHandlerEventArm(Fsm_t * fsm);

// armed state event handlers
static int fsmArmedHandlerEventSafety(Fsm_t * fsm);
static int fsmArmedHandlerEventDisarm(Fsm_t * fsm);

/**
 * Public functions
 */
int fsmHandleEvent(Fsm_t * fsm, eventID_t event) {
	assert(1);
	stateID_t old = fsm->stateID;
	switch(event) {
		case FSM_EVENT_ARM:
			fsm->handlerEventArm(fsm);
			break;
		case FSM_EVENT_DISARM:
			fsm->handlerEventDisarm(fsm);
			break;
		case FSM_EVENT_SAFETY:
			fsm->handlerEventSafety(fsm);
			break;
		default:
			break;
	};
	stateID_t new =  fsm->stateID;

	// armed and safety toggle -> safe
	assert(!(old == FSM_STATE_ARMED && event == FSM_EVENT_SAFETY)
			|| new == FSM_STATE_SAFE);

	// safe and arm request -> safe
	assert(!(old == FSM_STATE_SAFE && event == FSM_EVENT_ARM)
			|| new == FSM_STATE_SAFE);
	return 0;
};

int fsmInit(Fsm_t * fsm) {
	fsmInitState(fsm);
	fsmTransitionSafe(fsm);
	return 0;
}

const char * fsmGetStateName(Fsm_t * fsm) {
	return stateNames[fsmGetStateID(fsm)];
}

stateID_t fsmGetStateID(Fsm_t * fsm) {
	return fsm->stateID;
}

/**
 * Generic state event handlers
 */
static int fsmNullHandler(Fsm_t * fsm) {
	(void)(fsm);
	return 0;
}

/*
 * By default, don't allow any transitions
 */
static int fsmInitState(Fsm_t * fsm) {
	fsm->stateID = FSM_STATE_UNINIT;
	fsm->handlerEventArm = fsmNullHandler;
	fsm->handlerEventDisarm = fsmNullHandler;
	return 0;
}

/**
 * Safety state handlers
 */
static int fsmSafeHandlerEventSafety(Fsm_t * fsm) {
	fsmTransitionStandby(fsm);
	return 0;
}

// standby state
static int fsmStandbyHandlerEventSafety(Fsm_t * fsm) {
	fsmTransitionSafe(fsm);
	return 0;
}

static int fsmStandbyHandlerEventArm(Fsm_t * fsm) {
	fsmTransitionArmed(fsm);
	return 0;
}

// armed state
static int fsmArmedHandlerEventSafety(Fsm_t * fsm) {
	fsmTransitionSafe(fsm);
	return 0;
}

static int fsmArmedHandlerEventDisarm(Fsm_t * fsm) {
	fsmTransitionStandby(fsm);
	return 0;
}

// transitions
static int fsmTransitionSafe(Fsm_t * fsm) {
	fsmInitState(fsm);
	fsm->stateID = FSM_STATE_SAFE;
	fsm->handlerEventSafety = fsmSafeHandlerEventSafety;
	return 0;
}

static int fsmTransitionStandby(Fsm_t * fsm) {
	fsmInitState(fsm);
	fsm->stateID = FSM_STATE_STANDBY;
	fsm->handlerEventSafety = fsmStandbyHandlerEventSafety;
	fsm->handlerEventArm = fsmStandbyHandlerEventArm;
	return 0;
}

static int fsmTransitionArmed(Fsm_t * fsm) {
	fsmInitState(fsm);
	fsm->stateID = FSM_STATE_ARMED;
	fsm->handlerEventDisarm = fsmArmedHandlerEventDisarm;
	fsm->handlerEventSafety = fsmArmedHandlerEventSafety;
	return 0;
}

// access to state transitions for testing
int __fsmTransition(Fsm_t * fsm, stateID_t stateID) {
	switch(stateID) {
		case FSM_STATE_SAFE:
			return fsmTransitionSafe(fsm);
		case FSM_STATE_ARMED:
			return fsmTransitionArmed(fsm);
		case FSM_STATE_STANDBY:
			return fsmTransitionStandby(fsm);
		default:
			return -1;
	}
}
