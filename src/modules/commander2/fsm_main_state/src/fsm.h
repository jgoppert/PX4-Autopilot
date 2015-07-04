#pragma once

#include "inttypes.h"

typedef enum {
	FSM_EVENT_UNINIT,
	FSM_EVENT_ARM,
	FSM_EVENT_DISARM,
	FSM_EVENT_SAFETY,
	FSM_EVENT_MAX
} eventID_t;

typedef enum {
	FSM_STATE_UNINIT,
	FSM_STATE_SAFE,
	FSM_STATE_STANDBY,
	FSM_STATE_ARMED,
	FSM_STATE_MAX,
} stateID_t;

typedef struct fsm_t Fsm_t;
typedef struct state_t State_t;
typedef int (*Handler_t)(Fsm_t *);

struct fsm_t {
	stateID_t stateID;
	Handler_t handlerEventArm;
	Handler_t handlerEventDisarm;
	Handler_t handlerEventSafety;
};

int fsmHandleEvent(Fsm_t * fsm, eventID_t event);
int fsmInit(Fsm_t * fsm);
stateID_t fsmGetStateID(Fsm_t * fsm);
const char * fsmGetStateName(Fsm_t * fsm);

/***
 * You shouldn't use this, it is only for
 * testing. You should never directly set the
 * FSM state.
 */
int __fsmTransition(Fsm_t * fsm, stateID_t stateID);
