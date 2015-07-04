#include "fsm.h"
#include <assert.h>

unsigned int nondet_uint();

int main(int argc, char * argv[]) {
	(void)(argc);
	(void)(argv);
	Fsm_t fsm;
	fsmInit(&fsm);

	stateID_t stateID = (stateID_t)nondet_uint();
	__CPROVER_assume (stateID > 0 && stateID < FSM_STATE_MAX);
	__fsmTransition(&fsm, stateID);

	eventID_t  eventID = (eventID_t)nondet_uint();
	__CPROVER_assume (eventID > 0 && eventID < FSM_EVENT_MAX);

	fsmHandleEvent(&fsm, eventID);
	return 0;
};
