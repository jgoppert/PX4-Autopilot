#include <stdlib.h>

#include "cbmc_stub.h"

unsigned int nondet_uint() {
    return rand();
}

void __CPROVER_assume(int test) {
    (void)test;
}
