set(module work_queue)
set(stack)
set(main)

set(SRCS
	hrt_thread.c
	hrt_queue.c
	hrt_work_cancel.c
	work_thread.c
	work_lock.c
	work_queue.c
	work_cancel.c
	queue.c
	dq_addlast.c
	dq_remfirst.c
	sq_addlast.c
	sq_remfirst.c
	sq_addafter.c
	dq_rem.c
	)

add_library(${module} STATIC EXCLUDE_FROM_ALL ${SRCS})
set_target_properties(${module} PROPERTIES
	COMPILE_FLAGS "-Os")

# vim: set noet fenc=utf-8 ft=cmake ff=unix sts=4 sw=4 ts=4 :
