#ifndef SCHEDULER_H
#define	SCHEDULER_H

#include <xc.h> // include processor files - each processor file is guarded.  

typedef struct {
    int n; // number of HB loops elapsed since last time the task was executed
    int N; // ceiling to reach before exec the task
    int enable;
    void (*f)(void *);
    void* params;
} heartbeat;

void scheduler(heartbeat schedInfo[], int nTasks);

#endif	/* SCHEDULER_H */

