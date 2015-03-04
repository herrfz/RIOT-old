/*
 * Copyright (C) 2014 Triagnosys
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @ingroup     examples
 * @{
 *
 * @file
 * @brief       Latency Riot application
 *
 * @author      Xavier Perez <xavier.simo@triagnosys.com>
 *
 * @}
 */

//#include <stdio.h>
#include "board.h"
#include "vtimer.h"
#include "uart.h"
#include "gpio.h"
#include "thread.h"
#include "msg.h"
#include "ps.h"

/*Define test parameters*/
#define VITMER_MSG		1 //Select test latency

#define DIFF_PRIORITY	1 //Select same priority or thread latency with different priority
#define THREADS			3 //Select number of threads
#define THREAD_LATENCY	5  //Define PID thread latency. 4 thread means PID[3..6]
#define REPEATS			100
#define INTERVAL		1000 //Define interval in us

char stack[THREADS][KERNEL_CONF_STACKSIZE_MAIN]; /*Define multiple stack for all threads*/

/*Define time varialbes*/
#define MSEC (1000)
#define SEC (1000 * MSEC)

/*Define latency variables*/
#define MAX_LATENCY 1000
int latency[MAX_LATENCY] = {0}; /* define vector for latency */
int count[MAX_LATENCY] = {0};
int overflow = 0;

/*Define thread counters*/
int thread[THREADS] = {0}; //save the repetitions to execute each thread
int num_thread[THREADS +3];

/*Start funtion for all threads*/
void *second_thread(void *arg)
{
	(void) arg;

	kernel_pid_t pid;

	/*inizialize iteration parameters*/
	int test_repeats = REPEATS;
	int iteration = 0;

	/*define time variables*/
	timex_t now;
	timex_t next = timex_set(0, 0);
	timex_t diff;

	/*Define message to send*/
	msg_t m; //Allocated the message in thread
	m.content.ptr = "latency finish" ;

	/*define sleep variables*/
	timex_t interval = timex_set(0, INTERVAL); // set sleep interval to 1000 us = 1 ms
	vtimer_now(&now);
	next = timex_add(now, interval);

#if	VITMER_MSG
	int *msg_a; //pointer sending message
	msg_a = 0;
	msg_t m_a;  //pointer receiving message
	int temp = 0;
	vtimer_t timer;
#endif

	while(1){
		pid = thread_getpid();
		thread[num_thread[pid]]++;
		iteration++;

		if(iteration < test_repeats){  //while there are iterations
			if (pid == THREAD_LATENCY){

	#if VITMER_MSG
				vtimer_set_msg(&timer, interval, thread_getpid(), 0, msg_a );
				for (temp=0; temp < 100 ;temp++){}
				msg_receive(&m_a);
	#else
				vtimer_usleep(interval.microseconds); // sleep
	#endif
				vtimer_now(&now); // get actual time after sleep (:=now)
				diff = timex_sub(now, next); // compute difference between theoretical time after sleep (:=next)
                                     // and actual time after sleep (:=now)
				if (diff.microseconds > MAX_LATENCY - 1) // guard for overflow
					overflow++;
				else
					count[diff.microseconds] += 1; // store diff result for statistics
					//printf("the latency result is: %"PRIu32" \n", diff.microseconds);
				vtimer_now(&now); // get actual time for next iteration (:=now)
				next = timex_add(interval, now); // update theoretical time for next iteration
			} else{
				thread_yield(); //the rest threads force to change
			}
		} else	{  //iterations finish -> finish all threads
			if (pid == THREAD_LATENCY)
			msg_send(&m, 2); // thread latency send send finish message to main (pid=2)
			break;
		}
	}
	return NULL;
}

/*Start main program*/
int main(void)
{
	/*vector variable*/
	int j = 0;
	int c = 0;

	/*print values*/
	int n = 0;
	int d = 0;

	/*Maximum and minimum values*/
	int max_c = count[0];
	int max_l = latency[0];
	int max_time_c = count[0];
	int max_time_l = latency[0];
	int min_time_c = 0;
	int min_time_l = 0;
	int min_time_flag = 1;

	/*Define where are receiving message from thread*/
	msg_t m_receive; //Allocated the receive message in main

	/*Init program*/
	printf("# ********************************************* \n");
	printf("# ************* Latency RIOT test ************* \n");
	printf("# ********************************************* \n");
	printf("# config parameters:\n");
	//printf("# Interval sleep: %"PRIu32" sec and %"PRIu32" micro\n", interval.seconds, interval.microseconds);
	printf("# Samples: %i\n", MAX_LATENCY);
	//printf("# Repetitions: %i\n", test_repeats);
#ifdef DIFF_PRIORITY
	printf("# numeber threads: %i (with different priority)\n", THREADS);
#else
	printf("# numeber threads: %i (with same priority)\n", THREADS);
#endif
	printf("# ********************************************* \n");

	/*Init latency vector*/
	for(j = 0 ; j < MAX_LATENCY; j++) {
		latency[j] = c;
		c += 1;
	}
	printf("# vector init\n");
	vtimer_usleep(SEC);

	/*Define multiple threads*/
	int th=0;
	num_thread[0 & 1 & 2] = 0; //cancel  num_thread for  main thread and idle thread.
	kernel_pid_t pid;
	char buffer[THREADS][11];
#if DIFF_PRIORITY
	int priority = 0; //define different priorities
#endif

/*Creating multiple threads*/
	for(th=0 ; th < THREADS; th++){
		sprintf(buffer[th], "th_back_%d", th);
		//printf("buffer is:%s\n", buffer[th]);
	#if DIFF_PRIORITY
		if (THREAD_LATENCY == th + 3)  //Creating thread latency with major priority
			priority = 2;
		else
			priority = 1;
	#endif

		pid = thread_create(stack[th],
				KERNEL_CONF_STACKSIZE_MAIN,
	#if DIFF_PRIORITY
				PRIORITY_MAIN - priority,  //thread latency with major priority
	#else
				PRIORITY_MAIN -1,   //all the threads with major priority
	#endif
				CREATE_WOUT_YIELD | CREATE_STACKTEST,
				second_thread,
				NULL,
				buffer[th]);
				num_thread[pid]=th;
				//printf("num_threads is: %i\n", num_thread[pid]);
	}
	//thread_print_all();
	msg_receive(&m_receive); //waiting for receive message from main
	printf("#the receive message is: %s\n", m_receive.content.ptr);
	printf("# Test finish\n");
	printf("# print histogram\n");
	vtimer_usleep(SEC);

	/*print out values*/
	for(n = 0; n < MAX_LATENCY ; n++) {
		if(n < 10)
			printf("00%i %i\n", latency[n], count[n]);
		else if(n<100 && n>=10)
			printf("0%i %i\n", latency[n], count[n]);
		else
			printf("%i %i\n", latency[n], count[n]);

		/*Get the maximum repetitions*/
		if (max_c < count[n]) {
			max_c = count[n];
			max_l = latency[n];
		}

		/*Get time values*/
		if (count[n] >= 1){
			/*Get maximum time value*/
			if(max_time_l < latency[n]) {
				max_time_l = latency[n];
				max_time_c = count[n];
			}
			/*Get minimum time value*/
			if(min_time_flag){
				min_time_flag = 0;
				min_time_l = latency[n];
				min_time_c = count[n];
			}
		}
	}
	printf("# MIN time is: %i microseconds in %i repetitions\n", min_time_l, min_time_c);
	printf("# MAX time is: %i microseconds in %i repetitions\n", max_time_l, max_time_c);
	printf("# overflow is: %i\n", overflow);
	printf("# MAX repetitions are: %i repetitions in %i microseconds\n", max_c, max_l);
	for (d=3; d < THREADS + 3; d++)
		printf("#Thread pid %d is executed: %i times\n", d, thread[num_thread[d]]);
	LED_GREEN_OFF; //indicate test finish

	return 0;
}




