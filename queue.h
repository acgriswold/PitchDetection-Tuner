/*
Queue.h
By: Matt Westjohn. Version 2. 11/2/2017 = Interrupt handling for put and get
By Sean Carroll, Trine U, v. 2017_10_16, October 2017 = Initial Queue Setup.
Purpose: Define queues for use in the pitch sensor
*/

#include "mbed.h"
#include "arm_math.h"
#ifndef QUEUE_H
#define QUEUE_H
#define QBUFSIZE 10

class Queue
{
private:
    float32_t buffer[QBUFSIZE];
    float32_t *head;
    float32_t *tail;
    
public:
    Queue();
    bool get(float32_t *msg);
    bool put(float32_t *msg);
};

#endif