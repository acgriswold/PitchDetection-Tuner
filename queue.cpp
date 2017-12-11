/*
Queue.cpp
By: Matt Westjohn. Version 2. 11/2/2017 = Interrupt handling for put and get
By Sean Carroll, Trine U, v. 2017_10_16, October 2017 = Initial Queue Setup.
Purpose: Define queues for use in the pitch sensor
*/
#include "mbed.h"
#include "queue.h"

Queue::Queue() {
    head = buffer;
    tail = buffer;
}
    
bool Queue::get(float32_t *msg) {
    int copy_of_primask = __get_PRIMASK();
    __disable_irq();
    if (tail == head) {
        msg = 0;
        if(copy_of_primask){
             return false;
        } else {
            __enable_irq();
            return false;
            }
    } else {
        *msg = *head;
        head++;
        if (head >= (buffer + QBUFSIZE)) {
            head = buffer;
        }
        if(copy_of_primask){
            return true;
        } else {
            __enable_irq();
            return true;
        }
    }
}

bool Queue::put(float32_t *msg) {
    int copy_of_primask = __get_PRIMASK();
    __disable_irq();
    if ((head-tail == 1) || (tail-head == QBUFSIZE-1)) {
        msg = 0;
        if(copy_of_primask){
            return false;
        } else {
            __enable_irq();
            return false;
        }
    } else {
        *tail = *msg;
        tail++;
        if (tail >= (buffer + QBUFSIZE)) {
            tail = buffer;
        }
        if(copy_of_primask){
            return true;
        } else {
            __enable_irq();
            return true;
        }
    }
}
