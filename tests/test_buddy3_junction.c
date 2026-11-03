#include <assert.h>
#include <stdio.h>
#include <stdint.h>
#include "buddy3_junction.h"
static buddy3_junction_output_t step(buddy3_junction_t *s, unsigned side,
    uint32_t now, float l, float r) {
    return buddy3_junction_step(s,side,now,l,r,true,true,12000);
}
int main(void) {
    buddy3_junction_t s;
    buddy3_junction_output_t o;
    buddy3_junction_reset(&s);
    o=step(&s,0,0,0,0); assert(!o.handled);
    o=step(&s,3,10,0,0); assert(o.handled&&o.phase==B3J_CONFIRM&&o.left==o.right);
    o=step(&s,0,20,1,1); assert(!o.handled&&s.phase==B3J_IDLE);
    o=step(&s,3,100,0,0); assert(o.phase==B3J_CONFIRM);
    o=step(&s,3,130,2,2); assert(o.phase==B3J_CROSS);
    o=step(&s,1,150,5,5); assert(o.phase==B3J_CROSS&&o.left==o.right);
    o=step(&s,2,170,10,10); assert(o.phase==B3J_CROSS&&o.left==o.right);
    o=step(&s,0,180,20,20); assert(o.phase==B3J_CROSS);
    o=step(&s,0,190,24,24); assert(o.phase==B3J_CROSS);
    o=step(&s,0,200,25,25); assert(o.phase==B3J_CLEAR);
    o=step(&s,1,210,26,26); assert(!o.handled);
    buddy3_junction_reset(&s); step(&s,3,0,0,0);
    o=step(&s,3,30,5,7); assert(o.right<o.left&&o.right>=8000);
    buddy3_junction_reset(&s); step(&s,3,0,0,0);
    o=step(&s,3,30,7,5); assert(o.left<o.right&&o.left>=8000);
    buddy3_junction_reset(&s); step(&s,3,0,0,0);
    step(&s,3,30,2,2); step(&s,0,50,20,20);
    step(&s,1,60,23,23); /* A new branch edge invalidates early white clearance. */
    o=step(&s,0,70,25,25); assert(o.phase==B3J_CROSS);
    o=step(&s,0,80,30,30); assert(o.phase==B3J_CLEAR);
    buddy3_junction_reset(&s); step(&s,3,0,0,0);
    o=step(&s,3,B3_JUNCTION_MAX_MS,0,0); assert(o.stopped&&o.left==0&&o.right==0);
    o=step(&s,0,B3_JUNCTION_MAX_MS+10,2,2); assert(o.stopped);
    buddy3_junction_reset(&s); step(&s,3,0,0,0);
    o=step(&s,3,100,100,100); assert(o.stopped);
    buddy3_junction_reset(&s); step(&s,3,0,0,0);
    o=buddy3_junction_step(&s,3,10,0,0,false,true,12000); assert(o.stopped);
    buddy3_junction_reset(&s); step(&s,3,0,10,10);
    o=step(&s,3,10,0,0); assert(o.stopped); /* Odometry reset/reverse. */
    buddy3_junction_reset(&s); step(&s,3,0,0,0);
    o=step(&s,3,30,1,22); assert(o.stopped); /* Excessive heading deviation. */
    buddy3_junction_reset(&s);
    o=buddy3_junction_step(&s,3,0,0,0,true,false,12000);
    assert(!o.handled); /* Existing corner/recovery keeps ownership. */
    buddy3_junction_reset(&s); step(&s,3,UINT32_MAX-20,0,0);
    o=step(&s,3,9,1,1); assert(o.phase==B3J_CROSS);
    puts("PASS: junction debounce, straight commitment, branch suppression, distance exit, heading correction, fault/distance/time stops, wrap");
    return 0;
}
