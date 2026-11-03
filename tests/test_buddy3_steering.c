#include <assert.h>
#include <stdio.h>
#include <stdint.h>
#include "buddy3_steering.h"

static buddy3_steering_output_t step(buddy3_steering_t *s, unsigned stage,
                                      unsigned side, uint32_t now) {
    return buddy3_steering_step(s, stage, side, now, 12000, 2000);
}
int main(void) {
    buddy3_steering_t s;
    buddy3_steering_output_t o;
    buddy3_steering_reset(&s);
    o=step(&s,1,1,0); assert(o.left==10000 && o.right==14000 && o.phase==B3_MICRO);
    o=step(&s,1,1,1000); assert(o.phase==B3_MICRO);
    o=step(&s,1,0,1010); assert(o.left==12000 && o.right==12000);
    o=step(&s,1,0,9000); assert(o.phase==B3_TRACK);
    buddy3_steering_reset(&s);
    o=step(&s,1,2,0); assert(o.right==10000 && o.left==14000);
    o=step(&s,1,3,10); assert(o.phase==B3_AMBIGUOUS && o.right<o.left);
    o=step(&s,1,3,160); assert(o.phase==B3_STOP && o.left==0 && o.right==0);
    o=step(&s,1,0,200); assert(o.phase==B3_STOP);
    buddy3_steering_reset(&s);
    step(&s,2,1,0); o=step(&s,2,1,300);
    assert(o.phase==B3_CORNER && o.left==0 && o.right==12000);
    o=step(&s,2,0,310); assert(o.phase==B3_STOP);
    buddy3_steering_reset(&s);
    step(&s,3,2,0); step(&s,3,2,300);
    o=step(&s,3,0,310); assert(o.phase==B3_SEEK && o.right==0);
    o=step(&s,3,0,800); assert(o.phase==B3_SEEK);
    step(&s,3,2,810);
    o=step(&s,3,0,820); assert(o.phase==B3_ALIGN && o.left==6000 && o.right==6000);
    step(&s,3,0,920);
    o=step(&s,3,0,930); assert(o.phase==B3_TRACK);
    buddy3_steering_reset(&s);
    step(&s,3,1,0); step(&s,3,1,300); step(&s,3,0,310);
    o=step(&s,3,0,1510); assert(o.phase==B3_STOP);
    buddy3_steering_reset(&s);
    step(&s,2,2,0); step(&s,2,2,300);
    o=step(&s,2,2,2100); assert(o.phase==B3_STOP);
    buddy3_steering_reset(&s);
    step(&s,2,1,UINT32_MAX-100);
    o=step(&s,2,1,199); assert(o.phase==B3_CORNER);
    /* Requested stronger correction must not hit the old 20%-of-base cap. */
    buddy3_steering_reset(&s);
    o=buddy3_steering_step(&s,1,1,0,12000,4800);
    assert(o.left==7200 && o.right==16800);
    buddy3_steering_reset(&s);
    o=buddy3_steering_step(&s,1,2,0,12000,4800);
    assert(o.right==7200 && o.left==16800);
    /* Preserve main's outer boost while clamping forward PWM safely. */
    buddy3_steering_reset(&s);
    o=buddy3_steering_step(&s,1,1,0,12000,20000);
    assert(o.left==0 && o.right==24000);
    buddy3_steering_reset(&s);
    o=buddy3_steering_step(&s,1,2,0,25000,20000);
    assert(o.left==32767 && o.right==5000);
    buddy3_steering_reset(&s);
    o=buddy3_steering_step(&s,1,1,0,12000,-2000);
    assert(o.left==12000 && o.right==12000);
    o=step(&s,1,0,16000); assert(o.phase==B3_TRACK);
    o=step(&s,1,0,120000); assert(o.phase==B3_TRACK);
    puts("PASS: stronger symmetric correction, nonnegative clamp, continuous tracking, corner/recovery/ambiguous stops, timer wrap");
    return 0;
}
