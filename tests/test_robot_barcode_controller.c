#include <assert.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>
/* Compile the real command/button/handoff controller against fake hardware. */
#include "../firmware/app_program/robot_controller.c"

static bool following, motion, accept_move=true, calibrated=true, button_high=true;
static bool encoder_fault;
static uint8_t stage=1, event;
static unsigned starts, stops, lefts, rights, forwards;
int tm_printf(const unsigned char *fmt, ...) { (void)fmt; return 0; }
void hal_gpio_input_pullup(uint32_t pin) { (void)pin; }
bool hal_gpio_read(uint32_t pin) { (void)pin; return button_high; }
void buddy2_stop(void) { motion=false; ++stops; }
bool buddy2_motion_active(void) { return motion; }
void buddy2_get_motion_state(buddy2_motion_state_t *s) {
    buddy2_motion_state_t empty={0}; *s=empty; s->left_encoder_fault=encoder_fault;
}
void buddy2_set_motor_pwm(int16_t l,int16_t r) { (void)l; (void)r; }
bool buddy2_set_trim(uint8_t l,uint8_t r) { (void)l; (void)r; return true; }
bool buddy2_set_speed_limit(uint8_t p) { (void)p; return true; }
bool buddy2_turn_left(float a,int16_t p) { assert(a==90.0f); (void)p; ++lefts; motion=accept_move; return motion; }
bool buddy2_turn_right(float a,int16_t p) { assert(a==90.0f); (void)p; ++rights; motion=accept_move; return motion; }
bool buddy2_move_forward(float d,int16_t p) { assert(d==300.0f); (void)p; ++forwards; motion=accept_move; return motion; }
void buddy3_start_line_follow(void) { following=true; ++starts; }
void buddy3_stop_line_follow(void) { following=false; event=0; }
bool buddy3_line_following(void) { return following; }
bool buddy3_take_barcode(uint8_t *v) { *v=event; bool result=event!=0; event=0; return result; }
void buddy3_set_test_stage(uint8_t s) { stage=s; }
uint8_t buddy3_test_stage(void) { return stage; }
bool buddy3_barcode_calibrated(void) { return calibrated; }
bool buddy3_configure_barcode(uint16_t c,int16_t o) { (void)c; (void)o; return calibrated; }
void buddy3_get_calibration(buddy3_calibration_t *c) { buddy3_calibration_t z={0}; *c=z; }
void buddy3_set_calibration(const buddy3_calibration_t *c) { (void)c; }

static void reset(void) {
    robot_stop_all(); robot_controller_init();
    g_barcode_mode=1; stage=1; calibrated=true; accept_move=true;
    button_high=true; g_line_button_count=0; g_line_button_was=false;
    starts=stops=lefts=rights=forwards=0; event=0; encoder_fault=false;
}
static void command(const char *c,uint32_t n) { robot_controller_handle_command(c,n); }
int main(void) {
    reset(); command("line_follow",0); event=1; robot_controller_step(10);
    assert(following && lefts==0); /* Read-only cannot take motor control. */
    for(unsigned cmd=1;cmd<=3;++cmd) {
        reset();
        command("{\"cmd\":\"barcode_mode\",\"mode\":2}",0);
        /* GP21 starts the integrated run without a network connection. */
        button_high=false;
        robot_controller_step(10); robot_controller_step(30); robot_controller_step(50);
        button_high=true; robot_controller_step(70);
        assert(following);
        event=(uint8_t)cmd; robot_controller_step(90);
        assert(!following && motion && starts==1);
        assert(lefts+rights+forwards==1);
        assert((cmd==1&&lefts==1)||(cmd==2&&rights==1)||(cmd==3&&forwards==1));
        motion=false; robot_controller_step(110);
        assert(following && starts==2);
        robot_controller_step(130);
        assert(lefts+rights+forwards==1); /* No repeat from stale frame. */
    }
    reset(); command("barcode_mode 2",0); command("line_follow",0);
    accept_move=false; event=1; robot_controller_step(10);
    assert(!following && !motion && !strcmp(robot_controller_mode_name(),"idle"));
    reset(); command("barcode_mode 2",0); command("line_follow",0);
    event=2; robot_controller_step(10); robot_controller_step(8010);
    assert(!following && !motion && !strcmp(robot_controller_mode_name(),"idle"));
    reset(); command("barcode_mode 2",0); command("line_follow",0);
    event=1; robot_controller_step(10); command("stop",20); robot_controller_step(30);
    assert(!following && starts==1);
    reset(); command("barcode_mode 2",0); command("line_follow",0);
    event=1; robot_controller_step(10);
    button_high=false;
    robot_controller_step(30); robot_controller_step(50); robot_controller_step(70);
    assert(!following && !motion); /* GP21 cancels a barcode move. */
    reset(); calibrated=false; command("barcode_mode 2",0); assert(g_barcode_mode==1);
    reset(); stage=0; command("barcode_mode 2",0); assert(g_barcode_mode==1);
    reset(); command("barcode_mode 2",0); command("line_follow",0);
    event=3; robot_controller_step(10);
    assert(robot_controller_begin_obstacle_avoidance());
    motion=false; robot_controller_step(30);
    assert(!following && !g_barcode_action.active);
    reset(); command("barcode_mode 2",0); command("line_follow",0);
    event=1; robot_controller_step(10);
    encoder_fault=true; motion=false; robot_controller_step(30);
    assert(!following && starts==1 && !g_barcode_action.active);
    assert(!strcmp(robot_controller_mode_name(),"idle"));
    reset(); command("barcode_mode 2",0); command("line_follow",0);
    encoder_fault=true; event=2; robot_controller_step(10);
    assert(rights==0);
    puts("PASS: controller barcode handoff, GP21 cancellation, timeout, encoder-fault stop without resume, calibration gates");
    return 0;
}
