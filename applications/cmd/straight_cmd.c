#include <rtthread.h>
#include <rtdevice.h>
#include <board.h>
#include <chassis.h>
#include <stdio.h>
#include <stdlib.h>
#include <pos_pid_controller.h>
#include <ano.h>
#include <ab_phase_encoder.h>

#define SAMPLE_TIME          10
#define PULSE_PER_REVOL      2496
#define ONE_CM_PULSE         120


#define LEFT_MOTOR_OBJ       chas->c_wheels[0]->w_motor
#define RIGHT_MOTOR_OBJ      chas->c_wheels[1]->w_motor

#define LEFT_ENCODER_OBJ     chas->c_wheels[0]->w_encoder
#define RIGHT_ENCODER_OBJ    chas->c_wheels[1]->w_encoder

// Thread
#define THREAD_DELAY_TIME               10
#define THREAD_PRIORITY                 12
#define THREAD_STACK_SIZE               1024
#define THREAD_TIMESLICE                5

static rt_thread_t tid_straight = RT_NULL;
static pos_pid_controller_t pos_controller;
static float target_yaw;
static int active_control = RT_FALSE;
static int run_speed = 0;
static int32_t set_distance=0;     // unit:cm
static int32_t last_enc_count=0;

extern float stof(const char *s);

extern float inv_yaw_state;
extern chassis_t chas;

extern void ano_init_all(void);

static void clear_encoder_count(void)
{
    encoder_reset(LEFT_ENCODER_OBJ);
    encoder_reset(RIGHT_ENCODER_OBJ);
    if (encoder_read(LEFT_ENCODER_OBJ) != 0 || encoder_read(RIGHT_ENCODER_OBJ) != 0)
    {
        rt_kprintf("encoder reset error: %d %d\n", LEFT_ENCODER_OBJ->pulse_count, RIGHT_ENCODER_OBJ->pulse_count);
    }
}

static int32_t get_current_distance(void)
{
    return (int32_t)((LEFT_ENCODER_OBJ->pulse_count / 2) + (RIGHT_ENCODER_OBJ->pulse_count / 2));
}

static void reset_controller(void)
{
    pos_controller->integral = 0.0f;
}

void straight_thread(void *param)
{
    while(1)
    {
        rt_thread_mdelay(THREAD_DELAY_TIME);

        // distance check
        if ((get_current_distance() >= (last_enc_count + set_distance*ONE_CM_PULSE)) && (set_distance > 0))
        {
            last_enc_count = get_current_distance();
            active_control = RT_FALSE;
            motor_run(LEFT_MOTOR_OBJ, 0);
            motor_run(RIGHT_MOTOR_OBJ, 0);
            set_distance = 0;

            run_speed = 0;
            rt_kprintf("distance end. cur: %dcm\n", (last_enc_count/ONE_CM_PULSE));
        }

        if (active_control)
        {
            pos_pid_controller_update(pos_controller, inv_yaw_state);

            motor_run(LEFT_MOTOR_OBJ, run_speed - pos_controller->last_out);
            motor_run(RIGHT_MOTOR_OBJ, run_speed + pos_controller->last_out);
        }
        else
        {
            motor_run(LEFT_MOTOR_OBJ, 0);
            motor_run(RIGHT_MOTOR_OBJ, 0);
            reset_controller();
        }
        ano_send_user_data(1, inv_yaw_state, target_yaw, pos_controller->p_error, pos_controller->i_error, pos_controller->d_error, (int16_t)pos_controller->last_out, run_speed, 0, pos_controller->integral);
    }
}

void straight_init(void)
{
    // thread
    tid_straight = rt_thread_create("tStraight",
                              straight_thread, RT_NULL,
                              THREAD_STACK_SIZE,
                              THREAD_PRIORITY, THREAD_TIMESLICE);

    if (tid_straight != RT_NULL)
    {
        rt_thread_startup(tid_straight);
    }
}

static void st(int argc, char *argv[])
{
    if (argc < 2)
    {
        return;
    }

    if (!rt_strcmp("reset-target", argv[1]))
    {
        target_yaw = inv_yaw_state;
        // set controller target
        pos_controller->controller.target = target_yaw;
        reset_controller();
        rt_kprintf("target:%d current:%d\n", (int)target_yaw, (int)inv_yaw_state);
    }
    else if (!rt_strcmp("init", argv[1]))
    {
        pos_controller = pos_pid_controller_create(60.0f,1.0f,70.0f);
        target_yaw = inv_yaw_state;
        // set controller target
        pos_controller->controller.target = target_yaw;

        // set controller sample time
        pos_controller->controller.sample_time = SAMPLE_TIME;

        // set i output limit
        pos_controller->anti_windup_value = pos_controller->maximum * 0.618;

        // set output limit
        pos_controller->maximum = 800;
        pos_controller->minimum = -800;

        // start thread
        straight_init();
    }
    else if (!rt_strcmp("start", argv[1]))
    {
        active_control = RT_TRUE;
    }
    else if (!rt_strcmp("stop", argv[1]))
    {
        active_control = RT_FALSE;
    }
    else if (!rt_strcmp("enable-motor", argv[1]))
    {
        motor_enable(LEFT_MOTOR_OBJ);
        motor_enable(RIGHT_MOTOR_OBJ);
    }
    else if (!rt_strcmp("read", argv[1]))
    {
        rt_kprintf("target:%d current:%d\n", (int)pos_controller->controller.target, (int)inv_yaw_state);
        rt_kprintf("kpid: %d %d %d\n", (int)(pos_controller->kp), (int)(pos_controller->ki), (int)(pos_controller->kd));
        rt_kprintf("pluse-cnt:%d distance:%dcm\n", get_current_distance(), get_current_distance()/ONE_CM_PULSE);
    }
    else if (!rt_strcmp("init-all", argv[1]))
    {
        pos_controller = pos_pid_controller_create(60.0f,1.0f,70.0f);
        target_yaw = inv_yaw_state;
        // set controller target
        pos_controller->controller.target = target_yaw;

        // set controller sample time
        pos_controller->controller.sample_time = SAMPLE_TIME;

        // start thread
        straight_init();

        ano_init_all();
    }

    if (argc < 3)
    {
        return;
    }
    if (!rt_strcmp("set-target", argv[1]))
    {
        target_yaw = stof(argv[2]);
        // set controller target
        pos_controller->controller.target = target_yaw;
        reset_controller();
        rt_kprintf("target:%d current:%d\n", (int)target_yaw, (int)inv_yaw_state);
    }
    else if (!rt_strcmp("set-speed", argv[1]))
    {
        run_speed = atoi(argv[2]);
    }
    else if (!rt_strcmp("angle", argv[1]))
    {
        target_yaw = stof(argv[2]);
        pos_controller->controller.target = target_yaw;
        active_control = RT_TRUE;
    }

    if (argc < 4)
    {
        return;
    }

    if (!rt_strcmp("run", argv[1]))
    {
        long duration = atoi(argv[2]);
        run_speed = atoi(argv[3]);

        rt_kprintf("start\n");
        active_control = RT_TRUE;
        rt_thread_mdelay(duration);
        active_control = RT_FALSE;
        rt_kprintf("end\n");
        rt_kprintf("target:%d current:%d\n", (int)target_yaw, (int)inv_yaw_state);
        run_speed = 0;
    }
    else if (!rt_strcmp("straight", argv[1]))
    {
        set_distance = atoi(argv[2]);
        run_speed = atoi(argv[3]);

        rt_kprintf("start\n");
        reset_controller();
        active_control = RT_TRUE;
        
        // wait end
        // active_control = RT_FALSE;
    }
    else if (!rt_strcmp("keep-speed", argv[1]))
    {
        long duration = atoi(argv[2]);
        
        rt_kprintf("start\n");
        run_speed = atoi(argv[3]);
        rt_thread_mdelay(duration);
        run_speed = 0;
        rt_kprintf("end\n");
    }

    if (argc < 5)
    {
        return;
    }

    if (!rt_strcmp("set-kpid", argv[1]))
    {
        pos_controller->kp = stof(argv[2]);
        pos_controller->ki = stof(argv[3]);
        pos_controller->kd = stof(argv[4]);
    }

    
}
MSH_CMD_EXPORT(st, straight_test);


