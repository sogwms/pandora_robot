/*********************************************************
Change From YFRobot. www.yfrobot.com
**********************************************************/

#include <rtthread.h>
#include <drv_gpio.h>
#include "ps2_controller.h"
#include "ps2_controller_port.h"

extern hal_ps2_port_t hal_ps2_port;

void ps2_init(void)
{
    hal_ps2_port.init(0);
}

int ps2_scan(ps2_ctrl_data_t *pt)
{
    uint8_t temp_send[9] = {0x01, 0x42};
    uint8_t temp_recv[9] = {0x00};

    hal_ps2_port.send_then_recv(temp_send, 2, temp_recv, 7);
   
    pt->button = temp_recv[1] | (temp_recv[2] << 8);
    pt->right_stick_x = temp_recv[3];
    pt->right_stick_y = temp_recv[4];
    pt->left_stick_x = temp_recv[5];
    pt->left_stick_y = temp_recv[6];

    // hal_ps2_port.transfer(temp_send, temp_recv, 9);
    
    for (int i=0; i<7; i++)
    {
        rt_kprintf("0x%x ", temp_recv[i]);
    }

    if (temp_recv[0] == 0x5A)
    {
        return 1;
    }

    return 0;
}

/**
 * @brief 控制手柄振动
 * @param s_motor 右侧小振动电机振动幅度, 0x00 关, 其它开？
 * @param l_motor 左侧大振动电机振动幅度, 0x40~0xFF 电机开, 值越大振动越大
 * @note 需先调用 ps2_enter_config
 */
void ps2_vibrate(uint8_t s_motor, uint8_t l_motor)
{
    uint8_t temp[9];

    ps2_enter_config();
    ps2_open_vibration_mode();
    ps2_exit_config();

    temp[0] = 0x01;
    temp[1] = 0x42;
    temp[3] = s_motor;
    temp[4] = l_motor;
    hal_ps2_port.send(temp, sizeof(temp));
}

static void ps2_enter_config(void)
{
    uint8_t temp[9] = {0};

    temp[0] = 0x01;
    temp[1] = 0x43;
    temp[2] = 0x00;
    temp[3] = 0x01;

    temp[4] = 0x00;
    temp[5] = 0x00;
    temp[6] = 0x00;
    temp[7] = 0x00;
    temp[8] = 0x00;
    hal_ps2_port.send(temp, sizeof(temp));
}

static void ps2_open_vibration_mode(void)
{
    uint8_t temp[5] = {0};

    temp[0] = 0x01;
    temp[1] = 0x4D;
    temp[2] = 0x00;
    temp[3] = 0x00;
    temp[4] = 0x01;
    hal_ps2_port.send(temp, sizeof(temp));
}

static void ps2_exit_config(void)
{
    uint8_t temp[9] = {0};

    temp[0] = 0x01;
    temp[1] = 0x43;
    temp[2] = 0x00;
    temp[3] = 0x00;
    temp[4] = 0x5A;
    temp[5] = 0x5A;
    temp[6] = 0x5A;
    temp[7] = 0x5A;
    temp[8] = 0x5A;
    hal_ps2_port.send(temp, sizeof(temp));
}

/**
 * @return PS2_GREEN_MDOE or PS2_RED_MDOE or other(no connect) 
 */
int ps2_read_light(void)
{
    uint8_t temp_send = 0x01, temp_recv;

    hal_ps2_port.send_then_recv(&temp_send, 1, &temp_recv, 1);
    return temp_recv;    
}

#include <stdlib.h>

static void ps2(int argc, char **argv)
{
	if (argc < 2)
	{
		return;
	}
	if (!rt_strcmp(argv[1], "init"))
	{
		ps2_init();
	}
    if (!rt_strcmp(argv[1], "light"))
	{
        ps2_read_light();        
	}
	if (!rt_strcmp(argv[1], "scan"))
	{
        ps2_ctrl_data_t temp;
		ps2_scan(&temp);
        rt_kprintf("button:0x%x LX:%d LY:%d RX:%d RY:%d\r\n", temp.button, temp.left_stick_x, temp.left_stick_y, temp.right_stick_x, temp.right_stick_y);
	}

	if (argc < 3)
	{
		return;
	}

	if (argc < 4)
	{
		return;
	}
	if (!rt_strcmp(argv[1], "vibrating"))
	{
		ps2_vibrate(atoi(argv[2]), atoi(argv[3]));
	}
}
MSH_CMD_EXPORT(ps2, ps2);
