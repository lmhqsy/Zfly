/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2022-03-25     CGY       the first version
 */

#include <rtdbg.h>
#include <rtdevice.h>
#include <board.h>
#include <DL_SerialPort.h>
#include <loco_config.h>

////串口编号：                                    uart1  uart2  uart3  uart4  uart5  uart6  uart7  uart8
//uint16_t uartProtocol[8][2]={{1,1}, {2,2}, {3,3}, {4,4}, {5,5}, {6,6}, {7,7}, {8,8}};//默认的串口协议

//串口编号：                                                 uart1  uart2  uart3  uart4  uart5   uart6  uart7  uart8
uint16_t uartProtocol[8][2]={{1,-1}, {2,10}, {3,0}, {4,2}, {5,5}, {6,3}, {7,8}, {8,4}};//默认的串口协议
/***********************************************************
  *@brief  uart[8][2]数组第一列一次为串口1到串口8。第二列数字是对应串口号的功能，功能如下（1：发送数据  2：激光  3:光流 4：gps 5：视觉等）
  *@brief  特别注意：第二列的协议所对应的数字不可重复，重复时只对前一个串口起作用。
  *@brief  protocol: 各种协议所对应的数字
*************************************************************/
rt_device_t serial_device_find(enum SerialProtocol protocol)
{
    static rt_device_t serial;                /* 串口设备句柄 */
    const char * uart_name;

    for (int i = 0; i < SERIALMANAGER_NUM_PORTS; ++i) {

        if(uartProtocol[i][1] == protocol)
        {
            switch (uartProtocol[i][0]) {
            case 1:
                uart_name = UART1_NAME;
                break;
            case 2:
                uart_name = UART2_NAME;
                break;
            case 3:
                uart_name = UART3_NAME;
                break;
            case 4:
                uart_name = UART4_NAME;
                break;
            case 5:
                uart_name = UART5_NAME;
                break;
            case 6:
                uart_name = UART6_NAME;
                break;
            case 7:
                uart_name = UART7_NAME;
                break;
            case 8:
                uart_name = UART8_NAME;
                break;
            default:
                break;
           }
            serial = rt_device_find(uart_name);
            return serial;
        }
    }
    return RT_NULL;
}

//设置串口的波特率
void set_serial_device_baud_rate(rt_device_t serial,rt_uint32_t buad_rate)
{
    struct serial_configure config = RT_SERIAL_CONFIG_DEFAULT;  /* 初始化配置参数 */

    config.baud_rate = buad_rate;             //修改波特率为 buad_rate
    config.data_bits = DATA_BITS_8;           //数据位 8
    config.stop_bits = STOP_BITS_1;           //停止位 1
    config.bufsz     = 512;                   //修改缓冲区 buff size 为 128
    config.parity    = PARITY_NONE;           //无奇偶校验位

    /*控制串口设备。通过控制接口传入命令控制字，与控制参数 */
    rt_device_control(serial, RT_DEVICE_CTRL_CONFIG, &config);
}

int uart2_device_init(void)
{
    static rt_device_t serial;                /* 串口设备句柄 */
    struct serial_configure config = RT_SERIAL_CONFIG_DEFAULT;  /* 初始化配置参数 */

    /* step1：查找串口设备 */
    serial = rt_device_find(UART2_NAME);

    config.baud_rate = BAUD_RATE_115200;      //修改波特率为 115200
    config.data_bits = DATA_BITS_8;           //数据位 8
    config.stop_bits = STOP_BITS_1;           //停止位 1
    config.bufsz     = 512;                   //修改缓冲区 buff size 为 128
    config.parity    = PARITY_NONE;           //无奇偶校验位
    if(serial)
    /* step3：控制串口设备。通过控制接口传入命令控制字，与控制参数 */
    rt_device_control(serial, RT_DEVICE_CTRL_CONFIG, &config);

    if(serial)
        /* 以中断接收及轮询发送模式打开串口设备 */
        rt_device_open(serial, RT_DEVICE_FLAG_RDWR | RT_DEVICE_FLAG_INT_RX);
    return RT_EOK;
}
/* 导出到自动初始化 */
INIT_COMPONENT_EXPORT(uart2_device_init);

int uart3_device_init(void)
{
    static rt_device_t serial;                /* 串口设备句柄 */
    struct serial_configure config = RT_SERIAL_CONFIG_DEFAULT;  /* 初始化配置参数 */

    /* step1：查找串口设备 */
    serial = rt_device_find(UART3_NAME);

    config.baud_rate = BAUD_RATE_115200;      //修改波特率为 115200
    config.data_bits = DATA_BITS_8;           //数据位 8
    config.stop_bits = STOP_BITS_1;           //停止位 1
    config.bufsz     = 512;                   //修改缓冲区 buff size 为 128
    config.parity    = PARITY_NONE;           //无奇偶校验位
    if(serial)
    /* step3：控制串口设备。通过控制接口传入命令控制字，与控制参数 */
    rt_device_control(serial, RT_DEVICE_CTRL_CONFIG, &config);

    if(serial)
        /* 以中断接收及轮询发送模式打开串口设备 */
        rt_device_open(serial, RT_DEVICE_FLAG_RDWR | RT_DEVICE_FLAG_INT_RX);
    return RT_EOK;
}
/* 导出到自动初始化 */
INIT_COMPONENT_EXPORT(uart3_device_init);

int uart4_device_init(void)
{
    static rt_device_t serial;                /* 串口设备句柄 */
    struct serial_configure config = RT_SERIAL_CONFIG_DEFAULT;  /* 初始化配置参数 */

    /* step1：查找串口设备 */
    serial = rt_device_find(UART4_NAME);

    config.baud_rate = BAUD_RATE_115200;      //修改波特率为 115200
    config.data_bits = DATA_BITS_8;           //数据位 8
    config.stop_bits = STOP_BITS_1;           //停止位 1
    config.bufsz     = 512;                   //修改缓冲区 buff size 为 128
    config.parity    = PARITY_NONE;           //无奇偶校验位
    if(serial)
    /* step3：控制串口设备。通过控制接口传入命令控制字，与控制参数 */
    rt_device_control(serial, RT_DEVICE_CTRL_CONFIG, &config);

    if(serial)
        /* 以中断接收及轮询发送模式打开串口设备 */
        rt_device_open(serial, RT_DEVICE_FLAG_RDWR | RT_DEVICE_FLAG_INT_RX);
    return RT_EOK;
}
/* 导出到自动初始化 */
INIT_COMPONENT_EXPORT(uart4_device_init);

int uart5_device_init(void)
{
    static rt_device_t serial;                /* 串口设备句柄 */
    struct serial_configure config = RT_SERIAL_CONFIG_DEFAULT;  /* 初始化配置参数 */

    /* step1：查找串口设备 */
    serial = rt_device_find(UART5_NAME);

    config.baud_rate = BAUD_RATE_115200;      //修改波特率为 115200
    config.data_bits = DATA_BITS_8;           //数据位 8
    config.stop_bits = STOP_BITS_1;           //停止位 1
    config.bufsz     = 512;                   //修改缓冲区 buff size 为 128
    config.parity    = PARITY_NONE;           //无奇偶校验位
    if(serial)
    /* step3：控制串口设备。通过控制接口传入命令控制字，与控制参数 */
    rt_device_control(serial, RT_DEVICE_CTRL_CONFIG, &config);

//    if(serial)
//        /* 以中断接收及轮询发送模式打开串口设备 */
//        rt_device_open(serial, RT_DEVICE_FLAG_RDWR | RT_DEVICE_FLAG_INT_RX);
    return RT_EOK;
}
/* 导出到自动初始化 */
INIT_COMPONENT_EXPORT(uart5_device_init);

rt_mailbox_t uart6_mb;
#if(FBW_BORAD_TYPES != 2)
int uart6_device_init(void)
{
    static rt_device_t serial;                /* 串口设备句柄 */
    struct serial_configure config = RT_SERIAL_CONFIG_DEFAULT;  /* 初始化配置参数 */

    /* step1：查找串口设备 */
    serial = rt_device_find(UART6_NAME);

    config.baud_rate = BAUD_RATE_115200;      //修改波特率为 115200
    config.data_bits = DATA_BITS_8;           //数据位 8
    config.stop_bits = STOP_BITS_1;           //停止位 1
    config.bufsz     = 512;                   //修改缓冲区 buff size 为 128
    config.parity    = PARITY_NONE;           //无奇偶校验位
    if(serial)
    /* step3：控制串口设备。通过控制接口传入命令控制字，与控制参数 */
    rt_device_control(serial, RT_DEVICE_CTRL_CONFIG, &config);

    if(serial)
        /* 以中断接收及轮询发送模式打开串口设备 */
        rt_device_open(serial, RT_DEVICE_FLAG_RDWR | RT_DEVICE_FLAG_INT_RX);
    return RT_EOK;
}

#else
/* 接收数据回调函数 */
static rt_err_t uart6_input(rt_device_t dev, rt_size_t size)
{
    /* 发送邮件 */
    return rt_mb_send(uart6_mb, size);
}
int uart6_device_init(void)
{
    static rt_device_t serial;                /* 串口设备句柄 */
    struct serial_configure config = RT_SERIAL_CONFIG_DEFAULT;  /* 初始化配置参数 */

    /* step1：查找串口设备 */
    serial = rt_device_find(UART6_NAME);

    config.baud_rate = BAUD_RATE_115200;      //修改波特率为 115200
    config.data_bits = DATA_BITS_8;           //数据位 8
    config.stop_bits = STOP_BITS_1;           //停止位 1
    config.bufsz     = 512;                   //修改缓冲区 buff size 为 128
    config.parity    = PARITY_NONE;           //无奇偶校验位
    if(serial)
    /* step3：控制串口设备。通过控制接口传入命令控制字，与控制参数 */
    rt_device_control(serial, RT_DEVICE_CTRL_CONFIG, &config);

    if (uart6_mb == RT_NULL)
    {
        uart6_mb = rt_mb_create("uart6_mb",256, RT_IPC_FLAG_FIFO);  //创建邮箱
        if (uart6_mb == RT_NULL)
        {
            return RT_ERROR;
        }
    }
    rt_device_set_rx_indicate(serial,uart6_input);
#ifdef BSP_UART6_RX_USING_DMA
    if(serial)
        rt_device_open(serial,RT_DEVICE_FLAG_DMA_RX);    //以DMA读取方式打开串口设备
#else
    if(serial)
        /* 以中断接收及轮询发送模式打开串口设备 */
        rt_device_open(serial, RT_DEVICE_FLAG_RDWR|RT_DEVICE_FLAG_INT_RX);    //以中断读取方式打开串口设备
#endif
    return RT_EOK;
}

#endif

/* 导出到自动初始化 */
INIT_COMPONENT_EXPORT(uart6_device_init);

int uart7_device_init(void)
{
    static rt_device_t serial;                /* 串口设备句柄 */
    struct serial_configure config = RT_SERIAL_CONFIG_DEFAULT;  /* 初始化配置参数 */

    /* step1：查找串口设备 */
    serial = rt_device_find(UART7_NAME);

    config.baud_rate = BAUD_RATE_115200;      //修改波特率为 115200
    config.data_bits = DATA_BITS_8;           //数据位 8
    config.stop_bits = STOP_BITS_1;           //停止位 1
    config.bufsz     = 512;                   //修改缓冲区 buff size 为 128
    config.parity    = PARITY_NONE;           //无奇偶校验位
    if(serial)
    /* step3：控制串口设备。通过控制接口传入命令控制字，与控制参数 */
    rt_device_control(serial, RT_DEVICE_CTRL_CONFIG, &config);

    if(serial)
        /* 以中断接收及轮询发送模式打开串口设备 */
        rt_device_open(serial, RT_DEVICE_FLAG_RDWR | RT_DEVICE_FLAG_INT_RX);
    return RT_EOK;
}
/* 导出到自动初始化 */
INIT_COMPONENT_EXPORT(uart7_device_init);


int uart8_device_init(void)
{
    static rt_device_t serial;                /* 串口设备句柄 */
    struct serial_configure config = RT_SERIAL_CONFIG_DEFAULT;  /* 初始化配置参数 */

    /* step1：查找串口设备 */
    serial = rt_device_find(UART8_NAME);

    config.baud_rate = BAUD_RATE_115200;      //修改波特率为 115200
    config.data_bits = DATA_BITS_8;           //数据位 8
    config.stop_bits = STOP_BITS_1;           //停止位 1
    config.bufsz     = 512;                   //修改缓冲区 buff size 为 128
    config.parity    = PARITY_NONE;           //无奇偶校验位
    if(serial)
    /* step3：控制串口设备。通过控制接口传入命令控制字，与控制参数 */
    rt_device_control(serial, RT_DEVICE_CTRL_CONFIG, &config);

    if(serial)
        /* 以中断接收及轮询发送模式打开串口设备 */
        rt_device_open(serial, RT_DEVICE_FLAG_RDWR | RT_DEVICE_FLAG_INT_RX);
    return RT_EOK;
}
/* 导出到自动初始化 */
INIT_COMPONENT_EXPORT(uart8_device_init);


//external_sensor_id:           1      2      3      4       5      6       7     8
bool use_external_sensor[8] = {true,  true,  true,  false,  true,  false,  true,  true};   //默认使用的外部传感器
bool is_use_external_sensor(enum external_sensor_id id)  //用来判断是否采用外部的某个传感器
{
    bool is_use = false;
    switch (id) {
        case Tof_id:
            is_use = use_external_sensor[0];
           break;
        case OpticalFlow_id:
            is_use = use_external_sensor[1];
           break;
        case T265_id:
            is_use = use_external_sensor[2];
           break;
        case Openmv_id:
            is_use = use_external_sensor[3];
           break;
        case US100_id:
            is_use = use_external_sensor[4];
           break;
        case GPS_id:
            is_use = use_external_sensor[5];
           break;
        case LaserRadar_id:
            is_use = use_external_sensor[6];
           break;

        default:
           break;
        }
    return is_use;
}

