
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "../common.h"

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <uxr/client/transport.h>
#include <rmw_microxrcedds_c/config.h>
#include <rmw_microros/rmw_microros.h>
#include <std_msgs/msg/int32.h>

#include "turtle_bot_msgs/msg/coordinates.h"

void Navigation(void *argument);
void Calculation(void *argument);
void Transmission(void *argument);
void Left_Arm(void *argument);
void Left_Middle_Arm(void *argument);
void MicrorosTask(void *argument);
void Right_Arm(void *argument);
void Left_Arm(void *argument);

void * microros_allocate(size_t size, void * state);
void   microros_deallocate(void * pointer, void * state);
void * microros_reallocate(void * pointer, size_t size, void * state);
void * microros_zero_allocate(size_t number_of_elements, size_t size_of_element, void * state);

void coor_callback(const void * msgin);
void stop_all(void);
void update_param(void);

osThreadId_t Navigation_Task_Handle;
osThreadId_t Calculation_Task_Handle;
osThreadId_t Transmission_Task_Handle;
osThreadId_t Left_Arm_Task_Handle;
osThreadId_t Right_Arm_Task_Handle;
osThreadId_t Left_Middle_Arm_Task_Handle;
osThreadId_t Micrors_Task_Handle;
osSemaphoreId_t CalcSemaphore;

rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rclc_executor_t executor;

rcl_subscription_t coor_subscriber;

turtle_bot_msgs__msg__Coordinates coor_msg;


#ifdef __cplusplus
}
#endif



#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
