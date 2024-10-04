/* Includes ------------------------------------------------------------------*/

#include "main.h"
#include "cmsis_os.h"

bool cubemx_transport_open(struct uxrCustomTransport * transport);
bool cubemx_transport_close(struct uxrCustomTransport * transport);
size_t cubemx_transport_write(struct uxrCustomTransport* transport, const uint8_t * buf, size_t len, uint8_t * err);
size_t cubemx_transport_read(struct uxrCustomTransport* transport, uint8_t* buf, size_t len, int timeout, uint8_t* err);

/**
 * @brief  The application entry point.
 * @retval int
 */

uint16_t lh_main_joint_pwm = 1400;
uint16_t lh_mid_joint_pwm = 1800;
uint16_t lh_gripper_pwm = 500;
uint16_t lh_gripper_joint_pwm = 500;

int main(void)
{
	set();

	const osThreadAttr_t Navigation_Task_attributes = {
			.name = "Navigation",
			.stack_size = 256 *  4,
			.priority = (osPriority_t) osPriorityNormal,
	};


	const osThreadAttr_t Calculation_Task_attributes = {
			.name = "Calculation",
			.stack_size = 256 * 4,
			.priority = (osPriority_t) osPriorityNormal,
	};

	const osThreadAttr_t Transmission_Task_attributes = {
			.name = "Transmission",
			.stack_size = 256 * 4,
			.priority = (osPriority_t) osPriorityNormal,
	};

	const osThreadAttr_t Left_Arm_Task_attributes = {
			.name = "Left_Arm",
			.stack_size = 256 * 4,
			.priority = (osPriority_t) osPriorityNormal,
	};

	const osThreadAttr_t Left_Middle_Arm_Task_attributes = {
			.name = "Left_Middle_Arm",
			.stack_size = 256 * 4,
			.priority = (osPriority_t) osPriorityNormal,
	};

	const osSemaphoreAttr_t CalcSemaphore_attributes = {
			.name = "CalcSemaphore"
	};

	osKernelInitialize();

//	Navigation_Task_Handle   	= osThreadNew(Navigation, 	 NULL, &Navigation_Task_attributes);
	Calculation_Task_Handle  	= osThreadNew(Calculation,  NULL, &Calculation_Task_attributes);
	Transmission_Task_Handle 	= osThreadNew(Transmission, NULL, &Transmission_Task_attributes);
	Left_Arm_Task_Handle 	 	= osThreadNew(Left_Arm, 	 NULL, &Left_Arm_Task_attributes);
//	Left_Middle_Arm_Task_Handle = osThreadNew(Left_Middle_Arm, 	 NULL, &Left_Middle_Arm_Task_attributes);
	CalcSemaphore 			 = osSemaphoreNew(1, 0, &CalcSemaphore);

	osKernelStart();

	while(1){

	}

}

int timer, enable_flag, final_pos = 500;
float target_angle = 0.0;

void Navigation(void *argument){


	while(1){

		//		__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_4 ,1100); //Left


//				xr = (-ps4.joyL_x*cos(YawAngle*3.14/180)-ps4.joyL_y*sin(YawAngle*3.14/180));
//				yr = (-ps4.joyL_x*sin(YawAngle*3.14/180)+ps4.joyL_y*cos(YawAngle*3.14/180));


		//		while(ps4.button & CIRCLE){

		//			final_pos += 1;
		//			WriteBDC(&BDC6, final_pos);
		//			osDelay(4);


		//		}

		__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_3 ,1400); //Left red
		__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_4,1100); //Right red
		//			WriteBDC(&BDC3, 500); //Left Hand Gripper
		//			WriteBDC(&BDC4, 500); //Left Hand Gripper joint
		//			WriteBDC(&BDC5, 500); //Left Middle Joint
		WriteBDC(&BDC6, 2400); //Left Arm Joint
		__HAL_TIM_SET_COMPARE(&htim12,TIM_CHANNEL_1 ,2500); //Right Arm Joint
		//		__HAL_TIM_SET_COMPARE(&htim12,TIM_CHANNEL_2 ,500); //Right Middle joint
		//			WriteBDC(&BDC7, 500); //Right Arm gripper
		//			WriteBDC(&BDC8, 500); //Right gripper joint

		//		if(ps4.button & SQUARE){
		//
		//			for(final_pos = 1400; final_pos <= 1250; final_pos = final_pos - 1){
		//				__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_3, final_pos);
		//			}
		//		}

		osDelay(1);

	}

}

typedef struct{

	float m1;
	float m2;
	float m3;
	float m4;

} motor_check;

motor_check test;
uint8_t yaw_count, circle_count, automatic;

void Calculation(void *argument){


	while(1){

		osSemaphoreAcquire(CalcSemaphore, osWaitForever);

		if(!automatic){
			xr = -2.5*ps4.joyL_x;
			yr =  2.5*ps4.joyL_y;
		}
		else{
			xr = (Vx*cos(YawAngle*3.14/180)-Vy*sin(YawAngle*3.14/180));
			yr = (Vx*sin(YawAngle*3.14/180)+Vy*cos(YawAngle*3.14/180));
		}
		yaw_count++;

		RNSEnquire(RNS_COORDINATE_X_Y_Z, &rns);
		xpos      = rns.RNS_data.common_buffer[0].data;
		ypos      = rns.RNS_data.common_buffer[1].data;
		YawAngle  = rns.RNS_data.common_buffer[2].data;
		error_x   = target_pos_x - xpos;
		error_y	  =	target_pos_y - ypos;
		error_angle  = target_angle - YawAngle;


		if(ps4.button & CIRCLE){
			while(ps4.button & CIRCLE);
			circle_count=1;
			automatic =1;

		}


		if(ps4.button & CROSS){

			HAL_NVIC_SystemReset();
			automatic = 0;
			target_angle = 0;
			error_x = 0;
			error_y = 0;
			circle_count = 0;
			PIDDelayInit(&x_pid);
			PIDDelayInit(&y_pid);
			*y_pid.error = 0;
			*x_pid.error = 0;
			*y_pid.out_put = 0;
			*x_pid.out_put = 0;
			RNSStop(&rns);

		}

		else{

			target_angle = target_angle + 0.25 * ps4.joyR_2 - 0.25 * ps4.joyL_2;

		}

		if(fabs(v1) + fabs(v2) + fabs(v3) + fabs(v4) + abs(wr) > 0.05){

//			if(!automatic){
				RNSVelocity(v1, v2, v3, v4, &rns);
//			}

		}else{

			RNSStop(&rns);
		}

		PID(&yaw_pid);
		MODN(&Modn);

	}
}

float total_error;

void Transmission(void *argument){

	while(1){

		//total_error = fabs(*x_pid.error)+fabs(*y_pid.error);

		switch(circle_count){
		case 1:
			target_angle = 45;
			osDelay(2500);
			circle_count++;
			break;

		case 2:
			target_pos_x = 4.0;
			target_pos_y = 4.0;
			osDelay(1);
			PID(&x_pid);
			PID(&y_pid);
			osDelay(1);
			if(fabs(*y_pid.error) < 0.05){
				if(fabs(*x_pid.error) < 0.05){
//					circle_count++;
				}
			}
			break;
		case 3:
			PID(&x_pid);
			PID(&y_pid);
			target_angle = 0;
			osDelay(1000);
			circle_count++;
			break;

		case 4:
		target_pos_x = 0.0;
		target_pos_y = 0.0;
		osDelay(2);
		PID(&x_pid);
		PID(&y_pid);
		osDelay(2);
		if(fabs(*y_pid.error) < 0.05){
			if(fabs(*x_pid.error) < 0.05){
				circle_count++;
			}
		}
		break;

		case 5:
			PID(&x_pid);
			PID(&y_pid);
			break;

		default:
			circle_count = 0;
			break;
		}
	}

}

//
//void MicrorosTask(void *argument){
//	MX_USB_DEVICE_Init();
//
//rcl_publisher_t publisher;
////std_msgs_msg_int32 msg;
//rclc_support_t support;
//rcl_allocator_t allocator;
//rcl_node_t node;
//led3 = 1;
////
////
//rmw_uros_set_custom_transport(true, (void*) &hpcd_USB_OTG_FS,
//		cubemx_transport_open, cubemx_transport_close,
//		cubemx_transport_write, cubemx_transport_read);
//
//rcl_allocator_t freeRTOS_allocator =
//		rcutils_get_zero_initialized_allocator();
//freeRTOS_allocator.allocate = microros_allocate;
//freeRTOS_allocator.deallocate = microros_deallocate;
//freeRTOS_allocator.reallocate = microros_reallocate;
//freeRTOS_allocator.zero_allocate = microros_zero_allocate;
//rcutils_set_default_allocator(&freeRTOS_allocator);
//allocator = rcl_get_default_allocator();
//
///*****************INIT MSGS***********************/
////std_msgs_msgInt32_init(&msg);
//
///****************CONNECT***********************/
//if (RCL_RET_OK != rclc_support_init(&support, 0, NULL, &allocator)) {
//	led3 = 1;
//	osDelay(250);
//	NVIC_SystemReset();
//}
//
//rclc_node_init_default(&node, "node", "", &support);
//
//
///*****************PUBLISHERS***********************/
//// Initialize odom publisher
//rclc_publisher_init_best_effort(&publisher, &node,
//		ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32), "topico");
//
//
//
////msg.data = 0;
//
//for(;;)
//{
//
//	//		rcl_ret_t ret = rcl_publish(&publisher, &msg, NULL);
//	//		if (ret != RCL_RET_OK)
//	//		{
//	//			printf("Error publishing (line %d)\n", _LINE_);
//	//		}
//	//
//	//		msg.data++;
//
//	osDelay(10);
// }
//}
//
//void coor_callback(const void * msgin){
//
//
//}

uint8_t  L1_count;

void Left_Arm(void *argument){

	while(1){

		// Servo limits

		if(ps4.button == L1){
			__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_3 ,lh_main_joint_pwm--); //Left
			osDelay(2);
		}
		else if (ps4.button == R1){
			__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_3 ,lh_main_joint_pwm++); //Left
			osDelay(2);
		}

		if(ps4.button == SQUARE){
			WriteBDC(&BDC5, lh_mid_joint_pwm--);
			osDelay(2);
		}
		else if (ps4.button == TRIANGLE){
			WriteBDC(&BDC5, lh_mid_joint_pwm++);
			osDelay(2);
		}

		if(ps4.button == UP){
			WriteBDC(&BDC3, lh_gripper_pwm--);
			osDelay(2);
		}
		else if (ps4.button == DOWN){
			WriteBDC(&BDC3, lh_gripper_pwm++);
			osDelay(2);
		}

		if(ps4.button == LEFT){
			WriteBDC(&BDC4, lh_gripper_joint_pwm--);
			osDelay(2);
		}
		else if (ps4.button == RIGHT){
			WriteBDC(&BDC4, lh_gripper_joint_pwm++);
			osDelay(2);
		}


//		if(ps4.button == L1){
//			while(ps4.button == L1);
//			L1_count = 1;
//
//		}
//
//
//		if(L1_count == 1){
//			__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_3 ,lh_main_joint_pwm--); //Left
//			osDelay(1);
//			if(lh_main_joint_pwm <= 600){
//
//				L1_count = 2;
//			}
//		}
//		if(L1_count == 2){
//			__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_3 ,lh_main_joint_pwm++); //Left
//			osDelay(1);
//			if(lh_main_joint_pwm >= 1400){
//				L1_count = 3;
//			}
//			if(L1_count == 3){
//				vTaskDelay(pdMS_TO_TICKS(5000));
//				L1_count = 1;
//			}
//
//		}

	}


}


void Left_Middle_Arm(void *argument){

	while(1){

		if(ps4.button == L1){
			while(ps4.button == L1);
			L1_count = 1;

		}
		if(L1_count == 1){
			WriteBDC(&BDC5, lh_mid_joint_pwm--);
			osDelay(1);
		}
		if(L1_count == 2){
			WriteBDC(&BDC5, lh_mid_joint_pwm = 1800);
			osDelay(1);

		}

		if(L1_count == 3){
			WriteBDC(&BDC5, lh_mid_joint_pwm--);
			osDelay(1);
			if(lh_mid_joint_pwm <= 500){
				WriteBDC(&BDC5, lh_mid_joint_pwm = 1800);
				osDelay(100);
			}


		}


	}
}



uint8_t ps4_count;

void TIM7_IRQHandler(void) { //5ms

	ps4_count++;

	osSemaphoreRelease(CalcSemaphore);

	if(ps4_count >= 4){
		PSxConnectionHandler(&ps4);
		ps4_count = 0;
	}
	HAL_TIM_IRQHandler(&htim7);
}

void TIM6_DAC_IRQHandler(void) //20ms
{
	led1 = !led1;
	HAL_TIM_IRQHandler(&htim6);
}

//uint16_t move_servo_slow(uint16_t target){
//
//
//
//}
/**
 * @brief  This function is executed in case of error occurrence.
 */
void Error_Handler(void)
{


}
#ifdef  USE_FULL_ASSERT
/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t *file, uint32_t line)
{
	/* USER CODE BEGIN 6 */
	/* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
	/* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */


/************************ (C) COPYRIGHT STMicroelectronics END OF FILE*/
