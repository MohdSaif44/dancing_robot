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

// global variables

enum {
	red_servo,
	main_joint,
	mid_joint,
	gripper_joint,
	gripper,
	left,
	right,
	left_arm,
	right_arm
};


float left_current_position[5] = {1400.0, 2500.0, 2000.0, 1600.0, 2400.0};
float left_target_position[5]  = {1400.0, 2500.0, 2000.0, 1600.0, 2400.0};

float right_current_position[5] = {1100.0, 2300.0, 2500.0, 1750.0, 2400.0};
float right_target_position[5]  = {1100.0, 2300.0, 2500.0, 1750.0, 2400.0};

float    left_step_increment[5];

uint16_t left_num_steps;
uint16_t left_min_difference;
uint16_t left_difference;

float 	 right_step_increment[5];
uint16_t right_num_steps;
uint16_t right_min_difference;
uint16_t right_difference;

int mode;

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

	const osThreadAttr_t Right_Arm_Task_attributes = {
				.name = "Right_Arm",
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
	Right_Arm_Task_Handle 	 	= osThreadNew(Right_Arm, 	 NULL, &Right_Arm_Task_attributes);
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

uint8_t yaw_count, circle_count, automatic, rotating_flag = 0;

void Calculation(void *argument){


	while(1){

		osSemaphoreAcquire(CalcSemaphore, osWaitForever);


		if(ps4.button & L3){
			while(ps4.button & L3);
			mode = left_arm;
		}

		if(ps4.button & R3){
			while(ps4.button & R3);
			mode = right_arm;
		}

		if(!automatic){

			xr = -2.0*ps4.joyL_x;
			yr =  2.0*ps4.joyL_y;

		}
		else{

			xr = (Vx*cos(YawAngle*3.14/180)-Vy*sin(YawAngle*3.14/180));
			yr = (Vx*sin(YawAngle*3.14/180)+Vy*cos(YawAngle*3.14/180));
		}

		update_param();


		if(ps4.button & CIRCLE){
			while(ps4.button & CIRCLE);
			automatic 	 =	1;
			circle_count =  1;

		}


		if(ps4.button & CROSS){

			stop_all();

		}

		else{

			target_angle = target_angle + 0.25 * ps4.joyR_2 - 0.25 * ps4.joyL_2;

		}

		if(fabs(v1) + fabs(v2) + fabs(v3) + fabs(v4) + abs(wr) > 0.05){

				RNSVelocity(v1, v2, v3, v4, &rns);

		}else{

			RNSStop(&rns);
		}

		if(!rotating_flag){

			PID(&yaw_pid);

		}else{

			PID(&rotate_pid);
		}

		MODN(&Modn);
	}
}

uint8_t servo_state = 0;

void Transmission(void *argument){

	while(1){
		//		if (ps4.button == CIRCLE){
		//			// Pose 5
		//			//			servo_go_position(right, 1116, 2500, 2022, 1802, 2400, 1000); //ok
		//			right_target_position[red_servo] 	 = 1116;
		//			right_target_position[main_joint] 	 = 2500;
		//			right_target_position[mid_joint] 	 = 2020;
		//			right_target_position[gripper_joint] = 1800;
		//			right_target_position[gripper] 		 = 2400;
		//		}
		switch(circle_count){
		case 1:
			rotating_flag = 1;
			target_angle = -360;
			osDelay(4000);
			*rotate_pid.error = 0;
			*rotate_pid.out_put = 0;
			rotating_flag = 0;
			circle_count++;
			break;

		case 2:
			target_pos_x = -1.5;
			target_pos_y = -1.5;
			osDelay(1);
			PID(&x_pid);
			PID(&y_pid);
			osDelay(1);
			if(fabs(*y_pid.error) < 0.05){
				if(fabs(*x_pid.error) < 0.05){
					*y_pid.error = 0;
					*x_pid.error = 0;
					*y_pid.out_put = 0;
					*x_pid.out_put = 0;
					circle_count++;
				}
			}
			break;

		case 3:
			rotating_flag = 1;
			target_angle = -360;
			osDelay(4000);
			*rotate_pid.error = 0;
			*rotate_pid.out_put = 0;
			target_angle = 0;
			osDelay(4000);
			*rotate_pid.error = 0;
			*rotate_pid.out_put = 0;
			rotating_flag = 0;
			circle_count++;
			break;

		case 4:
			osDelay(4000);
			servo_state = 1;
			rotating_flag = 1;
			target_angle = -360 * 1;
			osDelay(4000);
			servo_state = 0;
			target_angle = -360 * 2;
			osDelay(4000);
			rotating_flag = 0;
			circle_count++;
			break;

		case 5:
			osDelay(4000);
			rotating_flag = 1;
			target_angle = -360 * 3;
			osDelay(4000);
			rotating_flag = 0;
			circle_count++;

		case 6:
			target_pos_x = -3.0;
			target_pos_y = 0;
			osDelay(1);
			PID(&x_pid);
			PID(&y_pid);
			osDelay(1);
			if(fabs(*y_pid.error) < 0.05){
				if(fabs(*x_pid.error) < 0.05){
					*y_pid.error = 0;
					*x_pid.error = 0;
					*y_pid.out_put = 0;
					*x_pid.out_put = 0;
					circle_count++;
				}
			}
			break;

		case 7:
			target_pos_x = -3.4;
			target_pos_y = 0;
			osDelay(1);
			PID(&x_pid);
			PID(&y_pid);
			osDelay(1);
			if(fabs(*y_pid.error) < 0.05){
				if(fabs(*x_pid.error) < 0.05){
					*y_pid.error = 0;
					*x_pid.error = 0;
					*y_pid.out_put = 0;
					*x_pid.out_put = 0;
					circle_count++;
				}
			}
			break;

		case 8:
			target_pos_x = -2.6;
			target_pos_y = 0;
			osDelay(1);
			PID(&x_pid);
			PID(&y_pid);
			osDelay(1);
			if(fabs(*y_pid.error) < 0.05){
				if(fabs(*x_pid.error) < 0.05){
					*y_pid.error = 0;
					*x_pid.error = 0;
					*y_pid.out_put = 0;
					*x_pid.out_put = 0;
					circle_count++;
				}
			}
			break;

		case 9:
			target_pos_x = -3.4;
			target_pos_y = 0;
			osDelay(1);
			PID(&x_pid);
			PID(&y_pid);
			osDelay(1);
			if(fabs(*y_pid.error) < 0.05){
				if(fabs(*x_pid.error) < 0.05){
					*y_pid.error = 0;
					*x_pid.error = 0;
					*y_pid.out_put = 0;
					*x_pid.out_put = 0;
					circle_count++;
				}
			}
			break;

		case 10:
			target_pos_x = -2.6;
			target_pos_y = 0;
			osDelay(1);
			PID(&x_pid);
			PID(&y_pid);
			osDelay(1);
			if(fabs(*y_pid.error) < 0.05){
				if(fabs(*x_pid.error) < 0.05){
					*y_pid.error = 0;
					*x_pid.error = 0;
					*y_pid.out_put = 0;
					*x_pid.out_put = 0;
					circle_count++;
				}
			}
			break;

		case 11:
			target_pos_x = -3.4;
			target_pos_y = 0;
			osDelay(1);
			PID(&x_pid);
			PID(&y_pid);
			osDelay(1);
			if(fabs(*y_pid.error) < 0.05){
				if(fabs(*x_pid.error) < 0.05){
					*y_pid.error = 0;
					*x_pid.error = 0;
					*y_pid.out_put = 0;
					*x_pid.out_put = 0;
					circle_count++;
				}
			}
			break;

		case 12:
			target_pos_x = -2.6;
			target_pos_y = 0;
			osDelay(1);
			PID(&x_pid);
			PID(&y_pid);
			osDelay(1);
			if(fabs(*y_pid.error) < 0.05){
				if(fabs(*x_pid.error) < 0.05){
					*y_pid.error = 0;
					*x_pid.error = 0;
					*y_pid.out_put = 0;
					*x_pid.out_put = 0;
					osDelay(25000);
					circle_count++;
				}
			}
			break;

		case 13:
			target_pos_x = -1.5;
			target_pos_y = 0;
			osDelay(1);
			PID(&x_pid);
			PID(&y_pid);
			osDelay(1);
			if(fabs(*y_pid.error) < 0.05){
				if(fabs(*x_pid.error) < 0.05){
					*y_pid.error = 0;
					*x_pid.error = 0;
					*y_pid.out_put = 0;
					*x_pid.out_put = 0;
					circle_count++;
				}
			}
			break;

		case 14:
			osDelay(4000);
			rotating_flag = 1;
			target_angle = -360 * 4;
			osDelay(4000);
			rotating_flag = 0;
			circle_count++;


		default:
			circle_count = 0;
			break;
		}
	}

}



uint8_t  L1_count;

void Left_Arm(void *argument){

	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_3 , (int)left_current_position[red_servo]);
	WriteBDC(&BDC6, (int)left_current_position[main_joint]);
	WriteBDC(&BDC5, (int)left_current_position[mid_joint]);
	WriteBDC(&BDC4, (int)left_current_position[gripper_joint]);
	WriteBDC(&BDC3, (int)left_current_position[gripper]);


	while(1){

			if (mode == left_arm){

				if(ps4.button == L1){ // ok
					__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_3 ,left_current_position[red_servo]--);
					osDelay(2);
				}
				else if (ps4.button == R1){
					__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_3 ,left_current_position[red_servo]++);
					osDelay(2);
				}

				if(ps4.button == TRIANGLE){ // ok
					WriteBDC(&BDC6, left_current_position[main_joint]--);
					osDelay(2);
				}
				else if (ps4.button == CROSS){
					WriteBDC(&BDC6, left_current_position[main_joint]++);
					osDelay(2);
				}

				if(ps4.button == UP){ // ok
					WriteBDC(&BDC5, left_current_position[mid_joint]--);
					osDelay(2);
				}
				else if (ps4.button == DOWN){
					WriteBDC(&BDC5, left_current_position[mid_joint]++);
					osDelay(2);
				}

				if(ps4.button == LEFT){ // ok
					WriteBDC(&BDC4, left_current_position[gripper_joint]--);
					osDelay(2);
				}
				else if (ps4.button == RIGHT){
					WriteBDC(&BDC4, left_current_position[gripper_joint]++);
					osDelay(2);
				}

				if(ps4.button == SQUARE){ // ok
					WriteBDC(&BDC3, left_current_position[gripper]--);
					osDelay(2);
				}
				else if (ps4.button == CIRCLE){
					WriteBDC(&BDC3, left_current_position[gripper]++);
					osDelay(2);
				}

			}

			if (circle_count == 1 || circle_count == 2 || circle_count == 3){
				// Pose 2
				left_target_position[red_servo] = 1400;
				left_target_position[main_joint] = 2500;
				left_target_position[mid_joint] = 1088;
				left_target_position[gripper_joint] = 826;
				left_target_position[gripper] = 2400;

			}

			if (circle_count == 4){
				// Pose 1
				if(servo_state == 0){
				left_target_position[red_servo] 	= 1424;
				left_target_position[main_joint] 	= 2500;
				left_target_position[mid_joint] 	= 1522;
				left_target_position[gripper_joint] = 760;
				left_target_position[gripper]		 = 2400;
				}
				if(servo_state == 1){
				left_target_position[red_servo] = 667;
				left_target_position[main_joint] = 2500;
				left_target_position[mid_joint] = 572;
				left_target_position[gripper_joint] = 743;
				left_target_position[gripper] = 2400;
				}
			}


			if (circle_count == 7){
				// Pose 2
				left_target_position[red_servo] = 751;
				left_target_position[main_joint] = 2500;
				left_target_position[mid_joint] = 2000;
				left_target_position[gripper_joint] = 659;
				left_target_position[gripper] = 2400;
			}

			if (ps4.button == TRIANGLE){
				// Pose 4
	//			servo_go_position(left, 667, 2500, 572, 743, 2400, 1000); //ok
				left_target_position[red_servo] = 667;
				left_target_position[main_joint] = 2500;
				left_target_position[mid_joint] = 572;
				left_target_position[gripper_joint] = 743;
				left_target_position[gripper] = 2400;
			}

//			if (ps4.button == CIRCLE){
//				// Pose 5
//	//			servo_go_position(left, 1424, 2500, 1522, 759, 2400, 1000); //ok
//				left_target_position[red_servo] = 1424;
//				left_target_position[main_joint] = 2500;
//				left_target_position[mid_joint] = 1522;
//				left_target_position[gripper_joint] = 759;
//				left_target_position[gripper] = 2400;
//			}

			if (ps4.button == UP){
				// Pose 6
	//			servo_go_position(left, 1365, 2103, 393, 759, 2400, 1000); // ok
				left_target_position[red_servo] = 1365;
				left_target_position[main_joint] = 2100;
				left_target_position[mid_joint] = 400;
				left_target_position[gripper_joint] = 760;
				left_target_position[gripper] = 2400;
			}

			if (ps4.button == DOWN){
				// Pose 7
	//			servo_go_position(left, 780, 2500, 1630, 500, 2400, 1000); // ok
				left_target_position[red_servo] = 780;
				left_target_position[main_joint] = 2500;
				left_target_position[mid_joint] = 1630;
				left_target_position[gripper_joint] = 500;
				left_target_position[gripper] = 2400;
			}

			if (ps4.button == RIGHT){
				// Pose 9
	//			servo_go_position(left, 1133, 2373, 1800, 900, 2400, 1000); // ok
				left_target_position[red_servo] = 1133;
				left_target_position[main_joint] = 2373;
				left_target_position[mid_joint] = 1800;
				left_target_position[gripper_joint] = 900;
				left_target_position[gripper] = 2400;
			}

			if (ps4.button == LEFT){
				// Normal
	//			servo_go_position(left, 1400, 2500, 2000, 500, 2400, 1000); // ok
				left_target_position[red_servo] = 1400;
				left_target_position[main_joint] = 2400;
				left_target_position[mid_joint] = 2000;
				left_target_position[gripper_joint] = 500;
				left_target_position[gripper] = 2400;
			}

			for (int j = 0; j < 5; j++){left_target_position[mid_joint] = 1088;
			left_target_position[gripper_joint] = 826;
			left_target_position[gripper] = 2400;

				if (fabs(left_target_position[j] - left_current_position[j]) > 10){

					left_min_difference = 2500;

					for (int i = 0; i < 5; i++) {
						left_difference = fabs(left_target_position[i] - left_current_position[i]);
						if (left_difference < left_min_difference && left_difference >= 100) {
							left_min_difference = left_difference;
						}
					}

					left_num_steps = left_min_difference;

					for (int i = 0; i < 5; i++) {
						left_step_increment[i] = (left_target_position[i] - left_current_position[i]) / (left_num_steps);
					}

					for (uint16_t step = 0; step <= left_num_steps; step++) {
						for (int i = 0; i < 5; i++) {
							left_current_position[i] += left_step_increment[i];
						}
						__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_3 , (int)left_current_position[red_servo]);
						WriteBDC(&BDC6, (int)left_current_position[main_joint]);
						WriteBDC(&BDC5, (int)left_current_position[mid_joint]);
						WriteBDC(&BDC4, (int)left_current_position[gripper_joint]);
						WriteBDC(&BDC3, (int)left_current_position[gripper]);
						for (int delay = 0; delay < 2000; delay++){

						}
					}
				}
				break;
			}
		}

}

void Right_Arm(void *argument){

	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_4,  right_current_position[red_servo]);
	__HAL_TIM_SET_COMPARE(&htim12,TIM_CHANNEL_1 ,right_current_position[main_joint]);
	__HAL_TIM_SET_COMPARE(&htim12,TIM_CHANNEL_2 ,right_current_position[mid_joint]);
	WriteBDC(&BDC7, right_current_position[gripper_joint]);
	WriteBDC(&BDC8, right_current_position[gripper]);


	while(1){

		if (mode == right_arm){

			if(ps4.button == L1){ // ok
				__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_4,  right_current_position[red_servo]--);
				osDelay(2);
			}
			else if (ps4.button == R1){
				__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_4,  right_current_position[red_servo]++);
				osDelay(2);
			}

			if(ps4.button == TRIANGLE){ // ok
				__HAL_TIM_SET_COMPARE(&htim12,TIM_CHANNEL_1 ,right_current_position[main_joint]--);
				osDelay(2);
			}
			else if (ps4.button == CROSS){
				__HAL_TIM_SET_COMPARE(&htim12,TIM_CHANNEL_1 ,right_current_position[main_joint]++);
				osDelay(2);
			}

			if(ps4.button == UP){ // ok
				__HAL_TIM_SET_COMPARE(&htim12,TIM_CHANNEL_2 ,right_current_position[mid_joint]--);
				osDelay(2);
			}
			else if (ps4.button == DOWN){
				__HAL_TIM_SET_COMPARE(&htim12,TIM_CHANNEL_2 ,right_current_position[mid_joint]++);
				osDelay(2);
			}

			if(ps4.button == LEFT){
				WriteBDC(&BDC8, right_current_position[gripper_joint]--);
				osDelay(2);
			}
			else if (ps4.button == RIGHT){
				WriteBDC(&BDC8, right_current_position[gripper_joint]++);
				osDelay(2);
			}

			if(ps4.button == SQUARE){
				WriteBDC(&BDC7, right_current_position[gripper_joint]--);
				osDelay(2);
			}
			else if (ps4.button == CIRCLE){
				WriteBDC(&BDC7, right_current_position[gripper_joint]++);
				osDelay(2);
			}
		}

		if (circle_count == 1 || circle_count == 2 || circle_count == 3){
			// Pose 2
			//			servo_go_position(right, 1116, 2500, 1575, 1802, 2400, 1000); //ok
			right_target_position[red_servo] 	 = 1100;
			right_target_position[main_joint] 	 = 2420;
			right_target_position[mid_joint] 	 = 2320;
			right_target_position[gripper_joint] = 1911;
			right_target_position[gripper] 		 = 1800;
		}

		if (circle_count == 4){
			// Pose 1
			//			servo_go_position(right, 1104, 2420, 2317, 1911, 1800, 1000); // ok
			if(servo_state == 0){
			right_target_position[red_servo] 	 = 1116;
			right_target_position[main_joint] 	 = 2500;
			right_target_position[mid_joint] 	 = 2022;
			right_target_position[gripper_joint] = 1802;
			right_target_position[gripper] 		 = 2400;
			}
			if(servo_state == 1){
			right_target_position[red_servo] 	 = 1800;
			right_target_position[main_joint] 	 = 2500;
			right_target_position[mid_joint] 	 = 1030;
			right_target_position[gripper_joint] = 1730;
			right_target_position[gripper] 		 = 2400;
			}
		}


		if (circle_count == 7){
			// Pose 4
			//			servo_go_position(right, 1797, 2500, 1030, 1726, 2400, 1000); //ok
			right_target_position[red_servo] 	 = 1826;
			right_target_position[main_joint] 	 = 2500;
			right_target_position[mid_joint] 	 = 1118;
			right_target_position[gripper_joint] = 1701;
			right_target_position[gripper] 		 = 2400;
		}

//		if (ps4.button == CIRCLE){
//			// Pose 5
//			//			servo_go_position(right, 1116, 2500, 2022, 1802, 2400, 1000); //ok
//			right_target_position[red_servo] 	 = 1116;
//			right_target_position[main_joint] 	 = 2500;
//			right_target_position[mid_joint] 	 = 2020;
//			right_target_position[gripper_joint] = 1800;
//			right_target_position[gripper] 		 = 2400;
//		}

		if (ps4.button == UP){
			// Pose 6
			//			servo_go_position(right, 1106, 2058, 1089, 1802, 2400, 1000); // ok
			right_target_position[red_servo] 	 = 1100;
			right_target_position[main_joint] 	 = 2060;
			right_target_position[mid_joint] 	 = 1090;
			right_target_position[gripper_joint] = 1800;
			right_target_position[gripper] 		 = 2400;
		}

		if (ps4.button == DOWN){
			// Pose 7
			//			servo_go_position(right, 1570, 2500, 1360, 1800, 2400, 1000); // ok
			right_target_position[red_servo] 	 = 1570;
			right_target_position[main_joint] 	 = 2500;
			right_target_position[mid_joint] 	 = 1360;
			right_target_position[gripper_joint] = 1800;
			right_target_position[gripper] 		 = 2400;
		}

		if (ps4.button == RIGHT){
			// Pose 9
			//			servo_go_position(right, 1300, 2340, 2340, 1800, 2400, 1000); // ok
			right_target_position[red_servo] 	 = 1300;
			right_target_position[main_joint] 	 = 2340;
			right_target_position[mid_joint] 	 = 2340;
			right_target_position[gripper_joint] = 1800;
			right_target_position[gripper] 		 = 2400;
		}

		if (ps4.button == LEFT){
			// Normal
			//			servo_go_position(right, 1100.0, 2500.0, 2500.0, 1750.0, 2400.0, 1000); // ok
			right_target_position[red_servo] 	 = 1100;
			right_target_position[main_joint] 	 = 2400;
			right_target_position[mid_joint] 	 = 2500;
			right_target_position[gripper_joint] = 1750;
			right_target_position[gripper] 		 = 2400;
		}

		for (int j = 0; j < 5; j++){

			if (fabs(right_target_position[j] - right_current_position[j]) > 10){

				right_min_difference = 2500;

				for (int i = 0; i < 5; i++) {
					right_difference = fabs(right_target_position[i] - right_current_position[i]);
					if (right_difference < right_min_difference && right_difference >= 100) {
						right_min_difference = right_difference;
					}
				}

				right_num_steps = right_min_difference;

				for (int i = 0; i < 5; i++) {
					right_step_increment[i] = (right_target_position[i] - right_current_position[i]) / (right_num_steps);
				}

				for (uint16_t step = 0; step <= right_num_steps; step++) {
					for (int i = 0; i < 5; i++) {
						right_current_position[i] += right_step_increment[i];
					}
					__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_4,  right_current_position[red_servo]);
					__HAL_TIM_SET_COMPARE(&htim12,TIM_CHANNEL_1 ,right_current_position[main_joint]);
					__HAL_TIM_SET_COMPARE(&htim12,TIM_CHANNEL_2 ,right_current_position[mid_joint]);
					WriteBDC(&BDC7, right_current_position[gripper_joint]);
					WriteBDC(&BDC8, right_current_position[gripper]);
					for (int delay = 0; delay < 2000; delay++){

					}
				}
			}
			break;
		}
	}
}

void stop_all(void){

	HAL_NVIC_SystemReset();
	update_param();
	target_angle = YawAngle;
	automatic = 0;
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

void update_param(void){

	RNSEnquire(RNS_COORDINATE_X_Y_Z, &rns);
	xpos      = rns.RNS_data.common_buffer[0].data;
	ypos      = rns.RNS_data.common_buffer[1].data;
	YawAngle  = rns.RNS_data.common_buffer[2].data;
	error_x   = target_pos_x - xpos;
	error_y	  =	target_pos_y - ypos;
	error_angle  = target_angle - YawAngle;

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
