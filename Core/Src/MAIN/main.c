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

uint8_t dance_flag;

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

uint8_t yaw_count, circle_count, automatic, rotating_flag = 0, circle_flag = 1;

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

			xr = 2*(-ps4.joyL_x*cos(YawAngle*3.14/180)-ps4.joyL_y*sin(YawAngle*3.14/180));
			yr = 2*(-ps4.joyL_x*sin(YawAngle*3.14/180)+ps4.joyL_y*cos(YawAngle*3.14/180));

		}
		else{

			xr = (Vx*cos(YawAngle*3.14/180)-Vy*sin(YawAngle*3.14/180));
			yr = (Vx*sin(YawAngle*3.14/180)+Vy*cos(YawAngle*3.14/180));
		}

		update_param();


		if((ps4.button & CIRCLE) && circle_flag){


				automatic 	 =	1;
				circle_count =  1;
//				circle_count++;
				circle_flag = 0;


		} else {

			circle_flag = 1;
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

uint8_t servo_state  = 0;
uint8_t servo_state2 = 0;

void Transmission(void *argument){

	while(1){

		switch(circle_count){
		case 1:
			rotating_flag = 1;
			target_angle = -360;
			osDelay(2000);
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
			if(fabs(*y_pid.error) < 0.07){
				if(fabs(*x_pid.error) < 0.07){
					*y_pid.error = 0;
					*x_pid.error = 0;
					*y_pid.out_put = 0;
					*x_pid.out_put = 0;
					circle_count++;
					osDelay(1000);
				}
			}
			break;

		case 3:
			rotating_flag = 1;
			target_angle = -360;
			osDelay(2000);
			*rotate_pid.error = 0;
			*rotate_pid.out_put = 0;
			target_angle = 0;
			osDelay(2000);
			dance_flag = 1;
			*rotate_pid.error = 0;
			*rotate_pid.out_put = 0;
			rotating_flag = 0;
			circle_count++;
			break;

		case 4:
			servo_state  = 1;
			servo_state2 = 0;
			osDelay(4000);
			rotating_flag = 1;
			target_angle = -360 * 1;
			osDelay(2000);
			servo_state  = 0;
			servo_state2 = 0;
			osDelay(1000);
			servo_state  = 1;
			servo_state2 = 0;
			osDelay(1000);
			servo_state  = 1;
			servo_state2 = 1;
			osDelay(1000);
			servo_state  = 0;
			servo_state2 = 1;
			osDelay(1000);
			servo_state  = 0;
			servo_state2 = 0;
			osDelay(1000);
			target_angle = -360 * 2;
			osDelay(2000);
			servo_state  = 0;
			servo_state2 = 0;
			osDelay(1000);
			servo_state  = 1;
			servo_state2 = 0;
			osDelay(1000);
			servo_state  = 1;
			servo_state2 = 1;
			osDelay(1000);
			servo_state  = 0;
			servo_state2 = 1;
			osDelay(1000);
			servo_state  = 0;
			servo_state2 = 0;
			osDelay(1000);
			target_angle = -360 * 3;
			osDelay(2000);
			servo_state  = 0;
			servo_state2 = 0;
			osDelay(1000);
			servo_state  = 1;
			servo_state2 = 0;
			osDelay(1000);
			servo_state  = 1;
			servo_state2 = 1;
			osDelay(1000);
			servo_state  = 0;
			servo_state2 = 1;
			osDelay(1000);
			servo_state  = 0;
			servo_state2 = 0;
			osDelay(1000);
			rotating_flag = 0;
			circle_count++;
			break;

		case 5:
			rotating_flag = 1;
			target_angle = -360 * 4;
			osDelay(2000);
			servo_state = 1;
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
			if(fabs(*y_pid.error) < 0.07){
				if(fabs(*x_pid.error) < 0.07){
					*y_pid.error = 0;
					*x_pid.error = 0;
					*y_pid.out_put = 0;
					*x_pid.out_put = 0;
					// 1
					servo_state  = 1; // left bend
					servo_state2 = 1; // right bend
					osDelay(1000);    // 2
					servo_state  = 0; // left strait
					servo_state2 = 1; // right bend
					osDelay(1000);    // 3
					servo_state  = 0; // left strait
					servo_state2 = 0; // right strait
					osDelay(1000);    // 4
					servo_state  = 1; // left bend
					servo_state2 = 0; // right strait
					osDelay(1000);    // 5
					servo_state  = 1; // left bend
					servo_state2 = 1; // right bend
					osDelay(1000);    // 6
					servo_state  = 0; // left strait
					servo_state2 = 1; // right bend
					osDelay(1000);    // 7
					servo_state  = 0; // left strait
					servo_state2 = 0; // right strait
					osDelay(1000);    // 8
					servo_state  = 1; // left bend
					servo_state2 = 1; // right bend
					osDelay(1000);    // 9
					servo_state  = 0; // left strait
					servo_state2 = 1; // right bend
					osDelay(1000);    // 10
					servo_state  = 0; // left strait
					servo_state2 = 0; // right strait
					osDelay(1000);    // 11
					servo_state  = 1; // left bend
					servo_state2 = 0; // right strait
					osDelay(1000);    // 12
					servo_state  = 1; // left bend
					servo_state2 = 1; // right bend
					osDelay(1000);    // 13
					servo_state  = 0; // left strait
					servo_state2 = 0; // right strait
					osDelay(1000);    // 14
					servo_state  = 1; // left bend
					servo_state2 = 0; // right strait
					osDelay(1000);    // 15
					servo_state  = 1; // left bend
					servo_state2 = 1; // right bend
					osDelay(1000);    // 16
					servo_state  = 0; // left strait
					servo_state2 = 1; // right bend
					osDelay(1000);    // 17
					servo_state  = 0; // left strait
					servo_state2 = 0; // right strait
					osDelay(1000);    // 18
					servo_state  = 1; // left bend
					servo_state2 = 1; // right bend
					osDelay(1000);
					servo_state  = 0; // left strait
					servo_state2 = 0; // right strait
					circle_count++;
				}
			}
			break;

		case 7:
			target_pos_x = -3.5;
			target_pos_y = 0;
			osDelay(1);
			PID(&x_pid);
			PID(&y_pid);
			osDelay(1);
			if(fabs(*y_pid.error) < 0.07){
				if(fabs(*x_pid.error) < 0.07){
					*y_pid.error = 0;
					*x_pid.error = 0;
					*y_pid.out_put = 0;
					*x_pid.out_put = 0;
					circle_count++;
				}
			}
			break;

		case 8:
			target_pos_x = -4.0;
			target_pos_y = 0;
			osDelay(1);
			PID(&x_pid);
			PID(&y_pid);
			osDelay(1);
			if(fabs(*y_pid.error) < 0.07){
				if(fabs(*x_pid.error) < 0.07){
					*y_pid.error = 0;
					*x_pid.error = 0;
					*y_pid.out_put = 0;
					*x_pid.out_put = 0;
					circle_count++;
				}
			}
			break;

		case 9:
			target_pos_x = -3.5;
			target_pos_y = 0;
			osDelay(1);
			PID(&x_pid);
			PID(&y_pid);
			osDelay(1);
			if(fabs(*y_pid.error) < 0.07){
				if(fabs(*x_pid.error) < 0.07){
					*y_pid.error = 0;
					*x_pid.error = 0;
					*y_pid.out_put = 0;
					*x_pid.out_put = 0;
					circle_count++;
				}
			}
			break;

		case 10:
			target_pos_x = -3.0;
			target_pos_y = 0;
			osDelay(1);
			PID(&x_pid);
			PID(&y_pid);
			osDelay(1);
			if(fabs(*y_pid.error) < 0.07){
				if(fabs(*x_pid.error) < 0.07){
					*y_pid.error = 0;
					*x_pid.error = 0;
					*y_pid.out_put = 0;
					*x_pid.out_put = 0;
					circle_count++;
				}
			}
			break;

		case 11:
			target_pos_x = -3.5;
			target_pos_y = 0;
			osDelay(1);
			PID(&x_pid);
			PID(&y_pid);
			osDelay(1);
			if(fabs(*y_pid.error) < 0.07){
				if(fabs(*x_pid.error) < 0.07){
					*y_pid.error = 0;
					*x_pid.error = 0;
					*y_pid.out_put = 0;
					*x_pid.out_put = 0;
					circle_count++;
				}
			}
			break;

		case 12:
			target_pos_x = -4.0;
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

		case 13:
			target_pos_x = -3.5;
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

		case 15:
			target_pos_x = -3.5;
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

		case 16:
			target_pos_x = -4.0;
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

		case 17:
			target_pos_x = -3.5;
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

		case 18:
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
					//					osDelay(25000);
					circle_count++;
				}
			}
			break;

		case 19:
			target_pos_x = -3.5;
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

		case 20:
			target_pos_x = -4.0;
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

		case 21:
			target_pos_x = -3.5;
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

		case 22:
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
					servo_state  = 0;
					servo_state2 = 0;
					osDelay(1000);
					servo_state  = 1;
					servo_state2 = 0;
					osDelay(1000);
					servo_state  = 1;
					servo_state2 = 1;
					osDelay(1000);
					servo_state  = 2;
					servo_state2 = 1;
					osDelay(1000);
					servo_state  = 2;
					servo_state2 = 2;
					osDelay(1000);
					servo_state  = 1;
					servo_state2 = 2;
					osDelay(1000);
					servo_state  = 1;
					servo_state2 = 1;
					osDelay(1000);
					servo_state  = 1;
					servo_state2 = 0;
					osDelay(1000);
					servo_state  = 0;
					servo_state2 = 0;
//					// 1
//					servo_state  = 1; // left bend
//					servo_state2 = 1; // right bend
//					osDelay(1000);    // 2
//					servo_state  = 0; // left strait
//					servo_state2 = 1; // right bend
//					osDelay(1000);    // 3
//					servo_state  = 0; // left strait
//					servo_state2 = 0; // right strait
//					osDelay(1000);    // 4
//					servo_state  = 1; // left bend
//					servo_state2 = 0; // right strait
//					osDelay(1000);    // 5
//					servo_state  = 1; // left bend
//					servo_state2 = 1; // right bend
//					osDelay(1000);    // 6
//					servo_state  = 0; // left strait
//					servo_state2 = 1; // right bend
//					osDelay(1000);    // 7
//					servo_state  = 0; // left strait
//					servo_state2 = 0; // right strait
//					osDelay(1000);    // 8
//					servo_state  = 1; // left bend
//					servo_state2 = 1; // right bend
//					osDelay(1000);    // 9
//					servo_state  = 0; // left strait
//					servo_state2 = 1; // right bend
//					osDelay(1000);    // 10
//					servo_state  = 0; // left strait
//					servo_state2 = 0; // right strait
//					osDelay(1000);    // 11
//					servo_state  = 1; // left bend
//					servo_state2 = 0; // right strait
//					osDelay(1000);    // 12
//					servo_state  = 1; // left bend
//					servo_state2 = 1; // right bend
//					osDelay(1000);    // 13
//					servo_state  = 0; // left strait
//					servo_state2 = 0; // right strait
//					osDelay(1000);    // 14
//					servo_state  = 1; // left bend
//					servo_state2 = 0; // right strait
//					osDelay(1000);    // 15
//					servo_state  = 1; // left bend
//					servo_state2 = 1; // right bend
//					osDelay(1000);    // 16
//					servo_state  = 0; // left strait
//					servo_state2 = 1; // right bend
//					osDelay(1000);    // 17
//					servo_state  = 0; // left strait
//					servo_state2 = 0; // right strait
//					osDelay(1000);    // 18
//					servo_state  = 1; // left bend
//					servo_state2 = 1; // right bend
//					osDelay(1000);
//					servo_state  = 0; // left strait
//					servo_state2 = 0; // right strait
					circle_count++;
				}
			}
			break;


		case 23:
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

		case 24:
//			osDelay(3000);
			rotating_flag = 1;
			target_angle = -360 * 5;
			osDelay(2000);
			target_angle = -360 * 4;
			rotating_flag = 0;

//			servo_state  = 0;
//			servo_state2 = 0;
//			osDelay(1000);
//			servo_state  = 1;
//			servo_state2 = 0;
//			osDelay(1000);
//			servo_state  = 1;
//			servo_state2 = 1;
//			osDelay(1000);
//			servo_state  = 2;
//			servo_state2 = 1;
//			osDelay(1000);
//			servo_state  = 2;
//			servo_state2 = 2;
//			osDelay(1000);
//			servo_state  = 1;
//			servo_state2 = 2;
//			osDelay(1000);
//			servo_state  = 1;
//			servo_state2 = 1;
//			osDelay(1000);
//			servo_state  = 1;
//			servo_state2 = 0;
//			osDelay(1000);
//			servo_state  = 0;
//			servo_state2 = 0;

			circle_count++;
			break;

		case 25:
			rotating_flag = 1;
			target_angle = -360 * 5;
			osDelay(2500);
			rotating_flag = 0;
			circle_count++;
			break;

		case 26:
			rotating_flag = 1;
			target_angle = -360 * 6;
			osDelay(2500);
			rotating_flag = 0;
			circle_count++;
			break;

		case 27:
			rotating_flag = 1;
			target_angle = -360 * 7;
			osDelay(2500);
			rotating_flag = 0;
			circle_count++;
			break;

		case 28:
			rotating_flag = 1;
			target_angle = -360 * 8;
			osDelay(2500);
			rotating_flag = 0;
			circle_count++;
			break;

		case 29:
			rotating_flag = 1;
			target_angle = -360 * 9;
			osDelay(2500);
			rotating_flag = 0;
			circle_count++;
			break;

		case 30:
			rotating_flag = 1;
			target_angle = -360 * 10;
			osDelay(2500);
			rotating_flag = 0;
			circle_count++;
			break;

		case 31:
			target_pos_x = -2.0;
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

		case 32:
			target_pos_x = -2.5;
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

		case 33:
			target_pos_x = -2.0;
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

		case 34:
			target_pos_x = -1.5;
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

		case 35:
			target_pos_x = -2.0;
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

		case 36:
			target_pos_x = -2.5;
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

		case 37:
			target_pos_x = -2.0;
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

		case 38:
			target_pos_x = -1.5;
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

		if(circle_count == 1 || circle_count == 24){

			left_target_position[red_servo] = 751;
			left_target_position[main_joint] = 2500;
			left_target_position[mid_joint] = 2000;
			left_target_position[gripper_joint] = 659;
			left_target_position[gripper] = 2400;
			apply_servo_param_left();

		}

		if (circle_count == 2 || circle_count == 3){
			// Pose 2
			//				{1400.0, 2500.0, 2000.0, 1600.0, 2400.0};
			left_target_position[red_servo] = 1400;
			left_target_position[main_joint] = 2500;
			left_target_position[mid_joint] = 2000;
			left_target_position[gripper_joint] = 1600;
			left_target_position[gripper] = 2400;
			apply_servo_param_left();

		}

		if (circle_count == 4){

			if(servo_state == 1){
				left_target_position[red_servo] = 667;
				left_target_position[main_joint] = 2500;
				left_target_position[mid_joint] = 678;
				left_target_position[gripper_joint] = 636;
				left_target_position[gripper] = 2400;
				apply_servo_param_left();

			}else{
				left_target_position[red_servo] = 751;
				left_target_position[main_joint] = 2500;
				left_target_position[mid_joint] = 2000;
				left_target_position[gripper_joint] = 659;
				left_target_position[gripper] = 2400;
				apply_servo_param_left();

			}
		}

		if (circle_count == 5){

			left_target_position[red_servo] = 751;
			left_target_position[main_joint] = 2500;
			left_target_position[mid_joint] = 2000;
			left_target_position[gripper_joint] = 659;
			left_target_position[gripper] = 2400;
			apply_servo_param_left();

		}


		if (circle_count == 7 || circle_count == 8 || circle_count == 11 || circle_count == 12 || circle_count == 15 || circle_count == 16 || circle_count == 19 || circle_count == 20  || circle_count == 31  || circle_count == 32  || circle_count == 35  || circle_count == 36 ){
			// Pose 2
			left_target_position[red_servo] = 751;
			left_target_position[main_joint] = 2500;
			left_target_position[mid_joint] = 2000;
			left_target_position[gripper_joint] = 659;
			left_target_position[gripper] = 2400;
			apply_servo_param_left();
		}

		if (circle_count == 9 || circle_count == 10 || circle_count == 13 || circle_count == 14 || circle_count == 17 || circle_count == 18 || circle_count == 21 || circle_count == 33  || circle_count == 34  || circle_count == 37  || circle_count == 38 ){
			// Pose 4
			//			servo_go_position(left, 667, 2500, 572, 743, 2400, 1000); //ok
			left_target_position[red_servo] = 667;
			left_target_position[main_joint] = 2500;
			left_target_position[mid_joint] = 678;
			left_target_position[gripper_joint] = 636;
			left_target_position[gripper] = 2400;
			apply_servo_param_left();
		}
		//
		if (circle_count == 6){
//			Pose 5
			if(servo_state == 1){
				left_target_position[red_servo] = 1365;
				left_target_position[main_joint] = 2100;
				left_target_position[mid_joint] = 500;
				left_target_position[gripper_joint] = 760;
				left_target_position[gripper] = 2400;
				apply_servo_param_left();
			}else{
				left_target_position[red_servo] = 1425;
				left_target_position[main_joint] = 2500;
				left_target_position[mid_joint] = 1520;
				left_target_position[gripper_joint] = 760;
				left_target_position[gripper] = 2400;
				apply_servo_param_left();

//				osDelay(1750);
			}
		}


		if (circle_count == 22){

			if(servo_state == 1){
				left_target_position[red_servo] = 667;
				left_target_position[main_joint] = 2500;
				left_target_position[mid_joint] = 678;
				left_target_position[gripper_joint] = 636;
				left_target_position[gripper] = 2400;
				apply_servo_param_left();

			}else if(servo_state == 2){
				left_target_position[red_servo] = 1110;
				left_target_position[main_joint] = 2500;
				left_target_position[mid_joint] = 2000;
				left_target_position[gripper_joint] = 659;
				left_target_position[gripper] = 2400;
				apply_servo_param_left();

			}
			else{
				left_target_position[red_servo] = 751;
				left_target_position[main_joint] = 2500;
				left_target_position[mid_joint] = 2000;
				left_target_position[gripper_joint] = 659;
				left_target_position[gripper] = 2400;
				apply_servo_param_left();
			}
		}

		if(dance_flag){

			left_target_position[red_servo] = 675;
			left_target_position[main_joint] = 2500;
			left_target_position[mid_joint] = 1200;
			left_target_position[gripper_joint] = 730;
			left_target_position[gripper] = 2400;

			apply_servo_param_left();

			osDelay(800);


			left_target_position[red_servo] = 1400;
			left_target_position[main_joint] = 2500;
			left_target_position[mid_joint] = 1200;
			left_target_position[gripper_joint] = 730;
			left_target_position[gripper] = 2400;

			apply_servo_param_left();

			osDelay(800);

			left_target_position[red_servo] = 675;
			left_target_position[main_joint] = 2500;
			left_target_position[mid_joint] = 1200;
			left_target_position[gripper_joint] = 730;
			left_target_position[gripper] = 2400;

			apply_servo_param_left();

			osDelay(800);


			left_target_position[red_servo] = 1400;
			left_target_position[main_joint] = 2500;
			left_target_position[mid_joint] = 1200;
			left_target_position[gripper_joint] = 730;
			left_target_position[gripper] = 2400;

			apply_servo_param_left();

			osDelay(800);

			left_target_position[red_servo] = 751;
			left_target_position[main_joint] = 2500;
			left_target_position[mid_joint] = 2000;
			left_target_position[gripper_joint] = 659;
			left_target_position[gripper] = 2400;

			apply_servo_param_left();

			dance_flag = 0;

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

		if(circle_count == 1 || circle_count == 24){
			// Pose 1

			right_target_position[red_servo] 	 = 1800;
			right_target_position[main_joint] 	 = 2300;
			right_target_position[mid_joint] 	 = 2500;
			right_target_position[gripper_joint] = 1800;
			right_target_position[gripper] 		 = 2400;
			apply_servo_param_right();

		}

		if (circle_count == 2 || circle_count == 3){
			// Pose 2
			//			servo_go_position(right, 1116, 2500, 1575, 1802, 2400, 1000); //ok
			right_target_position[red_servo] 	 = 1100;
			right_target_position[main_joint] 	 = 2420;
			right_target_position[mid_joint] 	 = 2320;
			right_target_position[gripper_joint] = 1911;
			right_target_position[gripper] 		 = 1800;
			apply_servo_param_right();
		}

//		if (circle_count == 4){
//			// Pose 1
//			//			servo_go_position(right, 1104, 2420, 2317, 1911, 1800, 1000); // ok
//			//			if(servo_state == 0){
//			//
//			//				right_target_position[red_servo] 	 = 1116;
//			//				right_target_position[main_joint] 	 = 2500;
//			//				right_target_position[mid_joint] 	 = 2022;
//			//				right_target_position[gripper_joint] = 1802;
//			//				right_target_position[gripper] 		 = 2400;
//			//
//			//			}
//			//			if(servo_state == 1){
//			//				right_target_position[red_servo] 	 = 1800;
//			//				right_target_position[main_joint] 	 = 2500;
//			//				right_target_position[mid_joint] 	 = 1030;
//			//				right_target_position[gripper_joint] = 1730;
//			//				right_target_position[gripper] 		 = 2400;
//			//			}
//		}

		if (circle_count == 4){

			if(servo_state2 == 1){
				right_target_position[red_servo] 	 = 1826;
				right_target_position[main_joint] 	 = 2500;
				right_target_position[mid_joint] 	 = 1118;
				right_target_position[gripper_joint] = 1701;
				right_target_position[gripper] 		 = 2400;
				apply_servo_param_right();

//				osDelay(1750);
			}else{
				right_target_position[red_servo] 	 = 1800;
				right_target_position[main_joint] 	 = 2300;
				right_target_position[mid_joint] 	 = 2500;
				right_target_position[gripper_joint] = 1800;
				right_target_position[gripper] 		 = 2400;

				apply_servo_param_right();
//				osDelay(1750);

			}

		}

		if(circle_count == 5){

			right_target_position[red_servo] 	 = 1800;
			right_target_position[main_joint] 	 = 2300;
			right_target_position[mid_joint] 	 = 2500;
			right_target_position[gripper_joint] = 1800;
			right_target_position[gripper] 		 = 2400;
			apply_servo_param_right();

		}


		if (circle_count == 7 || circle_count == 8 || circle_count == 11 || circle_count == 12 || circle_count == 15 || circle_count == 16 || circle_count == 19 || circle_count == 20 || circle_count == 31  || circle_count == 32  || circle_count == 35  || circle_count == 36 ){
			// Pose 4
			//			servo_go_position(right, 1797, 2500, 1030, 1726, 2400, 1000); //ok
			right_target_position[red_servo] 	 = 1826;
			right_target_position[main_joint] 	 = 2500;
			right_target_position[mid_joint] 	 = 1118;
			right_target_position[gripper_joint] = 1701;
			right_target_position[gripper] 		 = 2400;
			apply_servo_param_right();
		}

		if (circle_count == 9 || circle_count == 10 || circle_count == 13 || circle_count == 14 || circle_count == 17 || circle_count == 18 || circle_count == 21 || circle_count == 33  || circle_count == 34  || circle_count == 37  || circle_count == 38 ){
			// Pose 5
			//			servo_go_position(right, 1116, 2500, 2022, 1802, 2400, 1000); //ok
			right_target_position[red_servo] 	 = 1800;
			right_target_position[main_joint] 	 = 2300;
			right_target_position[mid_joint] 	 = 2500;
			right_target_position[gripper_joint] = 1800;
			right_target_position[gripper] 		 = 2400;
			apply_servo_param_right();
		}

		if (circle_count == 6){
			if(servo_state2 == 1){
				right_target_position[red_servo] = 1100;
				right_target_position[main_joint] = 2060;
				right_target_position[mid_joint] = 1090;
				right_target_position[gripper_joint] = 1800;
				right_target_position[gripper] = 2400;
				apply_servo_param_right();
			}else{
				right_target_position[red_servo] = 1120;
				right_target_position[main_joint] = 2500;
				right_target_position[mid_joint] = 2020;
				right_target_position[gripper_joint] = 1800;
				right_target_position[gripper] = 2400;
				apply_servo_param_right();
			}
		}

		if (circle_count == 22){

			if(servo_state2 == 1){
				right_target_position[red_servo] 	 = 1826;
				right_target_position[main_joint] 	 = 2500;
				right_target_position[mid_joint] 	 = 1118;
				right_target_position[gripper_joint] = 1701;
				right_target_position[gripper] 		 = 2400;
				apply_servo_param_right();

			}else if(servo_state2 == 2){
				right_target_position[red_servo] = 1380;
				right_target_position[main_joint] = 2500;
				right_target_position[mid_joint] = 2500;
				right_target_position[gripper_joint] = 659;
				right_target_position[gripper] = 2400;
				apply_servo_param_right();

			}
			else{
				right_target_position[red_servo] 	 = 1800;
				right_target_position[main_joint] 	 = 2300;
				right_target_position[mid_joint] 	 = 2500;
				right_target_position[gripper_joint] = 1800;
				right_target_position[gripper] 		 = 2400;
				apply_servo_param_right();
			}
		}

		if(dance_flag){

			right_target_position[red_servo] 	 = 1800;
			right_target_position[main_joint] 	 = 2300;
			right_target_position[mid_joint] 	 = 1700;
			right_target_position[gripper_joint] = 2200;
			right_target_position[gripper] 		 = 2400;

			apply_servo_param_right();

			osDelay(1000);


			right_target_position[red_servo] 	 = 1100;
			right_target_position[main_joint] 	 = 2200;
			right_target_position[mid_joint] 	 = 1800;
			right_target_position[gripper_joint] = 2200;
			right_target_position[gripper] 		 = 2400;


			apply_servo_param_right();

			osDelay(1000);

			right_target_position[red_servo] 	 = 1800;
			right_target_position[main_joint] 	 = 2300;
			right_target_position[mid_joint] 	 = 1700;
			right_target_position[gripper_joint] = 2200;
			right_target_position[gripper] 		 = 2400;

			apply_servo_param_right();

			osDelay(1000);


			right_target_position[red_servo] 	 = 1100;
			right_target_position[main_joint] 	 = 2200;
			right_target_position[mid_joint] 	 = 1800;
			right_target_position[gripper_joint] = 2200;
			right_target_position[gripper] 		 = 2400;


			apply_servo_param_right();

			osDelay(1000);

			right_target_position[red_servo] 	 = 1800;
			right_target_position[main_joint] 	 = 2300;
			right_target_position[mid_joint] 	 = 2500;
			right_target_position[gripper_joint] = 1800;
			right_target_position[gripper] 		 = 2400;
			apply_servo_param_right();

			dance_flag = 0;
		}
	}
}


void stop_all(void){

	HAL_NVIC_SystemReset();
	target_angle = YawAngle;
	automatic = 0;
	error_x = 0;
	error_y = 0;
	circle_count = 0;
	PIDDelayInit(&x_pid);
	PIDDelayInit(&y_pid);
	PIDDelayInit(&yaw_pid);
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

void apply_servo_param_right(void){


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

void apply_servo_param_left(void){

	for (int j = 0; j < 5; j++){

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
