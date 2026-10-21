
/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include <stdio.h>
#include <string.h>
#include "adc.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct
{
   	float kp, ki, kd; // 比例、积分、微分系数
    float error, lastError; // 当前误差、上一次误差
    float integral, maxIntegral; // 积分累计值、积分限幅
    float output, maxOutput; // 控制输出、输出限幅
    float deadzone; // 死区
}PID;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define ENCODER_DEADZONE 2  // 编码器增量绝对值小于此值时视为静止
#define PID_OUTPUT_DEADZONE 5  // PID输出绝对值小于此值时置零
// 走迷宫时用于计算前进距离的绝对值宏
#define ABS_INT(x) (( (x) >= 0 ) ? (x) : -(x))
// 手动模式下直行速度调节范围与步长
#define SPEED_MIN      20
#define SPEED_MAX      120
#define SPEED_STEP     10
#define STRAIGHT_SLEW_STEP 4   // 直行角度环输出变化限幅，抑制左右摆动
#define TURN_SLEW_STEP     6   // 转弯PID输出变化限幅，降低振荡
#define MOTOR_FILTER_ALPHA 0.45f // 编码器速度一阶低通滤波系数
#define STRAIGHT_BALANCE_KP 0.35f   // 直行速度环左右平衡系数
#define STRAIGHT_BALANCE_MAX 8      // 直行速度环最大修正量
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint32_t ms_Tick;
uint32_t last_send_time = 0;  // 上一次发送状态的时间
uint8_t uart3_send_buffer[150];  // USART3发送缓冲区

// 定时与按键扫描变量
unsigned char Key_Slow_Down;
//key
unsigned char Key_Value;
unsigned char Key_Old,Key_Down,Key_Up;
uint8_t Key_Test0,Key_Test1,Key_Test2;
// 红外传感器数据
uint8_t CGQ_Data1,CGQ_Data2,CGQ_Data3,CGQ_Data4;// 1、2、3、4分别对应前、后、左、右红外
// USART2接收JY61P姿态数据
uint8_t uartdata2;// 串口当前接收到的字节
uint8_t Uart_Receive_String2[20];
uint8_t Uart_Receive_Flag2=0,Uart_Index2=0,Uart_State2=0;
int16_t jy61p_Roll,jy61p_Pitch,jy61p_Yaw;
float Roll,Pitch,Yaw;
uint8_t jy61p_Sum;
// 编码器变量
int EncoderRight,enc1_last,enc1_now;// 右轮采样周期内的编码器增量及计数值
char DirectionRight;// 右轮方向标志，由编码器计数方向确定
int EncoderLeft,enc2_last,enc2_now;// 左轮采样周期内的编码器增量及计数值
char DirectionLeft;// 左轮方向标志，由编码器计数方向确定
//PID - 修改速度环PID参数，降低比例增益、适当提高微分，抑制左右摆动
float Motor_kp=1.7f,Motor_ki=0.25f,Motor_kd=1.8f;//电机速度环默认参数，可按现场情况微调,2.0 0.25 1.8
uint16_t MotorRight_Target=20;// 右轮基础速度目标
PID MotorRight_PID={0};
uint16_t MotorLeft_Target=20;// 左轮基础速度目标
PID MotorLeft_PID={0};
float EncoderRight_filt = 0.0f;  // 过滤后的右轮速度
float EncoderLeft_filt  = 0.0f;  // 过滤后的左轮速度
//偏航角度环PID - 调低比例，略升微分，减少直行摆动
float Angle_kp=1.5f,Angle_ki=0.004f,Angle_kd=1.8f;//1.3
PID Angle_PID={0};
// 转弯独立PID，转向时采用更柔和且低幅度的参数
float Turn_kp=1.0f,Turn_ki=0.0050f,Turn_kd=1.2f;//0.8 0.0050 0.9
PID Turn_PID={0};
float turn_output_filtered = 0.0f;   // 转弯PID输出平滑值
float Yaw_PID_Error = 0.0f;          // 记录当前使用的偏航PID误差
float Yaw_PID_Output = 0.0f;         // 记录当前使用的偏航PID输出
float Yaw_Target=0.0f;
uint8_t Angle_Target_Valid=0;
int32_t MotorRight_Target_Adjusted=0;
int32_t MotorLeft_Target_Adjusted=0;
uint16_t Straight_Speed = SPEED_MIN;       // 直行动作的默认目标速度，可通过按键调节
int16_t Straight_Trim = 6;                 // 直行左右轮平衡微调：>0 提升左轮/降低右轮，矫正左偏
uint8_t Current_Action = 4;         // 当前动作编号：0直行 1左转 2右转 3调头 4停止
// USART3状态发送



//==================== 迷宫与智能鼠走迷宫相关变量 ====================//
#define MAZE_SIZE          8	// 迷宫的大小
#define MAZE_CENTER_MIN    3   	// 中心区域 [3,3]~[4,4]
#define MAZE_CENTER_MAX    4	

// 地图不是一张图像，而是一个8×8的结构体数组,每个结构体描述一个格子，64个结构体就能表达整个迷宫。
typedef struct
{
	uint8_t walls;     // 当前格子四周哪些方向有墙,使用 walls 的低4个二进制位，每一位对应一个方向。
	uint8_t distance;  // 当前格子到终点的洪泛距离
	uint8_t visited;   // 是否走到过该格 0未访问，1已访问
} MazeCell;

// 方向定义：0北 1东 2南 3西
typedef enum
{
	DIR_NORTH = 0,
	DIR_EAST  = 1,
	DIR_SOUTH = 2,
	DIR_WEST  = 3
} Direction;

// 迷宫地图 & 机器人位姿
MazeCell maze[MAZE_SIZE][MAZE_SIZE];
int8_t robot_x = 7;            // 当前所在格子 X (0~7)，起点为右下角(7,0)
int8_t robot_y = 0;            // 当前所在格子 Y (0~7)，起点为右下角(7,0)
Direction robot_dir = DIR_NORTH; // 当前朝向，起始朝向为北（向上）

// 方向增量：0北 1东 2南 3西
const int8_t dir_dx[4] = {0, 1, 0, -1};		// 和Direction配合使用，往某个方向，就是针对着增减
const int8_t dir_dy[4] = {1, 0, -1, 0};



// 运动状态机
typedef enum
{
	MOTION_IDLE = 0,
	MOTION_MOVE_ONE_CELL,   // 正在前进一个单元格
	MOTION_TURNING          // 正在转向
} MotionState;


MotionState motion_state = MOTION_IDLE;
int32_t move_encoder_acc = 0;      // 用于统计前进距离的编码器累积值
uint32_t motion_start_tick = 0;    // 启动当前动作时的时间戳，用于超时检测
// 每个单元格对应的平均编码器增量（左右轮平均），需要根据实际底盘调试
#define COUNTS_PER_CELL     256

// 转弯完成判断阈值（角度误差小于该值判定转向结束）
#define TURN_ANGLE_EPS      2.0f

// 启动自动迷宫模式的标志（可通过按键控制）
uint8_t maze_auto_mode = 0;
uint8_t maze_finished   = 0;

// 迷宫控制函数前置声明
void Maze_Init(void);
void Maze_UpdateWalls(void);
void Maze_ComputeDistances(void);
uint8_t Maze_IsInCenter(uint8_t x, uint8_t y);
uint8_t Maze_ChooseNextMove(void);
void Maze_Controller(void);
void Motion_StartForwardOneCell(void);
void Motion_StartTurnLeft(void);
void Motion_StartTurnRight(void);
void Motion_StartTurnBack(void);
void Motion_UpdateByEncoders(void);
void Motion_Stop(void);

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void Key_Proc(void);
void JY61P_Proc(void);
void Set_RightLun_Duty(int Duty);
void Set_LeftLun_Duty(int Duty);
uint8_t CGQ_Read(uint8_t Num);
void SendString3(uint8_t* UartString);
void Send_Debug_Data(void);
void Reset_PID(void);
void PID_Init(PID *pid, float p, float i, float d, float maxI, float maxOut);
void Motor_Proc(void);
void Angle_Control_Update(void);
void Reset_Angle_PID(void);
float NormalizeAngle(float angle);
void Action(uint8_t num);
// 走迷宫相关函数声明
void Maze_Init(void);
void Maze_Controller(void);
void Maze_UpdateWalls(void);
void Maze_ComputeDistances(void);
uint8_t Maze_IsInCenter(uint8_t x, uint8_t y);
uint8_t Maze_ChooseNextMove(void);
void Motion_StartForwardOneCell(void);
void Motion_StartTurnLeft(void);
void Motion_StartTurnRight(void);
void Motion_StartTurnBack(void);
void Motion_Stop(void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */


// 这套电脑鼠真正实现了两种运行模式——手动测试模式和自动迷宫模式
// 手动测试模式:电后默认处于这个模式，而且初始是停止状态。
// 可以通过按键依次让电脑鼠直行、左转、右转、掉头和停止，也可以调整直行速度。调试底盘运动能力
// 自动迷宫模式:按下KEY0后，电脑鼠会重新初始化地图，开始执行
// 边探索、边规划，不是先把整个迷宫走完，再计算最优路径。

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM1_Init();
  MX_TIM4_Init();
  MX_TIM2_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  MX_TIM3_Init();
  MX_TIM5_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */
	HAL_TIM_Base_Start_IT(&htim2);// 启动TIM2周期中断
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1);// 启动TIM1通道1的PWM输出
	HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_1);// 启动TIM4通道1的PWM输出
	HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);// 启动TIM3编码器接口
	HAL_TIM_Encoder_Start(&htim5, TIM_CHANNEL_ALL);// 启动TIM5编码器接口
	
	// 初始化时先停止电机
	Set_RightLun_Duty(0);
	Set_LeftLun_Duty(0);
	
	HAL_UART_Receive_IT(&huart2, (uint8_t *)&uartdata2,1); // 启动USART2单字节中断接收，接收到uartdata2
	
	// 初始化速度环和角度环PID参数
	PID_Init(&MotorRight_PID, Motor_kp, Motor_ki, Motor_kd, 300.0f, 800.0f); // 初始化右轮速度环PID及积分、输出限幅
	PID_Init(&MotorLeft_PID, Motor_kp, Motor_ki, Motor_kd, 300.0f, 800.0f); // 初始化左轮速度环PID及积分、输出限幅
	PID_Init(&Angle_PID, Angle_kp, Angle_ki, Angle_kd, 150.0f, 150.0f); // 初始化直行角度环PID及积分、输出限幅
	PID_Init(&Turn_PID,  Turn_kp,  Turn_ki,  Turn_kd, 100.0f, 100.0f); // 转弯专用PID，限制输出幅度
	MotorRight_Target_Adjusted = MotorRight_Target;
	MotorLeft_Target_Adjusted = MotorLeft_Target;

 	// 开始时将编码器计数置为中值,防止跳变
 	__HAL_TIM_SET_COUNTER(&htim3, 32768);
	__HAL_TIM_SET_COUNTER(&htim5, 32768);
	enc1_now = 32768;
	enc1_last = 32768;
	enc2_now = 32768;
	enc2_last = 32768;

	// 初始化迷宫数据，但默认不启用迷宫自动模式
	Maze_Init();
	maze_auto_mode = 0;
	maze_finished  = 0;
	Current_Action = 4;   // 默认停止
	Action(4);
	
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	/* USER CODE END WHILE */

	/* USER CODE BEGIN 3 */
	Key_Proc();  // 代码中按键能够切换自动迷宫模式、切换手动动作和调整直行速度。 按键控制为什么放在while里面？
	JY61P_Proc();//处理JY61P姿态传感器发送的数据，主要获得机器人当前的偏航角 Yaw。
	Motor_Proc();//将左右电机PID控制器当前的输出值，送给电机驱动,并不负责计算PID，只是将已经计算好的控制量转换为PWM和方向信号。
	// 红外传感器读取
	CGQ_Data1=CGQ_Read(1);		// 前
	CGQ_Data2=CGQ_Read(2);		// 后
	CGQ_Data3=CGQ_Read(3);		// 左
	CGQ_Data4=CGQ_Read(4);		// 右
	// 在当前迷宫规划实现中，主要使用前、左、右三个方向的信息，后方传感器没有参与迷宫墙体更新。

	// 只有满足两个条件才执行自动迷宫控制：自动模式已开启 AND 尚未完成迷宫
	if(maze_auto_mode && !maze_finished){
		Maze_Controller();// 负责判断当前运动状态、更新地图、规划下一步，并启动相应的运动动作
		// 它并不直接负责计算左右电机PWM，而是给下层运动控制模块下达指令。
	}
	
	// 每500ms通过串口发送一次调试数据
	if(ms_Tick - last_send_time >= 500){	
		last_send_time = ms_Tick;
		Send_Debug_Data();
	}
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){   
	if(htim == &htim2){  // 只处理TIM2的周期中断
		ms_Tick++;
		if(++Key_Slow_Down ==10) Key_Slow_Down=0;
		
		// 每50ms采样一次编码器
		if((ms_Tick%50)==0){
			// 通过TIM3和TIM5读取编码器计数，计算轮速增量
			enc1_last=enc1_now;
			enc1_now = (uint16_t)(__HAL_TIM_GET_COUNTER(&htim3));// 读取TIM3计数值
			DirectionRight= __HAL_TIM_IS_TIM_COUNTING_DOWN(&htim3);// 读取右轮编码器计数方向，实际正反方向需实车标定
			
			// 根据计数方向计算右轮编码器增量
			if(DirectionRight){
				EncoderRight = enc1_now - enc1_last;
			}
			else{
				EncoderRight = enc1_last - enc1_now;
			}
			
			// 编码器死区处理，抑制静止时的小幅抖动
			if(EncoderRight > -ENCODER_DEADZONE && EncoderRight < ENCODER_DEADZONE){
				EncoderRight = 0;
			}
			
			enc2_last=enc2_now;
			enc2_now = (uint16_t)(__HAL_TIM_GET_COUNTER(&htim5));// 读取TIM5计数值
			if(__HAL_TIM_IS_TIM_COUNTING_DOWN(&htim5)==0){
				DirectionLeft=1;
			}
			else{
				DirectionLeft=0;
			}
			
			// 根据计数方向计算左轮编码器增量
			if(DirectionLeft){
				EncoderLeft = enc2_now - enc2_last;
			}
			else{
				EncoderLeft = enc2_last - enc2_now;
			}
			
			// 编码器死区处理，抑制静止时的小幅抖动
			if(EncoderLeft > -ENCODER_DEADZONE && EncoderLeft < ENCODER_DEADZONE){
				EncoderLeft = 0;
			}

			//=== 走迷宫：更新前进一个单元格的编码器累积，并做简单堵转检测 ===//
			if(motion_state == MOTION_MOVE_ONE_CELL){
				// 计算左右轮编码器的平均增量（取绝对值）
				int step_right = ABS_INT(EncoderRight);
				int step_left = ABS_INT(EncoderLeft);
				int step = (step_right + step_left) / 2;
				move_encoder_acc += step;
				
				// 检查是否达到一个单元格的距离（256个编码器计数）
				if(move_encoder_acc >= COUNTS_PER_CELL){
					move_encoder_acc = 0;
					// 根据当前朝向更新所在格子坐标
					int8_t nx = robot_x + dir_dx[robot_dir];
					int8_t ny = robot_y + dir_dy[robot_dir];
					if(nx >= 0 && nx < MAZE_SIZE && ny >= 0 && ny < MAZE_SIZE){
						robot_x = nx;
						robot_y = ny;
						maze[robot_x][robot_y].visited = 1;
					}
					Motion_Stop();
				}else{
					// 堵转检测：如果超过1.5秒且编码器累积值小于1/8单元格，认为前方被阻挡
					uint32_t elapsed = ms_Tick - motion_start_tick;
					if(elapsed > 1500 && move_encoder_acc < (COUNTS_PER_CELL / 8)){
						// 标记前方为墙
						uint8_t dir_front = robot_dir;
						uint8_t dir_back  = (robot_dir + 2) & 0x03;
						maze[robot_x][robot_y].walls |= (1 << dir_front);
						int8_t nx = robot_x + dir_dx[dir_front];
						int8_t ny = robot_y + dir_dy[dir_front];
						if(nx >= 0 && nx < MAZE_SIZE && ny >= 0 && ny < MAZE_SIZE){
							maze[nx][ny].walls |= (1 << dir_back);
						}
						Motion_Stop();
					}
				}
			}
		}
		
		// ============ 右轮速度环PID：每10ms更新一次 ============
		if((ms_Tick%10)==0){
			Angle_Control_Update();

			MotorRight_PID.lastError = MotorRight_PID.error;
			// 速度一阶滤波，抑制编码器抖动
			EncoderRight_filt += MOTOR_FILTER_ALPHA * ((float)EncoderRight - EncoderRight_filt);
			EncoderLeft_filt  += MOTOR_FILTER_ALPHA * ((float)EncoderLeft  - EncoderLeft_filt);

			MotorRight_PID.error = (float)MotorRight_Target_Adjusted - EncoderRight_filt;
			
			// 只有误差较小时才累积积分
			if(MotorRight_PID.error < 15 && MotorRight_PID.error > -15){
				MotorRight_PID.integral += MotorRight_PID.error;
			}
			else{
				// 误差较大时衰减积分，避免积分饱和
				MotorRight_PID.integral *= 0.95;
			}
			
			// 积分限幅，避免积分饱和
			if(MotorRight_PID.integral > MotorRight_PID.maxIntegral) {
				MotorRight_PID.integral = MotorRight_PID.maxIntegral;
			}
			if(MotorRight_PID.integral < -MotorRight_PID.maxIntegral) {
				MotorRight_PID.integral = -MotorRight_PID.maxIntegral;
			}
			
			// 计算PID控制输出
			MotorRight_PID.output = MotorRight_PID.kp * MotorRight_PID.error
								  + MotorRight_PID.ki * MotorRight_PID.integral
								  + MotorRight_PID.kd * (MotorRight_PID.error - MotorRight_PID.lastError);
			
			// 输出限幅，限制电机控制量
			if(MotorRight_PID.output > MotorRight_PID.maxOutput) {
				MotorRight_PID.output = MotorRight_PID.maxOutput;
			}
			if(MotorRight_PID.output < -MotorRight_PID.maxOutput) {
				MotorRight_PID.output = -MotorRight_PID.maxOutput;
			}
			
			// 输出死区处理，小幅控制输出直接置零
			if(MotorRight_PID.output > -PID_OUTPUT_DEADZONE && MotorRight_PID.output < PID_OUTPUT_DEADZONE){
				MotorRight_PID.output = 0;
			}
			
			// ============ 左轮速度环PID ============
			MotorLeft_PID.lastError = MotorLeft_PID.error;
			MotorLeft_PID.error = (float)MotorLeft_Target_Adjusted - EncoderLeft_filt;
			
			// 只有误差较小时才累积积分
			if(MotorLeft_PID.error < 15 && MotorLeft_PID.error > -15){
				MotorLeft_PID.integral += MotorLeft_PID.error;
			}
			else{
				// 误差较大时衰减积分，避免积分饱和
				MotorLeft_PID.integral *= 0.95;
			}
			
			// 积分限幅，避免积分饱和
			if(MotorLeft_PID.integral > MotorLeft_PID.maxIntegral) {
				MotorLeft_PID.integral = MotorLeft_PID.maxIntegral;
			}
			if(MotorLeft_PID.integral < -MotorLeft_PID.maxIntegral) {
				MotorLeft_PID.integral = -MotorLeft_PID.maxIntegral;
			}
			
			// 计算PID控制输出
			MotorLeft_PID.output = MotorLeft_PID.kp * MotorLeft_PID.error
								 + MotorLeft_PID.ki * MotorLeft_PID.integral
								 + MotorLeft_PID.kd * (MotorLeft_PID.error - MotorLeft_PID.lastError);
			
			// 输出限幅，限制电机控制量
			if(MotorLeft_PID.output > MotorLeft_PID.maxOutput) {
				MotorLeft_PID.output = MotorLeft_PID.maxOutput;
			}
			if(MotorLeft_PID.output < -MotorLeft_PID.maxOutput) {
				MotorLeft_PID.output = -MotorLeft_PID.maxOutput;
			}
			
			// 输出死区处理，小幅控制输出直接置零
			if(MotorLeft_PID.output > -PID_OUTPUT_DEADZONE && MotorLeft_PID.output < PID_OUTPUT_DEADZONE){
				MotorLeft_PID.output = 0;
			}
		}
   }
}

unsigned char Key_Read_BTN(void)
{
	unsigned char Key_Value;
	
	if(HAL_GPIO_ReadPin(KEY0_GPIO_Port,KEY0_Pin)==0) Key_Value=7;
	else if(HAL_GPIO_ReadPin(KEY1_GPIO_Port,KEY1_Pin)==0) Key_Value=6;
	else if(HAL_GPIO_ReadPin(KEY2_GPIO_Port,KEY2_Pin)==0) Key_Value=5;
	else Key_Value=0;
		
	return Key_Value;
}

void Key_Proc(void){
	if(Key_Slow_Down) return;
	Key_Slow_Down=1;
	
	Key_Value=Key_Read_BTN();
	Key_Down=Key_Value&(Key_Old^Key_Value);
	Key_Up=~Key_Value&(Key_Old^Key_Value);
	Key_Old=Key_Value;
	
	switch(Key_Down)
	{
		case 7:
			Key_Test0++;
			// 按键0：切换迷宫模式（开始/停止）
			if(maze_auto_mode == 0){
				// 进入迷宫自动模式
				Motion_Stop();
				Maze_Init();
				maze_finished = 0;
				maze_auto_mode = 1;
			}else{
				// 退出迷宫模式，停止运动并进入手动停止模式
				maze_auto_mode = 0;
				maze_finished = 0;
				Motion_Stop();
				Current_Action = 4;
				Action(4);
			}
		break;
		case 6:
			Key_Test1++;
			// 按键1：在非迷宫模式下，循环切换 action 的 5 种模式（0直行 1左转 2右转 3调头 4停止）
			if(maze_auto_mode == 0){
				Current_Action = (Current_Action + 1) % 5; // 0~4 轮询
				Action(Current_Action);
			}
		break;
		case 5:
			Key_Test2++;
			// 按键2：在非迷宫模式下调整直行速度，超过上限后回到下限
			if(maze_auto_mode == 0){
				if(Straight_Speed < SPEED_MIN){
					Straight_Speed = SPEED_MIN;
				}
				Straight_Speed += SPEED_STEP;
				if(Straight_Speed > SPEED_MAX){
					Straight_Speed = SPEED_MIN;
				}
				// 当前是直行模式时立即生效
				if(Current_Action == 0){
					Action(0);
				}
			}
		break;
	}
}

// 解析JY61P姿态帧，更新Roll、Pitch、Yaw
void JY61P_Proc(void){
	if(Uart_Receive_Flag2==0){
		return;
	}

	jy61p_Sum=(0x55+Uart_Receive_String2[0]+Uart_Receive_String2[1]+Uart_Receive_String2[2]+Uart_Receive_String2[3]+Uart_Receive_String2[4]+Uart_Receive_String2[5]+Uart_Receive_String2[6]+Uart_Receive_String2[7]+Uart_Receive_String2[8]);
	if(Uart_Receive_String2[9]==jy61p_Sum){
		if(Uart_Receive_String2[0]==0x53){
			jy61p_Roll=(int16_t)((Uart_Receive_String2[2]<<8)|(Uart_Receive_String2[1]));
			jy61p_Pitch=(int16_t)((Uart_Receive_String2[4]<<8)|(Uart_Receive_String2[3]));
			jy61p_Yaw=(int16_t)((Uart_Receive_String2[6]<<8)|(Uart_Receive_String2[5]));
			Roll=jy61p_Roll*180.0f/32768.0f;
			Pitch=jy61p_Pitch*180.0f/32768.0f;
			Yaw=jy61p_Yaw*180.0f/32768.0f;
		}
	}
	Uart_Receive_Flag2=0;
}

// 将速度环PID输出转换为整数电机控制量
void Motor_Proc(void){
	Set_LeftLun_Duty((int)MotorLeft_PID.output);// 将左轮PID输出送入PWM与方向控制函数
	Set_RightLun_Duty((int)MotorRight_PID.output);// 将右轮PID输出送入PWM与方向控制函数
}

// 设置右轮电机的方向和PWM占空比，Duty允许正负值
void Set_RightLun_Duty(int Duty){
	if(Duty>1000){
		Duty=1000;
	}
	else if(Duty<-1000){
		Duty=-1000;
	}
	if(Duty>=0){// 非负控制量对应正向输出
		HAL_GPIO_WritePin(AIN1_GPIO_Port,AIN1_Pin,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(AIN2_GPIO_Port,AIN2_Pin,GPIO_PIN_SET);
		__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,Duty);// 设置PWM比较值
	}
	else{// 负控制量对应反向输出
		HAL_GPIO_WritePin(AIN1_GPIO_Port,AIN1_Pin,GPIO_PIN_SET);
		HAL_GPIO_WritePin(AIN2_GPIO_Port,AIN2_Pin,GPIO_PIN_RESET);
		__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,-Duty);// 反向时使用控制量绝对值设置PWM比较值
	}
}

// 设置左轮电机的方向和PWM占空比，Duty允许正负值
void Set_LeftLun_Duty(int Duty){
	if(Duty>1000){
		Duty=1000;
	}
	else if(Duty<-1000){
		Duty=-1000;
	}
	if(Duty>=0){// 非负控制量对应正向输出
		HAL_GPIO_WritePin(BIN1_GPIO_Port,BIN1_Pin,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(BIN2_GPIO_Port,BIN2_Pin,GPIO_PIN_SET);
		__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_1,Duty);// 设置PWM比较值
	}
	else{// 负控制量对应反向输出
		HAL_GPIO_WritePin(BIN1_GPIO_Port,BIN1_Pin,GPIO_PIN_SET);
		HAL_GPIO_WritePin(BIN2_GPIO_Port,BIN2_Pin,GPIO_PIN_RESET);
		__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_1,-Duty);// 反向时使用控制量绝对值设置PWM比较值
	}
}

// 通过USART3发送字符串
void SendString3(uint8_t* UartString){
	HAL_UART_Transmit(&huart3, (uint8_t *)UartString, strlen((const char *)UartString), 1000);
}

// 发送位置、朝向、姿态角、红外状态和运行模式
void Send_Debug_Data(void){
	int len = 0;
	HAL_StatusTypeDef status;
	
	// 协议：s,X,Y,O,Angle,Front,Left,Right,Mode\r\n
	// X,Y: 坐标(0-7), O:朝向(0北1东2南3西), Angle:Yaw角(度), Front/Left/Right:传感器(0有墙1无墙), Mode:模式(0停止1迷宫)
	// 将Yaw角归一化到0-360度范围（0度=北，顺时针增加）
	float yaw_normalized = Yaw;
	while(yaw_normalized < 0.0f) yaw_normalized += 360.0f;
	while(yaw_normalized >= 360.0f) yaw_normalized -= 360.0f;
	
	len = snprintf((char*)uart3_send_buffer, sizeof(uart3_send_buffer),
		"s,%d,%d,%d,%.1f,%d,%d,%d,%d\r\n",
		(int)robot_x,           // X坐标
		(int)robot_y,           // Y坐标
		(int)robot_dir,         // O朝向(0北1东2南3西)
		yaw_normalized,        // Angle陀螺仪Yaw角(度)
		(int)CGQ_Data1,         // Front前传感器(0有墙1无墙)
		(int)CGQ_Data3,         // Left左传感器(0有墙1无墙)
		(int)CGQ_Data4,         // Right右传感器(0有墙1无墙)
		(int)maze_auto_mode     // Mode运行模式(0停止1迷宫模式)
	);
	
	// 确保数据完整发送，增加超时时间并检查发送状态
	if(len > 0 && len < sizeof(uart3_send_buffer)){
		// 等待上一次发送完成，避免数据被分割
		while(HAL_UART_GetState(&huart3) == HAL_UART_STATE_BUSY_TX){
			HAL_Delay(1);
		}
		// 一次性发送完整数据帧，超时时间增加到5000ms
		status = HAL_UART_Transmit(&huart3, uart3_send_buffer, (uint16_t)len, 5000);
		// 如果发送失败，可以在这里添加错误处理
		(void)status; // 避免未使用变量警告
	}
}

//==================== 迷宫与运动控制函数实现 ====================//
// 迷宫初始化：清空所有墙和距离，并设置外边界墙
void Maze_Init(void){
	for(uint8_t x = 0; x < MAZE_SIZE; x++){
		for(uint8_t y = 0; y < MAZE_SIZE; y++){
			maze[x][y].walls = 0;
			maze[x][y].distance = 0xFF;
			maze[x][y].visited = 0;
		}
	}
	// 设置外边界为墙
	for(uint8_t x = 0; x < MAZE_SIZE; x++){
		maze[x][0].walls |= (1 << DIR_SOUTH);                 // 最底行南侧为墙
		maze[x][MAZE_SIZE-1].walls |= (1 << DIR_NORTH);       // 最顶行为墙
	}
	for(uint8_t y = 0; y < MAZE_SIZE; y++){
		maze[0][y].walls |= (1 << DIR_WEST);                  // 最左列西侧为墙
		maze[MAZE_SIZE-1][y].walls |= (1 << DIR_EAST);        // 最右列东侧为墙
	}
	// 初始位置与朝向
	robot_x = 7;
	robot_y = 0;
	robot_dir = DIR_NORTH;
	maze[robot_x][robot_y].visited = 1;

	maze_finished = 0;
	motion_state = MOTION_IDLE;
	move_encoder_acc = 0;
}



// 根据当前朝向与红外传感器更新当前格子的墙信息
// 约定：CGQ_Data1=前，CGQ_Data3=左，CGQ_Data4=右（CGQ_Data2后方传感器不使用）
// 假定 0 = 有墙（检测到障碍），1 = 无墙


// 关键是：区分机器人自身的前后左右，与迷宫地图的东南西北。
void Maze_UpdateWalls(void){
	MazeCell *cell = &maze[robot_x][robot_y];
	
	// 程序首先根据 robot_dir 把相对方向转换成绝对方向。
	uint8_t dir_front = robot_dir;
	uint8_t dir_back  = (robot_dir + 2) & 0x03;
	uint8_t dir_left  = (robot_dir + 3) & 0x03;
	uint8_t dir_right = (robot_dir + 1) & 0x03;

	// 前方墙（使用CGQ_Data1）
	if(CGQ_Data1 == 0){
		cell->walls |= (1 << dir_front);
		int8_t nx = robot_x + dir_dx[dir_front];
		int8_t ny = robot_y + dir_dy[dir_front];
		if(nx >= 0 && nx < MAZE_SIZE && ny >= 0 && ny < MAZE_SIZE){
			maze[nx][ny].walls |= (1 << dir_back);
		}
	}
	// 左侧墙（使用CGQ_Data3）
	if(CGQ_Data3 == 0){
		cell->walls |= (1 << dir_left);
		int8_t nx = robot_x + dir_dx[dir_left];
		int8_t ny = robot_y + dir_dy[dir_left];
		if(nx >= 0 && nx < MAZE_SIZE && ny >= 0 && ny < MAZE_SIZE){
			maze[nx][ny].walls |= (1 << dir_right);
		}
	}
	// 右侧墙（使用CGQ_Data4）
	if(CGQ_Data4 == 0){
		cell->walls |= (1 << dir_right);
		int8_t nx = robot_x + dir_dx[dir_right];
		int8_t ny = robot_y + dir_dy[dir_right];
		if(nx >= 0 && nx < MAZE_SIZE && ny >= 0 && ny < MAZE_SIZE){
			maze[nx][ny].walls |= (1 << dir_left);
		}
	}
	// 注意：CGQ_Data2（后方传感器）不使用，因为后方信息对走迷宫无帮助
}


// 是否位于迷宫中心 2x2 区域
uint8_t Maze_IsInCenter(uint8_t x, uint8_t y){
	if(x >= MAZE_CENTER_MIN && x <= MAZE_CENTER_MAX &&
	   y >= MAZE_CENTER_MIN && y <= MAZE_CENTER_MAX){
		return 1;
	}
	return 0;
}

// 基于当前墙信息计算从每个格子到中心的距离（洪泛算法）
void Maze_ComputeDistances(void){
	// 初始化
	for(uint8_t x = 0; x < MAZE_SIZE; x++){
		for(uint8_t y = 0; y < MAZE_SIZE; y++){
			maze[x][y].distance = 0xFF;
		}
	}
	// 中心区域距离设为 0
	for(uint8_t x = MAZE_CENTER_MIN; x <= MAZE_CENTER_MAX; x++){
		for(uint8_t y = MAZE_CENTER_MIN; y <= MAZE_CENTER_MAX; y++){
			maze[x][y].distance = 0;
		}
	}

	// 迭代松弛，直到不再变化（由于 8x8 迷宫规模小，这里简单暴力实现即可）
	uint8_t changed;
	do{
		changed = 0;
		for(uint8_t x = 0; x < MAZE_SIZE; x++){
			for(uint8_t y = 0; y < MAZE_SIZE; y++){
				uint8_t cur_d = maze[x][y].distance;
				for(uint8_t d = 0; d < 4; d++){
					// 有墙则不可通行
					if(maze[x][y].walls & (1 << d)){
						continue;
					}
					int8_t nx = x + dir_dx[d];
					int8_t ny = y + dir_dy[d];
					if(nx < 0 || nx >= MAZE_SIZE || ny < 0 || ny >= MAZE_SIZE){
						continue;
					}
					uint8_t nd = maze[nx][ny].distance;
					if(nd + 1 < cur_d){
						maze[x][y].distance = nd + 1;
						cur_d = nd + 1;
						changed = 1;
					}
				}
			}
		}
	}while(changed);
}

// 从当前格子出发，选择距离值最小的相邻格子方向
// 增强版：实时根据传感器数据判断缺口，自动选择转向方向
// 返回：0 = 前进；1 = 左转；2 = 右转；3 = 掉头；0xFF = 无可走方向
uint8_t Maze_ChooseNextMove(void){
	// 获取当前朝向下的各方向
	uint8_t dir_front = robot_dir;
	uint8_t dir_left  = (robot_dir + 3) & 0x03;
	uint8_t dir_right = (robot_dir + 1) & 0x03;
	uint8_t dir_back  = (robot_dir + 2) & 0x03;
	
	// 根据传感器实时数据判断各方向是否有缺口（传感器值为1表示有缺口）
	// CGQ_Data1=前，CGQ_Data3=左，CGQ_Data4=右
	uint8_t sensor_gap_front = (CGQ_Data1 == 1) ? 1 : 0;  // 前方传感器检测到缺口
	uint8_t sensor_gap_left  = (CGQ_Data3 == 1) ? 1 : 0;  // 左侧传感器检测到缺口
	uint8_t sensor_gap_right = (CGQ_Data4 == 1) ? 1 : 0;  // 右侧传感器检测到缺口
	
	// 综合判断：传感器检测到缺口 OR 地图中未标记为墙
	uint8_t can_go_front = sensor_gap_front || !(maze[robot_x][robot_y].walls & (1 << dir_front));
	uint8_t can_go_left  = sensor_gap_left  || !(maze[robot_x][robot_y].walls & (1 << dir_left));
	uint8_t can_go_right = sensor_gap_right || !(maze[robot_x][robot_y].walls & (1 << dir_right));
	uint8_t can_go_back  = !(maze[robot_x][robot_y].walls & (1 << dir_back));  // 后方不检测传感器
	
	uint8_t bestDir = 0xFF;
	uint8_t bestDist = 0xFF;
	
	// 第一步：找出所有可达方向及其距离值
	uint8_t valid_dirs[4] = {0xFF, 0xFF, 0xFF, 0xFF};
	uint8_t valid_dists[4] = {0xFF, 0xFF, 0xFF, 0xFF};
	uint8_t sensor_flags[4] = {0, 0, 0, 0};  // 标记哪些方向是传感器检测到的缺口
	uint8_t neighbor_visited[4] = {0, 0, 0, 0}; // 标记相邻格是否已访问
	uint8_t valid_count = 0;
	
	// 检查前方
	if(can_go_front){
		int8_t nx = robot_x + dir_dx[dir_front];
		int8_t ny = robot_y + dir_dy[dir_front];
		if(nx >= 0 && nx < MAZE_SIZE && ny >= 0 && ny < MAZE_SIZE){
			valid_dirs[valid_count] = dir_front;
			valid_dists[valid_count] = maze[nx][ny].distance;
			sensor_flags[valid_count] = sensor_gap_front;  // 标记为传感器检测到的缺口
			neighbor_visited[valid_count] = maze[nx][ny].visited;
			valid_count++;
		}
	}
	
	// 检查左侧
	if(can_go_left){
		int8_t nx = robot_x + dir_dx[dir_left];
		int8_t ny = robot_y + dir_dy[dir_left];
		if(nx >= 0 && nx < MAZE_SIZE && ny >= 0 && ny < MAZE_SIZE){
			valid_dirs[valid_count] = dir_left;
			valid_dists[valid_count] = maze[nx][ny].distance;
			sensor_flags[valid_count] = sensor_gap_left;
			neighbor_visited[valid_count] = maze[nx][ny].visited;
			valid_count++;
		}
	}
	
	// 检查右侧
	if(can_go_right){
		int8_t nx = robot_x + dir_dx[dir_right];
		int8_t ny = robot_y + dir_dy[dir_right];
		if(nx >= 0 && nx < MAZE_SIZE && ny >= 0 && ny < MAZE_SIZE){
			valid_dirs[valid_count] = dir_right;
			valid_dists[valid_count] = maze[nx][ny].distance;
			sensor_flags[valid_count] = sensor_gap_right;
			neighbor_visited[valid_count] = maze[nx][ny].visited;
			valid_count++;
		}
	}
	
	// 检查后方（仅在必要时）
	if(can_go_back){
		int8_t nx = robot_x + dir_dx[dir_back];
		int8_t ny = robot_y + dir_dy[dir_back];
		if(nx >= 0 && nx < MAZE_SIZE && ny >= 0 && ny < MAZE_SIZE){
			valid_dirs[valid_count] = dir_back;
			valid_dists[valid_count] = maze[nx][ny].distance;
			sensor_flags[valid_count] = 0;  // 后方不检测传感器
			neighbor_visited[valid_count] = maze[nx][ny].visited;
			valid_count++;
		}
	}
	
	// 死胡同检测：如果前方、左、右都是墙（地图中标记为墙，且传感器未检测到缺口），只有后方可以走
	// 注意：如果传感器检测到缺口，即使地图中标记为墙，也不判定为死胡同（可能是新发现的路径）
	uint8_t front_wall_in_map = (maze[robot_x][robot_y].walls & (1 << dir_front)) != 0;
	uint8_t left_wall_in_map = (maze[robot_x][robot_y].walls & (1 << dir_left)) != 0;
	uint8_t right_wall_in_map = (maze[robot_x][robot_y].walls & (1 << dir_right)) != 0;
	uint8_t back_wall_in_map = (maze[robot_x][robot_y].walls & (1 << dir_back)) != 0;
	
	// 死胡同条件：前方、左、右在地图中都标记为墙，且传感器也未检测到缺口，只有后方可以走
	if(front_wall_in_map && !sensor_gap_front &&
	   left_wall_in_map && !sensor_gap_left &&
	   right_wall_in_map && !sensor_gap_right &&
	   !back_wall_in_map && valid_count > 0){
		// 确认后方方向在有效方向列表中
		for(uint8_t i = 0; i < valid_count; i++){
			if(valid_dirs[i] == dir_back){
				// 死胡同：强制选择掉头返回
				return 3;  // 直接返回掉头动作
			}
		}
	}
	
	if(valid_count == 0){
		return 0xFF;  // 完全封闭，无路可走
	}
	
	// 第二步：找出最短距离（优先未访问节点）。若存在未访问的相邻格，仅在未访问集合中找最短；否则在全部里找。
	uint8_t have_unvisited = 0;
	for(uint8_t i = 0; i < valid_count; i++){
		if(!neighbor_visited[i]){
			have_unvisited = 1;
			break;
		}
	}
	for(uint8_t i = 0; i < valid_count; i++){
		if(have_unvisited && neighbor_visited[i]){
			continue; // 本轮只在未访问格中寻找
		}
		if(valid_dists[i] < bestDist){
			bestDist = valid_dists[i];
		}
	}
	
	// 第三步：在达到最短距离的方向中，优先选择策略
	// 优先级：1. 未访问过的格子（记忆已走过路线，优先新路）
	//         2. 传感器检测到的缺口方向（新发现的路径）
	//         3. 直行方向（保持方向）
	//         4. 左转（左手法则）
	//         5. 右转
	//         6. 掉头
	
	uint8_t best_score = 0;
	uint8_t best_candidate = valid_dirs[0];
	
	for(uint8_t i = 0; i < valid_count; i++){
		if(valid_dists[i] != bestDist){
			continue;  // 只考虑最短距离的方向
		}
		
		uint8_t d = valid_dirs[i];
		uint8_t score = 0;
		
		int8_t nx = robot_x + dir_dx[d];
		int8_t ny = robot_y + dir_dy[d];
		
		// 加分项1：未访问过的格子（优先尝试新路径）
		if(!maze[nx][ny].visited){
			score += 25;
		}
		
		// 加分项2：传感器检测到的缺口（新发现的路径）
		if(sensor_flags[i]){
			score += 15;
		}
		
		// 加分项3：直行方向（保持方向）
		if(d == dir_front){
			score += 5;
		}
		
		// 加分项4：左手法则（优先左转）
		if(d == dir_left){
			score += 2;
		}
		
		// 加分项5：右转
		if(d == dir_right){
			score += 1;
		}
		
		// 选择得分最高的方向
		if(score > best_score){
			best_score = score;
			best_candidate = d;
		}
	}
	
	bestDir = best_candidate;
	
	// 第四步：将绝对方向转换为相对动作
	if(bestDir == robot_dir){
		return 0; // 前
	}else if(bestDir == dir_left){
		return 1; // 左
	}else if(bestDir == dir_right){
		return 2; // 右
	}else{
		return 3; // 掉头
	}
}

// 启动前进一个单元格
void Motion_StartForwardOneCell(void){
	if(motion_state != MOTION_IDLE){
		return;
	}
	Straight_Speed = 45;          // 迷宫直行基础速度，可根据实际调试调整（范围20-120）
	move_encoder_acc = 0;
	motion_start_tick = ms_Tick;
	Action(0);                    // 设置为直行动作（会锁定当前偏航为直行基准）
	motion_state = MOTION_MOVE_ONE_CELL;
}

// 启动左转 90°
void Motion_StartTurnLeft(void){
	if(motion_state != MOTION_IDLE){
		return;
	}
	Action(1);
	motion_start_tick = ms_Tick;
	motion_state = MOTION_TURNING;
	// 注意：robot_dir 在转向完成后再更新，见 Maze_Controller 中的转向完成判断
}

// 启动右转 90°
void Motion_StartTurnRight(void){
	if(motion_state != MOTION_IDLE){
		return;
	}
	Action(2);
	motion_start_tick = ms_Tick;
	motion_state = MOTION_TURNING;
	// 注意：robot_dir 在转向完成后再更新，见 Maze_Controller 中的转向完成判断
}

// 启动掉头 180°
void Motion_StartTurnBack(void){
	if(motion_state != MOTION_IDLE){
		return;
	}
	Action(3);
	motion_start_tick = ms_Tick;
	motion_state = MOTION_TURNING;
	// 注意：robot_dir 在转向完成后再更新，见 Maze_Controller 中的转向完成判断
}

// 停止当前运动
void Motion_Stop(void){
	Set_RightLun_Duty(0);
	Set_LeftLun_Duty(0);
	MotorRight_Target = 0;
	MotorLeft_Target = 0;
	Straight_Speed = 0;
	Reset_PID();
	Angle_Target_Valid = 0;
	motion_state = MOTION_IDLE;
}

// 迷宫上层控制：在主循环中周期性调用
void Maze_Controller(void){
	static uint32_t last_plan_tick = 0;
	static uint8_t turn_stable_count = 0;

	// 简单节流：每 50ms 规划一次，避免过于频繁
	if(ms_Tick - last_plan_tick < 50){
		return;
	}
	last_plan_tick = ms_Tick;

	if(maze_finished){
		return;
	}

	// 如果正在转弯，则根据角度误差判断是否结束
	if(motion_state == MOTION_TURNING){
		float err = NormalizeAngle(Yaw_Target - Yaw);
		// 转向完成判断：角度误差小于阈值，且持续一段时间（避免抖动）
		if(err < TURN_ANGLE_EPS && err > -TURN_ANGLE_EPS){
			if(turn_stable_count < 3){
				turn_stable_count++;
			}
			if(turn_stable_count >= 3){
			// 转向完成，更新机器人朝向
			// 根据当前动作判断转向方向
			if(Current_Action == 1){
				// 左转90°
				robot_dir = (Direction)((robot_dir + 3) & 0x03);
			}else if(Current_Action == 2){
				// 右转90°
				robot_dir = (Direction)((robot_dir + 1) & 0x03);
			}else if(Current_Action == 3){
				// 掉头180°
				robot_dir = (Direction)((robot_dir + 2) & 0x03);
			}
				turn_stable_count = 0;
			Motion_Stop();
			}
		}
		// 超时保护：如果转向超过3秒仍未完成，强制停止并更新方向
		else if((ms_Tick - motion_start_tick) > 3000){
			if(Current_Action == 1){
				robot_dir = (Direction)((robot_dir + 3) & 0x03);
			}else if(Current_Action == 2){
				robot_dir = (Direction)((robot_dir + 1) & 0x03);
			}else if(Current_Action == 3){
				robot_dir = (Direction)((robot_dir + 2) & 0x03);
			}
			turn_stable_count = 0;
			Motion_Stop();
		}
		else{
			turn_stable_count = 0;
		}
		return;
	}

	// 非转向状态，清零稳定计数
	turn_stable_count = 0;

	// 正在前进格子时，由定时中断检测距离是否到位，这里不做处理
	if(motion_state == MOTION_MOVE_ONE_CELL){
		return;
	}
	
	
// ***************************************************************************** //
// ***************************************************************************** //
	
	
	// 运动空闲：检查是否到达中心
	// Maze_IsInCenter() 用来判断电脑鼠是否已经进入中央2×2格区域。到达就停止，结束本次自动探索。
	if(Maze_IsInCenter((uint8_t)robot_x, (uint8_t)robot_y)) 
	{
		maze_finished = 1;
		Motion_Stop();
		return;
	}

	// 根据前、左、右红外传感器的信息，记录当前位置附近的墙壁
	Maze_UpdateWalls();
	// 规划下一步路线
	// Maze_ComputeDistances()：计算地图中各格到终点的洪泛距离。
	// Maze_ChooseNextMove()：根据距离、墙体、是否访问过等信息，决定下一步往哪个方向走。
	Maze_ComputeDistances();
	uint8_t move = Maze_ChooseNextMove();
	
	if(move == 0xFF){
		// 完全封闭，无路可走（所有方向都是墙）
		// 尝试重新计算距离，可能是地图信息有误
		// 如果仍然无路可走，则停止
		maze_finished = 1;
		Motion_Stop();
		return;
	}

	// 通过 switch(move)，选择直行、左转、右转或掉头。
	switch(move){
		case 0:
			Motion_StartForwardOneCell();
		break;
		case 1:
			Motion_StartTurnLeft();
		break;
		case 2:
			Motion_StartTurnRight();
		break;
		case 3:
			Motion_StartTurnBack();
		break;
		default:
		break;
	}
}

// 目标变化或动作切换时重置PID状态
void Reset_PID(void){
	MotorRight_PID.integral = 0;
	MotorRight_PID.error = 0;
	MotorRight_PID.lastError = 0;
	MotorRight_PID.output = 0;
	
	MotorLeft_PID.integral = 0;
	MotorLeft_PID.error = 0;
	MotorLeft_PID.lastError = 0;
	MotorLeft_PID.output = 0;
	Reset_Angle_PID();

	if(MotorRight_Target==0 && MotorLeft_Target==0){
		Angle_Target_Valid = 0;
	}else{
		if(!Angle_Target_Valid){
			Yaw_Target = Yaw;
		}
		Angle_Target_Valid = 1;
	}
}

// 初始化PID参数和内部状态
void PID_Init(PID *pid, float p, float i, float d, float maxI, float maxOut)
{
    pid->kp = p;
    pid->ki = i;
    pid->kd = d;
    pid->maxIntegral = maxI;
    pid->maxOutput = maxOut;
    pid->error = 0;
    pid->lastError = 0;
    pid->integral = 0;
    pid->output = 0;
}

void Angle_Control_Update(void){
	MotorRight_Target_Adjusted = MotorRight_Target;
	MotorLeft_Target_Adjusted = MotorLeft_Target;

	if(!Angle_Target_Valid){
		Yaw_PID_Error = 0.0f;
		Yaw_PID_Output = 0.0f;
		turn_output_filtered = 0.0f;
		return;
	}

	// 转弯动作使用转弯PID，其余动作使用直行PID
	uint8_t turning = (Current_Action == 1 || Current_Action == 2 || Current_Action == 3);
	PID *pid = turning ? &Turn_PID : &Angle_PID;

	pid->lastError = pid->error;
	pid->error = NormalizeAngle(Yaw_Target - Yaw);

	if(pid->error < 5.0f && pid->error > -5.0f){
		pid->integral += pid->error;
	}else{
		pid->integral *= 0.9f;
	}

	if(pid->integral > pid->maxIntegral){
		pid->integral = pid->maxIntegral;
	}
	if(pid->integral < -pid->maxIntegral){
		pid->integral = -pid->maxIntegral;
	}

	pid->output = pid->kp * pid->error
					 + pid->ki * pid->integral
					 + pid->kd * (pid->error - pid->lastError);

	if(pid->output > pid->maxOutput){
		pid->output = pid->maxOutput;
	}
	if(pid->output < -pid->maxOutput){
		pid->output = -pid->maxOutput;
	}

	// 转弯/直行输出平滑，避免振荡
	float yaw_output = pid->output;
	float slew = turning ? TURN_SLEW_STEP : STRAIGHT_SLEW_STEP;
	float delta = yaw_output - turn_output_filtered;
	if(delta > slew) delta = slew;
	if(delta < -slew) delta = -slew;
	turn_output_filtered += delta;
	yaw_output = turn_output_filtered;

	// 转弯收敛减速：误差越小输出越小，降低过冲
	if(turning){
		float abs_err = pid->error;
		if(abs_err < 0) abs_err = -abs_err;
		float scale = abs_err / 18.0f; // 误差小于18度开始线性减小
		if(scale > 1.0f) scale = 1.0f;
		if(scale < 0.15f) scale = 0.15f; // 保证最低驱动力
		yaw_output *= scale;
	}

	// 微小区间死区，减小直行抖动；转弯时不启用死区以避免残余角误差
	if(!turning){
		if(yaw_output < 1.5f && yaw_output > -1.5f){
			yaw_output = 0.0f;
			turn_output_filtered = 0.0f;
		}
	}

	Yaw_PID_Error = pid->error;
	Yaw_PID_Output = yaw_output;

	MotorRight_Target_Adjusted += (int32_t)yaw_output;
	MotorLeft_Target_Adjusted  -= (int32_t)yaw_output;

	// 直行时在速度环层面做左右平衡，抑制机械误差导致的偏航
	if(!turning && Current_Action == 0){
		int diff = EncoderRight - EncoderLeft; // >0 表示右轮更快
		float bal = diff * STRAIGHT_BALANCE_KP;
		if(bal > STRAIGHT_BALANCE_MAX) bal = STRAIGHT_BALANCE_MAX;
		if(bal < -STRAIGHT_BALANCE_MAX) bal = -STRAIGHT_BALANCE_MAX;
		int32_t bal_i = (int32_t)bal;
		MotorRight_Target_Adjusted -= bal_i;
		MotorLeft_Target_Adjusted  += bal_i;
	}
}

void Reset_Angle_PID(void){
	Angle_PID.integral = 0;
	Angle_PID.error = 0;
	Angle_PID.lastError = 0;
	Angle_PID.output = 0;
	Turn_PID.integral = 0;
	Turn_PID.error = 0;
	Turn_PID.lastError = 0;
	Turn_PID.output = 0;
	turn_output_filtered = 0.0f;
	Yaw_PID_Error = 0.0f;
	Yaw_PID_Output = 0.0f;
	MotorRight_Target_Adjusted = MotorRight_Target;
	MotorLeft_Target_Adjusted = MotorLeft_Target;
}

float NormalizeAngle(float angle){
	while(angle > 180.0f){
		angle -= 360.0f;
	}
	while(angle < -180.0f){
		angle += 360.0f;
	}
	return angle;
}

void Action(uint8_t num){
	float baseYaw = Yaw; // 记录当前偏航角，作为动作计算基准
	uint8_t previous_action = Current_Action;
 
 	switch(num){
 		case 0:
 			// 直行：设置双轮统一目标速度
 			MotorRight_Target = Straight_Speed;
 			MotorLeft_Target = Straight_Speed;
			// 直行微调：补偿机械左右差异，防止左偏
			MotorLeft_Target  += Straight_Trim;
			MotorRight_Target -= Straight_Trim;
 		break;
 		case 1:
 		case 2:
 		case 3:
 			// 转弯/调头：基础速度为0，完全依赖角度PID输出驱动
 			MotorRight_Target = 0;
 			MotorLeft_Target = 0;
		break;
		case 4:
			// 停止：目标速度为 0，不保持角度
			MotorRight_Target = 0;
			MotorLeft_Target = 0;
			Straight_Speed = 0;
 		break;
 		default:
 			return; // 非法动作直接忽略
 	}
 
 	Reset_PID(); // 清空速度环与角度环状态，防止上一个动作的积分干扰
 
 	switch(num){
 		case 0:
 			if(Straight_Speed > 0){
 				if(previous_action != 0){
 					Yaw_Target = baseYaw; // 首次进入直行动作时锁定当前偏航
 				}
 				Angle_Target_Valid = 1;
 			}else{
 				Angle_Target_Valid = 0; // 速度为0时无需保持角度
 			}
 		break;
 		case 1:
			Yaw_Target = NormalizeAngle(baseYaw + 90.0f -5.0f); // 左转90°
 			Angle_Target_Valid = 1;
 		break;
 		case 2:
			Yaw_Target = NormalizeAngle(baseYaw - 90.0f -6.0f); // 右转90°
 			Angle_Target_Valid = 1;
 		break;
 		case 3:
 			Yaw_Target = NormalizeAngle(baseYaw + 180.0f); // 调头
 			Angle_Target_Valid = 1;
		break;
		case 4:
			// 停止动作不需要角度控制
			Angle_Target_Valid = 0;
 		break;
 		default:
 		break;
 	}
 
 	// 重置后的调节目标需要立即同步到修正速度中
 	MotorRight_Target_Adjusted = MotorRight_Target;
 	MotorLeft_Target_Adjusted = MotorLeft_Target;
 	Current_Action = num;
}

//Num=1 2 3 4 前后左右红外
uint8_t CGQ_Read(uint8_t Num){
	uint8_t CGQ_Data;
	CGQ_Data=0;
	if(Num==1){
		CGQ_Data=HAL_GPIO_ReadPin(CGQ_Forward_GPIO_Port,CGQ_Forward_Pin);
	}
	else if(Num==2){
		CGQ_Data=HAL_GPIO_ReadPin(CGQ_Backward_GPIO_Port,CGQ_Backward_Pin);
	}
	else if(Num==3){
		CGQ_Data=HAL_GPIO_ReadPin(CGQ_Left_GPIO_Port,CGQ_Left_Pin);
	}
	else if(Num==4){
		CGQ_Data=HAL_GPIO_ReadPin(CGQ_Right_GPIO_Port,CGQ_Right_Pin);
	}
	return CGQ_Data;
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
	HAL_UART_Receive_IT(&huart2, (uint8_t *)&uartdata2,1); 
	if(Uart_State2==0){
		if(uartdata2==0x55 && Uart_Receive_Flag2 ==0){
			Uart_State2=1;
			Uart_Index2=0;
		}
	}
	else if(Uart_State2==1){
		if(Uart_Index2==10){
			Uart_State2=2;
		}
		else{
			Uart_Receive_String2[Uart_Index2]=uartdata2;
			Uart_Index2++;
		}
	}
	else if(Uart_State2==2){
		Uart_State2=0;
		Uart_Receive_String2[Uart_Index2]='\0';
		Uart_Receive_Flag2=1;
	}
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
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
