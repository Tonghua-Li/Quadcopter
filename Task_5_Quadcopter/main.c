/*
 *
 *		File	: main.c
 *		Author	: Furkan Cayci
 *
 */

#include "string.h"
#include "ch.h"
#include "hal.h"

#include "mpu6050.h"
#include "chprintf.h"
#include "math.h"

/*
 * PWM definitions for the motors. 500 might be increased to 512 to make it 2^9
 */
#define PWM_CLOCK_FREQUENCY 50000000
#define PWM_PERIOD_IN_TICKS 500
#define PWM_FREQUENCY (PWM_CLOCK_FREQUENCY/PWM_PERIOD_IN_TICKS)

#define MOTOR_EAST			0
#define MOTOR_WEST			1
#define MOTOR_NORTH			2
#define MOTOR_SOUTH			3

#define RAD_TO_DEG 57.2957786

#define DT 				0.002		// Expressed in seconds (for the PID)
#define COMP			0.93		// Variable for complementary filter

#define KP	1		// KP
#define	KI	(1 * DT)	// KI * DT = 1 * 0.002
#define KD	0		// KD / DT = 0 / 0.002




/*
 * Uart
 */
#define 	DebugUART	SD4
#define 	XBeeUART	SD3


static SerialConfig UartConfig = {
   115200,									/* 115200 baud rate */
   0,
   USART_CR2_STOP1_BITS | USART_CR2_LINEN,
   0
};

static void initUARTs(void){
	sdStart(&DebugUART, &UartConfig);

	// sdStart(&SD2, NULL); /* for default configuration */
	palSetPadMode(GPIOA, 0, PAL_MODE_ALTERNATE(8));
	palSetPadMode(GPIOA, 1, PAL_MODE_ALTERNATE(8));
}

//----------------------------------------------------------------------------------------

enum MotorType{
	Motor_EAST=0,
	Motor_WEST=1,
	Motor_NORTH=2,
	Motor_SOUTH=3
};

/*
 * Motor
 * PWM definitions for the motors. 500 might be increased to 512 to make it 2^9
 */
#define PWM_CLOCK_FREQUENCY 1000000
#define PWM_PERIOD_IN_TICKS 10000
#define PWM_FREQUENCY (PWM_CLOCK_FREQUENCY/PWM_PERIOD_IN_TICKS)
static PWMConfig tim2PWMConfig = {
		PWM_CLOCK_FREQUENCY,								/* 10kHz PWM clock frequency  */
		PWM_PERIOD_IN_TICKS,								/* PWM period (in ticks) 1S (1/10kHz=0.1mS 0.1ms*10000 ticks=1S) */
		NULL,									/* No Callback */
		{
			{PWM_OUTPUT_ACTIVE_HIGH, NULL},		/* Enable Channel 0 */
			{PWM_OUTPUT_DISABLED, NULL},
			{PWM_OUTPUT_ACTIVE_HIGH, NULL},
			{PWM_OUTPUT_ACTIVE_HIGH, NULL}
		},
		0  									/* HW dependent part.*/
};
static PWMConfig tim5PWMConfig = {
		PWM_CLOCK_FREQUENCY,								/* 10kHz PWM clock frequency  */
		PWM_PERIOD_IN_TICKS,								/* PWM period (in ticks) 1S (1/10kHz=0.1mS 0.1ms*10000 ticks=1S) */
		NULL,									/* No Callback */
		{
			{PWM_OUTPUT_DISABLED, NULL},		/* Enable Channel 0 */
			{PWM_OUTPUT_DISABLED, NULL},
			{PWM_OUTPUT_ACTIVE_HIGH, NULL},
			{PWM_OUTPUT_DISABLED, NULL}
		},
		0  									/* HW dependent part.*/
};

/*
 * Motor pwm initialization.
 * Motor 1 ->PA2
 * Motor 2 ->PA5
 * Motor 3 ->PB10
 * Motor 4 ->PB11
 */
static void initPWMs(void){
	palSetPadMode(GPIOA, GPIOA_PIN2, PAL_MODE_ALTERNATE(2)); 	// TIM5_CH3
	palSetPadMode(GPIOA, GPIOA_PIN5, PAL_MODE_ALTERNATE(1)); 	// TIM2_CH1
	palSetPadMode(GPIOB, GPIOB_PIN10, PAL_MODE_ALTERNATE(1)); 	// TIM2_CH3
	palSetPadMode(GPIOB, GPIOB_PIN11, PAL_MODE_ALTERNATE(1));	// TIM2_CH4

	pwmStart(&PWMD2, &tim2PWMConfig);
	pwmStart(&PWMD5, &tim5PWMConfig);
}

static void setMotorSpeed(enum MotorType motor, uint16_t speed){
	switch(motor){
	case Motor_EAST:
		pwmEnableChannel(&PWMD5, 2, speed);
		break;
	case Motor_WEST:
		pwmEnableChannel(&PWMD2, 0, speed);
		break;
	case Motor_NORTH:
		pwmEnableChannel(&PWMD2, 2, speed);
		break;
	case Motor_SOUTH:
		pwmEnableChannel(&PWMD2, 3, speed);
		break;
	default:break;
	}
}


//----------------------------------------------------------------------------------------
/* Configure I2C for sensors */
static const I2CConfig i2cfg1 = {
    OPMODE_I2C,
    400000,
    FAST_DUTY_CYCLE_2
};

int16_t XaccelRaw, YaccelRaw, ZaccelRaw;
int16_t XgyroRaw, YgyroRaw, ZgyroRaw;

double XaccelAngle, YaccelAngle;
double XgyroRate, YgyroRate;
double XcompAngle, YcompAngle;

static bool_t initSensors(void){
	bool_t sts = 0;

	/* Disable these for discovery board,
	 * don't think it will be problem on the quad tho */
	//palSetPadMode(GPIOB, 6, PAL_MODE_ALTERNATE(0));
	palSetPadMode(GPIOB, 9, PAL_MODE_ALTERNATE(0));

	i2cStart(&I2CD1, &i2cfg1);
	palSetPadMode(GPIOB, 6, PAL_MODE_ALTERNATE(4) | PAL_STM32_OTYPE_OPENDRAIN);
	palSetPadMode(GPIOB, 7, PAL_MODE_ALTERNATE(4) | PAL_STM32_OTYPE_OPENDRAIN);

	chThdSleepMilliseconds(100);

	/* MPU6050, AD0 is connected to VCC */
	MPU6050(MPU6050_ADDRESS_AD0_LOW);

	/* Test connection */
	sts = MPUtestConnection();
	if (!sts) return FALSE;

	MPUreset();
	MPUresetSensors();
	chThdSleepMilliseconds(100);
	MPUinitialize();
	chThdSleepMilliseconds(100);

	MPUgetMotion6(&XaccelRaw, &YaccelRaw, &ZaccelRaw, &XgyroRaw, &YgyroRaw, &ZgyroRaw);

	XaccelAngle = (atan2(YaccelRaw,ZaccelRaw)+M_PI)*RAD_TO_DEG;
	YaccelAngle = (atan2(XaccelRaw,ZaccelRaw)+M_PI)*RAD_TO_DEG;

	//XgyroRate = XaccelAngle;
	//YgyroRate = YaccelAngle;
	//XcompAngle = XaccelAngle;
	//YcompAngle = YaccelAngle;

	return TRUE;
}

/* Configure Heartbeat */
static WORKING_AREA(wahbeat, 128);

static msg_t thBlinker(void *arg){
	(void)arg;
	chRegSetThreadName("blinker");
	while (TRUE){
    	palTogglePad(GPIOD, GPIOD_LED3); /* Orange */
		chThdSleepMilliseconds(500);
	}
	return 0;
}



int main(void) {

	halInit();
	chSysInit();

	/* Create the heartbeat thread */
	chThdCreateStatic(wahbeat, sizeof(wahbeat), NORMALPRIO, thBlinker, NULL);

	initUARTs();
	initSensors();

	initPWMs();

	uint16_t speed=0;

	while (TRUE){
		speed+=200;
		setMotorSpeed(Motor_EAST,speed);
		if(speed>10000) speed=0;

		MPUgetMotion6(&XaccelRaw, &YaccelRaw, &ZaccelRaw, &XgyroRaw, &YgyroRaw, &ZgyroRaw);
		XaccelAngle = (atan2(YaccelRaw,ZaccelRaw)+M_PI)*RAD_TO_DEG;
		YaccelAngle = (atan2(XaccelRaw,ZaccelRaw)+M_PI)*RAD_TO_DEG;
		chprintf((BaseSequentialStream *)&DebugUART, "AX: %5d\tAY: %5d\t,AZ: %5d\t,GX: %5d\t,GY: %5d\t,GZ: %5d\tXA: %5f\t,YA: %5f\t\r\n",XaccelRaw,YaccelRaw,ZaccelRaw,XgyroRaw,YgyroRaw,ZgyroRaw,XaccelAngle,YaccelAngle);

		chThdSleepMilliseconds(200);

	}
}
