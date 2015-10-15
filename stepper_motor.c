#include	<stdio.h>
#include	"stepper_motor.h"

cmd_short					CMD_PING					= { DEV_ADDR, SENDER_UART, CMD_ID_PING };
cmd_short					CMD_READ_PRES_STRING		= { DEV_ADDR, SENDER_UART, CMD_ID_READ_PRES_STRING };
cmd_short					CMD_CHECK_MODE_SUPPORT		= { DEV_ADDR, SENDER_UART, CMD_ID_CHECK_MODE_SUPPORT };
cmd_short					CMD_STOP_DATA_STREAMING		= { DEV_ADDR, SENDER_UART, CMD_ID_STOP_DATA_STREAMING };
cmd_start_data_streaming	CMD_START_DATA_STREAMING	= { DEV_ADDR, SENDER_UART, CMD_ID_START_DATA_STREAMING, 0x0070, 100 };
cmd_motor_drive_t			CMD_MOTOR_DRIVE				= { DEV_ADDR, SENDER_UART, CMD_ID_MOTOR_DRIVE, 0, CW, 360/SPD, 2000, 200, 1 };
cmd_motor_get_state_t		CMD_MOTOR_GET_STATE			= { DEV_ADDR, SENDER_UART, CMD_ID_MOTOR_GET_STATE, 0 };
cmd_motor_test_t			CMD_MOTOR_TEST				= { DEV_ADDR, SENDER_UART, CMD_ID_MOTOR_TEST, 0, 0, 0, 0, 0 };

// calculate steps to accelerate
uint16_t cmd_motor_drive_step_accel( cmd_motor_drive_t *cmd ) {
	uint32_t	step_accel;
	
	if( cmd->period_accel==0 )
		return 0;
	else
		step_accel = (cmd->period_start - cmd->period_end) / cmd->period_accel;
	if( step_accel > cmd->steps/2 )
		step_accel = cmd->steps/2;
	return step_accel;
}

// in million seconds
uint32_t cmd_motor_drive_time( cmd_motor_drive_t *cmd ) {
	uint32_t	step_accel = cmd_motor_drive_step_accel(cmd);
	uint32_t	real_period_end = cmd->period_start - step_accel * cmd->period_accel;
	uint32_t	time_accel = (cmd->period_start + real_period_end ) * step_accel / 2 / 1000;	// in ms

	return time_accel*2 + (cmd->steps - step_accel*2) * real_period_end / 1000;
}

cmd_motor_drive_t* cmd_motor_drive_verify( cmd_motor_drive_t *cmd ) {
	uint32_t	step_accel = cmd_motor_drive_step_accel(cmd);
	uint32_t	real_period_end = cmd->period_start - step_accel * cmd->period_accel;
	uint32_t	time_accel = (cmd->period_start + real_period_end) * step_accel / 2 / 1000;	// in ms
	uint32_t	time_total = time_accel*2 + (cmd->steps - step_accel*2) * real_period_end / 1000;
	float		Tr = time_accel / ( 1000.0/real_period_end - 1000.0/cmd->period_start );
	
	fprintf( stderr, "step_accel=%d, time_accel=%dms, time=%dms, rpm=%f, Tr=%f\n", step_accel, time_accel, time_total, cmd->steps*SPD/360.0/time_total*1000*60, Tr );
//	assert( cmd->period_end >= MIN_PERIOD );
	if( cmd->period_end<MIN_PERIOD )
		fprintf( stderr, "Warning: period_end > MIN_PERIOD\n" );
	if( Tr < MIN_TR )
		fprintf( stderr, "Warning: Tr < %d\n", MIN_TR );
	return cmd;
}
