#include	<stdio.h>
#include	<stdlib.h>
#include	<assert.h>
#include	<limits.h>
#include	"stepper_motor.h"

// pseudo head
#define	SPD_PSEUDO_HORI						0.72	// steps per degree (R1 ON, STEP 0)
#define	DIFF_PSEUDO_HORI					24.5	// gear ratio
#define	SPD_PSEUDO_VERT						0.0288	// steps per degree (R1 ON, STEP 8)
#define	DIFF_PSEUDO_VERT					1		// gear ratio

// wibo head
#define	SPD_WIBO_HORI						0.72	// steps per degree (R1 ON, STEP 0)
#define	DIFF_WIBO_HORI						24.5	// gear ratio
#define	SPD_WIBO_VERT						0.0288	// steps per degree (R1 ON, STEP 8)
#define	DIFF_WIBO_VERT						1		// gear ratio

// limitation for Orientation Motor
#define	MIN_PERIOD							100		// us
#define	MIN_TR								20		// ms/KHz

motor_t pseudo_hori = { ID_HORI, FREQ, SPD_PSEUDO_HORI / DIFF_PSEUDO_HORI };
motor_t pseudo_vert = { ID_VERT, FREQ, SPD_PSEUDO_VERT / DIFF_PSEUDO_VERT };
motor_t wibo_hori = { ID_HORI, FREQ, SPD_WIBO_HORI / DIFF_WIBO_HORI };
motor_t wibo_vert = { ID_VERT, FREQ, SPD_WIBO_VERT / DIFF_WIBO_VERT };

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
	uint16_t	step_accel = cmd_motor_drive_step_accel(cmd);
	uint16_t	real_period_end = cmd->period_start - step_accel * cmd->period_accel;
	uint32_t	time_accel = (cmd->period_start + real_period_end ) * step_accel / 2 / 1000;	// in ms

	return time_accel*2 + (cmd->steps - step_accel*2) * (uint32_t) real_period_end / 1000;
}

uint16_t motor_deg2step( motor_t *motor, int degree ) {
	uint32_t	step = degree / motor->spd;
	if( step > USHRT_MAX ) {
		fprintf( stderr, "%s: step overflow detected! Angle/degree is too high.\n", __func__ );
		abort();
	}
	return step;
}

double motor_step2deg( motor_t *motor, uint16_t step ) {
	return step * motor->spd;
}

uint16_t motor_rpm2period( motor_t *motor, double rpm ) {
	uint32_t	period = motor->freq / motor_deg2step( motor, rpm*360/60 );
	if( period > USHRT_MAX ) {
		fprintf( stderr, "%s: period overflow detected! RPM is too slow.\n", __func__ );
		abort();
	}
	return period;
}

cmd_motor_drive_t* motor_cmd_motor_drive_verify( motor_t *motor, cmd_motor_drive_t *cmd ) {
	uint16_t	step_accel = cmd_motor_drive_step_accel(cmd);
	uint16_t	real_period_end = cmd->period_start - step_accel * cmd->period_accel;
	uint32_t	time_accel = (cmd->period_start + real_period_end) * step_accel / 2 / 1000;	// in ms
	uint32_t	time_total = time_accel*2 + (cmd->steps - step_accel*2) * real_period_end / 1000;
	double		Tr = time_accel / ( 1000.0/real_period_end - 1000.0/cmd->period_start );
	double		rpm = motor_step2deg(motor,cmd->steps)/360.0/time_total*1000*60;

	fprintf( stderr, "id=%d, dir=%d, period_start=%d, period_end=%d, period_accel=%d, step_accel=%d, time_accel=%dms, time=%dms, rpm=%f, Tr=%f\n", 
			motor->id, cmd->direction, cmd->period_start, cmd->period_end, cmd->period_accel, step_accel, time_accel, time_total, rpm, Tr );
//	assert( cmd->period_end >= MIN_PERIOD );
	assert( cmd->id < ID_MAX );
	assert( cmd->direction==CW || cmd->direction==NA || cmd->direction==CCW );
	assert( cmd->cmd==CMD_MOTOR_DRIVE );
	if( step_accel==0 )
		fprintf( stderr, "Warning: step_accel==0\n" );
	if( cmd->period_end<MIN_PERIOD )
		fprintf( stderr, "Warning: period_end > MIN_PERIOD\n" );
	if( Tr < MIN_TR )
		fprintf( stderr, "Warning: Tr < %d\n", MIN_TR );
	return cmd;
}

cmd_motor_drive_t* motor_gen_cmd_motor_drive( motor_t *motor, cmd_motor_drive_t *cmd, dir_t direction, uint16_t steps, uint16_t period_start, uint16_t period_end, uint16_t period_accel ) {
	*cmd = (cmd_motor_drive_t) { DEV_ADDR, SENDER_UART, CMD_MOTOR_DRIVE, motor->id, direction, steps, period_start, period_end, period_accel };
	assert( period_start >= period_end );
	motor_cmd_motor_drive_verify( motor, cmd );
	return cmd;
}

cmd_motor_drive_t* motor_gen_cmd_motor_drive_rpm( motor_t *motor, cmd_motor_drive_t *cmd, dir_t direction, int degree, double rpm_start, double rpm_end, int accel_time ) {
	uint16_t	steps = motor_deg2step( motor, degree );
	uint16_t	period_start = motor_rpm2period( motor, rpm_start );
	uint16_t	period_end = motor_rpm2period( motor, rpm_end );
	uint16_t	period_accel = ( period_start-period_end )*( (uint32_t) period_start+period_end ) / (2*accel_time/1000.0*motor->freq);
	assert( rpm_start <= rpm_end );
	return motor_gen_cmd_motor_drive( motor, cmd, direction, steps, period_start, period_end, period_accel );
}
