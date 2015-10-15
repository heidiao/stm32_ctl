#ifndef	__MOTOR_H
#define	__MOTOR_H

#include	"def.h"

#define	SPD									0.72	// steps per degree
#define	deg2step(d)							( (d) / SPD )
#define	FREQ								1000000	// 1Mhz

// limitation for Orientation Motor
#define	MIN_PERIOD							100		// us
#define	MIN_TR								20		// ms/KHz

#define	DEV_ADDR							50

#define	SENDER_UART							0x01
#define SENDER_USB							0x02
#define	SENDER_SPI							0x03

#define	CMD_ID_PING							0x01
#define CMD_ID_READ_PRES_STRING				0x02
#define	CMD_ID_NACK							0x03
#define CMD_ID_CHECK_MODE_SUPPORT			0x04
#define CMD_ID_START_DATA_STREAMING			0x08
#define CMD_ID_STOP_DATA_STREAMING			0x09

#define	CMD_ID_MOTOR_DRIVE					0x10
#define	CMD_ID_MOTOR_GET_STATE				0x11
#define	CMD_ID_MOTOR_TEST					0x1f

#define	CMD_ID_REPLY_ADD					0x80

#define	PRESENTATION_STRING					"MEMS shield demo"

typedef enum {
	CCW = -1,	// counter clockwise
	NA = 0,		// not avaliable (last state)
	CW = 1		// clockwise
} dir_t;

#pragma pack(push)
#pragma pack(1)
#define	DEFINE_CMD_START	typedef struct __attribute__((__packed__)) {
#define	DEFINE_CMD_END(x)	uint8_t	checksum;	\
							} x
DEFINE_CMD_START
	uint8_t		dev_addr;
	uint8_t		sender;
	uint8_t		cmd;
DEFINE_CMD_END(cmd_short);

DEFINE_CMD_START
	uint8_t		dev_addr;
	uint8_t		sender;
	uint8_t		cmd;
	uint32_t	sensors_enabled;
	uint32_t	data_tx_period;	
DEFINE_CMD_END(cmd_start_data_streaming);

DEFINE_CMD_START
	uint8_t		dev_addr;
	uint8_t		sender;
	uint8_t		cmd;
	uint8_t		id;
	dir_t		direction:8;
	uint16_t	steps;
	uint16_t	period_start;		// in us
	uint16_t	period_end;		// in us
	uint16_t	period_accel;		// in us
DEFINE_CMD_END(cmd_motor_drive_t);

DEFINE_CMD_START
	uint8_t		dev_addr;
	uint8_t		sender;
	uint8_t		cmd;
	uint8_t		id;
	uint8_t		pls;
	uint8_t		dir;
	uint8_t		awo;
	uint8_t		cs;
DEFINE_CMD_END(cmd_motor_test_t);

DEFINE_CMD_START
	uint8_t		dev_addr;
	uint8_t		sender;
	uint8_t		cmd;
	uint8_t		id;
DEFINE_CMD_END(cmd_motor_get_state_t);
#pragma pack(pop)

extern cmd_short				CMD_PING;
extern cmd_short				CMD_READ_PRES_STRING;
extern cmd_short				CMD_CHECK_MODE_SUPPORT;
extern cmd_short				CMD_STOP_DATA_STREAMING;
extern cmd_start_data_streaming	CMD_START_DATA_STREAMING;
extern cmd_motor_drive_t		CMD_MOTOR_DRIVE;
extern cmd_motor_get_state_t	CMD_MOTOR_GET_STATE;
extern cmd_motor_test_t			CMD_MOTOR_TEST;

uint32_t cmd_motor_drive_time( cmd_motor_drive_t *cmd );
cmd_motor_drive_t* cmd_motor_drive_verify( cmd_motor_drive_t *cmd );

// generate and verify compond literal (on stack)
#define gen_cmd_motor_drive( id, direction, steps, period_start, period_end, period_accel )	\
	*cmd_motor_drive_verify( &(cmd_motor_drive_t) { DEV_ADDR, SENDER_UART, CMD_ID_MOTOR_DRIVE, id, direction, steps, period_start, period_end, period_accel } )

#endif
