#include	<stdio.h>
#include	<fcntl.h>
#include	<sys/types.h>
#include	<sys/stat.h>
#include	<sys/select.h>
#include	<termios.h>
#include	<stdlib.h>
#include	<linux/limits.h>
#include	<unistd.h>
#include	<signal.h>
#include	<strings.h>
#include	<string.h>
#include	<ctype.h>
#include	"def.h"
#include	"serial_protocol.h"
#include	"stepper_motor.h"
#include	"util.h"

#define BAUDRATE							B921600
#define	DEFAULT_UART_DEVICE					"/dev/ttyUSB0"
#define	BUF_SIZE							PATH_MAX

#define	MIN_TEST_PERIOD						400

motor_t	*hori = &pseudo_hori;
motor_t	*vert = &pseudo_vert;

#define SENSORS_ENABLED						(SENSOR_ACCELEROMETER|SENSOR_GYROSCOPE|SENSOR_MAGNETIC)

cmd_short_t					cmd_ping					= { DEV_ADDR, SENDER_UART, CMD_PING };
cmd_short_t					cmd_read_pres_string		= { DEV_ADDR, SENDER_UART, CMD_READ_PRES_STRING };
cmd_short_t					cmd_check_mode_support		= { DEV_ADDR, SENDER_UART, CMD_CHECK_MODE_SUPPORT };
cmd_short_t					cmd_stop_data_streaming		= { DEV_ADDR, SENDER_UART, CMD_STOP_DATA_STREAMING };
cmd_start_data_streaming_t	cmd_start_data_streaming	= { DEV_ADDR, SENDER_UART, CMD_START_DATA_STREAMING, SENSORS_ENABLED, 100 };
cmd_motor_stop_t			cmd_motor_stop_hori_immed	= { DEV_ADDR, SENDER_UART, CMD_MOTOR_STOP, ID_HORI, 0, 0 };
cmd_motor_stop_t			cmd_motor_stop_vert_immed	= { DEV_ADDR, SENDER_UART, CMD_MOTOR_STOP, ID_VERT, 0, 0 };
cmd_motor_stop_t			cmd_motor_stop_hori			= { DEV_ADDR, SENDER_UART, CMD_MOTOR_STOP, ID_HORI, 600, 2 };
cmd_motor_stop_t			cmd_motor_stop_vert			= { DEV_ADDR, SENDER_UART, CMD_MOTOR_STOP, ID_VERT, 000, 2 };
cmd_short_t					cmd_lsm6ds0_init			= { DEV_ADDR, SENDER_UART, CMD_LSM6DS0_INIT };
cmd_short_t					cmd_lis3mdl_init			= { DEV_ADDR, SENDER_UART, CMD_LIS3MDL_INIT };

int	uart_fd;
struct termios oldtio_uart, oldtio_stdin;

void stdin_nonblock(void) {
	struct termios tio;
	
	tcgetattr( STDIN_FILENO, &oldtio_stdin );
	tcgetattr( STDIN_FILENO, &tio );
	tio.c_lflag &= ~ICANON;
	tcsetattr( STDIN_FILENO, TCSANOW, &tio );
}

void uart_init( int uart_fd ) {
	struct termios	newtio_uart;
	
	tcgetattr( uart_fd, &oldtio_uart );
	bzero( &newtio_uart, sizeof(newtio_uart) );
	tcgetattr( uart_fd, &newtio_uart );
	
	cfsetospeed( &newtio_uart, BAUDRATE );
	cfsetispeed( &newtio_uart, BAUDRATE );
	newtio_uart.c_cflag &= ~(CSIZE | PARENB | PARODD | CSTOPB);
	newtio_uart.c_cflag |= CS8 | CLOCAL | CREAD;
	newtio_uart.c_oflag = 0;
	newtio_uart.c_lflag = 0;
	newtio_uart.c_iflag &= ~(IXON | IXOFF | IXANY | ICRNL | INLCR);

	newtio_uart.c_cc[VMIN] = 1;			// block read mode until n chars received
	newtio_uart.c_cc[VTIME] = 1;		// inter-character timer in 0.1 secs

	tcflush( uart_fd, TCIOFLUSH );
	if(	tcsetattr( uart_fd, TCSAFLUSH, &newtio_uart )<0 )
		perror( "tcsetattr" );
}

void ctrl_break( int dummy ) {
	exit( -1 );
}

void restore( void ) {
	fprintf( stderr, "clean up...\n" );
	tcflush( uart_fd, TCIFLUSH );
	serial_send_command( uart_fd, &cmd_stop_data_streaming, sizeof(cmd_stop_data_streaming) );
	usleep(10000);
	tcflush( uart_fd, TCIOFLUSH );
	
	tcsetattr( uart_fd, TCSANOW, &oldtio_uart );
	tcsetattr( uart_fd, TCSANOW, &oldtio_stdin );
	close(uart_fd);
}

int main( int argc, char *argv[] ) {
	char			*uart_device;
	uint8_t			buf[BUF_SIZE+1];
	int				nread;
	fd_set			fds;
	struct timeval	timeout;

	if( argc==2 )
		uart_device = argv[1];
	else
		uart_device = (char*) DEFAULT_UART_DEVICE;

	// open uart device
	uart_fd = open( uart_device, O_RDWR | O_NOCTTY );
	if( uart_fd<0 ) {
		perror( uart_device );
		exit( -1 );
	}
	uart_init( uart_fd );

	// flush
	printf( "flush" );
	tcflush( uart_fd, TCIOFLUSH );
	serial_send_command( uart_fd, &cmd_stop_data_streaming, sizeof(cmd_stop_data_streaming) );
	serial_send_command( uart_fd, &cmd_stop_data_streaming, sizeof(cmd_stop_data_streaming) );
	serial_send_command( uart_fd, &cmd_motor_stop_hori_immed, sizeof(cmd_motor_stop_t) );
	serial_send_command( uart_fd, &cmd_motor_stop_vert_immed, sizeof(cmd_motor_stop_t) );
//	usleep(100000);
	while(1) {
		int	retval;
		timeout.tv_sec = 0;
		timeout.tv_usec = 100000;
		FD_ZERO( &fds );
		FD_SET( uart_fd, &fds );
		putchar( '.' );
		retval = select( uart_fd+1, &fds, NULL, NULL, &timeout );
		if( FD_ISSET( uart_fd, &fds ) ) {
			if( read( uart_fd, buf, BUF_SIZE )==0 )
				break;
		} else if( retval==0 )
			break;
	}
	putchar( '\n' );

	// ping device
	printf( "ping...\n" );
	serial_send_command( uart_fd, &cmd_ping, sizeof(cmd_ping) );
	nread = serial_read_message( uart_fd, buf, BUF_SIZE );
	dump( stdout, buf, nread );
	if( buf[2]!=(CMD_PING|CMD_REPLY_ADD) ) {
		printf( "ping failed\n" );
		exit( -1 );
	}

	// check presentation string
	printf( "check presentation string...\n" );
	serial_send_command( uart_fd, &cmd_read_pres_string, sizeof(cmd_read_pres_string) );
	nread = serial_read_message( uart_fd, buf, BUF_SIZE );
	dump( stdout, buf, nread );
	if( strncmp( (char*)buf+3, PRESENTATION_STRING, sizeof(PRESENTATION_STRING) )==0 ) {
		printf( "prenstation string not match\n" );
		exit( -1 );
	}

	// check mode
	printf( "check mode...\n" );
	serial_send_command( uart_fd, &cmd_check_mode_support, sizeof(cmd_check_mode_support) );
	nread = serial_read_message( uart_fd, buf, BUF_SIZE );
	dump( stdout, buf, nread );

	// init sensors
	printf( "init sensors...\n" );
	if( SENSORS_ENABLED ) {
		if( SENSORS_ENABLED & (SENSOR_ACCELEROMETER|SENSOR_GYROSCOPE) ) {
			serial_send_command( uart_fd, &cmd_lsm6ds0_init, sizeof(cmd_short_t) );
			nread = serial_read_message( uart_fd, buf, BUF_SIZE );
			dump( stdout, buf, nread );
		}
		if( SENSORS_ENABLED & SENSOR_MAGNETIC ) {
			serial_send_command( uart_fd, &cmd_lis3mdl_init, sizeof(cmd_short_t) );
			nread = serial_read_message( uart_fd, buf, BUF_SIZE );
			dump( stdout, buf, nread );
		}
	}

	// start data streaming
	printf( "start data streaming...\n" );
	serial_send_command( uart_fd, &cmd_start_data_streaming, sizeof(cmd_start_data_streaming) );
	nread = serial_read_message( uart_fd, buf, 5 );
	dump( stdout, buf, nread );

	printf( "init sensors...\n" );
	if( SENSORS_ENABLED ) {
		if( SENSORS_ENABLED & (SENSOR_ACCELEROMETER|SENSOR_GYROSCOPE) ) {
			serial_send_command( uart_fd, &cmd_lsm6ds0_init, sizeof(cmd_short_t) );
			nread = serial_read_message( uart_fd, buf, BUF_SIZE );
			dump( stdout, buf, nread );
		}
		if( SENSORS_ENABLED & SENSOR_MAGNETIC ) {
			serial_send_command( uart_fd, &cmd_lis3mdl_init, sizeof(cmd_short_t) );
			nread = serial_read_message( uart_fd, buf, BUF_SIZE );
			dump( stdout, buf, nread );
		}
	}

	// register clean up callback
	signal( SIGINT, ctrl_break );
	stdin_nonblock();
	atexit( restore );

	// main loop
	while(1) {
		int	retval;

		FD_ZERO( &fds );
		FD_SET( uart_fd, &fds );
		FD_SET( STDIN_FILENO, &fds );

		retval = select( uart_fd+1, &fds, NULL, NULL, NULL );
		if( retval<0 ) {
			perror( "select" );
		} 

		if( FD_ISSET( uart_fd, &fds ) ) {
			nread = serial_read_message( uart_fd, buf, BUF_SIZE );
			if( nread>0 ) {
				dump( stdout, buf, nread );
				if( buf[0]==SENDER_UART && buf[1]==DEV_ADDR ) {
					switch( buf[2] ) {
						case CMD_MOTOR_DRIVE|CMD_REPLY_ADD:
							printf( "motor driver %s\n", buf[3]==0 ? "failed" : "successful" );
							break;
						case CMD_MOTOR_GET_STATE|CMD_REPLY_ADD:
							printf( "steps left = %d\n", deserialize( buf+3, 2 ) );
							break;
						case CMD_START_DATA_STREAMING:
							if( SENSORS_ENABLED& SENSOR_ACCELEROMETER )
								printf( "accelerometer: x=%2d, y=%2d, z=%2d\n", deserialize(buf+15,4), deserialize(buf+19,4), deserialize(buf+23,4) );
							if( SENSORS_ENABLED & SENSOR_GYROSCOPE )
								printf( "gyroscope: x=%2d, y=%2d, z=%2d\n", deserialize(buf+27,4), deserialize(buf+31,4), deserialize(buf+35,4) );
							if( SENSORS_ENABLED & SENSOR_MAGNETIC )
								printf( "magnetic: x=%2d, y=%2d, z=%2d\n", deserialize(buf+39,4), deserialize(buf+43,4), deserialize(buf+47,4) );
							break;
						case CMD_MOTOR_STOP|CMD_REPLY_ADD:
						case CMD_MOTOR_STOP: {
							uint16_t real_steps = deserialize( buf+4, 2 );
							if( real_steps )
								printf( "receive motor stop async event, id=%d, real_steps = %d\n", buf[3], real_steps );
							break;
						}
						case CMD_MOTOR_REPORT_SENSOR|CMD_REPLY_ADD:
							printf( "receive motor sensor async event, id = %d, sensor = %d, steps = %d\n", buf[3], buf[4], deserialize( buf+5, 2 ) );
							break;
						default:
							printf( "response from cmd %02x\n", buf[2]-CMD_REPLY_ADD );
							break;
					}
				}
			}  else if( nread==-1 ) {
				// read()<0 error
			} else if( nread==-2 ) {
				// USB disconnect
				exit(-1);
			} else if( nread==-3 ) {
				// resync or checksum error
			}
		}
		if( FD_ISSET( STDIN_FILENO, &fds ) ) {
			cmd_motor_drive_t	cmd;
#if 0
			int	ch = tolower( getchar() );
#else
			uint8_t ch;

			if( !read( STDIN_FILENO, &ch, sizeof(uint8_t) ) )
				continue;
			ch = tolower( ch );
#endif
			if( !isalnum(ch) )
				continue;

			putchar( '\n' );
			if( ch=='1' ) {
				serial_send_command( uart_fd, motor_gen_cmd_motor_drive_rpm( hori, &cmd, CW, 90, 1, 12.5, 100 ), sizeof(cmd_motor_drive_t) );
			} else if( ch=='2' ) {
				serial_send_command( uart_fd, motor_gen_cmd_motor_drive_rpm( hori, &cmd, CCW, 90, 2, 15, 100 ), sizeof(cmd_motor_drive_t) );
			} else if( ch=='3' ) {
				serial_send_command( uart_fd, motor_gen_cmd_motor_drive_rpm( hori, &cmd, CW, 90, 3, 20, 100 ), sizeof(cmd_motor_drive_t) );
			} else if( ch=='4' ) {
				serial_send_command( uart_fd, motor_gen_cmd_motor_drive_rpm( hori, &cmd, CCW, 90, 4, 25, 100 ), sizeof(cmd_motor_drive_t) );
			} else if( ch=='a' ) {
				serial_send_command( uart_fd, motor_gen_cmd_motor_drive_rpm( vert, &cmd, CW, 30, 1, 12.5/2, 100 ), sizeof(cmd_motor_drive_t) );
			} else if( ch=='b' ) {
				serial_send_command( uart_fd, motor_gen_cmd_motor_drive_rpm( vert, &cmd, CCW, 30, 2, 15/2, 100 ), sizeof(cmd_motor_drive_t) );
			} else if( ch=='c' ) {
				serial_send_command( uart_fd, motor_gen_cmd_motor_drive_rpm( vert, &cmd, CW, 30, 3, 20/2, 100 ), sizeof(cmd_motor_drive_t) );
			} else if( ch=='d' ) {
				serial_send_command( uart_fd, motor_gen_cmd_motor_drive_rpm( vert, &cmd, CCW, 30, 4, 25/2, 100 ), sizeof(cmd_motor_drive_t) );
			} else if( ch=='x' ) {
				serial_send_command( uart_fd, motor_gen_cmd_motor_drive_rpm( hori, &cmd, CW, 90, 1, 1, 0 ), sizeof(cmd_motor_drive_t) );
			} else if( ch=='z' ) {
				serial_send_command( uart_fd, motor_gen_cmd_motor_drive( hori, &cmd, CCW, motor_deg2step(hori,90), 1000, MIN_TEST_PERIOD, 4 ), sizeof(cmd_motor_drive_t) );
			} else if( ch=='s' ) {
				serial_send_command( uart_fd, &cmd_motor_stop_hori, sizeof(cmd_motor_stop_t) );
				serial_send_command( uart_fd, &cmd_motor_stop_vert, sizeof(cmd_motor_stop_t) );
			} else if( ch=='t' ) {
				cmd_motor_test_t	cmd;
				cmd = (cmd_motor_test_t) { DEV_ADDR, SENDER_UART, CMD_MOTOR_TEST, ID_HORI, 0, 0 };
				serial_send_command( uart_fd, &cmd, sizeof(cmd_motor_test_t) );
				cmd = (cmd_motor_test_t) { DEV_ADDR, SENDER_UART, CMD_MOTOR_TEST, ID_HORI, 0, 1 };
				serial_send_command( uart_fd, &cmd, sizeof(cmd_motor_test_t) );
			} else if( ch=='g' ) {
				cmd_motor_get_state_t	cmd;
				cmd = (cmd_motor_get_state_t) { DEV_ADDR, SENDER_UART, CMD_MOTOR_GET_STATE, ID_HORI };
				serial_send_command( uart_fd, &cmd, sizeof(cmd_motor_get_state_t) );
			} else if( ch=='q' ) {
				break;
			} else {
				printf( "unknown command\n" );
			}
		}
	}

	return 0;
}
