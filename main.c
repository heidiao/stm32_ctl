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
#define	DEVICE								"/dev/ttyACM0"
#define	BUF_SIZE							PATH_MAX

int	uart_fd;
struct termios oldtio_uart, oldtio_stdin;
int	quit;

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

	newtio_uart.c_cc[VMIN] = 1;			// block mode
	newtio_uart.c_cc[VTIME] = 1;

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
	serial_send_command( uart_fd, &CMD_STOP_DATA_STREAMING, sizeof(CMD_STOP_DATA_STREAMING) );
	tcflush( uart_fd, TCIOFLUSH );
	
	tcsetattr( uart_fd, TCSANOW, &oldtio_uart );
	tcsetattr( uart_fd, TCSANOW, &oldtio_stdin );
	close(uart_fd);
}

int main( int argc, char *argv[] ) {
	uint8_t			buf[BUF_SIZE+1];
	int				nread;
	fd_set			fds;
	struct timeval	timeout;
	
	// open uart device
	uart_fd = open( DEVICE, O_RDWR | O_NOCTTY | O_SYNC );
	if( uart_fd<0 ) {
		perror( DEVICE );
		exit( -1 );
	}
	uart_init( uart_fd );
		
	// flush
	printf( "flush" );
	serial_send_command( uart_fd, &CMD_STOP_DATA_STREAMING, sizeof(CMD_STOP_DATA_STREAMING) );
	serial_send_command( uart_fd, &CMD_STOP_DATA_STREAMING, sizeof(CMD_STOP_DATA_STREAMING) );
	tcflush( uart_fd, TCIOFLUSH );
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
	serial_send_command( uart_fd, &CMD_PING, sizeof(CMD_PING) );
	nread = serial_read( uart_fd, buf, BUF_SIZE );
	dump( stdout, buf, nread );
	if( buf[2]!=(CMD_ID_PING|CMD_ID_REPLY_ADD) ) {
		printf( "ping failed\n" );
		exit( -1 );
	}

	// check presentation string
	printf( "check presentation string...\n" );
	serial_send_command( uart_fd, &CMD_READ_PRES_STRING, sizeof(CMD_READ_PRES_STRING) );
	nread = serial_read( uart_fd, buf, BUF_SIZE );
	dump( stdout, buf, nread );
	if( strncmp( (char*)buf+3, PRESENTATION_STRING, sizeof(PRESENTATION_STRING) )==0 ) {
		printf( "prenstation string not match\n" );
		exit( -1 );
	}

	// check mode
	printf( "check mode...\n" );
	serial_send_command( uart_fd, &CMD_CHECK_MODE_SUPPORT, sizeof(CMD_CHECK_MODE_SUPPORT) );
	nread = serial_read( uart_fd, buf, BUF_SIZE );
	dump( stdout, buf, nread );

	// start data streaming
	printf( "start data streaming...\n" );
	serial_send_command( uart_fd, &CMD_START_DATA_STREAMING, sizeof(CMD_START_DATA_STREAMING) );
	nread = serial_read( uart_fd, buf, 5 );
	dump( stdout, buf, nread );

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
			nread = serial_read( uart_fd, buf, BUF_SIZE );
			if( nread>0 ) {
				dump( stdout, buf, nread );
				if( buf[0]==SENDER_UART && buf[1]==DEV_ADDR ) {
					switch( buf[2]-CMD_ID_REPLY_ADD ) {
						case CMD_ID_MOTOR_GET_STATE:
							printf( "steps left = %d\n", buf[3] + (buf[4]<<8) );
							break;
						default:
							printf( "response from cmd %02x\n", buf[2]-CMD_ID_REPLY_ADD );
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
			int ch = tolower( getchar() );
			
			putchar( '\n' );
			if( ch=='a' ) {
				serial_send_command( uart_fd, &gen_cmd_motor_drive( 0, CW, deg2step(3600), 900, MIN_PERIOD, 4 ), sizeof(cmd_motor_drive_t) );
			} else if( ch=='b' ) {
				serial_send_command( uart_fd, &gen_cmd_motor_drive( 0, CW, deg2step(3600), 800, MIN_PERIOD, 3 ), sizeof(cmd_motor_drive_t) );
			} else if( ch=='c' ) {
				serial_send_command( uart_fd, &gen_cmd_motor_drive( 0, CW, deg2step(3600), 700, MIN_PERIOD, 2 ), sizeof(cmd_motor_drive_t) );
			} else if( ch=='d' ) {
				serial_send_command( uart_fd, &gen_cmd_motor_drive( 0, CW, deg2step(3600), 600, MIN_PERIOD, 1 ), sizeof(cmd_motor_drive_t) );
			} else if( ch=='x' ) {
				// 360 degree at 60 rpm
				serial_send_command( uart_fd, &gen_cmd_motor_drive( 0, NA, deg2step(360), FREQ/deg2step(360), FREQ/deg2step(360), 0 ), sizeof(cmd_motor_drive_t) );
			} else if( ch=='z' ) {
				serial_send_command( uart_fd, &gen_cmd_motor_drive( 0, CCW, deg2step(3600), 600, MIN_PERIOD, 1 ), sizeof(cmd_motor_drive_t) );
			} else if( ch=='t' ) {
				serial_send_command( uart_fd, &(cmd_motor_test_t) { DEV_ADDR, SENDER_UART, CMD_ID_MOTOR_TEST, 0, 0, 0, 0, 0 }, sizeof(cmd_motor_test_t) );
				usleep(1000);
				serial_send_command( uart_fd, &(cmd_motor_test_t) { DEV_ADDR, SENDER_UART, CMD_ID_MOTOR_TEST, 0, 0, 0, 1, 0 }, sizeof(cmd_motor_test_t) );
				usleep(5000);
				serial_send_command( uart_fd, &(cmd_motor_test_t) { DEV_ADDR, SENDER_UART, CMD_ID_MOTOR_TEST, 0, 0, 0, 0, 1 }, sizeof(cmd_motor_test_t) );
			} else if( ch=='g' ) {
				serial_send_command( uart_fd, &(cmd_motor_get_state_t) { DEV_ADDR, SENDER_UART, CMD_ID_MOTOR_GET_STATE, 0 }, sizeof(cmd_motor_get_state_t) );
			} else if( ch=='q' )
				break;
		}
	}
	
	return 0;
}
