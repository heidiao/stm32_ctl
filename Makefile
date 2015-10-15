CC = gcc
AR = ar
CFLAGS = -O2 -Wall -fPIC
LFLAGS = $(CFLAGS)
LIB = stm32_ctrl
TARGET = stm32_ctrl
OBJS = main.o
LIB_OBJS = byte_stuffing.o serial_protocol.o util.o stepper_motor.o

all: $(TARGET)

.o: .c
	$(CC) $(CFLAGS) $< -o $@

$(TARGET): $(OBJS) lib$(LIB).so lib$(LIB).a
#	$(CC) $(LFLAGS) $(OBJS) -L. -l$(LIB) -o $@		# dynamic link
	$(CC) $(LFLAGS) $(OBJS) lib$(LIB).a -o $@		# static link

lib$(LIB).so: $(LIB_OBJS)
	$(CC) -shared $(LFLAGS) $^ -o $@

lib$(LIB).a: $(LIB_OBJS)
	$(AR) rcs lib$(LIB).a $^

clean:
	@rm -f *.o *~ $(TARGET) lib$(LIB).so lib$(LIB).a

dep:
	$(CC) -MM *.c > makefile.dep
	
include makefile.dep
