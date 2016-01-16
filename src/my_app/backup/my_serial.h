#ifndef MY_SERIAL_H
#define MY_SERIAL_H


#include <systemlib/err.h>


int open_port(void);
int get_fd_serial(void);
int close_port(void);

int configure_port(void);

void write_port(char*, int);
void flush_port(void);
void read_port(char*, int);

#endif/* MYAPP_H */
