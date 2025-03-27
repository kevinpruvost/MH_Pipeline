#if defined(__linux__) || defined(__APPLE__)
#ifdef __linux__
#include <termio.h>
#else
#include <termios.h>
#endif
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>

#define gets_s gets

int getch(void);

int kbhit(void);
#else
#include <conio.h>
#endif