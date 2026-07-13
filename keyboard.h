#ifndef KEYBOARD_H 
#define KEYBOARD_H 
#include <ncurses.h>
#include "dcmotor.h" 

void keyboard_init();
int keyboard_run();
void keyboard_cleanup();

#endif

