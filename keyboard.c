#include "keyboard.h"
#include <ncurses.h>

void keyboard_init() 
{
	initscr();
	cbreak();
	noecho();
	keypad(stdscr, TRUE);
	nodelay(stdscr, TRUE);
}

void keyboard_cleanup() 
{
	endwin();
}

int keyboard_run() 
{
	int key = getch();

	switch(key)
	{
		case 32:
			motor_stop();
			endwin();
			return 0;
	}

	return 1;
}

