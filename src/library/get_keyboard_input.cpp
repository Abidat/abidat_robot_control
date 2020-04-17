#include "get_keyboard_input.h"

void GetKeyboardInput::getKey()
{
	initscr();
	keypad(stdscr, true);
	noecho();
	while (running_)
	{
		const int keyboard_input = getch();
		std::cout << keyboard_input;
		callback_(keyboard_input);
	}
	endwin();
}
