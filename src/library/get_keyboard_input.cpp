#include "get_keyboard_input.h"

namespace abidat {

namespace robot {

namespace control {

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

} //end namespace control

} //end namespace robot

} //end namespace abidat