#pragma once
// \ TODO MACH NE DOKU!!!
#include <iostream>
#include <ncurses.h>	// \Todo multible keyboard key imput possible? Do research!
#include <thread>
#include <functional>
#include <atomic>

class GetKeyboardInput
{
public:
	GetKeyboardInput(std::function<void(const int)> callback)
	{
		callback_ = callback;
	}
	~GetKeyboardInput()
	{
		stop();
	}

   	/**
	 * \brief Starts a new thread with the member function getKey after setting running_ true to catch inputs
	 *        through the keyboard.
 	 */ 
	void start()
	{
		running_ = true;	
		reading_thread_ = std::thread(&GetKeyboardInput::getKey, this);
	}

	/**
	 * \brief If running_ is true it will be set to false and the thread with getKey will be terminated.
 	 */ 
	void stop()
	{	
		// if background thread isn't running just return
		if (!running_) {
			return;
		}

		// stop background thread and join it
		running_ = false;	
		reading_thread_.join();	
	}

	/**
	 * \brief Checking if running_ is true.
	 * 
	 * \return true if the background thread is running.
 	 */ 
	inline bool isRunning() const noexcept { return running_; }

private:
	/**
	 * \brief Function to catch the keyboard input running in a separete 
 	 */ 
	void getKey();

	std::thread reading_thread_; //> background thread to get input keys
	std::function<void(const int)> callback_; //> callback function that is called by received key input
	std::atomic<bool> running_{false}; //> flag that indicates if the background capture thread is running
};
