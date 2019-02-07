Single Author info:

bwmcdon2 Brayden W McDonald

Group info:

angodse Anupam N Godse


# Bluetooth-controllable self-balancing EV3 Mindstorm robot
EV3 Mindstorm robot programmed with EV3RT

Description: We built a segway-style 2-wheeled vehicle that balances itself using a gyro sensor, and is capable of recieving and executing commands from the bluetooth console. It is able to rotate clockwise and counterclockwise, as well as move backwards and forwards at variable speeds. The segway bot is based on the Gyroboy code provided by the toppers examples, but has been modified to implement a rate-monotonic schedule, with the details outlined below.

This program was implemented using a rate monotone schedule containing the following tasks: (lower number means higher priority)

Task 1: touch sensor - Get touch sensor reading to start/stop the segway if pressed. Period: 1ms, Priority: 1, WECT: 69us

Task 2: gyro sensor - Get gyro sensor data and adjust the motor drive apropriately. Period: 2ms, Priority: 2, WECT: 91us

Task 3: balance task - Compute the speed and direction of the motors based on the values from the other tasks. Period: 4ms, Priority: 3, WECT: 121us

Task 4: bluetooth task - interpret the bluetooth command and adjust the motor drive apropriately. Period: 8ms, Priority: 4, WECT: 329us

Task 5: main task - read a command from the bluetooth console and store it in a buffer (also performs first-time setup). Not a periodic task; executes during idle time when no periodic tasks have been released. Priority 5.


Rate Monotonic Scheduler Info:

	We have 4 perodic tasks to schedule, as well as main, a background task. All of these tasks are simply periodic i.e. their periods are harmonic. The tasks are preemptable, and their deadlines are all equivalent to their periods, and cannot block eachother. So we can show that the system is scheduleable if its utilization is less than 1, and we can calculate the utilization of the system via the function:
		
			sum(ei / pi, i, 1, N)
			
	Where ei is the execution time of task i, pi is the period of task i, and N is the number of tasks in our system (in our case 4). 
	
			.069/1 + .091/2 + .121/4 + .329/8 = 0.185875
			
	So our utilization is less than one, and our system is scheduleable under the rate monotonic algorithm.
	
	The fgetc() command that is used to read input from the bluetooth console will block the execution of the task calling it until input is recieved, meaning that it cannot be placed in a periodic function, because it would make the worst-case-execution-time of the function effectively infinite. Thus, we need to ensure that our schedule has enough idle time in the execution that main, the lowest priority task, will get a chance to run within a reasonable time. Given that our utilization is significantly less than one, we have fulfilled this requirement.
	
	
Compilation and execution:

Unzip the segway folder and copy it to "/ev3rt-hrp2/sdk/workspace/"
cd to "/ev3rt-hrp2/sdk/workspace" which has a Makefile
do "make app=segway" which will generate the executable file named "app" in "/ev3rt-hrp2/sdk/workspace"

Copy "app" to the SD card having the ev3rt UImage file.
Go to apps and select "app" to start it.


Start:
Press the touch sensor to start the segway.

Stop:
Press the touch sensor to stop the segway.

Use bluetooth console to pass navigation commands to the segway:

    "F"/"f": move forward
    "B"/"b": move backward
    "U"/"u": speed up
    "D"/"d": slow down
    "L"/"l" : left turn for 90 degree
    "R"/"r": right turn for 90 degree
    others: invalid command 
