#include "robo.h"

// This is the code sketch to make the KeyeStudio robot do a choreographed dance routine.
//                                     ----------
// You don't need to change anything in this code sketch.
// Just upload it straight to the robot.

const int WHEELSPEED = 150;

void loop()
{
	forward(1000,WHEELSPEED,WHEELSPEED);  // forward basic movement routine for one second at full speed
	halt(500);

	reverse(1000,WHEELSPEED,WHEELSPEED);  // reverse basic movement routine for one second at full speed
	halt(500);

	leftSpin(2000,WHEELSPEED);     // turning left basic movement routine for two seconds at full speed
	halt(500);

	rightSpin(2000,WHEELSPEED);    // turning right basic movement routine for two seconds at full speed
	halt(500);

	forward(1000,WHEELSPEED,WHEELSPEED);  // forward basic movement routine for one second at full speed
	halt(500);

	leftSpin(800,WHEELSPEED);      // turning left basic movement routine for 4/5 of a second at full speed
	halt(500);

	forward(500,WHEELSPEED,WHEELSPEED);   // forward basic movement routine for 1/2 a second at full speed
	halt(500);

	reverse(1000,WHEELSPEED,WHEELSPEED);  // reverse basic movement routine for one second at full speed
	halt(500);

// SHOOGLE!!!

	rightSpin(800,WHEELSPEED);     // turning right basic movement routine for 4/5 of a second at full speed
	halt(500);              // wait for 1/2 a second
	shoogle();              // run the shoogle sub-routine

	rightSpin(800,WHEELSPEED);     // turning right basic movement routine for 4/5 of a second at full speed
	halt(500);              // wait for 1/2 a second
	shoogle();              // run the shoogle sub-routine

	leftSpin(500,WHEELSPEED);      // turning left basic movement routine for 1/2 a second at full speed
	halt(500);              // wait for 1/2 a second
	shoogle();              // run the shoogle sub-routine

	leftSpin(500,WHEELSPEED);      // turning left basic movement routine for 1/2 a second at full speed
	halt(500);              // wait for 1/2 a second
	shoogle();              // run the shoogle sub-routine

	leftSpin(3000,WHEELSPEED);     // turning left basic movement routine for 3 seconds at full speed
	halt(500);              // wait for 1/2 a second

	return;
}

void shoogle()
{
	int i=0;

	i = 5;
	while (i>0)
	{
		forward(100,WHEELSPEED,WHEELSPEED);  // forward basic movement routine for 1/10 of a second at full speed
		halt(200);             // wait for 1/5 of a second

		reverse(100,WHEELSPEED,WHEELSPEED);  // reverse basic movement routine for 1/10 of a second at full speed
		halt(200);             // wait for 1/5 of a second

		i--;
	}
}
