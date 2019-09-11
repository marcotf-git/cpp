#ifndef CONSTANTS_H
#define CONSTANTS_H

// Screen dimension constants
const double SCREEN_WIDTH = 480;
const double SCREEN_HEIGHT = 480;

// cylinder position and dimensions
const double CYLINDER_CENTER_POSITION_X = SCREEN_WIDTH / 2;
const double CYLINDER_CENTER_POSITION_Y = SCREEN_HEIGHT / 2;
const double CYLINDER_WIDTH = SCREEN_WIDTH / 2;
const double CYLINDER_HEIGHT = int(SCREEN_HEIGHT * 0.80);

// wall width
const double WALL_WIDTH = 20;


//// Simulation mode
//const double GAS_MODE = 1;
//
//// Ball constants
//const double NBALLS = 0;			// 100 max aprox  
//const double BALL_RADIUS = 8;		// max 0.5 * WALL_WIDTH aprox
//const double BALL_SPEED = 0;		// pixels/s (SPPED < RADIUS * 200 max aprox)
//const double BALL_MASS = 0.1;       // 0.01 * PISTON_MASS min aprox  
//const double BALL_GRAVITY = 0;      // if zero, simulate gas
//
//const double IS_TESTING = 1;
//const double BALL0_SPEED = 10;		// it will replace the random generated numbers
//const double BALL0_VEL_ANGLE = 0;
//const double BALL1_SPEED = 10;
//const double BALL1_VEL_ANGLE = -180;
//
//// Piston constants
//const double PISTON_MASS = 1000.0;
//const double PISTON_GRAVITY = 10.0;  // if zero, piston stops
//
//// Bottom constants
//const double BOTTOM_TEMP_MIN = 0.0;
//const double BOTTOM_TEMP_MAX = 100.0;


#endif

