// Simulation mode
double gas_mode = 0;

// Ball
double nballs = 8;				// 100 max aprox  
double ball_radius = 8;			// max 0.5 * WALL_WIDTH aprox
double ball_speed = 0;			// pixels/s (SPPED < RADIUS * 200 max aprox)
double ball_mass = 0.2;			// 0.01 * piston_mass min aprox  
double ball_gravity = 0;		// if zero, simulate gas

double is_testing = 0;
double ball0_speed = 10;		// it will replace the random generated numbers
double ball0_vel_angle = 0;
double ball1_speed = 10;
double ball1_vel_angle = -180;

// Piston 
double piston_mass = 10.0;
double piston_gravity = 10.0;  // if zero, piston stops

// Bottom 
double bottom_temp_min = 0.0;
double bottom_temp_max = 100.0;
