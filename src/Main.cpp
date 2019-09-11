#include <iostream>
#include <iomanip>
#include <fstream>
#include <string>
#include <thread>
#include <vector>
#include <array>
#include <random>
#include <time.h>
#include <vector>
#include <SDL.h>

#include "Constants.h"
#include "Globals.h"
#include "Ball.h"
#include "Wall.h"


// The window we will be rendering to
std::shared_ptr<SDL_Window> gWindow = nullptr;

// The window renderer
std::shared_ptr<SDL_Renderer> gRenderer = nullptr;

// Starts up SDL and creates window
bool initRenderer();

// Frees media and shuts down SDL
void closeRenderer();

// Draw circle using midpoint circle algorithm
void drawCircle(std::shared_ptr<SDL_Renderer> gRenderer, int32_t centreX, int32_t centreY, int32_t radius);

// Load parameters from file
bool chooseSimulations();
bool loadParametersFromFile();
void printParameters();

// Create objects;
void createObjects(std::vector<std::shared_ptr<Wall>>& walls, std::shared_ptr<Wall>& piston,
	std::shared_ptr<Wall>& bottom, std::vector<std::shared_ptr<Ball>>& balls);

// Render objects
void renderBall(std::shared_ptr<Ball> ball);
void renderWalls(std::vector<std::shared_ptr<Wall>> walls);

// Piston dynamics
void processPiston(std::shared_ptr<Wall> piston, std::shared_ptr<Wall> bottom,
	std::vector<std::shared_ptr<Ball>> balls, bool& finish);


/* Main function */
int main(int argc, char* args[]) // needs argc and args[] for the SDL 
{

	// Simulation objects
	std::vector<std::shared_ptr<Wall>> walls;
	std::shared_ptr<Wall> piston;
	std::shared_ptr<Wall> bottom;
	std::vector<std::shared_ptr<Ball>> balls;

	// Choose simulations
	if (!chooseSimulations())
		return 0;

	// create the objects
	createObjects(walls, piston, bottom, balls);

	if(balls.size() >= 1 && is_testing > 0)
	balls.at(0)->setSpecificDirection(ball0_speed, ball0_vel_angle);		// set for testing

	if(balls.size() >= 2 && is_testing > 0)
	balls.at(1)->setSpecificDirection(ball1_speed, ball1_vel_angle);		// set for testing

	// msg for stopping the threads
	bool finish = false;	

	// simulate
	for (auto ball : balls)
	{
		ball->simulate();
	}

	// process piston dynamics
	std::thread t1(processPiston, piston, bottom, std::ref(balls), std::ref(finish));

	// Start up SDL and create window
	if (!initRenderer())
	{
		std::cout << "Failed to initialize!" << std::endl;
	}
	else
	{
		// initialize variables
		bool quit = false; // loop flag
		int tempKey = 0;

		// Main loop
		while (!quit)
		{	
			
			SDL_Event e; // Event handler

			// Handle keycylinder events on queue
			while (SDL_PollEvent(&e) != 0)
			{
				// User requests quit
				if (e.type == SDL_QUIT)
				{
					quit = true;
				}

				// adjust temp at bottom
				if (e.type == SDL_KEYDOWN && e.key.repeat == 0)
				{
					switch (e.key.keysym.sym)
					{
					case SDLK_LEFT:
						tempKey = -1;
						break;
					case SDLK_RIGHT:
						tempKey = +1;
						break;
					default:
						break;
					}
				}

				if (e.type == SDL_KEYUP && e.key.repeat == 0)
				{
					switch (e.key.keysym.sym)
					{
					case SDLK_LEFT:
						if(tempKey < 0)
							tempKey = 0;
						break;
					case SDLK_RIGHT:
						if(tempKey > 0)
							tempKey = 0;
						break;
					default:
						break;
					}
				}
				
			}

			// update temp
			if (tempKey == +1) {
				double temp = bottom->getHeat();
				if (temp < bottom_temp_max) {
					temp++;
					bottom->setHeat(temp);
				}
			}
			if (tempKey == -1) {
				double temp = bottom->getHeat();
				if (temp > bottom_temp_min) {
					temp--;
					bottom->setHeat(temp);
				}
			}

			// Clear screen
			SDL_SetRenderDrawColor(gRenderer.get(), 0xFF, 0xFF, 0xFF, 0xFF);
			SDL_RenderClear(gRenderer.get());

			// Render ball
			for (auto ball : balls)
			{
				renderBall(ball);
			}

			// Render walls
			renderWalls(walls);

			// Update screen 
			SDL_RenderPresent(gRenderer.get());


		} // eof main loop

	}

	// close renderer
	closeRenderer();

	// ask ball:simulate to terminate
	for (auto ball : balls) {
		ball->setShutdown();
	}

	// wait for threads before returning
	finish = true;
	t1.join();

	return 0;
}


void createObjects(std::vector<std::shared_ptr<Wall>>& walls, std::shared_ptr<Wall>& piston, 
	std::shared_ptr<Wall>& bottom, std::vector<std::shared_ptr<Ball>>& balls)
{
	// create walls
	for (int nw = 0; nw < 4; nw++)
	{
		walls.push_back(std::make_shared<Wall>());
	}

	// lateral walls
	walls.at(0)->setPosition(CYLINDER_CENTER_POSITION_X - CYLINDER_WIDTH / 2, CYLINDER_CENTER_POSITION_Y);
	walls.at(0)->setSize(WALL_WIDTH, CYLINDER_HEIGHT + WALL_WIDTH);
	walls.at(0)->setWallType(WallType::lateral);

	walls.at(1)->setPosition(CYLINDER_CENTER_POSITION_X + CYLINDER_WIDTH / 2, CYLINDER_CENTER_POSITION_Y);
	walls.at(1)->setSize(WALL_WIDTH, CYLINDER_HEIGHT + WALL_WIDTH);
	walls.at(1)->setWallType(WallType::lateral);

	// piston and bottom walls
	walls.at(2)->setPosition(CYLINDER_CENTER_POSITION_X, CYLINDER_CENTER_POSITION_Y - CYLINDER_HEIGHT / 2);
	walls.at(2)->setSize(CYLINDER_WIDTH - WALL_WIDTH, WALL_WIDTH);
	walls.at(2)->setWallType(WallType::piston);

	walls.at(3)->setPosition(CYLINDER_CENTER_POSITION_X, CYLINDER_CENTER_POSITION_Y + CYLINDER_HEIGHT / 2);
	walls.at(3)->setSize(CYLINDER_WIDTH - WALL_WIDTH, WALL_WIDTH);
	walls.at(3)->setWallType(WallType::bottom);

	// create reference to piston
	piston = walls.at(2);
	piston->setMass(piston_mass);

	// create reference to bottom
	bottom = walls.at(3);
	bottom->setHeat(bottom_temp_min);

	// create cells into the cylinder
	int nCells = 0;		// calc number of cells
	int nRows, nCols;
	for (int n = 1; n <= nballs; n++)
	{
		nCells = n * n;
		nRows = n;
		nCols = n;
		if (nCells >= nballs)
		{
			break;
		}
	}

	struct Cell {
		double x, y;
	};

	std::vector<Cell> places;
	double inicX, inicY, aux;
	walls.at(0)->getPosition(inicX, aux);
	inicX += WALL_WIDTH * 0.5;
	walls.at(2)->getPosition(aux, inicY);
	inicY += WALL_WIDTH * 0.5;

	for (int nc = 0; nc < nCells; nc++)
	{
		Cell cell;
		cell.x = (inicX)+(nc % nCols) * ((CYLINDER_WIDTH - WALL_WIDTH) / nCols) + ((CYLINDER_WIDTH - WALL_WIDTH) / nCols) / 2.0;
		cell.y = (inicY)+(nc / nCols) * ((CYLINDER_HEIGHT - WALL_WIDTH) / nRows) + ((CYLINDER_HEIGHT - WALL_WIDTH) / nRows) / 2.0;
		places.push_back(cell);
	}

	// create balls at the cells
	for (int nb = 0; nb < nballs; nb++)
	{
		balls.push_back(std::make_shared<Ball>());
		balls.at(nb)->setPosition(places.at(nb).x, places.at(nb).y);

		// prevent ball size overflow
		if (nCols * 2.0 * ball_radius > CYLINDER_WIDTH * 0.98) {
			balls.at(nb)->setRadius(CYLINDER_WIDTH * 0.98 / nCols / 2.0);
		}
		else {
			balls.at(nb)->setRadius(ball_radius);
		}

		balls.at(nb)->setRandomDirection(ball_speed);
		balls.at(nb)->setMass(ball_mass);
		balls.at(nb)->setGravity(ball_gravity);
		balls.at(nb)->setGasMode((gas_mode > 0));
	}

	// set reference to other balls and walls into each ball
	for (auto ball : balls)
	{
		ball->setBalls(balls);
		ball->setWalls(walls);
	}
}

bool initRenderer()
{
	// Initialization flag
	bool success = true;

	// Initialize SDL
	if (SDL_Init(SDL_INIT_VIDEO) < 0)
	{
		std::cout << "SDL could not initialize! SDL Error: " << SDL_GetError() << std::endl;
		success = false;
	}
	else
	{
		// Create window
		gWindow = std::shared_ptr<SDL_Window>
			(SDL_CreateWindow("Elastic Collisions of Multiple Balls Simulation",
				SDL_WINDOWPOS_UNDEFINED, SDL_WINDOWPOS_UNDEFINED,
				SCREEN_WIDTH, SCREEN_HEIGHT, SDL_WINDOW_SHOWN),
				SDL_DestroyWindow);
		if (gWindow == nullptr)
		{
			std::cout << "Window could not be created! SDL Error: " << SDL_GetError() << std::endl;
			success = false;
		}
		else
		{
			// Create vsynced renderer for window
			gRenderer = std::shared_ptr<SDL_Renderer>
				(SDL_CreateRenderer(gWindow.get(),
					-1, SDL_RENDERER_ACCELERATED | SDL_RENDERER_PRESENTVSYNC),
					SDL_DestroyRenderer);
			if (gRenderer == NULL) {
				std::cout << "Renderer could not be created! SDL Error: " << SDL_GetError() << std::endl;
				success = false;
			}
			else
			{
				// Initialize renderer color
				SDL_SetRenderDrawColor(gRenderer.get(), 0xFF, 0xFF, 0xFF, 0xFF);
			}
		}
	}

	return success;
}

void closeRenderer() {
	// Quit SDL subsystems
	SDL_Quit();
}

void drawCircle(std::shared_ptr<SDL_Renderer> renderer, int32_t centreX, int32_t centreY, int32_t radius)
{
	const int32_t diameter = (radius * 2);

	int32_t x = (radius - 1);
	int32_t y = 0;
	int32_t tx = 1;
	int32_t ty = 1;
	int32_t error = (tx - diameter);

	while (x >= y)
	{
		//  Each of the following renders an octant of the circle
		SDL_RenderDrawPoint(renderer.get(), centreX + x, centreY - y);
		SDL_RenderDrawPoint(renderer.get(), centreX + x, centreY + y);
		SDL_RenderDrawPoint(renderer.get(), centreX - x, centreY - y);
		SDL_RenderDrawPoint(renderer.get(), centreX - x, centreY + y);
		SDL_RenderDrawPoint(renderer.get(), centreX + y, centreY - x);
		SDL_RenderDrawPoint(renderer.get(), centreX + y, centreY + x);
		SDL_RenderDrawPoint(renderer.get(), centreX - y, centreY - x);
		SDL_RenderDrawPoint(renderer.get(), centreX - y, centreY + x);

		if (error <= 0)
		{
			++y;
			error += ty;
			ty += 2;
		}

		if (error > 0)
		{
			--x;
			tx += 2;
			error += (tx - diameter);
		}
	}
}

void renderBall(std::shared_ptr<Ball> ball)
{
	double x, y, r;
	ball->getPosition(x, y);
	r = ball->getRadius();

	if (x < 0) return;

	SDL_SetRenderDrawColor(gRenderer.get(), 0x00, 0x00, 0x00, 0xFF);

	drawCircle(gRenderer, (int32_t) x, (int32_t) y, (int32_t) r);

}

void renderWalls(std::vector<std::shared_ptr<Wall>> walls)
{
	for (auto wall : walls) {

		// set the rectangle
		SDL_Rect rect;
		double x, y, w, h;
		wall->getPosition(x, y);
		wall->getSize(w, h);

		if (x < 0) continue;

		rect.x = int (x - w / 2);
		rect.y = int (y - h / 2);
		rect.w = int (w);
		rect.h = int (h);

		switch (wall->getWallType())
		{
			case WallType::lateral:
				SDL_SetRenderDrawColor(gRenderer.get(), 0x00, 0x00, 0x00, 0xFF);
				break;
			case WallType::piston:
				SDL_SetRenderDrawColor(gRenderer.get(), 0x00, 0x00, 0xFF, 0xFF);
				break;
			case WallType::bottom:
				double k = (wall->getHeat() / (bottom_temp_max - bottom_temp_min));
				uint16_t red = uint16_t(0xFF * 0.60 + 0xFF * 0.40 * k);
				SDL_SetRenderDrawColor(gRenderer.get(), red, 0x00, 0x00, 0xFF);
				break;
		}

		SDL_RenderDrawRect(gRenderer.get(), &rect);
	}
}

void processPiston(std::shared_ptr<Wall> piston, std::shared_ptr<Wall> bottom, 
	std::vector<std::shared_ptr<Ball>> balls, bool& finish)
{

	// Set to piston dynamics
	long pistoncollisions = 0;
	double upForce = 0.0;
	double mv = 0.0;
	double avgUpForce = 0.0;
	double downForce = 0.0;	// for testing
	downForce = piston_gravity * piston_mass;
	double pistonVel = 0.0;
	int cycleDuration = 10;  // define cycle duration (ms) to calc up force
	bool gasMode = (gas_mode > 0);  
	double temp = 0;

	// init stop watch
	std::chrono::time_point<std::chrono::system_clock> lastUpdate;
	lastUpdate = std::chrono::system_clock::now();
	
	auto f0 = [balls, cycleDuration, gasMode, &avgUpForce, &downForce, &temp, &finish]() {
		long simTime = 0;
		while (!finish) {
			std::this_thread::sleep_for(std::chrono::milliseconds(1000));
			// count balls inside piston (testing)
			int count = 0;
			for (auto ball : balls)
			{
				double x, y;
				ball->getPosition(x, y);
				if (x > CYLINDER_CENTER_POSITION_X - CYLINDER_WIDTH * 0.5 && x  <  CYLINDER_CENTER_POSITION_X + CYLINDER_WIDTH * 0.5 &&
					y > CYLINDER_CENTER_POSITION_Y - CYLINDER_HEIGHT * 0.5 && y < CYLINDER_CENTER_POSITION_Y + CYLINDER_HEIGHT * 0.5)
				{
					count++;
				}
			}
		
			std::cout << "Time:" << simTime << "\t";
			std::cout << "balls:" << count << "\t";
			
			if (gasMode) {
				std::cout << "gas avg(1.0s) up force : " << std::fixed << std::setprecision(2) << abs(avgUpForce) << "  \t";
				std::cout << "piston down force: " << std::setprecision(2) << downForce << " \t";
			}

			std::cout << "~temp(bottom) : " << std::setprecision(2) << temp << std::endl; 
			
			simTime++;
		}
	};

	std::thread t1(f0);		// thread for monitoring and testing


	while (!finish)
	{
		// compute time difference (in ms)
			auto timeSinceLastUpdate = std::chrono::duration_cast<std::chrono::milliseconds>
				(std::chrono::system_clock::now() - lastUpdate).count();

		// Receive and process msg from balls about collision with piston
		for (auto ball : balls)
		{
			while (ball->dataIsAvailable())
			{
				//std::cout << "Main: ball msg available" << std::endl;
				CollisionData msg = ball->receiveMsg();
				CollisionType colType = msg.type;
				double velocity = msg.velY;	// downward is positve
				switch (colType) {
				case CollisionType::bottomCollision:
					//std::cout << "ball-bottom collision" << std::endl;
					break;
				case CollisionType::pistonCollision:
					//std::cout << "*gas* ball-piston collision" << std::endl;
					pistoncollisions++;
					mv += 2.0 * ball_mass * velocity;	// force in the piston
					break;
				}
			}
		}


		// Process piston dymamics
		if (timeSinceLastUpdate > cycleDuration) {

			// F * t = m * v
			upForce = mv / (timeSinceLastUpdate / 1000.0);
			avgUpForce = avgUpForce + upForce / (1000/cycleDuration) - avgUpForce / (1000/cycleDuration);	// 1000ms average (for printing)

			double posX, posY;
			double pistonW, pistonH;
			double velX, velY;
			double acceleration;

			piston->getPosition(posX, posY);
			piston->getSize(pistonW, pistonH);
			piston->getVelocity(velX, velY);

			// F = m * a
			acceleration = (upForce + downForce) / (piston_mass + 1e-10);  // downward = positive; prevent overflow

			// calc piston position  (x - x0) = vo * t + 1/2 * a * t^2
			posY = posY + pistonVel * (timeSinceLastUpdate / 1000.0) + 0.5 * acceleration * pow(timeSinceLastUpdate / 1000.0, 2);   // aprox;

			// piston velocity for next cycle
			pistonVel = velY + acceleration * (timeSinceLastUpdate / 1000.0);

			// Limit piston movement inside cylinder area
			if (posY < CYLINDER_CENTER_POSITION_Y - CYLINDER_HEIGHT * 0.5)
			{
				posY = CYLINDER_CENTER_POSITION_Y - CYLINDER_HEIGHT * 0.5;

			}
			else if ((posY > CYLINDER_CENTER_POSITION_Y + CYLINDER_HEIGHT * 0.5 - WALL_WIDTH))
			{
				posY = CYLINDER_CENTER_POSITION_Y + CYLINDER_HEIGHT * 0.5 - WALL_WIDTH;
			}

			// piston at the top: no velocity upward
			if (posY <= (CYLINDER_CENTER_POSITION_Y - CYLINDER_HEIGHT * 0.5))
			{
				if (pistonVel < 0) pistonVel = 0;
			}
			// piston at the bottom: no velocity downward
			else if (posY >= (CYLINDER_CENTER_POSITION_Y + CYLINDER_HEIGHT * 0.5 - WALL_WIDTH))
			{
				if (pistonVel > 0) pistonVel = -pistonVel;
			}

			// update piston
			piston->setPosition(posX, posY);
			piston->setVelocity(0.0, pistonVel);

			// reset for next cycle
			pistoncollisions = 0;
			mv = 0.0;
			temp = bottom->getHeat();

			// reset stop watch for next cycle
			lastUpdate = std::chrono::system_clock::now();

		} // eof cycle computations


	} // eof while loop


	t1.join(); // it will close because it is monitoring the finish flag

}


bool loadParametersFromFile() 
{

	struct Reading {
		std::string name;
		double value;
	};

	std::cout << "Loading parameters file...\n" << std::endl;
	std::cout << "Please enter input file name: [Parameters.txt]";

	std::string iname;
	std::getline(std::cin, iname);

	if (iname.empty()) {
		iname = "Parameters.txt";
	}

	// the input stream
	std::ifstream ist{ iname };

	if (!ist)
	{
		std::cout << "\nCan't open input file " << iname << std::endl;
		return false;
	}

	if (ist) 
	{
		std::cout << "Reading the file " << iname << " ...\n" << std::endl;
	}

	// it will store the data
	std::vector<Reading> parameters;

	// reading from file
	while (ist) {
		std::string name{};
		double value{};
		ist >> name >> value;
		if (ist) {
			parameters.push_back(Reading{ name, value });
		}
	}

	// test the data
	if (parameters.size() != 15)
		return false;

	// load in memory
	for (auto parameter : parameters)
	{
		if (parameter.name == "GAS_MODE") gas_mode = parameter.value;

		if (parameter.name == "NBALLS")	nballs = parameter.value;
		if (parameter.name == "BALL_RADIUS") ball_radius = parameter.value;
		if (parameter.name == "BALL_SPEED") ball_speed = parameter.value;
		if (parameter.name == "BALL_MASS") ball_mass = parameter.value;
		if (parameter.name == "BALL_GRAVITY") ball_gravity = parameter.value;

		if (parameter.name == "IS_TESTING") is_testing = parameter.value;
		if (parameter.name == "BALL0_SPEED") ball0_speed = parameter.value;
		if (parameter.name == "BALL0_VEL_ANGLE") ball0_vel_angle = parameter.value;
		if (parameter.name == "BALL1_SPEED") ball1_speed = parameter.value;
		if (parameter.name == "BALL1_VEL_ANGLE") ball1_vel_angle = parameter.value;

		if (parameter.name == "PISTON_MASS") piston_mass = parameter.value;
		if (parameter.name == "PISTON_GRAVITY") piston_gravity = parameter.value;

		if (parameter.name == "BOTTOM_TEMP_MIN") bottom_temp_min = parameter.value;
		if (parameter.name == "BOTTOM_TEMP_MAX") bottom_temp_max = parameter.value;
	}

	return true;
}

void printParameters()
{
	std::cout << "GAS_MODE " << gas_mode << std::endl;

	std::cout << "NBALLS " << nballs << std::endl;
	std::cout << "BALL_RADIUS " << ball_radius << std::endl;
	std::cout << "BALL_SPEED " << ball_speed << std::endl;
	std::cout << "BALL_MASS " << ball_mass << std::endl;
	std::cout << "BALL_GRAVITY " << ball_gravity << std::endl;

	std::cout << "IS_TESTING " << is_testing << std::endl;
	std::cout << "BALL0_SPEED " << ball0_speed << std::endl;
	std::cout << "BALL0_VEL_ANGLE " << ball0_vel_angle << std::endl;
	std::cout << "BALL1_SPEED " << ball1_speed << std::endl;
	std::cout << "BALL1_VEL_ANGLE " << ball1_vel_angle << std::endl;

	std::cout << "PISTON_MASS " << piston_mass << std::endl;
	std::cout << "PISTON_GRAVITY " << piston_gravity << std::endl;

	std::cout << "BOTTOM_TEMP_MIN " << bottom_temp_min << std::endl;
	std::cout << "BOTTOM_TEMP_MAX " << bottom_temp_max << std::endl;
}


bool chooseSimulations()
{

	std::cout << "Please, choose one example or read parameters from file:" << std::endl;
	std::cout << "[1] Three balls. (default)" << std::endl;
	std::cout << "[2] One ball at rest. Piston with gravity." << std::endl;
	std::cout << "[3] Eight balls at rest. Piston with gravity." << std::endl;
	std::cout << "[4] Fifty balls (gas simulation approximation). Piston with gravity." << std::endl;
	std::cout << "[5] Load parameters from file." << std::endl;

	std::string input;
	std::cout << "\nOption: ";
	std::getline(std::cin, input);

	int menuOption = 1;

	if (!input.empty())
	{
		menuOption = stoi(input);
	}

	switch (menuOption)
	{
	case 1:
		// Simulation mode
		gas_mode = 0;
		// Ball
		nballs = 3;				// 100 max aprox  
		ball_radius = 20;			// max 0.5 * WALL_WIDTH aprox
		ball_speed = 60;			// pixels/s (SPPED < RADIUS * 200 max aprox)
		ball_mass = 0.1;			// 0.01 * piston_mass min aprox  
		ball_gravity = 0;		// if zero, simulate gas
		// Piston 
		piston_mass = 10.0;
		piston_gravity = 0.0;  // if zero, piston stops
		break;

	case 2:
		// Simulation mode
		gas_mode = 0;
		// Ball
		nballs = 1;				// 100 max aprox  
		ball_radius = 20;			// max 0.5 * WALL_WIDTH aprox
		ball_speed = 0;			// pixels/s (SPPED < RADIUS * 200 max aprox)
		ball_mass = 0.2;			// 0.01 * piston_mass min aprox  
		ball_gravity = 0;		// if zero, simulate gas
		// Piston 
		piston_mass = 10.0;
		piston_gravity = 10.0;  // if zero, piston stops
		break;

	case 3:
		// Simulation mode
		gas_mode = 0;
		// Ball
		nballs = 8;				// 100 max aprox  
		ball_radius = 8;			// max 0.5 * WALL_WIDTH aprox
		ball_speed = 0;			// pixels/s (SPPED < RADIUS * 200 max aprox)
		ball_mass = 0.2;			// 0.01 * piston_mass min aprox  
		ball_gravity = 0;		// if zero, simulate gas
		// Piston 
		piston_mass = 10.0;
		piston_gravity = 10.0;  // if zero, piston stops
		break;

	case 4:
		// Simulation mode
		gas_mode = 1;
		// Ball
		nballs = 50;				// 100 max aprox  
		ball_radius = 4;			// max 0.5 * WALL_WIDTH aprox
		ball_speed = 60;			// pixels/s (SPPED < RADIUS * 200 max aprox)
		ball_mass = 0.1;			// 0.01 * piston_mass min aprox  
		ball_gravity = 0;		// if zero, simulate gas
		// Piston 
		piston_mass = 10.0;
		piston_gravity = 10.0;  // if zero, piston stops
		std::cout << "\nPlease, click on cylinder and increase energy with '>' right key." << std::endl;
		break;

	case 5:
		if (!loadParametersFromFile())
		{
			std::cout << "Can not load the parameters file." << std::endl;
			std::cout << "Continue with the default? [y]" << std::endl;
			std::string tc;
			std::getline(std::cin, tc);
			if (tc.empty()) {
				tc = "y";
			}
			if (tc == "y")
			{
				// Simulation mode
				gas_mode = 0;
				// Ball
				nballs = 3;				// 100 max aprox  
				ball_radius = 20;			// max 0.5 * WALL_WIDTH aprox
				ball_speed = 60;			// pixels/s (SPPED < RADIUS * 200 max aprox)
				ball_mass = 0.1;			// 0.01 * piston_mass min aprox  
				ball_gravity = 0;		// if zero, simulate gas
				// Piston 
				piston_mass = 10.0;
				piston_gravity = 0.0;  // if zero, piston stops
				break;
			}
			
			return false;
		}
		break;

	default:
		// Simulation mode
		gas_mode = 0;
		// Ball
		nballs = 3;				// 100 max aprox  
		ball_radius = 20;			// max 0.5 * WALL_WIDTH aprox
		ball_speed = 60;			// pixels/s (SPPED < RADIUS * 200 max aprox)
		ball_mass = 0.1;			// 0.01 * piston_mass min aprox  
		ball_gravity = 0;		// if zero, simulate gas
		// Piston 
		piston_mass = 100.0;
		piston_gravity = 0.0;  // if zero, piston stops
		break;
	}

	std::cout << "\nPlease, verify the simulation parameters:\n" << std::endl;
	printParameters();

	std::cout << "\nPress enter to continue..." << std::endl;
	std::cin.ignore();

	return true;
}