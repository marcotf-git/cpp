#include <iostream>
#include <thread>
#include <future>
#include <memory>
#include <random>
#include <algorithm>
#include <math.h>
#include <array>

#include "Wall.h"
#include "Ball.h"


template <typename T>
T MessageQueue<T>::receive()
{

	std::unique_lock<std::mutex> uLock(_mutex);	// needs unique_lock because the lock will be temporarily unlocked inside wait 
	_cond.wait(uLock, [this] { return !_queue.empty(); });	// enter the wait state, release the lock and resume if new data is available

	T msg = std::move(_queue.back());
	_queue.pop_back();

	//std::cout << "Message " << msg  << " has been received from the Ball msg queue" << std::endl;

	return msg;
}

template <typename T>
void MessageQueue<T>::send(T&& msg)
{	

	std::lock_guard<std::mutex> uLock(_mutex);
	
	//std::cout << "Message " <<  msg << " has been sent to the Ball msg queue" << std::endl;
	
	_queue.push_back(std::move(msg));
	_cond.notify_one();

}

template <typename T>
int MessageQueue<T>::getSize()
{
	std::lock_guard<std::mutex> uLock(_mutex);
	return _queue.size();
}

// helper function declaration
double distanceToPoint(double x1, double y1, double x2, double y2);
bool squareCircleCollision(double x1, double y1, double w1, double h1, double x2, double y2, double r2);
bool circleCircleCollision(double x1, double y1, double r1, double x2, double y2, double r2);
void resolveCollision(double& posX, double& posY, double& velX, double& velY, double speed, double mass, double radius,
	double& otherX, double& otherY, double& otherVx, double& otherVy, double otherSpeed, double otherM, double otherR);

Ball::Ball()
{
	_type = ObjectType::objectBall;
	_radius = 0;
	_velX = 0;
	_velY = 0;
	_speed = 0;
	_shutDown = false;
}


void Ball::setSpecificDirection(double speed, double angle)
{
	// set velocity
	double vx, vy;
	double pi = acos(-1);
	vx = speed * cos(angle * 2 * pi / 360);
	vy = -speed * sin(angle * 2 * pi / 360);
	setVelocity(vx, vy);
}

void Ball::setRandomDirection(double speed)
{
	// pick angle at random and set direction of the ball
	double angle;
	std::random_device rd;
	std::mt19937 generator(rd());
	std::array<double, 5> intervals{ -60.0, 60.0, 120.0, 240.0, 360.0 };
	std::array<double, 4> weights{ 1.0, 1.0, 1.0, 1.0 };
	std::piecewise_constant_distribution<double>
		distribution(intervals.begin(), intervals.end(), weights.begin());
	angle = distribution(generator);

	std::cout << "Angle " << angle << std::endl;
	
	// set velocity
	setSpecificDirection(speed, angle);
}

bool Ball::dataIsAvailable()
{
	return (_msgQueue.getSize() > 0);
}

CollisionData Ball::receiveMsg()
{
	return _msgQueue.receive();
}

bool Ball::getShutdown()
{
	return _shutDown;
}

void Ball::setShutdown()
{
	_shutDown = true;
}

// implement the virtual function that will execute a member function into a thread
void Ball::simulate()
{
	// Start a thread with the member function "play" and the object "this"
	// Add the created thread into the _threads vector of parent class (using emplace_back which means 
	// move semantics)
	_threads.emplace_back(std::thread(&Ball::play, this));
}

// function which is executed in athread
void Ball::play()
{
	// print Ball id and thread id
	std::unique_lock<std::mutex> uLock(_mtxCout);
	std::cout << "Ball::simulate Ball _id=" << getID() << "  thread id=" << std::this_thread::get_id() << std::endl;
	uLock.unlock();

	// initialize variables

	// define cycle duration (to update ball position and check cylinder)
	int cycleDuration = 10;  // duration of a single simulation cycle in ms

	// init stop watch
	std::chrono::time_point<std::chrono::system_clock> lastUpdate;
	lastUpdate = std::chrono::system_clock::now();

	// infinite simulation loop
	while (!getShutdown()) 
	{
		// compute time difference to stop watch (in ms)
		auto timeSinceLastUpdate = std::chrono::duration_cast<std::chrono::milliseconds>
			(std::chrono::system_clock::now() - lastUpdate).count();

		// if past cycle time, update position and check cylinder
		if (timeSinceLastUpdate >= cycleDuration)
		{
			// reset stop watch for next cycle
			lastUpdate = std::chrono::system_clock::now();

			// calc next position
			double velX, velY, nextX, nextY;
			double posX, posY, dx, dy;
			getVelocity(velX, velY);
			getPosition(posX, posY);

			// calc gravity acceleration effect
			double dVy = _ball_gravity * (timeSinceLastUpdate / 1000.0);  // dVy = Ay * dt

			dx = velX * (timeSinceLastUpdate / 1000.0); // dx = Vx * dt
			dy = velY * (timeSinceLastUpdate / 1000.0); // dy = Vy * dt

			nextX = posX + dx;
			nextY = posY + dy;

			// check if ball has reached walls and invert direction
			bool hasColidedW = false;
			bool hasColidedP = false;
			bool hasColidedB = false;

			// process ball with wall collision
			for (auto wall : _walls) {

				// process piston->ball elastic collision (the piston moves in y axis): 
				// m0*v0i + m1*v1i = m0*v0f + m1*v1f
				// Ec0i + Ec1i = Ec0f + Ec1f
				if (wall->getWallType() == WallType::piston)
				{
					
					hasColidedP = (checkWallCollision(nextX, nextY, wall));

					// if ball is inside the piston: flag the collision
					if (hasColidedP) {

						//std::cout << "piston-ball colision" << std::endl;
						
						double otherVx, otherVy;
						wall->getVelocity(otherVx, otherVy);
						double x, y, w, h;
						wall->getPosition(x, y);
						wall->getSize(w, h);

						double velCollisionGas = velY - otherVy;

						double otherX, otherY;
						otherX = posX;	// this is like a collision with a ball at the same x coordinate
						otherY = y;

						double speed = getSpeed();
						double otherSpeed = wall->getSpeed();

						double mass = _mass;
						double otherM = wall->getMass();

						double radius = _radius;
						double otherR = h / 2;	// it is like a collision with a ball at the same x coordinate, with r = h/2

						resolveCollision(posX, posY, velX, velY, speed, mass, radius,
							otherX, otherY, otherVx, otherVy, otherSpeed, otherM, otherR);

						// Two models of simulation: in the gas mode, the piston is processed in main
						if (!(_gas_mode))
						{
							// update positions and velocities
							wall->setVelocity(0, otherVy);
							wall->setPosition(x, otherY);

							setVelocity(velX, velY);
							setPosition(posX, posY);
						}
						else
						{
							// msg to main to update piston:
							// F*dt = m0*(v0i-v0f)
							// F = m1*a
							_msgQueue.send({ CollisionType::pistonCollision, velCollisionGas });

							// update ball position and velocity
							// m0*v0i + m1*v1i = m0*v0f + m1*v1f
							// Ec0i + Ec1i = Ec0f + Ec1f
							setVelocity(velX, velY);
							setPosition(posX, posY);

						}

					}

				}
				else 
				{
					// process ball with wall and bottom collisions
					// m0*v0i = m0*v0f
					hasColidedW = checkWallCollision(nextX, nextY, wall);

					if (hasColidedW) {
						// verify if it has collided with bottom or lateral and send message to main
						if (wall->getWallType() == WallType::lateral) {
							//std::cout << "ball->lateral collision" << std::endl << std::endl;
							_msgQueue.send({ CollisionType::bottomCollision, velY });
						}
						else if (wall->getWallType() == WallType::bottom) {
							//std::cout << "ball->bottom collision" << std::endl << std::endl;
							_msgQueue.send({ CollisionType::bottomCollision, velY });
						}
					}


					// if there is a collision, change ball direction
					if (hasColidedW) {

						double x, y;
						wall->getPosition(x, y);

						// if it has collided in dx, set x component of velocity
						if (checkWallCollision(nextX, posY, wall))
						{
							if (posX < x) { // the wall is in front
								velX = -abs(velX);
							}
							else { // the wall is behind
								velX = abs(velX);
							}

						}
						// if it has collided in dy, set y component of velocity
						if (checkWallCollision(posX, nextY, wall))
						{
							if (posY > y) {  // the wall is behind
								velY = abs(velY);
							}
							else {	// the wall is in front
								velY = -abs(velY);
								
							}

							// Adjust ball velocity to aprox temperature effect of bottom
							if (wall->getWallType() == WallType::bottom) {
								if (abs(velY) < 1.0 * wall->getHeat()) 
								{
									velY = -1.0 * wall->getHeat();
								}
							}
						}

						// update velocity
						setVelocity(velX, velY);

						// update position
						//dx = velX * (timeSinceLastUpdate / 1000.0); // do not uncomment: system becomes incorrect
						//dy = velY * (timeSinceLastUpdate / 1000.0); // do not uncomment: system becomes incorrect
						//nextX = posX + dx;  // do not uncomment: system becomes incorrect
						//nextY = posY + dy;
						setPosition(nextX, nextY);

						break;
					}
				}
				
				
			} // eof ball-wall collision


			// process ball with ball collisions
			// m0*v0i + m1*v1i = m0*v0f + m1*v1f
			// Ec0i + Ec1i = Ec0f + Ec1f
			if (!hasColidedW && !hasColidedP)
			{
				int thisBallId = getID();
				for (auto ball : _balls)
				{
					int otherBallId = ball->getID();
					if (thisBallId != otherBallId)
					{
						hasColidedB = checkBallCollision(nextX, nextY, ball);
						//std::cout << "Ball collision id1 " << thisBallId << "  id2 "<< otherBallId << std::endl;
						//std::this_thread::sleep_for(std::chrono::milliseconds(3000));

						// if there is a collision, change ball direction (elastic collision)
						if (hasColidedB) {

							double otherX, otherY;
							ball->getPosition(otherX, otherY);

							double otherVx, otherVy;
							ball->getVelocity(otherVx, otherVy);

							double speed = getSpeed();
							double otherSpeed = ball->getSpeed();

							double mass = getMass();
							double otherM = ball->getMass();

							double radius = getRadius();
							double otherR = ball->getRadius();

							resolveCollision(posX, posY, velX, velY, speed, mass, radius,
								otherX, otherY, otherVx, otherVy, otherSpeed, otherM, otherR);

							// update velocities
							setVelocity(velX, velY + dVy);
							ball->setVelocity(otherVx, otherVy);

							// update positions
							setPosition(posX, posY);
							ball->setPosition(otherX, otherY);

							break;
						}
					}
				}

			} // eof new ball-ball collision


			// if has not colided, just update position and velocity
			if (!hasColidedW && !hasColidedB && !hasColidedP)
			{
				setPosition(nextX, nextY);
				setVelocity(velX, velY + dVy);
			} 


		} // eof cycle

		// sleep at every iteration to reduce CPU usage
		std::this_thread::sleep_for(std::chrono::milliseconds(1));

	} // eof simulation loop

}

// Verify and process the collision of the ball with the cylinder walls
bool Ball::checkWallCollision(double nextX, double nextY, std::shared_ptr<Wall> wall)
{
	double x, y, w, h;
	bool collision = false;

	wall->getPosition(x, y);
	wall->getSize(w, h);
	collision = squareCircleCollision(x, y, w, h, nextX, nextY, _radius);

	return collision;
}

// Verify and process the collision of the ball with the cylinder walls
bool Ball::checkBallCollision(double nextX, double nextY, std::shared_ptr<Ball> ball)
{
	double x, y, r;
	bool collision = false;
	
	ball->getPosition(x, y);
	r = ball->getRadius();
	collision = circleCircleCollision(nextX, nextY, _radius, x, y, r);

	return collision;
}

// verify collision between square x1,y1,w1,h1 and circle x2,y2,r2
bool squareCircleCollision(double x1, double y1, double w1, double h1, double x2, double y2, double r2)
{
	double closestX, closestY;
	bool collision = false;
	
	// find the closest x coordinate of the wall to the circle
	if (x1 + w1 * 0.5 < x2 - r2) { closestX = x1 + w1 * 0.5; }
	else if (x1 - w1 * 0.5 > x2 + r2) { closestX = x1 - w1 * 0.5; }
	else { closestX = x2; }

	// find the closest y coordinate of the wall to the circle
	if (y1 + h1 * 0.5 < y2 - r2) { closestY = y1 + h1 * 0.5; }
	else if (y1 - h1 * 0.5 > y2 + r2) { closestY = y1 - h1 * 0.5; }
	else { closestY = y2; }

	if (distanceToPoint(x2, y2, closestX, closestY) < r2) {
		collision = true;
	}

	return collision;
}

// verify collision between circle x1,y1,r1 and circle x2,y2,r2
bool circleCircleCollision(double x1, double y1, double r1, double x2, double y2, double r2)
{
	double distance;
	bool collision;

	distance = distanceToPoint(x1, y1, x2, y2);
	collision = (distance < (r1 + r2));

	return collision;
}

double distanceToPoint(double x1, double y1, double x2, double y2)
{
	double distance;

	distance = sqrt(pow(x1 - x2, 2.0) + pow(y1 - y2, 2.0));

	return distance;
}

// resolve collision elastic 
void resolveCollision(double& posX, double& posY, double& velX, double& velY, double speed, double mass, double radius, 
	double& otherX, double& otherY, double& otherVx, double& otherVy, double otherSpeed, double otherM, double otherR)
{

	if (mass == 0 || otherM == 0)
	{
		std::cout << "Error: mass is zero!" << std::endl;
		return;
	}

	double angleCol = atan2(otherY - posY, otherX - posX);
	double direction = atan2(velY, velX);
	double otherDirection = atan2(otherVy, otherVx);

	double new_xspeed = speed * cos(direction - angleCol);
	double new_yspeed = speed * sin(direction - angleCol);

	double new_xspeedOther = otherSpeed * cos(otherDirection - angleCol);
	double new_yspeedOther = otherSpeed * sin(otherDirection - angleCol);

	double final_xspeed = ((mass - otherM) * new_xspeed + (otherM + otherM) * new_xspeedOther) / (mass + otherM);
	double final_xspeedOther = ((mass + mass) * new_xspeed + (otherM - mass) * new_xspeedOther) / (mass + otherM);
	double final_yspeed = new_yspeed;
	double final_yspeedOther = new_yspeedOther;

	double cosAngle = cos(angleCol);
	double sinAngle = sin(angleCol);

	double newVelX, newVelY;
	newVelX = cosAngle * final_xspeed - sinAngle * final_yspeed;
	newVelY = sinAngle * final_xspeed + cosAngle * final_yspeed;

	double newOtherVelX, newOtherVelY;
	newOtherVelX = cosAngle * final_xspeedOther - sinAngle * final_yspeedOther;
	newOtherVelY = sinAngle * final_xspeedOther + cosAngle * final_yspeedOther;

	// get the minimum translation distance to push balls apart after intersecting
	struct Position {
		double x;
		double y;
		double length() {
			return sqrt(pow(x, 2.0) + pow(y, 2.0));
		}
	} pos1, pos2, posDiff, mtd;

	pos1.x = posX;
	pos1.y = posY;
	pos2.x = otherX;
	pos2.y = otherY;
	posDiff.x = pos1.x - pos2.x;
	posDiff.y = pos1.y - pos2.y;

	double d = posDiff.length();
	double k = (((radius + otherR) - d) / d);
	mtd.x = posDiff.x * k;
	mtd.y = posDiff.y * k;

	double im = 1 / mass;
	double imOther = 1 / otherM;

	// push-pull them apart based off their mass
	pos1.x = pos1.x + mtd.x * (im / (im + imOther));
	pos1.y = pos1.y + mtd.y * (im / (im + imOther));
	pos2.x = pos2.x - mtd.x * (imOther / (im + imOther));
	pos2.y = pos2.y - mtd.y * (imOther / (im + imOther));

	// Process ball with wall collision generated by the pushing balls apart
	if ((pos1.x + radius >= CYLINDER_CENTER_POSITION_X + CYLINDER_WIDTH / 2) ||
		(pos1.x - radius <= CYLINDER_CENTER_POSITION_X - CYLINDER_WIDTH / 2))
	{
		newVelX = -1.0 * newVelX;
	}

	if ((pos1.y + radius >= CYLINDER_CENTER_POSITION_Y + CYLINDER_HEIGHT / 2) ||
		(pos1.y - radius <= CYLINDER_CENTER_POSITION_Y - CYLINDER_HEIGHT / 2))
	{
		newVelY = -1.0 * newVelY;
	}

	if ((pos2.x + otherR >= CYLINDER_CENTER_POSITION_X + CYLINDER_WIDTH / 2) ||
		(pos2.x - otherR <= CYLINDER_CENTER_POSITION_X - CYLINDER_WIDTH / 2))
	{
		newOtherVelX = -1.0 * newOtherVelX;
	}

	if ((pos2.y + otherR >= CYLINDER_CENTER_POSITION_Y + CYLINDER_HEIGHT / 2) ||
		(pos2.y - otherR <= CYLINDER_CENTER_POSITION_Y - CYLINDER_HEIGHT / 2))
	{
		newOtherVelY = -1.0 * newOtherVelY;
	}

	velX = newVelX;
	velY = newVelY;

	posX = pos1.x;
	posY = pos1.y;

	otherVx = newOtherVelX;
	otherVy = newOtherVelY;

	otherX = pos2.x;
	otherY = pos2.y;

}