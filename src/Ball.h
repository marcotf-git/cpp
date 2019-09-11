#ifndef BALL_H
#define BALL_H

#include <mutex>
#include <deque>
#include <condition_variable>

#include "CylinderObject.h"
#include "Constants.h"

// forward declarations to avoid include cycle
class Wall;

// This is a message queue class for sending messages between threads
template <class T>
class MessageQueue
{
public:
	T receive();
	void send(T&& msg);
	int getSize();

private:
	std::mutex _mutex;
	std::condition_variable _cond;
	std::deque<T> _queue;
};

enum CollisionType {
	wallCollision,
	bottomCollision,
	pistonCollision,
};

struct CollisionData {
	CollisionType type;
	double velY;
};

class Ball : public CylinderObject, public std::enable_shared_from_this<Ball>
{
public:
	// constructor / destructor
	Ball();

	// getters / setters
	void setWalls(std::vector<std::shared_ptr<Wall>> walls) { _walls = walls; }
	void setBalls(std::vector<std::shared_ptr<Ball>> balls) { _balls = balls; }

	void setRadius(double r) { _radius = r; }
	double getRadius() { return _radius; }

	// This method will set a direction, calculating Vx and Vy based on speed and angle
	void setSpecificDirection(double speed, double angle);

	// This method will choose a random direction for the ball
	void setRandomDirection(double speed);

	void setGravity(double ball_gravity) { _ball_gravity = ball_gravity;  }
	void setGasMode(bool gas_mode) { _gas_mode = gas_mode;  }

	// typical behaviour methods
	bool dataIsAvailable();		// inform that there is a message from other thread
	CollisionData receiveMsg();	// receive msg from other thread (msg stored in queue)
	void simulate();		// process ball movements
	void setShutdown();		// set the flag to exit while-loop in simulate()

	// miscellaneous
	std::shared_ptr<Ball> get_shared_this() { return shared_from_this(); }

private:
	// typical behaviour methods
	void play();
	// check if has collided with a wall
	bool checkWallCollision(double nextX, double nextY, std::shared_ptr<Wall> wall);
	// check if has collided with another ball
	bool checkBallCollision(double nextX, double nextY, std::shared_ptr<Ball> ball);
	// receive direct message to shutdown (exit while-loop)
	bool getShutdown();
	
	// member attributes
	double _radius;						// ball radius
	double _ball_gravity;				// gravity
	bool _gas_mode;						// simulation mode
	std::vector<std::shared_ptr<Wall>> _walls;		// walls of the cylinder on which ball is on
	std::vector<std::shared_ptr<Ball>> _balls;		// balls of the cylinder on which ball is on

	MessageQueue<CollisionData> _msgQueue;	// msg queue for communications with main
	std::mutex _mutex;

	bool _shutDown;		// ask ball to end simulation
};

#endif
