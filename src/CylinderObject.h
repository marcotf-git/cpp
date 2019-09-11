#ifndef CylinderObject_H
#define CylinderObject_H

#include <vector>
#include <thread>
#include <mutex>

enum ObjectType
{
	noObject,
	objectBall,
	objectWall,
};

class CylinderObject
{
public:
	// constructor / destructor
	CylinderObject();
	~CylinderObject();

	// getters / setters
	int getID() { return _id; };
	// position of object on canvas (Ball, Wall, or Piston)
	void setPosition(double x, double y);
	void getPosition(double& x, double& y);
	// velocity of object
	void setVelocity(double velX, double velY);
	void getVelocity(double& velX, double& velY);
	// speed of object
	double getSpeed();
	// mass of object
	void setMass(double m);
	double getMass();
	ObjectType getType() { return _type; }

	// typical behaviour methods
	virtual void simulate() {};		// This will be implemented by the Ball class

protected:
	// member attributes
	ObjectType	_type;					// identifies the class type
	int _id;							// every cylinder object has its own id
	double	_posX, _posY;				// object position in pixels (center)
	double _velX, _velY, _speed;		// velocity and speed in pixels/s
	double _mass;						// object mass

	std::vector<std::thread> _threads;	// holds all threads that have been launched within this object
	std::mutex	_mutex;					// mutex to protect member attributes
	static std::mutex _mtxCout;			// mutex shared by all cylinder objects for protecting cout
private:
	// member attributes
	static int _idCnt;					// global variable for counting ids

};

#endif 
