#include <algorithm>

#include "CylinderObject.h"

// init static variables
int CylinderObject::_idCnt = 0;
std::mutex CylinderObject::_mtxCout;

CylinderObject::CylinderObject()
{
	_type = ObjectType::noObject;
	_id = _idCnt;
	_posX = 0;
	_posY = 0;
	_velX = 0;
	_velY = 0;
	_speed = 0;
	_mass = 0;
	_idCnt++;
}

void CylinderObject::setPosition(double x, double y) 
{
	std::lock_guard<std::mutex> lock(_mutex);
	_posX = x;
	_posY = y;
}

void  CylinderObject::getPosition(double& x, double& y)
{
	std::lock_guard<std::mutex> lock(_mutex);
	x = _posX;
	y = _posY;
}

void CylinderObject::setVelocity(double velX, double velY)
{
	std::lock_guard<std::mutex> lock(_mutex);
	_velX = velX;
	_velY = velY;
	_speed = sqrt(pow(velX, 2) + pow(velY, 2));
}

void CylinderObject::getVelocity(double& velX, double& velY)
{
	std::lock_guard<std::mutex> lock(_mutex);
	velX = _velX;
	velY = _velY;
}

double CylinderObject::getSpeed()
{
	std::lock_guard<std::mutex> lock(_mutex);
	_speed = sqrt(pow(_velX, 2) + pow(_velY, 2));
	return _speed;
}

void CylinderObject::setMass(double mass)
{
	_mass = mass;
}

double CylinderObject::getMass()
{
	return _mass;
}

CylinderObject::~CylinderObject()
{
	// set up thread barrier before this object is destroyed
	std::for_each(_threads.begin(), _threads.end(), [](std::thread& t) {
		t.join();
	});
}
