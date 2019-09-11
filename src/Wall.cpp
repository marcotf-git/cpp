#include "Wall.h"

Wall::Wall()
{
	_type = ObjectType::objectWall;
	_wallType = WallType::lateral;
	_width = 0;
	_height = 0;
	_temp = 0;
}

void  Wall::setSize(double w, double h)
{
	_width = w;
	_height = h;
}

void  Wall::getSize(double& w, double& h)
{
	w = _width;
	h = _height;
}

void Wall::setHeat(double ht)
{ 
	std::lock_guard<std::mutex> lock(_mutex);
	_temp = ht; 
}

double Wall::getHeat()
{ 
	std::lock_guard<std::mutex> lock(_mutex);
	return _temp; 
}