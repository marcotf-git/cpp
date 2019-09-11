#ifndef WALL_H
#define WALL_H

#include "CylinderObject.h"

enum WallType
{
	lateral,
	piston,
	bottom,
};

class Wall : public CylinderObject, public std::enable_shared_from_this<Wall>
{
public:
	// constructor / destructor
	Wall();

	// getters / setters
	// size (width and height)
	void setSize(double w, double h);
	void getSize(double& w, double& h);
	// heat or temperature
	void setHeat(double ht);
	double getHeat();
	// type of the wall: lateral, piston, bottom
	void setWallType(WallType type) { _wallType = type; }
	WallType getWallType() { return _wallType; }

	// miscellaneous
	std::shared_ptr<Wall> get_shared_this() { return shared_from_this(); }

private:

	// member attributes
	double _width, _height;		// object width and height in pixels
	double _temp;				// wall temperature
	WallType _wallType;			// wall type
};


#endif
