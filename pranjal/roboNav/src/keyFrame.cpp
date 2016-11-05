#include "keyFrame.h"

double keyFrame::getLatitude()
{
	return latitude;
}

double keyFrame::getLongitude()
{
	return longitude;
}

double keyFrame::getConfidence()
{
	return confidence;
}

std::vector< keyPoint > keyFrame::getkeyPoints()
{
	return keyPts;
}

cv::Mat keyFrame::getR()
{
	return R;
}

cv::Vec3d keyFrame::getT()
{
	return t;
}

double keyFrame::getFocalLength()
{
	return focalLength;
}

cv::Vec2d keyFrame::getRadialDist()
{
	return radialDist;
}

std::vector< edge* > keyFrame::getEdges()
{
	return edges;
}

void keyFrame::setR(cv::Mat R)
{
	this->R = R;
}

void keyFrame::setT(cv::Vec3d t)
{
	this->t = t;
}

void keyFrame::setLatitude(double latitude)
{
	this->latitude = latitude;
}

void keyFrame::setLongitude(double longitude)
{
	this->longitude = longitude;
}

void keyFrame::setConfidence(double confidence)
{
	this->confidence = confidence;
}

void keyFrame::addEdge(edge* e)
{
	edges.push_back(e);	
}

void keyFrame::addKeyPts(keyPoint k)
{
	keyPts.push_back(k);
}

void keyFrame::setFocalLength(double focalLength)
{
	this->focalLength = focalLength;
}

void keyFrame::setRadialDist(cv::Vec2d radialDist)
{
	this->radialDist = radialDist;
}
