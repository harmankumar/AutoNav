#include "keyPoint.h"

int keyPoint::getFeatureVecId()
{
// <<<<<<< HEAD
// 	f<<coordinate.x<<' '<<coordinate.y<<' '<<id<<' ';
// }

// void keyPoint::readKP( std::ifstream &f , std::unordered_map<int,cv::Vec3d*> dict3DPoints)
// {
// 	int id;
// 	f>>coordinate.x>>coordinate.y;
// 	f>>id;
// 	linkTo3D = dict3DPoints[id];
// =======
	return featureVecId;
// >>>>>>> 7daaf8525ac55a73309c26a1807e97809d88d42a
}

cv::Mat keyPoint::getFeatureVector()
{
	return featureVec;
}

// <<<<<<< HEAD
// // int keyPoint::getX()
// // {
// // 	return x;
// // }

// // int keyPoint::getY()
// // {
// // 	return y;
// // }

// cv::Point2d keyPoint::getCoordinate()
// {
// 	return coordinate;
// =======
cv::Point2d keyPoint::getCoordinate()
{
	return coordinate;
}

worldPoint* keyPoint::get3DLink()
{
	return linkTo3D;
// >>>>>>> 7daaf8525ac55a73309c26a1807e97809d88d42a
}

keyFrame* keyPoint::getParent()
{
	return parent;
}

void keyPoint::init( int featureVecId , cv::Point2d coordinate , cv::Mat featureVec , worldPoint* linkTo3D , keyFrame *parent)
{
	this->featureVecId = featureVecId;
// <<<<<<< HEAD
	// coordinate.x = x;
	// coordinate.y = y;
// =======
	this->coordinate = coordinate;
// >>>>>>> 7daaf8525ac55a73309c26a1807e97809d88d42a
	this->featureVec = featureVec;
	this->linkTo3D = linkTo3D;
	this->parent = parent;
}

void keyPoint::setCoordinate(cv::Point2d coordinate)
{
// <<<<<<< HEAD
// 	coordinate.x = x;
// 	coordinate.y = y;
// =======
	this->coordinate = coordinate;
// >>>>>>> 7daaf8525ac55a73309c26a1807e97809d88d42a
}

void keyPoint::setFeatueVec(cv::Mat featureVec)
{
	this->featureVec = featureVec;
}


void keyPoint::setFeatueVecId(int featureVecId)
{
	this->featureVecId = featureVecId;
}

void keyPoint::setLinkto3D( worldPoint* linkTo3D )
{
	this->linkTo3D = linkTo3D;
}

void keyPoint::setParent(keyFrame *parent)
{
	this->parent = parent;
}

