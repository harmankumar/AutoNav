#include "edge.h"

void edge::addEdge( keyFrame *src , keyFrame *dst , const cv::Mat &R , const cv::Vec3d &t)
{
	this->src = src;
	this->dst = dst;
	this->R = R;
	this->t = t;

}



void edge::removeEdge()
{
	src = 0;
	dst = 0;
	R.release();
}



keyFrame* edge::getSrc() const
{
	return src;
}


keyFrame* edge::getDst() const
{
	return dst;
}


cv::Mat edge::getR() const
{
	return R;
} 


cv::Vec3d edge::getT() const
{
	return t;
}


void edge::setSrc(keyFrame *src)
{
	this->src = src;
}


void edge::setDst(keyFrame *dst)
{
	this->dst = dst;
}


void edge::setR(cv::Mat R)
{
	this->R = R;
}


void edge::setT(cv::Vec3d t)
{
	this->t = t;
}


