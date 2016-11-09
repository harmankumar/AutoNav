#include <iostream>
// #include "../include/keyFrame.h"
#include "cell.h"
#include "TemplatedDatabase.h"
#include "TemplatedVocabulary.h"
#include "DBoW2.h"
#include <fstream>
// #include "include/edge.h"
// #include "include/keyPoint.h"


int main(int argc, char const *argv[])
{
	// keyFrame<int> *k1 = new keyFrame<int> ;
	// keyFrame<int> *k2 = new keyFrame<int> ;

	// // keyPoint<int> k;
	// // std::cout<<k.getX()<<std::endl;
	// // std::cout<<k.getLatitude()<<std::endl;
	// edge< keyFrame<int> > *e = new edge< keyFrame<int> > ;
	// cv::Mat R( 3 , 3 , CV_64F, 0.0);//  = cv::imread("/home/shashank/Pictures/lena.jpeg", CV_LOAD_IMAGE_COLOR);
	
	// e->setR(R);

	// e->setSrc(k1);
	// e->setDst(k2);
	
	// k1->addEdge(e);
	// k2->addEdge(e);

	// k1->setR(R);
	// k1->setT( cv::Vec3d(1,1,1) );

	// cell<int> c;

	// c.keyFrames.push_back(k1);
	// c.keyFrames.push_back(k2);
	// c.edges.push_back(e);

	// cv::Vec3d *v = new cv::Vec3d(1,2,3);
	// c.threeDPts.push_back(v);

	// v = new cv::Vec3d(2,2,2);
	// c.threeDPts.push_back(v);


	// c.writeCell("kfs","edges","threeD");
	
	// cell c;
	// c.readCell("kfs","edges","threeD");
	// std::cout<<*(c.threeDPts[1])<<"\n";
	// std::cout<<c.keyFrames.size()<<"\n";
	// std::cout<<c.edges.size()<<"\n";

	// std::cout<<c.edges[0]->getR();
	// std::cout<<c.keyFrames[0]->getT();



	// std::cout<<e.getSrc()<<std::endl;

	// std::ofstream o("data");
	// o<<e;

	// std::ifstream f("data");
	// f>>e;

	// std::cout<<e;

	#ifdef DEBUG
		std::cout << "This is debugging mode on " << std::endl;
		cell c;
		c.loadNVM("../data/outputVSFM_GB.nvm");
		keyFrame *k = c.keyFrames[0];
		std::cout<<k->getR()<<std::endl;
		// std::cout<<c.edges.size()<<std::endl;
		edge *e = c.edges[0];
		std::cout<<e->getT()<<std::endl;
	#endif
		std::cout << "hi just exiting " << std::endl;

	return 0;
}