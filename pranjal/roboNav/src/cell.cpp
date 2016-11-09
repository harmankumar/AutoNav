#include "cell.h"
#include <unordered_map>
// #include <stdio.h>
// <<<<<<< HEAD
#include "externVariables.h"
// =======
// #include "externvariables.cpp"
// >>>>>>> 7daaf8525ac55a73309c26a1807e97809d88d42a

void cell::changeStructureOfMat(const cv:: Mat &plain, std::vector< cv::Mat > &out, int L)
{
	for (int i = 0; i < plain.rows; ++i)
	{
		cv::Mat tmp = plain(cv::Rect(0,i,L,1));
		cv::Mat to_push;
		tmp.copyTo(to_push);
		out.push_back(to_push);
	}
}

cell::cell()
{

}


cell::cell(std::string inputFile)
{

}


cell::~cell()
{

}


void cell::locate(cv::Mat &R, cv::Mat &t, keyFrame * &nearestKeyFrame, std::vector<keyFrame *> nearestKeyFrames, cv::Mat inputImage)
{
	//orb constructor and compute orb features
	cv::Ptr<cv::ORB> orb = cv::ORB::create();
	std::vector<cv::KeyPoint> keypoints;
	cv::Mat descriptor;
	cv::Mat mask;
	orb->detectAndCompute(inputImage, mask, keypoints, descriptor);
	std::vector<cv::Mat> descriptors;
	changeStructureOfMat(descriptor, descriptors, orb->descriptorSize());
	// DBoW2::QueryResults ret;//vector of results
	// vocabDatabase.query(descriptors, ret, MAX_KEY_FRAMES);
	cv::Ptr<cv::DescriptorMatcher> matcher = cv::DescriptorMatcher::create("BruteForce-Hamming");

	// #ifdef DEBUG
	// 	std::cout << "cell::locate function" << std::endl << std::endl;
	// 	std::cout << ret.size() << " matches found from vocab database" << std::endl;
	// #endif

	// vector containing list of world points (with votes) for each image point
	std::vector< std::unordered_map<worldPoint *, int > > votesForPoints;
	votesForPoints.resize(descriptor.rows);

	int matchingFramesFound = 0;
	std:;vector< keyFrame * > possibleKeyFrame,

	for (int i = 0; i<nearestKeyFrames.size(); ++i)
	{
		// DBoW2::Result leader = ret[i];
		// int indexLeader = leader.Id;
		std::vector< keyPoint > toWorldPoints = nearestKeyFrames[i]->getkeyPoints();
		cv::Mat desc(toWorldPoints.size(),orb->descriptorSize(),descriptor.type());
		for (int j = 0; j < toWorldPoints.size(); ++j)
		{
			toWorldPoints[i].getFeatureVector().copyTo(desc(cv::Rect(0, j, orb->descriptorSize(), 1)));
			// for (int j = 0; j < orb->descriptorSize(); ++j)
			// {
			// 	desc.at<>(i,j) = toWorldPoints[i].getFeatureVector().at<>(0,j);
			// }
		}
		#ifdef DEBUG
			std::cout << "Found image with frame index " << i << " and with " << desc.rows << " keyPoints" << std::endl;
		#endif
		// std::vector<cv::KeyPoint> keypointsTrain;
		// keypointsTrain.reserve(toWorldPoints.size());
		// for (unsigned int i = 0; i < toWorldPoints.size(); ++i)
		// {
		// 	keypointsTrain[i].pt.x = (float)toWorldPoints.getCoordinate().x;
		// 	keypointsTrain[i].pt.y = (float)toWorldPoints.getCoordinate().y;
		// }

		//matching
		std::vector<cv::DMatch> matches;
	    std::vector<cv::Point2d> matched1, matched2;
		matcher->match(descriptor,desc,matches);
		for (int j = 0; j < matches.size(); ++j)
		{
			//multiply point coordinate by K inverse
			cv::Point2d to_push;
			to_push.x = (keypoints[matches[j].queryIdx].pt.x-cx)/fx;
			to_push.y = (keypoints[matches[j].queryIdx].pt.y-cy)/fy;
			matched1.push_back(to_push);
			matched2.push_back(toWorldPoints[matches[j].trainIdx].getCoordinate());
		}
		cv::Mat essential_matrix;
		double percentageInlier = 0.0;
		std::vector<bool> inliers;
		inliers.resize(matches.size());
		std::fill(inliers.begin(),inliers.end(),0);
		if(matches.size() > 10)
		{
			//equivalent to essential matrix estimation
			essential_matrix = cv::findFundamentalMat(matched1, matched2, cv::FM_RANSAC, 3, 0.99, inliers);
			for (int k = 0; k < matches.size(); ++k)
			{
				percentageInlier += (inliers[k]) ? 1.0 : 0.0;
				percentageInlier /= ((descriptor.rows+desc.rows)/2);
			}
		}
		#ifdef DEBUG
			std::cout << "keyPoints in input image = " << descriptor.rows << std::endl;
			std::cout << "keyPoints in plausible image = " << desc.rows << std::endl;
			std::cout << "inlier ratio = " << percentageInlier << std::endl;
			std::cout << "And essential matrix = \n" << essential_matrix << std::endl;
		#endif
		if(essential_matrix.rows == 3 && essential_matrix.cols == 3 && percentageInlier > INLIER_RATIO)
		{
			for (int j = 0; j < matches.size(); ++j)
			{
				//vote for corresponding keypoints and find most likely world point
				if(inliers[j])
				{
					if(votesForPoints[matches[j].queryIdx].find(toWorldPoints[matches[j].trainIdx].get3DLink()) == votesForPoints[matches[j].queryIdx].end())
					{
						votesForPoints[matches[j].queryIdx][toWorldPoints[matches[j].trainIdx].get3DLink()] = 1;
					}
					else
					{
						votesForPoints[matches[j].queryIdx][toWorldPoints[matches[j].trainIdx].get3DLink()] += 1;
					}
				}
			}
			possibleKeyFrame.push_back(nearestKeyFrames[i]);
			++matchingFramesFound;
		}
	}

	if(matchingFramesFound == 0)
	{
		return;
	}

	#ifdef DEBUG
		for (int i = 0; i < votesForPoints.size(); ++i)
		{
			std::cout << "Descriptor/Point ID " << i << " has " << votesForPoints[i].size() << " world points corresponding to it" << std::endl;
			for (std::unordered_map<worldPoint *, int >::iterator it = votesForPoints[i].begin(); it != votesForPoints[i].end(); ++it)
			{
				std::cout << "Votes to world points : " << it->second << std::endl;
			}
			std::cout << std::endl;
		}
	#endifcv::Mat &R, cv::Mat &t

	//pick the world points corresponding to the maximum votes
	std::vector< cv::Point2f > imagePoints;
	std::vector< cv::Point3f > objectPoints;
	for (int i = 0; i < votesForPoints.size(); ++i)
	{
		if(votesForPoints[i].size() > 0)
		{
			std::unordered_map<worldPoint *, int>::iterator maxIt = votesForPoints[i].begin();
			int votes = 0;
			for (std::unordered_map<worldPoint *, int>::iterator it = votesForPoints[i].begin(); it != votesForPoints[i].end(); ++it)
			{
				if(maxIt->second < it->second)
				{
					maxIt = it;
				}
				votes += it->second;
			}
			if(maxIt->second > votes/2)
			{
				imagePoints.push_back( cv::Point2f((keypoints[i].pt.x-cx)/fx,(keypoints[i].pt.y-cy)/fy));
				objectPoints.push_back( cv::Point3f(maxIt->first->location[0],maxIt->first->location[1],maxIt->first->location[2]));
			}
		}
	}


	// if(objectPoints.size() <= MIN_WORLD_CORRESPONDENCES)
	// {
	// 	return;
	// }

	assert(objectPoints.size() > MIN_WORLD_CORRESPONDENCES);

	//get R and t of the camera
	cv::Mat cameraMatrix(3,3,cv::DataType<double>::type);
	//identity because coordinates already camera centered
	cv::setIdentity(cameraMatrix);


	cv::Mat distCoeffs(4,1,cv::DataType<double>::type);
	distCoeffs.at<double>(0) = 0;
	distCoeffs.at<double>(1) = 0;
	distCoeffs.at<double>(2) = 0;
	distCoeffs.at<double>(3) = 0;

	cv::Mat rvec(3,1,cv::DataType<double>::type);
	cv::Mat tvec(3,1,cv::DataType<double>::type);

	cv::solvePnP(objectPoints, imagePoints, cameraMatrix, distCoeffs, rvec, tvec);

	cv::Mat Rtemp(3,3,cv::DataType<double>::type);
	cv::Rodrigues(rvec,Rtemp);

	cv::transpose( Rtemp, R);

	t = -R*tvec;

	double minDistance = (possibleKeyFrame[0]->getT()[0]-t.at<double>(0,0))*(possibleKeyFrame[0]->getT()[0]-t.at<double>(0,0));
	minDistance += (possibleKeyFrame[0]->getT()[1]-t.at<double>(1,0))*(possibleKeyFrame[0]->getT()[1]-t.at<double>(1,0));
	minDistance += (possibleKeyFrame[0]->getT()[2]-t.at<double>(2,0))*(possibleKeyFrame[0]->getT()[2]-t.at<double>(2,0));
	nearestKeyFrame = possibleKeyFrame[0];

	for (int i = 1; i < possibleKeyFrame.size(); ++i)
	{
		double tempDistance = (possibleKeyFrame[i]->getT()[0]-t.at<double>(0,0))*(possibleKeyFrame[i]->getT()[0]-t.at<double>(0,0));
		tempDistance += (possibleKeyFrame[i]->getT()[1]-t.at<double>(1,0))*(possibleKeyFrame[i]->getT()[1]-t.at<double>(1,0));
		tempDistance += (possibleKeyFrame[i]->getT()[2]-t.at<double>(2,0))*(possibleKeyFrame[i]->getT()[2]-t.at<double>(2,0));
		if(tempDistance < minDistance)
		{
			minDistance = tempDistance;
			nearestKeyFrame = possibleKeyFrame[i];
		}
	}
}


void cell::locate(cv::Mat &R, cv::Mat &t, keyFrame * &nearestKeyFrame, cv::Mat inputImage)
{
	//orb constructor and compute orb features
	cv::Ptr<cv::ORB> orb = cv::ORB::create();
	std::vector<cv::KeyPoint> keypoints;
	cv::Mat descriptor;
	cv::Mat mask;
	orb->detectAndCompute(inputImage, mask, keypoints, descriptor);
	std::vector<cv::Mat> descriptors;
	changeStructureOfMat(descriptor, descriptors, orb->descriptorSize());
	DBoW2::QueryResults ret;//vector of results
	vocabDatabase.query(descriptors, ret, MAX_KEY_FRAMES);

	#ifdef DEBUG
		std::cout << "cell::locate function" << std::endl << std::endl;
		std::cout << ret.size() << " matches found from vocab database" << std::endl;
	#endif

	// cv::Ptr<cv::DescriptorMatcher> matcher = cv::DescriptorMatcher::create("BruteForce-Hamming");
	
	// // vector containing list of world points (with votes) for each image point
	// std::vector< std::unordered_map<worldPoint *, int > > votesForPoints;
	// votesForPoints.resize(descriptor.rows);

	std::vector<keyFrame *> nearestKeyFrames;

	for (int i = 0; i<ret.size(); ++i)
	{
		DBoW2::Result leader = ret[i];
		int indexLeader = leader.Id;
		nearestKeyFrames.push_back(keyFrames[indexLeader]);
		// std::vector< keyPoint > toWorldPoints = keyFrames[indexLeader]->getkeyPoints();
		// cv::Mat desc(toWorldPoints.size(),orb->descriptorSize(),descriptor.type());
		// for (int j = 0; j < toWorldPoints.size(); ++j)
		// {
		// 	toWorldPoints[i].getFeatureVector().copyTo(desc(cv::Rect(0, j, orb->descriptorSize(), 1)));
		// 	// for (int j = 0; j < orb->descriptorSize(); ++j)
		// 	// {
		// 	// 	desc.at<>(i,j) = toWorldPoints[i].getFeatureVector().at<>(0,j);
		// 	// }
		// }
		// #ifdef DEBUG
		// 	std::cout << "Found image with ID " << indexLeader << " and with " << desc.rows << " keyPoints" << std::endl;
		// #endif
		// // std::vector<cv::KeyPoint> keypointsTrain;
		// // keypointsTrain.reserve(toWorldPoints.size());
		// // for (unsigned int i = 0; i < toWorldPoints.size(); ++i)
		// // {
		// // 	keypointsTrain[i].pt.x = (float)toWorldPoints.getCoordinate().x;
		// // 	keypointsTrain[i].pt.y = (float)toWorldPoints.getCoordinate().y;
		// // }

		// //matching
		// std::vector<cv::DMatch> matches;
	 //    std::vector<cv::Point2d> matched1, matched2;
		// matcher->match(descriptor,desc,matches);
		// for (int j = 0; j < matches.size(); ++j)
		// {
		// 	//multiply point coordinate by K inverse
		// 	cv::Point2d to_push;
		// 	to_push.x = (keypoints[matches[j].queryIdx].pt.x-cx)/fx;
		// 	to_push.y = (keypoints[matches[j].queryIdx].pt.y-cy)/fy;
		// 	matched1.push_back(to_push);
		// 	matched2.push_back(toWorldPoints[matches[j].trainIdx].getCoordinate());
		// }
		// cv::Mat essential_matrix;
		// double percentageInlier = 0.0;
		// std::vector<bool> inliers;
		// inliers.resize(matches.size());
		// std::fill(inliers.begin(),inliers.end(),0);
		// if(matches.size() > 10)
		// {
		// 	//equivalent to essential matrix estimation
		// 	essential_matrix = cv::findFundamentalMat(matched1, matched2, cv::FM_RANSAC, 3, 0.99, inliers);
		// 	for (int k = 0; k < matches.size(); ++k)
		// 	{
		// 		percentageInlier += (inliers[k]) ? 1.0 : 0.0;
		// 		percentageInlier /= ((descriptor.rows+desc.rows)/2);
		// 	}
		// }
		// #ifdef DEBUG
		// 	std::cout << "keyPoints in input image = " << descriptor.rows << std::endl;
		// 	std::cout << "keyPoints in plausible image = " << desc.rows << std::endl;
		// 	std::cout << "inlier ratio = " << percentageInlier << std::endl;
		// 	std::cout << "And essential matrix = \n" << essential_matrix << std::endl;
		// #endif
		// if(essential_matrix.rows == 3 && essential_matrix.cols == 3 && percentageInlier > INLIER_RATIO)
		// {
		// 	for (int j = 0; j < matches.size(); ++j)
		// 	{
		// 		//vote for corresponding keypoints and find most likely world point
		// 		if(inliers[j])
		// 		{
		// 			if(votesForPoints[matches[j].queryIdx].find(toWorldPoints[matches[j].trainIdx].get3DLink()) == votesForPoints[matches[j].queryIdx].end())
		// 			{
		// 				votesForPoints[matches[j].queryIdx][toWorldPoints[matches[j].trainIdx].get3DLink()] = 1;
		// 			}
		// 			else
		// 			{
		// 				votesForPoints[matches[j].queryIdx][toWorldPoints[matches[j].trainIdx].get3DLink()] += 1;
		// 			}
		// 		}
		// 	}
		// }
	}

	// #ifdef DEBUG
	// 	for (int i = 0; i < votesForPoints.size(); ++i)
	// 	{
	// 		std::cout << "Descriptor/Point ID " << i << " has " << votesForPoints[i].size() << " world points corresponding to it" << std::endl;
	// 		for (std::unordered_map<worldPoint *, int >::iterator it = votesForPoints[i].begin(); it != votesForPoints[i].end(); ++it)
	// 		{
	// 			std::cout << "Votes to world points : " << it->second << std::endl;
	// 		}
	// 		std::cout << std::endl;
	// 	}
	// #endif

	// //pick the world points corresponding to the maximum votes
	// std::vector< cv::Point2f > imagePoints;
	// std::vector< cv::Point3f > objectPoints;
	// for (int i = 0; i < votesForPoints.size(); ++i)
	// {
	// 	if(votesForPoints[i].size() > 0)
	// 	{
	// 		std::unordered_map<worldPoint *, int>::iterator maxIt = votesForPoints[i].begin();
	// 		int votes = 0;
	// 		for (std::unordered_map<worldPoint *, int>::iterator it = votesForPoints[i].begin(); it != votesForPoints[i].end(); ++it)
	// 		{
	// 			if(maxIt->second < it->second)
	// 			{
	// 				maxIt = it;
	// 			}
	// 			votes += it->second;
	// 		}
	// 		if(maxIt->second > votes/2)
	// 		{
	// 			imagePoints.push_back( cv::Point2f((keypoints[i].pt.x-cx)/fx,(keypoints[i].pt.y-cy)/fy));
	// 			objectPoints.push_back( cv::Point3f(maxIt->first->location[0],maxIt->first->location[1],maxIt->first->location[2]));
	// 		}
	// 	}
	// }

	// //get R and t of the camera
	// cv::Mat cameraMatrix(3,3,cv::DataType<double>::type);
	// cv::setIdentity(cameraMatrix);


	// cv::Mat distCoeffs(4,1,cv::DataType<double>::type);
	// distCoeffs.at<double>(0) = 0;
	// distCoeffs.at<double>(1) = 0;
	// distCoeffs.at<double>(2) = 0;
	// distCoeffs.at<double>(3) = 0;

	// cv::Mat rvec(3,1,cv::DataType<double>::type);
	// cv::Mat tvec(3,1,cv::DataType<double>::type);

	// cv::solvePnP(objectPoints, imagePoints, cameraMatrix, distCoeffs, rvec, tvec);

	// cv::Mat Rtemp(3,3,cv::DataType<double>::type);
	// cv::Rodrigues(rvec,Rtemp);

	// cv::transpose( Rtemp, R);

	// t = -R*tvec;

	locate( R, t, nearestKeyFrame, nearestKeyFrames, inputImage);

}


bool cell::track(keyFrame * frameToTrack, cv::Mat inputImage)
{
	cv::Mat R(3,3,cv::DataType<double>::type);
	cv::Mat t(3,1,cv::DataType<double>::type);

	R.at<double>(0,0) = 0;
	R.at<double>(0,1) = 0;
	R.at<double>(0,2) = 0;

	std::vector<keyFrame *> nearestKeyFrames;
	nearestKeyFrames.push_back(frameToTrack);

	keyFrame * nearestKeyFrame,

	locate( R, t, nearestKeyFrame, nearestKeyFrames, inputImage);

	if(R.at<double>(0,0) == 0 && R.at<double>(0,1) == 0 && R.at<double>(0,2) == 0)
	{
		return 0;
	}
	else
	{
		return 1;
	}
}

std::vector<cv::Point2d> cell::path(cv::Mat &R, cv::Mat &t, cv::Mat &RDest, cv::Mat &tDest)
{

}

void cell::save()
{

}

void cell::loadNVM(std::string filename)
{
	std::ifstream f(filename);
	std::string valString;
	int size;
	double valDouble;
	f>>valString;
	f>>size;
	
	//Start Reading for KeyFrames
	keyFrames.resize(size);
	for (int i = 0; i < size; ++i)
	{
		f>>valString;
		keyFrames[i] = new keyFrame();		
		cv::Mat R( 3 , 3 , CV_64F, 0.0);
		
		for (int j = 0; j < 3; ++j)
		{
			for (int k = 0; k < 3; ++k)
			{
				f>>valDouble;
				R.at<double>(k,j) = valDouble;
			}
		}
		
		keyFrames[i]->setR(R);
		cv::Vec3d t;

		for (int j = 0; j < 3; ++j)
		{
			f>>t[j];
		}
		keyFrames[i]->setT(t);

		f>>valDouble;
		keyFrames[i]->setFocalLength(valDouble);

		cv::Vec2d radialDist;
		for (int j = 0; j < 2; ++j)
		{
			f>>radialDist[j];
		}
		keyFrames[i]->setRadialDist(radialDist);
	}
	std::cout<<"KeyFrames Read : "<<keyFrames.size()<<std::endl;
	//Stop Reading for KeyFrames


	//Create Edges between successive frames
	edges.reserve(keyFrames.size()-1);
	for (int i = 1; i < keyFrames.size(); ++i)
	{
		edge *e = new edge();
		e->setSrc(keyFrames[i-1]);
		e->setDst(keyFrames[i]);
		cv::Mat R1,R2;
		R1 = keyFrames[i-1]->getR();
		R2 = keyFrames[i]->getR();
		// R = inv(R1)*R2
		cv::Mat R = R1.t()*R2;
		e->setR(R);
		// t = c2-c1
		cv::Mat t12 = -R2.t()*cv::Mat(keyFrames[i]->getT()) - (-R1.t()*cv::Mat( keyFrames[i-1]->getT()));
		e->setT( cv::Vec3d(t12) );
		edges.push_back(e);
		keyFrames[i-1]->addEdge(e);
		keyFrames[i]->addEdge(e);
	}
	std::cout<<"Edges created : "<<edges.size()<<std::endl;

	//Stop Creating Edges


	f>>size;
	threeDPts.resize(size);

	//Start Reading for KeyPoints
	for (int i = 0; i < size; ++i)
	{
		for (int j = 0; j < 3; ++j)
			f>>threeDPts[i].location[j];
		
		for (int j = 0; j < 3; ++j)
			f>>threeDPts[i].rgb[j];
		
		f>>size;

		for (int j = 0; j < size; ++j)
		{
			int frameId , featureVecId;
			cv::Point2d coordinate;
			keyPoint k;

			f>>frameId>>featureVecId;
			f>>coordinate.x>>coordinate.y;

			k.setFeatueVecId(featureVecId);
			k.setCoordinate(coordinate);
			k.setLinkto3D( &threeDPts[i] );

			keyFrames[i]->addKeyPts(k);

		}	
	}
	std::cout<<"KeyPoints Read : "<<threeDPts.size()<<std::endl;
	//Stop Reading for KeyFrames

}

