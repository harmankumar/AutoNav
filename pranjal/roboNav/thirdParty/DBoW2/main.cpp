/**
 * File: Demo.cpp
 * Date: November 2011
 * Author: Dorian Galvez-Lopez
 * Description: demo application of DBoW2
 * License: see the LICENSE.txt file
 */

#include <iostream>
#include <vector>

// DBoW2
#include "DBoW2.h" // defines Surf64Vocabulary and Surf64Database

#include "TemplatedVocabulary.h"
#include "TemplatedDatabase.h"
#include "BowVector.h"
#include "FeatureVector.h"
#include "QueryResults.h"
// #include "FSurf64.h"
// #include "FBrief.h"
#include "FORB.h"

// #include <DUtils/DUtils.h>
// #include <DVision/DVision.h>

// OpenCV
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/xfeatures2d/nonfree.hpp>
#include <opencv2/features2d.hpp>


using namespace DBoW2;
using namespace DUtils;
using namespace std;

template<class T>
class wrapper
{
public:
	wrapper(){};
	wrapper(int k, int L, WeightingType weight, ScoringType score){};//vocab(k, L, weight, score);}
	~wrapper(){};
	DBoW2::TemplatedVocabulary<typename T::TDescriptor, T> vocab;
	
};

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 

void loadFeatures(vector< vector< cv::Mat > > &features);
void changeStructure(const vector<float> &plain, vector<vector<float> > &out,
  int L);
void cs(const cv:: Mat &plain, vector< cv::Mat > &out,
  int L);
void testVocCreation(vector< vector< cv::Mat > > &features);
void testDatabase(const vector< vector< cv::Mat > > &features);
// void testVocCreation(const vector<vector<vector<float> > > &features);
// void testDatabase(const vector<vector<vector<float> > > &features);

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 

// number of training images
const int NIMAGES = 4;

// extended surf gives 128-dimensional vectors
const bool EXTENDED_SURF = false;

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 

void wait()
{
  cout << endl << "Press enter to continue" << endl;
  getchar();
}

// ----------------------------------------------------------------------------

int main()
{
  vector< vector< cv::Mat > > features;
  loadFeatures(features);

  testVocCreation(features);

  wait();

  testDatabase(features);

  return 0;
}

// ----------------------------------------------------------------------------

// void loadFeatures(vector<vector<vector<float> > > &features)
// {
//   features.clear();
//   features.reserve(NIMAGES);

//   cv::Ptr<cv::xfeatures2d::SURF> surf = cv::xfeatures2d::SURF::create(400, 4, 2, EXTENDED_SURF);

//   cout << "Extracting SURF features..." << endl;
//   for(int i = 0; i < NIMAGES; ++i)
//   {
//     stringstream ss;
//     ss << "images/image" << i << ".png";

//     cv::Mat image = cv::imread(ss.str(), 0);
//     cv::Mat mask;
//     vector<cv::KeyPoint> keypoints;
//     vector<float> descriptors;

//     surf->detectAndCompute(image, mask, keypoints, descriptors);

//     features.push_back(vector<vector<float> >());
//     changeStructure(descriptors, features.back(), surf->descriptorSize());
//   }
// }

void loadFeatures(vector< vector< cv::Mat > > &features)
{
  features.clear();
  features.reserve(NIMAGES);

  cv::Ptr<cv::ORB> orb = cv::ORB::create();

  cout << "Extracting SURF features..." << endl;
  for(int i = 0; i < NIMAGES; ++i)
  {
    stringstream ss;
    ss << "images/image" << i << ".png";

    cv::Mat image = cv::imread(ss.str(), 0);
    cv::Mat mask;
    vector<cv::KeyPoint> keypoints;
    cv::Mat descriptors;

    orb->detectAndCompute(image, mask, keypoints, descriptors);

    features.push_back(vector< cv::Mat >());

    cs(descriptors, features.back(), orb->descriptorSize());
  }
}

// ----------------------------------------------------------------------------

void cs(const cv:: Mat &plain, vector< cv::Mat > &out,
  int L)
{
	// out.resize(plain.rows);

	for (int i = 0; i < plain.rows; ++i)
	{
		cv::Mat tmp = plain(cv::Rect(0,i,L,1));
		cv::Mat to_push;
		tmp.copyTo(to_push);
		out.push_back(to_push);
	}
}

// ----------------------------------------------------------------------------

void changeStructure(const vector<float> &plain, vector<vector<float> > &out,
  int L)
{
  out.resize(plain.size() / L);

  unsigned int j = 0;
  for(unsigned int i = 0; i < plain.size(); i += L, ++j)
  {
    out[j].resize(L);
    std::copy(plain.begin() + i, plain.begin() + i + L, out[j].begin());
  }
}

// ----------------------------------------------------------------------------

void testVocCreation(vector< vector< cv::Mat > > &features)
{
  // branching factor and depth levels 
  const int k = 9;
  const int L = 3;
  const WeightingType weight = TF_IDF;
  const ScoringType score = L1_NORM;

  wrapper<DBoW2::FORB> temp(k, L, weight, score);

  DBoW2::TemplatedVocabulary<DBoW2::FORB::TDescriptor, DBoW2::FORB> voc(k, L, weight, score);

  temp.vocab = voc;

  // voc.lure();

  cout << "Creating a small " << k << "^" << L << " vocabulary..." << endl;
  vector< cv::Mat > removed = features.back();
  features.pop_back();
  // voc.create(features);
  temp.vocab.create(features);
  features.push_back(removed);
  cout << "... done!" << endl;

  cout << "Vocabulary information: " << endl
  << temp.vocab << endl << endl;

  // lets do something with this vocabulary
  // cout << "Matching images against themselves (0 low, 1 high): " << endl;
  // BowVector v1, v2;
  // for(int i = 0; i < NIMAGES; i++)
  // {
  //   voc.transform(features[i], v1);
  //   for(int j = 0; j < NIMAGES; j++)
  //   {
  //     voc.transform(features[j], v2);
      
  //     double score = voc.score(v1, v2);
  //     cout << "Image " << i << " vs Image " << j << ": " << score << endl;
  //   }
  // }

  // save the vocabulary to disk
  cout << endl << "Saving vocabulary..." << endl;
  // voc.save("small_voc.yml.gz");
  temp.vocab.save("small_voc.yml.gz");
  cout << "Done" << endl;
}

// ----------------------------------------------------------------------------

void testDatabase(const vector< vector< cv::Mat > > &features)
{
  cout << "Creating a small database..." << endl;

  // load the vocabulary from disk
  FORBVocabulary voc("small_voc.yml.gz");
  
  FORBDatabase db(voc, true, 0); // false = do not use direct index
  // (so ignore the last param)
  // The direct index is useful if we want to retrieve the features that 
  // belong to some vocabulary node.
  // db creates a copy of the vocabulary, we may get rid of "voc" now

  // add images to the database
  for(int i = 0; i < 3; i++)
  {
    db.add(features[i]);
  }

  cout << "... done!" << endl;

  cout << "Database information: " << endl << db << endl;

  // and query the database
  cout << "Querying the database: " << endl;

  QueryResults ret;
  for(int i = 0; i < NIMAGES; i++)
  {
    db.query(features[i], ret, 4);

    // ret[0] is always the same image in this case, because we added it to the 
    // database. ret[1] is the second best match.

    cout << "Searching for Image " << i << ". " << ret << endl;
  }

  cout << endl;

  // we can save the database. The created file includes the vocabulary
  // and the entries added
  cout << "Saving database..." << endl;
  db.save("small_db.yml.gz");
  cout << "... done!" << endl;
  
  // // once saved, we can load it again  
  // cout << "Retrieving database once again..." << endl;
  // Surf64Database db2("small_db.yml.gz");
  // cout << "... done! This is: " << endl << db2 << endl;
}

// ----------------------------------------------------------------------------

