#include <bits/stdc++.h>
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <lcm/lcm-cpp.hpp>
#include "exlcm/example_t.hpp"

using namespace cv;
using namespace std;

vector<floorpoint> floorPoints;
vector<floorpoint> floorPointsCoarse;
vector<floorpoint> boundaryPoints;
const int WindowSize = 200;
const int WindowSizeCoarse = 500;
const int StepSize = 40;
const int FLOOR_DIFF_THRES = 15;
const int FLOOR_COLOR_THRES = 100;
const float EMPTY_THRES = 0.05;

map<floorpoint, int> ComponentNumber;
int numComponents = 0;

Mat img;
int imagewidth, imageheight;
int currImage;

class Handler
 {
     public:
         ~Handler() {}

        void handleMessage(const lcm::ReceiveBuffer* rbuf,
                const std::string& chan,
                const exlcm::example_t* msg)
        {
            currImage = (int)msg->imageId;
        }
};

// dfs through points in image
void dfs(floorpoint currPoint, set<floorpoint>& visited, set<floorpoint>& allPoints) {
    int dx[4] = {0, 0, StepSize, -StepSize};
    int dy[4] = {StepSize, -StepSize, 0, 0};

    // if(visited.count(currPoint))
    //     return;

    ComponentNumber[currPoint] = numComponents;
    visited.insert(currPoint);

    for(int k=0; k<4; k++)
    {
        floorpoint newPoint = make_pair(currPoint.first + dx[k], currPoint.second + dy[k]);

        if(allPoints.count(newPoint) and (!visited.count(newPoint)))
            dfs(newPoint, visited, allPoints);
    }

}


vector<int> boundaryPoints()
{
  // Keep a sliding window of size k and have the minimum of that sliding window
  // The best one (farthest is the direction we move in)

  dequeue<Point>; // Point is some datatype

  for(auto it: input)
  {


  }
}

// Mark connected components in point vector
void mark(vector<floorpoint>& v) {
    set<floorpoint> allPoints;
    set<floorpoint> visited;

    numComponents = 0;

    for(auto it: v)
        allPoints.insert(it);

    for(auto it: allPoints)
    {
        if(visited.count(it))
            continue;
        numComponents++;
        dfs(it, visited, allPoints);
    }
}

// Get yaw angle of bot
double getZAngle(Point point, double imagewidth, double focalLength) {
    double theta = atan((double)(point.x - imagewidth)/focalLength);
    return theta;
}

// Mean point of largest component - to follow
floorpoint getMeanLargestComp(int num) {
    int* numPoints = new int[num+1];
    // Set to 0
    for(int i = 1; i <= num; ++i) {
        numPoints[i] = 0;
    }
    int maxComponent, maxPoints;
    for(auto iterator = ComponentNumber.begin(); iterator != ComponentNumber.end(); iterator++) {
        // cout << iterator->second;
        numPoints[iterator->second]++;
    }

    // Find component having max number of points
    maxPoints = 0;
    for(int i = 1; i <= num; ++i) {
        int numPoint = numPoints[i];
        if(maxPoints < numPoint) {
            maxPoints = numPoint;
            maxComponent = i;
        }
    }
    cout << "Max component: " << maxComponent << endl;
    cout << "Number of points: " << maxPoints << endl;

    // find center of connected component
    double target_x = 0, target_y = 0; int n = 0;
    for(auto iterator = ComponentNumber.begin(); iterator != ComponentNumber.end(); iterator++) {
        if(iterator->second == maxComponent) {
            n++;
            // cout << iterator->first.first << " " << iterator->first.second << endl;
            target_x += iterator->first.first;
            target_y += iterator->first.second;
        }
    }
    cout << n << endl;
    target_x = target_x / numPoints[maxComponent];
    target_y = target_y / numPoints[maxComponent];
    return make_pair((int)target_x, (int)target_y);
}

void checkGround(bool coarse) {
    int N;
    if(coarse) {
        N = WindowSizeCoarse / StepSize;
    }
    else {
        N = WindowSize / StepSize;
    }
    const int w = imagewidth / StepSize;
    const int h = imageheight / StepSize;

    int fCount[w][h];   // floor count dp in boxes
    int nfCount[w][h];  // non-floor count dp in boxes
    memset(fCount, -1, sizeof(fCount));
    memset(nfCount, -1, sizeof(nfCount));

    int countFloor = 0, countNFloor = 0;
    int start_i, end_i, start_j, end_j;
    Vec3b pixel; int pixel_b, pixel_g, pixel_r, pixel_avg;
    // Count number of floor and non-floor pixels in boxes of image
    for (int i = 0; i < w; i++) {
        for (int j = 0; j < h; j++) {
            countFloor = 0; countNFloor = 0;
            start_i = i * StepSize; end_i = start_i + StepSize;
            start_j = j * StepSize; end_j = start_j + StepSize;
            for (int k = start_i; k < end_i; k++) {
                for (int l = start_j; l < end_j; l++) {
                    pixel = img.at<Vec3b>(l, k);
                    pixel_b = (int)pixel.val[0];
                    pixel_g = (int)pixel.val[1];
                    pixel_r = (int)pixel.val[2];
                    pixel_avg = (pixel_b + pixel_g + pixel_r)/3;
                    if(pixel_b > FLOOR_COLOR_THRES and pixel_g > FLOOR_COLOR_THRES and pixel_r > FLOOR_COLOR_THRES and abs(pixel_b - pixel_avg) < FLOOR_DIFF_THRES and abs(pixel_g - pixel_avg) < FLOOR_DIFF_THRES and abs(pixel_r - pixel_avg) < FLOOR_DIFF_THRES) {
                        countFloor++;
                    }
                    else {
                        countNFloor++;
                    }
                }
            }
            fCount[i][j] = countFloor;
            nfCount[i][j] = countNFloor;
            // cout << countFloor << "," << countNFloor << " ";
        }
        // cout << endl;
    }

    // Find floor points in image
    for (int i = 0; i < w-N; i++) {
        for (int j = 0; j < h-N; j++) {
            countFloor = 0; countNFloor = 0;
            for (int k = i; k < i+N; k++) {
                for (int l = j; l < j+N; l++) {
                    countFloor += fCount[k][l];
                    countNFloor += nfCount[k][l];
                }
            }
            float floorRatio = ((float)countNFloor)/((float)countFloor + (float)countNFloor);
            // cout << floorRatio << " ";
            if(floorRatio < EMPTY_THRES) {
                if(coarse) {
                    floorPointsCoarse.push_back(make_pair((i+N/2) * StepSize, (j+N/2) * StepSize) );
                }
                else {
                    floorPoints.push_back(make_pair((i+N/2) * StepSize, (j+N/2) * StepSize) );
                }
            }
        }
        // cout << endl;
    }


}

// Get boundary points
void getBoundaryPoints() {
    const int Y_GAP = 10 * StepSize;
    int xBound = floorPoints[0].first, yBound = floorPoints[0].second;
    // cout << "Floor points: " << endl;
    for(auto rit = floorPoints.rbegin(); rit != floorPoints.rend(); rit++) {
        // cout << point.first << " " << point.second << endl;
        if(rit->first != xBound) {
            boundaryPoints.push_back(make_pair(xBound, yBound));
            xBound = rit->first;
            yBound = rit->second;
        }
        else {
            if(rit->second < yBound and (yBound - rit->second < Y_GAP)) {
                yBound = rit->second;
            }
        }
    }
}

void findTileLines() {
    Mat gray, filt, thres;
    // Convert to grayscale
    cvtColor(img, gray, CV_BGR2GRAY);
    // Median filter
    medianBlur(gray, filt, 5);
    // Threshold image
    // adaptiveThreshold(filt, thres, 255, ADAPTIVE_THRESH_GAUSSIAN_C, THRESH_BINARY, 11, 2);
    Canny(filt, thres, 50, 125);
    imwrite("floor_thres.jpg", thres);
    // Find hough transform
    vector<Vec2f> lines;
    HoughLines(thres, lines, 1, CV_PI/180, 500, 0, 0 );
    cout << lines.size() << endl;

    vector<pair<float, float> > line_params;
    for(auto tile_line:lines)
    {
        line_params.push_back(make_pair(tile_line[0], tile_line[1]));
    }
    sort(line_params.begin(), line_params.end());

    // Reduce lines by choosing only 1 from each cluster
    vector<pair<float, float> > line_params_unique;
    float cluster_thres = 10.0;
    float current_cluster_rho = line_params[0].first;
    line_params_unique.push_back(line_params[0]);

    for(auto params:line_params) {
        if(params.first - current_cluster_rho > cluster_thres) {
            current_cluster_rho = params.first;
            line_params_unique.push_back(params);
            cluster_thres = max((float)(0.05 * current_cluster_rho), cluster_thres);
        }
    }

    // draw lines
    const int len = 3000;

    for(auto tile_param: line_params_unique)
    {
        float rho = tile_param.first, theta = tile_param.second;
        cout << rho << " " << theta << endl;
        Point pt1, pt2;
        double a = cos(theta), b = sin(theta);
        double x0 = a*rho, y0 = b*rho;
        //Adding stuff!
        pt1.x = cvRound(x0 + len*(-b));
        pt1.y = cvRound(y0 + len*(a));
        pt2.x = cvRound(x0 - len*(-b));
        pt2.y = cvRound(y0 - len*(a));
        line( img, pt1, pt2, Scalar(0,0,255), 2, CV_AA);
    }
    imwrite("floor_lines.jpg", img);
}

// finds distance(pixels) of obstacle in given direction
// Direction of bot is currently assumed to be same as camera
int findDistance()
{
    const int X_SPAN = 6 * StepSize;  // Why?

    vector<int> pointYVec;
    for(auto point: floorPoints) {
        if(abs(point.first - imagewidth) < X_SPAN) {
            pointYVec.push_back(point.first);
        }
    }
    sort(pointYVec.begin(), pointYVec.end());

    return 0;
}


// Check if file exists
inline bool exists (const std::string& name) {
  struct stat buffer;
  return (stat (name.c_str(), &buffer) == 0);
}


int main(int argc, char const *argv[])
{
    lcm::LCM lcm;
    if(!lcm.good())
       return 1;

    exlcm::example_t data;
    Handler handlerObject;
    lcm.subscribe("PROCESSING_SEND", &Handler::handleMessage, &handlerObject);

    int i = 0;
    string fileName;
    while(true) {
      lcm.handle();
      fileName = to_string(currImage) + ".jpg";
      if(exists(fileName)) {
        img = imread(fileName, CV_LOAD_IMAGE_COLOR);
        imagewidth = img.cols;
        imageheight = img.rows;

        checkGround();
        data.distance = findDistance();
        data.angle = getZAngle()
        lcm.publish("PROCESSING_RECEIVE", &data);
        i++;
      }

    }

    return 0;
}
