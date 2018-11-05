#include <iostream>
#include <opencv2/core/core.hpp>
#include <jsoncpp/json/json.h> // or something

using namespace std;
using namespace cv;

int main() {
    // create the characters array
    Json::Value ch;
    ch[0]["name"] = "Jabberwock";
    ch[0]["chapter"] = 1;
    ch[1]["name"] = "Cheshire Cat";
    ch[1]["chapter"] = 6;
    ch[2]["name"] = "Mad Hatter";
    ch[2]["chapter"] = 7;

    // create the main object
    Json::Value val;
    val["book"] = "Alice in Wonderland";
    val["year"] = 1865;
    val["characters"] = ch;

    Json::Value config;
    Point2f ball = Point2f(0.0, 1.0);
    Point2f ballVel = Point2f(0.0, 0.0);
    double timeUntilImpact = 1.0;
    double timeAtEstimation = 1.0;
    bool postureConfig = false;
    bool balanceConfig = false;
    
    config[0]["ball"].append(ball.x);
    config[0]["ball"].append(ball.y);
    
    config[0]["ballVel"].append(ballVel.x);
    config[0]["ballVel"].append(ballVel.y);
    
    config[0]["timeUntilImpact"] = timeUntilImpact;
    config[0]["timeAtEstimation"] = timeAtEstimation;
    
    config[0]["postureConfig"] = postureConfig;
    config[0]["balanceConfig"] = balanceConfig;

    //cout << val << '\n';
    cout << config << '\n';
}
