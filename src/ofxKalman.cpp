#include "ofxKalman.h"
#include "ofMain.h"


void ofxKalman::setState(float state, float covariance){
 this->state      = state;
 this->covariance = covariance;
}

float ofxKalman::correct(float data){
 //time update - prediction
 X0 = F*state;
 P0 = F*covariance*F + Q;

 //measurement update - correction
 float K = H*P0/(H*P0*H + R);
 state   = X0 + K*(data - H*X0);
 covariance = (1 - K*H)*P0;

 return state;
}
