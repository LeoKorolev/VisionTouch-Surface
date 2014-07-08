/*
 * ofxCvKalman.h
 *
 *  Created on: 23-jun-2009
 *      Author: art
 */

#ifndef OFXKALMAN_H_
#define OFXKALMAN_H_

#include "ofMain.h"

class ofxKalman {
public:

	ofxKalman(){
	}

    void  setup(float q, float r, float f = 1, float h = 1){
     Q = q;
     R = r;
     F = f;
     H = h;
    };

	void  setState(float state, float covariance);
	float correct(float data);
//	ofVec3f correct(ofVec3f data);

    virtual ~ofxKalman(){
    };

    float X0;
    float P0;
    float F;
    float Q;
    float H;
    float R;

    float state;
    float covariance;
};

class ofxKalman3f {
 public:
   ofxKalman kx;
   ofxKalman ky;
   ofxKalman kz;

   void setup(float q, float r, float f = 1, float h = 1){
    kx.setup(q,r,f,h);
    kz = ky = kx;
   };

   void setState(ofVec3f state, float covariance){
    kx.setState(state.x,.1);
    ky.setState(state.y,.1);
    kz.setState(state.z,.1);
   };

   ofVec3f getCorrect(ofVec3f p){
    ofVec3f p1 = ofVec3f(kx.correct(p.x),ky.correct(p.y),kz.correct(p.z));
    return p1;
   }
};

class ofxKalman2f {
 public:
   ofxKalman kx;
   ofxKalman ky;

   void setup(float q, float r, float f = 1, float h = 1){
    kx.setup(q,r,f,h);
    ky = kx;
   };

   void setState(ofVec2f state, float covariance){
    kx.setState(state.x,.1);
    ky.setState(state.y,.1);
   };

   ofVec2f getCorrect(ofVec2f p){
    ofVec2f p1 = ofVec2f(kx.correct(p.x),ky.correct(p.y));
    return p1;
   }
};



#endif /* OFXCVKALMAN_H_ */
