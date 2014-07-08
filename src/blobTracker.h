#pragma once

#include "ofxOpenCv.h"
#include "ofxKalman.h"

class touch{	

	public:
		
		int id;
		ofxCvBlob blob;
		int framesToLive;
		ofxKalman2f filter;
};


class blobTracker {

    public:

        blobTracker();
        virtual ~blobTracker();

        void track(vector<ofVec3f>& points, int aliveFrames, float speedThreshold);
        vector<touch> & getTouchList();

		ofEvent<ofxCvBlob>    blobAdded;
		ofEvent<ofxCvBlob>    blobMoved;
		ofEvent<ofxCvBlob>    blobDeleted;

    private:

		int trackKnn(vector<touch> *newTouchList, ofxCvBlob *track, int k);

		///Массив с данными о прикосновениях
		vector<touch>	touchList;
        int				idCounter;

};
