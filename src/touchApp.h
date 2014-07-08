#pragma once

#define NOMINMAX

#include "cloudFilter.h"
#include "ofMain.h"
#include "ofxOpenCv.h"
#include "ofxXmlSettings.h"
#include "ofxUI.h"
#include "blobTracker.h"

#define USE_TUIO

#ifdef USE_TUIO
#include "TuioServer.h"
#endif


#define GUI_WIDTH			200
#define GUI_OFFSET_X		0
#define GUI_OFFSET_Y		0
#define GUI_HEIGHT			700

#define IMAGE_DRAW_OFFSET_X	220
#define IMAGE_DRAW_OFFSET_Y 20


class touchApp : public ofBaseApp {
	public:
			
		void setup();
		void update();
		void draw();

		void draw3D();
		void drawCalibration();

		void exit();
		void keyPressed  (int key);
		void mouseMoved(int x, int y );
		void mouseDragged(int x, int y, int button);
		void mousePressed(int x, int y, int button);
		void mouseReleased(int x, int y, int button);
		void windowResized(int w, int h);

		void setupKinect(bool grabCalibratedVideo);

		void setupGui();
		void guiEvent(ofxUIEventArgs &e);

		void touchAdded		(ofxCvBlob & blob);
		void touchMoved		(ofxCvBlob & blob);
		void touchDeleted	(ofxCvBlob & blob);

		void saveSettings();

		void calibrationMouseInputHandle(int x, int y);

		/// Основная функция обработки данных.
		int getTouchPoints(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, double radius, float radiusRefine, float thresholdDistance, float minHeight, int areaHalf);
		void get2DTouchPoints(vector <cloudPoint>& touchPoints3D, vector<ofVec3f>& touchPoints2D);

		void   getMinMaxPointToBox (pcl::PointCloud<pcl::PointXYZ> &cloud, const std::vector<int> &indices, 
               ofVec3f &min_pt, ofVec3f &max_pt);

		ofxUICanvas		* gui;
		ofxXmlSettings	xml;
		blobTracker		tracker;	

		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
		vector <cloudPoint>	cloudInFilter;	
		vector <cloudPoint>	touchPoints;	
		vector <ofVec3f>	touchPoints2d;	
		float radius;
	
		ofxKinectNui				kinect; 
		ofEasyCam					cam;
		int							angle;
		ofImage						depthImage;
		ofFbo						fboCam;

		spatialBoxFilterCloud cloudFilter;
		spatialCloudUtils	  cloudUtils;

		float volumeFilter;
		float zOffsetFilter;
		ofVec2f boundPos;
		ofVec2f boundDim;

		vector< pair<ofVec3f, ofVec3f> > bBoxes;

		bool isFreeze;
		bool isCalibrating;
		bool isDraw3D;
		bool testFullScreen;

		/// Радиус уточнения.
		float radiusRefine;
		/// Дистанция в мм от плоскости модели поверхности при которой считаем что касание есть. 
		float thresholdDistance;
		/// Минимальная высота блота.
		float minHeight;
		/// Радиус поиска на карте глубины.
		int	  areaHalf;	

		/// Размер экрана в дюймах.
		int sizeScreen;
		/// Время обработки одной итерации.
		int calcTime;
		/// Качество облака.
		int	qualityCloud;
		/// Число кадров, в течение которых моргнувший палец не исчезает
		int aliveFrames;
		/// Порог скорости для курсора, для защиты от дрожания
		float cursorSpeedThreshold;

		#ifdef USE_TUIO
		TUIO::TuioServer tuio;
		#endif

};

