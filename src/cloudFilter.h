#pragma once

#define NOMINMAX

#include "ofxKinectNui.h"

#undef nil

#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include "ofMain.h"

#define SCALE_CLOUD 1000

typedef struct{
  ofVec3f pos3d;
  ofVec2f pos2d;
  ofColor color;
} cloudPoint;

typedef struct {
 cloudPoint A;
 cloudPoint B;
 cloudPoint C;
 cloudPoint D;
} cloudBounding;

typedef struct {
 ofVec3f minPoint;
 ofVec3f maxPoint;
} subCloundBounding;


/// Работа с облаком точек.
class spatialCloudUtils{

	public:

		/// Найти дистанцию между плоскостью и облаком.
		float getDistanceFromPointToPlane(ofVec3f planePoint, ofVec3f planeNormal, ofVec3f point);
	
		/// Рисуем облако с кинекта.
		void drawCloud(ofVec3f centroid, int q, ofxKinectNui &kinectDevice);
	
		/// Получить облако по прямоугольному фрагменту карты глубины.
		void getCloud(int x, int y, int w, int h, int q, ofxKinectNui &kinectDevice, std::vector <cloudPoint> &outCloud);

		/// Получить облако по четырем точкам проекции.
		void getCloud(cloudBounding	&bounding, int q, ofxKinectNui &kinectDevice, std::vector <cloudPoint> &outCloud);

		/// Получить трехмерные координаты границ проекции на облаке точек.
		void getBounding3D(cloudBounding &bounding, ofxKinectNui &kinectDevice);

		/// Получить центры четвертей облака.
		void getSubCenters(vector<cloudPoint> &cloud, cloudBounding bounding, ofVec3f &A, ofVec3f &B, ofVec3f &C, ofVec3f &D, ofVec3f &centroid);
	
		/// Получить номера соседей вершин в окрестности точки node.
		void findNeighbourNode(vector<cloudPoint> &cloud,ofVec3f node, vector<int> &ind, float r);

		/// Получить центроид соседей облака.
		void getCentroid(vector<cloudPoint> &cloud, vector<int> &ind, ofVec3f &centroid);
		/// Получить единичную нормаль треугольника.
		ofVec3f getNormalTriangle(ofVec3f p1, ofVec3f p2, ofVec3f p3);
	
		/// Определить для облака точек.
		ofVec3f getNormalByCloud(vector<cloudPoint> &cloud, cloudBounding	&bounding, ofxKinectNui &kinectDevice, ofVec3f &centroidMyCloud);

		/// Проверить лежит ли точка внутри границ проекции.
		bool pointInBounding(cloudBounding	&bounding, ofVec2f p);

		/// Получить центроид.
		void getCentroid(vector<cloudPoint> &cloud, ofVec3f &centroid);
};


/// Пространственный фильтр отбора точек облака.
class spatialBoxFilterCloud{
	
	public:

		spatialBoxFilterCloud();

		void drawSpatialBoxFilter();

		/// Найти дистанцию между плоскостью и облаком.
		float getDistanceFromPointToPlane(ofVec3f point);

		/// Проверяем высоту облака по отношению к поверхности.
		bool checkBoundCloudOnMinHeight(float minHeight, subCloundBounding &b);

		/// Проверяем есть ли контакт с поверхностью или в некоторых пределах.
		bool isContactSurface(ofVec3f p, float threshold);

		/// Получение возможной точки контакты
		ofVec3f getTouch3DPointFromCloud(subCloundBounding &b, ofxKinectNui &kinectDevice, float radiusRefine, int areaHalf);

		/// Найти минимум и максимум из точек облака в системе координат поверхности.
		void  getMinMaxPointToBox (pcl::PointCloud<pcl::PointXYZ> &cloud, const std::vector<int> &indices, subCloundBounding &b);
	
		/// Рисуем облако точек и его центр и нормаль.
		void drawCloudSurface();

		/// Получить диагональ границ проекции на облаке точек в дюймах.
		int getSizeScreen(ofxKinectNui &kinectDevice);

		/// Создаёт пространственный фильтр по 4 точкам проекции, с прорежением q.
		void create(ofVec2f a, ofVec2f b, ofVec2f c, ofVec2f d, ofxKinectNui &kinectDevice, int q, float offsetNormal, float volume, float scaleDirection, ofVec3f position = ofVec3f(0,0,0));

		ofVec3f getCentroid();

		void update(ofxKinectNui &kinectDevice, int q, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);

		/// Получаем 2D значение индекса по 3D точке, используя 3-ступенчатый пирамидальный поиск
		ofVec2f get2dIndexBy3dPoint(ofVec3f p, ofxKinectNui &kinectDevice);

		bool insideSpatialBox(ofVec3f p);
		
		void drawFilterCloud(vector <cloudPoint> cloud);

		std::vector <cloudPoint>	cloudInFilter;	
		
		/// Нумерация вершин объемного тела фильтра по часовой стрелке.
		ofVec3f	nodeA, nodeB, nodeC, nodeD, nodeE, nodeF, nodeG, nodeH;

		ofVec3f& getNormal(){ return normal;}

	private:
		
		/// Исходное облако поверхности, по которому создаётся пространственный фильтр.
		std::vector <cloudPoint>	cloudSurface;	
		/// Нормаль поверхности.
		ofVec3f						normal;
		/// Геометрический центр.
		ofVec3f						centroid;
		/// Ориентация фильтра в пространстве.
		ofVec3f						orientation;
		/// Работа с облаками.
		spatialCloudUtils			utils;
		/// Границы работы с облаком.
		cloudBounding				bounding;
		/// 4 вершины - основание фильтра.
		ofVec3f A, B, C, D;
		
		/// Нормали к поверхностям объемного фильтра.
		vector<ofVec3f> normals;
		
		bool isCreated;

		ofVec3f testPoint;
};

