#include "cloudFilter.h"

//--------------------------------------------------------------
float spatialCloudUtils::getDistanceFromPointToPlane(ofVec3f planePoint, ofVec3f planeNormal, ofVec3f point){
	
	ofVec3f n = planeNormal;
	ofVec3f p = point;

	/// Высчитываем коэффициент для уравнения плоскости.
	float d = 0 - planePoint.x*n.x - planePoint.y*n.y - planePoint.z*n.z;
	/// Находим кратчайшее расстояние от руки до плоскости тела.
	return (n.x*p.x + n.y*p.y + n.z*p.z + d) / sqrt(n.x*n.x + n.y*n.y + n.z*n.z);
}

//--------------------------------------------------------------
void spatialCloudUtils::drawCloud(ofVec3f centroid, int q, ofxKinectNui &kinectDevice){
	
	ofMesh mesh;
	mesh.setMode(OF_PRIMITIVE_TRIANGLE_STRIP);

	for (int i = 0; i < kinectDevice.getDepthResolutionWidth(); i+= q)
		for (int j = 0; j < kinectDevice.getDepthResolutionHeight(); j+= q){
			ofColor c;
			c.setBrightness ( 255 * ofMap(kinectDevice.getDistanceAt(i,j),4000,800,0,1) ); 
			mesh.addColor( c );
			mesh.addVertex( kinectDevice.getWorldCoordinateAt(i,j) );
		}

	ofPushMatrix();
		ofTranslate(centroid.x ,centroid.y,centroid.z);
		ofScale(1, -1, -1);
		mesh.drawVertices();
	ofPopMatrix();
}

//--------------------------------------------------------------
void spatialCloudUtils::getCloud(int x, int y, int w, int h, int q, ofxKinectNui &kinectDevice, std::vector <cloudPoint> &outCloud){

	outCloud.clear();
	for (int i = x; i < x + w; i += q){
		for (int j = y; j < y + h; j += q) {
			cloudPoint p;
			p.pos3d		= kinectDevice.getWorldCoordinateFor(i, j) * SCALE_CLOUD;
			p.pos2d.x	= i;
			p.pos2d.y	= j;
			p.color.setBrightness ( 255 * ofMap(kinectDevice.getDistanceAt(i,j),4000,800,0,1) ); 
			if (!(p.pos3d.x == 0 && p.pos3d.y == 0 && p.pos3d.z == 0)) outCloud.push_back(p);	
		}
	}
}

//--------------------------------------------------------------
void spatialCloudUtils::getCloud(cloudBounding	&bounding, int q, ofxKinectNui &kinectDevice, std::vector <cloudPoint> &outCloud){

	outCloud.clear();
	/// 1 Шаг - определить основную область забора точек.

	/// Определяем ширину поиска.
	int AB = abs(bounding.B.pos2d.x - bounding.A.pos2d.x);
	int CD = abs(bounding.D.pos2d.x - bounding.C.pos2d.x);
	int w  = ( AB > CD ) ? AB : CD;

	/// Определяем высоту поиска.
	int BC = abs(bounding.C.pos2d.y - bounding.B.pos2d.y);
	int AD = abs(bounding.D.pos2d.y - bounding.A.pos2d.y);
	int h  = ( BC > AD ) ? BC : AD;

	/// Стартовая позиция для поиска.
	int x = ( AB > CD ) ? bounding.A.pos2d.x : bounding.D.pos2d.x;
	int y = ( AD > BC ) ? bounding.A.pos2d.y : bounding.B.pos2d.y;

	/// 2 Шаг - сформировать облако.
	for (int i = x; i < x + w; i += q){
		for (int j = y; j < y + h; j += q) {
			cloudPoint p;
			p.pos3d		= kinectDevice.getWorldCoordinateFor(i, j) * SCALE_CLOUD;
			p.pos2d.x	= i;
			p.pos2d.y	= j;
			p.color.setBrightness ( 255 * ofMap(kinectDevice.getDistanceAt(i,j),4000,800,0,1) ); 
			/*if ( pointInBounding( bounding, ofVec2f(i,j) ) ) */outCloud.push_back(p);	
		}
	}  
}

//--------------------------------------------------------------
void spatialCloudUtils::getBounding3D(cloudBounding &bounding, ofxKinectNui &kinectDevice){

	bounding.A.pos3d = kinectDevice.getWorldCoordinateFor(bounding.A.pos2d.x,bounding.A.pos2d.y) * SCALE_CLOUD;
	bounding.B.pos3d = kinectDevice.getWorldCoordinateFor(bounding.B.pos2d.x,bounding.B.pos2d.y) * SCALE_CLOUD;
	bounding.C.pos3d = kinectDevice.getWorldCoordinateFor(bounding.C.pos2d.x,bounding.C.pos2d.y) * SCALE_CLOUD;
	bounding.D.pos3d = kinectDevice.getWorldCoordinateFor(bounding.D.pos2d.x,bounding.D.pos2d.y) * SCALE_CLOUD;
}

//--------------------------------------------------------------
void spatialCloudUtils::getSubCenters(vector<cloudPoint> &cloud, cloudBounding bounding, ofVec3f &A, ofVec3f &B, ofVec3f &C, ofVec3f &D, ofVec3f &centroid){
	
	/// Определяем центр облака.
	getCentroid( cloud, centroid);
	
	/// Определяем расположение центров четвертей облака.
	vector <int> indexes;
	A = (bounding.A.pos3d + centroid) / 2;
	float radiusA = (bounding.A.pos3d - centroid).length() / 2;
	findNeighbourNode(cloud,A,indexes,radiusA);
	getCentroid(cloud, indexes, A);

	B = (bounding.B.pos3d + centroid) / 2;
	float radiusB = (bounding.B.pos3d - centroid).length() / 2;
	findNeighbourNode(cloud,B,indexes,radiusB);
	getCentroid(cloud, indexes, B);

	C = (bounding.C.pos3d + centroid) / 2;
	float radiusC = (bounding.C.pos3d - centroid).length() / 2;
	findNeighbourNode(cloud,C,indexes,radiusC);
	getCentroid(cloud, indexes, C);

	D = (bounding.D.pos3d + centroid) / 2;
	float radiusD = (bounding.D.pos3d - centroid).length() / 2;
	findNeighbourNode(cloud,D,indexes,radiusD);
	getCentroid(cloud, indexes, D);
}

//--------------------------------------------------------------
void spatialCloudUtils::findNeighbourNode(vector<cloudPoint> &cloud,ofVec3f node, vector<int> &ind, float r){
	ind.clear();
	for (int i = 0; i < cloud.size(); i++){
		float d = (node - cloud[i].pos3d).length();
		if (d < r){
			ind.push_back(i);
		}
	}
}

//--------------------------------------------------------------
void spatialCloudUtils::getCentroid(vector<cloudPoint> &cloud, vector<int> &ind, ofVec3f &centroid){
	centroid.set(0,0,0);
	int size = ind.size();
	for (int i = 0; i < size; i++){
		centroid += cloud[ind[i]].pos3d;
	}
	centroid /= size;
}
	
//--------------------------------------------------------------
ofVec3f spatialCloudUtils::getNormalTriangle(ofVec3f p1, ofVec3f p2, ofVec3f p3){
	ofVec3f v1 = p1 - p2;
	ofVec3f v2 = p3 - p2;
	return v1.getCrossed(v2);
}
	
//--------------------------------------------------------------
ofVec3f spatialCloudUtils::getNormalByCloud(vector<cloudPoint> &cloud, cloudBounding &bounding, ofxKinectNui &kinectDevice, ofVec3f &centroidMyCloud){
		
	ofVec3f	subCenterA, subCenterB, subCenterC, subCenterD;
	getSubCenters( cloud, bounding, subCenterA, subCenterB, subCenterC, subCenterD, centroidMyCloud);
	
	ofVec3f N1 = getNormalTriangle (subCenterA, subCenterB, centroidMyCloud);
	ofVec3f N2 = getNormalTriangle (subCenterB, subCenterC, centroidMyCloud);
	ofVec3f N3 = getNormalTriangle (subCenterC, subCenterD, centroidMyCloud);
	ofVec3f N4 = getNormalTriangle (subCenterD, subCenterA, centroidMyCloud);

	ofVec3f N5 = getNormalTriangle (bounding.A.pos3d, bounding.B.pos3d, centroidMyCloud);
	ofVec3f N6 = getNormalTriangle (bounding.B.pos3d, bounding.C.pos3d, centroidMyCloud);
	ofVec3f N7 = getNormalTriangle (bounding.C.pos3d, bounding.D.pos3d, centroidMyCloud);
	ofVec3f N8 = getNormalTriangle (bounding.D.pos3d, bounding.A.pos3d, centroidMyCloud);

	ofVec3f	N = (N1 + N2 + N3 + N4 + N5 + N6 + N7 + N8) / 8; 
	return  N;
}

//--------------------------------------------------------------
bool spatialCloudUtils::pointInBounding(cloudBounding	&bounding, ofVec2f p){
	ofVec2f PA = p - bounding.A.pos2d;
	ofVec2f PC = p - bounding.C.pos2d;
	ofVec2f PD = p - bounding.D.pos2d;
	ofVec2f PB = p - bounding.B.pos2d;
 
	float a1 = PA.angle(PB);
	float a2 = PB.angle(PC);
	float a3 = PC.angle(PD);
	float a4 = PD.angle(PA);

	int sumAngle = ceilf( a1 + a2 + a3 + a4 ); 
	if(sumAngle > 359) return true;
	else return false;
}

//--------------------------------------------------------------
void spatialCloudUtils::getCentroid(vector<cloudPoint> &cloud, ofVec3f &centroid){
	centroid.set(0,0,0);
	int size = cloud.size();
	for (int i = 0; i < size; i++) centroid += cloud[i].pos3d;
	centroid /= size;
}



//--------------------------------------------------------------
spatialBoxFilterCloud::spatialBoxFilterCloud(){
	
	isCreated = false;
}

//--------------------------------------------------------------
void spatialBoxFilterCloud::drawSpatialBoxFilter(){
	
	ofPushMatrix();
		ofTranslate(centroid.x ,centroid.y,centroid.z);
		ofScale(1, -1, -1);
			//ofSetColor(255,255,0);
			//ofCircle( nodeA, 8);
			//ofSetColor(0,255,0);
			//ofCircle( nodeB, 8);
			//ofSetColor(0,0,255);
			//ofCircle( nodeC, 8);
			ofSetColor(255,0,0);
			//ofCircle( nodeD, 3);

			//ofCircle( nodeE, 3);
			//ofCircle( nodeF, 3);
			//ofCircle( nodeG, 3);
			//ofCircle( nodeH, 3);

			ofLine(nodeA, nodeB);
			ofLine(nodeB, nodeC);
			ofLine(nodeC, nodeD);
			ofLine(nodeD, nodeA);

			ofLine(nodeE, nodeF);
			ofLine(nodeF, nodeG);
			ofLine(nodeG, nodeH);
			ofLine(nodeH, nodeE);

			ofLine(nodeA, nodeE);
			ofLine(nodeB, nodeF);
			ofLine(nodeC, nodeG);
			ofLine(nodeD, nodeH);

			ofCircle(centroid,5);
			ofVec3f OZ = ofVec3f(120,120,120) * orientation;
			ofLine( centroid, centroid + OZ);
			ofSetColor(255);

		

			ofSetColor(255);
	ofPopMatrix();
}
	
//--------------------------------------------------------------
float spatialBoxFilterCloud::getDistanceFromPointToPlane(ofVec3f point){
	
	ofVec3f n			= getNormal();
	ofVec3f p			= point;
	ofVec3f planePoint	= nodeA;

	/// Высчитываем коэффициент для уравнения плоскости.
	float d = 0 - planePoint.x*n.x - planePoint.y*n.y - planePoint.z*n.z;
	/// Находим кратчайшее расстояние от руки до плоскости тела.
	return (n.x*p.x + n.y*p.y + n.z*p.z + d) / sqrt(n.x*n.x + n.y*n.y + n.z*n.z);
}


//--------------------------------------------------------------
bool spatialBoxFilterCloud::checkBoundCloudOnMinHeight(float minHeight, subCloundBounding &b){
	/// Получаем высоту облака по отношению к системе координат поверхности
	return (abs(getDistanceFromPointToPlane(b.maxPoint)) > minHeight);
}

//--------------------------------------------------------------
bool spatialBoxFilterCloud::isContactSurface(ofVec3f p, float threshold){
	return ((getDistanceFromPointToPlane(p)) < threshold);
}

//--------------------------------------------------------------
ofVec3f spatialBoxFilterCloud::getTouch3DPointFromCloud(subCloundBounding &b, ofxKinectNui &kinectDevice, float radiusRefine, int areaHalf){
	ofVec3f touchPoint = ofVec3f(0.0,0.0,0.0);
	int	  numNeighBours = 0;
		
	/// Получаем 2D индекс точки ближайшей к плоскости поверхности. 
	ofVec2f pos2D = get2dIndexBy3dPoint(b.minPoint,kinectDevice);
		
	/// Ищем соседей для уточнения на исходных данных с кинекта.
	for (int x = pos2D.x - areaHalf; x < pos2D.x + areaHalf; x++){
		for (int y = pos2D.y - areaHalf; y < pos2D.y + areaHalf; y++){				
			if (y > 0 && x > 0 && x < kinectDevice.getDepthResolutionWidth() && y < kinectDevice.getDepthResolutionHeight()){
				ofVec3f p = kinectDevice.getWorldCoordinateAt(x,y);
				if ((b.minPoint - p).length() < radiusRefine) {
					touchPoint += p;
					numNeighBours++;				
				}
			}
		}
	}
	touchPoint /= numNeighBours;
	/// Если найденная точка оказалась дальше чем минимальная точка блоба - значит лушче использовать её???
	if ((getDistanceFromPointToPlane(touchPoint)) > (getDistanceFromPointToPlane(b.minPoint))) touchPoint = b.minPoint;
	return touchPoint;
}
	
//--------------------------------------------------------------
void  spatialBoxFilterCloud::getMinMaxPointToBox (pcl::PointCloud<pcl::PointXYZ> &cloud, const std::vector<int> &indices, subCloundBounding &b){
	
	float minDist = 1E10;
	float maxDist = 0;

	for(int i = 0; i < indices.size(); i++){
			
		ofVec3f point = ofVec3f(cloud.points[indices[i]].x, cloud.points[indices[i]].y, cloud.points[indices[i]].z);
		float dist = abs(getDistanceFromPointToPlane(point));
		if(dist < minDist){
			minDist = dist;
			b.minPoint = point;
		}

		if(dist > maxDist){
			maxDist = dist;
			b.maxPoint = point;
		}
	}
}
	
//--------------------------------------------------------------
void spatialBoxFilterCloud::drawCloudSurface(){
	ofMesh mesh;
	mesh.setMode(OF_PRIMITIVE_TRIANGLE_STRIP);
	for (int i = 0; i < cloudSurface.size(); i++){
		mesh.addColor( cloudSurface[i].color );
		mesh.addVertex( cloudSurface[i].pos3d );
	}

	ofPushMatrix();
		ofTranslate(centroid.x ,centroid.y,centroid.z);
		ofScale(1, -1, -1);
		mesh.drawVertices();
	ofPopMatrix();
}

//--------------------------------------------------------------
int spatialBoxFilterCloud::getSizeScreen(ofxKinectNui &kinectDevice){
	bounding.A.pos3d = kinectDevice.getWorldCoordinateFor(bounding.A.pos2d.x,bounding.A.pos2d.y) * SCALE_CLOUD;
	bounding.B.pos3d = kinectDevice.getWorldCoordinateFor(bounding.B.pos2d.x,bounding.B.pos2d.y) * SCALE_CLOUD;
	bounding.C.pos3d = kinectDevice.getWorldCoordinateFor(bounding.C.pos2d.x,bounding.C.pos2d.y) * SCALE_CLOUD;
	bounding.D.pos3d = kinectDevice.getWorldCoordinateFor(bounding.D.pos2d.x,bounding.D.pos2d.y) * SCALE_CLOUD;

	float distAC = (bounding.A.pos3d - bounding.C.pos3d).length() / 25.4;
	float distBD = (bounding.B.pos3d - bounding.D.pos3d).length() / 25.4;
			
	return (distAC + distBD) / 2; 
}

//--------------------------------------------------------------
void spatialBoxFilterCloud::create(ofVec2f a, ofVec2f b, ofVec2f c, ofVec2f d, ofxKinectNui &kinectDevice, int q, float offsetNormal, float volume, float scaleDirection, ofVec3f position){
	/// Шаг 1 - получаем границы облака по 4 точкам с проекции.			
	bounding.A.pos2d = a;
	bounding.B.pos2d = b;
	bounding.C.pos2d = c;
	bounding.D.pos2d = d;
	utils.getBounding3D(bounding,kinectDevice);

	/// Шаг 2 - получаем облако по ограничивающим точкам.
	utils.getCloud ( bounding, q, kinectDevice, cloudSurface );

	/// Шаг 3 - получаем нормаль поверхности. Определяем вектор ориентации в пространстве.
	normal		= utils.getNormalByCloud( cloudSurface, bounding, kinectDevice, centroid);
	orientation = (normal - centroid).getNormalized();

	/// Шаг 4 - получаем первые 4 вершины пространственного фильтра.
	float r = 100;
	vector <int> ind;

	utils.findNeighbourNode(cloudSurface,bounding.A.pos3d,ind,r);
	utils.getCentroid(cloudSurface,ind,nodeA);
			
	utils.findNeighbourNode(cloudSurface,bounding.B.pos3d,ind,r);
	utils.getCentroid(cloudSurface,ind,nodeB);
			
	utils.findNeighbourNode(cloudSurface,bounding.C.pos3d,ind,r);
	utils.getCentroid(cloudSurface,ind,nodeC);
			
	utils.findNeighbourNode(cloudSurface,bounding.D.pos3d,ind,r);
	utils.getCentroid(cloudSurface,ind,nodeD);
			
						
	/// Шаг 5 - Смещение вершин от центра. Определяет объем тела.			
	ofVec3f vec3fScaleDirection	= ofVec3f(scaleDirection, scaleDirection, scaleDirection );

	/// Шаг 6 - Смещение относительно нормали и объем тела.
	ofVec3f normalStartOffset   = ofVec3f( offsetNormal, offsetNormal, offsetNormal ) * orientation;
	ofVec3f normalEndOffset		= ofVec3f( volume, volume, volume ) * orientation;

	/// Шаг 7 - Определение конечных координат вершин.
	nodeA += position + normalStartOffset + vec3fScaleDirection * (nodeA - centroid).getNormalized();
	nodeB += position + normalStartOffset + vec3fScaleDirection * (nodeB - centroid).getNormalized();
	nodeC += position + normalStartOffset + vec3fScaleDirection * (nodeC - centroid).getNormalized();
	nodeD += position + normalStartOffset + vec3fScaleDirection * (nodeD - centroid).getNormalized();
			
	nodeE = nodeA + normalEndOffset;
	nodeF = nodeB + normalEndOffset;
	nodeG = nodeC + normalEndOffset;
	nodeH = nodeD + normalEndOffset;

	/// Определение нормалей к поверхностям тела.
	normals.clear();
	normals.push_back((nodeE - nodeA).getCrossed(nodeB - nodeA));
	normals.push_back((nodeF - nodeB).getCrossed(nodeC - nodeB));
	normals.push_back((nodeG - nodeC).getCrossed(nodeD - nodeC));
	normals.push_back((nodeH - nodeD).getCrossed(nodeE - nodeD));
	normals.push_back((nodeB - nodeD).getCrossed(nodeC - nodeD));
	normals.push_back((nodeG - nodeE).getCrossed(nodeF - nodeE));

	isCreated = true;

	testPoint = (nodeA + nodeB + nodeC + nodeD+ nodeE+ nodeF + nodeG + nodeH) / 8; 

	testPoint.z -= 800;
}

//--------------------------------------------------------------
ofVec3f spatialBoxFilterCloud::getCentroid() {return centroid;}

//--------------------------------------------------------------
void spatialBoxFilterCloud::update(ofxKinectNui &kinectDevice, int q, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud){

	if(!isCreated) return;

	cloudInFilter.clear();
	cloud->clear();
	for (int i = 0; i < kinectDevice.getDepthResolutionWidth(); i+= q)
		for (int j = 0; j < kinectDevice.getDepthResolutionHeight(); j+= q){
			ofVec3f point = kinectDevice.getWorldCoordinateAt(i,j);
			if (insideSpatialBox(point)) {
				cloudPoint p;
				p.pos2d.x = i;
				p.pos2d.y = j;
				p.pos3d   = point;
				p.color	  = ofColor(255,0,0);
				cloudInFilter.push_back(p);
				cloud->push_back(pcl::PointXYZ(point.x, point.y, point.z));
			}
		}		
}

//--------------------------------------------------------------
ofVec2f spatialBoxFilterCloud::get2dIndexBy3dPoint(ofVec3f p, ofxKinectNui &kinectDevice){
	int		step0 = 32;
	float	minDist = 1E10;
	int		indexX0, indexY0;

	/// 0 - Шаг.
	/// Сперва ищем ближайшую точку к точке p? на всем сильно прореженном изображении.
	for (int i = 0; i < kinectDevice.getDepthResolutionWidth(); i+= step0)
		for (int j = 0; j < kinectDevice.getDepthResolutionHeight(); j+= step0){
			float dist = (p - kinectDevice.getWorldCoordinateAt(i,j)).length();					
			if (minDist > dist){
				minDist = dist;
				indexX0 = i;
				indexY0 = j;
			}
		}
			
	/// 1 - Шаг.
	int step1 = 8;
	int	indexX1 = indexX0;
	int indexY1 = indexY0;
	/// Теперь ищем точку в ближайшей окрестности.
	for (int i = indexX0 - step0; i <  indexX0 + step0; i+= step1)
		for (int j = indexY0 - step0; j <  indexY0 + step0; j+= step1){
			if (j > 0 && i > 0 && i < kinectDevice.getDepthResolutionWidth() && j < kinectDevice.getDepthResolutionHeight()){
				float dist = (p - kinectDevice.getWorldCoordinateAt(i,j)).length();					
				if (minDist > dist){
					minDist = dist;
					indexX1 = i;
					indexY1 = j;
				}
			}
		}
		
	/// 2 - Шаг.
	int step2 = 1;
	int	indexX2 = indexX1;
	int indexY2 = indexY1;
	for (int i = indexX1 - step1; i <  indexX1 + step1; i+= step2)
		for (int j = indexY1 - step1; j <  indexY1 + step1; j+= step2){

			if (j > 0 && i > 0 && i < kinectDevice.getDepthResolutionWidth() && j < kinectDevice.getDepthResolutionHeight()){
				float dist = (p - kinectDevice.getWorldCoordinateAt(i,j)).length();		
				if (minDist > dist){
					minDist = dist;
					indexX2 = i;
					indexY2 = j;
				}
			}
	}


	return ofVec2f(indexX2, indexY2);
}

//--------------------------------------------------------------
bool spatialBoxFilterCloud::insideSpatialBox(ofVec3f p){

	bool sideN = (normals[0].dot(p - nodeA) > 0.0f);
	bool side = (normals[1].dot(p - nodeB) > 0.0f);
	if(side != sideN) return false;
	side = (normals[2].dot(p - nodeC) > 0.0f);
	if(side != sideN) return false;
	side = (normals[3].dot(p - nodeD) > 0.0f);
	if(side != sideN) return false;
	side = (normals[4].dot(p - nodeD) > 0.0f);
	if(side != sideN) return false;
	side = (normals[5].dot(p - nodeE) > 0.0f);
	if(side != sideN) return false;
	
	return true;
}

//--------------------------------------------------------------
void spatialBoxFilterCloud::drawFilterCloud(vector <cloudPoint> cloud){
			
	glPointSize(3);

	ofMesh mesh;
	mesh.setMode(OF_PRIMITIVE_TRIANGLE_STRIP);
	for (int i = 0; i < cloud.size(); i++){
		mesh.addColor( cloud[i].color );
		mesh.addVertex( cloud[i].pos3d );

		ofPushMatrix();
		ofTranslate(centroid.x ,centroid.y,centroid.z);
		ofScale(1, -1, -1);
			if(cloud[i].color.g == 255){
				ofSetColor(cloud[i].color);
				ofSphere(cloud[i].pos3d, 5);
			}
		ofPopMatrix();
	}
	/*for (int i = 0; i < cloud->size(); i++){
		mesh.addColor( ofColor(255,0,0) );
		mesh.addVertex( ofVec3f(cloud->points[i].x, cloud->points[i].y, cloud->points[i].z) );
	}*/

	ofPushMatrix();
		ofTranslate(centroid.x ,centroid.y,centroid.z);
		ofScale(1, -1, -1);
		mesh.drawVertices();
		ofCircle(testPoint, 10);
	ofPopMatrix();

	glPointSize(1);
}