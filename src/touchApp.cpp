#include "touchApp.h"

//--------------------------------------------------------------
void clickDown(){

	INPUT Input = {0};
	Input.type = INPUT_MOUSE;
	Input.mi.dwFlags = MOUSEEVENTF_LEFTDOWN;
	SendInput(1, &Input, sizeof(INPUT));
}

//--------------------------------------------------------------
void move(float x, float y, bool drag){

	const double XSCALEFACTOR = (double)65535 / (GetSystemMetrics(SM_CXSCREEN) - 1);
	const double YSCALEFACTOR = (double)65535 / (GetSystemMetrics(SM_CYSCREEN) - 1);

	INPUT Input = {0};
	Input.type = INPUT_MOUSE; 
	Input.mi.dx = x * XSCALEFACTOR;
	Input.mi.dy = y * YSCALEFACTOR;
	if(drag) Input.mi.dwFlags = MOUSEEVENTF_ABSOLUTE | MOUSEEVENTF_MOVE | MOUSEEVENTF_LEFTDOWN;
	else Input.mi.dwFlags = MOUSEEVENTF_ABSOLUTE | MOUSEEVENTF_MOVE;
	SendInput(1, &Input, sizeof(INPUT));
}

//--------------------------------------------------------------
void clickUp(){

	INPUT Input = {0};
	Input.type = INPUT_MOUSE; 
	Input.mi.dwFlags = MOUSEEVENTF_LEFTUP;
	SendInput(1, &Input, sizeof(INPUT));
}


//--------------------------------------------------------------
void touchApp::setup() {

	ofSetLogLevel(OF_LOG_VERBOSE);
	
	setupKinect(false);
	
	cloud = boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>>(new pcl::PointCloud<pcl::PointXYZ>); 

	ofBackground(100);

	fboCam.allocate(640, 480, GL_RGBA);
	fboCam.begin();
	ofClear(0, 0, 0, 0);
	fboCam.end();

	isFreeze		= false;
	isCalibrating	= false;
	isDraw3D		= false;
	testFullScreen	= false;
	sizeScreen		= 0; 
	
	if(!xml.loadFile("settings.xml"))
		cout << "Failed to load settings!" << endl;

	xml.pushTag("settings");

	angle = xml.getValue("angle", 0);
	kinect.setAngle(angle);

	ofAddListener(tracker.blobAdded, this,		&touchApp::touchAdded);
    ofAddListener(tracker.blobMoved, this,		&touchApp::touchMoved);
    ofAddListener(tracker.blobDeleted, this,	&touchApp::touchDeleted);

	/// Радиус уточнения.
	radiusRefine			= xml.getValue("radiusRefine", 0.0);
	/// Дистанция в мм от плоскости модели поверхности при которой считаем что касание есть. 
	thresholdDistance		= xml.getValue("thresholdDistance", 0.0);
	/// Минимальная высота блоба.
	minHeight				= xml.getValue("minHeight", 0.0);
	/// Радиус поиска на карте глубины.
	areaHalf				= xml.getValue("areaHalf", 0);
	/// Качество облака
	qualityCloud			= xml.getValue("qualityCloud", 0);
	/// Высота пространственного фильтра.
	volumeFilter			= xml.getValue("volumeFilter", 0.0);
	/// Смещение фильтра от поверхности, вперед, в направлении нормали поверхности.
	zOffsetFilter			= xml.getValue("zOffsetFilter", 0.0);
	/// Радиус для 3D сегментации.
	radius					= xml.getValue("radius", 0.0);
	/// Число кадров, через которое исчезает блоб, для защиты от промаргивания
	aliveFrames				= xml.getValue("aliveFrames", 0);
	/// Порог скорости для курсора, для защиты от дрожания
	cursorSpeedThreshold	= xml.getValue("cursorSpeedThreshold", 0.0);
	/// Границы интерактивной области
	int x = xml.getValue("boundX", 0);
	int y = xml.getValue("boundY", 0);
	int w = xml.getValue("boundW", 0);
	int h = xml.getValue("boundH", 0);
	
	boundPos.set(x, y);
	boundDim.set(w, h);

	setupGui();

	ofSetFrameRate(90);

	//tuio.setVerbose(true);
}

//--------------------------------------------------------------
void touchApp::setupKinect(bool grabCalibratedVideo){

	ofxKinectNui::InitSetting		initSetting;
	initSetting.grabVideo			= grabCalibratedVideo;
	initSetting.grabDepth			= true;
	initSetting.grabAudio			= false;
	initSetting.grabLabel			= false;
	initSetting.grabSkeleton		= false; 
	initSetting.grabCalibratedVideo	= grabCalibratedVideo;
	initSetting.grabLabelCv			= false; 
	initSetting.depthResolution		= NUI_IMAGE_RESOLUTION_640x480;
	initSetting.videoResolution		= NUI_IMAGE_RESOLUTION_640x480;
	
	kinect.init(initSetting);
	kinect.enableDepthNearValueWhite(false);
	kinect.setFarClippingDistance(800);
	kinect.setNearClippingDistance(4000);
	kinect.open();
}

//--------------------------------------------------------------
void touchApp::setupGui(){

	gui = new ofxUICanvas();
	gui->setFontSize(OFX_UI_FONT_SMALL,8);
	gui->setAutoDraw(false);
	gui->addLabel("Parameter settings", OFX_UI_FONT_LARGE);
	gui->addSpacer(); 
	//gui->addSpacer(GUI_WIDTH,5);
	gui->addSlider("volumeFilter",0,300,&volumeFilter);
	gui->addSpacer(GUI_WIDTH,2);
	gui->addSlider("zOffsetFilter",0,50,&zOffsetFilter);
	gui->addSpacer(GUI_WIDTH,2);
	gui->addSlider("radiusRefine",0,100,&radiusRefine);
	gui->addSpacer(GUI_WIDTH,2);
	gui->addSlider("thresholdDistance",0,30,&thresholdDistance);
	gui->addSpacer(GUI_WIDTH,2);
	gui->addSlider("Segmentation radius",0,50,&radius);
	gui->addSpacer(GUI_WIDTH,2);
	gui->addSlider("minHeight",0,100,&minHeight);
	gui->addSpacer(GUI_WIDTH,2);
	gui->addSlider("qualityCloud",1,4,qualityCloud)->setLabelPrecision(0);
	gui->addSpacer(GUI_WIDTH,2);
	gui->addSlider("areaHalf",0,50,areaHalf)->setLabelPrecision(0);
	gui->addSpacer(GUI_WIDTH,2);
	gui->addSlider("aliveFrames",0,30,aliveFrames)->setLabelPrecision(0);
	gui->addSpacer(GUI_WIDTH,2);
	gui->addSlider("cursorSpeedThreshold",0,1,&cursorSpeedThreshold);
	gui->addSpacer(GUI_WIDTH,2);

	//gui->addWidget(new ofxUILabelButton(20, 560, 100, false, "Save settings", OFX_UI_FONT_MEDIUM));
	gui->addLabelButton("Save settings", false);
	
	gui->addWidget(new ofxUILabelButton(220, 520, 100, false, "Calibrate", OFX_UI_FONT_MEDIUM));
	ofxUISlider * kinectAngleSlider = new ofxUISlider("Kinect tilt angle",-27,27,angle, 300, OFX_UI_GLOBAL_SLIDER_HEIGHT, 390, 520);
	gui->addWidget(kinectAngleSlider);
	kinectAngleSlider->setLabelPrecision(0);
	gui->addWidget(new ofxUILabelButton(760, 520, 100, false, "Create filter", OFX_UI_FONT_MEDIUM));

	gui->addWidget(new ofxUILabelButton(109, 520, 97, false, "Show 3D box", OFX_UI_FONT_MEDIUM));
	gui->addWidget(new ofxUILabelButton(5, 520, 97, false, "Freeze", OFX_UI_FONT_MEDIUM));
	gui->addWidget(new ofxUILabelButton(5, 490, 202, false, "Test fullscreen", OFX_UI_FONT_MEDIUM));

	gui->autoSizeToFitWidgets();

	gui->setVisible(true);
	ofAddListener(gui->newGUIEvent,this,&touchApp::guiEvent);	
}

//--------------------------------------------------------------
void touchApp::update() {
	
	if (isFreeze) return;

	//if(isDraw3D){
	//	
	//	bool state = cam.getMouseInputEnabled();
	//	ofRectangle camArea(IMAGE_DRAW_OFFSET_X, IMAGE_DRAW_OFFSET_Y, 640, 480);

	//	if(camArea.inside(mouseX, mouseY)){
	//		if(!state) cam.enableMouseInput();
	//	}

	//	else{
	//		if(state) cam.disableMouseInput();
	//	}
	//	
	//}
	
	kinect.update();
		
	if(kinect.isOpened() && kinect.isFrameNew()){
		
		if(isCalibrating) depthImage.setFromPixels(kinect.getVideoPixels());
		
		else{

			depthImage.setFromPixels(kinect.getDepthPixels());
		
			int startTime = ofGetElapsedTimeMillis();
		
			cloudFilter.update(kinect,qualityCloud,cloud); 
		
			getTouchPoints(cloud, radius, radiusRefine, thresholdDistance, minHeight, areaHalf);
			get2DTouchPoints(touchPoints, touchPoints2d);

			calcTime = ofGetElapsedTimeMillis() - startTime;

			tracker.track(touchPoints2d, aliveFrames, cursorSpeedThreshold);
		}

		vector<ofxCvBlob*>  blobs;

		for(int i = 0; i < tracker.getTouchList().size(); i++){

			blobs.push_back(&tracker.getTouchList()[i].blob);
		}

		#ifdef USE_TUIO
		tuio.initFrame(TUIO::TuioTime::getSessionTime());
		tuio.stopUntouchedMovingCursors();
		tuio.commitFrame();
		#endif
	}
}

//--------------------------------------------------------------
void touchApp::draw() {

	if(testFullScreen){
		
		for(int i = 0; i < tracker.getTouchList().size(); i++){
					
			ofVec2f pos = tracker.getTouchList()[i].blob.centroid;
				
			//ofCircle(pos.x*boundDim.x+boundPos.x+IMAGE_DRAW_OFFSET_X, pos.y*boundDim.y+boundPos.y+IMAGE_DRAW_OFFSET_Y, 5);
			ofCircle(pos.x*ofGetScreenWidth(), pos.y*ofGetScreenHeight(), 5);
			ofDrawBitmapString(ofToString(tracker.getTouchList()[i].id), pos.x*ofGetScreenWidth() + 10, pos.y*ofGetScreenHeight() - 10);
		}
	}

	else{

		gui->draw();

		if(isDraw3D){
		
			draw3D();
		}

		else{
			ofSetColor(255);
			depthImage.draw(IMAGE_DRAW_OFFSET_X, IMAGE_DRAW_OFFSET_Y, 640, 480);
			//ofFill();
			//ofSetColor(255,0,0);
			//if (isTouch) ofRect(640,0,128,128);

			if(isCalibrating){
				drawCalibration();
			}

			else{
				
				ofEnableAlphaBlending();
				//ofSetColor(0, 75, 0);
				//ofRect(boundPos.x - 3 + IMAGE_DRAW_OFFSET_X, boundPos.y - 3 + IMAGE_DRAW_OFFSET_Y, boundDim.x + 6, boundDim.y + 6); 
				ofSetColor(0,255,0, 20);
				ofRect(boundPos.x + IMAGE_DRAW_OFFSET_X, boundPos.y + IMAGE_DRAW_OFFSET_Y, boundDim.x, boundDim.y); 
				ofDisableAlphaBlending();
			
				ofSetColor(255, 100, 100);
				for(int i = 0; i < tracker.getTouchList().size(); i++){
					
					ofVec2f pos = tracker.getTouchList()[i].blob.centroid;
				
					ofCircle(pos.x*boundDim.x+boundPos.x+IMAGE_DRAW_OFFSET_X, pos.y*boundDim.y+boundPos.y+IMAGE_DRAW_OFFSET_Y, 5);
					ofDrawBitmapString(ofToString(tracker.getTouchList()[i].id), pos.x*boundDim.x+boundPos.x+IMAGE_DRAW_OFFSET_X + 10, pos.y*boundDim.y+boundPos.y+IMAGE_DRAW_OFFSET_Y-10);
				}

				//for(int i = 0; i < touchPoints.size(); i++){
				//		
				//	ofVec3f srcPos = touchPoints[i].pos3d;
				//	ofVec3f v = cloudFilter.nodeA - srcPos;
				//	ofVec3f normal = cloudFilter.getNormal().getNormalized();
				//	float dist = v.dot(normal);
				//	ofVec3f res = srcPos - dist*normal;
				//	//ofSetColor(255, 100, 100);
				//	//ofCircle(res, 5);
				//	float a = cloudFilter.nodeA.distance(cloudFilter.nodeB);
				//	float b = cloudFilter.nodeA.distance(res);
				//	float c = cloudFilter.nodeB.distance(res);
				//	float p = (a+b+c)/2;
				//	float square = sqrt(p * (p-a) * (p-b)* (p-c) );
				//	float h = 2 * square / a ;
				//	float w = sqrt(b*b - h*h);
				//	float wNorm = w / a;
				//	float d = cloudFilter.nodeA.distance(cloudFilter.nodeD);
				//	float hNorm = h / d;

				//	touchPoints2d.push_back(ofVec3f(wNorm, hNorm));

				//	ofSetColor(255);
				//	ofDrawBitmapString(ofToString(wNorm),900,165);
				//	ofDrawBitmapString(ofToString(hNorm),900,180);

				//	ofSetColor(255, 100, 100);
				//	ofCircle(wNorm*boundDim.x+boundPos.x+IMAGE_DRAW_OFFSET_X, hNorm*boundDim.y+boundPos.y+IMAGE_DRAW_OFFSET_Y, 5);
				//}
			}

			ofSetColor(255);
		}

		string s1 = "volume = "				+ ofToString(volumeFilter);
		string s2 = "offset = "				+ ofToString(zOffsetFilter);
		string s3 = "radius = "				+ ofToString(radius);
		string s4 = "thresholdDistance = "	+ ofToString(thresholdDistance);
		string s5 = "size screen(inch) = "	+ ofToString(sizeScreen);
		string s6 = "calc time(ms) = "		+ ofToString(calcTime);
		string s7 = "cloudQuality  = "		+ ofToString(qualityCloud);
		string s8 = "areaHalf  = "			+ ofToString(areaHalf);
		string s9 = "kinect angle  = "		+ ofToString(angle);
	
		ofDrawBitmapString(s1,900,15);
		ofDrawBitmapString(s2,900,30);
		ofDrawBitmapString(s3,900,45);
		ofDrawBitmapString(s4,900,60);
		ofDrawBitmapString(s5,900,75);
		ofDrawBitmapString(s6,900,90);
		ofDrawBitmapString(s7,900,105);
		ofDrawBitmapString(s8,900,120);
		ofDrawBitmapString(s9,900,135);

		ofDrawBitmapString("FPS:" + ofToString(ofGetFrameRate()),900,150);
	}
}

//--------------------------------------------------------------
void touchApp::draw3D(){
	
	fboCam.begin();
	ofClear(0, 0, 0, 255);

	glEnable(GL_DEPTH_TEST);
	ofRectangle camArea(0, 0, 640, 480);
		cam.begin(camArea);
			//cloudUtils.drawCloud(cloudFilter.getCentroid(), 4, kinect);
			cloudFilter.drawFilterCloud(cloudInFilter);

			ofPushMatrix();
				ofTranslate(cloudFilter.getCentroid());
				ofScale(1, -1, -1);

			//for(int i = 0; i < bBoxes.size(); i++){

			//	ofVec3f min = bBoxes[i].first;
			//	ofVec3f max = bBoxes[i].second;

			//	ofVec3f a = min;
			//	ofVec3f b = ofVec3f(min.x, min.y, max.z);
			//	ofVec3f c = ofVec3f(min.x, max.y, min.z);
			//	ofVec3f d = ofVec3f(min.x, max.y, max.z);
			//	ofVec3f e = ofVec3f(max.x, min.y, min.z);
			//	ofVec3f f = ofVec3f(max.x, min.y, max.z);
			//	ofVec3f g = ofVec3f(max.x, max.y, min.z);
			//	ofVec3f h = max;
			//	
			//	ofSetLineWidth(2);
			//	ofSetColor(0, 0, 255);
			//	ofLine(a, b);
			//	ofLine(b, d);
			//	ofLine(c, d);
			//	ofLine(c, a);

			//	ofLine(e, g);
			//	ofLine(g, h);
			//	ofLine(f, h);
			//	ofLine(f, e);

			//	ofLine(a, e);
			//	ofLine(b, f);
			//	ofLine(c, g);
			//	ofLine(d, h);

			//	ofSetColor(255, 0, 0);
	
			//	ofSetLineWidth(1);
			//	ofSetColor(255);	
			//}

			ofPopMatrix();

			cloudFilter.drawSpatialBoxFilter();
		cam.end();
		cam.draw();
	glDisable(GL_DEPTH_TEST);
	fboCam.end();

	fboCam.draw(IMAGE_DRAW_OFFSET_X, IMAGE_DRAW_OFFSET_Y);
}

//--------------------------------------------------------------
void touchApp::drawCalibration(){
	
	// Калибровочные данные
	ofEnableAlphaBlending();
	ofSetColor(0, 0, 0, 150);
	ofRect(IMAGE_DRAW_OFFSET_X, IMAGE_DRAW_OFFSET_Y, 640, 480);
	ofSetColor(255);
	depthImage.drawSubsection(boundPos.x + IMAGE_DRAW_OFFSET_X, boundPos.y + IMAGE_DRAW_OFFSET_Y, boundDim.x, boundDim.y, boundPos.x, boundPos.y, boundDim.x, boundDim.y);

	ofVec2f pA(boundPos.x + IMAGE_DRAW_OFFSET_X, boundPos.y + IMAGE_DRAW_OFFSET_Y);
	ofVec2f pB(boundPos.x + boundDim.x + IMAGE_DRAW_OFFSET_X, boundPos.y + IMAGE_DRAW_OFFSET_Y);
	ofVec2f pC(boundPos.x + boundDim.x + IMAGE_DRAW_OFFSET_X, boundPos.y + boundDim.y + IMAGE_DRAW_OFFSET_Y);
	ofVec2f pD(boundPos.x + IMAGE_DRAW_OFFSET_X, boundPos.y + boundDim.y + IMAGE_DRAW_OFFSET_Y);

	ofCircle(pA, 5);
	ofCircle(pB, 5);
	ofCircle(pC, 5);
	ofCircle(pD, 5);
	ofLine(pA, pB);
	ofLine(pB, pC);
	ofLine(pC, pD);
	ofLine(pD, pA);

	ofDisableAlphaBlending();
}

//--------------------------------------------------------------
void touchApp::exit() {
	kinect.setAngle(0);
	kinect.close();
	delete gui;
}

//--------------------------------------------------------------
void touchApp::guiEvent(ofxUIEventArgs &e){
	
	string name = e.widget->getName(); 
	int kind = e.widget->getKind(); 

	if(kind == OFX_UI_WIDGET_LABELBUTTON){

		ofxUIButton * b = ((ofxUIButton *)e.widget);

		if (name == "Show 3D box" && b->getValue() == 0) {

			if(isDraw3D) b->getLabel()->setLabel("Show 3D box");
			else b->getLabel()->setLabel("Show depth");

			isDraw3D = !isDraw3D;
		}

		if (name == "Freeze" && b->getValue() == 0) {

			isFreeze = !isFreeze;
		}

		if (name == "Calibrate" && b->getValue() == 0) {

			if(isCalibrating){
				b->getLabel()->setLabel("Calibrate");
				kinect.close();
				setupKinect(false);
			}
			else{
				b->getLabel()->setLabel("Done");
				kinect.close();
				setupKinect(true);
			}

			isCalibrating = !isCalibrating;	
		}

		if (name == "Create filter" && b->getValue() == 0) {

			cloudFilter.create(boundPos,ofVec2f(boundPos.x + boundDim.x,boundPos.y),
				ofVec2f(boundPos.x + boundDim.x,boundPos.y + boundDim.y),
				ofVec2f(boundPos.x,boundPos.y + boundDim.y),
				kinect,4,zOffsetFilter,volumeFilter,100);

			sizeScreen = cloudFilter.getSizeScreen(kinect);
		}

		if (name == "Save settings" && b->getValue() == 0) {

			saveSettings();
		}

		if (name == "Test fullscreen" && b->getValue() == 0) {

			testFullScreen = !testFullScreen;
			ofSetFullscreen(testFullScreen);
		}
	}

	if(kind == OFX_UI_WIDGET_SLIDER_H){
		
		ofxUISlider * slider = (ofxUISlider *)e.widget;
		if(slider->getCreationName() == "Kinect tilt angle"){
			angle = slider->getScaledValue();
			kinect.setAngle(angle);
			slider->setValue(angle);
		}

		if(slider->getCreationName() == "qualityCloud"){
			qualityCloud = slider->getScaledValue();
			slider->setValue(qualityCloud);
		}

		if(slider->getCreationName() == "areaHalf"){
			areaHalf = slider->getScaledValue();
			slider->setValue(areaHalf);
		}
		
		if(slider->getCreationName() == "aliveFrames"){
			aliveFrames = slider->getScaledValue();
			slider->setValue(aliveFrames);
		}
	}
}

//--------------------------------------------------------------
void touchApp::touchAdded(ofxCvBlob & blob){
	
	ofVec2f p = blob.centroid;

	#ifndef USE_TUIO
	p.x *= ofGetScreenWidth();
	p.y *= ofGetScreenHeight();
	
	move(p.x, p.y, false);
	clickDown();
	#else

	tuio.addTuioCursor(p.x,p.y);
	#endif
}

//--------------------------------------------------------------
void touchApp::touchMoved(ofxCvBlob & blob){
	
	ofVec2f p = blob.centroid;

	#ifndef USE_TUIO
	p.x *= ofGetScreenWidth();
	p.y *= ofGetScreenHeight();
	
	move(p.x, p.y, true);
	#else
	TUIO::TuioCursor * cur = tuio.getClosestTuioCursor(p.x, p.y);
	tuio.updateTuioCursor(cur,p.x,p.y);
	#endif
}

//--------------------------------------------------------------
void touchApp::touchDeleted(ofxCvBlob & blob){
	
	ofVec2f p = blob.centroid;
	#ifndef USE_TUIO
	p.x *= ofGetScreenWidth();
	p.y *= ofGetScreenHeight();

	move(p.x, p.y, false);
	clickUp();
	#else
	TUIO::TuioCursor * cur = tuio.getClosestTuioCursor(p.x, p.y);
	tuio.removeTuioCursor(cur);
	#endif
}

//--------------------------------------------------------------
void touchApp::saveSettings(){
	
	xml.setValue("angle", angle);
	xml.setValue("radiusRefine", radiusRefine);
	xml.setValue("thresholdDistance", thresholdDistance);
	xml.setValue("minHeight", minHeight);
	xml.setValue("areaHalf", areaHalf);
	xml.setValue("qualityCloud", qualityCloud);
	xml.setValue("volumeFilter", volumeFilter);
	xml.setValue("zOffsetFilter", zOffsetFilter);
	xml.setValue("radius", radius);
	xml.setValue("aliveFrames", aliveFrames);

	xml.setValue("boundX", boundPos.x);
	xml.setValue("boundY", boundPos.y);
	xml.setValue("boundW", boundDim.x);
	xml.setValue("boundH", boundDim.y);
	xml.saveFile();
}

//--------------------------------------------------------------
void touchApp::keyPressed (int key) {
	//	switch(key){
	//	case 'o':
	//	case 'O':
	//		kinect.open();
	//		break;
	//	case 'c':
	//	case 'C':
	//		kinect.close();
	//		break;
	//	case OF_KEY_UP:
	//		/*angle++;
	//		if(angle > 27){
	//			angle = 27;
	//		}
	//		kinect.setAngle(angle);*/
	//		volumeFilter += 5;
	//		cloudFilter.create(boundPos,ofVec2f(boundPos.x + boundDim.x,boundPos.y),ofVec2f(boundPos.x + boundDim.x,boundPos.y + boundDim.y),ofVec2f(boundPos.x,boundPos.y + boundDim.y),kinect,4,zOffsetFilter,volumeFilter,100);
	//		break;
	//	case OF_KEY_DOWN:
	//		/*angle--;
	//		if(angle < -27){
	//			angle = -27;
	//		}
	//		kinect.setAngle(angle);*/
	//		volumeFilter -= 5;
	//		cloudFilter.create(boundPos,ofVec2f(boundPos.x + boundDim.x,boundPos.y),ofVec2f(boundPos.x + boundDim.x,boundPos.y + boundDim.y),ofVec2f(boundPos.x,boundPos.y + boundDim.y),kinect,4,zOffsetFilter,volumeFilter,100);
	//		break;

	//	case OF_KEY_LEFT:
	//		zOffsetFilter -= 5;
	//		cloudFilter.create(boundPos,ofVec2f(boundPos.x + boundDim.x,boundPos.y),ofVec2f(boundPos.x + boundDim.x,boundPos.y + boundDim.y),ofVec2f(boundPos.x,boundPos.y + boundDim.y),kinect,4,zOffsetFilter,volumeFilter,100);
	//		break;

	//	case OF_KEY_RIGHT:
	//		zOffsetFilter += 5;
	//		cloudFilter.create(boundPos,ofVec2f(boundPos.x + boundDim.x,boundPos.y),ofVec2f(boundPos.x + boundDim.x,boundPos.y + boundDim.y),ofVec2f(boundPos.x,boundPos.y + boundDim.y),kinect,4,zOffsetFilter,volumeFilter,100);
	//		break;
	//}

	//if(key == '>') radius += 0.25;
	//if(key == '<') radius -= 0.25;
	//if (key == ' ' && kinect.isOpened()) {
	//	int startTime = ofGetElapsedTimeMillis();
	//	cloudFilter.create(boundPos,ofVec2f(boundPos.x + boundDim.x,boundPos.y),ofVec2f(boundPos.x + boundDim.x,boundPos.y + boundDim.y),ofVec2f(boundPos.x,boundPos.y + boundDim.y),kinect,4,zOffsetFilter,volumeFilter,100);
	//	sizeScreen = cloudFilter.getSizeScreen(kinect);
	//}
	//
	//if (key == '7') isFreeze = !isFreeze;

	//if (key == '1') {
	//	cout << cloudFilter.getSizeScreen(kinect) << endl;
	//}
	//
	//
	//if (key == '2') {
	//	thresholdDistance -= 2;
	//	cout << "thresholdDistance = " << thresholdDistance << endl;
	//}

	//if (key == '3') {
	//	thresholdDistance += 2;
	//	cout << "thresholdDistance = " << thresholdDistance << endl;
	//}
	//
	//if (key == '4') {
	//	qualityCloud -= 1;
	//	if (qualityCloud < 1) qualityCloud = 1;
	//}

	//if (key == '5') {
	//	qualityCloud += 1;
	//	if (qualityCloud > 8) qualityCloud = 8;
	//}
}

//--------------------------------------------------------------
int touchApp::getTouchPoints(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, double radius, float radiusRefine, float thresholdDistance, float minHeight, int areaHalf ){

	if(cloud->empty()) return 0;

	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
	tree->setInputCloud (cloud);

	std::vector<pcl::PointIndices> cluster_indices;
	pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
	ec.setClusterTolerance (radius); // 2cm
	ec.setMinClusterSize (5);
	ec.setMaxClusterSize (cloud->points.size());
	ec.setSearchMethod (tree);
	ec.setInputCloud (cloud);
	ec.extract (cluster_indices);

	cloud->clear();
	cloudInFilter.clear();
	bBoxes.clear();
	touchPoints.clear();
	
	/// Цвет облака.
	int colorID = 0;
	

	/// Перебираем найденые сегменты-облака.	
	for (int i = 0; i < cluster_indices.size(); i++){

		/// Ищем границы сегмента-облака
		subCloundBounding boundSubCloud;
		cloudFilter.getMinMaxPointToBox(*cloud.get(),cluster_indices[i].indices, boundSubCloud);

		/// Получаем высоту облака по отношению к системе координат поверхности
		if (cloudFilter.checkBoundCloudOnMinHeight(minHeight, boundSubCloud)){

			/// Получаем возможную точку касания.
			ofVec3f touchPos = cloudFilter.getTouch3DPointFromCloud(boundSubCloud,kinect,radiusRefine,areaHalf);
		
			/// Определяем есть ли касание.
			if (cloudFilter.isContactSurface(touchPos,thresholdDistance)){

				/// Производим уточнение точки.
				/// Считаем что касание есть!
	

				/// Фрагмент для debug и визуализации:		
				colorID++;
				ofColor c = ofColor(255 - (colorID*2+1)*30, 0 + (colorID*2+1)*30, 255 - (colorID*2+1)*30);
				/// Создаем бокс для найденного объекта-касания.
				bBoxes.push_back(pair<ofVec3f, ofVec3f>(boundSubCloud.minPoint,boundSubCloud.maxPoint));
				/// Переписываем найденные точки.							
				for (int j = 0; j < cluster_indices[i].indices.size(); j++) {
					cloudPoint p;	
					p.color = c;
					//p.color = ofColor(255, 255, 255);
					pcl::PointXYZ PclPoint = cloud->points[cluster_indices[i].indices[j]];
					p.pos3d = ofVec3f(PclPoint.x, PclPoint.y, PclPoint.z);
					cloudInFilter.push_back(p);
				}
				cloudPoint p;
				p.color = ofColor(0, 255, 0);
				//p.color = ofColor(255, 255, 255);
				//p.color = c;
				p.pos3d = touchPos;
				cloudInFilter.push_back(p);

				touchPoints.push_back(p);

				//cout << "TOUCH DIST = "  << cloudFilter.getDistanceFromPointToPlane(touchPos) << endl;
			 
			} 			
		}
	}

	return touchPoints.size();
}

//--------------------------------------------------------------
void touchApp::get2DTouchPoints(vector <cloudPoint>& touchPoints3D, vector<ofVec3f>& touchPoints2D){
	
	touchPoints2D.clear();

	for(int i = 0; i < touchPoints3D.size(); i++){
					
		ofVec3f srcPos = touchPoints3D[i].pos3d;
		ofVec3f v = cloudFilter.nodeA - srcPos;
		ofVec3f normal = cloudFilter.getNormal().getNormalized();
		float dist = v.dot(normal);
		ofVec3f res = srcPos - dist * normal;

		float a = cloudFilter.nodeA.distance(cloudFilter.nodeB);
		float b = cloudFilter.nodeA.distance(res);
		float c = cloudFilter.nodeB.distance(res);
		float p = (a + b + c) / 2;
		float square = sqrt(p * (p - a) * (p - b)* (p - c) );
		float h = 2 * square / a ;
		float w = sqrt(b * b - h * h);
		float wNorm = 1 - w / a;
		float d = cloudFilter.nodeA.distance(cloudFilter.nodeD);
		float hNorm = h / d;

		touchPoints2D.push_back(ofVec3f(wNorm, hNorm));
	}
}

//--------------------------------------------------------------
void touchApp::mouseMoved(int x, int y) {
}

//--------------------------------------------------------------
void touchApp::mouseDragged(int x, int y, int button){

	if(isCalibrating){
		// Движение интерактивной области на кадре
		calibrationMouseInputHandle(x, y);
	}
}

//--------------------------------------------------------------
void touchApp::calibrationMouseInputHandle(int x, int y){

	const int areaOffset = 10;
		
	ofRectangle pA;
	ofRectangle pB;
	ofRectangle pC;
	ofRectangle pD;
	pA.setFromCenter(boundPos.x + IMAGE_DRAW_OFFSET_X, boundPos.y + IMAGE_DRAW_OFFSET_Y, areaOffset, areaOffset);
	pB.setFromCenter(boundPos.x + boundDim.x + IMAGE_DRAW_OFFSET_X, boundPos.y + IMAGE_DRAW_OFFSET_Y, areaOffset, areaOffset);
	pC.setFromCenter(boundPos.x + boundDim.x + IMAGE_DRAW_OFFSET_X, boundPos.y + boundDim.y + IMAGE_DRAW_OFFSET_Y, areaOffset, areaOffset);
	pD.setFromCenter(boundPos.x + IMAGE_DRAW_OFFSET_X, boundPos.y + boundDim.y + IMAGE_DRAW_OFFSET_Y, areaOffset, areaOffset);

	ofVec2f lastMousePos(ofGetPreviousMouseX(), ofGetPreviousMouseY());
	float aDist = lastMousePos.distance(pA.getCenter());
	float bDist = lastMousePos.distance(pB.getCenter());
	float cDist = lastMousePos.distance(pC.getCenter());
	float dDist = lastMousePos.distance(pD.getCenter());

	ofVec2f currentMousePos(x, y);

	if(aDist <= bDist && aDist <= cDist && aDist <= dDist){
		boundPos += currentMousePos - lastMousePos;
		boundDim -= currentMousePos - lastMousePos;
	}

	if(bDist <= aDist && bDist <= cDist && bDist <= dDist){
			
		boundPos.y += currentMousePos.y - lastMousePos.y;
		boundDim.x += currentMousePos.x - lastMousePos.x;
		boundDim.y -= currentMousePos.y - lastMousePos.y;
	}

	if(cDist <= aDist && cDist <= bDist && cDist <= dDist){
		boundDim += currentMousePos - lastMousePos;
	}
		
	if(dDist <= aDist && dDist <= bDist && dDist <= cDist){
		boundPos.x += currentMousePos.x - lastMousePos.x;
		boundDim.x -= currentMousePos.x - lastMousePos.x;
		boundDim.y += currentMousePos.y - lastMousePos.y;
	}

	if(boundPos.x < 0) boundPos.x = 0;
	if(boundPos.x > 640) boundPos.x = 640;
	if(boundPos.y < 0) boundPos.y = 0;
	if(boundPos.y > 480) boundPos.y = 480;

	if(boundDim.x < 1) boundDim.x = 1;
	if(boundDim.x > 640) boundDim.x = 640;
	if(boundDim.y < 1) boundDim.y = 1;
	if(boundDim.y > 480) boundDim.y = 480;
}

//--------------------------------------------------------------
void touchApp::mousePressed(int x, int y, int button){
	//cam.setTarget(centroidMyCloud);

}

//--------------------------------------------------------------
void touchApp::mouseReleased(int x, int y, int button){
}

//--------------------------------------------------------------
void touchApp::windowResized(int w, int h){
}

