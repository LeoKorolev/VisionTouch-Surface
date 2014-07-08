#include "touchApp.h"
#include "ofMain.h"
#include "ofAppGlutWindow.h"

//--------------------------------------------------------------
int main( ){

    ofAppGlutWindow window;
	ofSetupOpenGL(&window, 870,560, OF_WINDOW);

	ofRunApp( new touchApp());
}
