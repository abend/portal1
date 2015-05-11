#include "ofApp.h"
#include <cstdio>

//--------------------------------------------------------------
void ofApp::setup() {
	ofEnableSmoothing();
	ofSetVerticalSync(true);

	//ofSetLogLevel(OF_LOG_VERBOSE);

	// enable depth->video image calibration
	kinect.setRegistration(true);

	kinect.init();
	//kinect.init(true); // shows infrared instead of RGB video image
	//kinect.init(false, false); // disable video image (faster fps)

	kinect.open();		// opens first available kinect

	// print the intrinsic IR sensor values
	// if (kinect.isConnected()) {
	// 	ofLogNotice() << "sensor-emitter dist: " << kinect.getSensorEmitterDistance() << "cm";
	// 	ofLogNotice() << "sensor-camera dist:  " << kinect.getSensorCameraDistance() << "cm";
	// 	ofLogNotice() << "zero plane pixel size: " << kinect.getZeroPlanePixelSize() << "mm";
	// 	ofLogNotice() << "zero plane dist: " << kinect.getZeroPlaneDistance() << "mm";
	// }

	colorImg.allocate(kinect.width, kinect.height);
	grayImage.allocate(kinect.width, kinect.height);
	grayThreshNear.allocate(kinect.width, kinect.height);
	grayThreshFar.allocate(kinect.width, kinect.height);

	nearThreshold = 255;
	farThreshold = 132;
	// nearThreshold = 230;
	// farThreshold = 70;

	ofSetFrameRate(60);

	// zero the tilt on startup
	// angle = 0;
	// kinect.setCameraTiltAngle(angle);

	// start from the front
	bDrawPointCloud = false;
	bDrawDepth = true;
	bDrawContour = true;
	bDrawHelp = true;

	//haarFinder.setup("haarcascade_frontalface_default.xml");
	//haarFinder.setup("haarcascade_mcs_nose.xml");
	haarFinder.setup("haarcascade_frontalface_alt.xml");

	//defining the real world coordinates of the window which is being headtracked is important for visual accuracy
	windowWidth = 375.0f; // mm
	windowHeight = 305.0f;

	windowTopLeft = ofVec3f(-windowWidth / 2.0f,
							+windowHeight / 2.0f,
							0.0f);
	windowBottomLeft = ofVec3f(-windowWidth / 2.0f,
							   - windowHeight / 2.0f,
							   0.0f);
	windowBottomRight = ofVec3f(+windowWidth / 2.0f,
								-windowHeight / 2.0f,
								0.0f);

	// this sets the camera's distance from the object
	//camera.setDistance(100);
	camera.disableMouseInput();

	//camera.setupOffAxisViewPortal(windowTopLeft, windowBottomLeft, windowBottomRight);

	//camera.lookAt(ofVec3f(0, 0, 0));

	headPosition = ofVec3f(0, 0, 500.0f);
}

//--------------------------------------------------------------
void ofApp::update() {

	// ofBackground(100, 100, 100);

	kinect.update();

	// there is a new frame and we are connected
	if (kinect.isFrameNew()) {

		// load grayscale depth image from the kinect source
		grayImage.setFromPixels(kinect.getDepthPixels(), kinect.width, kinect.height);

		// we do two thresholds - one for the far plane and one for the near plane
		// we then do a cvAnd to get the pixels which are a union of the two thresholds
		grayThreshNear = grayImage;
		grayThreshFar = grayImage;
		grayThreshNear.threshold(nearThreshold, true);
		grayThreshFar.threshold(farThreshold);
		cvAnd(grayThreshNear.getCvImage(), grayThreshFar.getCvImage(), grayImage.getCvImage(), NULL);

		// update the cv images
		grayImage.flagImageChanged();

		// find contours which are between the size of 20 pixels and 1/3 the w*h pixels.
		// also, find holes is set to true so we will get interior contours as well....
		contourFinder.findContours(grayImage, 50, (kinect.width*kinect.height)/2, 1, false);

		if (contourFinder.nBlobs > 0) {
			ofxCvBlob blob = contourFinder.blobs.at(0);

			ofPoint centroid = blob.centroid;

			// float distance = kinect.getDistanceAt(centroid.x, centroid.y);
			ofVec3f newHeadPosition = kinect.getWorldCoordinateAt(centroid.x, centroid.y);
			if (newHeadPosition.x != 0.0f &&
				newHeadPosition.y != 0.0f &&
				newHeadPosition.z != 0.0f)
			{
				headPosition = newHeadPosition;
				headPosition.x *= -1.0f;
				headPosition.y *= -1.0f;
			}
		}

		// face finding
		//haarFinder.findHaarObjects(kinect.getPixelsRef());

		if (haarFinder.blobs.size() > 0) {
			//get the head position in camera pixel coordinates
			const ofxCvBlob & blob = haarFinder.blobs.front();
			float cameraHeadX = blob.centroid.x;
			float cameraHeadY = blob.centroid.y;
		
			//do a really hacky interpretation of this, really you should be using something better to find the head (e.g. kinect skeleton tracking)
		
			//since camera isn't mirrored, high x in camera means -ve x in world
			// float worldHeadX = ofMap(cameraHeadX, 0, video.getWidth(), windowBottomRight.x, windowBottomLeft.x);
		
			//low y in camera is +ve y in world
			// float worldHeadY = ofMap(cameraHeadY, 0, video.getHeight(), windowTopLeft.y, windowBottomLeft.y);
		
			//set position in a pretty arbitrary way
			//headPosition = ofVec3f(worldHeadX, worldHeadY, viewerDistance);
		}
	}
}

void drawPoint(int x, int y, float dist) {
	char buff[100];
	sprintf(buff, "%0.2f", dist);
	ofDrawBitmapString(buff, x, y);
}

void ofApp::draw() {
	ofBackground(80);

	ofSetDepthTest(true);

	// ofVec3f headPosition(50, 0, distance);

	camera.setPosition(headPosition);
	camera.setupOffAxisViewPortal(windowTopLeft, windowBottomLeft, windowBottomRight);
	//camera.lookAt(ofVec3f(0, 0, 0));

	camera.begin();
	// ofRotateX(ofRadToDeg(.5));
	// ofRotateY(ofRadToDeg(-.5));

	ofPushStyle();
	ofNoFill();
    ofDrawBox(100.0f);
	ofPopStyle();

	// top
    ofPushMatrix();
	ofTranslate(0, windowHeight / 2, 0);
	ofRotateZ(90);
	ofSetColor(0,255,0);
	ofDrawGridPlane(windowWidth / 2);
    ofPopMatrix();
	// bottom
    ofPushMatrix();
	ofTranslate(0, -windowHeight / 2, 0);
	ofRotateZ(90);
	ofSetColor(0,255,0);
	ofDrawGridPlane(windowWidth / 2);
    ofPopMatrix();
	// left
    ofPushMatrix();
	ofTranslate(-windowWidth / 2, 0, 0);
	ofRotateZ(0);
	ofSetColor(0,255,0);
	ofDrawGridPlane(windowHeight / 2);
    ofPopMatrix();
	// right
    ofPushMatrix();
	ofTranslate(windowWidth / 2, 0, 0);
	ofRotateZ(0);
	ofSetColor(0,255,0);
	ofDrawGridPlane(windowHeight / 2);
    ofPopMatrix();



	ofPushStyle();

	ofSetColor(255,0,0);
	ofFill();
	ofDrawBox(30);
	ofNoFill();
	ofSetColor(0);
	ofDrawBox(30);

	ofPushMatrix();
	ofTranslate(0,0,20);
	ofSetColor(0,0,255);
	ofFill();
	ofDrawBox(5);
	ofNoFill();
	ofSetColor(0);
	ofDrawBox(5);
	ofPopMatrix();

	ofPopStyle();

	camera.end();


	ofSetDepthTest(false);

	if (bDrawDepth) {
		ofPushStyle();
		ofSetColor(255);
		kinect.drawDepth(10, 10, kinect.width / 2, kinect.height / 2);
		ofPopStyle();
	}

	if (bDrawContour) {
		ofPushStyle();
		contourFinder.draw(10, 10, kinect.width / 2, kinect.height / 2);

		ofSetColor(255, 0, 0);
		for (int i = 0; i < contourFinder.nBlobs; i++) {
			ofxCvBlob blob = contourFinder.blobs.at(i);

			ofPoint centroid = blob.centroid;

			int x = 10 + (centroid.x / 2);
			int y = 10 + (centroid.y / 2);

			ofCircle(x, y, 5);

			drawPoint(x, y, kinect.getDistanceAt(centroid.x, centroid.y));
		}

		ofPopStyle();
	}

	ofPushStyle();
	ofSetColor(0, 255, 255);
	ofNoFill();
	for (unsigned int i = 0; i < haarFinder.blobs.size(); i++) {
		ofRectangle cur = haarFinder.blobs[i].boundingRect;
		ofRect(10 + cur.x / 2, 10 + cur.y / 2, cur.width / 2, cur.height / 2);
	}
	ofPopStyle();

	if (bDrawHelp) {
		// draw instructions
		ofSetColor(255, 255, 255);
		stringstream reportStream;

		// if (kinect.hasAccelControl()) {
		// 	reportStream << "accel is: " << ofToString(kinect.getMksAccel().x, 2) << " / "
		// 				 << ofToString(kinect.getMksAccel().y, 2) << " / "
		// 				 << ofToString(kinect.getMksAccel().z, 2) << endl;
		// } else {
		// 	reportStream << "Note: this is a newer Xbox Kinect or Kinect For Windows device," << endl
		// 				 << "motor / led / accel controls are not currently supported" << endl << endl;
		// }

		reportStream << "head pos: " << headPosition << endl
			// << "press p to switch between images and point cloud" << endl
					 << "set near threshold " << nearThreshold << " (press: + -)" << endl
					 << "set far threshold " << farThreshold << " (press: < >) num blobs found " << contourFinder.nBlobs
					 << ", fps: " << ofGetFrameRate() << endl
					 << "h: toggle help, c: toggle contour, d: toggle depth" << endl;

		// if (kinect.hasCamTiltControl()) {
		// 	reportStream << "press UP and DOWN to change the tilt angle: " << angle << " degrees" << endl
		// 				 << "press 1-5 & 0 to change the led mode" << endl;
		// }

		ofDrawBitmapString(reportStream.str(), 20, 700);
	}
}

void ofApp::drawPointCloud() {
	int w = 640;
	int h = 480;
	ofMesh mesh;
	mesh.setMode(OF_PRIMITIVE_POINTS);
	int step = 2;
	for(int y = 0; y < h; y += step) {
		for(int x = 0; x < w; x += step) {
			if(kinect.getDistanceAt(x, y) > 0) {
				mesh.addColor(kinect.getColorAt(x,y));
				mesh.addVertex(kinect.getWorldCoordinateAt(x, y));
			}
		}
	}
	glPointSize(3);
	ofPushMatrix();
	// the projected points are 'upside down' and 'backwards'
	ofScale(1, -1, -1);
	ofTranslate(0, 0, -1000); // center the points a bit
	ofEnableDepthTest();
	mesh.drawVertices();
	ofDisableDepthTest();
	ofPopMatrix();
}

//--------------------------------------------------------------
void ofApp::exit() {
	// kinect.setCameraTiltAngle(0); // zero the tilt on exit
	kinect.close();
}

//--------------------------------------------------------------
void ofApp::keyPressed (int key) {
	switch (key) {

	case 'f':
		ofToggleFullscreen();
		break;

	case 'p':
		bDrawPointCloud = !bDrawPointCloud;
		break;

	case 'h':
		bDrawHelp = !bDrawHelp;
		break;

	case 'd':
		bDrawDepth = !bDrawDepth;
		break;

	case 'c':
		bDrawContour = !bDrawContour;
		break;

	case '>':
	case '.':
		farThreshold ++;
		if (farThreshold > 255) farThreshold = 255;
		break;

	case '<':
	case ',':
		farThreshold --;
		if (farThreshold < 0) farThreshold = 0;
		break;

	case '+':
	case '=':
		nearThreshold ++;
		if (nearThreshold > 255) nearThreshold = 255;
		break;

	case '-':
		nearThreshold --;
		if (nearThreshold < 0) nearThreshold = 0;
		break;

	case 'w':
		kinect.enableDepthNearValueWhite(!kinect.isDepthNearValueWhite());
		break;

	case 'o':
		kinect.setCameraTiltAngle(angle); // go back to prev tilt
		kinect.open();
		break;

	// case 'c':
	// 	kinect.setCameraTiltAngle(0); // zero the tilt
	// 	kinect.close();
	// 	break;

	case '1':
		kinect.setLed(ofxKinect::LED_GREEN);
		break;

	case '2':
		kinect.setLed(ofxKinect::LED_YELLOW);
		break;

	case '3':
		kinect.setLed(ofxKinect::LED_RED);
		break;

	case '4':
		kinect.setLed(ofxKinect::LED_BLINK_GREEN);
		break;

	case '5':
		kinect.setLed(ofxKinect::LED_BLINK_YELLOW_RED);
		break;

	case '0':
		kinect.setLed(ofxKinect::LED_OFF);
		break;

	case OF_KEY_UP:
		angle++;
		if(angle>30) angle=30;
		kinect.setCameraTiltAngle(angle);
		break;

	case OF_KEY_DOWN:
		angle--;
		if(angle<-30) angle=-30;
		kinect.setCameraTiltAngle(angle);
		break;
	}
}

//--------------------------------------------------------------
void ofApp::mouseDragged(int x, int y, int button)
{}

//--------------------------------------------------------------
void ofApp::mousePressed(int x, int y, int button)
{}

//--------------------------------------------------------------
void ofApp::mouseReleased(int x, int y, int button)
{}

//--------------------------------------------------------------
void ofApp::windowResized(int w, int h)
{}
