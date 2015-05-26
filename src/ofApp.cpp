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
	bDrawDepth = false;
	bDrawContour = true;
	bDrawHelp = true;

	//haarFinder.setup("haarcascade_frontalface_default.xml");
	//haarFinder.setup("haarcascade_mcs_nose.xml");
	// haarFinder.setup("haarcascade_frontalface_alt.xml");

	// real world coordinates of the "window"
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

	camera.disableMouseInput();

	headPosition = ofVec3f(0, 0, 500.0f);

    //model.loadModel("astroBoy_walk.dae", true);
	model.loadModel("scene.dae");
	model.setRotation(0, -180, 0, 0, 1);
    //model.setLoopStateForAllAnimations(OF_LOOP_NORMAL);
    //model.playAllAnimations();

	usePreview = true;
	//previewCamera.setDistance(3.0f);
	previewCamera.setNearClip(0.01f);
	previewCamera.setFarClip(5000.0f);
	previewCamera.setPosition(000.0f, 000.0f, 1000.0f);
	previewCamera.lookAt(ofVec3f(0.0f, 0.0f, 0.0f));
	

}

//--------------------------------------------------------------
void ofApp::update() {
	kinect.update();

	if (kinect.isFrameNew()) {
		headPosition = calcHeadPosition();
	}

    //model.update();

    mesh = model.getCurrentAnimatedMesh(0);
}


ofVec3f ofApp::calcHeadPosition() {
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
		ofRectangle bbox = blob.boundingRect;

		float y = centroid.y;//bbox.y + 50;  // arbitrary
		float x = centroid.x;

		// search around where we think the head is till we find something
		// float perturb = 2.0f;
		// for (float dist = 0; dist == 0; dist = kinect.getDistanceAt(x, y)) {
		// 	x += perturb;
		// 	//perturb += -perturb;
		// 	perturb *= -1.1;
		// }

		// float dist = kinect.getDistanceAt(x, y);

		headX = x;
		headY = y;

		ofVec3f newHeadPosition = kinect.getWorldCoordinateAt(x, y);
		// if (newHeadPosition.x != 0.0f &&
		// 	newHeadPosition.y != 0.0f &&
		// 	newHeadPosition.z != 0.0f)
		// {
		//headPosition = newHeadPosition;
		newHeadPosition.x *= -1.0f;
		newHeadPosition.y *= -1.0f;
		// }

		return newHeadPosition;
	}

	return ofVec3f(0, 0, 500.0f);
}

void drawPoint(int x, int y, float dist) {
	char buff[100];
	sprintf(buff, "%0.2f", dist);
	ofDrawBitmapString(buff, x, y);
}

void drawBackground(float width, float height) {
	float halfw = width / 2;
	float halfh = height / 2;
	float biggest = max(halfw, halfh);

	ofPushStyle();
	ofSetColor(0,255,0);

	// top
    ofPushMatrix();
	ofTranslate(0, halfh, 0);
	ofRotateZ(90);
	ofDrawGridPlane(halfw);
    ofPopMatrix();

	// bottom
    ofPushMatrix();
	ofTranslate(0, -halfh, 0);
	ofRotateZ(90);
	ofDrawGridPlane(halfw);
    ofPopMatrix();

	// left
    ofPushMatrix();
	ofTranslate(-halfw, 0, 0);
	ofDrawGridPlane(halfh);
    ofPopMatrix();

	// right
    ofPushMatrix();
	ofTranslate(halfw, 0, 0);
	ofDrawGridPlane(halfh);
    ofPopMatrix();

	// back
    ofPushMatrix();
	ofTranslate(0, 0, -biggest);
	ofRotateY(90);
	ofDrawGridPlane(halfh);
    ofPopMatrix();

	ofPopStyle();

}

void drawScene() {
	ofPushStyle();

	ofSetLineWidth(3.0f);

	ofSetColor(255,0,0);
	ofFill();
	ofDrawBox(50);
	ofNoFill();
	ofSetColor(0);
	ofDrawBox(50);

	ofPushMatrix();
	ofTranslate(-100,-50,100);
	ofFill();
	ofSetColor(0,0,255);
	ofDrawBox(25);
	ofNoFill();
	ofSetColor(0);
	ofDrawBox(25);
	ofPopMatrix();

	ofPushMatrix();
	ofTranslate(100, 45/2 - 305.0f / 2, 0);
	ofFill();
	ofSetColor(128,0,255);
	ofDrawBox(45);
	ofNoFill();
	ofSetColor(0);
	ofDrawBox(45);
	ofPopMatrix();

	ofPopStyle();
}

void ofApp::drawMesh(){
    ofSetColor(255);
    
    ofEnableBlendMode(OF_BLENDMODE_ALPHA);
    
	ofEnableDepthTest();
    
    glShadeModel(GL_SMOOTH); //some model / light stuff
    //light.enable();
    //ofEnableSeparateSpecularLight();

    // ofPushMatrix();
    // ofTranslate(model.getPosition().x+100, model.getPosition().y, 0);
    //ofRotate(-mouseX, 0, 1, 0);
    // ofTranslate(-model.getPosition().x, -model.getPosition().y, 0);
    model.drawFaces();
    // ofPopMatrix();

   // if(ofGetGLProgrammableRenderer()){
   // 		glPushAttrib(GL_ALL_ATTRIB_BITS);
   // 		glPushClientAttrib(GL_CLIENT_ALL_ATTRIB_BITS);
   //  }
    glEnable(GL_NORMALIZE);

    // ofPushMatrix();
    // ofTranslate(model.getPosition().x-300, model.getPosition().y, 0);
    // ofRotate(-mouseX, 0, 1, 0);
    // ofTranslate(-model.getPosition().x, -model.getPosition().y, 0);
    
    // ofxAssimpMeshHelper & meshHelper = model.getMeshHelper(0);
    
    // ofMultMatrix(model.getModelMatrix());
    // ofMultMatrix(meshHelper.matrix);
    
    // ofMaterial & material = meshHelper.material;
    // if(meshHelper.hasTexture()){
    //     meshHelper.getTextureRef().bind();
    // }
    // material.begin();
    // mesh.drawWireframe();
    // material.end();
    // if(meshHelper.hasTexture()){
    //     meshHelper.getTextureRef().unbind();
    // }
    // ofPopMatrix();

    // if(ofGetGLProgrammableRenderer()){
    // 	glPopAttrib();
    // }
    
    ofDisableDepthTest();
    // light.disable();
    // ofDisableLighting();
    // ofDisableSeparateSpecularLight();
}

void ofApp::drawCamera() {
	ofVec3f campos = camera.getPosition();

	ofPushStyle();

	// camera body
	ofPushMatrix();
	ofTranslate(campos);
	ofSetColor(255);
	ofFill();
	ofDrawBox(10.0f);

	ofPopMatrix();

	// camera frustum
	window.clear();
	window.setMode(OF_PRIMITIVE_LINES);
	window.addVertex(campos);
	window.addVertex(windowTopLeft);
	window.addVertex(campos);
	window.addVertex(windowBottomLeft);
	window.addVertex(campos);
	window.addVertex(windowBottomRight);
	window.draw();

	ofPopStyle();
}

void ofApp::draw() {
	ofBackground(80);

	ofSetDepthTest(true);

	camera.setPosition(headPosition);
	camera.setupOffAxisViewPortal(windowTopLeft, windowBottomLeft, windowBottomRight);

	if (usePreview) {
		previewCamera.begin();
	} else {
		camera.begin();
	}

	drawBackground(windowWidth, windowHeight);
	drawScene();
	//loader.draw();

	//drawMesh();

	if (usePreview) drawCamera();

	if (usePreview) {
		previewCamera.end();
	} else {
		camera.end();
	}

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

	// ofPushStyle();
	// ofSetColor(0, 255, 255);
	// ofNoFill();
	// for (unsigned int i = 0; i < haarFinder.blobs.size(); i++) {
	// 	ofRectangle cur = haarFinder.blobs[i].boundingRect;
	// 	ofRect(10 + cur.x / 2, 10 + cur.y / 2, cur.width / 2, cur.height / 2);
	// }
	// ofPopStyle();

	ofPushStyle();
	ofSetColor(255, 255, 64);
	ofCircle(headX / 2 + 10, headY / 2 + 10, 5);
	//ofRect(headX - 2, headY - 2, headX + 2, headY + 2);
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

	case ' ':
		usePreview = !usePreview;
		break;

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
