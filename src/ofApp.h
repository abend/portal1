#pragma once

#include "ofMain.h"
#include "ofxOpenCv.h"
#include "ofxKinect.h"
#include "ofxCvHaarFinder.h"
#include "ofx3DModelLoader.h"
#include "ofxAssimpModelLoader.h"

// Windows users:
// You MUST install the libfreenect kinect drivers in order to be able to use
// ofxKinect. Plug in the kinect and point your Windows Device Manager to the
// driver folder in:
//
//     ofxKinect/libs/libfreenect/platform/windows/inf
//
// This should install the Kinect camera, motor, & audio drivers.
//
// You CANNOT use this driver and the OpenNI driver with the same device. You
// will have to manually update the kinect device to use the libfreenect drivers
// and/or uninstall/reinstall it in Device Manager.
//
// No way around the Windows driver dance, sorry.

class ofApp : public ofBaseApp {
	public:
	
	void setup();
	void update();
	void drawMesh();
	void draw();
	void exit();
	
	void drawCamera();
	
	void keyPressed(int key);
	void mouseDragged(int x, int y, int button);
	void mousePressed(int x, int y, int button);
	void mouseReleased(int x, int y, int button);
	void windowResized(int w, int h);

	ofVec3f calcHeadPosition();
	
	ofxKinect kinect;
	
	ofxCvColorImage colorImage;
	ofxCvGrayscaleImage grayImage;

	ofxCvGrayscaleImage depthImage; // grayscale depth image
	ofxCvGrayscaleImage depthThreshNear; // the near thresholded image
	ofxCvGrayscaleImage depthThreshFar; // the far thresholded image
	
	ofxCvContourFinder contourFinder;
	ofxCvHaarFinder haarFinder;
	
	bool bDrawColor;
	bool bDrawDepth;
	bool bDrawContour;
	bool bDrawHelp;
	
	int nearThreshold;
	int farThreshold;
	
	int angle;
	
	ofEasyCam camera;
	ofEasyCam previewCamera;
	bool usePreview;
	ofVboMesh window;
	ofVboMesh dots;


	//the view window is defined by 3 corners
	ofVec3f windowTopLeft;
	ofVec3f windowBottomLeft;
	ofVec3f windowBottomRight;

	ofVec3f headPosition;
	deque<ofPoint> headPositions;

	float windowWidth;
	float windowHeight;

	float headX;
	float headY;

	// ofx3DModelLoader loader;

	ofxAssimpModelLoader model;
	ofMesh mesh;
	ofLight	light;
};
