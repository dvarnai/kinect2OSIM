#include <stdio.h>
#include <chrono>
#include <vector>
#include <string>
#include <sstream>
#include <fstream>
#include <iomanip>
#include <Windows.h>
#include <shobjidl.h> 
#include <Ole2.h>

#include <gl/GL.h>
#include <gl/GLU.h>
#include <gl/freeglut.h>

#include <Kinect.h>

#define width 1920
#define height 1080

// OpenGL Variables
GLuint textureId;              // ID of the texture to contain Kinect RGB Data
GLubyte data[width * height * 4];  // BGRA array containing the texture data
Joint joints[JointType_Count];
INT64 lastJoints = 0;
UINT64 trackedBodyId = ULLONG_MAX;

const GLfloat GL_COLOR_RED[3] = { 1.0, 0.0, 0.0 };

// Kinect variables
IKinectSensor* sensor;         // Kinect sensor
IMultiSourceFrameReader* reader;     
ICoordinateMapper* mapper;

const char* labels[] = {
	"SpineBase",
	"SpineMid",
	"Neck",
	"Head",
	"ShoulderLeft",
	"ElbowLeft",
	"WristLeft",
	"HandLeft",
	"ShoulderRight",
	"ElbowRight",
	"WristRight",
	"HandRight",
	"HipLeft",
	"KneeLeft",
	"AnkleLeft",
	"FootLeft",
	"HipRight",
	"KneeRight",
	"AnkleRight",
	"FootRight",
	"SpineShoulder",
	"HandTipLeft",
	"ThumbLeft",
	"HandTipRight",
	"ThumbRight",
	NULL
};

bool isRecording = false;
std::vector<CameraSpacePoint*> frames;

INT64 getNowMs() {
	return std::chrono::duration_cast<std::chrono::milliseconds>
		(std::chrono::system_clock::now().time_since_epoch()).count();
}

HRESULT showFileSaveDialog(wchar_t* outFilePath, size_t len)
{
	HRESULT hr = CoInitializeEx(NULL, COINIT_APARTMENTTHREADED |
		COINIT_DISABLE_OLE1DDE);
	COMDLG_FILTERSPEC fileTypes[] =
	{
		{L"Marker Files", L"*.trc"},
	};
	if (SUCCEEDED(hr))
	{
		IFileOpenDialog* pfd;
		hr = CoCreateInstance(CLSID_FileSaveDialog, NULL, CLSCTX_ALL,
			IID_IFileSaveDialog, reinterpret_cast<void**>(&pfd));

		if (SUCCEEDED(hr))
		{
			hr = pfd->SetFileTypes(1, fileTypes);
			if (SUCCEEDED(hr))
			{
				hr = pfd->SetDefaultExtension(L"trc");
				if (SUCCEEDED(hr))
				{
					hr = pfd->Show(NULL);
					if (SUCCEEDED(hr))
					{
						IShellItem* pItem;
						hr = pfd->GetResult(&pItem);
						if (SUCCEEDED(hr))
						{
							PWSTR pszFilePath;
							hr = pItem->GetDisplayName(SIGDN_FILESYSPATH, &pszFilePath);

							if (SUCCEEDED(hr))
							{
								wcsncpy_s(outFilePath, len, pszFilePath, len);
								CoTaskMemFree(pszFilePath);
							}
							pItem->Release();
						}
					}
				}
			}
			pfd->Release();
		}
		CoUninitialize();
	}
	return hr;
}

// Save motion frame for recording
void saveMotionFrame() {
	CameraSpacePoint* current = new CameraSpacePoint[JointType_Count];
	for (unsigned int i = 0; i < JointType_Count; ++i) {
		current[i] = joints[i].Position;
	}
	frames.push_back(current);
}

void releaseFrames() {
	for (auto frame : frames) {
		delete frame;
	}
	frames.clear();
}

void saveRecording() {
	
	// Show file dialog
	wchar_t path[MAX_PATH];
	showFileSaveDialog(path, sizeof(path));

	// Open file
	std::ofstream out(path);
	if (!out) {
		std::wostringstream text;
		text <<  "Failed to save recording to " << path;
		MessageBoxW(NULL, text.str().c_str(), L"Error saving file", MB_OK);
	}

	// Save trc format
	out << "PathFileType\t4\t(X/Y/Z)\toutput.trc\n";
	out << "DataRate\tCameraRate\tNumFrames\tNumMarkers\tUnits\tOrigDataRate\tOrigDataStartFrame\tOrigNumFrames\n";
	out << "30\t30\t" << frames.size() << "\t" << JointType_Count << "\tmm\t30\t0\t" << frames.size() << "\n";
	out << "Frame#\tTime";

	// Write labels
	unsigned int i = 0;
	while (labels[i++] != NULL) {
		out << "\t\t\t" << labels[i];
	}
	// Write unknown labels if there are more (shouldn't happen)
	for (; i < JointType_Count; ++i) {
		out << "\t\t\tunkJoint" << i;
	}
	out << "\n";

	// Write XYZ
	out << "\t";
	for (i = 1; i <= JointType_Count; ++i) {
		out << "\tX" << i << "\tY" << i << "\tZ" << i;
	}
	out << "\n";

	// Write data
	out << std::setprecision(5);
	for (i = 0; i < frames.size(); ++i) {
		out << i << "\t" << i * (1 / 30);
		for (auto j = 0; j < JointType_Count; ++j) {
			out << "\t" << frames[i][j].X << "\t" << frames[i][j].Y << "\t" << frames[i][j].Z;
		}
	}

	out.close();
	releaseFrames();
}

// Initialise kinect for colour and skeleton data
bool initKinect() {
	if (FAILED(GetDefaultKinectSensor(&sensor))) {
		return false;
	}
	if (sensor) {
		sensor->Open();
		sensor->get_CoordinateMapper(&mapper);
		sensor->OpenMultiSourceFrameReader(
			  FrameSourceTypes::FrameSourceTypes_Color
			| FrameSourceTypes::FrameSourceTypes_Body,
			&reader);
		return true;
	}
	else {
		return false;
	}
}

// Retrieve colour data from multi source frame
void getColorData(IMultiSourceFrame* frame) {

	IColorFrameReference* colorframeref = NULL;
	IColorFrame* colorframe = NULL;

	if (SUCCEEDED(frame->get_ColorFrameReference(&colorframeref))
		&& SUCCEEDED(colorframeref->AcquireFrame(&colorframe))) {

		colorframe->CopyConvertedFrameDataToArray(width * height * 4, data, ColorImageFormat_Bgra);
	}

	if (colorframe) colorframe->Release();
	if (colorframeref) colorframeref->Release();
}

// Retrieve skeleton data from multi source frame
void getSkeletonData(IMultiSourceFrame* frame) {

	IBodyFrameReference* bodyframeref = NULL;
	IBodyFrame* bodyframe = NULL;
	IBody* body[BODY_COUNT] = { 0 };
	BOOLEAN tracked;
	UINT64 trackingId;

	if (SUCCEEDED(frame->get_BodyFrameReference(&bodyframeref))
		&& SUCCEEDED(bodyframeref->AcquireFrame(&bodyframe))
		&& bodyframe != NULL
		&& SUCCEEDED(bodyframe->GetAndRefreshBodyData(BODY_COUNT, body))) {

		int bodyIdx = -1;
		int firstTracked = -1;
		int firstTrackedId = -1;
		for (int i = 0; i < BODY_COUNT; ++i) {
			body[i]->get_IsTracked(&tracked);
			body[i]->get_TrackingId(&trackingId);

			// Get already tracked body, if missing use first tracked found
			if (tracked) {
				if (firstTracked == -1) {
					firstTracked = i;
					firstTrackedId = trackingId;
				}
				if (trackedBodyId == ULLONG_MAX) trackedBodyId = trackingId;
				if (trackedBodyId == trackingId) {
					bodyIdx = i;
					break;
				}
			}
		}

		// Tracked body missing, use first found
		if (bodyIdx == -1 && firstTracked != -1) {
			bodyIdx = firstTracked;
			trackedBodyId = firstTrackedId;
		}

		if (bodyIdx != -1) {
			body[bodyIdx]->GetJoints(JointType_Count, joints);
			lastJoints = getNowMs();

			// Save skeleton data if recording
			if (isRecording) {
				saveMotionFrame();
			}
		}
	}

	if (bodyframe) bodyframe->Release();
	if (bodyframeref) bodyframeref->Release();
}

void getKinectData(GLubyte* dest) {
	IMultiSourceFrame* frame = NULL;

	if (SUCCEEDED(reader->AcquireLatestFrame(&frame))) {
		getColorData(frame);
		getSkeletonData(frame);
	}

	if (frame) frame->Release();
}

void drawLimb(int j1, int j2) {

	const CameraSpacePoint& lh = joints[j1].Position;
	const CameraSpacePoint& rh = joints[j2].Position;
	ColorSpacePoint lhh, rhh;

	mapper->MapCameraPointToColorSpace(lh, &lhh);
	mapper->MapCameraPointToColorSpace(rh, &rhh);
	
	int confidence = joints[j1].TrackingState + joints[j2].TrackingState;

	switch (confidence) {
	case 0:
		return;
	case 1:
		glColor3f(1.f, 0.7f, 0.f);
		break;
	case 2:
		glColor3f(1.f, 0.5f, 0.f);
		break;
	case 3:
		glColor3f(1.f, 0.3f, 0.f);
		break;
	case 4:
		glColor3f(1.f, 0.f, 0.f);
		break;
	}

	glVertex3f(lhh.X, lhh.Y, -0.5);
	glVertex3f(rhh.X, rhh.Y, -0.5);
}

void drawSkeletonData() {

	if (trackedBodyId != ULLONG_MAX) {

		glPushAttrib(GL_CURRENT_BIT);
		
		glPointSize(5.0);
		glLineWidth(5.0);
		glBegin(GL_LINES);

		drawLimb(JointType_HandRight, JointType_HandTipRight);
		drawLimb(JointType_HandRight, JointType_WristRight);
		drawLimb(JointType_ThumbRight, JointType_WristRight);
		drawLimb(JointType_WristRight, JointType_ElbowRight);
		drawLimb(JointType_ElbowRight, JointType_ShoulderRight);
		drawLimb(JointType_ShoulderRight, JointType_Neck);

		drawLimb(JointType_HandLeft, JointType_HandTipLeft);
		drawLimb(JointType_HandLeft, JointType_WristLeft);
		drawLimb(JointType_ThumbLeft, JointType_WristLeft);
		drawLimb(JointType_WristLeft, JointType_ElbowLeft);
		drawLimb(JointType_ElbowLeft, JointType_ShoulderLeft);
		drawLimb(JointType_ShoulderLeft, JointType_Neck);

		drawLimb(JointType_Neck, JointType_Head);
		drawLimb(JointType_Neck, JointType_SpineMid);
		drawLimb(JointType_SpineMid, JointType_SpineBase);

		drawLimb(JointType_SpineBase, JointType_HipRight);
		drawLimb(JointType_HipRight, JointType_KneeRight);
		drawLimb(JointType_KneeRight, JointType_AnkleRight);
		drawLimb(JointType_AnkleRight, JointType_FootRight);

		drawLimb(JointType_SpineBase, JointType_HipLeft);
		drawLimb(JointType_HipLeft, JointType_KneeLeft);
		drawLimb(JointType_KneeLeft, JointType_AnkleLeft);
		drawLimb(JointType_AnkleLeft, JointType_FootLeft);


		glEnd();
		glPopAttrib();
	}
}

void drawKinectData() {
	glBindTexture(GL_TEXTURE_2D, textureId);
	glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, width, height, GL_BGRA_EXT, GL_UNSIGNED_BYTE, (GLvoid*)data);
	glBegin(GL_QUADS);
	glTexCoord2f(0.0f, 0.0f);
	glVertex2f(0, 0);
	glTexCoord2f(1.0f, 0.0f);
	glVertex2f(width, 0);
	glTexCoord2f(1.0f, 1.0f);
	glVertex2f(width, height);
	glTexCoord2f(0.0f, 1.0f);
	glVertex2f(0, height);
	glEnd();
}

void RenderString(float x, float y, void* font, const char* string, const GLfloat rgb[3])
{
	glPushAttrib(GL_CURRENT_BIT);

	glColor3f(rgb[0], rgb[1], rgb[2]);
	glRasterPos2f(x, y);

	glutBitmapString(font, (const unsigned char*)string);
	glPopAttrib();
}

void draw() {

	getKinectData(data);

	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	drawKinectData();

	if (getNowMs() - lastJoints < 500) {
		drawSkeletonData();
	}
	
	// Draw recording status
	if (isRecording) {
		std::ostringstream text;
		text << "Recording " << frames.size() << " frames, press SPACE to save...";
		RenderString(20, 20, GLUT_BITMAP_HELVETICA_18, text.str().c_str(), GL_COLOR_RED);
	}

	glutSwapBuffers();
}

void execute() {
	glutMainLoop();
}

bool init(int argc, char* argv[]) {
	glutInit(&argc, argv);
	glutInitDisplayMode(GLUT_DEPTH | GLUT_DOUBLE | GLUT_RGBA);
	glutInitWindowSize(width, height);
	glutCreateWindow("Kinect SDK Tutorial");
	glutDisplayFunc(draw);
	glutIdleFunc(draw);
	return true;
}

void handleKeys(unsigned char key, int x, int y) {
	
	if (key == ' ') {
		isRecording = !isRecording;
		if (isRecording == false && frames.size() > 0) {
			saveRecording();
		}
	}
}

int main(int argc, char* argv[]) {

	if (!init(argc, argv)) return 1;
	if (!initKinect()) return 1;

	glutKeyboardFunc(handleKeys);

	// Initialize textures
	glGenTextures(1, &textureId);
	glBindTexture(GL_TEXTURE_2D, textureId);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA8, width, height,
		0, GL_BGRA_EXT, GL_UNSIGNED_BYTE, (GLvoid*)data);
	glBindTexture(GL_TEXTURE_2D, 0);

	// OpenGL setup
	glClearColor(0, 0, 0, 0);
	glClearDepth(1.0f);
	glEnable(GL_TEXTURE_2D);

	// Camera setup
    glViewport(0, 0, width, height);
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	glOrtho(0, width, height, 0, -5.0, 5.0);
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();

	// Main loop
	execute();
	return 0;
}
