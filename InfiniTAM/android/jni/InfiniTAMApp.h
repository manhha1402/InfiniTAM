#include <stdlib.h>

#include "../../Engine/IMUSourceEngine.h"
#include "../../Engine/ImageSourceEngine.h"
#include "../../ITMLib/ITMLib.h"
#include "../../Utils/NVTimer.h"

class InfiniTAMApp {
	public:
	static InfiniTAMApp* Instance(void)
	{
		if (globalInstance==NULL) globalInstance = new InfiniTAMApp();
		return globalInstance;
	}

	InfiniTAMApp(void);
	~InfiniTAMApp(void);

	void InitGL();
	void ResizeGL(int newWidth, int newHeight);
	void RenderGL(void);

	bool ProcessFrame(void);

	private:
	static InfiniTAMApp *globalInstance;

	InfiniTAM::Engine::ImageSourceEngine *mImageSource;
	InfiniTAM::Engine::IMUSourceEngine *mImuSource;
	ITMLib::Objects::ITMLibSettings *mInternalSettings;
	ITMLib::Engine::ITMMainEngine *mMainEngine;

	ITMUChar4Image *inputRGBImage; ITMShortImage *inputRawDepthImage;
	ITMIMUMeasurement *inputIMUMeasurement;

	StopWatchInterface *timer_instant;
	StopWatchInterface *timer_average;

	static const int NUM_WIN = 3;
	Vector4f winPos[NUM_WIN];
	uint textureId[NUM_WIN];
	ITMMainEngine::GetImageType winImageType[NUM_WIN];
	ITMUChar4Image *outImage[NUM_WIN];
};

