#include <jni.h>
#include "carclient.h"
#include <vehicles/car/api/CarRpcLibClient.hpp>

#include <opencv2/core.hpp>
#include <opencv2/core.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/features2d.hpp>
#include <vector>

using namespace msr::airlib;
using namespace std;
using namespace cv;

using std::cin;
using std::cout;
using std::endl;
using std::vector;

typedef ImageCaptureBase::ImageRequest ImageRequest;
typedef ImageCaptureBase::ImageResponse ImageResponse;
typedef ImageCaptureBase::ImageType ImageType;


CarRpcLibClient *m_client;

JNIEXPORT jboolean JNICALL Java_com_pervasive_helloairsim_MainActivity_CarConnect(JNIEnv *env, jobject)
{
    m_client = new CarRpcLibClient("192.168.43.69");
    m_client->confirmConnection();
    m_client->enableApiControl(true);
    bool isEnabled = m_client->isApiControlEnabled();
    return isEnabled;
}

JNIEXPORT void JNICALL Java_com_pervasive_helloairsim_MainActivity_CarForward(JNIEnv *env, jobject)
{
    if (!m_client)
        return;
    CarApiBase::CarControls controls;
    controls.throttle = 0.5f;
    controls.steering = 0.0f;
    m_client->setCarControls(controls);
}

JNIEXPORT void JNICALL Java_com_pervasive_helloairsim_MainActivity_GetImage(JNIEnv *env, jobject, jlong addrImg)
{
    // Check if a client has been instantiated
    if (!m_client)
        return;

    cv::Mat& img = *(Mat*)addrImg;
    m_client->confirmConnection();

    std::vector<ImageRequest> request = { ImageRequest("0", ImageType::Scene, false)};

    const std::vector<ImageResponse>& response = m_client->simGetImages(request);


    assert(response.size() > 0);
    if(response.size() > 0) {
        img = imdecode(response.at(0).image_data_uint8, ImreadModes::IMREAD_COLOR);
    }
   return;
}
