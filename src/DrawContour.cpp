// -*- C++ -*-
/*!
 * @file  DrawContour.cpp
 * @brief DrawContour RTC with common camera interface
 * @date $Date$
 *
 * $Id$
 */

#include "DrawContour.h"

// Module specification
// <rtc-template block="module_spec">
static const char* drawcontour_spec[] =
  {
    "implementation_id", "DrawContour",
    "type_name",         "DrawContour",
    "description",       "DrawContour RTC with common camera interface",
    "version",           "1.0.0",
    "vendor",            "Kenichi Ohara, Meijo University",
    "category",          "ImageProcessing",
    "activity_type",     "PERIODIC",
    "kind",              "DataFlowComponent",
    "max_instance",      "1",
    "language",          "C++",
    "lang_type",         "compile",
    // Configuration variables
    "conf.default.int_thickness", "2",
    "conf.default.ImageWindow", "on",
    // Widget
    "conf.__widget__.int_thickness", "text",
    "conf.__widget__.ImageWindow", "radio",
    // Constraints
    "conf.__constraints__.ImageWindow", "(on,off)",
    ""
  };
// </rtc-template>

/*!
 * @brief constructor
 * @param manager Maneger Object
 */
DrawContour::DrawContour(RTC::Manager* manager)
    // <rtc-template block="initializer">
  : RTC::DataFlowComponentBase(manager),
    m_InImageIn("InImage", m_InImage),
    m_OutImageOut("OutImage", m_OutImage),
    m_CameraCaptureServicePort("CameraCaptureService")

    // </rtc-template>
{
}

/*!
 * @brief destructor
 */
DrawContour::~DrawContour()
{
}



RTC::ReturnCode_t DrawContour::onInitialize()
{
  // Registration: InPort/OutPort/Service
  // <rtc-template block="registration">
  // Set InPort buffers
  addInPort("InImage", m_InImageIn);
  
  // Set OutPort buffer
  addOutPort("OutImage", m_OutImageOut);
  
  // Set service provider to Ports
  
  // Set service consumers to Ports
  m_CameraCaptureServicePort.registerConsumer("CameraCaptureService", "Img::CameraCaptureService", m_CameraCaptureService);
  
  // Set CORBA Service Ports
  addPort(m_CameraCaptureServicePort);
  
  // </rtc-template>

  // <rtc-template block="bind_config">
  // Bind variables and configuration variable
  bindParameter("int_thickness", m_int_thickness, "2");
  bindParameter("ImageWindow", m_ImageWindow, "on");
  // </rtc-template>
  
  return RTC::RTC_OK;
}

/*
RTC::ReturnCode_t DrawContour::onFinalize()
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t DrawContour::onStartup(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t DrawContour::onShutdown(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/


RTC::ReturnCode_t DrawContour::onActivated(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}


RTC::ReturnCode_t DrawContour::onDeactivated(RTC::UniqueId ec_id)
{
  image.release();
  src.release();
  dst.release();
  return RTC::RTC_OK;
}


RTC::ReturnCode_t DrawContour::onExecute(RTC::UniqueId ec_id)
{
  if(m_InImageIn.isNew()){
    m_InImageIn.read();

    fprintf(stderr, "tm.sec %ld\n", m_InImage.tm.sec);
    fprintf(stderr, "tm.nsec %ld\n", m_InImage.tm.nsec);

    width = m_InImage.data.image.width;
    height = m_InImage.data.image.height;
    channels = (m_InImage.data.image.format == CF_GRAY) ? 1 :
	 (m_InImage.data.image.format == CF_RGB) ? 3 :
	 (m_InImage.data.image.raw_data.length()/width/height);
    RTC_TRACE(("Capture image size %d x %d", width, height));
    RTC_TRACE(("Channels %d", channels));

    if(channels == 3)
	 image.create(height, width, CV_8UC3);
    else
	 image.create(height, width, CV_8UC1);

    for(int i=0; i<height; ++i){
	 memcpy(&image.data[i*image.step],
		   &m_InImage.data.image.raw_data[i*width*channels],
		   sizeof(unsigned char)*width*channels);
    }
    fprintf(stderr, "Intrinsic matrix element\n");
    for(int i=0; i<5; ++i){
	 fprintf(stderr, " %f",
		    m_InImage.data.intrinsic.matrix_element[i]);
	 RTC_TRACE(("Intrinsic matrix element[%d] %f",
			  i, m_InImage.data.intrinsic.matrix_element[i]));
    }
    fprintf(stderr, "\n");

    fprintf(stderr, "Distortion parameter\n");
    for(unsigned int i=0;
	   i<m_InImage.data.intrinsic.distortion_coefficient.length(); ++i){
	 fprintf(stderr, " %f",
		    m_InImage.data.intrinsic.distortion_coefficient[i]);
	 RTC_TRACE(("Distortion parameter[%d] %f",
			  i, m_InImage.data.intrinsic.distortion_coefficient[i]));
    }
    fprintf(stderr, "\n");


    /*********************In Put Part End*****************************/

    /*********************Image Processing Part Start*******************/

    if(channels == 3)
	 cv::cvtColor(image, src, CV_RGB2GRAY);
    else if(channels == 1){
	 src = image.clone();
	 channels = 3;
    }

    cv::threshold(src, src, 0, 255, cv::THRESH_BINARY|cv::THRESH_OTSU);

    dst = cv::Mat::zeros(src.rows, src.cols, CV_8UC3);

    src = src > 1;

    std::vector<std::vector<cv::Point> > contours;
    std::vector<cv::Vec4i> hierarchy;

    cv::findContours( src, contours, hierarchy,
				  CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE );

    // トップレベルにあるすべての輪郭を横断し，各連結成分を描きます．
    int idx = 0;
    for( ; idx >= 0; idx = hierarchy[idx][0] ){
	 cv::Scalar  color( rand()&255, rand()&255, rand()&255 );
	 cv::drawContours( dst, contours, idx, color, m_int_thickness, 8, hierarchy );
    }

    cv::cvtColor(dst, image, CV_BGR2RGB);

    /*********************Image Processing Part End*******************/

    /*********************Out Put Part Start**************************/

    setTimestamp(m_OutImage);
    m_OutImage.data.captured_time = m_OutImage.tm;

    // copy camera intrinsic matrix to Data Port
    m_OutImage.data.intrinsic.matrix_element[0] = m_InImage.data.intrinsic.matrix_element[0];
    m_OutImage.data.intrinsic.matrix_element[1] = m_InImage.data.intrinsic.matrix_element[1];
    m_OutImage.data.intrinsic.matrix_element[2] = m_InImage.data.intrinsic.matrix_element[2];
    m_OutImage.data.intrinsic.matrix_element[3] = m_InImage.data.intrinsic.matrix_element[3];
    m_OutImage.data.intrinsic.matrix_element[4] = m_InImage.data.intrinsic.matrix_element[4];
    m_OutImage.data.intrinsic.distortion_coefficient.length(m_InImage.data.intrinsic.distortion_coefficient.length());

    // copy distortion coefficient to Data Port
    for(int j = 0; j < 5; ++j){
	 m_OutImage.data.intrinsic.distortion_coefficient[j] = m_InImage.data.intrinsic.distortion_coefficient[j];
    }

    // copy TimedCameraImage data to Data Port
    m_OutImage.data.image.width = width;
    m_OutImage.data.image.height = height;
    m_OutImage.data.image.raw_data.length(width * height * channels);
    m_OutImage.data.image.format
	 = (channels == 3) ? Img::CF_RGB :
	   (channels == 1) ? Img::CF_GRAY : Img::CF_UNKNOWN;
    for(int i=0; i<height; ++i){
	 memcpy(&m_OutImage.data.image.raw_data[i*width*channels],
		   &image.data[i*image.step],
		   sizeof(unsigned char)*width*channels);
    }

    // send TimedCameraImage data
    m_OutImageOut.write();

    if(m_ImageWindow == "on"){
	 if(!dst.empty())
	   cv::imshow("DrawContour", dst);
	 cv::waitKey(5);
    }
  }
  return RTC::RTC_OK;
}

/*
RTC::ReturnCode_t DrawContour::onAborting(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t DrawContour::onError(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t DrawContour::onReset(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t DrawContour::onStateUpdate(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t DrawContour::onRateChanged(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/



extern "C"
{
 
  void DrawContourInit(RTC::Manager* manager)
  {
    coil::Properties profile(drawcontour_spec);
    manager->registerFactory(profile,
                             RTC::Create<DrawContour>,
                             RTC::Delete<DrawContour>);
  }
  
};


