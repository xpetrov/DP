#include <iostream>
#include <fstream>
#include <iomanip>
#include <NuiKinectFusionApi.h>

#include <opencv2/imgcodecs.hpp>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include "DepthCamera.h"

PseudoCamera::PseudoCamera( const std::string& _data_path ) :
   data_path{ _data_path }, cam_params{}, current_index{ 0 }
{
   std::ifstream cam_params_stream{ data_path + "seq_cparam.txt" };
   if( !cam_params_stream.is_open() )
      throw std::runtime_error{ "Camera parameters could not be read" };
   cam_params_stream >> cam_params.image_width >> cam_params.image_height;
   cam_params_stream >> cam_params.focal_x >> cam_params.focal_y;
   cam_params_stream >> cam_params.principal_x >> cam_params.principal_y;
};

InputFrame PseudoCamera::grab_frame() const
{
   std::stringstream depth_file;
   depth_file << data_path << std::setfill( '0' ) << std::setw( 5 ) << current_index << "-depth.png";
   std::stringstream color_file;
   color_file << data_path << std::setfill( '0' ) << std::setw( 5 ) << current_index << "-color.png";

   InputFrame frame{};
   cv::imread( depth_file.str(), -1 ).convertTo( frame.depth_map, CV_32FC1 );

   if( frame.depth_map.empty() ) {  // When this happens, we reached the end of the recording and have to
                                   // start at 0 again
      current_index = 0;
      depth_file = std::stringstream{};
      color_file = std::stringstream{};
      depth_file << data_path << "seq_depth" << std::setfill( '0' ) << std::setw( 5 ) << current_index << ".png";
      color_file << data_path << "seq_color" << std::setfill( '0' ) << std::setw( 5 ) << current_index << ".png";
      frame.depth_map = cv::imread( depth_file.str(), -1 );
      
      throw std::runtime_error("[INFO]: The end of the recording has been reached.");
   }

   frame.color_map = cv::imread( color_file.str() );

   ++current_index;

   return frame;
}

CameraParameters PseudoCamera::get_parameters() const
{
   return cam_params;
}

void KinectV2::SetStatusMessage( const char* aStatusMessage )
{
   strcpy_s( m_statusMessage, aStatusMessage );
}

HRESULT KinectV2::CopyDepth( IDepthFrame* pDepthFrame )
{
   if( NULL == pDepthFrame )
   {
      return E_INVALIDARG;
   }

   UINT nBufferSize = 0;
   UINT16* pBuffer = NULL;

   HRESULT hr = pDepthFrame->AccessUnderlyingBuffer( &nBufferSize, &pBuffer );
   if( FAILED( hr ) )
   {
      return hr;
   }
   
   memcpy( m_pDepthBuffer, pBuffer, nBufferSize * sizeof( UINT16 ) );

   return hr;
}

HRESULT KinectV2::CopyColor( IColorFrame* pColorFrame )
{
   HRESULT hr = S_OK;

   hr = pColorFrame->CopyConvertedFrameDataToArray( cColorWidth * cColorHeight * sizeof( RGBQUAD ), m_pColorBuffer, ColorImageFormat_Bgra );

   if( FAILED( hr ) )
   {
      SetStatusMessage( "Error copying color texture pixels." );
      hr = E_FAIL;
   }

   return hr;
}

const char* KinectV2::get_StatusMessage() const
{
   return m_statusMessage;
}

HRESULT KinectV2::initialize()
{
   HRESULT hr;

   hr = GetDefaultKinectSensor( &m_pSensor );
   if( FAILED( hr ) )
   {
      return hr;
   }

   if( m_pSensor )
   {
      // Initialize the Kinect and get the depth reader
      IDepthFrameSource* pDepthFrameSource = NULL;
      IColorFrameSource* pColorFrameSource = NULL;

      hr = m_pSensor->Open();

      if( SUCCEEDED( hr ) )
      {
         hr = m_pSensor->get_DepthFrameSource( &pDepthFrameSource );
      }

      if( SUCCEEDED( hr ) )
      {
         hr = pDepthFrameSource->OpenReader( &m_pDepthFrameReader );
      }

      // set default params first
      cam_params.image_width = NUI_DEPTH_RAW_WIDTH;
      cam_params.image_height = NUI_DEPTH_RAW_HEIGHT;
      cam_params.focal_x = NUI_KINECT_DEPTH_NORM_FOCAL_LENGTH_X;
      cam_params.focal_y = NUI_KINECT_DEPTH_NORM_FOCAL_LENGTH_Y;
      cam_params.principal_x =NUI_KINECT_DEPTH_NORM_PRINCIPAL_POINT_X;
      cam_params.principal_y = NUI_KINECT_DEPTH_NORM_PRINCIPAL_POINT_Y;

      if( SUCCEEDED( hr ) )
      {
         hr = m_pSensor->get_CoordinateMapper( &m_pMapper );
         /*
         if( SUCCEEDED( hr ) )
         {
            CameraIntrinsics intrinsics = {};

            hr = m_pMapper->GetDepthCameraIntrinsics( &intrinsics );

            if( SUCCEEDED( hr ) )
            {
               cam_params.image_width = cDepthWidth;
               cam_params.image_height = cDepthHeight;
               cam_params.focal_x = intrinsics.FocalLengthX / cDepthWidth;
               cam_params.focal_y = intrinsics.FocalLengthY / cDepthHeight;
               cam_params.principal_x = intrinsics.PrincipalPointX / cDepthWidth;
               cam_params.principal_y = intrinsics.PrincipalPointY / cDepthHeight;

               std::cout << cam_params.focal_x << " " << cam_params.focal_y << std::endl;
               std::cout << cam_params.principal_x << " " << cam_params.principal_y << std::endl;
            }
         }
         */
      }

      if( SUCCEEDED( hr ) )
      {
         hr = m_pMapper->SubscribeCoordinateMappingChanged( &m_coordinateMappingChangedEvent );
      }

      if( SUCCEEDED( hr ) )
      {
         hr = m_pSensor->get_ColorFrameSource( &pColorFrameSource );
      }

      if( SUCCEEDED( hr ) )
      {
         hr = pColorFrameSource->OpenReader( &m_pColorFrameReader );
      }

      SafeRelease( pDepthFrameSource );
      SafeRelease( pColorFrameSource );

      Sleep( 3000 );
   }

   if( !m_pSensor || FAILED( hr ) )
   {
      SetStatusMessage( "No ready Kinect found!" );
      return E_FAIL;
   }

   return hr;
}

InputFrame KinectV2::grab_frame() const
{
   cv::Mat colorMap( cColorHeight, cColorWidth, CV_8UC3 );

   BYTE* dstPtr = colorMap.data;
   BYTE* srcPtr = m_pColorBuffer;

   for( UINT i = 0; i < cColorLength; ++i )
   {
      *dstPtr++ = *( srcPtr + 2 ); // copy R
      *dstPtr++ = *( srcPtr + 1 ); // copy G
      *dstPtr++ = *( srcPtr );     // copy B
      srcPtr += 4;                 // skip A
   }
   //cv::cvtColor( t, t, cv::COLOR_BGR2RGB );
   //cv::flip( colorMap, colorMap, 1 );

   cv::Mat depthMap( cDepthHeight, cDepthWidth, CV_32FC1 );

   dstPtr = depthMap.data;
   srcPtr = m_pDepthBuffer;
   float depthValue = 0;

   for( UINT i = 0; i < cDepthLength; ++i )
   {
      depthValue = static_cast< float >( *reinterpret_cast< UINT16* >( srcPtr ) ) * .001f;
      *reinterpret_cast< float* >( dstPtr ) = depthValue;
      srcPtr += 2;
      dstPtr += 4;
   }
   //cv::flip( depthMap, depthMap, 1 );

   //memcpy( depthMap.data, m_pDepthBuffer, cDepthLength * sizeof( UINT16* ) );

   //std::cout << depthMap << std::endl;

   return InputFrame{ depthMap, colorMap };
   
   /*
   cv::Mat depthImg16U( cDepthHeight, cDepthWidth, CV_16U, m_pDepthBuffer );
   cv::Mat depth_image;
   depthImg16U.convertTo( depth_image, CV_32FC1 );
   //cv::flip( depth_image, depth_image, 1 );

   return InputFrame{ depth_image, colorMap };
   */
}

CameraParameters KinectV2::get_parameters() const
{
   return cam_params;
}

HRESULT KinectV2::GetKinectFrames()
{
   HRESULT hr = S_OK;

   hr = GetDepthFrame();
   
   hr = GetColorFrame();

   return hr;
}

HRESULT KinectV2::GetDepthFrame()
{
   HRESULT hr = S_OK;

   IDepthFrame* pDepthFrame = nullptr;

   hr = m_pDepthFrameReader->AcquireLatestFrame( &pDepthFrame );

   if( FAILED( hr ) )
   {
      SafeRelease( pDepthFrame );
      SetStatusMessage( "Kinect depth stream get frame call failed." );

      return hr;
   }

   hr = CopyDepth( pDepthFrame );

   SafeRelease( pDepthFrame );

   return hr;
}

HRESULT KinectV2::GetColorFrame()
{
   HRESULT hr = S_OK;

   IColorFrame* pColorFrame = nullptr;

   hr = m_pColorFrameReader->AcquireLatestFrame( &pColorFrame );

   if( FAILED( hr ) )
   {
      // Here we just do not integrate color rather than reporting an error
   }
   else
   {
      if( SUCCEEDED( hr ) )
      {
         CopyColor( pColorFrame );
      }

      SafeRelease( pColorFrame );
   }

   return hr;
}

void KinectV2::showColor()
{
   cv::Mat windowImage( cColorHeight, cColorWidth, CV_8UC4 );

   while( true )
   {
      GetColorFrame();

      memcpy( windowImage.data, m_pColorBuffer, cColorHeight * cColorWidth * sizeof( RGBQUAD ) );

      imshow( "Color", windowImage );
      
      cv::waitKey( 10 );
   }
}

void KinectV2::showDepth()
{
   cv::Mat windowImage( cDepthHeight, cDepthWidth, CV_8UC4 );

   while( true )
   {
      GetDepthFrame();

      BYTE* imageBufferPtr = reinterpret_cast< BYTE* >( windowImage.data );
      UINT16* pBuffU16 = reinterpret_cast< UINT16* >( m_pDepthBuffer );

      for( UINT16* p = pBuffU16; p < pBuffU16 + cDepthLength; ++p )
      {
         BYTE depthValue = static_cast< BYTE >( ( *p ) % 256 );
         *( imageBufferPtr++ ) = depthValue; // b
         *( imageBufferPtr++ ) = depthValue; // g
         *( imageBufferPtr++ ) = depthValue; // r
         *( imageBufferPtr++ ) = 0xff;       // a
      }

      imshow( "Depth", windowImage );

      cv::waitKey( 10 );
   }
}