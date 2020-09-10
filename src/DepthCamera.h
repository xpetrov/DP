#include <data_types.h>

#include <Kinect.h>

#pragma once

// Safe release for interfaces
template<class Interface>
inline void SafeRelease( Interface*& pInterfaceToRelease )
{
   if( pInterfaceToRelease != NULL )
   {
      pInterfaceToRelease->Release();
      pInterfaceToRelease = NULL;
   }
}

/**
 * Represents a single input frame
 * Packages a depth map with the corresponding RGB color map
 * The depth map is expected to hold float values, the color map 8 bit RGB values
 */
struct InputFrame
{
   cv::Mat_< float > depth_map;
   cv::Mat_< cv::Vec3b > color_map;
};

using kinectfusion::CameraParameters;

class DepthCamera
{
public:
   virtual ~DepthCamera() = default;
   

   virtual InputFrame grab_frame() const = 0;
   virtual CameraParameters get_parameters() const = 0;
};

/*
 * For testing purposes. This camera simply loads depth frames stored on disk.
 */
class PseudoCamera : public DepthCamera
{
public:
   explicit PseudoCamera( const std::string& _data_path );
   ~PseudoCamera() override = default;

   InputFrame grab_frame() const override;
   CameraParameters get_parameters() const override;

private:
   std::string data_path;
   CameraParameters cam_params;
   mutable size_t current_index;
};

class KinectV2 : public DepthCamera
{
   static const UINT cDepthWidth = 512;
   static const UINT cDepthHeight = 424;
   static const UINT cDepthLength = cDepthHeight * cDepthWidth;

   static const UINT cColorWidth = 1920;
   static const UINT cColorHeight = 1080;
   static const UINT cColorLength = cColorHeight * cColorWidth;

public:
   KinectV2() = default;
   ~KinectV2() override = default;

   HRESULT initialize();
   InputFrame grab_frame() const override;
   CameraParameters get_parameters() const override;
   const char* get_StatusMessage() const;
   HRESULT GetKinectFrames();
   HRESULT GetDepthFrame();
   HRESULT GetColorFrame();

   void showColor();
   void showDepth();

private:
   void SetStatusMessage( const char* aStatusMessage );
   HRESULT CopyDepth( IDepthFrame* pDepthFrame );
   HRESULT CopyColor( IColorFrame* pColorFrame );

public:
   IKinectSensor* m_pSensor = nullptr;
   IDepthFrameReader* m_pDepthFrameReader = nullptr;
   IColorFrameReader* m_pColorFrameReader = nullptr;
   ICoordinateMapper* m_pMapper = nullptr;
   WAITABLE_HANDLE m_coordinateMappingChangedEvent;
   mutable BYTE m_pDepthBuffer[ cDepthLength * sizeof( UINT16 ) ];
   mutable BYTE m_pColorBuffer[ cColorLength * sizeof( RGBQUAD ) ];


private:
   CameraParameters cam_params{};
   char m_statusMessage[ 512 ];
};
