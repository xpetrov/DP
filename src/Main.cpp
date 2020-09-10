#include <iostream>

#include <kinectfusion.h>
#include <opencv2/highgui.hpp>
#include <HIRO/HIRO.h>

#include "DepthCamera.h"

int main()
{
   hiro::SetAssetDirectory( "./Dependencies/HIRO/resources/" );
   hiro::SetIntermediateDirectory( "./temp/" );

   hiro::Initialize();

   while( hiro::IsOpen() )
   {
      hiro::Update();
   }

   hiro::Terminate();
   return 0;

   auto camera = std::make_shared< KinectV2 >();
   
   if( FAILED( camera->initialize() ) )
   {
      return 1;
   }

   //camera->showDepth();
   //return 0;
   
   // Get a global configuration (comes with default values) and adjust some parameters
   kinectfusion::GlobalConfiguration configuration;
   configuration.volume_size = make_int3( 256, 256, 256 );
   configuration.voxel_scale = 4.f;
   configuration.init_depth = 500.f;
   configuration.distance_threshold = 10.f;
   configuration.angle_threshold = 20.f;
   //configuration.icp_iterations = { 4, 5, 10 };
   configuration.use_output_frame = false;

   // Create a KinectFusion pipeline with the camera intrinsics and the global configuration
   kinectfusion::Pipeline pipeline{ camera->get_parameters(), configuration };

   // Then, just loop over the incoming frames
   //while( true ) {
   int good = 0;
   int bad = 0;
   //cv::namedWindow( "Pipeline Output" );
   for( int i = 0; i < 5; ++i )
   {
      camera->GetKinectFrames();
      // 1) Grab a frame from the data source
      InputFrame frame = camera->grab_frame();

      // 2) Have the pipeline fuse it into the global volume
      bool success = pipeline.process_frame( frame.depth_map, frame.color_map );
      if( !success )
         ++bad;
      else
         ++good;

      //imshow( "Pipeline Output", pipeline.get_last_model_frame() );
   }

   std::cout << "Succes:  " << good << std::endl;
   std::cout << "Failure: " << bad << std::endl;

   //return 0;

   // Retrieve camera poses
   auto poses = pipeline.get_poses();

   // Export surface mesh
   auto mesh = pipeline.extract_mesh();
   kinectfusion::export_ply( "data/mesh.ply", mesh );

   // Export pointcloud
   auto pointcloud = pipeline.extract_pointcloud();
   kinectfusion::export_ply( "data/pointcloud.ply", pointcloud );
}
