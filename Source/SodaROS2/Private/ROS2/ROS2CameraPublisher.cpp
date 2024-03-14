// Copyright 2023 SODA.AUTO UK LTD. All Rights Reserved.

#include "Soda/ROS2/ROS2CameraPublisher.h"
#include "Soda/VehicleComponents/Sensors/Base/CameraSensor.h"
#include "Soda/SodaApp.h"

UROS2CameraPublisher::UROS2CameraPublisher(const FObjectInitializer& ObjectInitializer)
	: Super(ObjectInitializer)
{
}

bool UROS2CameraPublisher::Advertise(UVehicleBaseComponent* Parent)
{
	Publisher = FSodaROS2Module::Get().CreatePublisher<sensor_msgs::msg::Image>(NodeNamespace, Topic, QoS);
	return Publisher.IsValid();
}

void UROS2CameraPublisher::Shutdown()
{
	Publisher.Reset();
	Msg.data.clear();
}

bool UROS2CameraPublisher::Publish(float DeltaTime, const FSensorDataHeader& Header, const FCameraFrame& CameraFrame, const TArray<FColor>& BGRA8, uint32 ImageStride)
{
	if (IsOk())
	{
		const auto Timestamp = soda::RawTimestamp<std::chrono::nanoseconds>(Header.Timestamp);

		//Msg.header.seq = Header.FrameIndex;
		Msg.header.stamp = ros2_ue_wrapper::FromNanoseconds(Timestamp);
		Msg.header.frame_id = TCHAR_TO_UTF8(*FrameID);
		Msg.is_bigendian = false;
		Msg.width = CameraFrame.Width;
		Msg.height = CameraFrame.Height;
		Msg.step = CameraFrame.Width * CameraFrame.GetChannels() * soda::GetDataTypeSize(CameraFrame.GetDataType());
		Msg.data.resize(CameraFrame.ComputeRawBufferSize());
		CameraFrame.ColorToRawBuffer(BGRA8, Msg.data.data(), ImageStride);

		switch (CameraFrame.GetShader())
		{
			
		case ECameraSensorShader::ColorBGR8: 
		case ECameraSensorShader::SegmBGR8:
		case ECameraSensorShader::HdrRGB8:
		case ECameraSensorShader::CFA:
			Msg.encoding = sensor_msgs::image_encodings::BGR8;
			break;
		case ECameraSensorShader::Depth8: 
		case ECameraSensorShader::Segm8:
			Msg.encoding = sensor_msgs::image_encodings::MONO8;
			break;
		case ECameraSensorShader::DepthFloat32: 
			Msg.encoding = sensor_msgs::image_encodings::TYPE_32FC1;
			break;
		case ECameraSensorShader::Depth16: 
			Msg.encoding = sensor_msgs::image_encodings::MONO16; 
			break;
		default:
			check(0);

		}
		Publisher->Publish(Msg);
		return true;
	}
	return false;
}

bool UROS2CameraPublisher::IsOk() const
{ 
	return Publisher.IsValid() && ros2_ue_wrapper::Ok();
}

FString UROS2CameraPublisher::GetRemark() const
{
	return Topic;
}