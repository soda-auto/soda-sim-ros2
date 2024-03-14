// Copyright 2023 SODA.AUTO UK LTD. All Rights Reserved.

#include "Soda/ROS2/ROS2LidarPoiontCloudPublisher.h"
#include "Soda/VehicleComponents/Sensors/Base/LidarSensor.h"
#include "Soda/SodaApp.h"

UROS2LidarPoiontCloudPublisher::UROS2LidarPoiontCloudPublisher(const FObjectInitializer& ObjectInitializer)
	: Super(ObjectInitializer)
{
}

bool UROS2LidarPoiontCloudPublisher::Advertise(UVehicleBaseComponent* Parent)
{
	Shutdown();
	Publisher = FSodaROS2Module::Get().CreatePublisher<sensor_msgs::msg::PointCloud>(NodeNamespace, Topic, QoS);
	return Publisher.IsValid();
}

void UROS2LidarPoiontCloudPublisher::Shutdown()
{
	Publisher.Reset();
	Msg.points.clear();
}

bool UROS2LidarPoiontCloudPublisher::Publish(float DeltaTime, const FSensorDataHeader& Header, const soda::FLidarSensorData& Scan)
{
	if (IsOk())
	{
		const auto Timestamp = soda::RawTimestamp<std::chrono::nanoseconds>(Header.Timestamp);

		//Msg.header.seq = Header.FrameIndex;
		Msg.header.stamp = ros2_ue_wrapper::FromNanoseconds(Timestamp);
		Msg.header.frame_id = TCHAR_TO_UTF8(*FrameID);

		Msg.points.resize(Scan.Points.Num());

		for (int k = 0; k < Scan.Points.Num(); ++k)
		{
			auto& Src = Scan.Points[k];
			auto& Dst = Msg.points[k];

			Dst.x = Src.Location.X / 100;
			Dst.y = -Src.Location.Y / 100;
			Dst.z = Src.Location.Z / 100;
		}

		Publisher->Publish(Msg);
		return true;
	}
	return false;
}

bool UROS2LidarPoiontCloudPublisher::IsOk() const
{ 
	return Publisher.IsValid() && ros2_ue_wrapper::Ok();
}

FString UROS2LidarPoiontCloudPublisher::GetRemark() const
{
	return Topic;
}