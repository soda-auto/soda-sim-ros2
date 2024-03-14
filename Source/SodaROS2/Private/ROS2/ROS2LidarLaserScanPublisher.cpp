// Copyright 2023 SODA.AUTO UK LTD. All Rights Reserved.

#include "Soda/ROS2/ROS2LidarLaserScanPublisher.h"
#include "Soda/VehicleComponents/Sensors/Base/LidarSensor.h"
#include "Soda/SodaApp.h"

UROS2LidarLaserScanPublisher::UROS2LidarLaserScanPublisher(const FObjectInitializer& ObjectInitializer)
	: Super(ObjectInitializer)
{
}

bool UROS2LidarLaserScanPublisher::Advertise(UVehicleBaseComponent* Parent)
{
	Shutdown();
	bScanIsValid = true;
	Publisher = FSodaROS2Module::Get().CreatePublisher<sensor_msgs::msg::LaserScan>(NodeNamespace, Topic, QoS);
	return Publisher.IsValid();
}

void UROS2LidarLaserScanPublisher::Shutdown()
{
	Publisher.Reset();
	Msg.ranges.clear();
	Msg.intensities.clear();
}

bool UROS2LidarLaserScanPublisher::Publish(float DeltaTime, const FSensorDataHeader& Header, const soda::FLidarSensorData& Scan)
{
	if (IsOk())
	{
		bScanIsValid = Scan.Size.IsSet() && Scan.Size->Y == 1 && Scan.Size->X > 0;

		if (!bScanIsValid)
		{
			UE_LOG(LogSoda, Error, TEXT("UROS2LidarLaserScanPublisher::Publish(%s): The size of scan is invalid "), *Topic);
			return false;
		}

		const auto Timestamp = soda::RawTimestamp<std::chrono::nanoseconds>(Header.Timestamp);

		//Msg.header.seq = Header.FrameIndex;
		Msg.header.stamp = ros2_ue_wrapper::FromNanoseconds(Timestamp);
		Msg.header.frame_id = TCHAR_TO_UTF8(*FrameID);

		Msg.angle_min = Scan.HorizontalAngleMin / 180.0 * M_PI;
		Msg.angle_max = Scan.HorizontalAngleMax / 180.0 * M_PI;;
		Msg.angle_increment = (Scan.HorizontalAngleMax - Scan.HorizontalAngleMin) / Scan.Size->X;
		Msg.time_increment = 0;
		Msg.scan_time = DeltaTime;
		Msg.range_min = Scan.RangeMin / 100;
		Msg.range_max = Scan.RangeMax / 100;
		Msg.ranges.resize(Scan.Points.Num());
		//Msg.intensities

		Msg.ranges.resize(Scan.Points.Num());
		for (int k = 0; k < Scan.Points.Num(); ++k)
		{
			Msg.ranges[k] = Scan.Points[k].Depth / 100.0;
		}

		Publisher->Publish(Msg);
		return true;
	}
	return false;
}

bool UROS2LidarLaserScanPublisher::IsOk() const
{ 
	return bScanIsValid && Publisher.IsValid() && ros2_ue_wrapper::Ok();
}

FString UROS2LidarLaserScanPublisher::GetRemark() const
{
	return Topic;
}