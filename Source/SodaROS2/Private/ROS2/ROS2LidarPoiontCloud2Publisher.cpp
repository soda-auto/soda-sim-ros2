// Copyright 2023 SODA.AUTO UK LTD. All Rights Reserved.

#include "Soda/ROS2/ROS2LidarPoiontCloud2Publisher.h"
#include "Soda/VehicleComponents/Sensors/Base/LidarSensor.h"
#include "Soda/SodaApp.h"

UROS2LidarPoiontCloud2Publisher::UROS2LidarPoiontCloud2Publisher(const FObjectInitializer& ObjectInitializer)
	: Super(ObjectInitializer)
{
}

bool UROS2LidarPoiontCloud2Publisher::Advertise(UVehicleBaseComponent* Parent)
{
	Shutdown();

	Msg.fields.resize(3);
	Msg.fields[0].name = "x";
	Msg.fields[0].offset = 0;
	Msg.fields[0].datatype = sensor_msgs::msg::PointField::FLOAT32;
	Msg.fields[0].count = 1;
	Msg.fields[1].name = "y";
	Msg.fields[1].offset = 4;
	Msg.fields[1].datatype = sensor_msgs::msg::PointField::FLOAT32;
	Msg.fields[1].count = 1;
	Msg.fields[2].name = "z";
	Msg.fields[2].offset = 8;
	Msg.fields[2].datatype = sensor_msgs::msg::PointField::FLOAT32;
	Msg.fields[2].count = 1;
	//Msg.fields.push_back({ "intensity", 12, sensor_msgs::PointField::FLOAT32, 1 });
	Msg.is_bigendian = false;
	Msg.point_step = 12;
	Msg.is_dense = false;

	Publisher = FSodaROS2Module::Get().CreatePublisher<sensor_msgs::msg::PointCloud2>(NodeNamespace, Topic, QoS);
	return Publisher.IsValid();
}

void UROS2LidarPoiontCloud2Publisher::Shutdown()
{
	Publisher.Reset();
	Msg.data.clear();
}

bool UROS2LidarPoiontCloud2Publisher::Publish(float DeltaTime, const FSensorDataHeader& Header, const soda::FLidarSensorData& Scan)
{
	if (IsOk())
	{
		const auto Timestamp = soda::RawTimestamp<std::chrono::nanoseconds>(Header.Timestamp);

		//Msg.header.seq = Header.FrameIndex;
		Msg.header.stamp = ros2_ue_wrapper::FromNanoseconds(Timestamp);
		Msg.header.frame_id = TCHAR_TO_UTF8(*FrameID);

		if (Scan.Size.IsSet())
		{
			Msg.width = Scan.Size->X;
			Msg.height = Scan.Size->Y;
		}
		else
		{
			Msg.height = 1;
			Msg.width = Scan.Points.Num();
		}

		Msg.row_step = Scan.Points.Num() * 12;
		Msg.data.resize(Scan.Points.Num() * 12);

		for (int k = 0; k < Scan.Points.Num(); ++k)
		{
			auto& Src = Scan.Points[k];
			float* Dst = (float*)&Msg.data[k * 12];
			Dst[0] = Src.Location.X / 100;
			Dst[1] = -Src.Location.Y / 100;
			Dst[2] = Src.Location.Z / 100;
		}

		Publisher->Publish(Msg);
		return true;
	}
	return false;
}

bool UROS2LidarPoiontCloud2Publisher::IsOk() const
{ 
	return Publisher.IsValid() && ros2_ue_wrapper::Ok();
}

FString UROS2LidarPoiontCloud2Publisher::GetRemark() const
{
	return Topic;
}