// Copyright 2023 SODA.AUTO UK LTD. All Rights Reserved.

#include "Soda/ROS2/ROS2NavSatFixPublisher.h"
#include "Soda/VehicleComponents/Sensors/Base/NavSensor.h"
#include "Soda/SodaApp.h"
#include "Soda/LevelState.h"

UROS2NavSatFixPublisher::UROS2NavSatFixPublisher(const FObjectInitializer& ObjectInitializer)
	: Super(ObjectInitializer)
{
}

bool UROS2NavSatFixPublisher::Advertise(UVehicleBaseComponent* InParent)
{
	Shutdown();

	check(InParent);
	Parent = InParent;

	Publisher = FSodaROS2Module::Get().CreatePublisher<sensor_msgs::msg::NavSatFix>(NodeNamespace, Topic, QoS);
	return Publisher.IsValid();
}

void UROS2NavSatFixPublisher::Shutdown()
{
	Publisher.Reset();
}

bool UROS2NavSatFixPublisher::Publish(float DeltaTime, const FSensorDataHeader& Header, const FTransform& RelativeTransform, const FPhysBodyKinematic& VehicleKinematic, const FImuNoiseParams& Covariance)
{
	if (IsOk())
	{
		FTransform WorldPose;
		FVector WorldVel;
		FVector LocalAcc;
		FVector Gyro;
		VehicleKinematic.CalcIMU(RelativeTransform, WorldPose, WorldVel, LocalAcc, Gyro);


		if (ALevelState* LevelState = Parent->GetLevelState())
		{
			LevelState->GetLLConverter().UE2LLA(WorldPose.GetLocation(), Msg.longitude, Msg.latitude, Msg.altitude);
			Msg.status.status = sensor_msgs::msg::NavSatStatus::STATUS_GBAS_FIX;
		}
		else
		{
			Msg.longitude = 0;
			Msg.latitude = 0;
			Msg.altitude = 0;
			Msg.status.status = sensor_msgs::msg::NavSatStatus::STATUS_NO_FIX;
		}

		const auto Timestamp = soda::RawTimestamp<std::chrono::nanoseconds>(Header.Timestamp);

		Msg.header.stamp = ros2_ue_wrapper::FromNanoseconds(Timestamp);
		Msg.header.frame_id = TCHAR_TO_UTF8(*FrameID);

		Msg.position_covariance = { 0, 0, 0, 0, 0, 0 ,0 ,0, 0 };
		Msg.position_covariance_type = sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_UNKNOWN;

		Publisher->Publish(Msg);
		return true;
	}
	return false;
}

bool UROS2NavSatFixPublisher::IsOk() const
{ 
	return Publisher.IsValid() && ros2_ue_wrapper::Ok();
}

FString UROS2NavSatFixPublisher::GetRemark() const
{
	return Topic;
}