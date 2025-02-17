// Copyright 2023 SODA.AUTO UK LTD. All Rights Reserved.

#include "Soda/ROS2/ROS2ImuPublisher.h"
#include "Soda/VehicleComponents/Sensors/Base/NavSensor.h"
#include "Soda/SodaApp.h"
#include "Soda/LevelState.h"

UROS2ImuPublisher::UROS2ImuPublisher(const FObjectInitializer& ObjectInitializer)
	: Super(ObjectInitializer)
{
}

bool UROS2ImuPublisher::Advertise(UVehicleBaseComponent* InParent)
{
	Shutdown();

	check(InParent);
	Parent = InParent;

	FormatedTopic = TopicSetup.GetFormatedTopic(Parent->GetName());
	Publisher = ros2::TPublisher<sensor_msgs::msg::Imu>::Create(NodeName, FormatedTopic, QoS);

	return Publisher.IsValid();
}

void UROS2ImuPublisher::Shutdown()
{
	Publisher.Reset();
}

bool UROS2ImuPublisher::Publish(float DeltaTime, const FSensorDataHeader& Header, const FTransform& RelativeTransform, const FPhysBodyKinematic& VehicleKinematic, const FImuNoiseParams& Covariance)
{
	if (IsOk())
	{
		FTransform WorldPose;
		FVector WorldVel;
		FVector LocalAcc;
		FVector Gyro;
		VehicleKinematic.CalcIMU(RelativeTransform, WorldPose, WorldVel, LocalAcc, Gyro);

		FRotator WorldRot = WorldPose.Rotator();
		if (ALevelState* LevelState = Parent->GetLevelState())
		{
			WorldRot = LevelState->GetLLConverter().ConvertRotationForward(WorldRot);
		}

		auto Quat = FRotator(WorldRot.Pitch, -WorldRot.Yaw, -WorldRot.Roll).Quaternion();

		const auto Timestamp = soda::RawTimestamp<std::chrono::nanoseconds>(Header.Timestamp);

		//Msg.header.seq = Header.FrameIndex;
		Msg.header.stamp = ros2_ue_wrapper::FromNanoseconds(Timestamp);
		Msg.header.frame_id = TCHAR_TO_UTF8(*FrameID);

		Msg.orientation.x = Quat.X;
		Msg.orientation.y = Quat.Y;
		Msg.orientation.z = Quat.Z;
		Msg.orientation.w = Quat.W;

		Msg.angular_velocity.x = -Gyro.X / 100.0;
		Msg.angular_velocity.y =  Gyro.Y / 100.0;
		Msg.angular_velocity.z = -Gyro.Z / 100.0;

		Msg.linear_acceleration.x =  LocalAcc.X / 100.0;
		Msg.linear_acceleration.y = -LocalAcc.Y / 100.0;
		Msg.linear_acceleration.z =  LocalAcc.Z / 100.0;

		Msg.orientation_covariance = { 0, 0, 0, 0, 0, 0 ,0 ,0, 0 };
		Msg.angular_velocity_covariance = { 0, 0, 0, 0, 0, 0 ,0 ,0, 0 };
		Msg.linear_acceleration_covariance = { 0, 0, 0, 0, 0, 0 ,0 ,0, 0 };

		Publisher->Publish(Msg);
		return true;
	}
	return false;
}

bool UROS2ImuPublisher::IsOk() const
{ 
	return Publisher.IsValid() && ros2_ue_wrapper::Ok();
}

FString UROS2ImuPublisher::GetRemark() const
{
	return Publisher.IsValid() ? FormatedTopic : TopicSetup.Topic;
}