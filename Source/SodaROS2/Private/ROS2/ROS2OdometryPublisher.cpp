// Copyright 2023 SODA.AUTO UK LTD. All Rights Reserved.

#include "Soda/ROS2/ROS2OdometryPublisher.h"
#include "Soda/VehicleComponents/Sensors/Base/NavSensor.h"
#include "Soda/SodaApp.h"
#include "Soda/LevelState.h"

UROS2OdometryPublisher::UROS2OdometryPublisher(const FObjectInitializer& ObjectInitializer)
	: Super(ObjectInitializer)
{
}

bool UROS2OdometryPublisher::Advertise(UVehicleBaseComponent* InParent)
{
	Shutdown();

	check(InParent);
	Parent = InParent;

	FormatedTopic = TopicSetup.GetFormatedTopic(Parent->GetName());
	Publisher = ros2::TPublisher<nav_msgs::msg::Odometry>::Create(NodeName, FormatedTopic, QoS);

	return Publisher.IsValid();
}

void UROS2OdometryPublisher::Shutdown()
{
	Publisher.Reset();
}

bool UROS2OdometryPublisher::Publish(float DeltaTime, const FSensorDataHeader& Header, const FTransform& RelativeTransform, const FPhysBodyKinematic& VehicleKinematic, const FImuNoiseParams& Covariance)
{
	if (IsOk())
	{
		FTransform WorldPose;
		FVector WorldVel;
		FVector LocalAcc;
		FVector Gyro;
		VehicleKinematic.CalcIMU(RelativeTransform, WorldPose, WorldVel, LocalAcc, Gyro);

		FVector WorldLoc = WorldPose.GetLocation();
		FRotator WorldRot = WorldPose.Rotator();

		/*
		if (ALevelState* LevelState = Parent->GetLevelState())
		{
			WorldRot = LevelState->GetLLConverter().ConvertRotationForward(WorldRot);
			WorldLoc = LevelState->GetLLConverter().Con(WorldRot);
		}
		*/

		auto Quat = FRotator(WorldRot.Pitch, -WorldRot.Yaw, -WorldRot.Roll).Quaternion();

		const auto Timestamp = soda::RawTimestamp<std::chrono::nanoseconds>(Header.Timestamp);

		Msg.header.stamp = ros2_ue_wrapper::FromNanoseconds(Timestamp);
		Msg.header.frame_id = TCHAR_TO_UTF8(*FrameID);
		Msg.child_frame_id = TCHAR_TO_UTF8(*ChildFrameID);

		Msg.pose.pose.position.x =  WorldLoc.X / 100.0;
		Msg.pose.pose.position.y = -WorldLoc.Y / 100.0;
		Msg.pose.pose.position.z =  WorldLoc.Z / 100.0;
		Msg.pose.pose.orientation.x = Quat.X;
		Msg.pose.pose.orientation.y = Quat.Y;
		Msg.pose.pose.orientation.z = Quat.Z;
		Msg.pose.pose.orientation.w = Quat.W;
		Msg.pose.covariance = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };

		Msg.twist.twist.linear.x =  WorldVel.X / 100.0;
		Msg.twist.twist.linear.y = -WorldVel.Y / 100.0;
		Msg.twist.twist.linear.z =  WorldVel.Z / 100.0;
		Msg.twist.twist.angular.x = -Gyro.X / 100.0;
		Msg.twist.twist.angular.y =  Gyro.Y / 100.0;
		Msg.twist.twist.angular.z = -Gyro.Z / 100.0;
		Msg.twist.covariance = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };

		Publisher->Publish(Msg);
		return true;
	}
	return false;
}

bool UROS2OdometryPublisher::IsOk() const
{ 
	return Publisher.IsValid() && ros2_ue_wrapper::Ok();
}

FString UROS2OdometryPublisher::GetRemark() const
{
	return Publisher.IsValid() ? FormatedTopic : TopicSetup.Topic;
}