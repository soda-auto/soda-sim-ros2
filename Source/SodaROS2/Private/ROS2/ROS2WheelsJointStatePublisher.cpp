// Copyright 2023 SODA.AUTO UK LTD. All Rights Reserved.

#include "Soda/ROS2/ROS2WheelsJointStatePublisher.h"
#include "Soda/VehicleComponents/Sensors/Generic/GenericWheeledVehicleSensor.h"
#include "Soda/VehicleComponents/VehicleDriverComponent.h"
#include "Soda/VehicleComponents/Mechanicles/VehicleGearBoxComponent.h"
#include "Soda/Vehicles/SodaWheeledVehicle.h"
#include "Soda/SodaApp.h"
#include "Soda/LevelState.h"

UROS2WheelsJointStatePublisher::UROS2WheelsJointStatePublisher(const FObjectInitializer& ObjectInitializer)
	: Super(ObjectInitializer)
{
}

bool UROS2WheelsJointStatePublisher::Advertise(UVehicleBaseComponent* InParent)
{
	Shutdown();

	check(InParent);
	Parent = InParent;

	Msg.name = 
	{
		"wheel_fl_pitch", "wheel_fl_yaw", "wheel_fl_offset",
		"wheel_fr_pitch", "wheel_fr_yaw", "wheel_fr_offset",
		"wheel_rl_pitch", "wheel_rl_yaw", "wheel_rl_offset",
		"wheel_rr_pitch", "wheel_rr_yaw", "wheel_rr_offset",
	};
	Msg.position.resize(4 * 3);
	Msg.velocity.resize(4 * 3);
	Msg.effort.resize(4 * 3);

	Publisher = FSodaROS2Module::Get().CreatePublisher<sensor_msgs::msg::JointState>(NodeNamespace, Topic, QoS);
	return Publisher.IsValid();
}

void UROS2WheelsJointStatePublisher::Shutdown()
{
	Publisher.Reset();
}

bool UROS2WheelsJointStatePublisher::Publish(float DeltaTime, const FSensorDataHeader& Header, const FWheeledVehicleSensorData& SensorData)
{
	if (IsOk() && SensorData.WheeledVehicle && SensorData.WheeledVehicle->Is4WDVehicle())
	{
		const auto Timestamp = soda::RawTimestamp<std::chrono::nanoseconds>(Header.Timestamp);

		const auto& Wheels = SensorData.WheeledVehicle->GetWheels4WD();

		//Msg.header.seq = Header.FrameIndex;
		Msg.header.stamp = ros2_ue_wrapper::FromNanoseconds(Timestamp);
		Msg.header.frame_id = TCHAR_TO_UTF8(*FrameID);

		// wheel_fl_pitch
		Msg.position[0] = 0;
		Msg.velocity[0] = Wheels[0]->AngularVelocity;
		Msg.effort[0] = Wheels[0]->ReqTorq - Wheels[0]->ReqBrakeTorque;

		// wheel_fl_yaw
		Msg.position[1] = -Wheels[0]->Steer;
		Msg.velocity[1] = 0;
		Msg.effort[1] = 0;

		// wheel_fl_offset
		Msg.position[2] = 0;
		Msg.velocity[2] = 0;
		Msg.effort[2] = 0;

		// wheel_fr_pitch
		Msg.position[3] = 0;
		Msg.velocity[3] = Wheels[1]->AngularVelocity;
		Msg.effort[3] = Wheels[1]->ReqTorq - Wheels[1]->ReqBrakeTorque;

		// wheel_fr_yaw
		Msg.position[4] = -Wheels[1]->Steer;
		Msg.velocity[4] = 0;
		Msg.effort[4] = 0;

		// wheel_fr_offset
		Msg.position[5] = 0;
		Msg.velocity[5] = 0;
		Msg.effort[5] = 0;

		// wheel_rl_pitch
		Msg.position[6] = 0;
		Msg.velocity[6] = Wheels[2]->AngularVelocity;
		Msg.effort[6] = Wheels[2]->ReqTorq - Wheels[2]->ReqBrakeTorque;

		// wheel_rl_yaw
		Msg.position[7] = -Wheels[2]->Steer;
		Msg.velocity[7] = 0;
		Msg.effort[7] = 0;

		// wheel_rl_offset
		Msg.position[8] = 0;
		Msg.velocity[8] = 0;
		Msg.effort[8] = 0;

		// wheel_rr_pitch
		Msg.position[9] = 0;
		Msg.velocity[9] = Wheels[3]->AngularVelocity;
		Msg.effort[9] = Wheels[3]->ReqTorq - Wheels[3]->ReqBrakeTorque;

		// wheel_rr_yaw
		Msg.position[10] = -Wheels[3]->Steer;
		Msg.velocity[10] = 0;
		Msg.effort[10] = 0;

		// wheel_rr_offset
		Msg.position[11] = 0;
		Msg.velocity[11] = 0;
		Msg.effort[11] = 0;

		Publisher->Publish(Msg);
		return true;
	}
	return false;
}

bool UROS2WheelsJointStatePublisher::IsOk() const
{ 
	return Publisher.IsValid() && ros2_ue_wrapper::Ok();
}

FString UROS2WheelsJointStatePublisher::GetRemark() const
{
	return Topic;
}