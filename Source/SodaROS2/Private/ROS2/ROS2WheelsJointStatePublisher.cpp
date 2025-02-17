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

	ASodaWheeledVehicle* SodaWheeledVehicle = Cast<ASodaWheeledVehicle>(InParent->GetVehicle());
	check(SodaWheeledVehicle);

	for (auto & Wheel: SodaWheeledVehicle->GetWheelsSorted())
	{
		FString WheelName = Wheel->GetName();
		Msg.name.push_back(TCHAR_TO_UTF8(*(WheelName + TEXT("_pitch"))));
		Msg.name.push_back(TCHAR_TO_UTF8(*(WheelName + TEXT("_yaw"))));
		Msg.name.push_back(TCHAR_TO_UTF8(*(WheelName + TEXT("_offset"))));
	}

	Msg.position.resize(4 * 3);
	Msg.velocity.resize(4 * 3);
	Msg.effort.resize(4 * 3);

	FormatedTopic = TopicSetup.GetFormatedTopic(Parent->GetName());
	Publisher = ros2::TPublisher<sensor_msgs::msg::JointState>::Create(NodeName, FormatedTopic, QoS);

	return Publisher.IsValid();
}

void UROS2WheelsJointStatePublisher::Shutdown()
{
	Publisher.Reset();
}

bool UROS2WheelsJointStatePublisher::Publish(float DeltaTime, const FSensorDataHeader& Header, const FWheeledVehicleSensorData& SensorData)
{
	if (IsOk() && SensorData.WheeledVehicle)
	{
		const auto Timestamp = soda::RawTimestamp<std::chrono::nanoseconds>(Header.Timestamp);

		const auto& Wheels = SensorData.WheeledVehicle->GetWheelsSorted();

		//Msg.header.seq = Header.FrameIndex;
		Msg.header.stamp = ros2_ue_wrapper::FromNanoseconds(Timestamp);
		Msg.header.frame_id = TCHAR_TO_UTF8(*FrameID);

		
		for (int i = 0; i < SensorData.WheeledVehicle->GetWheelsSorted().Num(); ++i)
		{
			const auto& Wheel = SensorData.WheeledVehicle->GetWheelsSorted()[i];
			const int ind = i * 3;

			// pitch
			Msg.position[ind + 0] = 0;
			Msg.velocity[ind + 0] = Wheel->AngularVelocity;
			Msg.effort[ind + 0] = Wheel->ReqTorq - Wheel->ReqBrakeTorque;

			// yaw
			Msg.position[ind + 1] = -Wheel->Steer;
			Msg.velocity[ind + 1] = 0;
			Msg.effort[ind + 1] = 0;

			// offset
			Msg.position[ind + 2] = 0;
			Msg.velocity[ind + 2] = 0;
			Msg.effort[ind + 2] = 0;
		}

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
	return Publisher.IsValid() ? FormatedTopic : TopicSetup.Topic;
}