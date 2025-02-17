// Copyright 2023 SODA.AUTO UK LTD. All Rights Reserved.

#include "Soda/VehicleComponents/ROS2TFPublisherComponent.h"
#include "Soda/VehicleComponents/SodaVehicleWheel.h"
#include "Components/SceneComponent.h"

UROS2TFPublisherComponent::UROS2TFPublisherComponent(const FObjectInitializer& ObjectInitializer)
	: Super(ObjectInitializer)
{
	GUI.Category = TEXT("ROS2");
	GUI.ComponentNameOverride = TEXT("TF Publisher");
	GUI.bIsPresentInAddMenu = true;

	PrimaryComponentTick.bCanEverTick = true;
	PrimaryComponentTick.TickGroup = TG_PostPhysics;

}

bool UROS2TFPublisherComponent::OnActivateVehicleComponent()
{
	if (!Super::OnActivateVehicleComponent())
	{
		return false;
	}

	Publisher.Reset();

	if (AActor *Owner = GetOwner())
	{
		if (USceneComponent* RootComponent = Owner->GetRootComponent())
		{
			if (IsValid(RootComponent))
			{
				SceneComponentsToPublish.Add(RootComponent);
			}
			else
			{
				SetHealth(EVehicleComponentHealth::Error, TEXT("RootComponent isn't valid"));
				return false;
			}
		}
		
		for (auto& It : ComponentsToPublish)
		{
			if (USodaVehicleWheelComponent* WheelComponent = It.GetObject<USodaVehicleWheelComponent>(Owner))
			{
				if (IsValid(WheelComponent))
				{
					WheelComponentsToPublish.Add(WheelComponent);
				}
			}
			else if (USceneComponent* SceneComponent = It.GetObject<USceneComponent>(Owner))
			{
				if (IsValid(SceneComponent))
				{
					SceneComponentsToPublish.Add(SceneComponent);
				}
			}
		}
	}

	Msg.transforms.resize(SceneComponentsToPublish.Num() + WheelComponentsToPublish.Num());
	Msg.transforms[0].header.frame_id = TCHAR_TO_UTF8(*FrameID);
	Msg.transforms[0].child_frame_id = TCHAR_TO_UTF8(*VehicleBaseChildFrameID);

	for (int i = 1; i < SceneComponentsToPublish.Num(); ++i)
	{
		Msg.transforms[i].header.frame_id = TCHAR_TO_UTF8(*VehicleBaseChildFrameID);
		Msg.transforms[i].child_frame_id = TCHAR_TO_UTF8(*SceneComponentsToPublish[i]->GetName());
	}

	for (int i = 0; i < WheelComponentsToPublish.Num(); ++i)
	{
		Msg.transforms[SceneComponentsToPublish.Num() + i].header.frame_id = TCHAR_TO_UTF8(*VehicleBaseChildFrameID);
		Msg.transforms[SceneComponentsToPublish.Num() + i].child_frame_id = TCHAR_TO_UTF8(*WheelComponentsToPublish[i]->GetName());
	}

	FormatedTopic = TopicSetup.GetFormatedTopic(GetName());
	Publisher = ros2::TPublisher<tf2_msgs::msg::TFMessage>::Create(NodeName, FormatedTopic, QoS);

	return Publisher.IsValid();
}

void UROS2TFPublisherComponent::OnDeactivateVehicleComponent()
{
	Super::OnDeactivateVehicleComponent();
	Shutdown();
}


void UROS2TFPublisherComponent::TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction)
{
	Super::TickComponent(DeltaTime, TickType, ThisTickFunction);
	if (!HealthIsWorkable()) return;

	if (Publisher.IsValid() && ros2_ue_wrapper::Ok())
	{
		FSensorDataHeader Header = GetHeaderGameThread();
		const auto Timestamp = soda::RawTimestamp<std::chrono::nanoseconds>(Header.Timestamp);
		auto ROSTime = ros2_ue_wrapper::FromNanoseconds(Timestamp);

		for (int i = 0; i < SceneComponentsToPublish.Num(); ++i)
		{
			const FVector Loc = SceneComponentsToPublish[i]->GetRelativeLocation();
			const FRotator Rot = SceneComponentsToPublish[i]->GetRelativeRotation();
			const auto Quat = FRotator(Rot.Pitch, -Rot.Yaw, -Rot.Roll).Quaternion();

			auto& Transform = Msg.transforms[i];
			Transform.header.stamp = ROSTime;
			Transform.transform.translation.x = Loc.X / 100.0;
			Transform.transform.translation.y = -Loc.Y / 100.0;
			Transform.transform.translation.z = Loc.Z / 100.0;
			Transform.transform.rotation.x = Quat.X;
			Transform.transform.rotation.y = Quat.Y;
			Transform.transform.rotation.z = Quat.Z;
			Transform.transform.rotation.w = Quat.W;
		}

		for (int i = 0; i < WheelComponentsToPublish.Num(); ++i)
		{
			const USodaVehicleWheelComponent* SodaWheel = WheelComponentsToPublish[i];

			FRotator Rot = SodaWheel->RestingRotation;
			Rot.Pitch += SodaWheel->Pitch / M_PI * 180;
			Rot.Yaw += SodaWheel->Steer / M_PI * 180;
			const FVector Loc = SodaWheel->GetWheelLocation();
			const auto Quat = FRotator(Rot.Pitch, -Rot.Yaw, -Rot.Roll).Quaternion();

			auto& Transform = Msg.transforms[i + SceneComponentsToPublish.Num()];
			Transform.header.stamp = ROSTime;
			Transform.transform.translation.x = Loc.X / 100.0;
			Transform.transform.translation.y = -Loc.Y / 100.0;
			Transform.transform.translation.z = Loc.Z / 100.0;
			Transform.transform.rotation.x = Quat.X;
			Transform.transform.rotation.y = Quat.Y;
			Transform.transform.rotation.z = Quat.Z;
			Transform.transform.rotation.w = Quat.W;
		}

		Publisher->Publish(Msg);
	}
	else
	{
		SetHealth(EVehicleComponentHealth::Error, TEXT("ROS2 is broken"));
	}
}

void UROS2TFPublisherComponent::DrawDebug(UCanvas* Canvas, float& YL, float& YPos)
{
	Super::DrawDebug(Canvas, YL, YPos);
}

FString UROS2TFPublisherComponent::GetRemark() const
{
	return Publisher.IsValid() ? FormatedTopic : TopicSetup.Topic;
}

void UROS2TFPublisherComponent::Shutdown()
{
	Publisher.Reset();
	WheelComponentsToPublish.Empty();
	SceneComponentsToPublish.Empty();
}