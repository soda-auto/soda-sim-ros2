// Copyright 2023 SODA.AUTO UK LTD. All Rights Reserved.

#include "Soda/VehicleComponents/ROS2WheelInterface.h"
#include "Soda/UnrealSoda.h"
#include "Soda/Vehicles/SodaWheeledVehicle.h"
#include "Soda/VehicleComponents/SodaVehicleWheel.h"
#include "Engine/Canvas.h"
#include "Engine/Engine.h"

static FString GetROS2WheelName(USodaVehicleWheelComponent* Wheel, bool bIs4WDVehicle)
{
	check(Wheel);

	FString WheelName = TEXT("wheel_");
	if (bIs4WDVehicle)
	{
		if (int(Wheel->WheelChassis) == 0)
		{
			WheelName += TEXT("f");
		}
		if (int(Wheel->WheelChassis) == 1)
		{
			WheelName += TEXT("r");
		}
		else
		{
			WheelName += TEXT("ch") + FString::FromInt(int(Wheel->WheelChassis));
		}
	}
	else
	{
		WheelName += TEXT("ch") + FString::FromInt(int(Wheel->WheelChassis));
	}

	switch (Wheel->WheelSida)
	{
	case EWheelSida::Left: WheelName += TEXT("l"); break;
	case EWheelSida::Right: WheelName += TEXT("r"); break;
	case EWheelSida::Center: WheelName += TEXT("c"); break;
	default: WheelName += TEXT("_s") + FString::FromInt(int(Wheel->WheelSida));
	}
	return WheelName;
}

UROS2WheelInterfaceComponent::UROS2WheelInterfaceComponent(const FObjectInitializer& ObjectInitializer)
	: Super(ObjectInitializer)
{
	GUI.Category = TEXT("Vehicle Mechanicles");
	GUI.IcanName = TEXT("SodaIcons.Motor");
	GUI.ComponentNameOverride = TEXT("ROS2 Wheels");
	GUI.bIsPresentInAddMenu = true;

	Common.Activation = EVehicleComponentActivation::OnStartScenario;

	PrimaryComponentTick.bCanEverTick = true;
	PrimaryComponentTick.TickGroup = TG_PostPhysics;

	TickData.bAllowVehiclePrePhysTick = true;
	TickData.bAllowVehiclePostPhysTick = true;
	TickData.PostPhysTickGroup = EVehicleComponentPostPhysTickGroup::TickGroup5;

}

bool UROS2WheelInterfaceComponent::OnActivateVehicleComponent()
{
	if (!Super::OnActivateVehicleComponent())
	{
		return false;
	}

	WheelsPubSub.Reset();

	UObject* TorqueTransmissionObject = LinkToTorqueTransmission.GetObject<UObject>(GetOwner());
	ITorqueTransmission* TorqueTransmissionInterface = Cast<ITorqueTransmission>(TorqueTransmissionObject);
	if (TorqueTransmissionObject && TorqueTransmissionInterface)
	{
		OutputTorqueTransmission.SetInterface(TorqueTransmissionInterface);
		OutputTorqueTransmission.SetObject(TorqueTransmissionObject);
	}
	else
	{
		SetHealth(EVehicleComponentHealth::Error, TEXT("Transmission isn't connected"));
		return false;
	}

	auto GetTopic = [this](FROS2TopicSetup& In, USodaVehicleWheelComponent* Wheel)
	{
		FROS2TopicSetup* Setup = &In;
		if (FROS2TopicSetup* Found = ConfiguredSignals.FindByPredicate([&In](const FROS2TopicSetup & It) {return It.DevName == In.DevName && It.SignalName == In.SignalName;}))
		{
			Setup = Found;
		}
		return Setup->GetFormatedTopic(GetROS2WheelName(Wheel, GetWheeledVehicle()->IsXWDVehicle(4)));
	};

	auto AddWheel = [this, &GetTopic](USodaVehicleWheelComponent* Wheel)
	{
		auto & WheelPubSub = WheelsPubSub.Add_GetRef({});
		WheelPubSub.Wheel = Wheel;

		if (!WheelPubSub.PublisherAngularVelocity.Start(NodeName, GetTopic(AngVelSignal, Wheel), PubQoS))
		{
			SetHealth(EVehicleComponentHealth::Error, TEXT("Can't create PublisherAngularVelocity"));
			return false;
		}
		if (!WheelPubSub.PublisherSteer.Start(NodeName, GetTopic(SteerSignal, Wheel), PubQoS))
		{
			SetHealth(EVehicleComponentHealth::Error, TEXT("Can't create PublisherSteer"));
			return false;
		}

		if (!WheelPubSub.PublisherPitch.Start(NodeName, GetTopic(PitchSignal, Wheel), PubQoS))
		{
			SetHealth(EVehicleComponentHealth::Error, TEXT("Can't create PublisherPitch"));
			return false;
		}

		if (!WheelPubSub.PublisherLonSlip.Start(NodeName, GetTopic(LonSlipSignal, Wheel), PubQoS))
		{
			SetHealth(EVehicleComponentHealth::Error, TEXT("Can't create PublisherLonSlip"));
			return false;
		}

		if (!WheelPubSub.PublisherLatSlip.Start(NodeName, GetTopic(LatSlipSignal, Wheel), PubQoS))
		{
			SetHealth(EVehicleComponentHealth::Error, TEXT("Can't create PublisherLatSlip"));
			return false;
		}

		if (!WheelPubSub.PublisherSuspensionOffset.Start(NodeName, GetTopic(SuspensionOffsetSignal, Wheel), PubQoS))
		{
			SetHealth(EVehicleComponentHealth::Error, TEXT("Can't create PublisherSuspensionOffset"));
			return false;
		}

		if (!WheelPubSub.SubscriptionReqTorq.Start(NodeName, GetTopic(ReqTorqSignal, Wheel), SubQoS))
		{
			SetHealth(EVehicleComponentHealth::Error, TEXT("Can't create SubscriptionReqTorq"));
			return false;
		}

		if (!WheelPubSub.SubscriptionReqBrakeTorque.Start(NodeName, GetTopic(ReqBrakeTorqueSignal, Wheel), SubQoS))
		{
			SetHealth(EVehicleComponentHealth::Error, TEXT("Can't create SubscriptionReqBrakeTorque"));
			return false;
		}

		if (!WheelPubSub.SubscriptionReqSteer.Start(NodeName, GetTopic(ReqSteerSignal, Wheel), SubQoS))
		{
			SetHealth(EVehicleComponentHealth::Error, TEXT("Can't create SubscriptionReqSteer"));
			return false;
		}
		return true;
	};

	for (USodaVehicleWheelComponent * Wheel : GetWheeledVehicle()->GetWheelsSorted())
	{
		if (!AddWheel(Wheel))
		{
			return false;
		}
	}
	
	return true;
}

void UROS2WheelInterfaceComponent::OnDeactivateVehicleComponent()
{
	Super::OnDeactivateVehicleComponent();
	WheelsPubSub.Reset();
	ConfiguredSignals.Reset();
}

bool UROS2WheelInterfaceComponent::ConfigureSignal(const FROS2TopicSetup& Setup)
{
	// TODO: check Setup.DevName and Setup.SignalName are valid
	ConfiguredSignals.Add(Setup);
	return true;
}

void UROS2WheelInterfaceComponent::PrePhysicSimulation(float DeltaTime, const FPhysBodyKinematic& VehicleKinematic, const TTimestamp& Timestamp)
{
	Super::PrePhysicSimulation(DeltaTime, VehicleKinematic, Timestamp);

	for (auto& It : WheelsPubSub)
	{
		if (It.SubscriptionReqTorq.IsSiganalValid())
		{
			It.Wheel->ReqTorq = It.SubscriptionReqTorq.GetSignal();
		}
		if (It.SubscriptionReqBrakeTorque.IsSiganalValid())
		{
			It.Wheel->ReqBrakeTorque = It.SubscriptionReqBrakeTorque.GetSignal();
		}
		if (It.SubscriptionReqSteer.IsSiganalValid())
		{
			It.Wheel->ReqSteer = It.SubscriptionReqSteer.GetSignal();
		}
	}
}

void UROS2WheelInterfaceComponent::PostPhysicSimulation(float DeltaTime, const FPhysBodyKinematic& VehicleKinematic, const TTimestamp& Timestamp)
{
	Super::PostPhysicSimulation(DeltaTime, VehicleKinematic, Timestamp);

	for (auto& It : WheelsPubSub)
	{
		It.PublisherSteer.Publish(It.Wheel->Steer);
		It.PublisherPitch.Publish(It.Wheel->Pitch);
		It.PublisherAngularVelocity.Publish(It.Wheel->AngularVelocity);
		It.PublisherLonSlip.Publish(It.Wheel->Slip.X);
		It.PublisherLatSlip.Publish(It.Wheel->Slip.Y);
		It.PublisherSuspensionOffset.Publish(It.Wheel->SuspensionOffset2.Z / 100);
	}
}

void UROS2WheelInterfaceComponent::DrawDebug(UCanvas* Canvas, float& YL, float& YPos)
{
	Super::DrawDebug(Canvas, YL, YPos);

	/*
	if (Common.bDrawDebugCanvas)
	{
		UFont* RenderFont = GEngine->GetSmallFont();
		Canvas->SetDrawColor(FColor::White);
		YPos += Canvas->DrawText(RenderFont, FString::Printf(TEXT("TorqReq: %.2f H/m"), TorqReq), 16, YPos);
		YPos += Canvas->DrawText(RenderFont, FString::Printf(TEXT("AngVel: %.2f rpm"), AngularVelocity * ANG2RPM), 16, YPos);
	}
	*/
}