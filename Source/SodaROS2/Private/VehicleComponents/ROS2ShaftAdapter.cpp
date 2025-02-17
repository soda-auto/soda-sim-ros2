// Copyright 2023 SODA.AUTO UK LTD. All Rights Reserved.

#include "Soda/VehicleComponents/ROS2ShaftAdapter.h"
#include "Soda/UnrealSoda.h"
#include "Soda/Vehicles/SodaWheeledVehicle.h"
#include "Engine/Canvas.h"
#include "Engine/Engine.h"

UROS2ShaftAdapterComponent::UROS2ShaftAdapterComponent(const FObjectInitializer& ObjectInitializer)
	: Super(ObjectInitializer)
{
	GUI.Category = TEXT("Vehicle Mechanicles");
	GUI.IcanName = TEXT("SodaIcons.Motor");
	GUI.ComponentNameOverride = TEXT("ROS2 Shaft Adapter");
	GUI.bIsPresentInAddMenu = true;

	Common.Activation = EVehicleComponentActivation::OnStartScenario;

	PrimaryComponentTick.bCanEverTick = true;
	PrimaryComponentTick.TickGroup = TG_PostPhysics;

	TickData.bAllowVehiclePrePhysTick = true;
	TickData.bAllowVehiclePostPhysTick = true;
	TickData.PostPhysTickGroup = EVehicleComponentPostPhysTickGroup::TickGroup5;
}

bool UROS2ShaftAdapterComponent::OnActivateVehicleComponent()
{
	if (!Super::OnActivateVehicleComponent())
	{
		return false;
	}

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

	if (!Publisher.Start(NodeName, AngVelTopicSignal.GetFormatedTopic(GetName()), PubQoS))
	{
		SetHealth(EVehicleComponentHealth::Error, TEXT("Can't create publisher"));
		return false;
	}

	if (!Subscription.Start(NodeName, ReqTopicSignal.GetFormatedTopic(GetName()), SubQoS))
	{
		SetHealth(EVehicleComponentHealth::Error, TEXT("Can't create subscription"));
		return false;
	}

	return true;
}

void UROS2ShaftAdapterComponent::OnDeactivateVehicleComponent()
{
	Super::OnDeactivateVehicleComponent();
	Publisher.Stop();
	Subscription.Stop();
}

void UROS2ShaftAdapterComponent::PrePhysicSimulation(float DeltaTime, const FPhysBodyKinematic& VehicleKinematic, const TTimestamp& Timestamp)
{
	Super::PrePhysicSimulation(DeltaTime, VehicleKinematic, Timestamp);

	if (Subscription.IsSiganalValid())
	{
		OutputTorqueTransmission->PassTorque(Subscription.GetSignal());
	}
}

void UROS2ShaftAdapterComponent::PostPhysicSimulation(float DeltaTime, const FPhysBodyKinematic& VehicleKinematic, const TTimestamp& Timestamp)
{
	Super::PostPhysicSimulation(DeltaTime, VehicleKinematic, Timestamp);

	float AngularVelocity = OutputTorqueTransmission->ResolveAngularVelocity(); // * Ratio * (bFlipAngularVelocity ? -1.0 : 1.0);
	Publisher.Publish(AngularVelocity);
}

void UROS2ShaftAdapterComponent::DrawDebug(UCanvas* Canvas, float& YL, float& YPos)
{
	Super::DrawDebug(Canvas, YL, YPos);

	if (Common.bDrawDebugCanvas)
	{
		UFont* RenderFont = GEngine->GetSmallFont();
		Canvas->SetDrawColor(FColor::White);
		YPos += Canvas->DrawText(RenderFont, FString::Printf(TEXT("TorqReq: %.2f H/m"), Subscription.GetSignal()), 16, YPos);
		YPos += Canvas->DrawText(RenderFont, FString::Printf(TEXT("AngVel: %.2f rpm"), Publisher.GetLastPublishedSignal() * ANG2RPM), 16, YPos);
	}
}

bool UROS2ShaftAdapterComponent::ConfigureSignal(const FROS2TopicSetup& Setup)
{
	// TODO: check Setup.DevName is valid

	if (Setup.SignalName == ReqTopicSignal.SignalName)
	{
		ReqTopicSignal = Setup;
		return true;
	}
	if (Setup.SignalName == AngVelTopicSignal.SignalName)
	{
		AngVelTopicSignal = Setup;
		return true;
	}
	return false;
}