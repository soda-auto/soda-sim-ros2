// Copyright 2023 SODA.AUTO UK LTD. All Rights Reserved.

#pragma once

#include "Soda/VehicleComponents/VehicleBaseComponent.h"
#include "Soda/ROS2.h"
#include "Soda/ROS2/IROS2FMU.h"
#include "std_msgs/msg/float32.hpp"
#include "ROS2ShaftAdapter.generated.h"

/**
 * UROS2ShaftAdapterComponent
 */
UCLASS(ClassGroup = Soda, BlueprintType, Blueprintable, meta = (BlueprintSpawnableComponent))
class SODAROS2_API UROS2ShaftAdapterComponent 
	: public UVehicleBaseComponent
	, public IROS2FMU
{
	GENERATED_UCLASS_BODY()

public:
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = Link, SaveGame, meta = (EditInRuntime, ReactivateActor, AllowedClasses = "/Script/UnrealSoda.TorqueTransmission"))
	FSubobjectReference LinkToTorqueTransmission{ TEXT("Differential") };

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = ROS2, SaveGame, meta = (EditInRuntime, ReactivateComponent))
	FString NodeName = DEFAULT_ROS2_NODE_NAME;

	/** [H/m] */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = ROS2, SaveGame, meta = (EditInRuntime, ReactivateComponent))
	FROS2TopicSetup ReqTopicSignal = { TOPIC_SIG_TEMPLATE, TEXT("torq_req")};

	/** [rad/s] */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = ROS2, SaveGame, meta = (EditInRuntime, ReactivateComponent))
	FROS2TopicSetup AngVelTopicSignal = { TOPIC_SIG_TEMPLATE, TEXT("angule_velocity") };

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = ROS2, SaveGame, meta = (DisplayName = "Pub QoS", EditInRuntime, ReactivateComponent))
	FQoS PubQoS{};

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = ROS2, SaveGame, meta = (DisplayName = "Sub QoS", EditInRuntime, ReactivateComponent))
	FQoS SubQoS{};

public:
	virtual bool ConfigureSignal(const FROS2TopicSetup& Setup) override;

protected:
	virtual bool OnActivateVehicleComponent() override;
	virtual void OnDeactivateVehicleComponent() override;

	virtual void DrawDebug(UCanvas* Canvas, float& YL, float& YPos) override;
	virtual void PrePhysicSimulation(float DeltaTime, const FPhysBodyKinematic& VehicleKinematic, const TTimestamp & Timestamp) override;
	virtual void PostPhysicSimulation(float DeltaTime, const FPhysBodyKinematic& VehicleKinematic, const TTimestamp& Timestamp) override;

protected:
	UPROPERTY(BlueprintReadOnly, Category = Engin)
	TScriptInterface<ITorqueTransmission> OutputTorqueTransmission;

	ros2::TPublisherSignal<std_msgs::msg::Float32> Publisher;
	ros2::TSubscriptionSignal<std_msgs::msg::Float32> Subscription;
};


