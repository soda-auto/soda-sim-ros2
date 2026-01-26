// Copyright 2023 SODA.AUTO UK LTD. All Rights Reserved.

#pragma once

#include "Soda/VehicleComponents/WheeledVehicleComponent.h"
#include "Soda/ROS2.h"
#include "Soda/ROS2/IROS2FMU.h"
#include "std_msgs/msg/float32.hpp"
#include "ROS2WheelInterface.generated.h"

/**
 * UROS2WheelInterfaceComponent
 */
UCLASS(ClassGroup = Soda, BlueprintType, Blueprintable, meta = (BlueprintSpawnableComponent))
class SODAROS2_API UROS2WheelInterfaceComponent 
	: public UWheeledVehicleComponent
	, public IROS2FMU
{
	GENERATED_UCLASS_BODY()

public:
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = Link, SaveGame, meta = (EditInRuntime, ReactivateActor, AllowedClasses = "/Script/UnrealSoda.TorqueTransmission"))
	FSubobjectReference LinkToTorqueTransmission{ TEXT("Differential") };

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = ROS2, SaveGame, meta = (EditInRuntime, ReactivateComponent))
	FString NodeName = DEFAULT_ROS2_NODE_NAME;

	/** Publication, [rad] */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = ROS2, SaveGame, meta = (EditInRuntime, ReactivateComponent))
	FROS2TopicSetup AngVelSignal{ TOPIC_SIG_TEMPLATE, TEXT("ang_vel") };

	/** Publication, [rad] */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = ROS2, SaveGame, meta = (EditInRuntime, ReactivateComponent))
	FROS2TopicSetup SteerSignal{ TOPIC_SIG_TEMPLATE, TEXT("steer") };

	/** Publication, [rad] */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = ROS2, SaveGame, meta = (EditInRuntime, ReactivateComponent))
	FROS2TopicSetup PitchSignal{ TOPIC_SIG_TEMPLATE, TEXT("pitch") };

	/** Publication, [normalized] */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = ROS2, SaveGame, meta = (EditInRuntime, ReactivateComponent))
	FROS2TopicSetup LonSlipSignal{ TOPIC_SIG_TEMPLATE, TEXT("lon_slip") };

	/** Publication, [normalized]  */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = ROS2, SaveGame, meta = (EditInRuntime, ReactivateComponent))
	FROS2TopicSetup LatSlipSignal{ TOPIC_SIG_TEMPLATE, TEXT("lat_slip") };

	/** Publication, [rad] */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = ROS2, SaveGame, meta = (EditInRuntime, ReactivateComponent))
	FROS2TopicSetup SuspensionOffsetSignal{ TOPIC_SIG_TEMPLATE, TEXT("suspension_offset") };

	/** Subscription, [H/m] */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = ROS2, SaveGame, meta = (EditInRuntime, ReactivateComponent))
	FROS2TopicSetup ReqTorqSignal{ TOPIC_SIG_TEMPLATE, TEXT("req_torq") };

	/** Subscription, [H/m] */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = ROS2, SaveGame, meta = (EditInRuntime, ReactivateComponent))
	FROS2TopicSetup ReqBrakeTorqueSignal{ TOPIC_SIG_TEMPLATE, TEXT("req_brake_torque") };

	/** Subscription, [rad] */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = ROS2, SaveGame, meta = (EditInRuntime, ReactivateComponent))
	FROS2TopicSetup ReqSteerSignal{ TOPIC_SIG_TEMPLATE, TEXT("req_steer") };

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

	struct FWheelPubSub
	{
		class USodaVehicleWheelComponent* Wheel = nullptr;

		ros2::TPublisherSignal<std_msgs::msg::Float32> PublisherAngularVelocity;
		ros2::TPublisherSignal<std_msgs::msg::Float32> PublisherSteer;
		ros2::TPublisherSignal<std_msgs::msg::Float32> PublisherPitch;
		ros2::TPublisherSignal<std_msgs::msg::Float32> PublisherLonSlip;
		ros2::TPublisherSignal<std_msgs::msg::Float32> PublisherLatSlip;
		ros2::TPublisherSignal<std_msgs::msg::Float32> PublisherSuspensionOffset;

		ros2::TSubscriptionSignal<std_msgs::msg::Float32> SubscriptionReqTorq;
		ros2::TSubscriptionSignal<std_msgs::msg::Float32> SubscriptionReqBrakeTorque;
		ros2::TSubscriptionSignal<std_msgs::msg::Float32> SubscriptionReqSteer;
	};

	TArray<FWheelPubSub> WheelsPubSub;
	TArray<FROS2TopicSetup> ConfiguredSignals;
};


