// Copyright 2023 SODA.AUTO UK LTD. All Rights Reserved.

#pragma once

#include "Soda/VehicleComponents/CANDev.h"
#include "Async/AsyncWork.h"
#include "Soda/ROS2.h"
#include <soda_msgs/msg/can_frame.hpp>
#include "CANDevROS2.generated.h"

namespace ros2_ue_wrapper
{
	class VCan;
	class FNode;
}

/**
 * USocketCANDevComponent provides CAN device via the SocketCAN driver (only for Linux)
 */
UCLASS(ClassGroup = Soda, BlueprintType, Blueprintable)
class SODAROS2_API UCANDevROS2 : public UCANDevComponent
{
	GENERATED_UCLASS_BODY()

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = ROS2, SaveGame, meta = (EditInRuntime, ReactivateComponent))
	FString NodeName = DEFAULT_ROS2_NODE_NAME;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = ROS2, SaveGame, meta = (EditInRuntime, ReactivateComponent))
	FROS2TopicSetup TopicSetup{ TEXT("{namespace}/vcan/{dev_name}") };

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = ROS2, SaveGame, meta = (DisplayName = "Pub QoS", EditInRuntime, ReactivateComponent))
	FQoS PubQoS{};

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = ROS2, SaveGame, meta = (DisplayName = "Sub QoS", EditInRuntime, ReactivateComponent))
	FQoS SubQoS{};

public:
	virtual int SendFrame(const dbc::FCanFrame& CanFrame) override;

protected:
	virtual bool OnActivateVehicleComponent() override;
	virtual void OnDeactivateVehicleComponent() override;

protected:
	TSharedPtr<ros2_ue_wrapper::FNode> Node;
	TSharedPtr<ros2::TPublisher<soda_msgs::msg::CANFrame>> Publisher;
	TSharedPtr<ros2::TSubscription<soda_msgs::msg::CANFrame>> Subscription;
	FString FormatedTopic;

};

