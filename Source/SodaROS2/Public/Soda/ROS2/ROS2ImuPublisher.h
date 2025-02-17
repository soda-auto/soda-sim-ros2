// Copyright 2023 SODA.AUTO UK LTD. All Rights Reserved.

#pragma once

#include "Soda/GenericPublishers/GenericNavPublisher.h"
#include "Soda/ROS2.h"
#include "sensor_msgs/msg/imu.hpp"
#include "ROS2ImuPublisher.generated.h"


UCLASS(ClassGroup = Soda, BlueprintType)
class SODAROS2_API UROS2ImuPublisher : public UGenericNavPublisher
{
	GENERATED_UCLASS_BODY()

public:
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = ROS2, SaveGame, meta = (EditInRuntime, ReactivateComponent))
	FString NodeName = DEFAULT_ROS2_NODE_NAME;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = ROS2, SaveGame, meta = (EditInRuntime, ReactivateComponent))
	FROS2TopicSetup TopicSetup{ TOPIC_SENSOR_TEMPLATE};

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = ROSLiROS2darPublisher, SaveGame, meta = (DisplayName = "QoS", EditInRuntime, ReactivateComponent))
	FQoS QoS;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = ROS2, SaveGame, meta = (EditInRuntime))
	FString FrameID = TEXT("base_link");

public:
	virtual bool Advertise(UVehicleBaseComponent* Parent) override;
	virtual void Shutdown() override;
	virtual bool IsOk() const override;
	virtual bool Publish(float DeltaTime, const FSensorDataHeader& Header, const FTransform& RelativeTransform, const FPhysBodyKinematic& VehicleKinematic, const FImuNoiseParams& Covariance) override;
	virtual FString GetRemark() const override;

private:
	TSharedPtr<ros2::TPublisher<sensor_msgs::msg::Imu>> Publisher;
	sensor_msgs::msg::Imu Msg;
	UVehicleBaseComponent* Parent = nullptr;
	FString FormatedTopic;
};

