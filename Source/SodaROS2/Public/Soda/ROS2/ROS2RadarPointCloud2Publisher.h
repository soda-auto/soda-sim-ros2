// Copyright 2023 SODA.AUTO UK LTD. All Rights Reserved.

#pragma once

#include "Soda/GenericPublishers/GenericRadarPublisher.h"
#include "Soda/VehicleComponents/Sensors/Base/RadarSensor.h"
#include "Soda/ROS2.h"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/point_field.hpp"
#include "ROS2RadarPointCloud2Publisher.generated.h"


UCLASS(ClassGroup = Soda, BlueprintType)
class SODAROS2_API UROS2RadarPointCloud2Publisher : public UGenericRadarPublisher
{
	GENERATED_UCLASS_BODY()

public:
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = ROS2, SaveGame, meta = (EditInRuntime, ReactivateComponent))
	FString NodeName = DEFAULT_ROS2_NODE_NAME;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = ROS2, SaveGame, meta = (EditInRuntime, ReactivateComponent))
	FROS2TopicSetup TopicSetup{ TOPIC_SENSOR_TEMPLATE };

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = ROS2, SaveGame, meta = (DisplayName = "QoS", EditInRuntime, ReactivateComponent))
	FQoS QoS;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = ROS2, SaveGame, meta = (EditInRuntime))
	FString FrameID = TEXT("base_link");

public:
	virtual bool Advertise(UVehicleBaseComponent* Parent) override;
	virtual void Shutdown() override;
	virtual bool IsOk() const override;
	virtual bool Publish(float DeltaTime, const FSensorDataHeader& Header, const TArray<FRadarParams>& Params, const FRadarClusters& Clusters, const FRadarObjects& Objects) override;
	virtual FString GetRemark() const override;

private:
	TSharedPtr<ros2::TPublisher<sensor_msgs::msg::PointCloud2>> Publisher;
	sensor_msgs::msg::PointCloud2 Msg;
	FString FormatedTopic;
	ERadarMode Mode;
};

