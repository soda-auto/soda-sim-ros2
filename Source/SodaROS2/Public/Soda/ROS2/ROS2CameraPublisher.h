// Copyright 2023 SODA.AUTO UK LTD. All Rights Reserved.

#pragma once

#include "Soda/GenericPublishers/GenericCameraPublisher.h"
#include "Soda/ROS2.h"
#include "sensor_msgs/image_encodings.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "ROS2CameraPublisher.generated.h"


UCLASS(ClassGroup = Soda, BlueprintType)
class SODAROS2_API UROS2CameraPublisher : public UGenericCameraPublisher
{
	GENERATED_UCLASS_BODY()

public:
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = ROSCameraPublisher, SaveGame, meta = (EditInRuntime, ReactivateComponent))
	FString NodeNamespace = "SodaSim";

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = ROSCameraPublisher, SaveGame, meta = (EditInRuntime, ReactivateComponent))
	FString Topic = "/vehicle/camera";

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = ROSCameraPublisher, SaveGame, meta = (DisplayName = "QoS", EditInRuntime, ReactivateComponent))
	FQoS QoS;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = ROSCameraPublisher, SaveGame, meta = (EditInRuntime))
	FString FrameID = TEXT("base_link");

public:
	virtual bool Advertise(UVehicleBaseComponent* Parent) override;
	virtual void Shutdown() override;
	virtual bool IsOk() const override;
	virtual bool Publish(float DeltaTime, const FSensorDataHeader& Header, const FCameraFrame& CameraFrame, const TArray<FColor>& BGRA8, uint32 ImageStride) override;
	virtual FString GetRemark() const override;

private:
	TSharedPtr<ros2::TPublisher<sensor_msgs::msg::Image>> Publisher;
	sensor_msgs::msg::Image Msg;
};

