// Copyright 2023 SODA.AUTO UK LTD. All Rights Reserved.

#pragma once

#include "Soda/VehicleComponents/VehicleBaseComponent.h"
#include "Soda/ROS2.h"
#include "tf2_msgs/msg/tf_message.hpp"
#include "ROS2TFPublisherComponent.generated.h"

class USodaVehicleWheelComponent;


UCLASS(ClassGroup = Soda, BlueprintType, Blueprintable, meta = (BlueprintSpawnableComponent))
class SODAROS2_API UROS2TFPublisherComponent : public UVehicleBaseComponent
{
	GENERATED_UCLASS_BODY()

protected:
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = TFPublisher, SaveGame, meta = (EditInRuntime, ReactivateComponent))
	FString NodeNamespace = "SodaSim";

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = TFPublisher, SaveGame, meta = (EditInRuntime, ReactivateComponent))
	FString Topic = "/tf";

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = TFPublisher, SaveGame, meta = (DisplayName = "QoS", EditInRuntime, ReactivateComponent))
	FQoS QoS;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = TFPublisher, SaveGame, meta = (EditInRuntime, ReactivateComponent))
	FString FrameID = TEXT("map");

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = TFPublisher, SaveGame, meta = (EditInRuntime, ReactivateComponent))
	FString VehicleBaseChildFrameID = TEXT("base_link");

	UPROPERTY(EditAnywhere, Category = TFPublisher, SaveGame, meta = (EditInRuntime, ReactivateComponent, AllowedClasses = "/Script/UnrealSoda.SodaVehicleComponent"))
	TArray<FSubobjectReference> ComponentsToPublish;

	
protected:
	virtual void TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction) override;

protected:
	virtual bool OnActivateVehicleComponent() override;
	virtual void OnDeactivateVehicleComponent() override;
	virtual void DrawDebug(UCanvas* Canvas, float& YL, float& YPos) override;
	virtual FString GetRemark() const override;
	
protected:
	void Shutdown();
	TSharedPtr<ros2::TPublisher<tf2_msgs::msg::TFMessage>> Publisher;
	tf2_msgs::msg::TFMessage Msg;

	TArray<USodaVehicleWheelComponent*> WheelComponentsToPublish;
	TArray<USceneComponent*> SceneComponentsToPublish;
};