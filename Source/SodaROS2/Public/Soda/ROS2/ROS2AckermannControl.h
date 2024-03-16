// Copyright 2023 SODA.AUTO UK LTD. All Rights Reserved.

#pragma once

#include "Soda/GenericPublishers/GenericWheeledVehicleControl.h"
#include "Soda/VehicleComponents/Drivers/GenericVehicleDriverComponent.h"
#include "Soda/ROS2.h"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include "ROS2AckermannControl.generated.h"

/**
* UROS2AckermannControl
*/


UCLASS(ClassGroup = Soda, BlueprintType)
class SODAROS2_API UROS2AckermannControl : public UGenericWheeledVehicleControlListener
{
	GENERATED_BODY()

public:
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = ROS2AckermannControl, SaveGame, meta = (EditInRuntime, ReactivateComponent))
	FString NodeNamespace = "SodaSim";

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = ROS2AckermannControl, SaveGame, meta = (EditInRuntime, ReactivateComponent))
	FString Topic = "/vehicle/vehcile_control";

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = ROS2AckermannControl, SaveGame, meta = (DisplayName="QoS", EditInRuntime, ReactivateComponent))
	FQoS QoS;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = ROS2AckermannControl, SaveGame, meta = (EditInRuntime))
	FString FrameID = TEXT("base_link");

public:
	virtual bool StartListen(UVehicleBaseComponent* Parent) override;
	virtual void StopListen() override;
	virtual bool IsOk() const;
	virtual void DrawDebug(UCanvas* Canvas, float& YL, float& YPos) override;
	virtual bool GetControl(soda::FGenericWheeledVehiclControl& Control) const override;
	virtual FString GetRemark() const override;

protected:

	TSharedPtr<ros2::TSubscription<ackermann_msgs::msg::AckermannDriveStamped>> Subscription;
	ackermann_msgs::msg::AckermannDriveStamped Msg;
	TTimestamp RecvTimestamp;
};


