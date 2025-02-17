// Copyright 2023 SODA.AUTO UK LTD. All Rights Reserved.

#pragma once

#include "Soda/VehicleComponents/IOBus.h"
#include "Misc/SpinLock.h"
#include "Soda/ROS2.h"
#include <soda_msgs/msg/io_pin_desc.hpp>
#include <soda_msgs/msg/io_pin_feedback.hpp>
#include <soda_msgs/msg/io_pin_src.hpp>
#include "IODevROS2.generated.h"


UCLASS()
class SODAROS2_API UIOPinROS2 : public UIOPin
{
	GENERATED_UCLASS_BODY()

public:
	void ConstruictPin(
		const UIOBusROS2Component * IOBusROS2Component,
		const FIOPinSetup& Setup);

	virtual void BeginDestroy() override;

	virtual void PublishSource(FIOPinSourceValue& Value) override;
	virtual void PublishFeedback(FIOPinFeedbackValue& Value) override;
	virtual const FIOPinSourceValue& GetInputSourceValue(FTimestamp* OutTimestamp = nullptr) const override;
	virtual const FIOPinFeedbackValue& GetInputFeedbackValue(FTimestamp* OutTimestamp = nullptr) const override;
	virtual const FIOPinSetup& GetLocalPinSetup() const override;
	virtual const FIOPinSetup& GetRemotePinSetup(FTimestamp* OutTimestamp = nullptr) const override;

	virtual const FIOPinSourceValue& GetLastPublishSourceValue() const { return LastPublishSourceValue; }
	virtual const FIOPinFeedbackValue& GetLastPublishFeedbackValue() const { return LastPublishFeedbackValue; }

protected:
	TSharedPtr <ros2_ue_wrapper::FNode> Node;

	TSharedPtr<ros2::TPublisher<soda_msgs::msg::IOPinSrc>> SourcePub; // Input & Output
	TSharedPtr<ros2::TPublisher<soda_msgs::msg::IOPinFeedback>> FeedbackPub; // Input only
	TSharedPtr<ros2::TPublisher<soda_msgs::msg::IOPinDesc>> DescPub;  // Input & Output

	TSharedPtr<ros2::TSubscription<soda_msgs::msg::IOPinSrc>> SourceSub; // Input only
	TSharedPtr<ros2::TSubscription<soda_msgs::msg::IOPinFeedback>> FeedbackSub; // Output only
	TSharedPtr<ros2::TSubscription<soda_msgs::msg::IOPinDesc>> DescSub;  // Input & Output

	FIOPinSourceValue InputSourceValue{};
	FIOPinFeedbackValue InputFeedbackValue{};
	FIOPinSourceValue LastPublishSourceValue{};
	FIOPinFeedbackValue LastPublishFeedbackValue{};
	FIOPinSetup LocalSetup{};
	FIOPinSetup RemoteSetup{};
	FTimestamp SourceValueTs{};
	FTimestamp FeedbackValueTs{};
	FTimestamp RemoteSetupTs{};
	mutable UE::FSpinLock SpinLock;

	FString FormatedSourceTopic;
	FString FormatedFeedbackTopic;
	FString FormatedDescTopic;

};


/**
 * UIOBusROS2Component
 */
UCLASS(ClassGroup = Soda, BlueprintType, Blueprintable, meta = (BlueprintSpawnableComponent))
class SODAROS2_API UIOBusROS2Component : public UIOBusComponent
{
	GENERATED_UCLASS_BODY()

public:
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = ROS2, SaveGame, meta = (EditInRuntime, ReactivateComponent))
	FString NodeName = DEFAULT_ROS2_NODE_NAME;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = ROS2, SaveGame, meta = (EditInRuntime, ReactivateComponent, EditCondition = bOverrideNamespace))
	FString TopicNamespace = DEFAULT_ROS2_TOPIC_NAMESPACE;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = ROS2, SaveGame, meta = (EditInRuntime, ReactivateComponent))
	bool bOverrideNamespace = false;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = ROS2, SaveGame, meta = (EditInRuntime, ReactivateComponent))
	FString SourceTopic = TEXT("{namespace}/vio/source/{signal_name}");

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = ROS2, SaveGame, meta = (EditInRuntime, ReactivateComponent))
	FString FeedbackTopic = TEXT("{namespace}/vio/feedback/{signal_name}");

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = ROS2, SaveGame, meta = (EditInRuntime, ReactivateComponent))
	FString DescTopic = TEXT("{namespace}/vio/desc/{signal_name}");

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = ROS2, SaveGame, meta = (DisplayName = "Pub Source QoS", EditInRuntime, ReactivateComponent))
	FQoS PubSourceQoS{};

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = ROS2, SaveGame, meta = (DisplayName = "Pub Feedback QoS", EditInRuntime, ReactivateComponent))
	FQoS PubFeedbackQoS{};

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = ROS2, SaveGame, meta = (DisplayName = "Pub Desc QoS", EditInRuntime, ReactivateComponent))
	FQoS PubDescQoS{};

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = ROS2, SaveGame, meta = (DisplayName = "Sub Source QoS", EditInRuntime, ReactivateComponent))
	FQoS SubSourceQoS{};

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = ROS2, SaveGame, meta = (DisplayName = "Sub Feedback QoS", EditInRuntime, ReactivateComponent))
	FQoS SubFeedbackQoS{};

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = ROS2, SaveGame, meta = (DisplayName = "Sub Desc QoS", EditInRuntime, ReactivateComponent))
	FQoS SubDescQoS{};

public:
	virtual UIOPin * CreatePin(const FIOPinSetup& Setup) override;
	virtual void ClearUnusedPins();

protected:
	virtual void TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction) override;

public:
	//virtual void OnSetExchangeValue(const UIOExchange* Link, const FIOPinSourceValue& Value) override;
	//virtual void OnChangePins() override;

protected:
	virtual bool OnActivateVehicleComponent() override;
	virtual void OnDeactivateVehicleComponent() override;
	virtual void DrawDebug(UCanvas* Canvas, float& YL, float& YPos) override;

protected:
	TArray<TWeakObjectPtr<UIOPinROS2>> Pins;
};
