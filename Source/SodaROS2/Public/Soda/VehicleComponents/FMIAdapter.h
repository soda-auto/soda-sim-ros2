// Copyright 2023 SODA.AUTO UK LTD. All Rights Reserved.

#pragma once

#include "Soda/VehicleComponents/VehicleBaseComponent.h"
#include "Soda/ROS2.h"
#include "Soda/ROS2/IROS2FMU.h"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/bool.hpp"

#if PLATFORM_WINDOWS
#include "Windows/AllowWindowsPlatformTypes.h" 
#include <fmilib.h>
#include "Windows/HideWindowsPlatformTypes.h"
#endif


#include "FMIAdapter.generated.h"

/**
 *  FPubliserSignalBase
*/
UENUM(BlueprintType)
enum class ERosMessageTyp : uint8 {
	Int      UMETA(DisplayName = "Int"),
	Float      UMETA(DisplayName = "Float"),
	Bool      UMETA(DisplayName = "Bool"),
};

/**
 * FFMUPublisher
 */
struct FFMUPublisher
{
	ERosMessageTyp MessageTyp;
	FString VarName;
	TSharedPtr<ros2::FPubliserSignalBase> Publisher;
};

/**
 * FFMUSubscriber
 */
struct FFMUSubscriber
{
	ERosMessageTyp MessageTyp;
	FString VarName;
	TSharedPtr<ros2::FSubscriberSignalBase> Subscriber;
};

/**
 * FFmuParam
 */
USTRUCT(BlueprintType)
struct FFmuParam
{
	GENERATED_BODY()

public:

	UPROPERTY(EditAnywhere, BlueprintReadWrite, SaveGame, meta = (EditInRuntime))
	FName Name;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, SaveGame, meta = (EditInRuntime))
	float Value = 0.0;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, SaveGame, meta = (EditInRuntime))
	ERosMessageTyp Type = ERosMessageTyp::Bool;
};




/**
 * UFMIAdapterComponent
 */
UCLASS(ClassGroup = Soda, BlueprintType, Blueprintable, meta = (BlueprintSpawnableComponent))
class SODAROS2_API UFMIAdapterComponent 
	: public UVehicleBaseComponent
	, public IROS2FMU
{
	GENERATED_UCLASS_BODY()

public:

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = ROS2, SaveGame, meta = (EditInRuntime, ReactivateComponent))
	FString NodeName = DEFAULT_ROS2_NODE_NAME;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = ROS2, SaveGame, meta = (EditInRuntime, ReactivateComponent))
	FString FmuPath;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = ROS2, SaveGame, meta = (EditInRuntime, ReactivateComponent))
	float StepSize = 0.01;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = ROS2, SaveGame, meta = (EditInRuntime))
	TArray<FFmuParam> Parameters;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = ROS2, SaveGame,meta = (EditInRuntime))
	TArray<FString> PublishersNames;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = ROS2, SaveGame,meta = (EditInRuntime))
	TArray<FString> SubscribersNames;
	
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = ROS2, SaveGame, meta = (DisplayName = "Pub QoS", EditInRuntime, ReactivateComponent))
	FQoS PubQoS{};
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = ROS2, SaveGame, meta = (DisplayName = "Sub QoS", EditInRuntime, ReactivateComponent))
	FQoS SubQoS{};

	UFUNCTION(CallInEditor, Category = LoadFMU, meta = (DisplayName = "Specify path to FMU model", CallInRuntime))
	void SpecifyFmuModel();

public:
	virtual bool ConfigureSignal(const FROS2TopicSetup& Setup) override;

#if PLATFORM_WINDOWS
	void InitializeFMU();
	void InitializeParametersAndTopics();
	void SetupPublishersAndSubscribers();
	void RunSimulationStep();
	void PublisOutputs();
	void UpdateInputs();
	void PerformNodeSimulation();

	void CleanUp();
	void HandleInput(const FString& VariableName, double value);
#endif


protected:
	virtual bool OnActivateVehicleComponent() override;
	virtual void OnDeactivateVehicleComponent() override;
	virtual void PostPhysicSimulationDeferred(float DeltaTime, const FPhysBodyKinematic& VehicleKinematic, const TTimestamp& Timestamp) override;
	virtual void DrawDebug(UCanvas* Canvas, float& YL, float& YPos) override;

protected:

	TArray<FFMUPublisher> PublisherArray;
	TArray<FFMUSubscriber> SubscriberArray;

private:

#if PLATFORM_WINDOWS
    double CurrentTime;
    std::string unpacked_dir_;
    fmi2_import_t *FmuModel;
    fmi_import_context_t *FmuContext;
    jm_callbacks FmuCallbacks;
    fmi_version_enu_t FmiVersion;
    const char* FmiTmpPath;
    fmi2_callback_functions_t CallBackFunctions;

	static void importlogger(jm_callbacks* c, jm_string module, jm_log_level_enu_t log_level, jm_string message) {};
#endif
};


