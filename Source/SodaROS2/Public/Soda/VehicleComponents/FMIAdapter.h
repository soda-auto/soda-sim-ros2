// Copyright 2023 SODA.AUTO UK LTD. All Rights Reserved.

#pragma once

#include "Soda/VehicleComponents/VehicleBaseComponent.h"
#include "Soda/ROS2.h"
#include "Soda/ROS2/IROS2FMU.h"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/bool.hpp"

#include "Windows/AllowWindowsPlatformTypes.h" 
#include <fmilib.h>
#include "Windows/HideWindowsPlatformTypes.h"


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


	struct FFMUPublisher
	{


		ERosMessageTyp MessageTyp;
		FString VarName;
		TSharedPtr<ros2::FPubliserSignalBase> Publisher;
	};

	struct FFMUSubscriber
	{


		ERosMessageTyp MessageTyp;
		FString VarName;
		TSharedPtr<ros2::FSubscriberSignalBase> Subscriber;
	};


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
	float  step_size_ = 0.01;

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


	void setup_fmu();
	void initialize_parameters_and_topics();
	void setup_publishers_and_subscribers();
	void run_step();
	void publish_outputs();
	void UpdateInputs();
	void RunNode();

	void cleanup();
	void handle_input(const FString& variable_name, double value);
	FString normalize_topic_name(const std::string& name);

protected:
	virtual bool OnActivateVehicleComponent() override;
	virtual void OnDeactivateVehicleComponent() override;
	virtual void PostPhysicSimulationDeferred(float DeltaTime, const FPhysBodyKinematic& VehicleKinematic, const TTimestamp& Timestamp) override;
	virtual void DrawDebug(UCanvas* Canvas, float& YL, float& YPos) override;

protected:

	TArray<FFMUPublisher> PublisherArray;
	TArray<FFMUSubscriber> SubscriberArray;

private:

    double current_time_;
    std::string unpacked_dir_;
    fmi2_import_t *fmu_;
    fmi_import_context_t *context_;
    jm_callbacks callbacks;
    fmi_version_enu_t version;
    const char* tmpPath;
    fmi2_callback_functions_t callBackFunctions;

	    static void importlogger(jm_callbacks* c, jm_string module, jm_log_level_enu_t log_level, jm_string message) 
		{

		}

};


