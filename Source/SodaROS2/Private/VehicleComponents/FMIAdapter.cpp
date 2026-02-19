// Copyright 2023 SODA.AUTO UK LTD. All Rights Reserved.

#include "Soda/VehicleComponents/FMIAdapter.h"
#include "Soda/UnrealSoda.h"
#include "Soda/Vehicles/SodaWheeledVehicle.h"
#include "Engine/Canvas.h"
#include "Misc/FileHelper.h"
#include "Misc/Paths.h"
#include "HAL/PlatformFileManager.h"
#include "Containers/UnrealString.h"
#include "Misc/OutputDeviceDebug.h"
#include "DesktopPlatformModule.h"
#include "Engine/Engine.h"
#include "Interfaces/IPluginManager.h"
#include "Algo/Find.h"


FString normalize_topic_name(const std::string& name)
{
	std::string normalized_name = name;
	std::replace(normalized_name.begin(), normalized_name.end(), '.', '_');
	std::replace(normalized_name.begin(), normalized_name.end(), '[', '_');
	std::replace(normalized_name.begin(), normalized_name.end(), ']', '_');
	std::replace(normalized_name.begin(), normalized_name.end(), '{', '_');
	std::replace(normalized_name.begin(), normalized_name.end(), '}', '_');
	return  FString(normalized_name.c_str());;
}



UFMIAdapterComponent::UFMIAdapterComponent(const FObjectInitializer& ObjectInitializer)
	: Super(ObjectInitializer)
{
	GUI.Category = TEXT("Vehicle Mechanicles");
	GUI.IcanName = TEXT("SodaIcons.Motor");
	GUI.ComponentNameOverride = TEXT("FMU Adapter");
	GUI.bIsPresentInAddMenu = true;

	Common.Activation = EVehicleComponentActivation::OnStartScenario;

	PrimaryComponentTick.bCanEverTick = true;
	TickData.bAllowVehiclePostDeferredPhysTick = true;
}



#if PLATFORM_WINDOWS

bool UFMIAdapterComponent::OnActivateVehicleComponent()
{
	if (!Super::OnActivateVehicleComponent())
	{
		return false;
	}

	InitializeFMU();
	

	return true;
}


void UFMIAdapterComponent::PostPhysicSimulationDeferred(float DeltaTime, const FPhysBodyKinematic& VehicleKinematic, const TTimestamp& Timestamp)
{
	Super::PostPhysicSimulationDeferred(DeltaTime, VehicleKinematic, Timestamp);

	if (GetHealth() != EVehicleComponentHealth::Ok)
	{
		return;
	}

	PerformNodeSimulation();

}

void UFMIAdapterComponent::OnDeactivateVehicleComponent()
{
	Super::OnDeactivateVehicleComponent();

	for (auto& Publisher : PublisherArray)
	{
		if (Publisher.Publisher.IsValid())
		{
			Publisher.Publisher->Stop();
		}
	}

	for (auto& Subscriber : SubscriberArray)
	{
		if (Subscriber.Subscriber.IsValid())
		{
			Subscriber.Subscriber->Stop();
		}
	}

	CleanUp();

}



void UFMIAdapterComponent::DrawDebug(UCanvas* Canvas, float& YL, float& YPos)
{
	Super::DrawDebug(Canvas, YL, YPos);
	
}

bool UFMIAdapterComponent::ConfigureSignal(const FROS2TopicSetup& Setup)
{	
	return false;
}



void UFMIAdapterComponent::InitializeFMU()
{
	if (!FPlatformFileManager::Get().GetPlatformFile().FileExists(*FmuPath))
	{
		UE_LOG(LogTemp, Error, TEXT("FMU does not exist: %s"), *FmuPath);
		SetHealth(EVehicleComponentHealth::Error, TEXT("FMU file doesn't exist"));
		return;
	}

	TSharedPtr<IPlugin> Plugin = IPluginManager::Get().FindPlugin(TEXT("soda-sim-ros2"));
	if (!Plugin.IsValid())
	{
		UE_LOG(LogTemp, Error, TEXT("Failed to find soda-sim-ros2 plugin. Cannot load fmilib_shared.dll."));
		SetHealth(EVehicleComponentHealth::Error, TEXT("Plugin not found"));
		return;
	}

	FString PluginBaseDir = Plugin->GetBaseDir();
	FString DllPath = FPaths::Combine(PluginBaseDir, TEXT("Binaries/Win64/fmilib_shared.dll"));

	FPlatformProcess::PushDllDirectory(*FPaths::GetPath(DllPath));

	void* Handle = FPlatformProcess::GetDllHandle(*DllPath);

	if (Handle)
	{
		UE_LOG(LogTemp, Log, TEXT("Successfully loaded DLL: %s"), *DllPath);
	}
	else
	{
		UE_LOG(LogTemp, Error, TEXT("Failed to load DLL: %s"), *DllPath);
		return;
	}

	FmuCallbacks.malloc = malloc;
	FmuCallbacks.calloc = calloc;
	FmuCallbacks.realloc = realloc;
	FmuCallbacks.free = free;
	FmuCallbacks.logger = importlogger;
	FmuCallbacks.log_level = jm_log_level_debug;
	FmuCallbacks.context = 0;

	FString TempDir = FPaths::ProjectSavedDir();
	std::string TempDirStr = TCHAR_TO_UTF8(*TempDir);

	FmiTmpPath = fmi_import_mk_temp_dir(&FmuCallbacks, TempDirStr.c_str(), nullptr);
	if (!FmiTmpPath) 
	{
		SetHealth(EVehicleComponentHealth::Error, TEXT("Failed to create temporary directory"));
	}
	
	// Allocate FMILibrary context
	FmuContext = fmi_import_allocate_context(&FmuCallbacks);
	
	std::string FmuPathStr = TCHAR_TO_UTF8(*FmuPath);
	FmiVersion = fmi_import_get_fmi_version(FmuContext, FmuPathStr.c_str(), FmiTmpPath);	

	if (FmiVersion != fmi_version_2_0_enu) {

		SetHealth(EVehicleComponentHealth::Error, TEXT("Only FMI version 2.0 is supported"));
	}

	FmuModel = fmi2_import_parse_xml(FmuContext, FmiTmpPath, nullptr);
	if (!FmuModel) 
	{
		SetHealth(EVehicleComponentHealth::Error, TEXT("Error parsing FMU XML"));
	}

	if (fmi2_import_get_fmu_kind(FmuModel) != fmi2_fmu_kind_cs) 
	{
		SetHealth(EVehicleComponentHealth::Error, TEXT("Only Co-Simulation is supported"));

	}

	CallBackFunctions.logger = fmi2_log_forwarding;
	CallBackFunctions.allocateMemory = calloc;
	CallBackFunctions.freeMemory = free;
	CallBackFunctions.stepFinished = nullptr;
	CallBackFunctions.componentEnvironment = FmuModel;

	if (fmi2_import_create_dllfmu(FmuModel, fmi2_fmu_kind_cs, &CallBackFunctions) != jm_status_success) 
	{
		SetHealth(EVehicleComponentHealth::Error, TEXT("Failed to create DLL loading mechanism"));
	}

	if (fmi2_import_instantiate(FmuModel, "FMU_instance", fmi2_cosimulation, nullptr, fmi2_false) != jm_status_success) 
	{
		SetHealth(EVehicleComponentHealth::Error, TEXT("Failed to instantiate FMU"));
	}

	if (fmi2_import_setup_experiment(FmuModel, fmi2_true, 1e-4, 0.0, fmi2_false, 0.0) != fmi2_status_ok) 
	{
		SetHealth(EVehicleComponentHealth::Error, TEXT("Failed to setup FMU experiment"));
	}

	if (fmi2_import_enter_initialization_mode(FmuModel) != fmi2_status_ok) 
	{
		SetHealth(EVehicleComponentHealth::Error, TEXT("Failed to enter initialization mode"));
	}

	if (fmi2_import_exit_initialization_mode(FmuModel) != fmi2_status_ok) 
	{
		SetHealth(EVehicleComponentHealth::Error, TEXT("Failed to exit initialization mode"));
	}

	SetupPublishersAndSubscribers();
}



void UFMIAdapterComponent::InitializeParametersAndTopics()
{


	fmi2_import_variable_list_t* variable_list = fmi2_import_get_variable_list(FmuModel, 0);
	size_t num_variables = fmi2_import_get_variable_list_size(variable_list);
	Parameters.Empty();

	for (size_t i = 0; i < num_variables; i++) {
		auto variable = fmi2_import_get_variable(variable_list, i);
		
		const char* var_name = fmi2_import_get_variable_name(variable);
		auto var_type = fmi2_import_get_variable_base_type(variable);
		fmi2_causality_enu_t causality = fmi2_import_get_causality(variable);

		if (causality != fmi2_causality_enu_parameter)
		{
			continue;
		}

		if (var_type == fmi2_base_type_real) 
		{	
			fmi2_value_reference_t vr = fmi2_import_get_variable_vr(variable);
			fmi2_real_t value;
			fmi2_import_get_real(FmuModel, &vr, 1, &value);

			FFmuParam Param;
			Param.Name = FName(UTF8_TO_TCHAR(var_name));
			Param.Type = ERosMessageTyp::Float;
			Param.Value = value;

			Parameters.Add(Param);

		}
		else if (var_type == fmi2_base_type_int) 
		{
			fmi2_value_reference_t vr = fmi2_import_get_variable_vr(variable);
			fmi2_integer_t value;
			fmi2_import_get_integer(FmuModel, &vr, 1, &value);

			FFmuParam Param;
			Param.Name = FName(UTF8_TO_TCHAR(var_name));
			Param.Type = ERosMessageTyp::Int;
			Param.Value = value;

			Parameters.Add(Param);

		}
		else if (var_type == fmi2_base_type_bool) 
		{
			fmi2_value_reference_t vr = fmi2_import_get_variable_vr(variable);
			fmi2_boolean_t value;
			fmi2_import_get_boolean(FmuModel, &vr, 1, &value);

			FFmuParam Param;
			Param.Name = FName(UTF8_TO_TCHAR(var_name));
			Param.Type = ERosMessageTyp::Bool;
			Param.Value = value == fmi2_true ? 1.0 : 0.0;

			Parameters.Add(Param);
		}
	}

	fmi2_import_free_variable_list(variable_list);
}

void UFMIAdapterComponent::SetupPublishersAndSubscribers()
{
	SubscribersNames.Empty();
	PublishersNames.Empty();

	fmi2_import_variable_list_t* variable_list = fmi2_import_get_variable_list(FmuModel, 0);
	size_t num_variables = fmi2_import_get_variable_list_size(variable_list);

	for (size_t i = 0; i < num_variables; i++) {
		auto variable = fmi2_import_get_variable(variable_list, i);
		const char* var_name = fmi2_import_get_variable_name(variable);
		auto var_causality = fmi2_import_get_causality(variable);
		auto var_type = fmi2_import_get_variable_base_type(variable);

		// create TMap of publishers and subscripers

		FString topic_name = normalize_topic_name(std::string("/") + var_name);

		if (var_causality == fmi2_causality_enu_output)
		{
			if (var_type == fmi2_base_type_real) 
			{				
				auto Publisher = MakeShared<ros2::TPublisherSignal<std_msgs::msg::Float32>>();				

				if (Publisher->Start(NodeName, topic_name, PubQoS))
				{
					UE_LOG(LogSoda, Log, TEXT("UFMIAdapterComponent create float32 publisher %s"),*topic_name);
					PublishersNames.Add(FString("Float32: ") + topic_name);
				}
				FFMUPublisher NewPublisher;

				NewPublisher.MessageTyp = ERosMessageTyp::Float;
				NewPublisher.Publisher = Publisher;
				NewPublisher.VarName = topic_name;
				PublisherArray.Add(NewPublisher);		

			}
			else if (var_type == fmi2_base_type_int) 
			{
				auto Publisher = MakeShared<ros2::TPublisherSignal<std_msgs::msg::Int32>>();

				if (Publisher->Start(NodeName, topic_name, PubQoS))
				{
					UE_LOG(LogSoda, Log, TEXT("UFMIAdapterComponent create int publisher %s"), *topic_name);
					PublishersNames.Add(FString("int: ") + topic_name);
				}
				FFMUPublisher NewPublisher;

				NewPublisher.MessageTyp = ERosMessageTyp::Int;
				NewPublisher.Publisher = Publisher;
				NewPublisher.VarName = topic_name;
				PublisherArray.Add(NewPublisher);
			}
			else if (var_type == fmi2_base_type_bool) 
			{
				auto Publisher = MakeShared<ros2::TPublisherSignal<std_msgs::msg::Bool>>();

				if (Publisher->Start(NodeName, topic_name, PubQoS))
				{
					UE_LOG(LogSoda, Log, TEXT("UFMIAdapterComponent create bool publisher %s"), *topic_name);
					PublishersNames.Add(FString("bool: ") + topic_name);
				}
				FFMUPublisher NewPublisher;

				NewPublisher.MessageTyp = ERosMessageTyp::Bool;
				NewPublisher.Publisher = Publisher;
				NewPublisher.VarName = topic_name;
				PublisherArray.Add(NewPublisher);
			}
		}
		else if (var_causality == fmi2_causality_enu_input)
		{
			if (var_type == fmi2_base_type_real)
			{
				auto Subscriber = MakeShared<ros2::TSubscriptionSignal<std_msgs::msg::Float32>>();

				if (Subscriber->Start(NodeName, topic_name, PubQoS))
				{
					UE_LOG(LogSoda, Log, TEXT("UFMIAdapterComponent create float subscription %s"), *topic_name);
					SubscribersNames.Add(FString("Float32: ") + topic_name);
				}
				FFMUSubscriber NewSubscriber;

				NewSubscriber.MessageTyp = ERosMessageTyp::Float;
				NewSubscriber.Subscriber = Subscriber;
				NewSubscriber.VarName = topic_name;
				SubscriberArray.Add(NewSubscriber);					

			}
			else if (var_type == fmi2_base_type_int)
			{
				auto Subscriber = MakeShared<ros2::TSubscriptionSignal<std_msgs::msg::Int32>>();

				if (Subscriber->Start(NodeName, topic_name, PubQoS))
				{
					UE_LOG(LogSoda, Log, TEXT("UFMIAdapterComponent create int32 subscription %s"), *topic_name);
					SubscribersNames.Add(FString("int: ") + topic_name);
				}
				FFMUSubscriber NewSubscriber;

				NewSubscriber.MessageTyp = ERosMessageTyp::Int;
				NewSubscriber.Subscriber = Subscriber;
				NewSubscriber.VarName = topic_name;
				SubscriberArray.Add(NewSubscriber);
			}
			else if (var_type == fmi2_base_type_bool) 
			{
				auto Subscriber = MakeShared<ros2::TSubscriptionSignal<std_msgs::msg::Bool>>();

				if (Subscriber->Start(NodeName, topic_name, PubQoS))
				{
					UE_LOG(LogSoda, Log, TEXT("UFMIAdapterComponent create bool subscription %s"), *topic_name);
					SubscribersNames.Add(FString("Bool: ") + topic_name);
				}
				FFMUSubscriber NewSubscriber;

				NewSubscriber.MessageTyp = ERosMessageTyp::Bool;
				NewSubscriber.Subscriber = Subscriber;
				NewSubscriber.VarName = topic_name;
				SubscriberArray.Add(NewSubscriber);
			}
		}
	}

	fmi2_import_free_variable_list(variable_list);
	InitializeParametersAndTopics();
}


 void UFMIAdapterComponent::RunSimulationStep()
 {

	 if (fmi2_import_do_step(FmuModel, CurrentTime, StepSize, fmi2_true) != fmi2_status_ok)
	 {
		 UE_LOG(LogSoda, Error, TEXT("UFMIAdapterComponent step failed"));
	 }

	 CurrentTime += StepSize;
	 PublisOutputs();
	  
 }


 void UFMIAdapterComponent::PublisOutputs()
 {
	 fmi2_import_variable_list_t* variable_list = fmi2_import_get_variable_list(FmuModel, 0);
	 size_t num_variables = fmi2_import_get_variable_list_size(variable_list);

	 for (size_t i = 0; i < num_variables; i++) {
		 auto variable = fmi2_import_get_variable(variable_list, i);
		 const char* var_name = fmi2_import_get_variable_name(variable);
		 auto var_causality = fmi2_import_get_causality(variable);
		 auto var_type = fmi2_import_get_variable_base_type(variable);

		 if (var_causality == fmi2_causality_enu_output)
		 {

			 FFMUPublisher* FoundPublisher = Algo::FindByPredicate(PublisherArray, [var_name](const FFMUPublisher& Publisher)
				 {
					 return Publisher.VarName == FString("/")+UTF8_TO_TCHAR(var_name);
				 });


			 for (auto& Elem : PublisherArray)
			 {
				 UE_LOG(LogTemp, Log, TEXT("publisher name %s"), *Elem.VarName);
			 }

			 if (FoundPublisher == nullptr)
			 {
				 UE_LOG(LogTemp, Log, TEXT("Didn't find publisher with the name %s"), UTF8_TO_TCHAR(var_name));
			 }
			 else
			 {
				 UE_LOG(LogTemp, Log, TEXT("Find publisher with the name %s"), UTF8_TO_TCHAR(var_name));
			 }

			 if (FoundPublisher != nullptr)
			 {
				 fmi2_value_reference_t vr = fmi2_import_get_variable_vr(variable);

				 if (var_type == fmi2_base_type_real)
				 {
					 fmi2_real_t value;
					 fmi2_import_get_real(FmuModel, &vr, 1, &value);

					 auto TypedPuplisher = StaticCastSharedPtr<ros2::TPublisherSignal<std_msgs::msg::Float32>>(FoundPublisher->Publisher);
					 if (TypedPuplisher.IsValid())
					 {
						 TypedPuplisher->Publish(value);

					 }
				 }
				 else if (var_type == fmi2_base_type_int)
				 {
					 fmi2_integer_t value;
					 fmi2_import_get_integer(FmuModel, &vr, 1, &value);

					 auto TypedPuplisher = StaticCastSharedPtr<ros2::TPublisherSignal<std_msgs::msg::Int32>>(FoundPublisher->Publisher);
					 if (TypedPuplisher.IsValid())
					 {
						 TypedPuplisher->Publish(value);

					 }

				 }
				 else if (var_type == fmi2_base_type_bool)
				 {
					 fmi2_boolean_t value;
					 fmi2_import_get_boolean(FmuModel, &vr, 1, &value);

					 auto TypedPuplisher = StaticCastSharedPtr<ros2::TPublisherSignal<std_msgs::msg::Bool>>(FoundPublisher->Publisher);
					 if (TypedPuplisher.IsValid())
					 {
						 bool OutputValue = (value == fmi2_true);						
						 UE_LOG(LogTemp, Log, TEXT("Bool output detected %d"),(int32)OutputValue);
						 TypedPuplisher->Publish(OutputValue);

					 }
				 }
			 }
		 }
	 }

	 fmi2_import_free_variable_list(variable_list);
 }

 void UFMIAdapterComponent::UpdateInputs()
 {

	 for (FFMUSubscriber& Subscriber : SubscriberArray)
	 {
		 if (Subscriber.MessageTyp == ERosMessageTyp::Bool)
		 {
			 auto TypedSubscriber = StaticCastSharedPtr<ros2::TSubscriptionSignal<std_msgs::msg::Bool>>(Subscriber.Subscriber);
			 if (TypedSubscriber.IsValid())
			 {
				 HandleInput(Subscriber.VarName, (double)TypedSubscriber->GetSignal());
			 }
		 }
		 else if (Subscriber.MessageTyp == ERosMessageTyp::Int)
		 {
			 auto TypedSubscriber = StaticCastSharedPtr<ros2::TSubscriptionSignal<std_msgs::msg::Int32>>(Subscriber.Subscriber);
			 if (TypedSubscriber.IsValid())
			 {
				 HandleInput(Subscriber.VarName, (double)TypedSubscriber->GetSignal());
			 }
		 }
		 else if (Subscriber.MessageTyp == ERosMessageTyp::Float)
		 {
			 auto TypedSubscriber = StaticCastSharedPtr<ros2::TSubscriptionSignal<std_msgs::msg::Float32>>(Subscriber.Subscriber);
			 if (TypedSubscriber.IsValid())
			 {
				 UE_LOG(LogTemp, Log, TEXT("Float input detected, value %f"), TypedSubscriber->GetSignal());
				 HandleInput(Subscriber.VarName, (double)TypedSubscriber->GetSignal());
			 }
		 }
	 }

 }


	void UFMIAdapterComponent::PerformNodeSimulation()
	{
		UpdateInputs();
		RunSimulationStep();
	}

	void UFMIAdapterComponent::CleanUp()
	{
		if (GetHealth() != EVehicleComponentHealth::Ok)
		{
			return;
		}

		if (FmuModel) 
		{
			fmi2_import_terminate(FmuModel);
			fmi2_import_free_instance(FmuModel);
			fmi2_import_destroy_dllfmu(FmuModel);
			fmi2_import_free(FmuModel);
		}

		if (FmuContext) 
		{
			fmi_import_free_context(FmuContext);
		}

		if (fmi_import_rmdir(&FmuCallbacks, FmiTmpPath)) 
		{
			UE_LOG(LogSoda, Fatal, TEXT("FProblem when deleting FMU unpack directory"));
		}

		FmuCallbacks.free((void*)FmiTmpPath);
	}

	void UFMIAdapterComponent::HandleInput(const FString& VariableName, double value)
	{

		// Find the variable in the FMU by its name
		fmi2_import_variable_list_t* variable_list = fmi2_import_get_variable_list(FmuModel, 0);
		size_t num_variables = fmi2_import_get_variable_list_size(variable_list);

		for (size_t i = 0; i < num_variables; i++) {
			auto variable = fmi2_import_get_variable(variable_list, i);
			const char* var_name = fmi2_import_get_variable_name(variable);

			UE_LOG(LogSoda, Warning, TEXT("UFMIAdapterComponent: compate variable %s with %s"), *VariableName, *FString(UTF8_TO_TCHAR(var_name)));

			FString VarAsFString = FString("/") + FString(UTF8_TO_TCHAR(var_name));

			if (VariableName == VarAsFString)
			{
				fmi2_value_reference_t vr = fmi2_import_get_variable_vr(variable);
				auto var_type = fmi2_import_get_variable_base_type(variable);

				// Update the FMU variable based on its type
				if (var_type == fmi2_base_type_real)
				{
					fmi2_real_t real_value = static_cast<fmi2_real_t>(value);
					fmi2_import_set_real(FmuModel, &vr, 1, &real_value);
					UE_LOG(LogSoda, Warning, TEXT("UFMIAdapterComponent: variable %s was written with %f value"), *VariableName, real_value);
				}
				else if (var_type == fmi2_base_type_int)
				{
					fmi2_integer_t int_value = static_cast<fmi2_integer_t>(value);
					fmi2_import_set_integer(FmuModel, &vr, 1, &int_value);
				}
				else if (var_type == fmi2_base_type_bool)
				{
					fmi2_boolean_t bool_value = static_cast<fmi2_boolean_t>(value != 0.0);
					fmi2_import_set_boolean(FmuModel, &vr, 1, &bool_value);
				}
				else
				{
					UE_LOG(LogSoda, Error, TEXT("UFMIAdapterComponent: unsupported variable type for input %s"), *VariableName);
				}
				fmi2_import_free_variable_list(variable_list);
				return;
			}
		}


		fmi2_import_free_variable_list(variable_list);

	}




void UFMIAdapterComponent::SpecifyFmuModel()
{
	IDesktopPlatform* DesktopPlatform = FDesktopPlatformModule::Get();
	if (!DesktopPlatform)
	{
		UE_LOG(LogSoda, Error, TEXT("UVehicleInputOpenLoopComponent::LoadCustomInput() Can't get the IDesktopPlatform ref"));
		return;
	}

	const FString FileTypes = TEXT("(*.fmu)|*.fmu");

	TArray<FString> OpenFilenames;
	int32 FilterIndex = -1;
	if (!DesktopPlatform->OpenFileDialog(nullptr, TEXT("Import FMU model"), TEXT(""), TEXT(""), FileTypes, EFileDialogFlags::None, OpenFilenames, FilterIndex) || OpenFilenames.Num() <= 0)
	{
		soda::ShowNotification(ENotificationLevel::Error, 5.0, TEXT("Load Error: can't open the FMU model"));
		return;
	}

	FmuPath = OpenFilenames[0];
}



#else


void UFMIAdapterComponent::SpecifyFmuModel()
{	
	UE_LOG(LogSoda, Error, TEXT("FMI adapter is not supported on Linux"));
}


bool UFMIAdapterComponent::OnActivateVehicleComponent()
{
	if (!Super::OnActivateVehicleComponent())
	{
		return false;
	}	

	SetHealth(EVehicleComponentHealth::Error, TEXT("FMI adapter is not supported on Linux"));

	return true;
}


void UFMIAdapterComponent::PostPhysicSimulationDeferred(float DeltaTime, const FPhysBodyKinematic& VehicleKinematic, const TTimestamp& Timestamp)
{
	Super::PostPhysicSimulationDeferred(DeltaTime, VehicleKinematic, Timestamp);

	if (GetHealth() != EVehicleComponentHealth::Ok)
	{
		return;
	}



}

void UFMIAdapterComponent::OnDeactivateVehicleComponent()
{
	Super::OnDeactivateVehicleComponent();
	
}



void UFMIAdapterComponent::DrawDebug(UCanvas* Canvas, float& YL, float& YPos)
{
	Super::DrawDebug(Canvas, YL, YPos);

}

bool UFMIAdapterComponent::ConfigureSignal(const FROS2TopicSetup& Setup)
{
	return false;
}




#endif