// Copyright 2023 SODA.AUTO UK LTD. All Rights Reserved.

#include "Soda/VehicleComponents/IODevROS2.h"
#include "Engine/Canvas.h"
#include "Engine/Engine.h"

static inline bool Compare(const rmw_gid_t& gidA, const rmw_gid_t& gidB)
{
	return std::memcmp((char*)gidA.data, (char*)gidB.data, RMW_GID_STORAGE_SIZE) == 0;
}

UIOPinROS2::UIOPinROS2(const FObjectInitializer& ObjectInitializer)
	: Super(ObjectInitializer)
{
}

void UIOPinROS2::ConstruictPin(
		const UIOBusROS2Component* IOBusROS2Component,
		const FIOPinSetup& Setup)
{
	Node = FSodaROS2Module::Get().RegistreNode(*IOBusROS2Component->NodeName);
	LocalSetup = Setup;

	FormatedSourceTopic = IOBusROS2Component->SourceTopic;
	FormatedFeedbackTopic = IOBusROS2Component->FeedbackTopic;
	FormatedDescTopic = IOBusROS2Component->DescTopic;

	FormatedSourceTopic = FormatedSourceTopic.Replace(TEXT("{signal_name}"), *Setup.WireKey.ToString(), ESearchCase::CaseSensitive);
	FormatedFeedbackTopic = FormatedFeedbackTopic.Replace(TEXT("{signal_name}"), *Setup.WireKey.ToString(), ESearchCase::CaseSensitive);
	FormatedDescTopic = FormatedDescTopic.Replace(TEXT("{signal_name}"), *Setup.WireKey.ToString(), ESearchCase::CaseSensitive);

	if (IOBusROS2Component->bOverrideNamespace)
	{
		FormatedSourceTopic = FormatedSourceTopic.Replace(TEXT("{namespace}"), *IOBusROS2Component->TopicNamespace, ESearchCase::CaseSensitive);
		FormatedFeedbackTopic = FormatedFeedbackTopic.Replace(TEXT("{namespace}"), *IOBusROS2Component->TopicNamespace, ESearchCase::CaseSensitive);
		FormatedDescTopic = FormatedDescTopic.Replace(TEXT("{namespace}"), *IOBusROS2Component->TopicNamespace, ESearchCase::CaseSensitive);
	}

	SourcePub = ros2::TPublisher<soda_msgs::msg::IOPinSrc>::Create(Node.ToSharedRef(), FormatedSourceTopic, IOBusROS2Component->PubSourceQoS);
	FeedbackPub = ros2::TPublisher<soda_msgs::msg::IOPinFeedback>::Create(Node.ToSharedRef(), FormatedFeedbackTopic, IOBusROS2Component->PubFeedbackQoS);
	DescPub = ros2::TPublisher<soda_msgs::msg::IOPinDesc>::Create(Node.ToSharedRef(), FormatedDescTopic, IOBusROS2Component->PubDescQoS);

	SourceSub = ros2::TSubscription<soda_msgs::msg::IOPinSrc>::Create(Node.ToSharedRef(), FormatedSourceTopic, IOBusROS2Component->SubSourceQoS,
		[this](const soda_msgs::msg::IOPinSrc::SharedPtr Msg, const rmw_message_info_t& Info)
		{
			if(!Compare(Info.publisher_gid, SourcePub->GetGid()))
			{
				UE::TScopeLock<UE::FSpinLock> ScopeLock(SpinLock);
				InputSourceValue.Duty = Msg->duty;
				InputSourceValue.Frequency = Msg->frequency;
				InputSourceValue.LogicalVal = Msg->logical_value;
				InputSourceValue.Mode = static_cast<EIOPinMode>(Msg->pin_mode);
				InputSourceValue.Voltage = Msg->measured_voltage;
				//Msg->stamp;
				SourceValueTs = soda::Now();
			}
		});

	FeedbackSub = ros2::TSubscription<soda_msgs::msg::IOPinFeedback>::Create(Node.ToSharedRef(), FormatedFeedbackTopic, IOBusROS2Component->SubSourceQoS,
		[this](const soda_msgs::msg::IOPinFeedback::SharedPtr Msg, const rmw_message_info_t& Info)
		{
			if (!Compare(Info.publisher_gid, FeedbackPub->GetGid()))
			{
				UE::TScopeLock<UE::FSpinLock> ScopeLock(SpinLock);
				InputFeedbackValue.bIsMeasuredCurrentValid = Msg->current_is_valid;
				InputFeedbackValue.bIsMeasuredVoltageValid = Msg->voltage_is_valid;
				InputFeedbackValue.MeasuredCurrent = Msg->current;
				InputFeedbackValue.MeasuredVoltage = Msg->voltage;
				//Msg->stamp;
				FeedbackValueTs = soda::Now();
			}
		});

	DescSub = ros2::TSubscription<soda_msgs::msg::IOPinDesc>::Create(Node.ToSharedRef(), FormatedDescTopic, IOBusROS2Component->SubSourceQoS,
		[this](const soda_msgs::msg::IOPinDesc::SharedPtr Msg, const rmw_message_info_t& Info)
		{
			if (!Compare(Info.publisher_gid, DescPub->GetGid()))
			{
				UE::TScopeLock<UE::FSpinLock> ScopeLock(SpinLock);
				RemoteSetup.MaxCurrent = Msg->max_current;
				RemoteSetup.MaxVoltage = 0;
				RemoteSetup.MinVoltage = 0;
				RemoteSetup.PinFunction = UTF8_TO_TCHAR(Msg->pin_function.c_str());
				RemoteSetup.PinName = UTF8_TO_TCHAR(Msg->pin_name.c_str());
				RemoteSetup.DeviceName = UTF8_TO_TCHAR(Msg->device_name.c_str());
				//Msg->pin_index;
				//Msg->stamp;
				RemoteSetupTs = soda::Now();
			}
		});

	soda_msgs::msg::IOPinDesc DescMsg;
	DescMsg.device_name = TCHAR_TO_UTF8(*Setup.DeviceName.ToString());
	DescMsg.max_current = Setup.MaxCurrent;
	DescMsg.pin_function = TCHAR_TO_UTF8(*Setup.PinFunction.ToString());
	DescMsg.pin_index = (uint32)Setup.PinIndex;
	DescMsg.pin_name = TCHAR_TO_UTF8(*Setup.PinName.ToString());
	DescMsg.stamp = Node->getRosTime();

	DescPub->Publish(DescMsg);
}

void UIOPinROS2::BeginDestroy()
{
	Super::BeginDestroy();

	SourcePub.Reset();
	FeedbackPub.Reset();
	DescPub.Reset();

	SourceSub.Reset();
	FeedbackSub.Reset();
	DescSub.Reset();

	Node.Reset();

	if (UIOBusROS2Component* IOBus = Cast<UIOBusROS2Component>(GetOuter()))
	{
		IOBus->ClearUnusedPins();
	}

	FSodaROS2Module::Get().CheckUnregisterNode();
}

void UIOPinROS2::PublishSource(FIOPinSourceValue& Value)
{
	soda_msgs::msg::IOPinSrc Msg;
	Msg.duty = Value.Duty;
	Msg.frequency = Value.Frequency;
	Msg.logical_value = Value.LogicalVal;
	Msg.measured_current = 0;
	Msg.measured_voltage = 0;
	Msg.out_voltage = Value.Voltage;
	Msg.pin_index = (uint32)LocalSetup.PinIndex;
	Msg.pin_mode = (uint8)Value.Mode;
	Msg.stamp = Node->getRosTime();
	SourcePub->Publish(Msg);
	LastPublishSourceValue = Value;
}

void UIOPinROS2::PublishFeedback(FIOPinFeedbackValue& Value)
{
	soda_msgs::msg::IOPinFeedback Msg;
	Msg.pin_index = LocalSetup.PinIndex;
	Msg.stamp = Node->getRosTime();
	Msg.current = Value.MeasuredCurrent;
	Msg.current_is_valid = Value.bIsMeasuredCurrentValid;
	Msg.voltage = Value.MeasuredVoltage;
	Msg.voltage_is_valid = Value.bIsMeasuredVoltageValid;
	FeedbackPub->Publish(Msg);
	LastPublishFeedbackValue = Value;
}

const FIOPinSourceValue& UIOPinROS2::GetInputSourceValue(FTimestamp* OutTimestamp) const
{
	UE::TScopeLock<UE::FSpinLock> ScopeLock(SpinLock);
	if (OutTimestamp) *OutTimestamp = SourceValueTs;
	return InputSourceValue;
}

const FIOPinFeedbackValue& UIOPinROS2::GetInputFeedbackValue(FTimestamp* OutTimestamp) const
{
	UE::TScopeLock<UE::FSpinLock> ScopeLock(SpinLock);
	if (OutTimestamp) *OutTimestamp = FeedbackValueTs;
	return InputFeedbackValue;
}

const FIOPinSetup& UIOPinROS2::GetLocalPinSetup() const
{
	return LocalSetup;
}

const FIOPinSetup& UIOPinROS2::GetRemotePinSetup(FTimestamp* OutTimestamp) const
{
	UE::TScopeLock<UE::FSpinLock> ScopeLock(SpinLock);
	if (OutTimestamp) *OutTimestamp = RemoteSetupTs;
	return RemoteSetup;
}

//---------------------------------------------------------------------------

UIOBusROS2Component::UIOBusROS2Component(const FObjectInitializer& ObjectInitializer)
	: Super(ObjectInitializer)
{
	GUI.bIsPresentInAddMenu = true;
	GUI.ComponentNameOverride = TEXT("IO Bus ROS2");

	PrimaryComponentTick.bCanEverTick = true;
	PrimaryComponentTick.TickGroup = TG_PostPhysics;

	PubSourceQoS.Depth = 1;
	PubSourceQoS.HistoryPolicy = EHistoryPolicy::KeepLast;
	PubSourceQoS.DurabilityPolicy = EDurabilityPolicy::TransientLocal;
	PubSourceQoS.ReliabilityPolicy = EReliabilityPolicy::BestEffort;

	PubFeedbackQoS.Depth = 1;
	PubFeedbackQoS.HistoryPolicy = EHistoryPolicy::KeepLast;
	PubFeedbackQoS.DurabilityPolicy = EDurabilityPolicy::TransientLocal;
	PubFeedbackQoS.ReliabilityPolicy = EReliabilityPolicy::BestEffort;

	PubDescQoS.Depth = 1;
	PubDescQoS.HistoryPolicy = EHistoryPolicy::KeepLast;
	PubDescQoS.DurabilityPolicy = EDurabilityPolicy::TransientLocal;
	PubDescQoS.ReliabilityPolicy = EReliabilityPolicy::Reliable;

	SubSourceQoS.Depth = 1;
	SubSourceQoS.HistoryPolicy = EHistoryPolicy::KeepLast;
	SubSourceQoS.DurabilityPolicy = EDurabilityPolicy::TransientLocal;
	SubSourceQoS.ReliabilityPolicy = EReliabilityPolicy::BestEffort;

	SubFeedbackQoS.Depth = 1;
	SubFeedbackQoS.HistoryPolicy = EHistoryPolicy::KeepLast;
	SubFeedbackQoS.DurabilityPolicy = EDurabilityPolicy::TransientLocal;
	SubFeedbackQoS.ReliabilityPolicy = EReliabilityPolicy::BestEffort;

	SubDescQoS.Depth = 1;
	SubDescQoS.HistoryPolicy = EHistoryPolicy::KeepLast;
	SubDescQoS.DurabilityPolicy = EDurabilityPolicy::TransientLocal;
	SubDescQoS.ReliabilityPolicy = EReliabilityPolicy::Reliable;
}

UIOPin * UIOBusROS2Component::CreatePin(const FIOPinSetup& Setup)
{
	ClearUnusedPins();

	auto * Pin = NewObject<UIOPinROS2>(this);
	check(Pin);
	Pin->ConstruictPin(this, Setup);
	Pins.Add(Pin);
	return Pin;
}

void UIOBusROS2Component::ClearUnusedPins()
{
	for (auto It = Pins.CreateIterator(); It; ++It)
	{
		if(!It->IsValid())
		{
			It.RemoveCurrent();
		}
	}
}

void UIOBusROS2Component::TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction)
{
	Super::TickComponent(DeltaTime, TickType, ThisTickFunction);
	if (!IsTickOnCurrentFrame() || !HealthIsWorkable()) return;
}

bool UIOBusROS2Component::OnActivateVehicleComponent()
{
	if (!Super::OnActivateVehicleComponent())
	{
		return false;
	}

	return true;
}

void UIOBusROS2Component::OnDeactivateVehicleComponent()
{
	Super::OnDeactivateVehicleComponent();

	Pins.Reset();
	ClearUnusedPins();
}

void UIOBusROS2Component::DrawDebug(UCanvas* Canvas, float& YL, float& YPos)
{
	Super::DrawDebug(Canvas, YL, YPos);

	static const FString::ElementType* IOPinModeToStr[] =
	{
		TEXT("Undefined"),
		TEXT("Fixed"),
		TEXT("Analogue"),
		TEXT("PWM"),
		TEXT("Input")
	};

	if (Common.bDrawDebugCanvas)
	{
		UFont* RenderFont = GEngine->GetSmallFont();
		Canvas->SetDrawColor(FColor::White);

		for (auto& Pin : Pins)
		{
			if (Pin.IsValid())
			{
				YPos += Canvas->DrawText(RenderFont, FString::Printf(TEXT("Pin name: %s->%s [%s]"),
					*Pin->GetLocalPinSetup().PinName.ToString(),
					*Pin->GetLocalPinSetup().WireKey.ToString(),
					IOPinModeToStr[int(Pin->GetLastPublishSourceValue().Mode)]),
					16, YPos);
				YPos += Canvas->DrawText(RenderFont, FString::Printf(TEXT("   Value: [%i] [%.1fV] [%.1fHz %f.1f]"),
					Pin->GetLastPublishSourceValue().LogicalVal,
					Pin->GetLastPublishSourceValue().Voltage,
					Pin->GetLastPublishSourceValue().Frequency,
					Pin->GetLastPublishSourceValue().Duty),
					16, YPos);
				YPos += Canvas->DrawText(RenderFont, FString::Printf(TEXT("   Measured: %.1fA[%s] %.1fV[%s]"),
					Pin->GetLastPublishFeedbackValue().MeasuredCurrent,
					Pin->GetLastPublishFeedbackValue().bIsMeasuredCurrentValid ? TEXT("T") : TEXT("F"),
					Pin->GetLastPublishFeedbackValue().MeasuredVoltage,
					Pin->GetLastPublishFeedbackValue().bIsMeasuredVoltageValid ? TEXT("T") : TEXT("F")),
					16, YPos);
			}
		}
	}
}
