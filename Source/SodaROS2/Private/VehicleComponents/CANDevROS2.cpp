// Copyright 2023 SODA.AUTO UK LTD. All Rights Reserved.

#include "Soda/VehicleComponents/CANDevROS2.h"
#include "Soda/UnrealSoda.h"
#include "Soda/SodaApp.h"

static inline bool Compare(const rmw_gid_t& gidA, const rmw_gid_t& gidB)
{
	return std::memcmp((char*)gidA.data, (char*)gidB.data, RMW_GID_STORAGE_SIZE) == 0;
}


UCANDevROS2::UCANDevROS2(const FObjectInitializer& ObjectInitializer)
	: Super(ObjectInitializer)
{
	GUI.ComponentNameOverride = TEXT("CAN Dev ROS2");
	GUI.bIsPresentInAddMenu = true;

	PubQoS.Depth = 50;
	PubQoS.HistoryPolicy = EHistoryPolicy::KeepLast;
	PubQoS.DurabilityPolicy = EDurabilityPolicy::Volatile;
	PubQoS.ReliabilityPolicy = EReliabilityPolicy::BestEffort;

	SubQoS.Depth = 50;
	SubQoS.HistoryPolicy = EHistoryPolicy::KeepLast;
	SubQoS.DurabilityPolicy = EDurabilityPolicy::Volatile;
	SubQoS.ReliabilityPolicy = EReliabilityPolicy::BestEffort;
}

bool UCANDevROS2::OnActivateVehicleComponent()
{
	if (!Super::OnActivateVehicleComponent())
	{
		return false;
	}

	FormatedTopic = TopicSetup.GetFormatedTopic(GetName());
	Node = FSodaROS2Module::Get().RegistreNode(FName(*NodeName));
	Publisher = ros2::TPublisher<soda_msgs::msg::CANFrame>::Create(Node.ToSharedRef(), FormatedTopic, PubQoS);
	Subscription = ros2::TSubscription<soda_msgs::msg::CANFrame>::Create(Node.ToSharedRef(), FormatedTopic, SubQoS,
		[this](const soda_msgs::msg::CANFrame::SharedPtr Msg, const rmw_message_info_t& Info)
		{
			if (!Compare(Info.publisher_gid, Publisher->GetGid()))
			{
				dbc::FCanFrame Frame;
				Frame.ID = Msg->can_id;
				Frame.Length = (uint8)Msg->data.size();
				::memcpy(Frame.Data, &Msg->data[0], Frame.Length);
				ProcessRecvMessage(SodaApp.GetRealtimeTimestamp(), Frame);
			}
		});

	return true;
}

void UCANDevROS2::OnDeactivateVehicleComponent()
{
	Super::OnDeactivateVehicleComponent();
	Publisher.Reset();
	Subscription.Reset();
	Node.Reset();
}

int UCANDevROS2::SendFrame(const dbc::FCanFrame & CanFrame)
{
	Super::SendFrame(CanFrame);

	if (GetHealth() != EVehicleComponentHealth::Ok)
	{
		return -1;
	}

	if(CanFrame.Length > 64)
	{
		UE_LOG(LogSoda, Error, TEXT("UCANDevROS2::SendFrame(%s); (Length == %i) > 64, CAN_ID == %i"), *FormatedTopic, CanFrame.Length, CanFrame.ID);
		return -1;
	}


	soda_msgs::msg::CANFrame Msg;
	Msg.stamp = Node->getRosTime();
	Msg.can_id = CanFrame.ID;
	Msg.data.resize(CanFrame.Length);
	Msg.flags = 0;
	if (Msg.can_id > 0xFF)
	{
		Msg.flags |= soda_msgs::msg::CANFrame::FLAG_EXTENDED;
	}
	::memcpy(&Msg.data[0], CanFrame.Data, CanFrame.Length);
	Publisher->Publish(Msg);

	//TransmitErrorsCounter = 0;
	
	return 0;
	
}

