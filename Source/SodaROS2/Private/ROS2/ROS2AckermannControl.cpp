// Copyright 2023 SODA.AUTO UK LTD. All Rights Reserved.

#include "Soda/ROS2/ROS2AckermannControl.h"
#include "Soda/VehicleComponents/Drivers/GenericVehicleDriverComponent.h"
#include "Soda/SodaApp.h"
#include "Engine/Engine.h"
#include "Engine/Canvas.h"

bool UROS2AckermannControl::StartListen(UVehicleBaseComponent* Parent)
{
	StopListen();

	Subscription = FSodaROS2Module::Get().CreateSubscription<ackermann_msgs::msg::AckermannDriveStamped>(
		NodeNamespace, Topic, QoS,
		[this](const std::shared_ptr<ackermann_msgs::msg::AckermannDriveStamped> InMsg) {
			Msg = *InMsg;
			RecvTimestamp = soda::Now();
		});
	return Subscription.IsValid();
}

void UROS2AckermannControl::StopListen()
{
	Subscription.Reset();
}

bool UROS2AckermannControl::IsOk() const
{
	return Subscription.IsValid() && ros2_ue_wrapper::Ok();
}

bool UROS2AckermannControl::GetControl(soda::FGenericWheeledVehiclControl& Control) const
{
	Control.SteerReq.ByAngle = -Msg.drive.steering_angle;
	Control.DriveEffortReq.ByAcc = Msg.drive.acceleration * 100;
	Control.TargetSpeedReq = Msg.drive.speed * 100;
	Control.SteeringAngleVelocity = Msg.drive.steering_angle_velocity;
	Control.bGearIsSet = false;
	Control.bTargetSpeedIsSet = !FMath::IsNearlyZero(Msg.drive.speed);
	Control.bSteeringAngleVelocitySet = !FMath::IsNearlyZero(Msg.drive.steering_angle_velocity);
	Control.SteerReqMode = soda::FGenericWheeledVehiclControl::ESteerReqMode::ByAngle;
	Control.DriveEffortReqMode = soda::FGenericWheeledVehiclControl::EDriveEffortReqMode::ByAcc;
	Control.Timestamp = RecvTimestamp;

	return true;
}

FString UROS2AckermannControl::GetRemark() const
{
	return Topic;
}

void UROS2AckermannControl::DrawDebug(UCanvas* Canvas, float& YL, float& YPos)
{
	Super::DrawDebug(Canvas, YL, YPos);

	UFont* RenderFont = GEngine->GetSmallFont();

	Canvas->SetDrawColor(FColor::White);
	YPos += Canvas->DrawText(RenderFont, FString::Printf(TEXT("steering_angle: %.2f"), Msg.drive.steering_angle), 16, YPos);
	YPos += Canvas->DrawText(RenderFont, FString::Printf(TEXT("steering_angle_velocity: %.2f"), Msg.drive.steering_angle_velocity), 16, YPos);
	YPos += Canvas->DrawText(RenderFont, FString::Printf(TEXT("speed: %.2f"), Msg.drive.speed), 16, YPos);
	YPos += Canvas->DrawText(RenderFont, FString::Printf(TEXT("acceleration: %.2f"), Msg.drive.acceleration), 16, YPos);

	const uint64 dt = std::chrono::duration_cast<std::chrono::milliseconds>(SodaApp.GetRealtimeTimestamp() - RecvTimestamp).count();
	if (dt > 10000)
	{
		YPos += Canvas->DrawText(RenderFont, FString::Printf(TEXT("dtime: timeout")), 16, YPos);
	}
	else
	{
		YPos += Canvas->DrawText(RenderFont, FString::Printf(TEXT("dtime: %dms"), dt), 16, YPos);
	}
}