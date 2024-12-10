// Copyright 2023 SODA.AUTO UK LTD. All Rights Reserved.

#include "Soda/ROS2/ROS2RadarPointCloud2Publisher.h"
#include "Soda/VehicleComponents/Sensors/Base/RadarSensor.h"
#include "Soda/SodaApp.h"

UROS2RadarPointCloud2Publisher::UROS2RadarPointCloud2Publisher(const FObjectInitializer& ObjectInitializer)
    : Super(ObjectInitializer)
{
}

bool UROS2RadarPointCloud2Publisher::Advertise(UVehicleBaseComponent* Parent)
{
    Shutdown();

    URadarSensor* RadarSensor = Cast<URadarSensor>(Parent);
    if (RadarSensor && RadarSensor->GetRadarMode() == ERadarMode::ObjectMode)
    {
        UE_LOG(LogTemp, Warning, TEXT("UROS2RadarPointCloud2Publisher: Radar is in ObjectMode. Publisher will not be created."));
        return false;
    }

    Msg.fields.resize(3);
    Msg.fields[0].name = "x";
    Msg.fields[0].offset = 0;
    Msg.fields[0].datatype = sensor_msgs::msg::PointField::FLOAT32;
    Msg.fields[0].count = 1;
    Msg.fields[1].name = "y";
    Msg.fields[1].offset = 4;
    Msg.fields[1].datatype = sensor_msgs::msg::PointField::FLOAT32;
    Msg.fields[1].count = 1;
    Msg.fields[2].name = "z";
    Msg.fields[2].offset = 8;
    Msg.fields[2].datatype = sensor_msgs::msg::PointField::FLOAT32;
    Msg.fields[2].count = 1;

    Msg.is_bigendian = false;
    Msg.point_step = 12;
    Msg.is_dense = false;

    Publisher = FSodaROS2Module::Get().CreatePublisher<sensor_msgs::msg::PointCloud2>(NodeNamespace, Topic, QoS);
    return Publisher.IsValid();
}

void UROS2RadarPointCloud2Publisher::Shutdown()
{
    Publisher.Reset();
    Msg.data.clear();
}

bool UROS2RadarPointCloud2Publisher::Publish(float DeltaTime, const FSensorDataHeader& Header, const TArray<FRadarParams>& Params, const FRadarClusters& Clusters, const FRadarObjects& Objects)
{
    if (IsOk())
    {
        const auto Timestamp = soda::RawTimestamp<std::chrono::nanoseconds>(Header.Timestamp);
        Msg.header.stamp = ros2_ue_wrapper::FromNanoseconds(Timestamp);
        Msg.header.frame_id = TCHAR_TO_UTF8(*FrameID);

        int32 TotalPoints = 0;
        for (const FRadarCluster& Cluster : Clusters.Clusters)
        {
            TotalPoints += Cluster.Hits.Num();
        }

        Msg.height = 1;
        Msg.width = TotalPoints;

        Msg.row_step = Msg.point_step * Msg.width;
        Msg.data.resize(Msg.row_step);

        int32 PointIndex = 0;
        for (const FRadarCluster& Cluster : Clusters.Clusters)
        {
            for (const FHitResult* Hit : Cluster.Hits)
            {

                float* Dst = (float*)&Msg.data[PointIndex * Msg.point_step];
                Dst[0] = Cluster.LocalHitPosition.X / 100.0f;  
                Dst[1] = -Cluster.LocalHitPosition.Y / 100.0f;
                Dst[2] = Cluster.LocalHitPosition.Z / 100.0f;
                ++PointIndex;
            }
        }

        Publisher->Publish(Msg);
        return true;
    }
    return false;
}

bool UROS2RadarPointCloud2Publisher::IsOk() const
{
    return Publisher.IsValid() && ros2_ue_wrapper::Ok();
}

FString UROS2RadarPointCloud2Publisher::GetRemark() const
{
    return Topic;
}
