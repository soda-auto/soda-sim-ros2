// Copyright 2023 SODA.AUTO UK LTD. All Rights Reserved.

#include "Soda/ROS2/ROS2RadarPointCloud2Publisher.h"
#include "Soda/SodaApp.h"

UROS2RadarPointCloud2Publisher::UROS2RadarPointCloud2Publisher(const FObjectInitializer& ObjectInitializer)
    : Super(ObjectInitializer)
{
}

bool UROS2RadarPointCloud2Publisher::Advertise(UVehicleBaseComponent* Parent)
{
    Shutdown();

    URadarSensor* RadarSensor = Cast<URadarSensor>(Parent);
    check(RadarSensor);

    Mode = RadarSensor->GetRadarMode();

    if (RadarSensor->GetRadarMode() == ERadarMode::ObjectMode)
    {
        Msg.fields.resize(10);
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
        Msg.fields[3].name = "depth";
        Msg.fields[3].offset = 12;
        Msg.fields[3].datatype = sensor_msgs::msg::PointField::FLOAT32;
        Msg.fields[3].count = 1;
        Msg.fields[4].name = "width";
        Msg.fields[4].offset = 16;
        Msg.fields[4].datatype = sensor_msgs::msg::PointField::FLOAT32;
        Msg.fields[4].count = 1;
        Msg.fields[5].name = "height";
        Msg.fields[5].offset = 20;
        Msg.fields[5].datatype = sensor_msgs::msg::PointField::FLOAT32;
        Msg.fields[5].count = 1;
        Msg.fields[6].name = "lon";
        Msg.fields[6].offset = 24;
        Msg.fields[6].datatype = sensor_msgs::msg::PointField::FLOAT32;
        Msg.fields[6].count = 1;
        Msg.fields[7].name = "lat";
        Msg.fields[7].offset = 28;
        Msg.fields[7].datatype = sensor_msgs::msg::PointField::FLOAT32;
        Msg.fields[7].count = 1;
        Msg.fields[8].name = "rsc";
        Msg.fields[8].offset = 32;
        Msg.fields[8].datatype = sensor_msgs::msg::PointField::FLOAT32;
        Msg.fields[8].count = 1;
        Msg.fields[9].name = "id";
        Msg.fields[9].offset = 36;
        Msg.fields[9].datatype = sensor_msgs::msg::PointField::UINT32;
        Msg.fields[9].count = 1;
        Msg.point_step = 10 * 4;
    }
    else
    {
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
        Msg.point_step = 3 * 4;
    }

    Msg.is_bigendian = false;
    Msg.is_dense = false;

    FormatedTopic = TopicSetup.GetFormatedTopic(Parent->GetName());
    Publisher = ros2::TPublisher<sensor_msgs::msg::PointCloud2>::Create(NodeName, FormatedTopic, QoS);

    return Publisher.IsValid();
}

void UROS2RadarPointCloud2Publisher::Shutdown()
{
    Publisher.Reset();
    Msg.data.clear();
}

bool UROS2RadarPointCloud2Publisher::Publish(float DeltaTime, const FSensorDataHeader& Header, const TArray<FRadarParams>& Params, const FRadarClusters& Clusters, const FRadarObjects& Objects)
{
    if (!IsOk())
    {
        return false;
    }

    const auto Timestamp = soda::RawTimestamp<std::chrono::nanoseconds>(Header.Timestamp);
    Msg.header.stamp = ros2_ue_wrapper::FromNanoseconds(Timestamp);
    Msg.header.frame_id = TCHAR_TO_UTF8(*FrameID);

    if (Mode == ERadarMode::ObjectMode)
    {
        Msg.height = 1;
        Msg.width = Objects.Objects.Num();
        Msg.row_step = Msg.point_step * Msg.width;
        Msg.data.resize(Msg.row_step);

        int32 PointIndex = 0;
        for (const auto& [Key, Object] : Objects.Objects)
        {
            uint8* ItemPtr = (uint8*)&Msg.data[PointIndex * Msg.point_step];
            *(float*)&ItemPtr[0] = Object.GetObjectPoint().X / 100.0;
            *(float*)&ItemPtr[4] = -Object.GetObjectPoint().Y / 100.0;
            *(float*)&ItemPtr[8] = Object.GetObjectPoint().Z / 100.0;
            *(float*)&ItemPtr[12] = FMath::Max(0.01f, Object.LocalBounds.GetSize().X / 100); // Depth
            *(float*)&ItemPtr[16] = FMath::Max(0.01f, Object.LocalBounds.GetSize().Y / 100); // Width
            *(float*)&ItemPtr[20] = FMath::Max(0.01f, Object.LocalBounds.GetSize().Z / 100); // Height
            *(float*)&ItemPtr[24] = Object.Lon;
            *(float*)&ItemPtr[28] = -Object.Lat;
            *(float*)&ItemPtr[32] = -Object.RCS;
            *(std::uint32_t*)&ItemPtr[36] = Object.RadarObjectId;
            ++PointIndex;
        }
    }
    else
    {
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
    }

    Publisher->Publish(Msg);
    return true;

}

bool UROS2RadarPointCloud2Publisher::IsOk() const
{
    return Publisher.IsValid() && ros2_ue_wrapper::Ok();
}

FString UROS2RadarPointCloud2Publisher::GetRemark() const
{
    return Publisher.IsValid() ? FormatedTopic : TopicSetup.Topic;
}
