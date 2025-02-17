// Copyright 2023 SODA.AUTO UK LTD. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "UObject/Interface.h"
#include "IROS2FMU.generated.h"

struct FROS2TopicSetup;

UINTERFACE()
class SODAROS2_API UROS2FMU : public UInterface
{
	GENERATED_BODY()
};

/** This interface is used to allow reconfiguration of the FMU signals at OnPreActivateVehicleComponent() by external participants. */
class SODAROS2_API IROS2FMU
{
	GENERATED_BODY()

public:

	/** 
	 * Called in OnPreActivateVehicleComponent()
	 */
	virtual bool ConfigureSignal(const FROS2TopicSetup& TopicSetup) = 0;

};

