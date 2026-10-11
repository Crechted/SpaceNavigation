#pragma once

#include "CoreMinimal.h"
#include "Delegates.generated.h"

UDELEGATE()
DECLARE_DYNAMIC_MULTICAST_DELEGATE(FSimpleDynamicDelegate);

DECLARE_DYNAMIC_MULTICAST_DELEGATE_OneParam(FStringDynamicDelegate, const FString&, StringText);

DECLARE_DYNAMIC_MULTICAST_DELEGATE_OneParam(FBoolDynamicDelegate, bool, bValue);