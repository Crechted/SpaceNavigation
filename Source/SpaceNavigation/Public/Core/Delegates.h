#pragma once

#include "CoreMinimal.h"
#include "Delegates.generated.h"

UDELEGATE()
DECLARE_DYNAMIC_MULTICAST_DELEGATE_OneParam(FStringDelegate, const FString&, StringText);

DECLARE_DYNAMIC_MULTICAST_DELEGATE(FSimpleDynamicDelegate);
