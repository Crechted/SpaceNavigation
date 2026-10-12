// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "UObject/Interface.h"
#include "Importer.generated.h"

#pragma region ImportTypes

/**
 * Типы парамеров.
 */
UENUM(BlueprintType)
enum class EParamTypes : uint8
{
    Int32 UMETA(DisplayName = "int32"),
    Double UMETA(DisplayName = "double"),
    Bool UMETA(DisplayName = "bool"),
};

#pragma endregion

#pragma region ParameterData

/**
 * Абстрактный параметр
 * Нужен для гибкой работы с интерфейсом
 */
USTRUCT(BlueprintType)
struct FParameter
{
    GENERATED_BODY()

    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    FName LocalName;

    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    EParamTypes ParamType;

    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    FString Value;

    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    FString Units;
};

/**
 * Набор параметров.
 */
USTRUCT(BlueprintType)
struct FParameters
{
    GENERATED_BODY()

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Import")
    TArray<FParameter> Parameters;
};

#pragma endregion

#pragma region ImportableInterface

UINTERFACE(MinimalAPI, Blueprintable)
class UImportable : public UInterface
{
    GENERATED_BODY()
};

class SPACENAVIGATION_API IImportable
{
    GENERATED_BODY()

public:
    UFUNCTION(BlueprintCallable, BlueprintNativeEvent, Category = "Import")
    FParameters ExtractParameters();

    UFUNCTION(BlueprintCallable, BlueprintNativeEvent, Category = "Import")
    bool ApplyParameters(const FParameters& InParameters);
};

#pragma endregion

#pragma region ImporterClass

UCLASS(Blueprintable)
class SPACENAVIGATION_API UImporter : public UObject
{
    GENERATED_BODY()

public:
};

#pragma endregion
