// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "Core/Delegates.h"
#include "Core/Importer.h"
#include "Core/SimTypes.h"
#include "ScenarioGenerator.generated.h"

/**
 * Детерминированный генератор планетной системы.
 *
 * Генерация полностью определяется FScenarioParams::RandomSeed:
 * при одинаковом зерне всегда получается идентичная система.
 */
UCLASS(Blueprintable)
class SPACENAVIGATION_API UScenarioGenerator : public UObject, public IImportable
{
    GENERATED_BODY()

#pragma region DelegateProperties

public:
    UPROPERTY(BlueprintAssignable, BlueprintCallable, Category = "Delegates")
    FSimpleDynamicDelegate OnGenerated;

#pragma endregion

#pragma region DataProperties

public:
    /** Параметры генерации. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Scenario")
    FScenarioParams Params;

    /** Данные центральной звезды. */
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Scenario")
    FCelestialBodyData StarData;

    /** Данные сгенерированных планет в порядке возрастания орбитального радиуса. */
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Scenario")
    TArray<FCelestialBodyData> PlanetData;

    /** Граница системы, км: либо TargetDistance, либо K * максимальный орбитальный радиус. */
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Scenario", meta = (Units = "km"))
    double SystemBoundary = 0.0;

#pragma endregion

#pragma region OverridedFunctions

public:
    UScenarioGenerator();

    virtual FParameters ExtractParameters_Implementation() override;
    virtual bool ApplyParameters_Implementation(const FParameters& InParameters) override;

#pragma endregion

#pragma region ControlFunctions

public:
    /**
     * Сгенерировать систему согласно Params.
     * Детерминированно: один и тот же seed даёт один и тот же результат.
     */
    UFUNCTION(BlueprintCallable, Category = "Scenario")
    void GenerateScenario();

    /** Очистить сгенерированные данные. */
    UFUNCTION(BlueprintCallable, Category = "Scenario")
    void ClearScenario();

    /** Полный список тел системы: звезда на индексе 0, далее планеты. */
    UFUNCTION(BlueprintCallable, Category = "Scenario")
    TArray<FCelestialBodyData> GetAllBodies() const;

    /** Получить тело по индексу. Возвращает nullptr при выходе за границы. */
    UFUNCTION(BlueprintCallable, Category = "Scenario")
    bool GetBodyByIndex(int32 BodyIndex, FCelestialBodyData& OutBody) const;

    /** Граница системы, км. */
    UFUNCTION(BlueprintCallable, Category = "Scenario")
    double GetSystemBoundary() const;

    /** Радиус сферы влияния тела с учётом порога малой силы, км. */
    UFUNCTION(BlueprintCallable, Category = "Scenario")
    double GetBodySOI(const FCelestialBodyData& Body) const;

    /** Опорный орбитальный радиус звезды для расчёта стартовой орбиты аппарата, км. */
    UFUNCTION(BlueprintCallable, Category = "Scenario")
    double GetBodyOrbitRadius(int32 BodyIndex) const;

    /** Опорная масса центрального тела для расчёта SOI, кг. */
    UFUNCTION(BlueprintCallable, Category = "Scenario")
    double GetCentralMass() const;
#pragma endregion
};