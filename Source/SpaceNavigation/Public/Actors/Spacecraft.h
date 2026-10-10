// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "Core/SimTypes.h"
#include "CelestialBody.h"
#include "Core/Importer.h"
#include "Spacecraft.generated.h"

/**
 * Космический аппарат.
 *
 * Состояние хранится относительно текущего центрального тела (patched conics):
 * RelativePositionKm и RelativeVelocityKmS — векторы в системе координат
 * тела, на которое указывает CurrentCentralBody.
 */
UCLASS(Blueprintable)
class SPACENAVIGATION_API ASpacecraft : public AActor, public IImportable
{
    GENERATED_BODY()

#pragma region DataProperties

public:
    /** Параметры аппарата: массы, тяга, удельный импульс, точка старта. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Spacecraft")
    FSpacecraftParams Params;

#pragma endregion

#pragma region Components

protected:
    UPROPERTY(EditDefaultsOnly, BlueprintReadWrite, Category = "Components")
    class UStaticMeshComponent* StaticMeshComponent;

#pragma endregion

#pragma region RuntimeProperties

public:
    /** Текущее центральное тело (patched conics). */
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "State")
    TObjectPtr<ACelestialBody> CurrentCentralBody = nullptr;

    /** Позиция относительно текущего центрального тела, км. */
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "State", meta = (Units = "km"))
    FVector RelativePositionKm = FVector::ZeroVector;

    /** Скорость относительно текущего центрального тела, км/с. */
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "State", meta = (Units = "km"))
    FVector RelativeVelocityKmS = FVector::ZeroVector;

    /** Позиция относительно звезды (абсолютная), км. */
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "State", meta = (Units = "km"))
    FVector AbsolutePositionKm = FVector::ZeroVector;

    /** Абсолютная скорость относительно звезды, км/с. */
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "State", meta = (Units = "km"))
    FVector AbsoluteVelocityKmS = FVector::ZeroVector;

    /** Остаток топлива, кг. */
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "State", meta = (Units = "kg"))
    double FuelMassKg = 0.0;

    /** Сухая масса плюс остаток топлива, кг. */
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "State", meta = (Units = "kg"))
    double CurrentMassKg = 0.0;

    /** Суммарный расход скорости, км/с. */
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "State", meta = (Units = "km"))
    double TotalDeltaVSpentKmS = 0.0;

    /** Суммарное израсходованное топливо, кг. */
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "State", meta = (Units = "kg"))
    double TotalFuelSpentKg = 0.0;

    /** Время полёта от старта, с. */
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "State", meta = (Units = "s"))
    double FlightTimeSeconds = 0.0;

    /** Признак работающего двигателя. */
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "State")
    bool bEngineRunning = false;

private:
    /** Направление тяги в относительной системе координат. */
    FVector ThrustDirectionLocal = FVector::ZeroVector;

#pragma endregion

#pragma region OverridedFunctions

public:
    ASpacecraft();

protected:
    virtual void BeginPlay() override;
    virtual void Tick(float DeltaTime) override;

    virtual TArray<FParameter> ExtractParameters_Implementation() override;
    virtual bool ApplyParameters_Implementation(const TArray<FParameter>& InParameters) override;
    
#pragma endregion

#pragma region ControlFunctions
public:

    /**
     * Инициализировать аппарат на стартовом теле.
     *
     * @param StartBody     тело, вокруг которого начинается движение
     * @param StarBody      центральная звезда системы
     */
    UFUNCTION(BlueprintCallable, Category = "Spacecraft")
    void InitializeSpacecraft(ACelestialBody* StartBody, ACelestialBody* StarBody);

    /**
     * Шаг интегрирования по времени.
     *
     * @param DeltaTimeSeconds          длительность шага, с
     * @param TargetVelocityDirectionKmS требуемое направление скорости (единичный вектор)
     */
    UFUNCTION(BlueprintCallable, Category = "Spacecraft")
    void IntegrateStep(double DeltaTimeSeconds);

    /**
     * Выполнить импульсный манёвр заданной величины.
     *
     * @param DeltaVKmS величина изменения скорости, км/с
     * @param Direction  направление изменения скорости
     * @return фактически достигнутый DeltaV, км/с
     */
    UFUNCTION(BlueprintCallable, Category = "Spacecraft")
    double ApplyImpulse(double DeltaVKmS, const FVector& Direction);

    /** Включить или выключить двигатель. */
    UFUNCTION(BlueprintCallable, Category = "Spacecraft")
    void SetEngineRunning(bool bRunning);

    /** Расчётное время работы двигателя для заданного DeltaV, с. */
    UFUNCTION(BlueprintPure, Category = "Spacecraft")
    double GetBurnDuration(double DeltaVKmS) const;

    /** Текущий запас скорости при полном баке, км/с. */
    UFUNCTION(BlueprintPure, Category = "Spacecraft")
    double GetRemainingDeltaV() const;

    /** Достаточно ли топлива для заданного DeltaV. */
    UFUNCTION(BlueprintPure, Category = "Spacecraft")
    bool HasEnoughFuel(double DeltaVKmS) const;

    /** Расстояние до центрального тела, км. */
    UFUNCTION(BlueprintPure, Category = "Spacecraft")
    double GetDistanceToCentralBody() const;

    /** Скорость убегания от текущего центрального тела на текущем расстоянии, км/с. */
    UFUNCTION(BlueprintPure, Category = "Spacecraft")
    double GetEscapeVelocity() const;

    /** Пересчитать абсолютные координаты от текущих относительных. */
    UFUNCTION(BlueprintCallable, Category = "Spacecraft")
    void UpdateAbsoluteState(ACelestialBody* StarBody);

    /** Сбросить состояние к начальным значениям. */
    UFUNCTION(BlueprintCallable, Category = "Spacecraft")
    void ResetState();

    /** Расстояние от звезды, км. */
    UFUNCTION(BlueprintPure, Category = "Spacecraft")
    double GetDistanceFromStar(ACelestialBody* StarBody) const;

#pragma endregion
};

