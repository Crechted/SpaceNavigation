// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "Core/Delegates.h"
#include "GameFramework/Actor.h"
#include "Core/SimTypes.h"
#include "CelestialBody.generated.h"

class ASimulationManager;

/**
 * Небесное тело: звезда или планета.
 *
 * Физические величины хранятся в физических единицах (км, кг, км/с, с),
 * а ActorLocation используется только для отображения.
 */
UCLASS(Blueprintable)
class SPACENAVIGATION_API ACelestialBody : public AActor
{
    GENERATED_BODY()

#pragma region DelegateProperties

public:
    UPROPERTY(BlueprintAssignable, BlueprintCallable, Category = "Delegates")
    FSimpleDynamicDelegate OnBodyInitialized;

#pragma endregion

#pragma region DataProperties

public:
    /** Параметры тела: масса, радиусы, орбитальные элементы, сфера влияния. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Body")
    FCelestialBodyData BodyData;

#pragma endregion

#pragma region Components

protected:
    UPROPERTY(EditDefaultsOnly, BlueprintReadWrite, Category = "Components")
    class UStaticMeshComponent* StaticMeshComponent;

#pragma endregion

#pragma region RuntimeProperties

public:
    /** Позиция тела относительно центральной звезды, км (в системе координат звезды). */
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "State", meta = (Units = "km"))
    FVector PositionKm = FVector::ZeroVector;

    /** Скорость тела относительно центральной звезды, км/с. */
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "State", meta = (Units = "km"))
    FVector VelocityKmS = FVector::ZeroVector;

    /** Масштаб радиуса при отрисовке: реальный радиус слишком мал, чтобы быть видимым. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Visual", meta = (ClampMin = "1.0"))
    double VisualRadiusScale = 400.0;

    /** Цвет отрисовки в debug-визуализации. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Visual")
    FColor DebugColor = FColor::Cyan;

protected:
    UPROPERTY(BlueprintReadOnly, Category = "References")
    const ASimulationManager* SimulationManager;

private:
    /** Наклон плоскости орбиты вокруг оси X. */
    UPROPERTY()
    FQuat OrbitPlaneRotation = FQuat::Identity;

#pragma endregion

#pragma region Functions

public:
    ACelestialBody();

    /**
     * Задать круговое орбитальное состояние вокруг звезды.
     */
    UFUNCTION(BlueprintCallable, Category = "Body")
    virtual void Initialize(const FCelestialBodyData& InBodyData, const ASimulationManager* InSimulationManager);

    /** Обновить положение тела по текущему времени симуляции, с. */
    UFUNCTION(BlueprintCallable, Category = "Body")
    virtual void UpdatePosition(double SimulationTimeSeconds);

    /** Сфера влияния тела, км. */
    UFUNCTION(BlueprintPure, Category = "Body")
    virtual double GetSOIKm() const;

    /** Радиус орбиты тела вокруг звезды, км. */
    UFUNCTION(BlueprintPure, Category = "Body")
    double GetOrbitRadiusKm() const;

    /** Гравитационное ускорение в точке на расстоянии DistanceKm от центра тела, км/с^2. */
    UFUNCTION(BlueprintPure, Category = "Body")
    virtual double GetGravitationalAcceleration(double DistanceKm) const;

    /** Угловая скорость движения по орбите, рад/с. */
    UFUNCTION(BlueprintPure, Category = "Body")
    double GetAngularVelocity() const;

    /**
     * Единичная нормаль плоскости орбиты: ось вращения тела вокруг центральной
     * звезды. Для тела без наклона совпадает с +Z мировой системы координат.
     */
    UFUNCTION(BlueprintPure, Category = "Body")
    FVector GetOrbitPlaneNormal() const;

    /**
     * Ортонормированный базис плоскости орбиты: OutAxisU и OutAxisV лежат в
     * плоскости орбиты, OutAxisU x OutAxisV = GetOrbitPlaneNormal().
     * Нужен для отрисовки окружности орбиты ровно в той плоскости,
     * в которой движется тело.
     */
    UFUNCTION(BlueprintPure, Category = "Body")
    void GetOrbitPlaneAxes(FVector& OutAxisU, FVector& OutAxisV) const;

    /** Пересчитать положение Actor'а из физических координат с учётом масштаба мира. */
    UFUNCTION(BlueprintCallable, Category = "Visual")
    void SyncTransform(double KmPerWorldUnit);

    /** Радиус тела, увеличенный в VisualRadiusScale раз для отображения, км. */
    UFUNCTION(BlueprintPure, Category = "Visual")
    double GetVisualRadiusKm() const;

    /** Радиус сферы влияния в тех же увеличенных единицах, км. */
    UFUNCTION(BlueprintPure, Category = "Visual")
    double GetVisualSOIKm() const;

protected:
    virtual void BeginPlay() override;
    virtual void Tick(float DeltaTime) override;

#pragma endregion
};

/**
 * Небесное тело: звезда.
 *
 * Пока нужен только для определения, что тело центральное в системе
 */
UCLASS(Blueprintable)
class SPACENAVIGATION_API AStarBody : public ACelestialBody
{
    GENERATED_BODY()

public:
    AStarBody();
};