// Copyright Epic Games, Inc. All Rights Reserved.

#include "Actors/CelestialBody.h"

#include "Actors/SimulationManager.h"
#include "Components/SceneComponent.h"
#include "Core/OrbitMath.h"


ACelestialBody::ACelestialBody()
{
    PrimaryActorTick.bCanEverTick = true;
    PrimaryActorTick.bStartWithTickEnabled = false;

    RootComponent = CreateDefaultSubobject<USceneComponent>(TEXT("Root"));
    SetRootComponent(RootComponent);

    StaticMeshComponent = CreateDefaultSubobject<UStaticMeshComponent>(TEXT("StaticMesh"));
    StaticMeshComponent->SetupAttachment(RootComponent);
}

void ACelestialBody::BeginPlay()
{
    Super::BeginPlay();

    if (!StaticMeshComponent)
        return;

    FLinearColor Color(DebugColor);
    StaticMeshComponent->SetCustomPrimitiveDataVector4(0, FVector4(Color.R, Color.G, Color.B, Color.A));
}

void ACelestialBody::Tick(float DeltaTime)
{
    Super::Tick(DeltaTime);
}

void ACelestialBody::Initialize(const FCelestialBodyData& InBodyData, const ASimulationManager* InSimulationManager)
{
    BodyData = InBodyData;

    // Наклон задаёт поворот плоскости орбиты.
    OrbitPlaneRotation = FQuat(FVector::XAxisVector, BodyData.OrbitalInclination);

    SimulationManager = InSimulationManager;

    UpdatePosition(0.0);
}

void ACelestialBody::UpdatePosition(double SimulationTimeSeconds)
{
    if (BodyData.OrbitalRadius <= 0.0)
    {
        PositionKm = FVector::ZeroVector;
        VelocityKmS = FVector::ZeroVector;
        return;
    }

    // Равномерное движение по круговой орбите.
    const double AngularVelocity = GetAngularVelocity();
    const double Angle = BodyData.InitialPhase + AngularVelocity * SimulationTimeSeconds;

    const FVector LocalOrbitPosition(BodyData.OrbitalRadius * FMath::Cos(Angle), BodyData.OrbitalRadius * FMath::Sin(Angle), 0.0);
    const FVector LocalOrbitVelocity(
        -BodyData.OrbitalRadius * AngularVelocity * FMath::Sin(Angle),
        BodyData.OrbitalRadius * AngularVelocity * FMath::Cos(Angle),
        0.0);

    PositionKm = OrbitPlaneRotation.RotateVector(LocalOrbitPosition);
    VelocityKmS = OrbitPlaneRotation.RotateVector(LocalOrbitVelocity);

    SyncTransform(SimulationManager->KmPerWorldUnit);
}

double ACelestialBody::GetAngularVelocity() const
{
    if (BodyData.OrbitalRadius <= 0.0 || BodyData.OrbitalVelocity <= 0.0)
    {
        return 0.0;
    }

    return BodyData.OrbitalVelocity / BodyData.OrbitalRadius;
}

FVector ACelestialBody::GetOrbitPlaneNormal() const
{
    // Поворот плоскости задан кватернионом OrbitPlaneRotation:
    // нормаль исходной плоскости (+Z) поворачивается вместе с ней.
    return OrbitPlaneRotation.RotateVector(FVector::UpVector);
}

void ACelestialBody::GetOrbitPlaneAxes(FVector& OutAxisU, FVector& OutAxisV) const
{
    // Ось вращения плоскости — X, поэтому U остаётся мировым ForwardVector,
    // а V (исходная ось Y) наклоняется на угол OrbitalInclination.
    OutAxisU = OrbitPlaneRotation.RotateVector(FVector::ForwardVector);
    OutAxisV = OrbitPlaneRotation.RotateVector(FVector::RightVector);
}

double ACelestialBody::GetSOIKm() const
{
    return BodyData.SphereOfInfluence;
}

double ACelestialBody::GetOrbitRadiusKm() const
{
    return BodyData.OrbitalRadius;
}

double ACelestialBody::GetGravitationalAcceleration(double DistanceKm) const
{
    return UOrbitMath::GravitationalAcceleration(BodyData.Mass, DistanceKm);
}

double ACelestialBody::GetVisualRadiusKm() const
{
    return BodyData.Radius * VisualRadiusScale;
}

double ACelestialBody::GetVisualSOIKm() const
{
    return GetSOIKm() * VisualRadiusScale;
}

void ACelestialBody::SyncTransform(double KmPerWorldUnit)
{
    if (KmPerWorldUnit <= 0.0)
    {
        return;
    }

    // Физические координаты переводим в мировые единицы Unreal (см).
    SetActorLocation(PositionKm / KmPerWorldUnit);
}

AStarBody::AStarBody()
{
    DebugColor = FColor::Yellow;
}