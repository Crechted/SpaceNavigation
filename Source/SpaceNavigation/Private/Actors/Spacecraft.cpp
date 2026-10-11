// Copyright Epic Games, Inc. All Rights Reserved.

#include "Actors/Spacecraft.h"
#include "Components/SceneComponent.h"
#include "Core/OrbitMath.h"
#include "Core/TwoBodyPropagator.h"

namespace
{
const FName DryMassName(TEXT("Масса без топлива"));
const FName FuelMassName(TEXT("Масса топлива"));
const FName MaxThrustName(TEXT("Максимальная тяга"));
const FName SpecificImpulseName(TEXT("Удельный импульс"));
const FName StartingBodyIndexName(TEXT("Индекс стартового тела"));
const FName StartingOrbitAltitudeName(TEXT("Радиус начальной орбиты"));
const FName InitialPhaseName(TEXT("Начальная фаза"));
const FName InitialVelocityName(TEXT("Начальная скорость"));
const FName InitialOrbitCircularName(TEXT("Ускорять ли начальный вектор скорости до круговой скорости, если InitialVelocity == 0"));
}

ASpacecraft::ASpacecraft()
{
    PrimaryActorTick.bCanEverTick = false;

    RootComponent = CreateDefaultSubobject<USceneComponent>(TEXT("Root"));
    SetRootComponent(RootComponent);

    StaticMeshComponent = CreateDefaultSubobject<UStaticMeshComponent>(TEXT("StaticMesh"));
    StaticMeshComponent->SetupAttachment(RootComponent);
}

void ASpacecraft::BeginPlay()
{
    Super::BeginPlay();
}

void ASpacecraft::Tick(float DeltaTime)
{
    Super::Tick(DeltaTime);
}

FParameters ASpacecraft::ExtractParameters_Implementation()
{
    FParameters Result;

    Result.Parameters.Add(FParameter(DryMassName, EParamTypes::Double, LexToString(Params.DryMass), TEXT("кг")));
    Result.Parameters.Add(FParameter(FuelMassName, EParamTypes::Double, LexToString(Params.FuelMass), TEXT("кг")));
    Result.Parameters.Add(FParameter(MaxThrustName, EParamTypes::Double, LexToString(Params.MaxThrust), TEXT("Н")));
    Result.Parameters.Add(FParameter(SpecificImpulseName, EParamTypes::Double, LexToString(Params.SpecificImpulse), TEXT("с")));
    Result.Parameters.Add(FParameter(StartingBodyIndexName, EParamTypes::Int32, LexToString(Params.StartingBodyIndex)));
    Result.Parameters.Add(FParameter(StartingOrbitAltitudeName, EParamTypes::Double, LexToString(Params.StartingOrbitAltitude),
        TEXT("км")));
    Result.Parameters.Add(FParameter(InitialPhaseName, EParamTypes::Double, LexToString(Params.InitialPhase), TEXT("рад")));
    Result.Parameters.Add(FParameter(InitialVelocityName, EParamTypes::Double, LexToString(Params.InitialVelocity), TEXT("км/с")));
    Result.Parameters.Add(FParameter(InitialOrbitCircularName, EParamTypes::Bool, LexToString(Params.bInitialOrbitCircular)));

    return Result;
}

bool ASpacecraft::ApplyParameters_Implementation(const FParameters& InParameters)
{
    for (const auto& Parameter : InParameters.Parameters)
    {
        if (Parameter.LocalName == DryMassName)
            Params.DryMass = FCString::Atod(*Parameter.Value);

        if (Parameter.LocalName == FuelMassName)
            Params.FuelMass = FCString::Atod(*Parameter.Value);

        if (Parameter.LocalName == MaxThrustName)
            Params.MaxThrust = FCString::Atod(*Parameter.Value);

        if (Parameter.LocalName == SpecificImpulseName)
            Params.SpecificImpulse = FCString::Atod(*Parameter.Value);

        if (Parameter.LocalName == StartingBodyIndexName)
            Params.StartingBodyIndex = FCString::Atod(*Parameter.Value);

        if (Parameter.LocalName == StartingOrbitAltitudeName)
            Params.StartingOrbitAltitude = FCString::Atod(*Parameter.Value);

        if (Parameter.LocalName == InitialPhaseName)
            Params.InitialPhase = FCString::Atod(*Parameter.Value);

        if (Parameter.LocalName == InitialVelocityName)
            Params.InitialVelocity = FCString::Atod(*Parameter.Value);

        if (Parameter.LocalName == InitialOrbitCircularName)
            Params.bInitialOrbitCircular = Parameter.Value.ToBool();
    }

    return true;
}

void ASpacecraft::InitializeSpacecraft(ACelestialBody* StartBody, ACelestialBody* StarBody)
{
    ResetState();

    CurrentCentralBody = StartBody;

    if (!StartBody)
    {
        return;
    }

    // Радиус стартовой орбиты отсчитывается от поверхности тела.
    //
    // Именно BodyData.Radius, а не GetOrbitRadiusKm(): последний возвращает
    // гелиоцентрический радиус орбиты планеты, и для тела на 1 AU дал бы
    // стартовую орбиту радиусом порядка 1.3e8 км вокруг самой планеты.
    const double OrbitRadiusKm = FMath::Max(StartBody->BodyData.Radius + Params.StartingOrbitAltitude, 1.0);

    // Начальная скорость: круговая по умолчанию, иначе берётся из параметров.
    const double CircularSpeed = UOrbitMath::CircularOrbitalVelocity(StartBody->BodyData.Mass, OrbitRadiusKm);
    const double SpeedKmS = Params.bInitialOrbitCircular || FMath::IsNearlyZero(Params.InitialVelocity)
                                ? CircularSpeed
                                : Params.InitialVelocity;

    // Парковочную орбиту ориентируем по движению центрального тела (prograde).
    //
    // Планировщик рассчитывает импульс отхода в предположении, что скорость на
    // парковочной орбите сонаправлена с орбитальным движением тела. Если задать
    // орбиту в произвольной плоскости (например, в мировой XY), отход уходит в
    // сторону: гелиоцентрическая траектория получается произвольной, и аппарат
    // может остаться на замкнутой орбите у звезды, так и не дойдя до границы.
    FVector ProgradeDirection = StartBody->VelocityKmS.GetSafeNormal();

    // Неподвижное центральное тело (звезда): направление движения задаёт фаза.
    if (ProgradeDirection.IsNearlyZero())
    {
        ProgradeDirection = FVector(-FMath::Sin(Params.InitialPhase), FMath::Cos(Params.InitialPhase), 0.0);
    }

    // Положение берём перпендикулярно движению: в начальный момент скорость
    // строго по движению, а радиус равен парковочному.
    const FVector ReferenceAxis = FMath::Abs(ProgradeDirection.Z) < 0.9 ? FVector::UpVector : FVector::ForwardVector;
    const FVector RadialDirection = FVector::CrossProduct(ReferenceAxis, ProgradeDirection).GetSafeNormal();

    RelativePositionKm = RadialDirection * OrbitRadiusKm;
    RelativeVelocityKmS = ProgradeDirection * SpeedKmS;

    UpdateAbsoluteState(StarBody);
}

void ASpacecraft::ResetState()
{
    FuelMassKg = FMath::Max(Params.FuelMass, 0.0);
    CurrentMassKg = Params.DryMass + FuelMassKg;
    TotalDeltaVSpentKmS = 0.0;
    TotalFuelSpentKg = 0.0;
    FlightTimeSeconds = 0.0;
    bEngineRunning = false;
    ThrustDirectionLocal = FVector::ZeroVector;
    RelativePositionKm = FVector::ZeroVector;
    RelativeVelocityKmS = FVector::ZeroVector;
    AbsolutePositionKm = FVector::ZeroVector;
    AbsoluteVelocityKmS = FVector::ZeroVector;
}

void ASpacecraft::UpdateAbsoluteState(ACelestialBody* StarBody)
{
    if (!CurrentCentralBody)
    {
        AbsolutePositionKm = RelativePositionKm;
        AbsoluteVelocityKmS = RelativeVelocityKmS;
        return;
    }

    // Абсолютное состояние = состояние центрального тела + относительное состояние аппарата.
    AbsolutePositionKm = CurrentCentralBody->PositionKm + RelativePositionKm;
    AbsoluteVelocityKmS = CurrentCentralBody->VelocityKmS + RelativeVelocityKmS;

    if (CurrentCentralBody->IsA(AStarBody::StaticClass()) && StarBody && CurrentCentralBody != StarBody)
    {
        // Центральное тело не совпадает со звездой: приводим координаты к системе звезды.
        AbsolutePositionKm += StarBody->PositionKm;
        AbsoluteVelocityKmS += StarBody->VelocityKmS;
    }
}

void ASpacecraft::IntegrateStep(double DeltaTimeSeconds)
{
    if (DeltaTimeSeconds <= 0.0)
    {
        return;
    }

    if (!CurrentCentralBody)
    {
        return;
    }

    FlightTimeSeconds += DeltaTimeSeconds;

    if (bEngineRunning)
    {
        // Работающий двигатель: гравитация плюс тяга. Интегрируем симплектическим методом Эйлера.
        const double Mass = FMath::Max(CurrentMassKg, 1.0e-6);
        const double Distance = RelativePositionKm.Size();

        // Тяга в ньютонах даёт ускорение в м/с^2; переводим в км/с^2.
        const double ThrustAcceleration = (Params.MaxThrust / Mass) / 1000.0;

        // Расход топлива по формуле mdot = F / (Isp * g0).
        const double FuelRate = UOrbitMath::FuelConsumptionRate(Params.MaxThrust, Params.SpecificImpulse);
        const double FuelToBurn = FMath::Min(FuelRate * DeltaTimeSeconds, FuelMassKg);

        if (FuelToBurn > 0.0)
        {
            FuelMassKg -= FuelToBurn;
            TotalFuelSpentKg += FuelToBurn;
            CurrentMassKg = Params.DryMass + FuelMassKg;
        }
        else
        {
            // Топливо закончилось — двигатель выключается.
            bEngineRunning = false;
        }

        const FVector GravityAcceleration =
            RelativePositionKm.GetSafeNormal() * (-UOrbitMath::GravitationalAcceleration(CurrentCentralBody->BodyData.Mass, Distance));

        ThrustDirectionLocal = RelativeVelocityKmS.GetSafeNormal();
        const FVector TotalAcceleration = GravityAcceleration + ThrustDirectionLocal * ThrustAcceleration;

        RelativeVelocityKmS += TotalAcceleration * DeltaTimeSeconds;
        RelativePositionKm += RelativeVelocityKmS * DeltaTimeSeconds;
    }
    else
    {
        // Двигатель выключен: используем аналитическую кеплеровскую propagацию.
        FTwoBodyState NewState;
        const bool bPropagated = UTwoBodyPropagator::PropagateOrbit(
            RelativePositionKm,
            RelativeVelocityKmS,
            CurrentCentralBody->BodyData.Mass,
            DeltaTimeSeconds,
            NewState);

        if (bPropagated)
        {
            RelativePositionKm = NewState.PositionKm;
            RelativeVelocityKmS = NewState.VelocityKmS;
        }
        else
        {
            // Резервный вариант: симплектический шаг Эйлера по гравитации.
            const double Distance = RelativePositionKm.Size();
            const FVector Acceleration =
                RelativePositionKm.GetSafeNormal() * (-UOrbitMath::GravitationalAcceleration(CurrentCentralBody->BodyData.Mass, Distance));

            RelativeVelocityKmS += Acceleration * DeltaTimeSeconds;
            RelativePositionKm += RelativeVelocityKmS * DeltaTimeSeconds;
        }
    }
}

double ASpacecraft::ApplyImpulse(double DeltaVKmS, const FVector& Direction)
{
    if (DeltaVKmS <= 0.0 || FuelMassKg <= 0.0)
    {
        return 0.0;
    }

    const FVector ImpulseDirection = Direction.GetSafeNormal();
    if (ImpulseDirection.IsNearlyZero())
    {
        return 0.0;
    }

    // Проверяем, хватит ли топлива. Расход считаем по формуле Циолковского.
    const double RequiredFuel = UOrbitMath::RequiredFuelMass(Params.DryMass, DeltaVKmS, Params.SpecificImpulse);
    double AppliedDeltaV = DeltaVKmS;

    if (RequiredFuel > FuelMassKg)
    {
        // Топлива недостаточно: ограничиваем импульс величиной, достижимой на текущем запасе.
        AppliedDeltaV = GetRemainingDeltaV();
    }

    if (AppliedDeltaV <= 0.0)
    {
        return 0.0;
    }

    const double FuelUsed = FMath::Min(UOrbitMath::RequiredFuelMass(Params.DryMass, AppliedDeltaV, Params.SpecificImpulse), FuelMassKg);

    FuelMassKg -= FuelUsed;
    TotalFuelSpentKg += FuelUsed;
    CurrentMassKg = Params.DryMass + FuelMassKg;
    TotalDeltaVSpentKmS += AppliedDeltaV;
    RelativeVelocityKmS += ImpulseDirection * AppliedDeltaV;

    if (FuelMassKg <= 0.0)
    {
        bEngineRunning = false;
    }

    return AppliedDeltaV;
}

void ASpacecraft::SetEngineRunning(bool bRunning)
{
    bEngineRunning = bRunning && FuelMassKg > 0.0 && Params.MaxThrust > 0.0;
}

double ASpacecraft::GetBurnDuration(double DeltaVKmS) const
{
    if (DeltaVKmS <= 0.0 || Params.MaxThrust <= 0.0)
    {
        return 0.0;
    }

    const double FuelRate = UOrbitMath::FuelConsumptionRate(Params.MaxThrust, Params.SpecificImpulse);
    if (FuelRate <= 0.0)
    {
        return 0.0;
    }

    return UOrbitMath::RequiredFuelMass(Params.DryMass, DeltaVKmS, Params.SpecificImpulse) / FuelRate;
}

double ASpacecraft::GetRemainingDeltaV() const
{
    return UOrbitMath::MaxDeltaV(Params.DryMass, FuelMassKg, Params.SpecificImpulse);
}

bool ASpacecraft::HasEnoughFuel(double DeltaVKmS) const
{
    return FuelMassKg >= UOrbitMath::RequiredFuelMass(Params.DryMass, DeltaVKmS, Params.SpecificImpulse);
}

double ASpacecraft::GetDistanceToCentralBody() const
{
    return RelativePositionKm.Size();
}

double ASpacecraft::GetEscapeVelocity() const
{
    if (!CurrentCentralBody)
    {
        return 0.0;
    }

    return UOrbitMath::EscapeVelocity(CurrentCentralBody->BodyData.Mass, GetDistanceToCentralBody());
}

double ASpacecraft::GetDistanceFromStar(ACelestialBody* StarBody) const
{
    return (StarBody ? AbsolutePositionKm - StarBody->PositionKm : AbsolutePositionKm).Size();
}