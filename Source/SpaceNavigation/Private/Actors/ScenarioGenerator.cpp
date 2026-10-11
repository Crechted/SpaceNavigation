// Copyright Epic Games, Inc. All Rights Reserved.

#include "Actors/ScenarioGenerator.h"
#include "Core/OrbitMath.h"
#include "Core/SimTypes.h"
#include "Math/RandomStream.h"

namespace
{
const FName RandomSeedName(TEXT("Зерно генератора случайных чисел"));
const FName NumberOfPlanetsName(TEXT("Число планет"));
const FName MinOrbitalRadiusName(TEXT("Минимальный орбитальный радиус планет"));
const FName UseTargetDistanceName(TEXT("Использовать фиксированное целевое расстояние"));
const FName TargetDistanceName(TEXT("Целевая дистанция (если выше true)"));
const FName SystemBoundaryKName(TEXT("Коэффициент расчета (K * максимальный орбитальный радиус)"));
const FName UseComputedSOIName(TEXT("Использовать фиксированную сферу влияния тел"));
const FName SOIKName(TEXT("Коэффициент для упрощенного расчета сферы влияния (если выше false)"));
const FName MassVariationName(TEXT("Разброс масс планет относительно номинальной земной, доля"));
const FName RadiusVariationName(TEXT("Разброс плотности планет (радиуса относительно земного), доля"));
const FName SpacingVariationName(TEXT("Разброс интервала между соседними орбитами, доля от базового шага"));
const FName MaxInclinationName(TEXT("Максимальный наклон орбиты, радианы"));
const FName StarMassOverrideName(TEXT("Масса центральной звезды, кг. Ноль — значение по умолчанию (масса Солнца)"));
const FName StarRadiusOverrideName(TEXT("Радиус центральной звезды, км. Ноль — значение по умолчанию"));
}

namespace ScenarioGeneratorInternal
{
/** Масса Земли, кг — номинал для генерации планет. */
constexpr double EarthMass = 5.9722e24;

/** Радиус Земли, км — номинал для генерации планет. */
constexpr double EarthRadius = 6371.0;
}

UScenarioGenerator::UScenarioGenerator()
{
}

TArray<FParameter> UScenarioGenerator::ExtractParameters_Implementation()
{
    TArray<FParameter> Result;

    Result.Add(FParameter(RandomSeedName, EParamTypes::Int32, LexToString(Params.RandomSeed)));
    Result.Add(FParameter(NumberOfPlanetsName, EParamTypes::Int32, LexToString(Params.NumberOfPlanets)));
    Result.Add(FParameter(MinOrbitalRadiusName, EParamTypes::Double, LexToString(Params.MinOrbitalRadius), TEXT("км")));
    Result.Add(FParameter(UseTargetDistanceName, EParamTypes::Double, LexToString(Params.bUseTargetDistance)));
    Result.Add(FParameter(TargetDistanceName, EParamTypes::Double, LexToString(Params.TargetDistance), TEXT("км")));
    Result.Add(FParameter(SystemBoundaryKName, EParamTypes::Double, LexToString(Params.SystemBoundaryK)));
    Result.Add(FParameter(UseComputedSOIName, EParamTypes::Bool, LexToString(Params.bUseComputedSOI)));
    Result.Add(FParameter(SOIKName, EParamTypes::Double, LexToString(Params.SOIK)));

    Result.Add(FParameter(MassVariationName, EParamTypes::Double, LexToString(Params.MassVariation)));
    Result.Add(FParameter(RadiusVariationName, EParamTypes::Double, LexToString(Params.RadiusVariation)));
    Result.Add(FParameter(SpacingVariationName, EParamTypes::Double, LexToString(Params.SpacingVariation)));
    Result.Add(FParameter(MaxInclinationName, EParamTypes::Double, LexToString(Params.MaxInclination), TEXT("рад")));

    Result.Add(FParameter(StarMassOverrideName, EParamTypes::Double, LexToString(Params.StarMassOverride), TEXT("кг")));
    Result.Add(FParameter(StarRadiusOverrideName, EParamTypes::Double, LexToString(Params.StarRadiusOverride), TEXT("км")));

    return Result;
}

bool UScenarioGenerator::ApplyParameters_Implementation(const TArray<FParameter>& InParameters)
{

    for (const auto& Parameter : InParameters)
    {
        if (Parameter.LocalName == RandomSeedName)
            Params.RandomSeed = FCString::Atoi(*Parameter.Value);

        if (Parameter.LocalName == NumberOfPlanetsName)
            Params.NumberOfPlanets = FCString::Atoi(*Parameter.Value);

        if (Parameter.LocalName == MinOrbitalRadiusName)
            Params.MinOrbitalRadius = FCString::Atod(*Parameter.Value);

        if (Parameter.LocalName == UseTargetDistanceName)
            Params.bUseTargetDistance = Parameter.Value.ToBool();

        if (Parameter.LocalName == TargetDistanceName)
            Params.TargetDistance = FCString::Atod(*Parameter.Value);

        if (Parameter.LocalName == SystemBoundaryKName)
            Params.SystemBoundaryK = FCString::Atod(*Parameter.Value);

        if (Parameter.LocalName == UseComputedSOIName)
            Params.bUseComputedSOI = Parameter.Value.ToBool();

        if (Parameter.LocalName == SOIKName)
            Params.SOIK = FCString::Atod(*Parameter.Value);

        if (Parameter.LocalName == MassVariationName)
            Params.MassVariation = FCString::Atod(*Parameter.Value);

        if (Parameter.LocalName == RadiusVariationName)
            Params.RadiusVariation = FCString::Atod(*Parameter.Value);

        if (Parameter.LocalName == SpacingVariationName)
            Params.SpacingVariation = FCString::Atod(*Parameter.Value);

        if (Parameter.LocalName == MaxInclinationName)
            Params.MaxInclination = FCString::Atod(*Parameter.Value);

        if (Parameter.LocalName == StarMassOverrideName)
            Params.StarMassOverride = FCString::Atod(*Parameter.Value);

        if (Parameter.LocalName == StarRadiusOverrideName)
            Params.StarRadiusOverride = FCString::Atod(*Parameter.Value);
    }

    return true;
}

void UScenarioGenerator::GenerateScenario()
{
    PlanetData.Reset();
    PlanetData.Reserve(FMath::Max(Params.NumberOfPlanets, 0));

    // Детерминированный поток: весь случайный выбор идёт только через него.
    FRandomStream Stream(Params.RandomSeed);

    // --- Центральная звезда ---
    StarData = FCelestialBodyData();
    StarData.BodyIndex = 0;
    StarData.BodyName = TEXT("Star");
    StarData.Mass = Params.StarMassOverride > 0.0 ? Params.StarMassOverride : SimConstants::SolarMass;
    StarData.Radius = Params.StarRadiusOverride > 0.0 ? Params.StarRadiusOverride : SimConstants::SolarRadius;
    StarData.OrbitalRadius = 0.0;
    StarData.OrbitalVelocity = 0.0;
    StarData.InitialPhase = 0.0;
    StarData.OrbitalInclination = 0.0;
    StarData.SphereOfInfluence = 0.0;

    // --- Планеты ---
    const int32 PlanetCount = FMath::Max(Params.NumberOfPlanets, 0);
    const double BaseStep = SimConstants::AuToKm(0.5);
    double CurrentRadius = FMath::Max(Params.MinOrbitalRadius, StarData.Radius * 3.0);

    for (int32 Index = 0; Index < PlanetCount; ++Index)
    {
        FCelestialBodyData Body;
        Body.BodyIndex = Index + 1;
        Body.BodyName = FName(*FString::Printf(TEXT("Planet_%02d"), Index + 1));

        // Орбитальный радиус: равномерный шаг с детерминированным разбросом.
        const double Spacing = BaseStep * (1.0 + Stream.FRandRange(-Params.SpacingVariation, Params.SpacingVariation));
        Body.OrbitalRadius = CurrentRadius + FMath::Max(Spacing, BaseStep * 0.25);

        // Масса и радиус: номинал земли с детерминированным разбросом.
        Body.Mass = ScenarioGeneratorInternal::EarthMass * (1.0 + Stream.FRandRange(-Params.MassVariation, Params.MassVariation));
        Body.Radius = ScenarioGeneratorInternal::EarthRadius * (1.0 + Stream.FRandRange(-Params.RadiusVariation, Params.RadiusVariation));
        Body.Mass = FMath::Max(Body.Mass, SimConstants::MinimumBodyMass);
        Body.Radius = FMath::Max(Body.Radius, 1.0);

        // Начальная фаза: равномерно по всей орбите.
        Body.InitialPhase = Stream.FRandRange(0.0, 2.0 * UE_DOUBLE_PI);

        // Наклон орбиты (опциональный параметр ТЗ).
        Body.OrbitalInclination = Stream.FRandRange(-Params.MaxInclination, Params.MaxInclination);

        // Орбитальная скорость — вычисляемая величина (видимая скорость по круговой орбите).
        Body.OrbitalVelocity = UOrbitMath::CircularOrbitalVelocity(StarData.Mass, Body.OrbitalRadius);

        // Сфера влияния: если заданное значение считается малой силой — считаем по формуле.
        Body.SphereOfInfluence = GetBodySOI(Body);

        PlanetData.Add(Body);
        CurrentRadius = Body.OrbitalRadius;
    }

    // Сфера влияния звезды не вычисляется: она центральная и ограничивается границей системы.
    StarData.SphereOfInfluence = GetSystemBoundary();

    SystemBoundary = GetSystemBoundary();
    OnGenerated.Broadcast();
}

void UScenarioGenerator::ClearScenario()
{
    PlanetData.Reset();
    SystemBoundary = 0.0;
}

TArray<FCelestialBodyData> UScenarioGenerator::GetAllBodies() const
{
    TArray<FCelestialBodyData> Result;
    Result.Reserve(PlanetData.Num() + 1);
    Result.Add(StarData);
    Result.Append(PlanetData);
    return Result;
}

bool UScenarioGenerator::GetBodyByIndex(int32 BodyIndex, FCelestialBodyData& OutBody) const
{
    if (BodyIndex == 0)
    {
        OutBody = StarData;
        return true;
    }

    const int32 PlanetIndex = BodyIndex - 1;
    if (PlanetData.IsValidIndex(PlanetIndex))
    {
        OutBody = PlanetData[PlanetIndex];
        return true;
    }

    return false;
}

double UScenarioGenerator::GetCentralMass() const
{
    return StarData.Mass;
}

double UScenarioGenerator::GetSystemBoundary() const
{
    // По ТЗ доступны два варианта границы системы.
    if (Params.bUseTargetDistance)
    {
        return FMath::Max(Params.TargetDistance, StarData.Radius);
    }

    double MaxOrbitalRadius = 0.0;
    for (const FCelestialBodyData& Body : PlanetData)
    {
        MaxOrbitalRadius = FMath::Max(MaxOrbitalRadius, Body.OrbitalRadius);
    }

    if (MaxOrbitalRadius <= 0.0)
    {
        // Система без планет: граница — минимум из заданного радиуса и радиуса звезды.
        return FMath::Max(Params.MinOrbitalRadius * Params.SystemBoundaryK, StarData.Radius);
    }

    return FMath::Max(Params.SystemBoundaryK * MaxOrbitalRadius, StarData.Radius);
}

double UScenarioGenerator::GetBodySOI(const FCelestialBodyData& Body) const
{
    // Фиксированный вариант: сфера влияния как кратное радиуса тела.
    const double FixedRadius = Body.Radius * Params.SOIK;

    if (!Params.bUseComputedSOI)
    {
        return FixedRadius;
    }

    // Физическая сфера действия r_SOI = a * (m/M)^(2/5). При bUseComputedSOI
    // используем именно её, а не фиксированный коэффициент: при малом SOI
    // (кратном радиусу) аппарат покидает сферу влияния задолго до выхода на
    // асимптоту гиперболы, и патч-конический отход не совпадает с расчётом
    // планировщика (теряется часть гелиоцентрической скорости -> промах).
    return UOrbitMath::ComputeSphereOfInfluence(
        Body.Mass,
        StarData.Mass,
        Body.OrbitalRadius,
        FixedRadius);
}

double UScenarioGenerator::GetBodyOrbitRadius(int32 BodyIndex) const
{
    FCelestialBodyData Body;
    if (!GetBodyByIndex(BodyIndex, Body))
    {
        return 0.0;
    }

    // Для звезды используем радиус орбиты самой внутренней планеты:
    // стартовая орбита вокруг звезды задаётся относительно её края.
    if (BodyIndex == 0)
    {
        if (PlanetData.Num() > 0)
        {
            return PlanetData[0].OrbitalRadius;
        }
        return FMath::Max(Params.MinOrbitalRadius, StarData.Radius);
    }

    return Body.OrbitalRadius;
}