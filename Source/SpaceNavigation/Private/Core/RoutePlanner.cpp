// Copyright Epic Games, Inc. All Rights Reserved.

#include "Core/RoutePlanner.h"
#include "Core/OrbitMath.h"
#include "Core/RouteCriterion.h"
#include "Core/SimConstants.h"

#include "Async/Async.h"

DECLARE_LOG_CATEGORY_CLASS(LogRoutePlanner, Log, All);

namespace
{
const FName MaxLegsName(TEXT("Максимальное число участков в маршруте"));
const FName BeamWidthName(TEXT("Максимальное число вариантов, удерживаемых в пучке на каждом шаге"));
const FName MaxFlightTimeName(TEXT("Максимальное время полёта, с. Ноль — ограничение не проверяется"));
const FName AllowInwardTransfersName(TEXT("Разрешить переходы к внутренним планетам"));
}

namespace RoutePlannerDetail
{
/** Множители сетки TOF: время перелёта = множитель × параболический минимум. */
constexpr double ToFMultipliers[] = {1.6, 2.2, 3.0};

/** Число окон фазирования (значений k) на каждое значение TOF. */
constexpr int32 MaxPhasingWindows = 4;

/**
 * Высота сближения с планетой назначения на инклинационном (3D) перелёте
 * Ламберта. Для оценки риска берём константу безопасного пролёта — она не
 * зависит от конфигурируемой высоты парковочной орбиты StartingOrbitAltitude.
 */
constexpr double LambertApproachAltitudeKm = 400.0;

/**
 * Параболический (минимально возможный) перелёт между двумя позициями.
 * s — полупериметр, c — хорда.
 */
double ParabolicTransferTime(double MuKm3PerS2, double R1Km, double R2Km, double ChordKm)
{
    if (MuKm3PerS2 <= 0.0 || R1Km <= 0.0 || R2Km <= 0.0 || ChordKm <= 0.0)
    {
        return -1.0;
    }

    const double SemiPerimeter = 0.5 * (R1Km + R2Km + ChordKm);
    const double Tail = SemiPerimeter - ChordKm;
    if (SemiPerimeter <= 0.0 || Tail <= 0.0)
    {
        return -1.0;
    }

    return FMath::Sqrt(2.0 / MuKm3PerS2) *
           (FMath::Pow(SemiPerimeter, 1.5) - FMath::Pow(Tail, 1.5)) / 3.0;
}
}

FParameters URoutePlanner::ExtractParameters_Implementation()
{
    FParameters Result;

    Result.Parameters.Add(FParameter(MaxLegsName, EParamTypes::Int32, LexToString(PlannerLimits.MaxLegs)));
    Result.Parameters.Add(FParameter(BeamWidthName, EParamTypes::Int32, LexToString(PlannerLimits.BeamWidth)));
    Result.Parameters.Add(FParameter(MaxFlightTimeName, EParamTypes::Double, LexToString(PlannerLimits.MaxFlightTime), TEXT("c")));
    Result.Parameters.Add(FParameter(AllowInwardTransfersName, EParamTypes::Bool, LexToString(PlannerLimits.bAllowInwardTransfers)));

    return Result;
}

bool URoutePlanner::ApplyParameters_Implementation(const FParameters& InParameters)
{
    for (const auto& Parameter : InParameters.Parameters)
    {
        if (Parameter.LocalName == MaxLegsName)
            PlannerLimits.MaxLegs = FCString::Atoi(*Parameter.Value);

        if (Parameter.LocalName == BeamWidthName)
            PlannerLimits.BeamWidth = FCString::Atoi(*Parameter.Value);

        if (Parameter.LocalName == MaxFlightTimeName)
            PlannerLimits.MaxFlightTime = FCString::Atod(*Parameter.Value);

        if (Parameter.LocalName == AllowInwardTransfersName)
            PlannerLimits.bAllowInwardTransfers = Parameter.Value.ToBool();
    }

    return true;
}

void URoutePlanner::Initialize(
    const TArray<FCelestialBodyData>& InBodies,
    const FSpacecraftParams& InSpacecraftParams,
    double InSystemBoundaryKm)
{
    Bodies = InBodies;
    SpacecraftParams = InSpacecraftParams;
    SystemBoundaryKm = InSystemBoundaryKm;

    StartBodyIndex = Bodies.IsValidIndex(SpacecraftParams.StartingBodyIndex)
                         ? SpacecraftParams.StartingBodyIndex
                         : 0;
}

double URoutePlanner::GetHeliocentricRadius(int32 BodyIndex) const
{
    if (BodyIndex == BoundaryTargetIndex)
    {
        return SystemBoundaryKm;
    }

    if (!Bodies.IsValidIndex(BodyIndex))
    {
        return 0.0;
    }

    // Для центральной звезды берём радиус орбиты аппарата вокруг её поверхности.
    if (BodyIndex == 0)
    {
        return FMath::Max(Bodies[0].Radius + SpacecraftParams.StartingOrbitAltitude, 1.0);
    }

    return Bodies[BodyIndex].OrbitalRadius;
}

double URoutePlanner::GetPlanetParkingRadius(int32 BodyIndex) const
{
    if (!Bodies.IsValidIndex(BodyIndex))
    {
        return 0.0;
    }

    return FMath::Max(Bodies[BodyIndex].Radius + SpacecraftParams.StartingOrbitAltitude, 1.0);
}

double URoutePlanner::ComputeRequiredFuel(double TotalDeltaVKmS) const
{
    return UOrbitMath::RequiredFuelMass(SpacecraftParams.DryMass, TotalDeltaVKmS, SpacecraftParams.SpecificImpulse);
}

double URoutePlanner::GetBodyAngularVelocity(int32 BodyIndex) const
{
    if (!Bodies.IsValidIndex(BodyIndex) || BodyIndex == 0)
    {
        return 0.0;
    }

    const FCelestialBodyData& Body = Bodies[BodyIndex];
    if (Body.OrbitalRadius <= 0.0 || Body.OrbitalVelocity <= 0.0)
    {
        return 0.0;
    }

    // Как в CelestialBody::GetAngularVelocity: круговая орбита.
    return UOrbitMath::CircularOrbitalVelocity(Bodies[0].Mass, Body.OrbitalRadius) / Body.OrbitalRadius;
}

FVector URoutePlanner::GetBodyPositionKm(int32 BodyIndex, double AbsoluteTimeSeconds) const
{
    if (!Bodies.IsValidIndex(BodyIndex) || BodyIndex == 0)
    {
        return FVector::ZeroVector;
    }

    const FCelestialBodyData& Body = Bodies[BodyIndex];
    if (Body.OrbitalRadius <= 0.0)
    {
        return FVector::ZeroVector;
    }

    // Та же кинематика, что в CelestialBody::UpdateOrbitPosition:
    // поворот вокруг оси X на наклон, локальная круговая орбита в плоскости XY.
    const double AngularVelocity = GetBodyAngularVelocity(BodyIndex);
    const double Angle = Body.InitialPhase + AngularVelocity * AbsoluteTimeSeconds;
    const FVector LocalPosition(
        Body.OrbitalRadius * FMath::Cos(Angle),
        Body.OrbitalRadius * FMath::Sin(Angle),
        0.0);

    return FQuat(FVector::XAxisVector, Body.OrbitalInclination).RotateVector(LocalPosition);
}

void URoutePlanner::AddLog(const FString& LogText)
{
    Logs.Add(LogText);
    OnLogAdded.Broadcast(LogText);
    UE_LOG(LogRoutePlanner, Log, TEXT("[Plan] %s"), *LogText);
}

FVector URoutePlanner::GetBodyVelocityKmS(int32 BodyIndex, double AbsoluteTimeSeconds) const
{
    if (!Bodies.IsValidIndex(BodyIndex) || BodyIndex == 0)
    {
        return FVector::ZeroVector;
    }

    const FCelestialBodyData& Body = Bodies[BodyIndex];
    if (Body.OrbitalRadius <= 0.0)
    {
        return FVector::ZeroVector;
    }

    const double AngularVelocity = GetBodyAngularVelocity(BodyIndex);
    const double Angle = Body.InitialPhase + AngularVelocity * AbsoluteTimeSeconds;
    const FVector LocalVelocity(
        -Body.OrbitalRadius * AngularVelocity * FMath::Sin(Angle),
        Body.OrbitalRadius * AngularVelocity * FMath::Cos(Angle),
        0.0);

    return FQuat(FVector::XAxisVector, Body.OrbitalInclination).RotateVector(LocalVelocity);
}

bool URoutePlanner::ComputeManeuverHohmann(int32 FromBodyIndex, int32 ToBodyIndex, FManeuver& OutManeuver) const
{
    OutManeuver = FManeuver();

    const bool bToBoundary = ToBodyIndex == BoundaryTargetIndex;
    if (!bToBoundary && !Bodies.IsValidIndex(ToBodyIndex))
    {
        return false;
    }

    if (!bToBoundary && FromBodyIndex == ToBodyIndex)
    {
        return false;
    }

    if (!bToBoundary && FromBodyIndex >= Bodies.Num())
    {
        return false;
    }

    const double StartRadius = GetHeliocentricRadius(FromBodyIndex);
    const double EndRadius = bToBoundary ? SystemBoundaryKm : Bodies[ToBodyIndex].OrbitalRadius;

    if (StartRadius <= 0.0 || EndRadius <= 0.0)
    {
        return false;
    }

    const double StarMass = Bodies[0].Mass;
    if (StarMass <= 0.0)
    {
        return false;
    }

    const FHohmannTransfer Transfer = UOrbitMath::ComputeHohmannTransfer(StarMass, StartRadius, EndRadius);
    const double RadiusRatio = FMath::Max(StartRadius, EndRadius) / FMath::Min(StartRadius, EndRadius);

    OutManeuver.FromBodyIndex = FromBodyIndex;
    OutManeuver.ToBodyIndex = ToBodyIndex;
    OutManeuver.StartRadius = StartRadius;
    OutManeuver.EndRadius = EndRadius;
    OutManeuver.DurationSeconds = Transfer.TransferTime;
    OutManeuver.bBiElliptic = RadiusRatio > 11.94;

    const bool bFromStar = (FromBodyIndex == 0);
    double DepartureDeltaV = 0.0;
    double ArrivalDeltaV = 0.0;
    // Относительная скорость входа в сферу влияния тела назначения, км/с.
    double ArrivalExcessSpeed = 0.0;

    if (bFromStar)
    {
        // Стартуем из орбиты вокруг звезды: классический импульс перехода.
        DepartureDeltaV = FMath::Abs(Transfer.TransferVelocityStart - Transfer.VelocityStart);
    }
    else
    {
        // Покидаем сферу влияния планеты.
        const double BodyMass = Bodies[FromBodyIndex].Mass;
        const double ParkingRadius = GetPlanetParkingRadius(FromBodyIndex);
        const double CircularSpeed = UOrbitMath::CircularOrbitalVelocity(BodyMass, ParkingRadius);
        const double EscapeSpeed = UOrbitMath::EscapeVelocity(BodyMass, ParkingRadius);

        // Избыточная скорость относительно звезды на радиусе StartRadius.
        const double ExcessSpeed = FMath::Abs(Transfer.TransferVelocityStart - Transfer.VelocityStart);
        const double HyperbolicSpeed = FMath::Sqrt(ExcessSpeed * ExcessSpeed + EscapeSpeed * EscapeSpeed);

        DepartureDeltaV = HyperbolicSpeed - CircularSpeed;
    }

    if (!bToBoundary)
    {
        // Захват в орбиту планеты назначения.
        const double BodyMass = Bodies[ToBodyIndex].Mass;
        const double ParkingRadius = GetPlanetParkingRadius(ToBodyIndex);
        const double CircularSpeed = UOrbitMath::CircularOrbitalVelocity(BodyMass, ParkingRadius);
        const double EscapeSpeed = UOrbitMath::EscapeVelocity(BodyMass, ParkingRadius);

        const double ExcessSpeed = FMath::Abs(Transfer.TransferVelocityEnd - Transfer.VelocityEnd);
        const double HyperbolicSpeed = FMath::Sqrt(ExcessSpeed * ExcessSpeed + EscapeSpeed * EscapeSpeed);

        ArrivalDeltaV = FMath::Abs(CircularSpeed - HyperbolicSpeed);
        ArrivalExcessSpeed = ExcessSpeed;
    }

    // На очень больших отношениях радиусов биэллиптический переход дешевле по DeltaV.
    if (OutManeuver.bBiElliptic)
    {
        const double BiEllipticDeltaV = UOrbitMath::ComputeOptimalTransferDeltaV(StarMass, StartRadius, EndRadius);
        const double PureHohmann = FMath::Abs(Transfer.TransferVelocityStart - Transfer.VelocityStart);

        // Биэллиптический переход выигрывает только в чисто звёздной части участка.
        const double Saving = FMath::Max(0.0, BiEllipticDeltaV - PureHohmann);
        DepartureDeltaV = bFromStar ? BiEllipticDeltaV : DepartureDeltaV + Saving;
    }

    DepartureDeltaV = FMath::Max(DepartureDeltaV, 0.0);
    ArrivalDeltaV = FMath::Max(ArrivalDeltaV, 0.0);

    OutManeuver.DeltaVDeparture = DepartureDeltaV;
    OutManeuver.DeltaVArrival = ArrivalDeltaV;
    OutManeuver.DeltaV = DepartureDeltaV + ArrivalDeltaV;

    // Сближение с поверхностью нужно критериям риска.
    if (bToBoundary || !Bodies.IsValidIndex(ToBodyIndex))
    {
        OutManeuver.ApproachAltitudeKm = -1.0;
        OutManeuver.TargetBodyRadiusKm = 0.0;
        OutManeuver.ApproachRelativeVelocityKmS = 0.0;
    }
    else
    {
        OutManeuver.TargetBodyRadiusKm = Bodies[ToBodyIndex].Radius;
        OutManeuver.ApproachAltitudeKm = GetPlanetParkingRadius(ToBodyIndex) - Bodies[ToBodyIndex].Radius;
        OutManeuver.ApproachRelativeVelocityKmS = ArrivalExcessSpeed;
    }

    return true;
}

bool URoutePlanner::ComputeLambertManeuver(int32 FromBodyIndex, int32 ToBodyIndex, double DepartureTimeSeconds, double ToFSeconds,
    FManeuver& OutManeuver) const
{
    OutManeuver = FManeuver();

    if (!Bodies.IsValidIndex(FromBodyIndex) || FromBodyIndex <= 0 || !Bodies.IsValidIndex(ToBodyIndex) || ToBodyIndex <= 0)
    {
        return false;
    }

    if (FromBodyIndex == ToBodyIndex || DepartureTimeSeconds < 0.0 || ToFSeconds <= 0.0)
    {
        return false;
    }

    const double StarMass = Bodies[0].Mass;
    if (StarMass <= 0.0)
    {
        return false;
    }

    const FVector R1 = GetBodyPositionKm(FromBodyIndex, DepartureTimeSeconds);
    const FVector R2 = GetBodyPositionKm(ToBodyIndex, DepartureTimeSeconds + ToFSeconds);
    if (R1.SizeSquared() <= 0.0 || R2.SizeSquared() <= 0.0)
    {
        return false;
    }

    const FLambertSolution Solution = UOrbitMath::SolveLambert(R1, R2, StarMass, ToFSeconds);
    if (!Solution.bValid || Solution.TransferTime <= 0.0)
    {
        return false;
    }

    const FVector PlanetVDeparture = GetBodyVelocityKmS(FromBodyIndex, DepartureTimeSeconds);
    const FVector PlanetVArrival = GetBodyVelocityKmS(ToBodyIndex, DepartureTimeSeconds + Solution.TransferTime);

    // Отход от планеты: избыточная скорость относительно звезды = |v1 - V_A|.
    const double DepartureExcess = (Solution.DepartureVelocityKmS - PlanetVDeparture).Size();
    const double FromBodyMass = Bodies[FromBodyIndex].Mass;
    const double FromParkingRadius = GetPlanetParkingRadius(FromBodyIndex);
    const double FromEscapeSpeed = UOrbitMath::EscapeVelocity(FromBodyMass, FromParkingRadius);
    const double FromCircularSpeed = UOrbitMath::CircularOrbitalVelocity(FromBodyMass, FromParkingRadius);
    const double DepartureHyperbolic = FMath::Sqrt(DepartureExcess * DepartureExcess + FromEscapeSpeed * FromEscapeSpeed);
    const double DepartureDeltaV = FMath::Max(0.0, DepartureHyperbolic - FromCircularSpeed);

    // Захват у планеты назначения: избыточная скорость = |v2 - V_B|.
    const double ArrivalExcess = (Solution.ArrivalVelocityKmS - PlanetVArrival).Size();
    const double ToBodyMass = Bodies[ToBodyIndex].Mass;
    const double ToParkingRadius = GetPlanetParkingRadius(ToBodyIndex);
    const double ToEscapeSpeed = UOrbitMath::EscapeVelocity(ToBodyMass, ToParkingRadius);
    const double ToCircularSpeed = UOrbitMath::CircularOrbitalVelocity(ToBodyMass, ToParkingRadius);
    const double ArrivalHyperbolic = FMath::Sqrt(ArrivalExcess * ArrivalExcess + ToEscapeSpeed * ToEscapeSpeed);
    const double ArrivalDeltaV = FMath::Max(0.0, FMath::Abs(ToCircularSpeed - ArrivalHyperbolic));

    // Sanity: reject extremely degenerate branches that blow up DV
    if (Solution.Eccentricity > 0.999 && (DepartureDeltaV > 40.0 || ArrivalDeltaV > 40.0))
    {
        return false;
    }
    if (DepartureDeltaV + ArrivalDeltaV > 120.0)
    {
        return false;
    }

    OutManeuver.FromBodyIndex = FromBodyIndex;
    OutManeuver.ToBodyIndex = ToBodyIndex;
    OutManeuver.StartRadius = R1.Size();
    OutManeuver.EndRadius = R2.Size();
    OutManeuver.DeltaVDeparture = DepartureDeltaV;
    OutManeuver.DeltaVArrival = ArrivalDeltaV;
    OutManeuver.DeltaV = DepartureDeltaV + ArrivalDeltaV;
    OutManeuver.DurationSeconds = Solution.TransferTime;
    OutManeuver.bBiElliptic = false;
    OutManeuver.DepartureVelocityKmS = Solution.DepartureVelocityKmS;
    OutManeuver.ArrivalVelocityKmS = Solution.ArrivalVelocityKmS;

    OutManeuver.TargetBodyRadiusKm = Bodies[ToBodyIndex].Radius;
    OutManeuver.ApproachAltitudeKm = RoutePlannerDetail::LambertApproachAltitudeKm;
    OutManeuver.ApproachRelativeVelocityKmS = ArrivalExcess;

    return true;
}

double URoutePlanner::LambertNominalToF(int32 FromBodyIndex, int32 ToBodyIndex, double DepartureTimeSeconds, double Multiplier) const
{
    if (FromBodyIndex <= 0 || ToBodyIndex <= 0 ||
        !Bodies.IsValidIndex(FromBodyIndex) || !Bodies.IsValidIndex(ToBodyIndex))
    {
        return -1.0;
    }

    const double StarMass = Bodies[0].Mass;
    if (StarMass <= 0.0)
    {
        return -1.0;
    }

    const FVector R1 = GetBodyPositionKm(FromBodyIndex, DepartureTimeSeconds);
    const FVector R2 = GetBodyPositionKm(ToBodyIndex, DepartureTimeSeconds);
    const double Mu = SimConstants::GravitationalConstant * StarMass;
    const double TParab = RoutePlannerDetail::ParabolicTransferTime(Mu, R1.Size(), R2.Size(), (R1 - R2).Size());
    if (TParab <= 0.0)
    {
        return -1.0;
    }

    return Multiplier * TParab;
}

void URoutePlanner::GenerateLambertCandidates(const FSearchNode& Node, int32 ToBodyIndex, TArray<FManeuver>& OutCandidates) const
{
    const int32 FromBodyIndex = Node.CurrentBodyIndex;
    if (FromBodyIndex <= 0 || ToBodyIndex <= 0 || FromBodyIndex == ToBodyIndex)
    {
        return;
    }

    if (!Bodies.IsValidIndex(FromBodyIndex) || !Bodies.IsValidIndex(ToBodyIndex))
    {
        return;
    }

    const double StarMass = Bodies[0].Mass;
    if (StarMass <= 0.0)
    {
        return;
    }

    // Масштаб TOF: параболический минимум по геометрии в стартовый момент.
    // Он не зависит от t_dep и позволяет строить окна детерминированно.
    const FVector R1Base = GetBodyPositionKm(FromBodyIndex, Node.AbsoluteTimeSeconds);
    const FVector R2Base = GetBodyPositionKm(ToBodyIndex, Node.AbsoluteTimeSeconds);
    const double Mu = SimConstants::GravitationalConstant * StarMass;
    const double TParab = RoutePlannerDetail::ParabolicTransferTime(
        Mu, R1Base.Size(), R2Base.Size(), (R1Base - R2Base).Size());
    if (TParab <= 0.0)
    {
        return;
    }

    const double OmegaA = GetBodyAngularVelocity(FromBodyIndex);
    const double OmegaB = GetBodyAngularVelocity(ToBodyIndex);
    const double OmegaDiff = OmegaA - OmegaB;
    if (FMath::Abs(OmegaDiff) < 1e-12)
    {
        // Синхронные орбиты: фазировка не сдвигает окно, сетка не строится.
        return;
    }

    const double ThetaA0 = Bodies[FromBodyIndex].InitialPhase;
    const double ThetaB0 = Bodies[ToBodyIndex].InitialPhase;
    const double T0 = Node.AbsoluteTimeSeconds;

    // Условие синодической фазы: B опережает A на π в момент прибытия.
    //   θ_B0 + ω_B (t_dep + ToF) = θ_A0 + ω_A t_dep + π + 2πk
    //   t_dep = (θ_B0 − θ_A0 + ω_B·ToF − π + 2πk) / (ω_A − ω_B)
    //
    // Период окон — синодический: окна с шагом 2π/|ω_A − ω_B|. При ω_A < ω_B
    // (отлёт от внешней планеты к внутренней) фаза растёт против хода k.
    const int32 StepDirection = OmegaDiff > 0.0 ? 1 : -1;

    for (const double Multiplier : RoutePlannerDetail::ToFMultipliers)
    {
        // Номинальный TOF дуги для данного множителя.
        const double ToF = Multiplier * TParab;

        // Фаза в момент отбытия для конкретного TOF.
        const double Phase = ThetaB0 - ThetaA0 + OmegaB * ToF - UE_PI;

        // Первое окно около T0: floor по корню уравнения t_dep(k) = T0,
        // а skip ниже обрабатывает граничную ошибку.
        const int32 K0 = FMath::FloorToInt((T0 * OmegaDiff - Phase) / (2.0 * UE_PI));

        const double TSyn = 2.0 * UE_PI / FMath::Abs(OmegaDiff);
        for (int32 i = 0; i < RoutePlannerDetail::MaxPhasingWindows; ++i)
        {
            const int32 K = K0 + StepDirection * i;
            const double TDep0 = (Phase + 2.0 * UE_PI * K) / OmegaDiff;
            if (TDep0 < T0)
            {
                continue;
            }
            // Try window and small offsets to avoid near-antipodal branch instability
            const double SweepFracsArr[6] = {-0.015, -0.005, -0.002, 0.002, 0.005, 0.015};
            for (int32 s = 0; s < 6; ++s)
            {
                const double TDep = TDep0 + SweepFracsArr[s] * TSyn;
                if (TDep < T0)
                {
                    continue;
                }
                FManeuver Maneuver;
                if (ComputeLambertManeuver(FromBodyIndex, ToBodyIndex, TDep, ToF, Maneuver))
                {
                    Maneuver.DelaySeconds = TDep - T0;
                    OutCandidates.Add(Maneuver);
                }
            }
        }
    }
}

bool URoutePlanner::ComputeManeuver(int32 FromBodyIndex, int32 ToBodyIndex, FManeuver& OutManeuver, double DepartureTimeSeconds) const
{
    const bool bToBoundary = ToBodyIndex == BoundaryTargetIndex;
    const bool bPlanetToPlanet = !bToBoundary && FromBodyIndex > 0 && ToBodyIndex > 0;

    // Планетный участок с заданным временем отбытия: перебираем сетку TOF
    // и берём самый дешёвый допустимый вариант дуги Ламберта.
    if (bPlanetToPlanet && DepartureTimeSeconds > 0.0)
    {
        FManeuver Best;
        bool bFound = false;
        for (const double Multiplier : RoutePlannerDetail::ToFMultipliers)
        {
            const double ToF = LambertNominalToF(FromBodyIndex, ToBodyIndex, DepartureTimeSeconds, Multiplier);
            if (ToF > 0.0)
            {
                FManeuver Candidate;
                if (ComputeLambertManeuver(FromBodyIndex, ToBodyIndex, DepartureTimeSeconds, ToF, Candidate) &&
                    (!bFound || Candidate.DeltaV < Best.DeltaV))
                {
                    Best = Candidate;
                    bFound = true;
                }
                // Try nearby departure times if first attempt fails
                else
                {
                    const double Offsets[4] = {-3600.0, 3600.0, -7200.0, 7200.0};
                    for (int32 s = 0; s < 4; ++s)
                    {
                        const double TDep2 = DepartureTimeSeconds + Offsets[s];
                        if (TDep2 < 0.0) continue;
                        if (ComputeLambertManeuver(FromBodyIndex, ToBodyIndex, TDep2, ToF, Candidate))
                        {
                            Candidate.DelaySeconds += Offsets[s]; // adjust if needed
                            if (!bFound || Candidate.DeltaV < Best.DeltaV)
                            {
                                Best = Candidate;
                                bFound = true;
                            }
                            break;
                        }
                    }
                }
            }
        }

        if (bFound)
        {
            OutManeuver = Best;
            return true;
        }

        // Ламберт не дал решения: переходим к классическому Хохману без фазирования.
        return ComputeManeuverHohmann(FromBodyIndex, ToBodyIndex, OutManeuver);
    }

    return ComputeManeuverHohmann(FromBodyIndex, ToBodyIndex, OutManeuver);
}

void URoutePlanner::ApplyManeuver(const FManeuver& Maneuver, FSearchNode& Node) const
{
    Node.Maneuvers.Add(Maneuver);
    Node.DeltaV += Maneuver.DeltaV;

    // Полное время участка: ожидание окна фазирования (DelaySeconds)
    // плюс сам перелёт. У Хохмана-участков задержки нет.
    Node.FlightTime += Maneuver.DelaySeconds + Maneuver.DurationSeconds;
    Node.AbsoluteTimeSeconds += Maneuver.DelaySeconds + Maneuver.DurationSeconds;

    if (!Bodies.IsValidIndex(Maneuver.ToBodyIndex))
    {
        return;
    }

    // Ближайшее сближение с поверхностью планеты на маршруте.
    // Вместе с ним запоминаем радиус тела: критериям риска нужно знать
    // безопасную дистанцию, выраженную в радиусах именно этого тела.
    //
    // Условие >= 0.0, а не > 0.0: высота ровно 0 — это касание поверхности,
    // самый опасный вариант, и он не должен отбрасываться. Признаком того,
    // что сближения не было, служит отрицательная ApproachAltitudeKm.
    if (Maneuver.ApproachAltitudeKm >= 0.0 &&
        (Node.ClosestApproachKm < 0.0 || Maneuver.ApproachAltitudeKm < Node.ClosestApproachKm))
    {
        Node.ClosestApproachKm = Maneuver.ApproachAltitudeKm;
        Node.ClosestApproachBodyRadius = Maneuver.TargetBodyRadiusKm;
        Node.ClosestApproachRelativeVelocityKmS = Maneuver.ApproachRelativeVelocityKmS;
    }
}

FRouteEvaluationContext URoutePlanner::BuildContext(const FSearchNode& Node, TArrayView<const FManeuver> Maneuvers, bool bComplete) const
{
    FRouteEvaluationContext Context;

    Context.TotalDeltaV = Node.DeltaV;
    Context.TotalTimeSeconds = Node.FlightTime;
    Context.ClosestApproachKm = Node.ClosestApproachKm;
    Context.ClosestApproachBodyRadius = Node.ClosestApproachBodyRadius;
    Context.ClosestApproachRelativeVelocityKmS = Node.ClosestApproachRelativeVelocityKmS;
    Context.AvailableDeltaVKmS = UOrbitMath::MaxDeltaV(
        SpacecraftParams.DryMass, SpacecraftParams.FuelMass, SpacecraftParams.SpecificImpulse);
    Context.StartBodyIndex = StartBodyIndex;
    Context.ManeuverCount = Maneuvers.Num();
    Context.bReachesBoundary = Node.bReachedBoundary;
    Context.bComplete = bComplete;
    Context.Maneuvers = Maneuvers;

    return Context;
}

double URoutePlanner::ScoreNode(const FSearchNode& Node) const
{
    // Без критериев оценка не определена: используем суммарный расход топлива.
    if (Critera.IsEmpty())
    {
        return Node.DeltaV;
    }

    const FRouteEvaluationContext Context = BuildContext(Node, MakeArrayView(Node.Maneuvers), Node.bReachedBoundary);

    double TotalCost = 0.0;
    for (const TObjectPtr<URouteCriterion>& Criterion : Critera)
    {
        if (Criterion)
        {
            TotalCost += Criterion->Weight * Criterion->EvaluateRoute(Context);
        }
    }

    return TotalCost;
}

FRoutePlan URoutePlanner::BuildPlan(const FSearchNode& Node) const
{
    FRoutePlan Plan;
    Plan.bIsValid = true;
    Plan.VisitedBodyIndices = Node.VisitedBodies;
    Plan.TotalDeltaV = Node.DeltaV;
    Plan.TotalTime = Node.FlightTime;
    Plan.RequiredFuelMass = ComputeRequiredFuel(Node.DeltaV);

    // Последовательность манёвров уже накоплена в узле при построении.
    // Число манёвров равно числу посещённых тел: последнее тело всегда связано
    // с границей системы, обозначенной виртуальным индексом BoundaryTargetIndex.
    // Сам индекс границы в VisitedBodyIndices не входит — он добавляется
    // отдельно при подготовке активного маршрута.
    Plan.Maneuvers = Node.Maneuvers;

    if (Plan.Maneuvers.Num() != Node.VisitedBodies.Num())
    {
        // Узел собран непоследовательно: маршрут непригоден.
        Plan.bIsValid = false;
        Plan.Maneuvers.Reset();
        return Plan;
    }

    // Оценка маршрута набором критериев: каждый вклад сохраняем отдельно,
    // чтобы интерфейс мог показать, за счёт чего маршрут оказался лучше.
    const FRouteEvaluationContext Context = BuildContext(Node, MakeArrayView(Plan.Maneuvers), Node.bReachedBoundary);

    Plan.CriterionScores.Reserve(Critera.Num());
    for (const TObjectPtr<URouteCriterion>& Criterion : Critera)
    {
        if (!Criterion)
        {
            continue;
        }

        FCriterionScore Score;
        Score.CriterionName = Criterion->GetCriterionName();
        Score.Weight = Criterion->Weight;
        Score.Cost = Criterion->EvaluateRoute(Context);
        Score.WeightedCost = Score.Weight * Score.Cost;

        Plan.CriterionScores.Add(Score);
        Plan.TotalCost += Score.WeightedCost;
    }

    return Plan;
}

void URoutePlanner::EnsureCriteria()
{
    // Пустой набор критериев не оставляем: подставляем пресет "Balanced" из ТЗ.
    if (Critera.IsEmpty())
    {
        Critera = URouteCriterionPresets::MakeBalanced(this);
        return;
    }

    // Некорректные ссылки удаляем, чтобы планировщик не считал пустые критерии.
    Critera.RemoveAll([](const TObjectPtr<URouteCriterion>& Criterion)
    {
        return !IsValid(Criterion);
    });
}

TArray<FRoutePlan> URoutePlanner::GetCandidateRoutes() const
{
    TArray<FRoutePlan> Candidates;

    if (Bodies.Num() < 2 || SystemBoundaryKm <= 0.0)
    {
        return Candidates;
    }

    const int32 RootBodyIndex = FMath::Clamp(SpacecraftParams.StartingBodyIndex, 0, Bodies.Num() - 1);
    const double AvailableDeltaV = UOrbitMath::MaxDeltaV(SpacecraftParams.DryMass, SpacecraftParams.FuelMass,
        SpacecraftParams.SpecificImpulse);

    TArray<FSearchNode> Beam;
    FSearchNode Root;
    Root.CurrentBodyIndex = RootBodyIndex;
    Root.VisitedBodies.Add(RootBodyIndex);
    Beam.Add(Root);

    TSet<FString> SeenSignatures;
    int32 EvaluatedVariants = 0;

    const int32 MaxLegs = FMath::Clamp(PlannerLimits.MaxLegs, 1, 16);
    const int32 BeamWidth = FMath::Max(PlannerLimits.BeamWidth, 1);

    for (int32 Depth = 0; Depth < MaxLegs; ++Depth)
    {
        if (bCancelRequested.load(std::memory_order_relaxed))
        {
            break;
        }

        TArray<FSearchNode> NextBeam;

        for (const FSearchNode& Node : Beam)
        {
            // К границе системы можно уйти с любого тела: жёсткого ограничения нет.
            // Стоимость такого перелёта (и его риск из-за дальности) отсеивается
            // критериями, а не топологией.
            const bool bCanReachBoundary = Node.DeltaV < AvailableDeltaV;

            if (bCanReachBoundary)
            {
                // Вариант завершения маршрута: идти к границе системы.
                FManeuver ManeuverToBoundary;
                if (ComputeManeuver(Node.CurrentBodyIndex, BoundaryTargetIndex, ManeuverToBoundary))
                {
                    FSearchNode Candidate = Node;
                    ApplyManeuver(ManeuverToBoundary, Candidate);
                    Candidate.CurrentBodyIndex = BoundaryTargetIndex;
                    Candidate.bReachedBoundary = true;
                    ++EvaluatedVariants;

                    if (Candidate.DeltaV <= AvailableDeltaV)
                    {
                        const bool bWithinTimeLimit =
                            PlannerLimits.MaxFlightTime <= 0.0 || Candidate.FlightTime <= PlannerLimits.MaxFlightTime;

                        if (bWithinTimeLimit)
                        {
                            // Ключ подписи защищает от повторного добавления одного маршрута.
                            // С фазированием один набор тел может быть пройден за разное
                            // абсолютное время, поэтому подпись учитывает и время.
                            FString Signature;
                            for (const int32 BodyIndex : Candidate.VisitedBodies)
                            {
                                Signature += FString::FromInt(BodyIndex) + TEXT("-");
                            }
                            Signature += FString::Printf(TEXT("%.0f"), Candidate.AbsoluteTimeSeconds);

                            if (!SeenSignatures.Contains(Signature))
                            {
                                SeenSignatures.Add(Signature);
                                Candidates.Add(BuildPlan(Candidate));
                            }
                        }
                    }
                }
            }

            // Варианты продолжения через промежуточные планеты (гравитационные манёвры).
            if (Depth + 1 < MaxLegs)
            {
                for (int32 TargetIndex = 1; TargetIndex < Bodies.Num(); ++TargetIndex)
                {
                    if (bCancelRequested.load(std::memory_order_relaxed))
                    {
                        break;
                    }

                    if (TargetIndex == Node.CurrentBodyIndex || Node.VisitedBodies.Contains(TargetIndex))
                    {
                        continue;
                    }

                    // Переходы к внутренним планетам разрешены только если это явно разрешено.
                    const bool bInward = Bodies[TargetIndex].OrbitalRadius < Bodies[Node.CurrentBodyIndex].OrbitalRadius;
                    if (bInward && !PlannerLimits.bAllowInwardTransfers)
                    {
                        continue;
                    }

                    // Лег планета->планета: сетка окон фазирования с дугами Ламберта.
                    // Участки от звезды и к границе остаются на Хомане.
                    TArray<FManeuver> LegCandidates;
                    if (Node.CurrentBodyIndex > 0 && TargetIndex > 0)
                    {
                        GenerateLambertCandidates(Node, TargetIndex, LegCandidates);
                    }

                    // Если ламбертовская сетка не дала вариантов (вырожденная геометрия,
                    // синхронные орбиты) — остаётся классический Хоман без времени.
                    if (LegCandidates.IsEmpty())
                    {
                        FManeuver Hohmann;
                        if (ComputeManeuverHohmann(Node.CurrentBodyIndex, TargetIndex, Hohmann))
                        {
                            LegCandidates.Add(Hohmann);
                        }
                    }

                    for (const FManeuver& Maneuver : LegCandidates)
                    {
                        FSearchNode Candidate = Node;
                        ApplyManeuver(Maneuver, Candidate);
                        ++EvaluatedVariants;

                        // Отсечение по топливу и времени.
                        if (Candidate.DeltaV > AvailableDeltaV)
                        {
                            continue;
                        }

                        if (PlannerLimits.MaxFlightTime > 0.0 && Candidate.FlightTime > PlannerLimits.MaxFlightTime)
                        {
                            continue;
                        }

                        Candidate.CurrentBodyIndex = TargetIndex;
                        Candidate.VisitedBodies.Add(TargetIndex);
                        Candidate.Score = ScoreNode(Candidate);
                        NextBeam.Add(Candidate);
                    }
                }
            }
        }

        if (NextBeam.IsEmpty())
        {
            break;
        }

        // Отбор лучших узлов пучка по активному критерию.
        // Оценка кэширована в узле при его создании: пересчёт контекста на
        // каждое сравнение был основным расходом сортировки.
        NextBeam.Sort([](const FSearchNode& A, const FSearchNode& B)
        {
            return A.Score < B.Score;
        });

        if (NextBeam.Num() > BeamWidth)
        {
            NextBeam.SetNum(BeamWidth);
        }

        Beam = MoveTemp(NextBeam);
    }

    // Финальная сортировка найденных маршрутов по итоговой стоимости.
    Candidates.Sort([this](const FRoutePlan& A, const FRoutePlan& B)
    {
        if (!FMath::IsNearlyEqual(A.TotalCost, B.TotalCost))
        {
            return A.TotalCost < B.TotalCost;
        }

        // При равной стоимости предпочитаем более короткий маршрут.
        return A.Maneuvers.Num() < B.Maneuvers.Num();
    });

    for (FRoutePlan& Plan : Candidates)
    {
        Plan.EvaluatedVariants = EvaluatedVariants;
    }

    return Candidates;
}

FRoutePlan URoutePlanner::FindBestRoute() const
{
    FRoutePlan Best;
    const TArray<FRoutePlan> Candidates = GetCandidateRoutes();

    if (!Candidates.IsEmpty())
    {
        Best = Candidates[0];
    }

    return Best;
}

bool URoutePlanner::StartPlanning()
{
    EnsureCriteria();

    if (bPlanning.load(std::memory_order_relaxed))
    {
        return false;
    }

    // Сбрасываем результаты предыдущего поиска: они относятся к старым входным
    // данным и не должны попасть в UI до завершения нового поиска.
    LastCandidateRoutes.Reset();
    LastRoute = FRoutePlan();

    bCancelRequested.store(false, std::memory_order_relaxed);
    bPlanning.store(true, std::memory_order_relaxed);

    OnPlanningStarted.Broadcast();

    AsyncTask(ENamedThreads::AnyBackgroundThreadNormalTask, [this]()
    {
        RunPlanningAsync();
    });

    return true;
}

void URoutePlanner::CancelPlanning()
{
    bCancelRequested.store(true, std::memory_order_relaxed);
}

bool URoutePlanner::IsPlanning() const
{
    return bPlanning.load(std::memory_order_relaxed);
}

void URoutePlanner::SetCriteria(const TArray<URouteCriterion*>& NewCriteria)
{
    // Останавливаем текущий поиск до изменения критериев: фоновый поток не
    // должен читать массив, который пересобирается на игровом.
    if (IsPlanning())
    {
        CancelPlanning();
    }
    
    Critera.Reset();
    Critera.Reserve(NewCriteria.Num());

    for (URouteCriterion* Criterion : NewCriteria)
    {
        Critera.Add(Criterion);
    }
}

const TArray<URouteCriterion*>& URoutePlanner::GetCriteria() const
{
    return Critera;
}

TArray<FRoutePlan> URoutePlanner::GetLastCandidateRoutes() const
{
    return LastCandidateRoutes;
}

FRoutePlan URoutePlanner::GetLastRoute() const
{
    return LastRoute;
}

void URoutePlanner::RunPlanningAsync()
{
    // Тяжёлый поиск выполняется на фоновом потоке. Он читает неизменяемые
    // входные данные планировщика (тела, параметры аппарата, критерии),
    // поэтому контракт вызова требует не менять их до завершения поиска.
    TArray<FRoutePlan> Results = GetCandidateRoutes();

    const bool bWasCancelled = bCancelRequested.load(std::memory_order_relaxed);

    // Результаты записываются и делегаты транслируются на игровом потоке,
    // чтобы UI мог обновляться без дополнительной синхронизации.
    AsyncTask(ENamedThreads::GameThread, [this, Results = MoveTemp(Results), bWasCancelled]() mutable
    {
        bPlanning.store(false, std::memory_order_relaxed);

        if (bWasCancelled)
        {
            // Запрос отмены: частичный результат не публикуется, завершение
            // приходит с признаком неудачи, чтобы UI снял индикатор загрузки.
            bCancelRequested.store(false, std::memory_order_relaxed);
            LastCandidateRoutes.Reset();
            LastRoute = FRoutePlan();
            OnPlanningCompleted.Broadcast(false);
            return;
        }

        LastCandidateRoutes = MoveTemp(Results);
        if (!LastCandidateRoutes.IsEmpty())
        {
            LastRoute = LastCandidateRoutes[0];
        }

        OnPlanningCompleted.Broadcast(!LastCandidateRoutes.IsEmpty());
    });
}