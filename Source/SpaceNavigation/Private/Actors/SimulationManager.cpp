// Copyright Epic Games, Inc. All Rights Reserved.

#include "Actors/SimulationManager.h"
#include "Core/OrbitMath.h"
#include "Core/RoutePlanner.h"
#include "Core/RouteCriterion.h"
#include "DrawDebugHelpers.h"
#include "GameFramework/Actor.h"
#include "Core/SimTypes.h"
#include "Actors/CelestialBody.h"
#include "Actors/Spacecraft.h"
#include "Actors/ScenarioGenerator.h"
#include "Engine/World.h"

DECLARE_LOG_CATEGORY_CLASS(LogSimulationManager, Log, All);

ASimulationManager::ASimulationManager()
{
    PrimaryActorTick.bCanEverTick = true;
}

void ASimulationManager::BeginPlay()
{
    Super::BeginPlay();

    if (!AllPropertiesIsValid()) return;

    if (!Spacecraft)
    {
        FActorSpawnParameters SpawnParameters;
        SpawnParameters.Owner = this;
        Spacecraft = GetWorld()->SpawnActor<ASpacecraft>(SpacecraftClass, SpawnParameters);
    }

    GenerateScenario();
    PlanRoute();
}

void ASimulationManager::Tick(float DeltaTime)
{
    Super::Tick(DeltaTime);

    if (SimState == ESimState::Simulating)
    {
        // Ускорение времени: игровое время масштабируется множителем.
        StepSimulation(DeltaTime * TimeScale);
    }

    UpdateBodiesTransforms();
    UpdateSpacecraftTransform();

    if (bDrawDebug)
    {
        DrawDebugVisualization();
    }
}

void ASimulationManager::EndPlay(const EEndPlayReason::Type EndPlayReason)
{
    Super::EndPlay(EndPlayReason);
}

void ASimulationManager::GenerateScenario()
{
    SimState = ESimState::Idle;
    Outcome = ESimOutcome::None;
    SimulationTime = 0.0;
    TrajectoryHistory.Reset();
    Logs.Reset();
    CurrentManeuverIndex = 0;
    CurrentManeuverElapsed = 0.0;
    bDepartureImpulseFired = false;
    ActiveRoute.Reset();

    if (!ScenarioGenerator)
    {
        return;
    }

    ScenarioGenerator->GenerateScenario();

    SystemBoundary = ScenarioGenerator->GetSystemBoundary();

    // Уничтожаем ранее созданные тела, чтобы сцена пересобиралась без накопления.
    for (TObjectPtr<ACelestialBody>& Body : Bodies)
    {
        if (IsValid(Body))
        {
            Body->Destroy();
        }
    }
    Bodies.Reset();

    // Создаём тела системы: Bodies[0] — звезда.
    FActorSpawnParameters SpawnParameters;
    SpawnParameters.Owner = this;

    const FCelestialBodyData StarData = ScenarioGenerator->StarData;

    ACelestialBody* Star = GetWorld()->SpawnActor<AStarBody>(StarClass, SpawnParameters);
    Star->Initialize(StarData, this);
    Bodies.Add(Star);

    for (FCelestialBodyData& BodyData : ScenarioGenerator->PlanetData)
    {
        BodyData.OrbitalVelocity = UOrbitMath::CircularOrbitalVelocity(StarData.Mass, BodyData.OrbitalRadius);
        ACelestialBody* Planet = GetWorld()->SpawnActor<ACelestialBody>(CelestialBodyClass, SpawnParameters);
        Planet->Initialize(BodyData, this);
        Bodies.Add(Planet);
    }

    AddLog(FString::Printf(
        TEXT("Сценарий сгенерирован: seed=%d, планет=%d, граница=%.0f км"),
        ScenarioGenerator->Params.RandomSeed,
        ScenarioGenerator->PlanetData.Num(),
        SystemBoundary));
}

void ASimulationManager::PlanRoute()
{
    RoutePlan = FRoutePlan();
    CandidateRoutes.Reset();

    // Критерии проверяются здесь, а не только в SetCriteria: так любой путь
    // к планированию гарантированно работает с непустым набором критериев.
    EnsureCriteria();

    if (Bodies.Num() < 2 || !Spacecraft)
    {
        return;
    }

    // Планировщик работает на данных тел, а не на Actor'ах.
    TArray<FCelestialBodyData> BodyData;
    BodyData.Reserve(Bodies.Num());
    for (const TObjectPtr<ACelestialBody>& Body : Bodies)
    {
        BodyData.Add(Body->BodyData);
    }

    // Ограничение по максимальному времени полёта из настроек менеджера.
    FPlannerLimits Limits = PlannerLimits;
    if (MaxFlightTime > 0.0)
    {
        Limits.MaxFlightTime = MaxFlightTime;
    }

    // Критерии живут на менеджере как UPROPERTY; планировщику передаём ссылки.
    TArray<URouteCriterion*> ActiveCriteria;
    ActiveCriteria.Reserve(Criteria.Num());
    for (const TObjectPtr<URouteCriterion>& Criterion : Criteria)
    {
        if (Criterion)
        {
            ActiveCriteria.Add(Criterion);
        }
    }

    URoutePlanner* Planner = NewObject<URoutePlanner>(this);
    Planner->Initialize(BodyData, Spacecraft->Params, SystemBoundary, ActiveCriteria, Limits);

    CandidateRoutes = Planner->GetCandidateRoutes();

    if (!CandidateRoutes.IsEmpty())
    {
        RoutePlan = CandidateRoutes[0];
        SimState = ESimState::Idle;

        Planner->AddLog(FString::Printf(
            TEXT("Маршрут спланирован: манёвров=%d, DeltaV=%.1f км/с, время=%.0f с, стоимость=%.3f"),
            RoutePlan.Maneuvers.Num(),
            RoutePlan.TotalDeltaV,
            RoutePlan.TotalTime,
            RoutePlan.TotalCost));

        // Разбивка стоимости по критериям: показывает, за счёт чего выбран маршрут.
        for (const FCriterionScore& Score : RoutePlan.CriterionScores)
        {
            Planner->AddLog(FString::Printf(
                TEXT("  %s: стоимость=%.3f, вес=%.2f, вклад=%.3f"),
                *Score.CriterionName, Score.Cost, Score.Weight, Score.WeightedCost));
        }
    }
    else
    {
        Planner->AddLog(TEXT("Допустимый маршрут не найден"));
    }
}

void ASimulationManager::StartSimulation()
{
    if (RoutePlan.Maneuvers.IsEmpty() || !Spacecraft)
    {
        return;
    }

    SimState = ESimState::Simulating;
    Outcome = ESimOutcome::None;
    SimulationTime = 0.0;
    CurrentManeuverIndex = 0;
    CurrentManeuverElapsed = 0.0;
    TrajectoryHistory.Reset();
    PrepareActiveRoute();

    ACelestialBody* Star = Bodies.IsValidIndex(0) ? Bodies[0].Get() : nullptr;
    const int32 StartBodyIndex = FMath::Clamp(Spacecraft->Params.StartingBodyIndex, 0, Bodies.Num() - 1);
    ACelestialBody* StartBody = Bodies[StartBodyIndex].Get();

    Spacecraft->InitializeSpacecraft(StartBody, Star, SimulationTime);

    UpdateBodiesTransforms();
    UpdateSpacecraftTransform();
    TrajectoryHistory.Add(Spacecraft->AbsolutePositionKm);

    // Первый манёвр стартует в окно фазирования: импульс отхода применяем,
    // когда истечёт DelaySeconds участка (нулевой — сразу на первом шаге).
    bDepartureImpulseFired = false;

    AddLog(TEXT("Симуляция запущена"));
}

void ASimulationManager::StopSimulation()
{
    if (SimState == ESimState::Simulating)
    {
        SimState = ESimState::Idle;
    }
}

void ASimulationManager::ResetSimulation()
{
    SimState = ESimState::Idle;
    Outcome = ESimOutcome::None;
    SimulationTime = 0.0;
    CurrentManeuverIndex = 0;
    CurrentManeuverElapsed = 0.0;
    bDepartureImpulseFired = false;
    TrajectoryHistory.Reset();
    Logs.Reset();

    if (Spacecraft)
    {
        Spacecraft->ResetState();
    }
}

void ASimulationManager::SetTimeScale(double NewTimeScale)
{
    TimeScale = FMath::Max(NewTimeScale, 0.0);
}

void ASimulationManager::SetCriteria(const TArray<URouteCriterion*>& NewCriteria)
{
    Criteria.Reset();
    Criteria.Reserve(NewCriteria.Num());

    for (URouteCriterion* Criterion : NewCriteria)
    {
        Criteria.Add(Criterion);
    }

    PlanRoute();
}

void ASimulationManager::SetCriterionPreset(const FString& PresetName)
{
    Criteria = URouteCriterionPresets::MakeByPresetName(this, PresetName);
    PlanRoute();
}

void ASimulationManager::EnsureCriteria()
{
    // Пустой набор критериев не оставляем: подставляем пресет "Balanced" из ТЗ.
    if (Criteria.IsEmpty())
    {
        Criteria = URouteCriterionPresets::MakeBalanced(this);
        return;
    }

    // Некорректные ссылки удаляем, чтобы планировщик не считал пустые критерии.
    Criteria.RemoveAll([](const TObjectPtr<URouteCriterion>& Criterion)
    {
        return !IsValid(Criterion);
    });
}

ACelestialBody* ASimulationManager::FindCentralBodyBySOI() const
{
    if (Bodies.IsEmpty())
    {
        return nullptr;
    }

    const ACelestialBody* Star = Bodies[0].Get();
    const FVector AbsolutePosition = Spacecraft->AbsolutePositionKm;

    ACelestialBody* BestBody = nullptr;
    double BestDistanceSquared = 0.0;

    for (int32 Index = 0; Index < Bodies.Num(); ++Index)
    {
        const ACelestialBody* Body = Bodies[Index].Get();
        if (!Body)
        {
            continue;
        }

        const double DistanceSquared = FVector::DistSquared(AbsolutePosition, Body->PositionKm);
        const double SOI = Body->GetSOIKm();

        // Внутри сферы влияния тело может стать центральным.
        if (SOI <= 0.0 || DistanceSquared > SOI * SOI)
        {
            continue;
        }

        // При пересечении сфер влияния ближайшее тело становится центральным.
        if (!BestBody || DistanceSquared < BestDistanceSquared)
        {
            BestBody = const_cast<ACelestialBody*>(Body);
            BestDistanceSquared = DistanceSquared;
        }
    }

    // Вне сфер влияния планет аппарат обращается вокруг звезды.
    if (!BestBody && Star)
    {
        BestBody = const_cast<ACelestialBody*>(Star);
    }

    return BestBody;
}

void ASimulationManager::SwitchCentralBody(ACelestialBody* NewCentralBody)
{
    if (!Spacecraft || !NewCentralBody || NewCentralBody == Spacecraft->CurrentCentralBody)
    {
        return;
    }

    // Сохраняем абсолютное состояние и пересчитываем относительное относительно нового центра.
    const FVector SavedAbsolutePosition = Spacecraft->AbsolutePositionKm;
    const FVector SavedAbsoluteVelocity = Spacecraft->AbsoluteVelocityKmS;

    Spacecraft->CurrentCentralBody = NewCentralBody;
    Spacecraft->RelativePositionKm = SavedAbsolutePosition - NewCentralBody->PositionKm;
    Spacecraft->RelativeVelocityKmS = SavedAbsoluteVelocity - NewCentralBody->VelocityKmS;

    AddLog(FString::Printf(
        TEXT("Переключение центрального тела: %s (r=%.0f км)"),
        *NewCentralBody->BodyData.BodyName.ToString(),
        Spacecraft->GetDistanceToCentralBody()));
}

void ASimulationManager::PrepareActiveRoute()
{
    ActiveRoute.Reset();
    CurrentManeuverIndex = 0;
    CurrentManeuverElapsed = 0.0;

    if (RoutePlan.Maneuvers.IsEmpty())
    {
        return;
    }

    // Маршрут: посещённые тела и граница системы на конце.
    ActiveRoute.Append(RoutePlan.VisitedBodyIndices);
    ActiveRoute.Add(-1);
}

bool ASimulationManager::IsFinalManeuver() const
{
    return CurrentManeuverIndex >= RoutePlan.Maneuvers.Num() - 1;
}

void ASimulationManager::ApplyManeuverImpulse()
{
    if (!Spacecraft || !RoutePlan.Maneuvers.IsValidIndex(CurrentManeuverIndex))
    {
        return;
    }

    const FManeuver& Maneuver = RoutePlan.Maneuvers[CurrentManeuverIndex];

    if (Maneuver.DeltaVDeparture <= 0.0)
    {
        return;
    }

    const ACelestialBody* FromBody = Bodies.IsValidIndex(Maneuver.FromBodyIndex) ? Bodies[Maneuver.FromBodyIndex].Get() : nullptr;
    if (!FromBody)
    {
        return;
    }

    // Импульс отхода направлен по орбитальному движению тела (prograde) для
    // перелёта наружу и против него — внутрь. Тогда гелиоцентрическая скорость
    // остаётся касательной, а избыточная скорость совпадает по направлению с
    // движением планеты (или противоположна ему).
    FVector ProgradeDirection = FromBody ? FromBody->VelocityKmS.GetSafeNormal() : FVector::ZeroVector;
    if (ProgradeDirection.IsNearlyZero())
    {
        // Звезда или неподвижное тело: опираемся на текущую орбиту аппарата.
        ProgradeDirection = Spacecraft->RelativeVelocityKmS.GetSafeNormal();
    }
    if (ProgradeDirection.IsNearlyZero())
    {
        return;
    }

    // Целевая скорость аппарата относительно центрального тела после импульса.
    FVector DesiredRelativeVelocity;
    if (FromBody && FromBody->IsA(AStarBody::StaticClass()))
    {
        // Старт со звезды: чистый гелиоцентрический импульс перехода.
        const FHohmannTransfer Transfer = UOrbitMath::ComputeHohmannTransfer(
            Bodies[0]->BodyData.Mass, Maneuver.StartRadius, Maneuver.EndRadius);
        DesiredRelativeVelocity = ProgradeDirection * Transfer.TransferVelocityStart;
    }
    else
    {
        // Покидание сферы влияния планеты. Планировщик задаёт приращение
        // DeltaVDeparture для перехода на гиперболу с избыточной скоростью
        // excess = sqrt(hyper^2 - vesc^2).
        const double BodyMu = SimConstants::GravitationalConstant * FromBody->BodyData.Mass;
        const double ParkingRadius = FMath::Max(FromBody->BodyData.Radius + Spacecraft->Params.StartingOrbitAltitude, 1.0);
        const double CircularSpeed = UOrbitMath::CircularOrbitalVelocity(FromBody->BodyData.Mass, ParkingRadius);
        const double HyperbolicSpeed = Maneuver.DeltaVDeparture + CircularSpeed;

        // Ламберт-участок (планета→планета): импульс направляем по избыточной
        // скорости вне сферы влияния û = (v1 − V_A)/|v1 − V_A|. Направление
        // задано планетоцентрически, поэтому применимо в системе центрального тела.
        const FVector ExcessDir = !Maneuver.DepartureVelocityKmS.IsNearlyZero()
                                      ? (Maneuver.DepartureVelocityKmS - FromBody->VelocityKmS).GetSafeNormal()
                                      : FVector::ZeroVector;

        if (!ExcessDir.IsNearlyZero())
        {
            DesiredRelativeVelocity = ExcessDir * HyperbolicSpeed;
        }
        else
        {
            // Хохман-участок: импульс строго вдоль движения даёт асимптоту
            // гиперболы, повёрнутую на угол убегания от направления движения,
            // из-за чего теряется часть гелиоцентрической скорости и аппарат
            // не дотягивает до цели. Компенсируем наклон: подбираем направление
            // импульса так, чтобы асимптота совпала с орбитальным движением
            // тела (для перелёта наружу) или была противоположна (внутрь).
            const double ExcessSpeed = FMath::Sqrt(FMath::Max(
                HyperbolicSpeed * HyperbolicSpeed - 2.0 * BodyMu / ParkingRadius, 0.0));

            // Решаем cos(phi) * (1 - A * sin(phi)) = K на [0; 1.2] методом бисекции.
            // phi — наклон импульса от направления движения в сторону от планеты.
            const double A = ExcessSpeed * ParkingRadius * HyperbolicSpeed / BodyMu;
            const double K = BodyMu / (2.0 * BodyMu + ParkingRadius * ExcessSpeed * ExcessSpeed);
            double Low = 0.0;
            double High = 1.2;
            for (int32 Iter = 0; Iter < 40; ++Iter)
            {
                const double Mid = 0.5 * (Low + High);
                const double Value = FMath::Cos(Mid) * (1.0 - A * FMath::Sin(Mid)) - K;
                if (Value > 0.0)
                {
                    Low = Mid;
                }
                else
                {
                    High = Mid;
                }
            }
            const double BurnAngle = 0.5 * (Low + High);

            const double DirectionSign = (Maneuver.EndRadius >= Maneuver.StartRadius) ? 1.0 : -1.0;
            const FVector RadialDirection = Spacecraft->RelativePositionKm.GetSafeNormal();

            DesiredRelativeVelocity = (ProgradeDirection * (DirectionSign * FMath::Cos(BurnAngle))
                                       + RadialDirection * FMath::Sin(BurnAngle)) * HyperbolicSpeed;
        }
    }

    const FVector DeltaVVelocity = DesiredRelativeVelocity - Spacecraft->RelativeVelocityKmS;
    const FVector PreBurnRelativeVelocity = Spacecraft->RelativeVelocityKmS;
    const FVector PreBurnRelativePosition = Spacecraft->RelativePositionKm;
    const double AppliedDeltaV = Spacecraft->ApplyImpulse(DeltaVVelocity.Size(), DeltaVVelocity.GetSafeNormal());

    UE_LOG(LogSimulationManager, Log,
        TEXT(
            "[Sim] Burn telemetry: t=%.3e s, central=%s, relPos=(%.3f,%.3f,%.3f) |relPos|=%.3f, relVel=(%.6f,%.6f,%.6f) |relVel|=%.4f, desired=(%.4f,%.4f,%.4f) |desired|=%.4f, delta=|%.4f| applied=%.4f"
        ),
        SimulationTime,
        Spacecraft->CurrentCentralBody ? *Spacecraft->CurrentCentralBody->BodyData.BodyName.ToString() : TEXT("?"),
        PreBurnRelativePosition.X, PreBurnRelativePosition.Y, PreBurnRelativePosition.Z,
        PreBurnRelativePosition.Size(),
        PreBurnRelativeVelocity.X, PreBurnRelativeVelocity.Y, PreBurnRelativeVelocity.Z,
        PreBurnRelativeVelocity.Size(),
        DesiredRelativeVelocity.X, DesiredRelativeVelocity.Y, DesiredRelativeVelocity.Z,
        DesiredRelativeVelocity.Size(),
        DeltaVVelocity.Size(), AppliedDeltaV);

    Spacecraft->UpdateAbsoluteState(Bodies.IsValidIndex(0) ? Bodies[0].Get() : nullptr);

    AddLog(FString::Printf(
        TEXT("Манёвр %d: импульс отхода %.1f км/с (применено %.1f км/с)"),
        CurrentManeuverIndex + 1,
        Maneuver.DeltaVDeparture,
        AppliedDeltaV));

    if (AppliedDeltaV + 1.0e-6 < DeltaVVelocity.Size())
    {
        AddLog(TEXT("Топлива не хватило: импульс ограничен запасом"));
    }
}

void ASimulationManager::AdvanceToNextManeuver()
{
    if (!RoutePlan.Maneuvers.IsValidIndex(CurrentManeuverIndex))
    {
        return;
    }

    const FManeuver& Maneuver = RoutePlan.Maneuvers[CurrentManeuverIndex];

    // Gate by actual proximity to target body's SOI to avoid fake captures
    if (Maneuver.ToBodyIndex >= 0 && Bodies.IsValidIndex(Maneuver.ToBodyIndex) && Spacecraft)
    {
        if (ACelestialBody* ToBody = Bodies[Maneuver.ToBodyIndex].Get())
        {
            const double Dist = FVector::Dist(Spacecraft->AbsolutePositionKm, ToBody->PositionKm);
            const double SOI = ToBody->GetSOIKm();
            if (Dist > SOI + 5.0e3)
            {
                AddLog(FString::Printf(
                    TEXT("Манёвр %d: прибытие к %s за пределами SOI (dist=%.3e км, SOI=%.3e км) — пропускаем захват по плану, продолжаем полёт"),
                    CurrentManeuverIndex + 1, *ToBody->BodyData.BodyName.ToString(), Dist, SOI));
                // Don't abort - just skip capture; let simulation continue
            }
            if (ToBody != Spacecraft->CurrentCentralBody)
            {
                SwitchCentralBody(ToBody);
            }
            if (Maneuver.DeltaVArrival > 0.0)
            {
                const FVector RetrogradeDirection = -Spacecraft->RelativeVelocityKmS.GetSafeNormal();
                const double AppliedDeltaV = Spacecraft->ApplyImpulse(Maneuver.DeltaVArrival, RetrogradeDirection);
                Spacecraft->UpdateAbsoluteState(Bodies.IsValidIndex(0) ? Bodies[0].Get() : nullptr);
                AddLog(FString::Printf(
                    TEXT("Манёвр %d: импульс захвата %.1f км/с (применено %.1f км/с)"),
                    CurrentManeuverIndex + 1,
                    Maneuver.DeltaVArrival,
                    AppliedDeltaV));
            }
        }
    }

    ++CurrentManeuverIndex;
    CurrentManeuverElapsed = 0.0;

    // Последний манёвр ведёт к границе системы: дальше переходов нет.
    if (CurrentManeuverIndex < RoutePlan.Maneuvers.Num())
    {
        AddLog(FString::Printf(TEXT("Переход к манёвру %d"), CurrentManeuverIndex + 1));
    }
    else
    {
        AddLog(TEXT("Маршрут выполнен, выполняется добирание до границы системы"));
    }
}

void ASimulationManager::StepSimulation(double DeltaTimeSeconds)
{
    if (SimState != ESimState::Simulating || !Spacecraft || Bodies.IsEmpty())
    {
        return;
    }

    if (DeltaTimeSeconds <= 0.0)
    {
        return;
    }

    // Ограничиваем шаг, чтобы не пропустить пересечение сферы влияния.
    double RemainingTime = DeltaTimeSeconds;
    const double MaxStep = FMath::Max(MaxIntegrationStep, 1.0);
    int32 SafetyIterations = 0;

    while (RemainingTime > 0.0 && SafetyIterations < 64)
    {
        const double Step = FMath::Min(RemainingTime, MaxStep);
        RemainingTime -= Step;
        ++SafetyIterations;

        SimulationTime += Step;
        CurrentManeuverElapsed += Step;

        // Обработка текущего манёвра. Импульс отхода выполняем ДО первого шага
        // интегрирования, пока аппарат находится в исходной точке парковочной
        // орбиты, где его скорость сонаправлена гелиоцентрической скорости
        // планеты. Если стрелять после шага, аппарат успевает сместиться по
        // орбите, и раскадровка желаемой скорости относительно текущего вектора
        // разрушается (импульс получается почти ретроградным).
        if (RoutePlan.Maneuvers.IsValidIndex(CurrentManeuverIndex))
        {
            const FManeuver& Maneuver = RoutePlan.Maneuvers[CurrentManeuverIndex];

            if (!bDepartureImpulseFired && CurrentManeuverElapsed >= Maneuver.DelaySeconds)
            {
                ApplyManeuverImpulse();
                bDepartureImpulseFired = true;
            }
        }

        // Двигаем тела до текущего времени симуляции.
        for (const TObjectPtr<ACelestialBody>& Body : Bodies)
        {
            if (Body)
            {
                Body->UpdatePosition(SimulationTime);
            }
        }

        Spacecraft->IntegrateStep(Step);
        Spacecraft->UpdateAbsoluteState(Bodies[0].Get());

        CurrentDistanceFromStar = Spacecraft->GetDistanceFromStar(Bodies[0].Get());
        if (SystemBoundary > 0.0)
        {
            ProgressPercent = FMath::Clamp(CurrentDistanceFromStar / SystemBoundary * 100.0, 0.0, 100.0);
        }

        // Проверка столкновения и границ системы.
        if (CheckCollision())
        {
            FinishSimulation(ESimState::Failed, ESimOutcome::Collision);
            return;
        }

        if (CheckBoundaryReached())
        {
            if (bAutoStopOnCompletion)
            {
                FinishSimulation(ESimState::Success, ESimOutcome::BoundaryReached);
            }
            return;
        }

        if (CheckTimeLimit())
        {
            FinishSimulation(ESimState::Failed, ESimOutcome::TimeLimitExceeded);
            return;
        }

        if (CheckFuelExhausted())
        {
            FinishSimulation(ESimState::Failed, ESimOutcome::FuelExhausted);
            return;
        }

        // Определяем, не сменилось ли центральное тело при пересечении сфер влияния.
        if (ACelestialBody* NewCentralBody = FindCentralBodyBySOI())
        {
            if (NewCentralBody != Spacecraft->CurrentCentralBody)
            {
                SwitchCentralBody(NewCentralBody);
            }
        }

        // Завершение манёвра по полному времени участка: ожидание окна + перелёт.
        if (RoutePlan.Maneuvers.IsValidIndex(CurrentManeuverIndex))
        {
            const FManeuver& Maneuver = RoutePlan.Maneuvers[CurrentManeuverIndex];

            const double ManeuverDuration = Maneuver.DelaySeconds + Maneuver.DurationSeconds;
            if (CurrentManeuverElapsed >= ManeuverDuration)
            {
                if (IsFinalManeuver())
                {
                    // Последний манёвр — перелёт к границе системы. Дальше только добирание.
                    CurrentManeuverIndex = RoutePlan.Maneuvers.Num();
                    CurrentManeuverElapsed = 0.0;
                    AddLog(TEXT("Добирание до границы системы"));
                }
                else
                {
                    AdvanceToNextManeuver();
                    bDepartureImpulseFired = false;
                }
            }
        }
    }

    Spacecraft->FlightTimeSeconds = SimulationTime;

    // Сохраняем точку траектории для отрисовки.
    if (TrajectoryHistory.IsEmpty() || FVector::DistSquared(TrajectoryHistory.Last(), Spacecraft->AbsolutePositionKm) >
        FMath::Square(1.0e7))
    {
        TrajectoryHistory.Add(Spacecraft->AbsolutePositionKm);
    }
}

bool ASimulationManager::CheckCollision() const
{
    if (!Spacecraft)
    {
        return false;
    }

    for (const TObjectPtr<ACelestialBody>& Body : Bodies)
    {
        if (!Body)
        {
            continue;
        }

        const double Distance = FVector::Dist(Spacecraft->AbsolutePositionKm, Body->PositionKm);
        if (Distance <= Body->BodyData.Radius)
        {
            UE_LOG(LogSimulationManager, Error,
                TEXT("[Sim] COLLISION with '%s' at t=%.3e s: dist=%.1f km, radius=%.1f km, penetration=%.1f km "
                    "(spacecraft pos %.1f,%.1f,%.1f; body pos %.1f,%.1f,%.1f)"),
                *Body->GetName(),
                SimulationTime,
                Distance,
                Body->BodyData.Radius,
                Body->BodyData.Radius - Distance,
                Spacecraft->AbsolutePositionKm.X,
                Spacecraft->AbsolutePositionKm.Y,
                Spacecraft->AbsolutePositionKm.Z,
                Body->PositionKm.X,
                Body->PositionKm.Y,
                Body->PositionKm.Z);
            return true;
        }
    }

    return false;
}

bool ASimulationManager::CheckBoundaryReached() const
{
    if (!Spacecraft || SystemBoundary <= 0.0)
    {
        return false;
    }

    return CurrentDistanceFromStar >= SystemBoundary;
}

bool ASimulationManager::CheckTimeLimit() const
{
    if (MaxFlightTime <= 0.0)
    {
        return false;
    }

    return SimulationTime > MaxFlightTime;
}

bool ASimulationManager::CheckFuelExhausted() const
{
    if (!Spacecraft)
    {
        return false;
    }

    // Топливо закончено и граница ещё не достигнута — миссия провалена.
    return Spacecraft->FuelMassKg <= 0.0 && !CheckBoundaryReached();
}

void ASimulationManager::FinishSimulation(ESimState FinalState, ESimOutcome Reason)
{
    SimState = FinalState;
    Outcome = Reason;

    if (TrajectoryHistory.IsEmpty() == false)
    {
        TrajectoryHistory.Add(Spacecraft->AbsolutePositionKm);
    }

    AddLog(FString::Printf(
        TEXT("Симуляция завершена: состояние=%d, причина=%d, время=%.0f с, DeltaV=%.1f км/с, расстояние=%.0f км"),
        static_cast<int32>(FinalState),
        static_cast<int32>(Reason),
        SimulationTime,
        Spacecraft->TotalDeltaVSpentKmS,
        CurrentDistanceFromStar));
}

void ASimulationManager::UpdateBodiesTransforms()
{
    if (KmPerWorldUnit <= 0.0)
    {
        return;
    }

    for (const TObjectPtr<ACelestialBody>& Body : Bodies)
    {
        if (Body)
        {
            Body->UpdatePosition(SimulationTime);
            Body->SyncTransform(KmPerWorldUnit);
        }
    }
}

void ASimulationManager::UpdateSpacecraftTransform()
{
    if (!Spacecraft || KmPerWorldUnit <= 0.0)
    {
        return;
    }

    Spacecraft->SetActorLocation(Spacecraft->AbsolutePositionKm / KmPerWorldUnit);
}

void ASimulationManager::DrawDebugVisualization()
{
    if (!GetWorld() || KmPerWorldUnit <= 0.0)
    {
        return;
    }

    const double Scale = 1.0 / KmPerWorldUnit;
    const ACelestialBody* Star = Bodies.IsValidIndex(0) ? Bodies[0].Get() : nullptr;

    // Орбиты планет.
    //
    // Окружность рисуется в плоскости самой орбиты: DrawDebugCircle строит
    // точку как Center + R * (YAxis * cos + ZAxis * sin), поэтому ему и
    // передаются базисные оси плоскости. Иначе все окружности лежали бы в
    // мировой плоскости YZ, а планеты двигались бы в наклонённых плоскостях
    // мимо нарисованных орбит.
    //
    // bDrawAxis=false отключает встроенную "крестовину" осей: по умолчанию
    // DrawDebugCircle добавляет её и рисовал бы одинаковые оси вдоль мировых
    // Y и Z у каждого тела.
    for (const TObjectPtr<ACelestialBody>& Body : Bodies)
    {
        if (!Body || Body->IsA(AStarBody::StaticClass()) || Body->BodyData.OrbitalRadius <= 0.0)
        {
            continue;
        }

        FVector AxisU, AxisV;
        Body->GetOrbitPlaneAxes(AxisU, AxisV);

        const FVector Center = (Star ? Star->PositionKm : FVector::ZeroVector) * Scale;
        const double RadiusWorld = Body->BodyData.OrbitalRadius * Scale;

        DrawDebugCircle(GetWorld(), Center, RadiusWorld, 64, Body->DebugColor, false, -1.0f, 0, 1.5f, AxisU, AxisV, false);

        // Сфера влияния: сечение в той же плоскости орбиты.
        const double SOIWorld = Body->GetSOIKm() * Scale;
        if (SOIWorld > 0.0)
        {
            DrawDebugCircle(GetWorld(), Body->PositionKm * Scale, SOIWorld, 32, SOIColor, false, -1.0f, 0, 1.0f, AxisU, AxisV, false);
        }

        // Стрелка направления скорости: куда планета движется в данный момент.
        const FVector BodyPosition = Body->PositionKm * Scale;
        const FVector VelocityDirection = Body->VelocityKmS.GetSafeNormal();
        const float ArrowLength = static_cast<float>(DebugArrowLength);
        const float ArrowSize = FMath::Max(ArrowLength * 0.15f, 1.0f);

        if (!VelocityDirection.IsNearlyZero())
        {
            DrawDebugDirectionalArrow(
                GetWorld(),
                BodyPosition,
                BodyPosition + VelocityDirection * ArrowLength,
                ArrowSize,
                TrajectoryColor,
                false,
                -1.0f,
                0,
                2.0f);
        }

        // Стрелка оси вращения: нормаль плоскости орбиты в точке планеты.
        // Вместе со стрелкой скорости показывает и плоскость, и направление
        // обхода — то, что раньше не показывала крестовина мировых осей.
        const FVector OrbitNormal = Body->GetOrbitPlaneNormal();
        DrawDebugDirectionalArrow(
            GetWorld(),
            BodyPosition,
            BodyPosition + OrbitNormal * ArrowLength,
            ArrowSize,
            Body->DebugColor,
            false,
            -1.0f,
            0,
            2.0f);
    }

    // Граница системы строится в базовой плоскости системы (XY).
    if (Star && SystemBoundary > 0.0)
    {
        DrawDebugCircle(GetWorld(), Star->PositionKm * Scale, SystemBoundary * Scale, 96, FColor::Orange, false, -1.0f, 0, 2.0f,
            FVector::ForwardVector, FVector::RightVector, false);
    }

    // Пройденная траектория аппарата.
    if (TrajectoryHistory.Num() >= 2)
    {
        for (int32 Index = 1; Index < TrajectoryHistory.Num(); ++Index)
        {
            DrawDebugLine(
                GetWorld(),
                TrajectoryHistory[Index - 1] * Scale,
                TrajectoryHistory[Index] * Scale,
                TrajectoryColor,
                false,
                -1.0f,
                0,
                2.0f);
        }
    }

    // Текущее положение аппарата.
    if (Spacecraft)
    {
        DrawDebugSphere(GetWorld(), Spacecraft->AbsolutePositionKm * Scale, static_cast<float>(DebugSpacecraftRadius), 12, TrajectoryColor,
            false, -1.0f, 0, 2.0f);
    }
}

FString ASimulationManager::GetStateSummary() const
{
    return FString::Printf(
        TEXT("Состояние: %d | Исход: %d | Время: %.0f с | Расстояние: %.0f / %.0f км (%.1f%%) | DeltaV: %.1f км/с | Топливо: %.0f кг"),
        static_cast<int32>(SimState),
        static_cast<int32>(Outcome),
        SimulationTime,
        CurrentDistanceFromStar,
        SystemBoundary,
        ProgressPercent,
        Spacecraft ? Spacecraft->TotalDeltaVSpentKmS : 0.0,
        Spacecraft ? Spacecraft->FuelMassKg : 0.0);
}

void ASimulationManager::AddLog(const FString& LogText)
{
    Logs.Add(LogText);
    OnLogAdded.Broadcast(LogText);
    UE_LOG(LogSimulationManager, Log, TEXT("[Sim] %s"), *LogText);
}

bool ASimulationManager::AllPropertiesIsValid()
{
    if (!ScenarioGeneratorClass)
    {
        UE_LOG(LogSimulationManager, Error, TEXT("Forgot setup ScenarioGeneratorClass!"))
        return false;
    }

    if (!SpacecraftClass)
    {
        UE_LOG(LogSimulationManager, Error, TEXT("Forgot setup SpacecraftClass!"))
        return false;
    }

    if (!CelestialBodyClass)
    {
        UE_LOG(LogSimulationManager, Error, TEXT("Forgot setup CelestialBodyClass!"))
        return false;
    }

    if (!StarClass)
    {
        UE_LOG(LogSimulationManager, Error, TEXT("Forgot setup CelestialBodyClass!"))
        return false;
    }

    if (!ScenarioGenerator)
    {
        ScenarioGenerator = NewObject<UScenarioGenerator>(this, ScenarioGeneratorClass);
    }

    return true;
}