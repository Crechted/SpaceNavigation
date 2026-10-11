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

    if (!ScenarioGenerator)
    {
        ScenarioGenerator = NewObject<UScenarioGenerator>(this, ScenarioGeneratorClass);
    }

    if (!Spacecraft)
    {
        FActorSpawnParameters SpawnParameters;
        SpawnParameters.Owner = this;
        Spacecraft = GetWorld()->SpawnActor<ASpacecraft>(SpacecraftClass, SpawnParameters);
    }

    if (!RoutePlanner)
    {
        RoutePlanner = NewObject<URoutePlanner>(this, RoutePlannerClass);
        RoutePlanner->OnPlanningStarted.AddDynamic(this, &ASimulationManager::HandlePlannerPlanningStarted);
        RoutePlanner->OnPlanningCompleted.AddDynamic(this, &ASimulationManager::HandlePlannerPlanningCompleted);
    }
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

    RemoveBodies();
    CreateBodies();
    AddLog(FString::Printf(
        TEXT("Сценарий сгенерирован: seed=%d, планет=%d, граница=%.0f км"),
        ScenarioGenerator->Params.RandomSeed,
        ScenarioGenerator->PlanetData.Num(),
        SystemBoundary));

    OnScenarioGenerated.Broadcast();
}

bool ASimulationManager::ScenarioWasGenerated() const
{
    if (!ScenarioGenerator || Bodies.Num() < 2 || !Spacecraft)
    {
        return false;
    }

    return true;
}

void ASimulationManager::PlanRoute()
{
    if (!ScenarioWasGenerated()) return;

    if (RoutePlanner->IsPlanning())
    {
        RoutePlanner->AddLog(TEXT("Planning in progress!"));
        return;
    }

    RoutePlanner->Initialize(ScenarioGenerator->GetAllBodies(), Spacecraft->Params, SystemBoundary);

    RoutePlanner->StartPlanning();
}

bool ASimulationManager::HasPlanRoute() const
{
    if (!RoutePlanner || RoutePlanner->GetLastRoute().Maneuvers.IsEmpty())
    {
        return false;
    }

    return true;
}

bool ASimulationManager::IsRoutePlanning() const
{
    return RoutePlanner && RoutePlanner->IsPlanning();
}

void ASimulationManager::HandlePlannerPlanningStarted()
{
    OnPlanningStarted.Broadcast();
}

void ASimulationManager::HandlePlannerPlanningCompleted(bool bSuccess)
{
    if (!RoutePlanner)
    {
        OnPlanningCompleted.Broadcast(false);
        return;
    }

    const auto& CandidateRoutes = RoutePlanner->GetLastCandidateRoutes();
    const auto& RoutePlan = CandidateRoutes.IsEmpty() ? FRoutePlan() : CandidateRoutes[0];

    if (!CandidateRoutes.IsEmpty())
    {
        SimState = ESimState::Idle;

        RoutePlanner->AddLog(FString::Printf(
            TEXT("Маршрут спланирован: манёвров=%d, DeltaV=%.1f км/с, время=%.0f с, стоимость=%.3f"),
            RoutePlan.Maneuvers.Num(),
            RoutePlan.TotalDeltaV,
            RoutePlan.TotalTime,
            RoutePlan.TotalCost));

        // Разбивка стоимости по критериям: показывает, за счёт чего выбран маршрут.
        for (const FCriterionScore& Score : RoutePlan.CriterionScores)
        {
            RoutePlanner->AddLog(FString::Printf(
                TEXT("  %s: стоимость=%.3f, вес=%.2f, вклад=%.3f"),
                *Score.CriterionName, Score.Cost, Score.Weight, Score.WeightedCost));
        }
    }
    else
    {
        RoutePlanner->AddLog(TEXT("Допустимый маршрут не найден"));
    }

    OnPlanningCompleted.Broadcast(bSuccess && !CandidateRoutes.IsEmpty());
}

void ASimulationManager::StartSimulation()
{
    if (!HasPlanRoute() || !Spacecraft)
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

    Spacecraft->InitializeSpacecraft(StartBody, Star);

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
    if (!RoutePlanner)
    {
        return;
    }

    RoutePlanner->SetCriteria(NewCriteria);
}

TArray<URouteCriterion*> ASimulationManager::GetCriteria() const
{
    if (!RoutePlanner)
    {
        return {};
    }

    return RoutePlanner->GetCriteria();
}

void ASimulationManager::RemoveBodies()
{
    for (TObjectPtr<ACelestialBody>& Body : Bodies)
    {
        if (IsValid(Body))
        {
            Body->Destroy();
        }
    }
    Bodies.Reset();
}

void ASimulationManager::CreateBodies()
{
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

        // При прямом гелиоцентрическом отходе планета отправления не должна
        // перезахватывать аппарат, пока он не выйдет из её сферы влияния.
        if (Index == DirectDepartureIgnoreBodyIndex)
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

    const auto& RoutePlan = RoutePlanner->GetLastRoute();
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
    return CurrentManeuverIndex >= RoutePlanner->GetLastRoute().Maneuvers.Num() - 1;
}

void ASimulationManager::ApplyManeuverImpulse()
{
    if (!RoutePlanner)
    {
        return;
    }

    const auto& RoutePlan = RoutePlanner->GetLastRoute();

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
    FVector ProgradeDirection = FromBody->VelocityKmS.GetSafeNormal();
    if (ProgradeDirection.IsNearlyZero())
    {
        // Звезда или неподвижное тело: опираемся на текущую орбиту аппарата.
        ProgradeDirection = Spacecraft->RelativeVelocityKmS.GetSafeNormal();
    }
    if (ProgradeDirection.IsNearlyZero())
    {
        return;
    }

    if (FromBody->IsA(AStarBody::StaticClass()))
    {
        // Старт со звезды: чистый гелиоцентрический импульс перехода.
        const FHohmannTransfer Transfer = UOrbitMath::ComputeHohmannTransfer(
            Bodies[0]->BodyData.Mass, Maneuver.StartRadius, Maneuver.EndRadius);
        const FVector DesiredRelativeVelocity = ProgradeDirection * Transfer.TransferVelocityStart;
        const FVector DeltaVVelocity = DesiredRelativeVelocity - Spacecraft->RelativeVelocityKmS;
        const double AppliedDeltaV = Spacecraft->ApplyImpulse(DeltaVVelocity.Size(), DeltaVVelocity.GetSafeNormal());

        Spacecraft->UpdateAbsoluteState(Bodies.IsValidIndex(0) ? Bodies[0].Get() : nullptr);

        AddLog(FString::Printf(
            TEXT("Манёвр %d: импульс отхода %.1f км/с (применено %.1f км/с)"),
            CurrentManeuverIndex + 1,
            Maneuver.DeltaVDeparture,
            AppliedDeltaV));
        return;
    }

    // --- Прямой гелиоцентрический отход с планеты. ---
    // Планировщик строит гелиоцентрическую дугу, начинающуюся в t_dep со
    // скоростью v1 (для участков Ламберта — Maneuver.DepartureVelocityKmS; для
    // Хохмана — скорость перехода в начальной точке). Реальный выход из сферы
    // влияния занимает время и смещает эпоху, из-за чего гелиоцентрическая
    // скорость планеты успевает повернуться и дуга «уезжает» на миллионы км.
    // Поэтому вместо гиперболического разгона сразу выставляем аппарату
    // плановую гелиоцентрическую скорость и переводим его на орбиту вокруг
    // звезды. Топливо списываем по плановой стоимости DeltaVDeparture.
    FVector DesiredHeliocentricVelocity;
    if (!Maneuver.DepartureVelocityKmS.IsNearlyZero())
    {
        DesiredHeliocentricVelocity = Maneuver.DepartureVelocityKmS;
    }
    else
    {
        const FHohmannTransfer Transfer = UOrbitMath::ComputeHohmannTransfer(
            Bodies[0]->BodyData.Mass, Maneuver.StartRadius, Maneuver.EndRadius);
        const double DirectionSign = (Maneuver.EndRadius >= Maneuver.StartRadius) ? 1.0 : -1.0;
        DesiredHeliocentricVelocity = ProgradeDirection * (DirectionSign * Transfer.TransferVelocityStart);
    }

    if (DesiredHeliocentricVelocity.IsNearlyZero())
    {
        return;
    }

    const FVector PreBurnAbsoluteVelocity = Spacecraft->AbsoluteVelocityKmS;
    const double AppliedDeltaV = Spacecraft->ApplyImpulse(
        Maneuver.DeltaVDeparture, DesiredHeliocentricVelocity.GetSafeNormal());

    // Мгновенно ставим аппарат на плановую гелиоцентрическую скорость.
    Spacecraft->AbsoluteVelocityKmS = DesiredHeliocentricVelocity;

    // Планета отправления не должна «захватывать» аппарат обратно, пока он
    // внутри её сферы влияния.
    DirectDepartureIgnoreBodyIndex = Maneuver.FromBodyIndex;

    if (Bodies.IsValidIndex(0))
    {
        ACelestialBody* Star = Bodies[0].Get();
        if (Spacecraft->CurrentCentralBody != Star)
        {
            SwitchCentralBody(Star);
        }
        else
        {
            Spacecraft->RelativeVelocityKmS = DesiredHeliocentricVelocity - Star->VelocityKmS;
            Spacecraft->UpdateAbsoluteState(Star);
        }
    }

    UE_LOG(LogSimulationManager, Log,
        TEXT(
            "[Sim] Direct burn telemetry: t=%.3e s, origin=%s, absVel=(%.6f,%.6f,%.6f)|%.4f -> desired=(%.6f,%.6f,%.6f)|%.4f, planned=%.4f applied=%.4f"
        ),
        SimulationTime,
        *FromBody->BodyData.BodyName.ToString(),
        PreBurnAbsoluteVelocity.X, PreBurnAbsoluteVelocity.Y, PreBurnAbsoluteVelocity.Z,
        PreBurnAbsoluteVelocity.Size(),
        DesiredHeliocentricVelocity.X, DesiredHeliocentricVelocity.Y, DesiredHeliocentricVelocity.Z,
        DesiredHeliocentricVelocity.Size(),
        Maneuver.DeltaVDeparture, AppliedDeltaV);

    AddLog(FString::Printf(
        TEXT("Манёвр %d: прямой гелиоцентрический отход %.1f км/с (применено %.1f км/с)"),
        CurrentManeuverIndex + 1,
        Maneuver.DeltaVDeparture,
        AppliedDeltaV));

    if (AppliedDeltaV + 1.0e-6 < Maneuver.DeltaVDeparture)
    {
        AddLog(TEXT("Топлива не хватило: импульс ограничен запасом"));
    }
}

void ASimulationManager::AdvanceToNextManeuver()
{
    if (!RoutePlanner)
    {
        return;
    }

    const auto& RoutePlan = RoutePlanner->GetLastRoute();

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
                // Аппарат не добрался до сферы влияния цели. Честно пропускаем
                // захват: не меняем центральное тело и не применяем тормозной
                // импульс на большом расстоянии (иначе это фиктивный захват).
                AddLog(FString::Printf(
                    TEXT(
                        "Манёвр %d: прибытие к %s за пределами SOI (dist=%.3e км, SOI=%.3e км) — пропускаем захват по плану, продолжаем полёт"),
                    CurrentManeuverIndex + 1, *ToBody->BodyData.BodyName.ToString(), Dist, SOI));
            }
            else
            {
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
    if (SimState != ESimState::Simulating || !Spacecraft || Bodies.IsEmpty() || !RoutePlanner)
    {
        return;
    }

    if (DeltaTimeSeconds <= 0.0)
    {
        return;
    }
    const auto& RoutePlan = RoutePlanner->GetLastRoute();

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

        // Пока идёт ожидание окна отхода, аппарат висит в парковочной точке
        // (относительное состояние не интегрируем). Иначе грубый шаг
        // MaxIntegrationStep (~1 ч) полностью алиасит быструю парковочную орбиту
        // и может «закопать» аппарат в планету. Парковочная фаза для плана не
        // важна: отход выполняется прямым выставлением гелиоцентрической
        // скорости.
        const bool bWaitingForDeparture =
            RoutePlan.Maneuvers.IsValidIndex(CurrentManeuverIndex)
            && !bDepartureImpulseFired
            && CurrentManeuverElapsed < RoutePlan.Maneuvers[CurrentManeuverIndex].DelaySeconds;

        if (!bWaitingForDeparture)
        {
            Spacecraft->IntegrateStep(Step);
        }
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

        // Снимаем блокировку перезахвата, как только аппарат покинул сферу
        // влияния планеты отправления.
        if (DirectDepartureIgnoreBodyIndex != INDEX_NONE)
        {
            const ACelestialBody* IgnoreBody = Bodies.IsValidIndex(DirectDepartureIgnoreBodyIndex)
                                                   ? Bodies[DirectDepartureIgnoreBodyIndex].Get()
                                                   : nullptr;
            if (!IgnoreBody
                || FVector::Dist(Spacecraft->AbsolutePositionKm, IgnoreBody->PositionKm) > IgnoreBody->GetSOIKm())
            {
                DirectDepartureIgnoreBodyIndex = INDEX_NONE;
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
                    // Финальный участок — переход к границе системы: его
                    // завершение по плану и есть достижение границы (апогей
                    // перехода лежит ровно на границе).
                    CurrentManeuverIndex = RoutePlan.Maneuvers.Num();
                    CurrentManeuverElapsed = 0.0;

                    if (bAutoStopOnCompletion)
                    {
                        AddLog(TEXT("Достигнута граница системы"));
                        FinishSimulation(ESimState::Success, ESimOutcome::BoundaryReached);
                        return;
                    }

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

    // Апогей финального перехода лишь касается границы, поэтому допускаем
    // небольшое недотягивание (см. BoundaryToleranceFraction).
    const double Tolerance = FMath::Max(
        SystemBoundary * FMath::Clamp(BoundaryToleranceFraction, 0.0, 1.0), 1.0e5);
    return CurrentDistanceFromStar >= SystemBoundary - Tolerance;
}

bool ASimulationManager::CheckTimeLimit() const
{
    if (RoutePlanner->PlannerLimits.MaxFlightTime <= 0.0)
    {
        return false;
    }

    return SimulationTime > RoutePlanner->PlannerLimits.MaxFlightTime;
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

    auto StateStr = StaticEnum<ESimState>()->GetDisplayNameTextByIndex(static_cast<int32>(FinalState)).ToString();
    auto ReasonStr = StaticEnum<ESimOutcome>()->GetDisplayNameTextByIndex(static_cast<int32>(Reason)).ToString();

    AddLog(FString::Printf(
        TEXT("Симуляция завершена: состояние=%s, причина=%s, время=%.0f с, DeltaV=%.1f км/с, расстояние=%.0f км"),
        *StateStr,
        *ReasonStr,
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

    if (!RoutePlannerClass)
    {
        UE_LOG(LogSimulationManager, Error, TEXT("Forgot setup RoutePlannerClass!"))
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

    return true;
}