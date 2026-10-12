// Copyright Epic Games, Inc. All Rights Reserved.

#include "Core/RouteCriterion.h"
#include "Core/OrbitMath.h"

namespace RouteCriterionInternal
{
/** Подстановка опорного DeltaV, если заданное значение непригодно, км/с. */
constexpr double DefaultDeltaVReference = 10.0;

/** Подстановка опорного времени, если заданное значение непригодно, с. */
constexpr double DefaultTimeReference = SimConstants::SecondsPerYear * 5.0;

/** Подстановка числа манёвров для максимального риска. */
constexpr int32 DefaultReferenceManeuverCount = 8;

// Подстановка безопасного сближения в радиусах тела. */
constexpr double DefaultSafeProximityBodyRadii = 20.0;

/** Подстановка относительной скорости сближения для максимального риска, км/с. */
constexpr double DefaultReferenceApproachVelocity = 5.0;

/** Подстановка отношения радиусов одиночного перелёта для максимального риска. */
constexpr double DefaultReferenceJumpRadiusRatio = 10.0;

/** Подстановка достаточного запаса deltaV на коррекцию, км/с. */
constexpr double DefaultReferenceFuelReserve = 2.0;

/** Отношение гелиоцентрических радиусов участка (>= 1). */
double TransferRadiusRatio(double StartRadius, double EndRadius)
{
    const double Lo = FMath::Max(FMath::Min(StartRadius, EndRadius), 1.0);
    const double Hi = FMath::Max(StartRadius, EndRadius);
    return Hi / Lo;
}

/** Значение > 0, если параметр пригоден для использования. */
bool IsUsable(double Value)
{
    return FMath::IsFinite(Value) && Value > 0.0;
}

/** Безопасная высота сближения для тела заданного радиуса, км. */
double SafeApproachKm(double BodyRadius, double SafeProximityBodyRadii)
{
    const double Radii = IsUsable(SafeProximityBodyRadii) ? SafeProximityBodyRadii : DefaultSafeProximityBodyRadii;
    return FMath::Max(BodyRadius, 0.0) * FMath::Max(Radii, 1.0);
}
}  // namespace RouteCriterionInternal

namespace
{
const FName WeightName(TEXT("Вес критерия"));

const FName DeltaVReferenceName(TEXT("Опорный DeltaV для нормировки, км/с"));
const FName TimeReferenceName(TEXT("Опорное время полёта для нормировки, с"));

const FName ProximityWeightName(TEXT("Доля близости сближения в риске манёвра"));
const FName ApproachVelocityWeightName(TEXT("Доля скорости сближения в риске манёвра"));
const FName ToleranceWeightName(TEXT("Доля чувствительности к ошибке в риске манёвра"));
const FName JumpWeightName(TEXT("Доля дальности одиночного перелёта в риске манёвра"));
const FName FuelReserveWeightName(TEXT("Доля запаса топлива в оценке риска"));
const FName ManeuverCountWeightName(TEXT("Доля числа манёвров в оценке риска"));
const FName ReferenceManeuverCountName(TEXT("Число манёвров, при котором штраф за их число максимален"));
const FName SafeProximityBodyRadiiName(TEXT("Безопасная высота сближения, в радиусах тела"));
const FName ReferenceApproachVelocityName(TEXT("Скорость сближения, считающаяся максимально рискованной, км/с"));
const FName ReferenceJumpRadiusRatioName(TEXT("Отношение радиусов перелёта, при котором риск дальности максимален"));
const FName ReferenceManeuverDurationName(TEXT("Длительность манёвра с максимальной чувствительностью к ошибке, с"));
const FName ReferenceFuelReserveName(TEXT("Запас deltaV, при котором топливо на коррекцию достаточно, км/с"));
}  // namespace

#pragma region URouteCriterion

#pragma region OverridedFunctions

FParameters URouteCriterion::ExtractParameters_Implementation()
{
    FParameters Result;

    Result.Parameters.Add(FParameter(WeightName, EParamTypes::Double, LexToString(Weight)));

    return Result;
}

bool URouteCriterion::ApplyParameters_Implementation(const FParameters& InParameters)
{
    for (const FParameter& Parameter : InParameters.Parameters)
    {
        if (Parameter.LocalName == WeightName)
        {
            Weight = FCString::Atod(*Parameter.Value);
        }
    }

    return true;
}

double URouteCriterion::EvaluateManeuver(const FManeuver& Maneuver, const FRouteEvaluationContext& Context) const
{
    // Базовый критерий не высказывает мнения о стоимости отдельного манёвра.
    return 0.0;
}

double URouteCriterion::EvaluateRoute(const FRouteEvaluationContext& Context) const
{
    // Среднее по манёврам: для маршрута без манёвров оценка нулевая.
    const int32 Count = Context.Maneuvers.Num();
    if (Count <= 0)
    {
        return 0.0;
    }

    double Sum = 0.0;
    for (const FManeuver& Maneuver : Context.Maneuvers)
    {
        Sum += EvaluateManeuver(Maneuver, Context);
    }

    return Sum / static_cast<double>(Count);
}

FString URouteCriterion::GetCriterionName() const
{
    return TEXT("Criterion");
}

#pragma endregion

#pragma endregion

#pragma region UFuelCriterion

#pragma region OverridedFunctions

FParameters UFuelCriterion::ExtractParameters_Implementation()
{
    FParameters Result = Super::ExtractParameters_Implementation();

    Result.Parameters.Add(FParameter(DeltaVReferenceName, EParamTypes::Double, LexToString(DeltaVReference), TEXT("км/с")));

    return Result;
}

bool UFuelCriterion::ApplyParameters_Implementation(const FParameters& InParameters)
{
    for (const FParameter& Parameter : InParameters.Parameters)
    {
        if (Parameter.LocalName == DeltaVReferenceName)
        {
            DeltaVReference = FCString::Atod(*Parameter.Value);
        }
    }

    return Super::ApplyParameters_Implementation(InParameters);
}

double UFuelCriterion::EvaluateManeuver(const FManeuver& Maneuver, const FRouteEvaluationContext& Context) const
{
    const double Reference =
        RouteCriterionInternal::IsUsable(DeltaVReference) ? DeltaVReference : RouteCriterionInternal::DefaultDeltaVReference;

    return UOrbitMath::NormalizeRange(Maneuver.DeltaV, 0.0, Reference);
}

double UFuelCriterion::EvaluateRoute(const FRouteEvaluationContext& Context) const
{
    const double Reference =
        RouteCriterionInternal::IsUsable(DeltaVReference) ? DeltaVReference : RouteCriterionInternal::DefaultDeltaVReference;

    return UOrbitMath::NormalizeRange(Context.TotalDeltaV, 0.0, Reference);
}

FString UFuelCriterion::GetCriterionName() const
{
    return TEXT("Fuel");
}

#pragma endregion

#pragma endregion

#pragma region UTimeCriterion

#pragma region OverridedFunctions

FParameters UTimeCriterion::ExtractParameters_Implementation()
{
    FParameters Result = Super::ExtractParameters_Implementation();

    Result.Parameters.Add(FParameter(TimeReferenceName, EParamTypes::Double, LexToString(TimeReference), TEXT("с")));

    return Result;
}

bool UTimeCriterion::ApplyParameters_Implementation(const FParameters& InParameters)
{
    for (const FParameter& Parameter : InParameters.Parameters)
    {
        if (Parameter.LocalName == TimeReferenceName)
        {
            TimeReference = FCString::Atod(*Parameter.Value);
        }
    }

    return Super::ApplyParameters_Implementation(InParameters);
}

double UTimeCriterion::EvaluateManeuver(const FManeuver& Maneuver, const FRouteEvaluationContext& Context) const
{
    const double Reference = RouteCriterionInternal::IsUsable(TimeReference) ? TimeReference : RouteCriterionInternal::DefaultTimeReference;

    return UOrbitMath::NormalizeRange(Maneuver.DurationSeconds, 0.0, Reference);
}

double UTimeCriterion::EvaluateRoute(const FRouteEvaluationContext& Context) const
{
    const double Reference = RouteCriterionInternal::IsUsable(TimeReference) ? TimeReference : RouteCriterionInternal::DefaultTimeReference;

    return UOrbitMath::NormalizeRange(Context.TotalTimeSeconds, 0.0, Reference);
}

FString UTimeCriterion::GetCriterionName() const
{
    return TEXT("Time");
}

#pragma endregion

#pragma endregion

#pragma region URiskCriterion

#pragma region OverridedFunctions

FParameters URiskCriterion::ExtractParameters_Implementation()
{
    FParameters Result = Super::ExtractParameters_Implementation();

    Result.Parameters.Add(FParameter(ProximityWeightName, EParamTypes::Double, LexToString(ProximityWeight)));
    Result.Parameters.Add(FParameter(ApproachVelocityWeightName, EParamTypes::Double, LexToString(ApproachVelocityWeight)));
    Result.Parameters.Add(FParameter(ToleranceWeightName, EParamTypes::Double, LexToString(ToleranceWeight)));
    Result.Parameters.Add(FParameter(JumpWeightName, EParamTypes::Double, LexToString(JumpWeight)));
    Result.Parameters.Add(FParameter(FuelReserveWeightName, EParamTypes::Double, LexToString(FuelReserveWeight)));
    Result.Parameters.Add(FParameter(ManeuverCountWeightName, EParamTypes::Double, LexToString(ManeuverCountWeight)));
    Result.Parameters.Add(FParameter(ReferenceManeuverCountName, EParamTypes::Int32, LexToString(ReferenceManeuverCount)));
    Result.Parameters.Add(
        FParameter(SafeProximityBodyRadiiName, EParamTypes::Double, LexToString(SafeProximityBodyRadii), TEXT("радиусов тела")));
    Result.Parameters.Add(
        FParameter(ReferenceApproachVelocityName, EParamTypes::Double, LexToString(ReferenceApproachVelocity), TEXT("км/с")));
    Result.Parameters.Add(FParameter(ReferenceJumpRadiusRatioName, EParamTypes::Double, LexToString(ReferenceJumpRadiusRatio)));
    Result.Parameters.Add(
        FParameter(ReferenceManeuverDurationName, EParamTypes::Double, LexToString(ReferenceManeuverDuration), TEXT("с")));
    Result.Parameters.Add(FParameter(ReferenceFuelReserveName, EParamTypes::Double, LexToString(ReferenceFuelReserve), TEXT("км/с")));

    return Result;
}

bool URiskCriterion::ApplyParameters_Implementation(const FParameters& InParameters)
{
    for (const FParameter& Parameter : InParameters.Parameters)
    {
        if (Parameter.LocalName == ProximityWeightName)
        {
            ProximityWeight = FCString::Atod(*Parameter.Value);
        }
        else if (Parameter.LocalName == ApproachVelocityWeightName)
        {
            ApproachVelocityWeight = FCString::Atod(*Parameter.Value);
        }
        else if (Parameter.LocalName == ToleranceWeightName)
        {
            ToleranceWeight = FCString::Atod(*Parameter.Value);
        }
        else if (Parameter.LocalName == JumpWeightName)
        {
            JumpWeight = FCString::Atod(*Parameter.Value);
        }
        else if (Parameter.LocalName == FuelReserveWeightName)
        {
            FuelReserveWeight = FCString::Atod(*Parameter.Value);
        }
        else if (Parameter.LocalName == ManeuverCountWeightName)
        {
            ManeuverCountWeight = FCString::Atod(*Parameter.Value);
        }
        else if (Parameter.LocalName == ReferenceManeuverCountName)
        {
            ReferenceManeuverCount = FCString::Atoi(*Parameter.Value);
        }
        else if (Parameter.LocalName == SafeProximityBodyRadiiName)
        {
            SafeProximityBodyRadii = FCString::Atod(*Parameter.Value);
        }
        else if (Parameter.LocalName == ReferenceApproachVelocityName)
        {
            ReferenceApproachVelocity = FCString::Atod(*Parameter.Value);
        }
        else if (Parameter.LocalName == ReferenceJumpRadiusRatioName)
        {
            ReferenceJumpRadiusRatio = FCString::Atod(*Parameter.Value);
        }
        else if (Parameter.LocalName == ReferenceManeuverDurationName)
        {
            ReferenceManeuverDuration = FCString::Atod(*Parameter.Value);
        }
        else if (Parameter.LocalName == ReferenceFuelReserveName)
        {
            ReferenceFuelReserve = FCString::Atod(*Parameter.Value);
        }
    }

    return Super::ApplyParameters_Implementation(InParameters);
}

double URiskCriterion::EvaluateManeuver(const FManeuver& Maneuver, const FRouteEvaluationContext& Context) const
{
    // Риск манёвра складывается из четырёх физических факторов, каждый строго
    // в [0; 1]. Веса нормируются их суммой, поэтому результат тоже в [0; 1]
    // независимо от абсолютных значений весов (важно для EvaluateRoute,
    // который берёт максимум по манёврам и умножает на суммарный вес блока).

    // 1. Глубина сближения с телом назначения. Манёвр к границе сближения не имеет.
    double ProximityRisk = 0.0;
    if (Maneuver.ApproachAltitudeKm >= 0.0 && Maneuver.TargetBodyRadiusKm > 0.0)
    {
        const double SafeApproach = RouteCriterionInternal::SafeApproachKm(Maneuver.TargetBodyRadiusKm, SafeProximityBodyRadii);
        ProximityRisk = FMath::Clamp(1.0 - UOrbitMath::NormalizeRange(Maneuver.ApproachAltitudeKm, 0.0, SafeApproach), 0.0, 1.0);
    }

    // 2. Относительная скорость сближения: чем выше, тем больше энергия входа
    // в сферу влияния и тем меньше времени на реакцию.
    double ApproachVelocityRisk = 0.0;
    if (Maneuver.ApproachAltitudeKm >= 0.0)
    {
        const double VelocityReference = RouteCriterionInternal::IsUsable(ReferenceApproachVelocity)
                                             ? ReferenceApproachVelocity
                                             : RouteCriterionInternal::DefaultReferenceApproachVelocity;
        ApproachVelocityRisk =
            FMath::Clamp(UOrbitMath::NormalizeRange(Maneuver.ApproachRelativeVelocityKmS, 0.0, VelocityReference), 0.0, 1.0);
    }

    // 3. Допустимая погрешность параметров манёвра. Прямой меры чувствительности
    // нет, поэтому используется длительность отдельного плеча: длинный пассивный
    // участок усиливает ошибку выведения и уменьшает допустимую погрешность.
    const double DurationReference = RouteCriterionInternal::IsUsable(ReferenceManeuverDuration)
                                         ? ReferenceManeuverDuration
                                         : RouteCriterionInternal::DefaultTimeReference;
    const double ToleranceRisk = FMath::Clamp(UOrbitMath::NormalizeRange(Maneuver.DurationSeconds, 0.0, DurationReference), 0.0, 1.0);

    // 4. Дальность одиночного перелёта: отношение гелиоцентрических радиусов
    // участка. Большой прыжок через несколько орбит исполним, но слабо
    // страхуем и очень чувствителен к ошибкам — это и есть «ограничение
    // одного участка как параметр риска».
    const double JumpReference = RouteCriterionInternal::IsUsable(ReferenceJumpRadiusRatio)
                                     ? ReferenceJumpRadiusRatio
                                     : RouteCriterionInternal::DefaultReferenceJumpRadiusRatio;
    const double JumpRatio = RouteCriterionInternal::TransferRadiusRatio(Maneuver.StartRadius, Maneuver.EndRadius);
    const double JumpRisk =
        FMath::Clamp(UOrbitMath::NormalizeRange(JumpRatio, 1.0, FMath::Max(JumpReference, 1.0 + KINDA_SMALL_NUMBER)), 0.0, 1.0);

    const double ProximityW = FMath::Max(ProximityWeight, 0.0);
    const double VelocityW = FMath::Max(ApproachVelocityWeight, 0.0);
    const double ToleranceW = FMath::Max(ToleranceWeight, 0.0);
    const double JumpW = FMath::Max(JumpWeight, 0.0);
    const double TotalW = ProximityW + VelocityW + ToleranceW + JumpW;
    if (TotalW <= 0.0)
    {
        return 0.0;
    }

    const double Risk =
        (ProximityW * ProximityRisk + VelocityW * ApproachVelocityRisk + ToleranceW * ToleranceRisk + JumpW * JumpRisk) / TotalW;
    return FMath::Clamp(Risk, 0.0, 1.0);
}

double URiskCriterion::EvaluateRoute(const FRouteEvaluationContext& Context) const
{
    // 1. Риск манёвров: берём ХУДШИЙ манёвр маршрута, а не среднее.
    // Вызов EvaluateManeuver гарантирует, что учитывается каждый отдельный
    // манёвр, включая его собственную длительность и сближение с телом.
    double WorstManeuverRisk = 0.0;
    for (const FManeuver& Maneuver : Context.Maneuvers)
    {
        WorstManeuverRisk = FMath::Max(WorstManeuverRisk, EvaluateManeuver(Maneuver, Context));
    }

    // Вклад блока манёвров равен сумме весов четырёх его факторов.
    const double ManeuverBlockWeight = FMath::Max(ProximityWeight, 0.0) + FMath::Max(ApproachVelocityWeight, 0.0) +
                                       FMath::Max(ToleranceWeight, 0.0) + FMath::Max(JumpWeight, 0.0);

    // 2. Риск числа манёвров: каждый импульс — дополнительный источник отказов.
    const int32 ManeuverReference =
        ReferenceManeuverCount >= 1 ? ReferenceManeuverCount : RouteCriterionInternal::DefaultReferenceManeuverCount;
    const double ManeuverCountRisk =
        UOrbitMath::NormalizeRange(static_cast<double>(Context.ManeuverCount), 1.0, static_cast<double>(FMath::Max(ManeuverReference, 1)));

    // 3. Риск нехватки топлива на коррекцию: чем меньше остаток бюджета deltaV
    // (AvailableDeltaV − TotalDeltaV), тем выше риск. Если бюджет неизвестен
    // (AvailableDeltaVKmS <= 0), фактор не начисляется.
    double FuelReserveRisk = 0.0;
    if (Context.AvailableDeltaVKmS > 0.0)
    {
        const double ReserveReference = RouteCriterionInternal::IsUsable(ReferenceFuelReserve)
                                            ? ReferenceFuelReserve
                                            : RouteCriterionInternal::DefaultReferenceFuelReserve;
        const double Reserve = Context.AvailableDeltaVKmS - Context.TotalDeltaV;
        FuelReserveRisk = FMath::Clamp(1.0 - UOrbitMath::NormalizeRange(Reserve, 0.0, ReserveReference), 0.0, 1.0);
    }

    const double Risk = ManeuverBlockWeight * WorstManeuverRisk + FMath::Max(ManeuverCountWeight, 0.0) * ManeuverCountRisk +
                        FMath::Max(FuelReserveWeight, 0.0) * FuelReserveRisk;

    return FMath::Clamp(Risk, 0.0, 1.0);
}

FString URiskCriterion::GetCriterionName() const
{
    return TEXT("Risk");
}

#pragma endregion

#pragma endregion

#pragma region URouteCriterionPresets

#pragma region PresetFunctions

TArray<URouteCriterion*> URouteCriterionPresets::MakeFuelEfficient(UObject* Outer)
{
    TArray<TObjectPtr<URouteCriterion>> Result;

    UFuelCriterion* Criterion = NewObject<UFuelCriterion>(Outer);
    Criterion->Weight = 1.0;
    Result.Add(Criterion);

    return Result;
}

TArray<URouteCriterion*> URouteCriterionPresets::MakeTimeEfficient(UObject* Outer)
{
    TArray<TObjectPtr<URouteCriterion>> Result;

    UTimeCriterion* Criterion = NewObject<UTimeCriterion>(Outer);
    Criterion->Weight = 1.0;
    Result.Add(Criterion);

    return Result;
}

TArray<URouteCriterion*> URouteCriterionPresets::MakeBalanced(UObject* Outer)
{
    // Веса из ТЗ: WFuel = 0.6, WTime = 0.3, WRisk = 0.1.
    TArray<TObjectPtr<URouteCriterion>> Result;

    UFuelCriterion* Fuel = NewObject<UFuelCriterion>(Outer);
    Fuel->Weight = 0.6;
    Result.Add(Fuel);

    UTimeCriterion* Time = NewObject<UTimeCriterion>(Outer);
    Time->Weight = 0.3;
    Result.Add(Time);

    URiskCriterion* Risk = NewObject<URiskCriterion>(Outer);
    Risk->Weight = 0.1;
    Result.Add(Risk);

    return Result;
}

TArray<URouteCriterion*> URouteCriterionPresets::MakeRiskDominant(UObject* Outer)
{
    // Чистый риск: вес 1. Топливо и время не учитываются, поэтому выбор
    // маршрута определяется только оценкой риска (в т.ч. дальностью участка).
    TArray<TObjectPtr<URouteCriterion>> Result;

    URiskCriterion* Risk = NewObject<URiskCriterion>(Outer);
    Risk->Weight = 1.0;
    Result.Add(Risk);

    return Result;
}

TArray<URouteCriterion*> URouteCriterionPresets::MakeByPresetName(UObject* Outer, const FString& PresetName)
{
    if (PresetName.Equals(TEXT("Fuel"), ESearchCase::IgnoreCase))
    {
        return MakeFuelEfficient(Outer);
    }

    if (PresetName.Equals(TEXT("Time"), ESearchCase::IgnoreCase))
    {
        return MakeTimeEfficient(Outer);
    }

    if (PresetName.Equals(TEXT("Risk"), ESearchCase::IgnoreCase))
    {
        return MakeRiskDominant(Outer);
    }

    return MakeBalanced(Outer);
}

#pragma endregion

#pragma endregion
