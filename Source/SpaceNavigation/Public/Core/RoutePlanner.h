// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "Delegates.h"
#include "UObject/Object.h"
#include "SimTypes.h"
#include "RoutePlanner.generated.h"

class URouteCriterion;
/**
 * Планировщик маршрута.
 *
 * Задача: найти последовательность гравитационных манёвров (переходов между
 * орбитами планет), которая выводит аппарат за границу системы с минимальной
 * стоимостью по выбранному критерию.
 *
 * Модель перехода — упрощённые patched conics:
 *  - перелёт между орбитами считается переходом Хохмана (либо биэллиптическим,
 *    если он дешевле при отношении радиусов больше ~11.94);
 *  - выход из сферы влияния планеты требует импульса на покидание её
 *    гравитационного поля (v_inf + скорость убегания);
 *  - прибытие к планете требует импульса на захват в орбиту.
 *
 * Целевая точка — граница системы: либо фиксированная TargetDistance,
 * либо K * максимальный орбитальный радиус.
 *
 * Оценка вариантов делегирована массиву критериев URouteCriterion:
 * планировщик не содержит switch по видам критериев, а просто суммирует
 * Weight * EvaluateRoute. Добавление нового критерия не требует правок здесь.
 */
UCLASS(BlueprintType)
class SPACENAVIGATION_API URoutePlanner : public UObject
{
    GENERATED_BODY()

#pragma region DelegateProperties

public:
    UPROPERTY(BlueprintAssignable, BlueprintCallable, Category = "Delegates")
    FStringDelegate OnLogAdded;

#pragma endregion

#pragma region InnerProperties

public:
    /** Метки планирования для отладки и вывода в UI. */
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Log")
    TArray<FString> Logs;

private:
    /** Индекс виртуальной цели: граница системы. */
    static constexpr int32 BoundaryTargetIndex = -1;

    /** Данные системы: Bodies[0] — звезда. */
    TArray<FCelestialBodyData> Bodies;

    /** Параметры аппарата. */
    FSpacecraftParams SpacecraftParams;

    /** Граница системы, км. */
    double SystemBoundaryKm = 0.0;

    /**
     * Активные критерии оптимизации.
     *
     * Каждый критерий сам определяет свою нормировку, поэтому планировщику
     * не нужны общие опорные значения топлива и времени.
     */
    UPROPERTY(Transient)
    TArray<TObjectPtr<URouteCriterion>> Criteria;

    /** Ограничения поиска. */
    FPlannerLimits Limits;

    /** Индекс стартового тела для контекста оценки. */
    int32 StartBodyIndex = 0;

#pragma endregion

#pragma region ControlFunctions

public:
    /** Подготовить планировщик: задать систему тел и параметры задачи. */
    UFUNCTION(BlueprintCallable, Category = "Route Planner")
    void Initialize(
        const TArray<FCelestialBodyData>& Bodies,
        const FSpacecraftParams& SpacecraftParams,
        double SystemBoundaryKm,
        const TArray<URouteCriterion*>& InCriteria,
        const FPlannerLimits& InLimits);

    /**
     * Найти лучший маршрут по активному критерию.
     * Детерминированный поиск: результат зависит только от входных данных.
     */
    UFUNCTION(BlueprintCallable, Category = "Route Planner")
    FRoutePlan FindBestRoute() const;

    /**
     * Расчёт стоимости одного участка маршрута.
     *
     * @param FromBodyIndex индекс исходного тела (0 — звезда)
     * @param ToBodyIndex   индекс конечного тела; отрицательное значение означает границу системы
     * @param OutManeuver   параметры манёвра
     * @param DepartureTimeSeconds абсолютное время отбытия от планеты, с.
     *                             Ноль (по умолчанию) — классический Хохман без
     *                             фазирования; для планетных участков с временем —
     *                             перелёт по дуге Ламберта (берётся лучший TOF из сетки).
     */
    UFUNCTION(BlueprintCallable, Category = "Route Planner")
    bool ComputeManeuver(int32 FromBodyIndex, int32 ToBodyIndex, FManeuver& OutManeuver, double DepartureTimeSeconds = 0.0) const;

    /** Полный список найденных допустимых маршрутов, отсортированных по стоимости. */
    UFUNCTION(BlueprintCallable, Category = "Route Planner")
    TArray<FRoutePlan> GetCandidateRoutes() const;

    /** Гелиоцентрический радиус орбиты, на которой находится аппарат у тела, км. */
    UFUNCTION(BlueprintPure, Category = "Route Planner")
    double GetHeliocentricRadius(int32 BodyIndex) const;

    /**
     * Гелиоцентрическое положение тела в заданный момент времени, км.
     *
     * Повторяет кинематику рантайма (CelestialBody: круговая орбита, наклон
     * вокруг оси X, θ = InitialPhase + ω·t). Индекс 0 (звезда) — нулевой вектор.
     */
    UFUNCTION(BlueprintPure, Category = "Route Planner")
    FVector GetBodyPositionKm(int32 BodyIndex, double AbsoluteTimeSeconds) const;

    UFUNCTION(BlueprintCallable, Category = "Logs")
    void AddLog(const FString& LogText);

private:
    /** Состояние узла поиска. */
    struct FSearchNode
    {
        TArray<int32> VisitedBodies;

        /**
         * Манёвры, пройденные узлом, в порядке следования.
         *
         * Кэшируются по мере построения: ScoreNode вызывается из компаратора
         * сортировки пучка, и пересчёт манёвров на каждой оценке был бы
         * лишней работой (расчёт Хохмана плюс импульсы ухода и захвата).
         */
        TArray<FManeuver, TInlineAllocator<4>> Maneuvers;

        int32 CurrentBodyIndex = 0;
        double DeltaV = 0.0;
        double FlightTime = 0.0;
        /**
         * Абсолютное время симуляции (от старта сценария), в которое аппарат
         * находится на орбите CurrentBodyIndex (в начале следующего участка).
         * Накопленный прирост складывается из задержек фазирования на участках
         * (DelaySeconds) и времён перелёта (DurationSeconds).
         */
        double AbsoluteTimeSeconds = 0.0;
        /** Отрицательное значение — сближения с телами не было. */
        double ClosestApproachKm = -1.0;
        double ClosestApproachBodyRadius = 0.0;
        /** Относительная скорость в точке ближайшего сближения, км/с. */
        double ClosestApproachRelativeVelocityKmS = 0.0;
        bool bReachedBoundary = false;
    };

    /** Радиус орбиты, на которой аппарат обращается вокруг планеты, км. */
    double GetPlanetParkingRadius(int32 BodyIndex) const;

    /** Учёт манёвра в накопленных метриках узла и в списке манёвров. */
    void ApplyManeuver(const FManeuver& Maneuver, FSearchNode& Node) const;

    /**
     * Угловая скорость тела (круговая орбита), рад/с.
     * Повторяет CelestialBody::GetAngularVelocity.
     */
    double GetBodyAngularVelocity(int32 BodyIndex) const;

    /**
     * Гелиоцентрическая скорость тела в заданный момент времени, км/с.
     * Повторяет кинематику рантайма (CelestialBody::UpdateOrbitPosition).
     */
    FVector GetBodyVelocityKmS(int32 BodyIndex, double AbsoluteTimeSeconds) const;

    /**
     * Номинальный TOF для сетки фазирования: множитель × параболический минимум
     * для текущей геометрии (хорды между телами) в момент отбытия, с.
     */
    double LambertNominalToF(int32 FromBodyIndex, int32 ToBodyIndex, double DepartureTimeSeconds, double Multiplier) const;

    /**
     * Перелёт планета→планета по дуге Ламберта с фазированием.
     *
     * @param FromBodyIndex      планета отбытия (>= 1)
     * @param ToBodyIndex        планета прибытия (>= 1)
     * @param DepartureTimeSeconds абсолютное время отбытия с планеты, с
     * @param ToFSeconds         время перелёта по дуге, с
     * @param OutManeuver        параметры манёвра (DelaySeconds = 0)
     */
    bool ComputeLambertManeuver(int32 FromBodyIndex, int32 ToBodyIndex, double DepartureTimeSeconds, double ToFSeconds,
        FManeuver& OutManeuver) const;

    /** Внутренний расчёт без опционального параметра (используется поиском). */
    bool ComputeManeuverHohmann(int32 FromBodyIndex, int32 ToBodyIndex, FManeuver& OutManeuver) const;

    /**
     * Сгенерировать кандидатов-перелётов планета→планета через сетку фазирования
     * (t_dep × TOF) с дугами Ламберта. Результат дополняет DurationSeconds
     * ламбертовским временем и DelaySeconds задержкой до окна, а скорости —
     * гелиоцентрическими v1/v2 дуги.
     */
    void GenerateLambertCandidates(const FSearchNode& Node, int32 ToBodyIndex, TArray<FManeuver>& OutCandidates) const;

    /** Контекст оценки: агрегаты узла плюс представление его манёвров. */
    FRouteEvaluationContext BuildContext(const FSearchNode& Node, TArrayView<const FManeuver> Maneuvers, bool bComplete) const;

    /**
     * Стоимость узла: сумма Weight * EvaluateRoute по всем критериям.
     *
     * Если критерии не заданы, используется суммарный DeltaV — это сохраняет
     * поведение без настроенной оптимизации.
     */
    double ScoreNode(const FSearchNode& Node) const;

    /** Требуемая масса топлива для заданного суммарного DeltaV, кг. */
    double ComputeRequiredFuel(double TotalDeltaVKmS) const;

    /** Собрать FRoutePlan из узла. */
    FRoutePlan BuildPlan(const FSearchNode& Node) const;

#pragma endregion
};