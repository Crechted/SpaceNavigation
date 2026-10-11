// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "Delegates.h"
#include "UObject/Object.h"
#include "SimTypes.h"
#include <atomic>
#include "Importer.h"
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
class SPACENAVIGATION_API URoutePlanner : public UObject, public IImportable
{
    GENERATED_BODY()

#pragma region DelegateProperties

public:
    UPROPERTY(BlueprintAssignable, BlueprintCallable, Category = "Delegates")
    FStringDynamicDelegate OnLogAdded;

    /** Уведомление о старте фонового поиска маршрута. */
    UPROPERTY(BlueprintAssignable, BlueprintCallable, Category = "Delegates")
    FSimpleDynamicDelegate OnPlanningStarted;

    /**
     * Уведомление о завершении фонового поиска маршрута.
     * Вызывается на игровом потоке, поэтому можно безопасно обновить UI.
     * @param bSuccess true, если найден допустимый маршрут.
     */
    UPROPERTY(BlueprintAssignable, BlueprintCallable, Category = "Delegates")
    FBoolDynamicDelegate OnPlanningCompleted;

#pragma endregion
#pragma region ParameterProperies

public:
    /** Ограничения планировщика маршрута. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Optimization")
    FPlannerLimits PlannerLimits;

protected:
    /**
     * Критерии оптимизации маршрута.
     *
     * Каждый критерий сам определяет свою оценку и вес, а планировщик
     * суммирует их вклады. Режимы ТЗ ("Fuel", "Time", "Balanced") — это
     * готовые наборы критериев из URouteCriterionPresets.
     *
     * Если массив пуст, используется пресет "Balanced".
     */
    UPROPERTY(EditAnywhere, Instanced, Category = "Optimization")
    TArray<TObjectPtr<URouteCriterion>> Critera;

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

    /** Индекс стартового тела для контекста оценки. */
    int32 StartBodyIndex = 0;

    /** Идёт ли фоновый поиск (атомарный доступ с других потоков). */
    std::atomic<bool> bPlanning = false;

    /** Запрос на кооперативную отмену фонового поиска. */
    std::atomic<bool> bCancelRequested = false;

    /**
     * Результаты последнего завершённого поиска.
     * Записываются и читаются только на игровом потоке (в обработчике
     * завершения фоновой задачи), поэтому отдельная синхронизация не нужна.
     */
    TArray<FRoutePlan> LastCandidateRoutes;
    FRoutePlan LastRoute;

#pragma endregion

#pragma region OverridedFunctions

public:
    virtual TArray<FParameter> ExtractParameters_Implementation() override;
    virtual bool ApplyParameters_Implementation(const TArray<FParameter>& InParameters) override;

#pragma endregion

#pragma region ControlFunctions

public:
    /** Подготовить планировщик: задать систему тел и параметры задачи. */
    UFUNCTION(BlueprintCallable, Category = "Route Planner")
    void Initialize(
        const TArray<FCelestialBodyData>& Bodies,
        const FSpacecraftParams& SpacecraftParams,
        double SystemBoundaryKm);

    /**
     * Найти лучший маршрут по активному критерию.
     * Детерминированный поиск: результат зависит только от входных данных.
     */
    UFUNCTION(BlueprintCallable, Category = "Route Planner")
    FRoutePlan FindBestRoute() const;

    /**
     * Запустить поиск маршрута в фоновом потоке.
     *
     * Тяжёлый поиск выполняется на пуле потоков, главный поток не блокируется.
     * Входные данные (тела, параметры аппарата, критерии) не должны меняться
     * до завершения поиска — контракт вызывающей стороны.
     *
     * Результаты сохраняются в GetLastCandidateRoutes / GetLastRoute, а о
     * завершении сообщает делегат OnPlanningCompleted (на игровом потоке).
     *
     * @return false, если поиск уже идёт (повторный запуск игнорируется).
     */
    UFUNCTION(BlueprintCallable, Category = "Route Planner")
    bool StartPlanning();

    /**
     * Запросить отмену текущего фонового поиска (кооперативная).
     * Завершение всё равно придёт по OnPlanningCompleted с параметром false.
     */
    UFUNCTION(BlueprintCallable, Category = "Route Planner")
    void CancelPlanning();

    /** Идёт ли сейчас фоновый поиск маршрута. */
    UFUNCTION(BlueprintPure, Category = "Route Planner")
    bool IsPlanning() const;

    /**
     * Заменить набор критериев оптимизации и перепланировать маршрут.
     * Пустой массив восстанавливает пресет "Balanced".
     */
    UFUNCTION(BlueprintCallable, Category = "Simulation")
    void SetCriteria(const TArray<URouteCriterion*>& NewCriteria);

    /**
     * Получение текущий критериев
     */
    UFUNCTION(BlueprintPure, Category = "Simulation")
    const TArray<URouteCriterion*>& GetCriteria() const;

    /** Кандидаты последнего завершённого поиска (лучший первым). */
    UFUNCTION(BlueprintCallable, Category = "Route Planner")
    TArray<FRoutePlan> GetLastCandidateRoutes() const;

    /** Лучший маршрут последнего завершённого поиска. */
    UFUNCTION(BlueprintCallable, Category = "Route Planner")
    FRoutePlan GetLastRoute() const;

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

        /**
         * Кэшированная оценка узла (ScoreNode).
         *
         * Вычисляется один раз при добавлении кандидата в пучок. Сортировка
         * пучка сравнивает сохранённые оценки, а не пересчитывает контекст
         * оценки (копию манёвров и проход по критериям) на каждое сравнение.
         */
        double Score = 0.0;
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

    /**
     * Фоновый исполнитель поиска маршрута.
     *
     * Вызывается на потоке из пула и возвращает управление на игровой поток,
     * записывая результаты и транслируя OnPlanningCompleted.
     */
    void RunPlanningAsync();

    /** Требуемая масса топлива для заданного суммарного DeltaV, кг. */
    double ComputeRequiredFuel(double TotalDeltaVKmS) const;

    /** Собрать FRoutePlan из узла. */
    FRoutePlan BuildPlan(const FSearchNode& Node) const;

    /**
     * Проверить набор критериев и подставить пресет "Balanced", если он пуст.
     * Вызывается перед планированием.
     */
    void EnsureCriteria();

#pragma endregion
};