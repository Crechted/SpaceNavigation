// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "SimConstants.h"
#include "SimTypes.generated.h"

/**
 * Состояние симуляции.
 */
UENUM(BlueprintType)
enum class ESimState : uint8
{
    Idle UMETA(DisplayName = "Idle"),
    Planning UMETA(DisplayName = "Planning"),
    Simulating UMETA(DisplayName = "Simulating"),
    Success UMETA(DisplayName = "Success"),
    Failed UMETA(DisplayName = "Failed")
};

/**
 * Причина завершения симуляции.
 */
UENUM(BlueprintType)
enum class ESimOutcome : uint8
{
    None UMETA(DisplayName = "Not Started"),
    BoundaryReached UMETA(DisplayName = "Boundary Reached"),
    FuelExhausted UMETA(DisplayName = "Fuel Exhausted"),
    TimeLimitExceeded UMETA(DisplayName = "Time Limit Exceeded"),
    RouteInvalid UMETA(DisplayName = "Route Invalid"),
    Collision UMETA(DisplayName = "Collision With Body")
};

/**
 * Параметры небесного тела.
 *
 * Единицы: масса — кг, радиусы — км, скорость — км/с, фаза — радианы.
 */
USTRUCT(BlueprintType)
struct FCelestialBodyData
{
    GENERATED_BODY()

    /** Масса тела, кг. Определяет силу гравитационного воздействия. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Body", meta = (Units = "kg"))
    double Mass = 0.0;

    /** Радиус тела, км. Определяет размер и допустимое сближение. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Body", meta = (Units = "km"))
    double Radius = 0.0;

    /** Орбитальный радиус относительно центральной звезды, км. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Orbit", meta = (Units = "km"))
    double OrbitalRadius = 0.0;

    /** Орбитальная скорость, км/с. Вычисляемая величина. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Orbit", meta = (Units = "km"))
    double OrbitalVelocity = 0.0;

    /** Начальная фаза / положение на орбите, радианы. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Orbit", meta = (Units = "rad"))
    double InitialPhase = 0.0;

    /** Наклон орбиты относительно базовой плоскости, радианы. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Orbit", meta = (Units = "rad"))
    double OrbitalInclination = 0.0;

    /** Сфера влияния, км. Может быть фиксированной либо вычисляемой. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Orbit", meta = (Units = "km"))
    double SphereOfInfluence = 0.0;

    /** Индекс тела в системе (0 — центральная звезда, 1..N — планеты). */
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Body")
    int32 BodyIndex = INDEX_NONE;

    /** Отображаемое имя. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Body")
    FName BodyName = NAME_None;
};

/**
 * Параметры генерации сценария.
 */
USTRUCT(BlueprintType)
struct FScenarioParams
{
    GENERATED_BODY()

    /** Зерно генератора случайных чисел. Одинаковое зерно — идентичная система. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Scenario")
    int32 RandomSeed = 12345;

    /** Количество генерируемых планет. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Scenario", meta = (ClampMin = "1", ClampMax = "20"))
    int32 NumberOfPlanets = 4;

    /** Минимальный орбитальный радиус планеты, км. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Scenario", meta = (Units = "km"))
    double MinOrbitalRadius = SimConstants::AuToKm(0.4);

    /**
    * Использовать фиксированную целевую дистанцию вместо относительной границы.
    *
    * По умолчанию выключено: граница задаётся как SystemBoundaryK * максимальный
    * орбитальный радиус. При NumberOfPlanets = 4 и шаге 0.5 AU это около 4.8 AU,
    * что даёт выполнимый сценарий. Значение 120 AU из ТЗ доступно как осознанно
    * сложный вариант, но требует существенно большего запаса топлива.
    */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Scenario")
    bool bUseTargetDistance = false;

    /** Целевая дистанция из ТЗ, км (120 AU). Используется при bUseTargetDistance. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Scenario", meta = (Units = "km"))
    double TargetDistance = SimConstants::AuToKm(120.0);

    /** Коэффициент K: System Boundary = K * максимальный орбитальный радиус. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Scenario", meta = (ClampMin = "1.0"))
    double SystemBoundaryK = 2.0;

    /** Вычислять сферу влияния по формуле вместо фиксированной. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Scenario")
    bool bUseComputedSOI = true;

    /** Коэффициент для простого расчета SOI, если оно фиксированное. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Scenario")
    double SOIK = 20.0;

    /** Порог, ниже которого заданная сила считается малой и пересчитывается по формуле, км. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Scenario", meta = (Units = "km"))
    double WeakForceThreshold = SimConstants::WeakForceThreshold;

    /** Разброс масс планет относительно номинальной земной, доля. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Scenario", meta = (ClampMin = "0.0", ClampMax = "1.0"))
    double MassVariation = 0.8;

    /** Разброс плотности планет (радиуса относительно земного), доля. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Scenario", meta = (ClampMin = "0.0", ClampMax = "1.0"))
    double RadiusVariation = 0.5;

    /** Разброс интервала между соседними орбитами, доля от базового шага. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Scenario", meta = (ClampMin = "0.0", ClampMax = "3.0"))
    double SpacingVariation = 0.6;

    /** Максимальный наклон орбиты, радианы (опционально, ТЗ). */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Scenario", meta = (Units = "rad", ClampMin = "0.0"))
    double MaxInclination = 0.1745; // ~10 градусов

    /** Масса центральной звезды, кг. Ноль — значение по умолчанию (масса Солнца). */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Scenario", meta = (Units = "kg"))
    double StarMassOverride = 0.0;

    /** Радиус центральной звезды, км. Ноль — значение по умолчанию. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Scenario", meta = (Units = "km"))
    double StarRadiusOverride = 0.0;
};

/**
 * Параметры космического аппарата.
 *
 * Значения по умолчанию подобраны так, чтобы сценарий по умолчанию был
 * выполним: при старте с первой планеты прямой переход к границе системы
 * требует около 12.6 км/с, а аппарат располагает примерно 15.8 км/с.
 * Удельный импульс 900 с соответствует ядерно-электрической схеме тяги.
 */
USTRUCT(BlueprintType)
struct FSpacecraftParams
{
    GENERATED_BODY()

    /** Сухая масса аппарата, кг. При расходовании топлива масса уменьшается. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Spacecraft", meta = (Units = "kg"))
    double DryMass = 40000.0;

    /** Доступное количество топлива, кг. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Spacecraft", meta = (Units = "kg"))
    double FuelMass = 200000.0;

    /** Максимальная тяга двигателя, ньютоны. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Spacecraft", meta = (Units = "N"))
    double MaxThrust = 450000.0;

    /** Удельный импульс, с. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Spacecraft", meta = (Units = "s"))
    double SpecificImpulse = 900.0;

    /**
    * Индекс стартового тела (0 — звезда, N — планета).
    *
    * Старт со звезды соответствует низкой орбите у её поверхности, что требует
    * чрезмерного расхода топлива. Практичный выбор — старт с планеты.
    */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Spacecraft", meta = (ClampMin = "0"))
    int32 StartingBodyIndex = 1;

    /** Высота начальной орбиты над поверхностью стартового тела, км. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Spacecraft", meta = (Units = "km"))
    double StartingOrbitAltitude = 400.0;

    /** Начальная фаза на стартовой орбите, радианы. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Spacecraft", meta = (Units = "rad"))
    double InitialPhase = 0.0;

    /** Начальная скорость относительно стартового тела, км/с. Ноль — круговая по радиусу орбиты. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Spacecraft", meta = (Units = "km"))
    double InitialVelocity = 0.0;

    /** Ускорять ли начальный вектор скорости до круговой скорости, если InitialVelocity == 0. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Spacecraft")
    bool bInitialOrbitCircular = true;
};

/**
 * Оценка одного критерия оптимизации для маршрута.
 *
 * Позволяет интерфейсу показать вклад каждого критерия в итоговую стоимость
 * без знания конкретных типов критериев.
 */
USTRUCT(BlueprintType)
struct FCriterionScore
{
    GENERATED_BODY()

    /** Отображаемое имя критерия, например "Fuel". */
    UPROPERTY(BlueprintReadOnly, Category = "Route")
    FString CriterionName;

    /** Вес критерия в итоговой стоимости. */
    UPROPERTY(BlueprintReadOnly, Category = "Route")
    double Weight = 0.0;

    /** Оценка критерия в его собственной нормировке. */
    UPROPERTY(BlueprintReadOnly, Category = "Route")
    double Cost = 0.0;

    /** Вклад критерия в итоговую стоимость: Weight * Cost. */
    UPROPERTY(BlueprintReadOnly, Category = "Route")
    double WeightedCost = 0.0;
};

/**
 * Манёвр маршрута: переход от одного небесного тела к другому.
 *
 * Описывает физику перехода. Стоимость здесь не хранится — её вычисляют
 * критерии оптимизации (URouteCriterion), чтобы логика оценки не была
 * зашита в структуру маршрута.
 */
USTRUCT(BlueprintType)
struct FManeuver
{
    GENERATED_BODY()

    /** Индекс начального тела. */
    UPROPERTY(BlueprintReadOnly, Category = "Route")
    int32 FromBodyIndex = INDEX_NONE;

    /** Индекс конечного тела. Отрицательное значение — граница системы. */
    UPROPERTY(BlueprintReadOnly, Category = "Route")
    int32 ToBodyIndex = INDEX_NONE;

    /** Гелиоцентрический радиус начального тела, км. */
    UPROPERTY(BlueprintReadOnly, Category = "Route", meta = (Units = "km"))
    double StartRadius = 0.0;

    /** Гелиоцентрический радиус конечного тела, км. */
    UPROPERTY(BlueprintReadOnly, Category = "Route", meta = (Units = "km"))
    double EndRadius = 0.0;

    /** Суммарный расход скорости на манёвре, км/с. */
    UPROPERTY(BlueprintReadOnly, Category = "Route", meta = (Units = "km"))
    double DeltaV = 0.0;

    /** Импульс на покидание начального тела, км/с. */
    UPROPERTY(BlueprintReadOnly, Category = "Route", meta = (Units = "km"))
    double DeltaVDeparture = 0.0;

    /** Импульс на захват в орбиту конечного тела, км/с. Ноль для перелёта к границе. */
    UPROPERTY(BlueprintReadOnly, Category = "Route", meta = (Units = "km"))
    double DeltaVArrival = 0.0;

    /** Время перехода на манёвре, с. */
    UPROPERTY(BlueprintReadOnly, Category = "Route", meta = (Units = "s"))
    double DurationSeconds = 0.0;

    /**
     * Задержка перед импульсом отхода, с.
     *
     * Для планетных участков планетой это время ожидания на стартовой
     * парковочной орбите, пока не наступит фазирование с целью (departure
     * window). Для участков к границе системы и стартового участка равно 0.
     */
    UPROPERTY(BlueprintReadOnly, Category = "Route", meta = (Units = "s"))
    double DelaySeconds = 0.0;

    /**
     * Гелиоцентрическая скорость аппарата на дуге Ламберта в точке отбытия, км/с.
     *
     * Нулевой вектор — переход не решался через Ламберт (Hohmann/граница).
     */
    UPROPERTY(BlueprintReadOnly, Category = "Route", meta = (Units = "km"))
    FVector DepartureVelocityKmS = FVector::ZeroVector;

    /**
     * Гелиоцентрическая скорость аппарата на дуге Ламберта в точке прибытия, км/с.
     *
     * Нулевой вектор — переход не решался через Ламберт (Hohmann/граница).
     */
    UPROPERTY(BlueprintReadOnly, Category = "Route", meta = (Units = "km"))
    FVector ArrivalVelocityKmS = FVector::ZeroVector;

    /**
     * Высота ближайшего сближения с поверхностью конечного тела, км.
     *
     * Для тела назначения это высота парковочной орбиты, на которую
     * выполняется захват: именно там аппарат оказывается ближе всего.
     * Отрицательное значение означает перелёт к границе системы, где
     * сближения с телом нет.
     */
    UPROPERTY(BlueprintReadOnly, Category = "Route", meta = (Units = "km"))
    double ApproachAltitudeKm = -1.0;

    /** Радиус конечного тела, км. Позволяет выражать сближение в радиусах тела. */
    UPROPERTY(BlueprintReadOnly, Category = "Route", meta = (Units = "km"))
    double TargetBodyRadiusKm = 0.0;

    /**
     * Относительная скорость аппарата и тела назначения в точке сближения, км/с.
     *
     * Это избыточная (гиперболическая) скорость входа в сферу влияния:
     * |v_аппарата − v_тела| на подлёте. Ноль — сближения нет (перелёт к
     * границе системы) либо данные недоступны.
     */
    UPROPERTY(BlueprintReadOnly, Category = "Route", meta = (Units = "km"))
    double ApproachRelativeVelocityKmS = 0.0;

    /** Признак использования биэллиптического перехода. */
    UPROPERTY(BlueprintReadOnly, Category = "Route")
    bool bBiElliptic = false;
};

/**
 * Данные маршрута, передаваемые критериям оптимизации.
 *
 * Содержит как агрегаты по всему маршруту, так и список манёвров:
 * критерий может оценивать маршрут целиком либо разбирать его по манёврам.
 * Список передаётся не копией, а представлением — контекст создаётся для
 * каждой оценки в цикле поиска, и копировать манёвры было бы лишним расходом.
 *
 * Контекст бывает частичным: во время поиска beam search критерии
 * оценивают ещё не завершённые варианты, поэтому агрегаты описывают
 * пройденную часть маршрута.
 *
 * Структура намеренно не отражена в UHT: это внутренний вход функции оценки,
 * а не данные для сериализации.
 */
struct FRouteEvaluationContext
{
    /** Суммарный расход скорости по маршруту, км/с. */
    double TotalDeltaV = 0.0;

    /** Суммарное время перелёта по маршруту, с. */
    double TotalTimeSeconds = 0.0;

    /**
     * Ближайшее сближение с поверхностью посещённых тел, км.
     *
     * Отрицательное значение — сближения не было (все манёвры уходили
     * к границе системы). Ноль означает касание поверхности, поэтому
     * признаком отсутствия данных служит именно отрицательное значение:
     * при нуле пролёт максимально опасен, и затирать его нулём нельзя.
     */
    double ClosestApproachKm = -1.0;

    /**
     * Радиус тела в точке ближайшего сближения, км.
     * Позволяет критериям выражать безопасную дистанцию в радиусах тела.
     */
    double ClosestApproachBodyRadius = 0.0;

    /**
     * Относительная скорость в точке ближайшего сближения, км/с.
     * Ноль — сближения не было либо данные недоступны.
     */
    double ClosestApproachRelativeVelocityKmS = 0.0;

    /**
     * Полный располагаемый запас скорости аппарата, км/с.
     *
     * Служит для оценки резерва топлива на коррекцию траектории:
     * остаток = AvailableDeltaVKmS − TotalDeltaV. Значение <= 0 означает,
     * что данные о бюджете недоступны, и вклад риска по топливу не начисляется.
     */
    double AvailableDeltaVKmS = 0.0;

    /** Индекс стартового тела. */
    int32 StartBodyIndex = INDEX_NONE;

    /** Число манёвров в маршруте. */
    int32 ManeuverCount = 0;

    /** Признак, что маршрут доведён до границы системы. */
    bool bReachesBoundary = false;

    /** Манёвры маршрута. Не владеет памятью. */
    TArrayView<const FManeuver> Maneuvers;

    /** Полный ли маршрут: false во время промежуточных оценок в beam search. */
    bool bComplete = true;
};

/**
 * Результат планирования маршрута.
 */
USTRUCT(BlueprintType)
struct FRoutePlan
{
    GENERATED_BODY()

    /** Найден ли хотя бы один маршрут. */
    UPROPERTY(BlueprintReadOnly, Category = "Route")
    bool bIsValid = false;

    /** Последовательность манёвров. */
    UPROPERTY(BlueprintReadOnly, Category = "Route")
    TArray<FManeuver> Maneuvers;

    /** Индексы посещённых тел, начиная со стартового. */
    UPROPERTY(BlueprintReadOnly, Category = "Route")
    TArray<int32> VisitedBodyIndices;

    /** Суммарный расход скорости, км/с. */
    UPROPERTY(BlueprintReadOnly, Category = "Route", meta = (Units = "km"))
    double TotalDeltaV = 0.0;

    /** Суммарное время полёта, с. */
    UPROPERTY(BlueprintReadOnly, Category = "Route", meta = (Units = "s"))
    double TotalTime = 0.0;

    /** Суммарная требуемая масса топлива, кг. */
    UPROPERTY(BlueprintReadOnly, Category = "Route", meta = (Units = "kg"))
    double RequiredFuelMass = 0.0;

    /**
     * Вклад каждого критерия в итоговую стоимость маршрута.
     *
     * Заполняется планировщиком и позволяет интерфейсу показать, за счёт чего
     * маршрут оказался лучше, не зная типов критериев.
     */
    UPROPERTY(BlueprintReadOnly, Category = "Route")
    TArray<FCriterionScore> CriterionScores;

    /** Итоговая стоимость: сумма Weight * Cost по всем критериям. */
    UPROPERTY(BlueprintReadOnly, Category = "Route")
    double TotalCost = 0.0;

    /** Число рассмотренных вариантов (для отладки и бенчмарка). */
    UPROPERTY(BlueprintReadOnly, Category = "Route|Debug")
    int32 EvaluatedVariants = 0;
};

/**
 * Критерий остановки поиска маршрута.
 */
USTRUCT(BlueprintType)
struct FPlannerLimits
{
    GENERATED_BODY()

    /** Максимальное число участков в маршруте. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Planner", meta = (ClampMin = "1", ClampMax = "16"))
    int32 MaxLegs = 5;

    /** Максимальное число вариантов, удерживаемых в пучке на каждом шаге. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Planner", meta = (ClampMin = "1"))
    int32 BeamWidth = 64;

    /** Максимальное время полёта, с. Ноль — ограничение не проверяется. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Planner", meta = (Units = "s"))
    double MaxFlightTime = 0.0;

    /**
     * Разрешить переходы к внутренним планетам (гравитационные помощники).
     *
     * В текущей модели каждый перелёт к телу платит и уход, и захват, поэтому
     * движение внутрь не даёт выигрыша по топливу. Флаг оставлен как точка
     * расширения под будущую модель гравитационных манёвров.
     */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Planner")
    bool bAllowInwardTransfers = true;

    // Перелёт к границе системы разрешён с любого тела: жёсткого топологического
    // ограничения нет. Раньше здесь были bAllowBoundaryFromAnyBody и
    // BoundaryUnlockOutermostBodies, заставлявшие сначала дойти до внешних планет.
    // Теперь «дороговизну» большого одиночного перелёта задаёт критерий риска
    // (см. URiskCriterion.JumpWeight): далёкий прыжок рискует больше, поэтому в
    // риск-доминирующем режиме планировщик сам предпочтёт промежуточные участки.
};