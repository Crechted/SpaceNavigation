// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "Importer.h"
#include "Kismet/BlueprintFunctionLibrary.h"
#include "UObject/Object.h"
#include "SimTypes.h"
#include "RouteCriterion.generated.h"

/**
 * Абстрактный критерий оптимизации маршрута.
 *
 * Планировщик хранит массив критериев и в точке оценки проходит по всем:
 *
 *     TotalCost = sum(Weight_i * EvaluateRoute_i)
 *
 * Каждый критерий владеет собственным весом и собственной нормировкой,
 * поэтому планировщику не нужно знать, что именно оценивает конкретный
 * критерий. Расширение сводится к наследованию от этого класса — switch
 * по видам критериев в коде отсутствует.
 *
 * Два уровня оценки:
 *  - EvaluateManeuver() — оценка одного манёвра;
 *  - EvaluateRoute()    — оценка всего маршрута.
 *
 * Планировщик вызывает только EvaluateRoute(). Реализация по умолчанию
 * берёт среднее по манёврам, поэтому простые критерии (например, "суммарный
 * расход топлива") могут переопределить только её. Критерий, которому нужны
 * невыразимые через сумму величины — число манёвров, ближайшее сближение —
 * переопределяет EvaluateRoute() целиком.
 *
 * Требования к реализации:
 *  - детерминированность: оценка зависит только от контекста, без времени
 *    и случайных величин (по ТЗ один seed даёт одну и ту же конфигурацию);
 *  - отсутствие внутреннего состояния между вызовами;
 *  - нормировка внутри диапазона [0; 1], чтобы веса разных критериев были
 *    сравнимы между собой.
 *
 * Расширение намеренно сделано только на C++: EvaluateManeuver() и
 * EvaluateRoute() — обычные virtual-методы, не UFUNCTION. Blueprint-подкласс
 * не смог бы их переопределить, поэтому классы не помечены Blueprintable,
 * чтобы не обещать неработающую точку расширения.
 */
UCLASS(Abstract, BlueprintType)
class SPACENAVIGATION_API URouteCriterion : public UObject, public IImportable
{
    GENERATED_BODY()

public:
    /** Вес критерия в итоговой стоимости маршрута. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Criterion", meta = (ClampMin = "0.0"))
    double Weight = 1.0;

#pragma region ImportedParameters

    /**
     * Параметры критерия для передачи/восстановления: вес (общий для всех
     * критериев) плюс параметры подкласса. Подклассы добавляют свои параметры
     * к результату базовой реализации.
     */
    virtual FParameters ExtractParameters_Implementation() override;
    virtual bool ApplyParameters_Implementation(const FParameters& InParameters) override;

#pragma endregion

    /**
     * Оценка одного манёвра.
     *
     * Вызывается реализацией EvaluateRoute() по умолчанию, а также может
     * вызываться напрямую для отладочного разбора маршрута.
     *
     * @param Maneuver  манёвр для оценки
     * @param Context   агрегаты маршрута, которому принадлежит манёвр
     * @return оценка в нормировке критерия, обычно [0; 1]
     */
    virtual double EvaluateManeuver(const FManeuver& Maneuver, const FRouteEvaluationContext& Context) const;

    /**
     * Оценка всего маршрута.
     *
     * Базовая реализация возвращает среднее значение EvaluateManeuver()
     * по всем манёврам контекста.
     */
    virtual double EvaluateRoute(const FRouteEvaluationContext& Context) const;

    /** Короткое имя критерия для интерфейса и отладочного вывода. */
    virtual FString GetCriterionName() const;
};

/**
 * Критерий расхода топлива: минимизация суммарного DeltaV.
 *
 * Нормировка не влияет на порядок маршрутов внутри одного запуска поиска
 * (это положительное линейное преобразование), но нужна, чтобы вес критерия
 * был сравним с весами остальных в комбинированной оценке.
 */
UCLASS(EditInlineNew, DefaultToInstanced)
class SPACENAVIGATION_API UFuelCriterion : public URouteCriterion
{
    GENERATED_BODY()

#pragma region CriterionProperties

public:
    /**
     * Опорный DeltaV для нормировки, км/с.
     * Значение <= 0 подставляет SimConstants-ориентир по умолчанию.
     */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Criterion|Fuel", meta = (Units = "km"))
    double DeltaVReference = 10.0;
#pragma endregion

#pragma region OverridedFunctions

public:
    virtual FParameters ExtractParameters_Implementation() override;
    virtual bool ApplyParameters_Implementation(const FParameters& InParameters) override;

    virtual double EvaluateManeuver(const FManeuver& Maneuver, const FRouteEvaluationContext& Context) const override;
    virtual double EvaluateRoute(const FRouteEvaluationContext& Context) const override;
    virtual FString GetCriterionName() const override;

#pragma endregion
};

/**
 * Критерий времени полёта: минимизация суммарного времени перелётов.
 */
UCLASS(EditInlineNew, DefaultToInstanced)
class SPACENAVIGATION_API UTimeCriterion : public URouteCriterion
{
    GENERATED_BODY()

#pragma region CriterionProperties

public:
    /**
     * Опорное время полёта для нормировки, с.
     * Значение <= 0 подставляет ориентир по умолчанию (5 лет).
     */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Criterion|Time", meta = (Units = "s"))
    double TimeReference = SimConstants::SecondsPerYear * 5.0;
#pragma endregion

#pragma region OverridedFunctions

public:
    virtual FParameters ExtractParameters_Implementation() override;
    virtual bool ApplyParameters_Implementation(const FParameters& InParameters) override;

    virtual double EvaluateManeuver(const FManeuver& Maneuver, const FRouteEvaluationContext& Context) const override;
    virtual double EvaluateRoute(const FRouteEvaluationContext& Context) const override;
    virtual FString GetCriterionName() const override;

#pragma endregion
};

/**
 * Критерий риска: оценка в диапазоне [0; 1], где 0 — минимальный риск.
 *
 * Модель соответствует ТЗ: риск отражает, насколько траектория манёвра близка
 * к недопустимым условиям и насколько она чувствительна к ошибкам выполнения.
 * Учитываются четыре фактора:
 *
 *  1. минимальное расстояние до небесного тела (глубина сближения);
 *  2. относительная скорость при сближении (энергия входа в сферу влияния);
 *  3. допустимая погрешность параметров манёвра — аппроксимируется временем
 *     перелёта: чем дольше пассивный участок, тем сильнее малая ошибка
 *     выведения накапливается к точке встречи и тем меньше допустимая ошибка;
 *  4. дальность одиночного перелёта — отношение гелиоцентрических радиусов
 *     участка. Большой «прыжок» через несколько орбит физически исполним, но
 *     очень чувствителен к ошибкам и без промежуточных тел нечем страховать
 *     коррекцию, поэтому он считается рискованным даже если выгоден по
 *     топливу и времени. Именно этот фактор в риск-доминирующем режиме
 *     заставляет планировщик разбивать далёкий перелёт на промежуточные;
 *  5. доступный запас топлива для коррекции траектории (остаток бюджета deltaV).
 *
 * Первые четыре фактора оцениваются по каждому манёвру отдельно
 * (EvaluateManeuver), а на уровне маршрута берётся ХУДШИЙ манёвр (максимум,
 * а не среднее): трасса опасна настолько, насколько опасен её самый
 * рискованный участок. Запас топлива — свойство маршрута в целом.
 * Дополнительно сохраняется штраф за число манёвров: каждый импульс —
 * отдельный источник отказов.
 *
 * Погрешность параметров оценивается по длительности ОТДЕЛЬНОГО манёвра,
 * а не по суммарному времени полёта: короткий перелёт с одним длинным плечом
 * и длинный маршрут из коротких плеч должны различаться.
 *
 * Жёсткие ограничения (столкновение, превышение предельной скорости)
 * по ТЗ должны не повышать стоимость, а признавать манёвр недопустимым;
 * такой отбор выполняется в планировщике, а не здесь.
 */
UCLASS(EditInlineNew, DefaultToInstanced)
class SPACENAVIGATION_API URiskCriterion : public URouteCriterion
{
    GENERATED_BODY()
#pragma region CriterionProperties

public:
    /** Доля глубины сближения с телами в оценке риска манёвров. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Criterion|Risk", meta = (ClampMin = "0.0", ClampMax = "1.0"))
    double ProximityWeight = 0.25;

    /** Доля относительной скорости сближения в оценке риска манёвров. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Criterion|Risk", meta = (ClampMin = "0.0", ClampMax = "1.0"))
    double ApproachVelocityWeight = 0.20;

    /** Доля чувствительности манёвра к ошибке (длительность плеча) в оценке риска манёвров. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Criterion|Risk", meta = (ClampMin = "0.0", ClampMax = "1.0"))
    double ToleranceWeight = 0.15;

    /**
     * Доля дальности одиночного перелёта в оценке риска манёвров.
     *
     * Это и есть «ограничение одного участка как параметр риска»: чем больше
     * отношение гелиоцентрических радиусов участка, тем выше риск, поэтому
     * большой прыжок стоит дорого даже при выгодном deltaV.
     */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Criterion|Risk", meta = (ClampMin = "0.0", ClampMax = "1.0"))
    double JumpWeight = 0.30;

    /** Доля недостатка топлива на коррекцию в итоговой оценке риска. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Criterion|Risk", meta = (ClampMin = "0.0", ClampMax = "1.0"))
    double FuelReserveWeight = 0.05;

    /** Доля числа манёвров в итоговой оценке риска. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Criterion|Risk", meta = (ClampMin = "0.0", ClampMax = "1.0"))
    double ManeuverCountWeight = 0.05;

    /** Число манёвров, при котором штраф за количество считается максимальным. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Criterion|Risk", meta = (ClampMin = "1"))
    int32 ReferenceManeuverCount = 8;

    /** Высота сближения, считающаяся полностью безопасной (в радиусах тела). */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Criterion|Risk", meta = (ClampMin = "1.0"))
    double SafeProximityBodyRadii = 20.0;

    /** Относительная скорость сближения, считающаяся максимально рискованной, км/с. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Criterion|Risk", meta = (Units = "km", ClampMin = "0.0"))
    double ReferenceApproachVelocity = 5.0;

    /**
     * Отношение радиусов одиночного перелёта, при котором риск дальности
     * считается максимальным. Больше — мягче к длинным перелётам. Значение <= 1
     * подставляет ориентир (10).
     */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Criterion|Risk", meta = (ClampMin = "0.0"))
    double ReferenceJumpRadiusRatio = 10.0;

    /**
     * Длительность отдельного манёвра, при которой чувствительность к ошибке
     * считается максимальной, с. Значение <= 0 подставляет ориентир (5 лет).
     */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Criterion|Risk", meta = (Units = "s"))
    double ReferenceManeuverDuration = SimConstants::SecondsPerYear * 5.0;

    /**
     * Запас deltaV, при котором топливо на коррекцию считается достаточным, км/с.
     * Риск по топливу равен 0, когда остаток бюджета не меньше этого значения,
     * и 1, когда остаток исчерпан. Значение <= 0 подставляет ориентир.
     */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Criterion|Risk", meta = (Units = "km", ClampMin = "0.0"))
    double ReferenceFuelReserve = 2.0;

#pragma endregion

#pragma region OverridedFunctions

public:
    virtual FParameters ExtractParameters_Implementation() override;
    virtual bool ApplyParameters_Implementation(const FParameters& InParameters) override;

    virtual double EvaluateManeuver(const FManeuver& Maneuver, const FRouteEvaluationContext& Context) const override;
    virtual double EvaluateRoute(const FRouteEvaluationContext& Context) const override;
    virtual FString GetCriterionName() const override;
#pragma endregion
};

/**
 * Готовые наборы критериев — пресеты трёх режимов из ТЗ.
 *
 * Раньше режимы были значениями enum, и «Balanced» был отдельным случаем
 * в switch. Теперь это просто разные наборы критериев, поэтому пресеты —
 * обычные функции, создающие нужные объекты.
 */
UCLASS()
class SPACENAVIGATION_API URouteCriterionPresets : public UBlueprintFunctionLibrary
{
    GENERATED_BODY()

public:
    /** Пресет "Fuel Efficient": один критерий расхода топлива с весом 1. */
    UFUNCTION(BlueprintCallable, Category = "Route|Criterion")
    static TArray<URouteCriterion*> MakeFuelEfficient(UObject* Outer);

    /** Пресет "Time Efficient": один критерий времени с весом 1. */
    UFUNCTION(BlueprintCallable, Category = "Route|Criterion")
    static TArray<URouteCriterion*> MakeTimeEfficient(UObject* Outer);

    /** Пресет "Balanced" из ТЗ: топливо 0.6, время 0.3, риск 0.1. */
    UFUNCTION(BlueprintCallable, Category = "Route|Criterion")
    static TArray<URouteCriterion*> MakeBalanced(UObject* Outer);

    /**
     * Пресет "Risk": только критерий риска с весом 1.
     *
     * Дополнительно к трём режимам ТЗ: нужен, чтобы наблюдать, как риск сам
     * ограничивает дальность одиночного перелёта и вынуждает промежуточные
     * участки. Стоимость здесь — чистый риск, топливо и время не учитываются.
     */
    UFUNCTION(BlueprintCallable, Category = "Route|Criterion")
    static TArray<URouteCriterion*> MakeRiskDominant(UObject* Outer);

    /**
     * Пресет по имени пресета.
     * Имена соответствуют режимам ТЗ: "Fuel", "Time", "Balanced".
     * Для неизвестного имени возвращается Balanced.
     */
    UFUNCTION(BlueprintCallable, Category = "Route|Criterion")
    static TArray<URouteCriterion*> MakeByPresetName(UObject* Outer, const FString& PresetName);
};