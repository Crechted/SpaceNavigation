// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "Kismet/BlueprintFunctionLibrary.h"
#include "SimConstants.h"
#include "TwoBodyPropagator.generated.h"

#pragma region TwoBodyTypes

/**
 * Состояние тела в задаче двух тел.
 * Позиция — км, скорость — км/с.
 */
USTRUCT(BlueprintType)
struct FTwoBodyState
{
    GENERATED_BODY()

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Two Body", meta = (Units = "km"))
    FVector PositionKm = FVector::ZeroVector;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Two Body", meta = (Units = "km"))
    FVector VelocityKmS = FVector::ZeroVector;

    /** Признак валидного состояния. */
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Two Body")
    bool bValid = false;
};

/**
 * Орбитальные элементы двухтельной орбиты.
 */
USTRUCT(BlueprintType)
struct FOrbitalElements
{
    GENERATED_BODY()

    /** Большая полуось, км. Отрицательна для гиперболической орбиты. */
    UPROPERTY(BlueprintReadOnly, Category = "Elements", meta = (Units = "km"))
    double SemiMajorAxisKm = 0.0;

    /** Масса центрального тела, кг. Нужна для восстановления скоростей. */
    UPROPERTY(BlueprintReadOnly, Category = "Elements", meta = (Units = "kg"))
    double CentralMassKg = 0.0;

    /** Эксцентриситет. */
    UPROPERTY(BlueprintReadOnly, Category = "Elements")
    double Eccentricity = 0.0;

    /** Наклон орбиты, радианы. */
    UPROPERTY(BlueprintReadOnly, Category = "Elements", meta = (Units = "rad"))
    double Inclination = 0.0;

    /** Радиус перицентра, км. */
    UPROPERTY(BlueprintReadOnly, Category = "Elements", meta = (Units = "km"))
    double PeriapsisKm = 0.0;

    /** Радиус апоцентра, км. Может быть отрицательным для гиперболы. */
    UPROPERTY(BlueprintReadOnly, Category = "Elements", meta = (Units = "km"))
    double ApoapsisKm = 0.0;

    /** Средняя скорость изменения истинной аномалии, рад/с. */
    UPROPERTY(BlueprintReadOnly, Category = "Elements", meta = (Units = "rad/s"))
    double MeanMotion = 0.0;

    /** Период обращения, с. Для гиперболической орбиты — 0. */
    UPROPERTY(BlueprintReadOnly, Category = "Elements", meta = (Units = "s"))
    double PeriodSeconds = 0.0;

    /** Признак эллиптической орбиты. */
    UPROPERTY(BlueprintReadOnly, Category = "Elements")
    bool bElliptic = true;

    /**
     * Признак вырожденной (практически параболической) орбиты.
     *
     * Для неё большая полуось стремится к бесконечности, поэтому аналитическое
     * решение Кеплера неприменимо: PropagateOrbit переключается на численное
     * интегрирование. Значения элементов при этом остаются приблизительными.
     */
    UPROPERTY(BlueprintReadOnly, Category = "Elements")
    bool bNearParabolic = false;
};

#pragma endregion

#pragma region TwoBodyPropagatorLibrary

/**
 * Аналитическая propagация двухтельной задачи (решение уравнения Кеплера).
 *
 * Используется для участков полёта без работающего двигателя: это точнее
 * и быстрее численного интегрирования, а главное — допускает крупные шаги по
 * времени, что необходимо для перелётов длиной в годы.
 */
UCLASS()
class SPACENAVIGATION_API UTwoBodyPropagator : public UBlueprintFunctionLibrary
{
    GENERATED_BODY()

public:
#pragma region PropagationFunctions

    /**
     * Перевести состояние на DeltaTimeSeconds вперёд по кеплеровской орбите
     * вокруг тела массой CentralMassKg.
     *
     * Для практически параболических траекторий, где аналитическое решение
     * вырождается, используется численное интегрирование Рунге-Кутты.
     *
     * @param PositionKm         начальная позиция, км
     * @param VelocityKmS        начальная скорость, км/с
     * @param CentralMassKg      масса центрального тела, кг
     * @param DeltaTimeSeconds   время полёта, с
     * @param OutState           состояние в конце интервала
     * @return false, если состояние некорректно (нулевая масса, вырожденная орбита)
     */
    UFUNCTION(BlueprintCallable, Category = "Two Body")
    static bool PropagateOrbit(
        const FVector& PositionKm, const FVector& VelocityKmS, double CentralMassKg, double DeltaTimeSeconds, FTwoBodyState& OutState);

    /**
     * Численно перевести состояние вперёд методом Рунге-Кутты 4-го порядка.
     *
     * Шаг подбирается так, чтобы смещение не превышало
     * NumericalStepRadiusFraction от текущего радиуса.
     */
    UFUNCTION(BlueprintCallable, Category = "Two Body")
    static bool PropagateOrbitNumerically(
        const FVector& PositionKm, const FVector& VelocityKmS, double CentralMassKg, double DeltaTimeSeconds, FTwoBodyState& OutState);

#pragma endregion

#pragma region ElementsFunctions

    /** Разложить состояние на орбитальные элементы. */
    UFUNCTION(BlueprintPure, Category = "Two Body")
    static bool StateToElements(const FVector& PositionKm, const FVector& VelocityKmS, double CentralMassKg, FOrbitalElements& OutElements);

    /** Построить состояние из элементов на заданную истинную аномалию. */
    UFUNCTION(BlueprintPure, Category = "Two Body")
    static bool ElementsToState(const FOrbitalElements& Elements, double TrueAnomalyRad, FTwoBodyState& OutState);

    /** Радиус орбиты по элементам и истинной аномалии, км. */
    UFUNCTION(BlueprintPure, Category = "Two Body")
    static double GetOrbitRadiusKm(const FOrbitalElements& Elements, double TrueAnomalyRad);

    /** Удельная энергия, км^2/с^2. */
    UFUNCTION(BlueprintPure, Category = "Two Body")
    static double GetSpecificEnergy(double SpeedKmS, double RadiusKm, double CentralMassKg);

#pragma endregion
};

#pragma endregion
