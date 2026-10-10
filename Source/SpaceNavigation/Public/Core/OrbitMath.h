// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "Kismet/BlueprintFunctionLibrary.h"
#include "SimConstants.h"
#include "OrbitMath.generated.h"

/**
 * Результат расчёта перехода Хохмана между двумя круговыми орбитами.
 */
USTRUCT(BlueprintType)
struct FHohmannTransfer
{
	GENERATED_BODY()

	/** Скорость на начальной круговой орбите, км/с. */
	UPROPERTY(BlueprintReadOnly, Category = "Transfer")
	double VelocityStart = 0.0;

	/** Скорость на конечной круговой орбите, км/с. */
	UPROPERTY(BlueprintReadOnly, Category = "Transfer")
	double VelocityEnd = 0.0;

	/** Скорость в перицентре переходной орбиты, км/с. */
	UPROPERTY(BlueprintReadOnly, Category = "Transfer")
	double TransferVelocityStart = 0.0;

	/** Скорость в апоцентре переходной орбиты, км/с. */
	UPROPERTY(BlueprintReadOnly, Category = "Transfer")
	double TransferVelocityEnd = 0.0;

	/** Импульс в перицентре, км/с. */
	UPROPERTY(BlueprintReadOnly, Category = "Transfer")
	double DeltaVStart = 0.0;

	/** Импульс в апоцентре, км/с. */
	UPROPERTY(BlueprintReadOnly, Category = "Transfer")
	double DeltaVEnd = 0.0;

	/** Суммарный расход скорости, км/с. */
	UPROPERTY(BlueprintReadOnly, Category = "Transfer")
	double TotalDeltaV = 0.0;

	/** Время перехода, с. */
	UPROPERTY(BlueprintReadOnly, Category = "Transfer")
	double TransferTime = 0.0;
};

/**
 * Результат решения задачи Ламберта (перелёт по кеплеровской дуге
 * между двумя точками за заданное время).
 */
USTRUCT(BlueprintType)
struct FLambertSolution
{
	GENERATED_BODY()

	/** Скорость аппарата в начале дуги (гелиоцентрическая), км/с. */
	UPROPERTY(BlueprintReadOnly, Category = "Transfer")
	FVector DepartureVelocityKmS = FVector::ZeroVector;

	/** Скорость аппарата в конце дуги (гелиоцентрическая), км/с. */
	UPROPERTY(BlueprintReadOnly, Category = "Transfer")
	FVector ArrivalVelocityKmS = FVector::ZeroVector;

	/** Большая полуось переходного эллипса, км. */
	UPROPERTY(BlueprintReadOnly, Category = "Transfer")
	double SemiMajorAxisKm = 0.0;

	/** Эксцентриситет переходной орбиты (0 — круг). */
	UPROPERTY(BlueprintReadOnly, Category = "Transfer")
	double Eccentricity = 0.0;

	/** Достигнутое время перелёта, с (равно запрошенному). */
	UPROPERTY(BlueprintReadOnly, Category = "Transfer")
	double TransferTime = 0.0;

	/** Угол перелёта между радиус-векторами в плоскости дуги, рад. */
	UPROPERTY(BlueprintReadOnly, Category = "Transfer")
	double DeltaAnomaly = 0.0;

	/** true, если решение найдено. */
	UPROPERTY(BlueprintReadOnly, Category = "Transfer")
	bool bValid = false;
};

/**
 * Упрощённая орбитальная математика (patched conics).
 *
 * Все функции используют согласованную систему единиц из SimConstants:
 * расстояние — км, масса — кг, скорость — км/с, время — с.
 */
UCLASS()
class SPACENAVIGATION_API UOrbitMath : public UBlueprintFunctionLibrary
{
	GENERATED_BODY()

public:
	/** Гравитационное ускорение, км/с^2. */
	UFUNCTION(BlueprintPure, Category = "Orbit Math")
	static double GravitationalAcceleration(double MassKg, double DistanceKm);

	/** Круговая орбитальная скорость, км/с. */
	UFUNCTION(BlueprintPure, Category = "Orbit Math")
	static double CircularOrbitalVelocity(double MassKg, double RadiusKm);

	/** Скорость убегания, км/с. */
	UFUNCTION(BlueprintPure, Category = "Orbit Math")
	static double EscapeVelocity(double MassKg, double RadiusKm);

	/**
	 * Сфера влияния по формуле Лапласа: r_SOI = a * (m/M)^(2/5).
	 * Если массы неположительны или тело слишком малое — возвращается FallbackRadiusKm.
	 */
	UFUNCTION(BlueprintPure, Category = "Orbit Math")
	static double ComputeSphereOfInfluence(double BodyMassKg, double CentralMassKg, double SemiMajorAxisKm, double FallbackRadiusKm);

	/**
	 * Сфера влияния с учётом порога "малой силы".
	 *
	 * Если заданная (фиксированная) сфера влияния признаётся незначимой —
	 * её значение меньше SimConstants::WeakForceThreshold, — она пересчитывается
	 * по формуле r_SOI = a * (m/M)^(2/5). Иначе используется фиксированное значение.
	 */
	UFUNCTION(BlueprintPure, Category = "Orbit Math")
	static double ComputeSphereOfInfluenceWithThreshold(double BodyMassKg, double CentralMassKg, double SemiMajorAxisKm, double FixedRadiusKm, double WeakForceThreshold);

	/**
	 * Считается ли сила пренебрежимо малой.
	 * Возвращает true, если ускорение ниже SimConstants::GravitySignificanceThreshold.
	 */
	UFUNCTION(BlueprintPure, Category = "Orbit Math")
	static bool IsForceNegligible(double MassKg, double DistanceKm);

	/** Расчёт перехода Хохмана между двумя круговыми орбитами вокруг одного центра. */
	UFUNCTION(BlueprintPure, Category = "Orbit Math")
	static FHohmannTransfer ComputeHohmannTransfer(double CentralMassKg, double RadiusStartKm, double RadiusEndKm);

	/** Суммарный расход скорости для перехода Хохта (или биэллиптического, если он выгоднее). */
	UFUNCTION(BlueprintPure, Category = "Orbit Math")
	static double ComputeOptimalTransferDeltaV(double CentralMassKg, double RadiusStartKm, double RadiusEndKm);

	/** Оптимальное время перехода между орбитами, с. */
	UFUNCTION(BlueprintPure, Category = "Orbit Math")
	static double ComputeTransferTime(double CentralMassKg, double RadiusStartKm, double RadiusEndKm);

	/**
	 * Задача Ламберта: перелёт по кеплеровской дуге между двумя точками
	 * за заданное время.
	 *
	 * @param R1Km — радиус-вектор точки отбытия, км (гелиоцентрический).
	 * @param R2Km — радиус-вектор точки прибытия, км.
	 * @param CentralMassKg — масса центрального тела, кг (мулей центра).
	 * @param TransferTimeSeconds — желаемое время перелёта, с (> 0).
	 *
	 * Решается эллиптическая ветка (0 < e < 1) с прогрейдной дугой
	 * (N̂·ẑ ≥ 0). В вырожденном случае (угол перелёта ровно π)
	 * возвращается эллипс Хомана. Если решения нет — bValid = false.
	 */
	UFUNCTION(BlueprintPure, Category = "Orbit Math")
	static FLambertSolution SolveLambert(FVector R1Km, FVector R2Km, double CentralMassKg, double TransferTimeSeconds);

	/** Требуемая масса топлива под заданный DeltaV по формуле Циолковского, кг. */
	UFUNCTION(BlueprintPure, Category = "Orbit Math")
	static double RequiredFuelMass(double DryMassKg, double DeltaVKmS, double SpecificImpulseSeconds);

	/** Максимальный DeltaV при полном запасе топлива, км/с. */
	UFUNCTION(BlueprintPure, Category = "Orbit Math")
	static double MaxDeltaV(double DryMassKg, double FuelMassKg, double SpecificImpulseSeconds);

	/** Расход топлива в секунду при работе двигателя с заданной тягой, кг/с. */
	UFUNCTION(BlueprintPure, Category = "Orbit Math")
	static double FuelConsumptionRate(double ThrustNewtons, double SpecificImpulseSeconds);

	/** Нормализация значения в диапазон [0; 1] по границам. */
	UFUNCTION(BlueprintPure, Category = "Orbit Math")
	static double NormalizeRange(double Value, double MinValue, double MaxValue);

	/** Безопасное деление: при нулевом знаменателе возвращает запасное. */
	UFUNCTION(BlueprintPure, Category = "Orbit Math")
	static double SafeDivide(double Numerator, double Denominator, double Fallback = 0.0);
};