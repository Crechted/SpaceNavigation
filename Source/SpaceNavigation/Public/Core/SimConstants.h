// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"

/**
 * Согласованная система единиц для всей симуляции.
 *
 * Расстояние  : километры (км)
 * Масса       : килограммы (кг)
 * Время       : секунды (с)
 * Скорость    : км/с
 * Ускорение   : км/с^2
 *
 * Вычисления ведутся в double, чтобы не терять точность на больших
 * межзвёздных расстояниях (float не способен хранить 1.5e8 км с нужной точностью).
 */
namespace SimConstants
{
#pragma region PhysicalConstants

/** Гравитационная постоянная, км^3 / (кг * с^2). */
inline constexpr double GravitationalConstant = 6.67430e-20;

/** Астрономическая единица, км. */
inline constexpr double AstronomicalUnit = 1.495978707e8;

/** Стандартное ускорение свободного падения на поверхности Земли, м/с^2. */
inline constexpr double StandardGravity = 9.80665;

/** Масса Солнца, кг — значение по умолчанию для центральной звезды. */
inline constexpr double SolarMass = 1.98892e30;

/** Радиус Солнца, км — значение по умолчанию для центральной звезды. */
inline constexpr double SolarRadius = 6.957e5;

#pragma endregion

#pragma region TimeConstants

/** Сутки в секундах. */
inline constexpr double SecondsPerDay = 86400.0;

/** Год в секундах (приблизительно, для пересчёта больших интервалов). */
inline constexpr double SecondsPerYear = 3.15576e7;

#pragma endregion

#pragma region Thresholds

/** Порог гравитационного ускорения, ниже которого влияние тела считается пренебрежимо малым, км/с^2. */
inline constexpr double GravitySignificanceThreshold = 1.0e-9;

/** Порог "малой" силы: если значение SOI меньше этой константы, оно считается незначимым и вычисляется по формуле. */
inline constexpr double WeakForceThreshold = 1.0e3;

/** Степень в формуле сферы влияния (Лаплас / Хилл): r_SOI = a * (m/M)^(2/5). */
inline constexpr double SphereOfInfluenceExponent = 2.0 / 5.0;

/** Минимальная масса тела (кг), для которого имеет смысл считать SOI. */
inline constexpr double MinimumBodyMass = 1.0e15;

/**
 * Относительная близость большой полуоси к бесконечности (1/a), при которой
 * орбита считается параболической, а аналитическое решение Кеплера неприменимо.
 * Сравнение идёт с |2/r - v^2/mu| относительно суммы тех же членов.
 */
inline constexpr double NearParabolicTolerance = 1.0e-9;

/** Максимальный шаг численного интегрирования, доля от текущего радиуса. */
inline constexpr double NumericalStepRadiusFraction = 0.01;

/** Ограничение на число шагов численного интегрирования за один вызов. */
inline constexpr int32 MaxNumericalSubSteps = 4096;

/** Наименьшая допустимая скорость при подборе шага интегрирования, км/с. */
inline constexpr double MinSpeedForStepEstimate = 1.0e-9;

#pragma endregion

#pragma region ConversionFunctions

/** AU -> км. */
FORCEINLINE double AuToKm(double Au)
{
    return Au * AstronomicalUnit;
}

/** км -> AU. */
FORCEINLINE double KmToAu(double Km)
{
    return Km / AstronomicalUnit;
}

/** Секунды -> дни. */
FORCEINLINE double SecondsToDays(double Seconds)
{
    return Seconds / SecondsPerDay;
}

#pragma endregion
}  // namespace SimConstants
