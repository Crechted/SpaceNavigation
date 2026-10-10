// Copyright Epic Games, Inc. All Rights Reserved.

#include "Core/TwoBodyPropagator.h"
#include "cmath"

namespace TwoBodyInternal
{
/** Точность сходимости решателя уравнения Кеплера. */
constexpr double AnomalyTolerance = 1.0e-12;

/** Максимальное число итераций решателя. */
constexpr int32 MaxIterations = 64;

/** Ниже этого эксцентриситета орбита считается практически круговой. */
constexpr double CircularEccentricity = 1.0e-9;

/** Отклонение от единицы, при котором орбита считается практически параболической. */
constexpr double NearParabolicDelta = 1.0e-8;

/** Базис плоскости орбиты: P — на перицентр, Q — поперечный, W — нормаль. */
struct FOrbitBasis
{
    FVector P = FVector::ForwardVector;
    FVector Q = FVector::RightVector;
    FVector W = FVector::UpVector;
};

/** Гравитационное ускорение в точке Position, км/с^2. */
FVector AccelerationAt(const FVector& Position, double Mu)
{
    const double DistanceSquared = Position.SizeSquared();
    if (DistanceSquared <= 0.0)
    {
        return FVector::ZeroVector;
    }

    return Position * (-Mu / (DistanceSquared * FMath::Sqrt(DistanceSquared)));
}

/**
 * Построение ортогонального базиса плоскости орбиты.
 * Ось P направлена на перицентр, если эксцентриситет заметен,
 * иначе берётся направление на восходящий узел.
 */
FOrbitBasis BuildBasis(const FVector& PositionKm, const FVector& VelocityKmS, double Eccentricity, const FVector& EccentricityVector)
{
    FOrbitBasis Basis;
    Basis.W = FVector::CrossProduct(PositionKm, VelocityKmS).GetSafeNormal();

    FVector P;
    if (Eccentricity > CircularEccentricity && !EccentricityVector.IsNearlyZero())
    {
        P = EccentricityVector / Eccentricity;
    }
    else
    {
        // Круговая орбита: перицентр не определён, берём направление на восходящий узел.
        const FVector NodeVector = FVector::CrossProduct(FVector::UpVector, Basis.W);
        P = NodeVector.IsNearlyZero() ? FVector::ForwardVector : NodeVector.GetSafeNormal();
    }

    Basis.P = P.GetSafeNormal();
    Basis.Q = FVector::CrossProduct(Basis.W, Basis.P).GetSafeNormal();
    return Basis;
}

/** Решение уравнения Кеплера для эллиптической орбиты. */
bool SolveEllipticAnomaly(double MeanAnomaly, double Eccentricity, double& OutEccentricAnomaly)
{
    double E = MeanAnomaly + Eccentricity * FMath::Sin(MeanAnomaly);

    for (int32 Iteration = 0; Iteration < MaxIterations; ++Iteration)
    {
        const double SinE = FMath::Sin(E);
        const double CosE = FMath::Cos(E);
        const double Residual = E - Eccentricity * SinE - MeanAnomaly;
        const double Derivative = 1.0 - Eccentricity * CosE;

        if (FMath::IsNearlyZero(Derivative, 1.0e-14))
        {
            break;
        }

        const double Delta = Residual / Derivative;
        E -= Delta;

        if (FMath::Abs(Delta) < AnomalyTolerance)
        {
            OutEccentricAnomaly = E;
            return true;
        }
    }

    OutEccentricAnomaly = E;
    return FMath::IsFinite(E);
}

/** Решение аналога уравнения Кеплера для гиперболической орбиты. */
bool SolveHyperbolicAnomaly(double MeanAnomaly, double Eccentricity, double& OutHyperbolicAnomaly)
{
    double H = std::asinh(MeanAnomaly / FMath::Max(Eccentricity, 1.0));

    for (int32 Iteration = 0; Iteration < MaxIterations; ++Iteration)
    {
        const double SinhH = FMath::Sinh(H);
        const double CoshH = FMath::Cosh(H);
        const double Residual = Eccentricity * SinhH - H - MeanAnomaly;
        const double Derivative = Eccentricity * CoshH - 1.0;

        if (FMath::IsNearlyZero(Derivative, 1.0e-14))
        {
            break;
        }

        const double Delta = Residual / Derivative;
        H -= Delta;

        if (FMath::Abs(Delta) < AnomalyTolerance)
        {
            OutHyperbolicAnomaly = H;
            return true;
        }
    }

    OutHyperbolicAnomaly = H;
    return FMath::IsFinite(H);
}

/** Переход от истинной аномалии к эксцентрической аномалии. */
double TrueToEccentricAnomaly(double TrueAnomaly, double Eccentricity)
{
    const double HalfNu = 0.5 * TrueAnomaly;
    const double X = FMath::Sqrt(FMath::Max(1.0 - Eccentricity, 0.0)) * FMath::Sin(HalfNu);
    const double Y = FMath::Sqrt(FMath::Max(1.0 + Eccentricity, 0.0)) * FMath::Cos(HalfNu);
    return 2.0 * FMath::Atan2(X, Y);
}

/** Переход от истинной аномалии к гиперболической аномалии. */
double TrueToHyperbolicAnomaly(double TrueAnomaly, double Eccentricity)
{
    const double CosNu = FMath::Cos(TrueAnomaly);
    const double Denominator = 1.0 + Eccentricity * CosNu;

    if (Denominator <= 0.0)
    {
        // Вырожденный случай: аномалия стремится к бесконечности.
        return FMath::Sign(TrueAnomaly) * 10.0;
    }

    const double CoshH = (Eccentricity + CosNu) / Denominator;
    if (CoshH <= 1.0)
    {
        return 0.0;
    }

    // Знак сохраняем: |nu| симметрично для +/-H, но и средняя аномалия, и
    // обратный переход (sinh H) обязаны знать, по какую сторону от перицентра
    // лежит состояние.
    return FMath::Sign(TrueAnomaly) * std::acosh(FMath::Min(CoshH, 1.0e12));
}

/** Переход от эксцентрической аномалии к истинной (эллипс). */
double EccentricToTrueAnomaly(double EccentricAnomaly, double Eccentricity)
{
    const double SinE = FMath::Sin(EccentricAnomaly);
    const double CosE = FMath::Cos(EccentricAnomaly);
    return FMath::Atan2(FMath::Sqrt(FMath::Max(1.0 - Eccentricity * Eccentricity, 0.0)) * SinE, CosE - Eccentricity);
}

/** Переход от гиперболической аномалии к истинной. */
double HyperbolicToTrueAnomaly(double HyperbolicAnomaly, double Eccentricity)
{
    const double SinhH = FMath::Sinh(HyperbolicAnomaly);
    const double CoshH = FMath::Cosh(HyperbolicAnomaly);
    const double X = Eccentricity - CoshH;
    const double Y = FMath::Sqrt(FMath::Max(Eccentricity * Eccentricity - 1.0, 0.0)) * SinhH;
    return FMath::Atan2(Y, X);
}

/** Уравнение Кеплера в обобщённом виде для эллипса. */
double EllipticMeanAnomaly(double EccentricAnomaly, double Eccentricity)
{
    return EccentricAnomaly - Eccentricity * FMath::Sin(EccentricAnomaly);
}

/** Уравнение Кеплера в обобщённом виде для гиперболы. */
double HyperbolicMeanAnomaly(double HyperbolicAnomaly, double Eccentricity)
{
    return Eccentricity * FMath::Sinh(HyperbolicAnomaly) - HyperbolicAnomaly;
}
}

double UTwoBodyPropagator::GetSpecificEnergy(double SpeedKmS, double RadiusKm, double CentralMassKg)
{
    if (RadiusKm <= 0.0)
    {
        return 0.0;
    }

    return 0.5 * SpeedKmS * SpeedKmS - SimConstants::GravitationalConstant * CentralMassKg / RadiusKm;
}

bool UTwoBodyPropagator::StateToElements(const FVector& PositionKm, const FVector& VelocityKmS, double CentralMassKg,
    FOrbitalElements& OutElements)
{
    using namespace TwoBodyInternal;

    if (CentralMassKg <= 0.0)
    {
        return false;
    }

    const double Radius = PositionKm.Size();
    const double Speed = VelocityKmS.Size();
    if (Radius <= 0.0)
    {
        return false;
    }

    const double Mu = SimConstants::GravitationalConstant * CentralMassKg;
    const FVector AngularMomentum = FVector::CrossProduct(PositionKm, VelocityKmS);
    if (AngularMomentum.IsNearlyZero())
    {
        // Вырожденная (радиальная) орбита: кеплеровская propagация не применима.
        return false;
    }

    const FVector EccentricityVector = FVector::CrossProduct(VelocityKmS, AngularMomentum) / Mu - PositionKm / Radius;
    double Eccentricity = EccentricityVector.Size();

    // Практически круговая орбита: обнуляем эксцентриситет, чтобы избежать вырожденных случаев.
    if (Eccentricity < CircularEccentricity)
    {
        Eccentricity = 0.0;
    }

    // Практически параболическая орбита: слегка смещаем в сторону эллипса или гиперболы.
    if (FMath::Abs(Eccentricity - 1.0) < NearParabolicDelta)
    {
        Eccentricity = Eccentricity < 1.0 ? 1.0 - NearParabolicDelta : 1.0 + NearParabolicDelta;
    }

    // Большая полуось следует из vis-viva: 1/a = 2/r - v^2/mu.
    //
    // На параболе разность обращается в нуль, и прямое деление даёт бесконечность.
    // Тогда орбита ошибочно считалась бы эллиптической (inf > 0) со средней
    // скоростью 0, то есть корабль просто замирал. Поэтому вырожденный случай
    // распознаётся явно и помечается флагом bNearParabolic.
    const double InverseAxisTerm = 2.0 / Radius - (Speed * Speed) / Mu;
    const double InverseAxisScale = 2.0 / Radius + (Speed * Speed) / Mu;

    bool bNearParabolic = FMath::Abs(InverseAxisTerm) <= SimConstants::NearParabolicTolerance * InverseAxisScale;

    double SemiMajorAxis = 0.0;
    if (bNearParabolic)
    {
        // Подставляем конечную величину, согласованную с текущим радиусом,
        // чтобы элементы оставались осмысленными для отладки и UI.
        const double EccentricityOffset = FMath::Max(FMath::Abs(1.0 - Eccentricity), NearParabolicDelta);
        SemiMajorAxis = Radius / EccentricityOffset;
        if (SemiMajorAxis < 0.0)
        {
            SemiMajorAxis = -SemiMajorAxis;
        }
    }
    else
    {
        SemiMajorAxis = 1.0 / InverseAxisTerm;
    }

    if (!FMath::IsFinite(SemiMajorAxis) || FMath::IsNearlyZero(SemiMajorAxis, 1.0e-30))
    {
        return false;
    }

    OutElements.SemiMajorAxisKm = SemiMajorAxis;
    OutElements.CentralMassKg = CentralMassKg;
    OutElements.Eccentricity = Eccentricity;
    OutElements.bNearParabolic = bNearParabolic;
    OutElements.Inclination = FMath::Acos(FMath::Clamp(AngularMomentum.Z / AngularMomentum.Size(), -1.0, 1.0));
    OutElements.PeriapsisKm = SemiMajorAxis * (1.0 - Eccentricity);
    OutElements.bElliptic = SemiMajorAxis > 0.0;
    OutElements.ApoapsisKm = OutElements.bElliptic ? SemiMajorAxis * (1.0 + Eccentricity) : -SemiMajorAxis * (Eccentricity - 1.0);
    OutElements.MeanMotion = FMath::Sqrt(Mu / FMath::Pow(FMath::Abs(SemiMajorAxis), 3.0));
    OutElements.PeriodSeconds = OutElements.bElliptic ? 2.0 * UE_DOUBLE_PI / OutElements.MeanMotion : 0.0;

    return FMath::IsFinite(OutElements.SemiMajorAxisKm) && FMath::IsFinite(OutElements.Eccentricity);
}

bool UTwoBodyPropagator::ElementsToState(const FOrbitalElements& Elements, double TrueAnomalyRad, FTwoBodyState& OutState)
{
    using namespace TwoBodyInternal;

    if (Elements.CentralMassKg <= 0.0 || !FMath::IsFinite(Elements.SemiMajorAxisKm))
    {
        return false;
    }

    const double SemiMajorAxis = Elements.SemiMajorAxisKm;
    const double Eccentricity = Elements.Eccentricity;
    const double Mu = SimConstants::GravitationalConstant * Elements.CentralMassKg;

    double PerifocalX = 0.0;
    double PerifocalY = 0.0;
    double PerifocalVx = 0.0;
    double PerifocalVy = 0.0;
    double Radius = 0.0;

    if (Elements.bElliptic && SemiMajorAxis > 0.0)
    {
        const double EccentricAnomaly = TrueToEccentricAnomaly(TrueAnomalyRad, Eccentricity);
        const double SinE = FMath::Sin(EccentricAnomaly);
        const double CosE = FMath::Cos(EccentricAnomaly);
        const double RootOneMinusE2 = FMath::Sqrt(FMath::Max(1.0 - Eccentricity * Eccentricity, 0.0));

        Radius = SemiMajorAxis * (1.0 - Eccentricity * CosE);
        PerifocalX = SemiMajorAxis * (CosE - Eccentricity);
        PerifocalY = SemiMajorAxis * RootOneMinusE2 * SinE;

        if (Radius <= 0.0)
        {
            return false;
        }

        // vx = -sqrt(Mu*a) * sinE / r, vy = sqrt(Mu*a*(1-e^2)) * cosE / r
        const double VelocityScale = FMath::Sqrt(Mu * SemiMajorAxis);
        PerifocalVx = -VelocityScale * SinE / Radius;
        PerifocalVy = VelocityScale * RootOneMinusE2 * CosE / Radius;
    }
    else
    {
        const double HyperbolicAnomaly = TrueToHyperbolicAnomaly(TrueAnomalyRad, Eccentricity);
        const double SinhH = FMath::Sinh(HyperbolicAnomaly);
        const double CoshH = FMath::Cosh(HyperbolicAnomaly);
        const double MagnitudeA = FMath::Abs(SemiMajorAxis);
        const double RootE2Minus1 = FMath::Sqrt(FMath::Max(Eccentricity * Eccentricity - 1.0, 0.0));

        Radius = MagnitudeA * (Eccentricity * CoshH - 1.0);
        if (Radius <= 0.0)
        {
            return false;
        }

        PerifocalX = MagnitudeA * (Eccentricity - CoshH);
        PerifocalY = MagnitudeA * RootE2Minus1 * SinhH;

        const double VelocityScale = FMath::Sqrt(Mu * MagnitudeA);
        PerifocalVx = -VelocityScale * SinhH / Radius;
        PerifocalVy = VelocityScale * RootE2Minus1 * CoshH / Radius;
    }

    if (!FMath::IsFinite(Radius) || Radius <= 0.0)
    {
        return false;
    }

    OutState.PositionKm = FVector(PerifocalX, PerifocalY, 0.0);
    OutState.VelocityKmS = FVector(PerifocalVx, PerifocalVy, 0.0);
    OutState.bValid = FMath::IsFinite(OutState.PositionKm.X) && FMath::IsFinite(OutState.VelocityKmS.X);

    return OutState.bValid;
}

bool UTwoBodyPropagator::PropagateOrbit(const FVector& PositionKm, const FVector& VelocityKmS, double CentralMassKg,
    double DeltaTimeSeconds, FTwoBodyState& OutState)
{
    using namespace TwoBodyInternal;

    OutState = FTwoBodyState();

    if (CentralMassKg <= 0.0)
    {
        return false;
    }

    const double Radius = PositionKm.Size();
    if (Radius <= 0.0)
    {
        return false;
    }

    // Нулевая длительность: состояние не меняется.
    if (FMath::IsNearlyZero(DeltaTimeSeconds, 1.0e-9))
    {
        OutState.PositionKm = PositionKm;
        OutState.VelocityKmS = VelocityKmS;
        OutState.bValid = true;
        return true;
    }

    FOrbitalElements Elements;
    if (!StateToElements(PositionKm, VelocityKmS, CentralMassKg, Elements))
    {
        return false;
    }

    // Вырожденная (практически параболическая) орбита: большая полуось стремится
    // к бесконечности и аналитическое решение даёт нулевую среднюю скорость.
    // Переключаемся на численное интегрирование.
    if (Elements.bNearParabolic)
    {
        return PropagateOrbitNumerically(PositionKm, VelocityKmS, CentralMassKg, DeltaTimeSeconds, OutState);
    }

    const double Mu = SimConstants::GravitationalConstant * CentralMassKg;
    const FVector AngularMomentum = FVector::CrossProduct(PositionKm, VelocityKmS);
    const FVector EccentricityVector = FVector::CrossProduct(VelocityKmS, AngularMomentum) / Mu - PositionKm / Radius;

    // Базис плоскости орбиты нужен и для аномалии, и для переноса состояния:
    // отсчёт идёт от оси P (перицентр при заметной эксцентричности, иначе
    // узловое направление) в сторону Q = W x P.
    const FOrbitBasis Basis = BuildBasis(PositionKm, VelocityKmS, Elements.Eccentricity, EccentricityVector);

    // Истинная аномалия: atan2(sin, cos), где
    //   cos(nu) = (P . r) / R, sin(nu) = ((P x r) . h) / (|h| R).
    //
    // Раньше знак брался из (r . (e x v)). Это скалярное тройное произведение
    // трёх векторов плоскости орбиты, поэтому оно тождественно равно нулю:
    // знак определялся только округлением и в основном оставался
    // положительным. Как только аномалия становилась отрицательной (после
    // импульса или выхода из сферы влияния ось перицентра поворачивалась),
    // аппарат зеркалился через ось перицентр-апоцентр и прыгал на расстояние
    // до 2r. Для практически круговой орбиты аномалия вообще обнулялась, и
    // аппарат прилипал к узловому направлению.
    const double AnomalyDenominator = AngularMomentum.Size() * Radius;
    const double SinTrueAnomaly = FVector::DotProduct(FVector::CrossProduct(Basis.P, PositionKm), AngularMomentum) / AnomalyDenominator;
    const double CosTrueAnomaly = FVector::DotProduct(Basis.P, PositionKm) / Radius;
    const double TrueAnomaly = FMath::Atan2(SinTrueAnomaly, CosTrueAnomaly);

    // Средняя аномалия на текущий момент.
    double MeanAnomaly = 0.0;
    if (Elements.bElliptic)
    {
        MeanAnomaly = EllipticMeanAnomaly(TrueToEccentricAnomaly(TrueAnomaly, Elements.Eccentricity), Elements.Eccentricity);
    }
    else
    {
        MeanAnomaly = HyperbolicMeanAnomaly(TrueToHyperbolicAnomaly(TrueAnomaly, Elements.Eccentricity), Elements.Eccentricity);
    }

    // Продвигаем среднюю аномалию на заданное время.
    MeanAnomaly += Elements.MeanMotion * DeltaTimeSeconds;

    // Решаем уравнение Кеплера и возвращаемся к истинной аномалии.
    double NewTrueAnomaly = 0.0;
    if (Elements.bElliptic)
    {
        double EccentricAnomaly = 0.0;
        if (!SolveEllipticAnomaly(MeanAnomaly, Elements.Eccentricity, EccentricAnomaly))
        {
            return false;
        }
        NewTrueAnomaly = EccentricToTrueAnomaly(EccentricAnomaly, Elements.Eccentricity);
    }
    else
    {
        double HyperbolicAnomaly = 0.0;
        if (!SolveHyperbolicAnomaly(MeanAnomaly, Elements.Eccentricity, HyperbolicAnomaly))
        {
            return false;
        }
        NewTrueAnomaly = HyperbolicToTrueAnomaly(HyperbolicAnomaly, Elements.Eccentricity);
    }

    // Строим периферические координаты и переносим их в инерциальную систему.
    FTwoBodyState StateAtAnomaly;
    if (!ElementsToState(Elements, NewTrueAnomaly, StateAtAnomaly))
    {
        return false;
    }

    OutState.PositionKm = Basis.P * StateAtAnomaly.PositionKm.X + Basis.Q * StateAtAnomaly.PositionKm.Y;
    OutState.VelocityKmS = Basis.P * StateAtAnomaly.VelocityKmS.X + Basis.Q * StateAtAnomaly.VelocityKmS.Y;
    OutState.bValid = OutState.PositionKm.Size() > 0.0;

    return OutState.bValid && FMath::IsFinite(OutState.PositionKm.X) && FMath::IsFinite(OutState.VelocityKmS.X);
}

double UTwoBodyPropagator::GetOrbitRadiusKm(const FOrbitalElements& Elements, double TrueAnomalyRad)
{
    using namespace TwoBodyInternal;

    if (!Elements.bElliptic)
    {
        const double HyperbolicAnomaly = TrueToHyperbolicAnomaly(TrueAnomalyRad, Elements.Eccentricity);
        return FMath::Abs(Elements.SemiMajorAxisKm) * (Elements.Eccentricity * FMath::Cosh(HyperbolicAnomaly) - 1.0);
    }

    return Elements.SemiMajorAxisKm * (1.0 - Elements.Eccentricity * FMath::Cos(TrueToEccentricAnomaly(TrueAnomalyRad,
                                           Elements.Eccentricity)));
}

bool UTwoBodyPropagator::PropagateOrbitNumerically(const FVector& PositionKm, const FVector& VelocityKmS, double CentralMassKg,
    double DeltaTimeSeconds, FTwoBodyState& OutState)
{
    using namespace TwoBodyInternal;

    OutState = FTwoBodyState();

    if (CentralMassKg <= 0.0)
    {
        return false;
    }

    FVector Position = PositionKm;
    FVector Velocity = VelocityKmS;

    if (Position.SizeSquared() <= 0.0)
    {
        return false;
    }

    // Нулевая длительность: состояние не меняется.
    if (FMath::IsNearlyZero(DeltaTimeSeconds, 1.0e-9))
    {
        OutState.PositionKm = Position;
        OutState.VelocityKmS = Velocity;
        OutState.bValid = true;
        return true;
    }

    const double Mu = SimConstants::GravitationalConstant * CentralMassKg;

    // Подшаг ограничиваем долей радиуса, чтобы на сильно вытянутых участках
    // шаг не рос и интегрирование оставалось точным.
    const double Radius = Position.Size();
    const double Speed = Velocity.Size();
    const double MaxDisplacement = SimConstants::NumericalStepRadiusFraction * Radius;
    const double NominalStep = MaxDisplacement / FMath::Max(Speed, SimConstants::MinSpeedForStepEstimate);

    int32 Steps = FMath::CeilToInt(FMath::Abs(DeltaTimeSeconds) / FMath::Max(NominalStep, UE_DOUBLE_SMALL_NUMBER));
    Steps = FMath::Clamp(Steps, 1, SimConstants::MaxNumericalSubSteps);

    const double Step = DeltaTimeSeconds / Steps;

    for (int32 Iteration = 0; Iteration < Steps; ++Iteration)
    {
        const FVector K1Velocity = AccelerationAt(Position, Mu);
        const FVector K1Position = Velocity;

        const FVector K2Velocity = AccelerationAt(Position + K1Position * (Step * 0.5), Mu);
        const FVector K2Position = Velocity + K1Velocity * (Step * 0.5);

        const FVector K3Velocity = AccelerationAt(Position + K2Position * (Step * 0.5), Mu);
        const FVector K3Position = Velocity + K2Velocity * (Step * 0.5);

        const FVector K4Velocity = AccelerationAt(Position + K3Position * Step, Mu);
        const FVector K4Position = Velocity + K3Velocity * Step;

        Position += (K1Position + (K2Position + K3Position) * 2.0 + K4Position) * (Step / 6.0);
        Velocity += (K1Velocity + (K2Velocity + K3Velocity) * 2.0 + K4Velocity) * (Step / 6.0);
    }

    OutState.PositionKm = Position;
    OutState.VelocityKmS = Velocity;
    OutState.bValid = FMath::IsFinite(Position.X) && FMath::IsFinite(Velocity.X) && Position.SizeSquared() > 0.0;

    return OutState.bValid;
}