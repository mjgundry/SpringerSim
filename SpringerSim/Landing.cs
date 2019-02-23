//
// Copywrite (c) 2016 Matthew J Gundry
//
// Written by Matthew J Gundry <mjgundry@faa-engineers.com>
//
// This file is part of SpringerSim.
//
// SpringerSim is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
// 
// SpringerSim is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
// 
// You should have received a copy of the GNU General Public License
// along with SpringerSim.  If not, see <http://www.gnu.org/licenses/>.

using System;
using System.Xml.Serialization;
using System.Collections.Generic;
using System.Linq;

namespace SpringerSim
{

    enum TransType { CubicParabola, Circle, Cycloid };

    enum Priority { HN, HS };

    [Serializable()]
    public class Landing
    {
        private const double g = 9.80665;
        private const double xinc = 1.0;

        private double _staU = 0;
        private double _eleU = 0;
        private double _s = 0;
        private double _s_OR = 0;
        private double _rL = 0;
        private double _rL_OR = 0;
        private double _r2L = 0;
        private double _r2L_OR = 0;
        private double _r2 = 0;
        private double _r2_OR = 0;
        private double _r2Lmin = 0;
        private double _vL = 0;
        private double _hP = 0;
        private double _nP = 0;
        private double _hL = 0;
        private double _nL = 0;
        private double _hU = 0;
        private double _nU = 0;
        private double _drop = 0.7;
        private double _drop_OR = 0;
        private double _beta = 0;
        private double _beta_OR = 0;
        private double _dBeta = 2;
        private double _beta0 = 0;
        private double _betaP = 0;
        private double _betaL = 0;
        private double _betaL_OR = 0;
        private double _rho = 1.0;
        private double _l1;
        private double _l2;
        private double _perf = 1.06;
        private TransType _transType = TransType.CubicParabola;

        private Trajectory _desFlight;
        private Trajectory _flightA;
        private Trajectory _flightB;

        private Slide _slide;
        private CSpline _hill;
        private bool _isSolved = false;
        private bool _calculating = false;

        public Landing()
        {
            _desFlight = new Trajectory();  // Create new flight trajectory solver
            _flightA = new Trajectory();  // Create new flight trajectory solver
            _flightB = new Trajectory();  // Create new flight trajectory solver
            _slide = new Slide();           // Create new sliding physics solver
            _hill = new CSpline();
            Polyline ks = new Polyline();
            ks.SetCoords(new List<double>() { -1000, 1000 }, new List<double>() { 0.004, 0.004 });
            _slide.AeroDrag = ks;        // Drag (air) coefficient
            _slide.Rho = 1;                 // Sliding friction angle
        }

        // Landing class is set up for update on read
        // Property setters also set a dirty bit
        // Property getters 
        public double StationU
        {
            get => _staU;
            set => _staU = value;
        }

        public double ElevationU
        {
            get => _eleU;
            set => _eleU = value;
        }

        public double TakeoffSpeed
        {
            get => _desFlight.TakeoffSpeed;
            set
            {
                _desFlight.TakeoffSpeed = value;
                _isSolved = false;
            }
        }

        public double MinimumSpeed
        {
            get
            {
                _solve();
                _flightB.TakeoffSpeed = _desFlight.TakeoffSpeed;
                _flightB.TakeoffAngle = _desFlight.TakeoffAngle;
                _flightB.JumpDistance = HillSize;
                _flightB.Gradient = -Lz / Lx;
                _flightB.WindSpeed = 4.0;
                return _flightB.RequiredSpeed / _perf;
            }
        }

        public double MaximumSpeed
        {
            get
            {
                _solve();
                _flightA.TakeoffSpeed = _desFlight.TakeoffSpeed;
                _flightA.TakeoffAngle = _desFlight.TakeoffAngle;
                _flightA.JumpDistance = _desFlight.JumpDistance;
                _flightA.Gradient = _desFlight.Gradient;
                _flightA.WindSpeed = -3.0;
                return _flightA.RequiredSpeed;
            }
        }

        // Push velocity on the takeoff
        public double PushSpeed
        {
            get => _desFlight.PushSpeed;
            set
            {
                _desFlight.PushSpeed = value;
                _isSolved = false;
            }
        }

        // Performance Increase, used to adjust the minimum required speed
        // for calculating the lowest start location
        public double PerformanceIncrease
        {
            get => _perf;
            set
            {
                _perf = value;
            }
        }

        public double TakeoffAngle
        {
            get => _desFlight.TakeoffAngle;
            set
            {
                _desFlight.TakeoffAngle = value;
                _isSolved = false;
            }
        }

        public double HillSize
        {
            get
            {
                _solve();
                return Math.Round(_desFlight.JumpDistance + _l2, 0);
            }
        }

        // K point
        public double W
        {
            get => Math.Round(_desFlight.JumpDistance);
            set
            {
                _desFlight.JumpDistance = value;
                _isSolved = false;
            }
        }

        // Hill gradient or H/N
        public double Gradient
        {
            get => _desFlight.Gradient;
            set
            {
                _desFlight.Gradient = value;
                _isSolved = false;
            }
        }

        // Change in angle from P to K
        public double DeltaBeta
        {
            get => _dBeta;
            set
            {
                _dBeta = value;
                _isSolved = false;
            }
        }

        [XmlIgnore]
        public double Kz => _desFlight.Kz;

        [XmlIgnore]
        public double Kx => _desFlight.Kx;

        [XmlIgnore]
        public double LandingSpeed
        {
            get
            {
                _solve();
                return _desFlight.LandingSpeed;
            }
        }

        [XmlIgnore]
        public double Pz
        {
            get
            {
                _solve();
                return -_hP;
            }
        }

        [XmlIgnore]
        public double Px
        {
            get
            {
                _solve();
                return _nP;
            }
        }

        [XmlIgnore]
        public double Lz
        {
            get
            {
                _solve();
                return -_hL;
            }
        }

        [XmlIgnore]
        public double Lx
        {
            get
            {
                _solve();
                return _nL;
            }
        }

        [XmlIgnore]
        public double Ux
        {
            get
            {
                _solve();
                return _nU;
            }
        }

        [XmlIgnore]
        public double Uz
        {
            get
            {
                _solve();
                return -_hU;
            }
        }

        [XmlIgnore]
        public double UpperLandingLength
        {
            get
            {
                _solve();
                return _l1;
            }
        }

        [XmlIgnore]
        public double LowerLandingLength
        {
            get
            {
                _solve();
                return _l2;
            }
        }

        // 
        public double EquivalentLandingHeight
        {
            get
            {
                _solve();
                return _drop;
            }
            set
            {
                _drop_OR = value;
                _beta_OR = 0;
                _isSolved = false;
            }
        }

        public bool ShouldSerializeEquivalentLandingHeight()
        {
            return _drop_OR != 0;
        }

        public double AngleAtK
        {
            get
            {
                _solve();
                return _beta;
            }
            set
            {
                _beta_OR = value;
                _drop_OR = 0;
                _isSolved = false;
            }
        }

        public bool ShouldSerializeAngleAtK()
        {
            return _beta_OR != 0;
        }

        public double AngleAtL
        {
            get
            {
                _solve();
                return _betaL;
            }
        }

        [XmlIgnore]
        public double AngleAtP
        {
            get
            {
                _solve();
                return _betaP;
            }
        }

        [XmlIgnore]
        public double AngleAtTakeoff
        {
            get
            {
                _solve();
                return _beta0;
            }
        }

        public double LandingRadius
        {
            get
            {
                _solve();
                return _rL;
            }
            set
            {
                _rL_OR = 0;
            }
        }

        [XmlIgnore]
        public double InitialTransitionSpeed
        {
            get
            {
                _solve();
                return _vL;
            }
        }

        public double TakeoffHeight
        {
            get
            {
                _solve();
                return _s;
            }
            set
            {
                _s_OR = value;
            }
        }

        public bool ShouldSerializeTakeoffHeight()
        {
            return _s_OR != 0;
        }

        [XmlIgnore]
        public List<double> Xs
        {
            get
            {
                _solve();
                return _hill.Xs;
            }
        }

        [XmlIgnore]
        public List<double> Zs
        {
            get
            {
                _solve();
                return _hill.Zs;
            }
        }

        [XmlIgnore]
        public List<double> KFlightXs
        {
            get => _desFlight.Xs;
        }

        [XmlIgnore]
        public List<double> KFlightZs
        {
            get => _desFlight.Zs;
        }

        [XmlIgnore]
        public List<double> KFlightVs
        {
            get => _desFlight.Vs;
        }

        [XmlIgnore]
        public List<double> SlideXs
        {
            get => _slide.Xs;
        }

        [XmlIgnore]
        public List<double> SlideVs
        {
            get => _slide.Vs;
        }

        [XmlIgnore]
        public List<double> SlideRAs
        {
            get => _slide.RAs;
        }

        [XmlIgnore]
        public List<double> SlideKappas
        {
            get => _slide.Kappas;
        }

        [XmlIgnore]
        public List<double> SlidePhis
        {
            get => _slide.Phis;
        }

        public double TransitionRadiusAtL
        {
            get
            {
                _solve();
                return _r2L;
            }
            set
            {
                _r2L_OR = value;
                _isSolved = false;
            }
        }

        public bool ShouldSerializeTransitionRadiusAtL()
        {
            return (_r2L_OR > 0);
        }

        public double TransitionRadiusAtU
        {
            get
            {
                _solve();
                return _r2;
            }
            set
            {
                _r2_OR = value;
                _isSolved = false;
            }
        }

        public bool ShouldSerializeTransitionRadiusAtU()
        {
            return (_r2_OR > 0);
        }

        public double HeightAt(double x)
        {
            return _hill.ValueAt(x);
        }

//        public bool Calculating
//        {
//            get
//            {
//                return _calculating || _flightA.Calculating || _flightB.Calculating;
//            }
//
//            private set
//            {
//                _calculating = value;
//            }
//        }

        private void _solve()
        {
            if (_isSolved)
                return;

//            Calculating = true;

            // Only reference private variables herein

            if (_drop_OR == 0 && _beta_OR == 0)
            {
                // Find the speed perpendicular to terrain based on effective drop height
                double vPl = Math.Round(Math.Sqrt((2.0 * _drop) * g), 3);
                // Hill slope at K is calculated from flight approach angle and acceptable
                // deflection calculated from equivalent landing height.
                _beta = Math.Round(_desFlight.LandingAngle - (180.0 / Math.PI) * Math.Asin(vPl / _desFlight.LandingSpeed), 2);
            } else if (_drop_OR != 0)
            {
                _drop = _drop_OR;
                double vPl = Math.Round(Math.Sqrt((2.0 * _drop) * g), 3);
                _beta = Math.Round(_desFlight.LandingAngle - (180.0 / Math.PI) * Math.Asin(vPl / _desFlight.LandingSpeed), 2);
            } else if (_beta_OR != 0)
            {
                _beta = _beta_OR;
                double vPl = _desFlight.LandingSpeed * Math.Sin((_desFlight.LandingAngle - _beta) * Math.PI / 180.0);
                _drop = 0.5 * Math.Pow(vPl, 2.0) / g;
            }

            if (_betaL_OR == 0)
                _betaL = Math.Round(_beta - 1.4 / _desFlight.LandingSpeed * (180.0 / Math.PI), 2);
            else
                _betaL = _beta_OR;
            _betaP = Math.Round(_beta + _dBeta, 2);
            _beta0 = Math.Round(_betaP / 6, 1);

            if (_s_OR == 0)
            {
                _s = Math.Max(_desFlight.JumpDistance / 40.0, 0.5);
            } else
            {
                _s = _s_OR;
            }

            if (_rL_OR == 0.00)
                _rL = Math.Ceiling(Math.Max(Math.Pow(_desFlight.LandingSpeed, 2.0) * _desFlight.JumpDistance / 380.0,
                      Math.Pow(_desFlight.LandingSpeed, 2.0) / 8.0) / 10) * 10;
            else
                _rL = _rL_OR;

            _nP = _desFlight.Kx - _rL * (Math.Sin(_betaP / 180.0 * Math.PI) - Math.Sin(_beta / 180.0 * Math.PI));
            _hP = -_desFlight.Kz + _rL * (Math.Cos(_betaP / 180.0 * Math.PI) - Math.Cos(_beta / 180.0 * Math.PI));
            _nL = _desFlight.Kx + _rL * (Math.Sin(_beta / 180.0 * Math.PI) - Math.Sin(_betaL / 180.0 * Math.PI));
            _hL = -_desFlight.Kz - _rL * (Math.Cos(_beta / 180.0 * Math.PI) - Math.Cos(_betaL / 180.0 * Math.PI));

            _l1 = _dBeta * _rL * Math.PI / 180.0;
            _l2 = 1.4 * _rL / _desFlight.LandingSpeed;

            _vL = _desFlight.LandingSpeed - 16.0 / _desFlight.LandingSpeed - 0.1 * _rho;
            _r2Lmin = Math.Pow(_vL, 2.0) / (1.8 * g - g * Math.Cos(_betaL * 180 / Math.PI));

            // If the user hasn't overridden the transition radii, try automatically
            // finding radii values that meet FIS criteria
            if (_r2L_OR == 0)
                _r2L = _rL;
            else
                _r2L = _r2L_OR;

            if (_r2_OR == 0)
                _r2 = Math.Ceiling(_r2Lmin / 10) * 10.0;
            else
                _r2 = _r2_OR;

            _generateCoords();

            if (_r2_OR == 0)
            {
                // Due to increasing speed in transition, flattening terrain, and decreasing radius
                // max radial acceleration is typically in the bottom of the transition. Increase r2
                // until the radial acceleration is less than the 18m/s^2 max. If r2 gets as large
                // as r2L (which is set to rL), then we can't improve any further.
                while (_slide.RAs.Max() > 18.05)
                {
                    _r2 += 10;
                    if (_r2 > _r2L)
                    {
                        _r2 = _r2L;
                        _generateCoords();
                        break;
                    }
                    _generateCoords();
                }
            }

            if (_r2L_OR == 0)
            {
                // If the landing is over 88 meters high, reduce r2L. If r2L gets as small
                // as r2, then we can't improve any further.
                // Ignore this step if the transition is a circle, as the radius is set by r2
                while ((_hill.Zs.Last() < -88.0) && !(_transType == TransType.Circle))
                {
                    _r2L -= 10;
                    if (_r2L < _r2)
                    {
                        _r2L = _r2;
                        _generateCoords();
                        break;
                    }
                    _generateCoords();
                }
            }

            _isSolved = true;
//            Calculating = false;
        }

        private void _generateCoords()
        {
            List<double> xs = new List<double>();
            List<double> zs = new List<double>();
            double u, v, x, z;

            // First generate the cubic parabola for the knoll
            u = _hP - _s - (_nP * Math.Tan(_beta0 / 180.0 * Math.PI));
            v = _nP * (Math.Tan(_betaP / 180.0 * Math.PI) - Math.Tan(_beta0 / 180.0 * Math.PI));
            x = 0;

            while (x < _nP)
            {
                z = (-_s - x * Math.Tan(_beta0 / 180.0 * Math.PI) - (3 * u - v) *
                      Math.Pow((x / _nP), 2.0) + (2 * u - v) * Math.Pow((x / _nP), 3.0));
                xs.Add(x);
                zs.Add(z);
                x += xinc;
            }

            // Force a point at P
            xs.Add(_nP);
            zs.Add(-_hP);

            // Now generate the circular landing area up to K
            while (x < _desFlight.Kx)
            {
                z = -_hP + _rL * (Math.Cos(_betaP / 180.0 * Math.PI) -
                    Math.Cos(Math.Asin(Math.Sin(_betaP / 180.0 * Math.PI) - (x - _nP) / _rL)));
                xs.Add(x);
                zs.Add(z);
                x += xinc;
            }

            // Force a point at K
            xs.Add(_desFlight.Kx);
            zs.Add(_desFlight.Kz);

            // Generate the rest of the landing area up to L
            while (x < _nL)
            {
                z = -_hP + _rL * (Math.Cos(_betaP / 180.0 * Math.PI) -
                    Math.Cos(Math.Asin(Math.Sin(_betaP / 180.0 * Math.PI) - (x - _nP) / _rL)));
                xs.Add(x);
                zs.Add(z);
                x += xinc;
            }

            // Force a point at L
            xs.Add(_nL);
            zs.Add(-_hL);

            switch (_transType)
            {
                case TransType.CubicParabola:
                    {
                        // FIS has simplified the equation for tau by assuming 1/cos(tau) => 1
                        // This can cause small (negligible) errors for our generic slide solver because
                        // the resulting (small) coordinate errors result in splines with increased curvature
                        // A more accurate solution is easily found iteratively
                        //double tau_p = 0.0;
                        double tau = -0.04;
                        //while (Math.Abs(tau - tau_p) > 0.000001)
                        //{
                        //    tau_p = tau;
                        //    tau = Math.Atan((Math.Cos(_betaL / 180.0 * Math.PI) -
                        //          (1 / Math.Cos(tau_p)) * Math.Pow((_r2 / _r2L), (1.0 / 3.0))) /
                        //          Math.Sin(_betaL / 180.0 * Math.PI));
                        //}

                        tau = Math.Atan((Math.Cos(_betaL / 180.0 * Math.PI) - Math.Pow((_r2 / _r2L), (1.0 / 3.0))) / Math.Sin(_betaL / 180.0 * Math.PI));

                        double C = 1.0 / (2.0 * _r2 * Math.Pow(Math.Cos(tau), 3.0));
                        double a = -Math.Tan(_betaL / 180.0 * Math.PI + tau) / 2.0 / C;
                        double b = -Math.Tan(tau) / 2.0 / C;

                        _nU = _nL + C * Math.Sin(tau) * (Math.Pow(a, 2.0) - Math.Pow(b, 2.0)) +
                             Math.Cos(tau) * (b - a);
                        _hU = _hL + C * Math.Cos(tau) * (Math.Pow(a, 2.0) - Math.Pow(b, 2.0)) -
                             Math.Sin(tau) * (b - a);
                        while (x < _nU)
                        {
                            double xi = (Math.Cos(tau) - Math.Sqrt(Math.Pow(Math.Cos(tau), 2.0) -
                                        4.0 * C * (x - _nL - C * Math.Pow(a, 2.0) * Math.Sin(tau) +
                                        a * Math.Cos(tau)) * Math.Sin(tau))) / 2 / C / Math.Sin(tau);
                            z = -_hL - C * Math.Cos(tau) * (Math.Pow(a, 2.0) - Math.Pow(xi, 2.0)) -
                                Math.Sin(tau) * (a - xi);
                            xs.Add(x);
                            zs.Add(z);
                            x += xinc;
                        }
                        xs.Add(_nU);
                        zs.Add(-_hU);
                        break;
                    }

                case TransType.Circle:
                    {
                        break;
                    }

                case TransType.Cycloid:
                    {
                        break;
                    }
            }

            // Create a clamped spline with the hill coordinates
            _hill.SetCoords(xs, zs);
            _hill.Alpha = -Math.Tan(_beta0 / 180.0 * Math.PI);
            _hill.Beta = 0.0;
            _slide.Curve = _hill;

            // Get the position and speed at landing from the design trajectory
            _slide.InitialPosition = _nL;
            _slide.InitialSpeed = _vL;
        }
    }

}