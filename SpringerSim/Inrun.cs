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

namespace SpringerSim
{
    [Serializable()]
    public class Inrun
    {
        private bool _isSolved = false;

        private double _v0 = 19.0;
        private double _x0 = 0;
        private double _alpha = 9.0 * Math.PI / 180.0;
        private double _gamma = 35.0 * Math.PI / 180.0;
        private double _r1;
        private double _r1_OR;
        private double _t;
        private double _t_OR;
        private double _l;
        private double _vA;
        private double _vB;
        private double _Ax;
        private double _Az;
        private double _Bx;
        private double _Bz;
        private double _E1x;
        private double _E1z;
        private double _E2x;
        private double _E2z;
        private List<double> _xs = new List<double>();
        private List<double> _zs = new List<double>();

        private CSpline _curve = new CSpline();
        private Slide _slide = new Slide();

        private double _ke = 0.0011;
        private double _kt = 0.0014;

        public Inrun()
        {
        }

        // The basic design speed of the inrun
        public double TakeoffSpeed
        {
            get => _v0;
            set
            {
                _v0 = value;
                _isSolved = false;
            }
        }

        [XmlIgnore]
        // The speed needed at the takeoff to reach K
        // with a 3 m/s tailwind.
        public double TopStartSpeed
        {
            get => _vA;
            set
            {
                _vA = value;
//                _isSolved = false;
            }
        }

        [XmlIgnore]
        // The speed needed at the takeoff to limit distance to L
        // with a 4 m/s headwind. Solve using rho = 1.
        public double BottomStartSpeed
        {
            get => _vB;
            set
            {
                _vB = value;
//                _isSolved = false;
            }
        }

        // Angle of the takeoff table
        public double TakeoffAngle
        {
            get => _alpha * 180 / Math.PI;
            set
            {
                _alpha = value * Math.PI / 180.0;
                _isSolved = false;
            }
        }

        // Angle of the inrun ramp
        public double RampAngle
        {
            get => _gamma * 180 / Math.PI;
            set
            {
                _gamma = value * Math.PI / 180.0;
                _isSolved = false;
            }
        }

        // Friction angle of the track
        // 1° for ice and refrigerated tracks
        // 3° for natural snow
        public double Rho
        {
            get => _slide.Rho;
            set
            {
                _slide.Rho = value;
                _isSolved = false;
            }
        }

        [XmlIgnore]
        // Distance from the top start to the beginning of the takeoff table
        public double InrunLength
        {
            get
            {
                _solve();
                _slide.InitialSpeed = -_vA;
                _slide.InitialPosition = 0;

                _Ax = _slide.FinalPosition;

                // Check to see if iverse operation gives proper speed
                //_slide.InitialPosition = _Ax;
                //_slide.InitialSpeed = 0;

                //Console.WriteLine("Speed = " + _slide.FinalSpeed);
                _Az = _curve.ValueAt(_Ax);

                //_xs[0] = _Ax;
                //_zs[0] = _Az;

                //_curve.SetCoords(_xs, _zs);

                //Restore the nominal settings
                _slide.InitialPosition = _x0;
                _slide.InitialSpeed = 0.0;

                return _curve.PathLength(_Ax, _E2x);
            }
            private set => InrunLength = value;
        }

        [XmlIgnore]
        // Distance from bottom start to top start
        public double StartLength
        {
            get
            {
                _solve();
                _slide.InitialSpeed = -_vB;
                _slide.InitialPosition = 0;

                double oldrho = _slide.Rho;
                _slide.Rho = 1.0;
                _Bx = _slide.FinalPosition;
                _Bz = _curve.ValueAt(_Bx);

                //Restore the nominal settings
                _slide.Rho = oldrho;
                _slide.InitialPosition = _x0;
                _slide.InitialSpeed = 0.0;
                _isSolved = false;

                return InrunLength - _curve.PathLength(_Bx, _E2x);
            }
            private set => StartLength = value;
        }

        // Length of the takeoff table
        public double TableLength
        {
            get
            {
                _solve();
                return _t;
            }
            set => _t_OR = value;
        }

        public bool ShouldSerializeTableLength()
        {
            return _t_OR > 0;
        }

        // Minimum radius of the inrun transition curve
        public double TransitionRadius
        {
            get
            {
                _solve();
                return _r1;
            }
            set
            {
                _r1_OR = value;
                _isSolved = false;
            }
        }

        public bool ShouldSerializeTransitionRadius()
        {
            return _r1_OR > 0;
        }

        // Aerodynamic drag coefficient from starting position to beginning of takeoff table
        public double RampAeroDrag
        {
            get => _ke;
            set
            {
                _ke = value;
                _isSolved = false;
            }
        }

        // Aerodynamic drag coefficient on the takeoff table
        public double TakeoffAeroDrag
        {
            get => _kt;
            set
            {
                _kt = value;
                _isSolved = false;
            }
        }

        [XmlIgnore]
        public List<double> Xs
        {
            get => new List<double>(_xs);
        }

        [XmlIgnore]
        public List<double> Zs
        {
            get => new List<double>(_zs);
        }

        [XmlIgnore]
        public List<double> SlideXs
        {
            get
            {
                _solve();
                return new List<double>(_slide.Xs);
            }
        }

        [XmlIgnore]
        public List<double> SlideVs
        {
            get
            {
                _solve();
                return new List<double>(_slide.Vs);
            }
        }

        [XmlIgnore]
        public List<double> SlideRAs
        {
            get
            {
                _solve();
                return new List<double>(_slide.RAs);
            }
        }

        [XmlIgnore]
        public List<double> SlideTAs
        {
            get
            {
                _solve();
                return new List<double>(_slide.TangentAccels);
            }
        }

        [XmlIgnore]
        public List<double> SlideKappas
        {
            get
            {
                _solve();
                return new List<double>(_slide.Kappas);
            }
        }

        [XmlIgnore]
        public List<double> SlidePhis
        {
            get
            {
                _solve();
                return new List<double>(_slide.Phis);
            }
        }

        [XmlIgnore]
        public double E1x
        {
            get
            {
                _solve();
                return _E1x;
            }
        }

        [XmlIgnore]
        public double E1z
        {
            get
            {
                _solve();
                return _E1z;
            }
        }

        [XmlIgnore]
        public double E2x
        {
            get
            {
                _solve();
                return _E2x;
            }
        }

        [XmlIgnore]
        public double E2z
        {
            get
            {
                _solve();
                return _E2z;
            }
        }

        [XmlIgnore]
        public double Ax
        {
            get
            {
                _solve();
                return _Ax;
            }
        }

        [XmlIgnore]
        public double Az
        {
            get
            {
                _solve();
                return _Az;
            }
        }

        [XmlIgnore]
        public double Bx
        {
            get
            {
                _solve();
                return _Bx;

            }
        }

        [XmlIgnore]
        public double Bz
        {
            get
            {
                _solve();
                return _Bz;
            }
        
        }

        // 
        private void _solve()
        {
            if (_isSolved)
                return;

            if (_t_OR == 0)
            {
                _t = 0.25 * (_v0 + 0.95);
            } else
            {
                _t = _t_OR;
            }
            if (_r1_OR == 0)
            {
                _r1 = 0.14 * Math.Pow(_v0 + 0.95, 2.0);
            } else
            {
                _r1 = _r1_OR;
            }

            double d = 2.0 * _r1 * Math.Sin(_gamma - _alpha) * Math.Pow(Math.Cos(_gamma - _alpha), 2.0);
            double C = Math.Tan(_gamma - _alpha) / (3 * Math.Pow(d, 2.0));
            double f = Math.Tan(_gamma - _alpha) * d / 3.0;

            _l = d * (1.0 + 0.1 * Math.Pow(Math.Tan(_gamma - _alpha), 2.0));

            double P = 1 / (Math.Tan(_gamma) * 3 * C);

            _E1x = -(_t * Math.Cos(_alpha) + f * Math.Sin(_gamma) + d * Math.Cos(_gamma));
            _E1z = _t * Math.Sin(_alpha) - f * Math.Cos(_gamma) + d * Math.Sin(_gamma);
            _E2x = -_t * Math.Cos(_alpha);
            _E2z = _t * Math.Sin(_alpha);
            double x = _E1x;
            double denom = 2.0 * C * Math.Sin(_gamma);

            _xs.Clear();
            _zs.Clear();

            // First point is a placeholder
            // Place 100m up-slope of E1
            _xs.Add(_E1x - 100 * Math.Cos(_gamma));
            _zs.Add(_E1z + 100 * Math.Sin(_gamma));

            while (x < _E2x)
            {
                double Q = (x - _E1x) / denom;
                double QP = Math.Sqrt(Math.Pow(Q, 2.0)+ Math.Pow(P, 3.0));
                double xi = Math.Pow(QP + Q, 1.0 / 3.0) - Math.Pow(QP - Q, 1.0 / 3.0);
                double z = _E1z - xi * Math.Sin(_gamma) + C * Math.Pow(xi, 3.0) * Math.Cos(_gamma);

                // Add the intermediate transition point
                _xs.Add(x);
                _zs.Add(z);

                x += 0.5;
            }

            // Add E2
            _xs.Add(_E2x);
            _zs.Add(_E2z);

            // Add the end of takeoff table
            _xs.Add(0.0);
            _zs.Add(0.0);

            _curve.SetCoords(_xs, _zs);
            _curve.Alpha = -Math.Tan(RampAngle * Math.PI / 180.0);
            _curve.Beta = -Math.Tan(TakeoffAngle * Math.PI / 180.0);
            _slide.Curve = _curve;

            Polyline ks = new Polyline();
            ks.SetCoords(new List<double>() {-200.0, _E2x-0.1, _E2x+0.1, 0.1 },
                new List<double>() { _ke, _ke, _kt, _kt });
            _slide.AeroDrag = ks;

            _slide.InitialPosition = 0.0;
            _slide.InitialSpeed = -_v0;
            _x0 = _slide.FinalPosition;
            _isSolved = true;
        }
    }
}
