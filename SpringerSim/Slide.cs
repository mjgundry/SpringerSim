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
using System.Collections.Generic;
using System.Diagnostics;

namespace SpringerSim
{
    // Solver for sliding physics which includes air resistance and friction
    // If given an initial speed >= 0 and a position in the curve it will
    // give the speed at the end of the curve.
    // If given a negative speed and position, it will find the starting position
    // needed to achieve that speed at that position
    // The slide curve must slope down to the right
    class Slide
    {
        private const double g = 9.80665;

        private List<double> _v = new List<double>();
        private List<double> _x = new List<double>();
        private List<double> _z = new List<double>();
        private List<double> _kappa;
        private List<double> _phi;
        private List<double> _ar;
        private List<double> _at;

        private double _dt = 0.05;
        private double _rho = 1.0 / 180.0 * Math.PI;
        private Polyline _ks;
        private CSpline _curve;

        private bool _isSolved = false;

        private delegate void Solve();
        Solve _solve;

        public Slide()
        {
            _curve = new CSpline();
            _ks = new Polyline();
            _ks.SetCoords(new List<double>() { -1000, 1000 }, new List<double>() { 0.004, 0.004 });
        }

        // Set the starting position X coordinate
        public double InitialPosition
        {
            get => _x[0];

            set
            {
                Debug.Assert((value <= _curve.Xs[_curve.Xs.Count - 1]) && (value >= _curve.Xs[0]));
                _x = new List<double>() { value };
                _z = new List<double>() { _curve.ValueAt(value) };
                _isSolved = false;
            }
        }

        public double FinalPosition
        {
            get
            {
                _solvePosi();
                _isSolved = false;

                return _x[_x.Count - 1];
            }
        }

        // Set the initial velocity. If negative, the velocity is up the curve
        // and solver will find starting point necessary to generate that velocity
        public double InitialSpeed
        {
            get => _v[0];

            set
            {
                _v = new List<double>() { value };
//                if (value < 0)
//                    _solve = _solvePosi;
//                else
                    _solve = _solveV;
                _isSolved = false;
            }
        }

        public double FinalSpeed
        {
            get
            {
                _solve();
                return _v[_v.Count - 1];
            }
        }

        // Set the friction angle in degrees of the sliding surface
        public double Rho
        {
            get => _rho * 180.0 / Math.PI;

            set
            {
                Debug.Assert((value <= 4.0) && (value >= 0));
                _rho = value * Math.PI / 180.0;
                _isSolved = false;
            }
        }

        // Aerodynamic drag coefficient
        public Polyline AeroDrag
        {
            get => _ks;

            set
            {
                _ks = value;
                _isSolved = false;
            }
        }

        // The curve to be evaluated
        public CSpline Curve
        {
            get => _curve;

            set
            {
                _curve = value;
                _isSolved = false;
            }
        }

        // Get the X positions per timestep
        public List<double> Xs
        {
            get
            {
                _solve();
                return new List<double>(_x);
            }
        }

        // Get the Z positions per timestep
        public List<double> Zs
        {
            get
            {
                _solve();
                return new List<double>(_z);
            }
        }

        // Get the angle values per timestep
        public List<double> Phis
        {
            get
            {
                _solve();
                return new List<double>(_phi);
            }
        }

        // Get the curvature values per timestep
        public List<double> Kappas
        {
            get
            {
                _solve();
                return new List<double>(_kappa);
            }
        }

        // Get the speed values per timestep
        public List<double> Vs
        {
            get
            {
                _solve();
                return new List<double>(_v);
            }
        }

        // Get the radial acceleration values per timestep
        public List<double> RAs
        {
            get
            {
                _solve();
                return new List<double>(_ar);
            }
        }

        // Get the tangential acceleration values per timestep
        public List<double> TangentAccels
        {
            get
            {
                _solve();
                return new List<double>(_at);
            }
        }

        // ODE for rate of change in horizontal position based on current speed and slope (radians)
        private double _dxdt(double v, double phi)
        {
            return v * Math.Cos(phi);
        }

        // ODE for rate of change in horizontal position based on slope (radians)
        private double _dxds(double phi)
        {
            return Math.Cos(phi);
        }

        // ODE for rate of change in vertical position based on current speed and slope (radians)
        // We could just use getValueAt(x), but computing z this way gives a check on solver accuracy
        private double _dzdt(double v, double phi)
        {
            return -v * Math.Sin(phi);
        }

        // ODE for rate of change in vertical position based on slope (radians)
        // We could just use getValueAt(x), but computing z this way gives a check on solver accuracy
        private double _dzds(double phi)
        {
            return -Math.Sin(phi);
        }

        // ODE for rate of change in speed squared in position based on current speed, slope, and
        // curvature. We don't use it here, but include is as it is the equation provided by FIS.
        // Assumes Sin(rho) = rho for small angles when finding friction due to added centripal force
        private double _dv2ds(double vsq, double phi, double kappa)
        {
            return (2 * g * Math.Sin(phi - _rho)) - (2 * (_ks.ValueAt(_x[_x.Count - 1]) + _rho * kappa) * vsq);
        }

        // ODE for rate of change in speed in time based on current speed, slope, and curvature
        // This equation is derived from the FIS equation given above.
        private double _dvdt(double v, double phi, double kappa)
        {
            double ks = _ks.ValueAt(_x[_x.Count - 1]);
            //            return (g * Math.Sin(phi - _rho)) - ((_ks.ValueAt(_x[_x.Count - 1]) + _rho * kappa) * Math.Pow(v, 2.0));
            return (g * Math.Sin(phi - _rho)) - (ks + Math.Sin(_rho) * kappa) * Math.Pow(v, 2.0);
        }

        // Iterate through the time steps until the end of the curve is reached
        private void _solveV()
        {
            if (_isSolved)
                return;

            // delete the previous slide data except for the initial conditions at [0]
            _v = new List<double>() { _v[0] };
            _x = new List<double>() { _x[0] };
            _z = new List<double>() { _curve.ValueAt(_x[0]) };
            _kappa = new List<double>() { _curve.CurvatureAt(_x[0]) };
            _phi = new List<double>() { -Math.Atan(_curve.SlopeAt(_x[0])) };
            _ar = new List<double>() { g * Math.Cos(_phi[0]) + Math.Pow(_v[0], 2.0) * _kappa[0] };
            _at = new List<double>() { g * Math.Sin(_phi[0]) };

            //self._kappa[0] = self._curve.curvatureAt(self._x[0])
            //self._phi[0] = -math.atan(self._curve.slopeAt(self._x[0]))

            // Keep advancing time steps until we reach the end of the curve
            while (_x[_x.Count - 1] < _curve.Xs[_curve.Xs.Count - 1])
                _advanceTimeStep();

            // Pop the last point which is past the end of the curve and
            // interpolate a value for the end
            double x2 = _x[_x.Count - 1];
            double z2 = _z[_z.Count - 1];
            double v2 = _v[_v.Count - 1];
            double ar2 = _ar[_ar.Count - 1];
            double at2 = _at[_at.Count - 1];

            double x1 = _x[_x.Count - 2];
            double z1 = _z[_z.Count - 2];
            double v1 = _v[_v.Count - 2];
            double ar1 = _ar[_ar.Count - 2];
            double at1 = _at[_at.Count - 2];

            // Set the last x position to the end of the curve
            double x = _curve.Xs[_curve.Xs.Count - 1];

            _x[_x.Count - 1] = x;
            _z[_z.Count - 1] = (z1 + (z2 - z1) * (x - x1) / (x2 - x1));
            _v[_v.Count - 1] = (v1 + (v2 - v1) * (x - x1) / (x2 - x1));
            _ar[_ar.Count - 1] = (ar1 + (ar2 - ar1) * (x - x1) / (x2 - x1));
            _at[_at.Count - 1] = (at1 + (at2 - at1) * (x - x1) / (x2 - x1));

            // Check to be sure our calculated z matches the curve z within tolerance
            Debug.Assert(Math.Abs(_z[_z.Count - 1] - _curve.ValueAt(_x[_x.Count - 1])) < 0.01);

            _isSolved = true;
        }

        // Iterate through the time steps until velocity becomes zero
        private void _solvePosi()
        {
//            if (_isSolved)
//                return;

            // delete the previous slide data except for the initial conditions at [0]
            _v = new List<double>() { _v[0] };
            _x = new List<double>() { _x[0] };
            _z = new List<double>() { _z[0] };
            _kappa = new List<double>() { _curve.CurvatureAt(_x[0]) };
            _phi = new List<double>() { -Math.Atan(_curve.SlopeAt(_x[0])) };
            _ar = new List<double>();
            _at = new List<double>();

            // Keep advancing time steps until velocity increases above 0 or start of curve is reached
            while ((_v[_v.Count - 1] < 0) && (_x[_x.Count - 1] > _curve.Xs[0]))
                _advancePositionStep();

            // Pop the last point which is past the end of the curve and
            // interpolate a value for the end
            //double x2 = _x[_x.Count - 1];
            //double z2 = _z[_z.Count - 1];
            //double v2 = _v[_v.Count - 1];
            //double ar2 = _ar[_ar.Count - 1];
            //double at2 = _at[_at.Count - 1];

            //double x1 = _x[_x.Count - 2];
            //double z1 = _z[_z.Count - 2];
            //double v1 = _v[_v.Count - 2];
            //double ar1 = _ar[_ar.Count - 2];
            //double at1 = _at[_at.Count - 2];

            // Set the last x position to the end of the curve
            //double x = _curve.Xs[_curve.Xs.Count - 1];

            //_x[_x.Count - 1] = x;
            //_z[_z.Count - 1] = (z1 + (z2 - z1) * (x - x1) / (x2 - x1));
            //_v[_v.Count - 1] = (v1 + (v2 - v1) * (x - x1) / (x2 - x1));
            //_ar[_ar.Count - 1] = (ar1 + (ar2 - ar1) * (x - x1) / (x2 - x1));
            //_at[_at.Count - 1] = (at1 + (at2 - at1) * (x - x1) / (x2 - x1));

            // Check to be sure our calculated z matches the curve z within tolerance
            //    Debug.Assert(Math.Abs(_z[_z.Count - 1] - _curve.ValueAt(_x[_x.Count - 1])) < 0.01);

            _isSolved = true;
        }

        // Calculate slide data at the next time step by solving the system
        // of ODEs using the 4th order Runge Kutta method
        private void _advanceTimeStep()
        {
            // Get position, speed, and direction from last time step
            double x1 = _x[_x.Count - 1];
            double z1 = _z[_z.Count - 1];
            double v1 = _v[_v.Count - 1];
            double phi1 = -Math.Atan(_curve.SlopeAt(x1));
            double kappa1 = _curve.CurvatureAt(x1);

            // Find k1 values
            double dx1 = _dt * _dxdt(v1, phi1);
            double dz1 = _dt * _dzdt(v1, phi1);
            double dv1 = _dt * _dvdt(v1, phi1, kappa1);

            // Find speed, slope, and curvature k1
            double v2 = v1 + dv1 / 2;
            double phi2 = -Math.Atan(_curve.SlopeAt(x1 + dx1 / 2));
            double kappa2 = _curve.CurvatureAt(x1 + dx1 / 2);

            // Find k2 values
            double dx2 = _dt * _dxdt(v2, phi2);
            double dz2 = _dt * _dzdt(v2, phi2);
            double dv2 = _dt * _dvdt(v2, phi2, kappa2);

            // Find speed, slope, and curvature k2
            double v3 = v1 + dv2 / 2;
            double phi3 = -Math.Atan(_curve.SlopeAt(x1 + dx2 / 2));
            double kappa3 = _curve.CurvatureAt(x1 + dx2 / 2);

            // Find k3 values        
            double dx3 = _dt * _dxdt(v3, phi3);
            double dz3 = _dt * _dzdt(v3, phi3);
            double dv3 = _dt * _dvdt(v3, phi3, kappa3);

            // Find speed, slope and curvature using k3
            double v4 = v1 + dv3;
            double phi4 = -Math.Atan(_curve.SlopeAt(x1 + dx3));
            double kappa4 = _curve.CurvatureAt(x1 + dx3);

            // Find k4 values
            double dx4 = _dt * _dxdt(v4, phi4);
            double dz4 = _dt * _dzdt(v4, phi4);
            double dv4 = _dt * _dvdt(v4, phi4, kappa4);

            // Add the weighted averages for position and speed to their
            // respective initial values and append those values to the end of their lists
            _x.Add(x1 + (dx1 + 2 * dx2 + 2 * dx3 + dx4) / 6);
            _z.Add(z1 + (dz1 + 2 * dz2 + 2 * dz3 + dz4) / 6);
            double dv = (dv1 + 2 * dv2 + 2 * dv3 + dv4) / 6;
            double v = v1 + dv;
            _v.Add(v);
            double kappa = (kappa1 + 2 * kappa2 + 2 * kappa3 + kappa4) / 6;
            _kappa.Add(kappa);
            double phi = (phi1 + 2 * phi2 + 2 * phi3 + phi4) / 6;
            _phi.Add(phi);
            _ar.Add(g * Math.Cos(phi) + Math.Pow(v, 2.0) * kappa);
            _at.Add(dv / _dt);
        }

        // Using the FIS method, ODEs based on change in position along curve
        private void _advancePositionStep()
        {
            double _ds = 0.3 * Math.Sign(_v[_v.Count - 1]);

            // Get position, speed, and direction from last position step
            double x1 = _x[_x.Count - 1];
            double z1 = _z[_z.Count - 1];
            double vs1 = Math.Pow(_v[_v.Count - 1], 2.0);
            double phi1 = -Math.Atan(_curve.SlopeAt(x1));
            double kappa1 = _curve.CurvatureAt(x1);

            // Find k1 values
            double dx1 = _ds * _dxds(phi1);
            double dz1 = _ds * _dzds(phi1);
            double dvs1 = _ds * _dv2ds(vs1, phi1, kappa1);

            // Find speed, slope, and curvature k1
            double vs2 = vs1 + dvs1 / 2;
            double phi2 = -Math.Atan(_curve.SlopeAt(x1 + dx1 / 2));
            double kappa2 = _curve.CurvatureAt(x1 + dx1 / 2);

            // Find k2 values
            double dx2 = _ds * _dxds(phi2);
            double dz2 = _ds * _dzds(phi2);
            double dvs2 = _ds * _dv2ds(vs2, phi2, kappa2);

            // Find speed, slope, and curvature k2
            double vs3 = vs1 + dvs2 / 2;
            double phi3 = -Math.Atan(_curve.SlopeAt(x1 + dx2 / 2));
            double kappa3 = _curve.CurvatureAt(x1 + dx2 / 2);

            // Find k3 values        
            double dx3 = _ds * _dxds(phi3);
            double dz3 = _ds * _dzds(phi3);
            double dvs3 = _ds * _dv2ds(vs3, phi3, kappa3);

            // Find speed, slope and curvature using k3
            double vs4 = vs1 + dvs3;
            double phi4 = -Math.Atan(_curve.SlopeAt(x1 + dx3));
            double kappa4 = _curve.CurvatureAt(x1 + dx3);

            // Find k4 values
            double dx4 = _ds * _dxds(phi4);
            double dz4 = _ds * _dzds(phi4);
            double dvs4 = _ds * _dv2ds(vs4, phi4, kappa4);

            // Add the weighted averages for position and speed to their
            // respective initial values and append those values to the end of their lists
            _x.Add(x1 + (dx1 + 2 * dx2 + 2 * dx3 + dx4) / 6);
            _z.Add(z1 + (dz1 + 2 * dz2 + 2 * dz3 + dz4) / 6);
            double dvs = (dvs1 + 2 * dvs2 + 2 * dvs3 + dvs4) / 6;
            double v = 0.0;

            if (Math.Abs(dvs) > vs1)
                v = -Math.Sign(_v[_v.Count - 1]) * Math.Sqrt(-vs1 - dvs);
            else
                v = Math.Sign(_v[_v.Count - 1]) * Math.Sqrt(vs1 + dvs);
            _v.Add(v);
            double kappa = (kappa1 + 2 * kappa2 + 2 * kappa3 + kappa4) / 6;
            _kappa.Add(kappa);
            double phi = (phi1 + 2 * phi2 + 2 * phi3 + phi4) / 6;
            _phi.Add(phi);
            _ar.Add(g * Math.Cos(phi) + Math.Pow(v, 2.0) * kappa);
//            _at.Add(dv / _ds);
        }
    }
}

