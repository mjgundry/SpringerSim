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
    public enum Skill { Advanced=0, Intermediate=1, Novice=2, User, Default };

    public class Trajectory
    {
        // List of flight trajectory angles with the horizontal plane
        // to be used with the wind and lift coefficients to follow
        // phi = [-5, 0, 2, 4, 6, 8, 10, 12, 14, 16, 18, 20, 22, 24, 26, 28, 30, 32, 34, 36, 38, 40 ,42, 44, 50]
        private static readonly List<double> phi = new List<double>
        { -0.139626340, 0, 0.034906667, 0.069813333, 0.10472, 0.139626667, 0.174533333, 0.20944, 0.244346667,
            0.279253333, 0.31416, 0.349066667, 0.383973333, 0.41888, 0.453786667, 0.488693333, 0.5236,
            0.558506667, 0.593413333, 0.62832, 0.663226667, 0.698133333, 0.73304, 0.767946667, 0.8726646};

        // Blank wind and lift coefficients to be filled with user defined values
        private List<double> userkW = new List<double> { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
        private List<double> userkA = new List<double> { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };

        // Wind (drag) coefficients from Engelberg 2006
        // kW[0] is of the best jumpers, kW[2] is of the worst
        // These have been extrapolated by 2 degrees at each end
        private readonly List<double>[] kW = new [] {
            new List<double> { 0.001090, 0.00185, 0.00204, 0.00223, 0.00243, 0.00261, 0.00281, 0.00301, 0.00319, 0.00338, 0.00355, 0.00372,
                0.00388, 0.00403, 0.00418, 0.00432, 0.00447, 0.00462, 0.00479, 0.00502, 0.00537, 0.00614, 0.00691, 0.00767, 0.00995 },
            new List<double> { 0.00180, 0.00232, 0.00245, 0.00258, 0.00272, 0.00285, 0.00298, 0.00311, 0.00325, 0.00337, 0.00350, 0.00362,
                0.00374, 0.00386, 0.00398, 0.00410, 0.00422, 0.00436, 0.00453, 0.00474, 0.00504, 0.00553, 0.00602, 0.00651, 0.00798 },
            new List<double> { 0.00221, 0.00261, 0.00271, 0.00282, 0.00293, 0.00304, 0.00315, 0.00326, 0.00337, 0.00347, 0.00357, 0.00367,
                0.00376, 0.00386, 0.00396, 0.00407, 0.00419, 0.00432, 0.00449, 0.00471, 0.00503, 0.00555, 0.00606, 0.00658, 0.00814 } };

        // Lift coefficients from Engelberg 2006
        // kA[0] is of the best jumpers, kA[2] is of the worst
        private readonly List<double>[] kA = new [] {
            new List<double> { -0.00091, 0.00093, 0.00139, 0.00185, 0.00231, 0.00275, 0.00316, 0.00354, 0.00390, 0.00424, 0.00455, 0.00484,
                0.00511, 0.00534, 0.00555, 0.00574, 0.00591, 0.00605, 0.00617, 0.00628, 0.00638, 0.00655, 0.00672, 0.00689, 0.0074 },
            new List<double> { -0.00140, 0.00116, 0.00180, 0.00244, 0.00308, 0.00365, 0.00396, 0.00424, 0.00450, 0.00472, 0.00492, 0.00508,
                0.00522, 0.00534, 0.00543, 0.00550, 0.00555, 0.00560, 0.00565, 0.00571, 0.00582, 0.00606, 0.00629, 0.00652, 0.00721 },
            new List<double> { -0.00058, 0.00130, 0.00177, 0.00224, 0.00270, 0.00316, 0.00350, 0.00382, 0.00411, 0.00436, 0.00459, 0.00479,
                0.00496, 0.00510, 0.00521, 0.00531, 0.00538, 0.00545, 0.00551, 0.00558, 0.00569, 0.00590, 0.00611, 0.00632, 0.00695 } };

        private const double g = 9.80665;

        // Jumper skill for selecting flight coefficients
        // FIS 2008/2012 uses best set (0) for large hills (alpha 10 - 11.5)
        // and novice (2) for smaller hills (alpha 8 to 9.5)
        Skill _skill = Skill.Advanced;
        private double _dt = 0.01;

        // Speed perpendicular to table due to jump thrust
        // FIS 2015 uses 2.2 m/s for all hills. This is equivalent to a ~0.3m   
        // vertical jump.
        private double _vp = 2.2;

        // Initialize speed and takeoff angle to Fig 7 of FIS    
        // this should give a default h/n of 0.518 and beta of 32 degrees
        // for a k-point of 65m
        private double _v0 = 22.0;
        private double _W = 65.0;
        private double _HN = -1.0;
        private double _wind = 0;
        private double _alpha = 9.0 / 180.0 * Math.PI;

        // Array of speed, angle, horizontal displacement, and vertical displacement
        // values per time step thru trajectory
        private List<double> _v;
        private List<double> _phi;
        private List<double> _x;
        private List<double> _z;

        // For design, default to using the skill setting per FIS
        // However, allow the skill setting to be overridden to enable
        // off-design evaluation of a hill. If this is override is true,
        // then the code will not set the default skill when changing alpha
        private bool _overrideSkill = false;

        private bool _isSolved = false;
        private delegate void Solve();
        Solve _solve;

        private CSpline _kA;
        private CSpline _kW;

        private CSpline _hill;

        public Trajectory()
        {
            // setup interpolating splines for flight coefficients
            // linear interpolation probably works as well 
           // _kA = new CSpline[Enum.GetNames(typeof(Skill)).Length];
           // _kW = new CSpline[Enum.GetNames(typeof(Skill)).Length];

            _kA = new CSpline(phi, kA[(int)_skill]);
            _kW = new CSpline(phi, kW[(int)_skill]);

            // Solve the trajectory for a for known HN
            _solve = _solveForHN;
//            Calculating = false;
        }

//        public bool Calculating
//        {
//            get;
//            private set;
//        }

        // Vertical distance from the takeoff to landing
        public double Kz
        {
            get
            {
                _solve();
                return -_W * Math.Sin(Math.Atan(_HN)) / 1.005;
            }
        }

        // Horizontal distance from the takeoff to landing
        public double Kx
        {
            get
            {
                _solve();
                return _W * Math.Cos(Math.Atan(_HN)) / 1.005;
            }
        }

        // List of flight velocity values corresponding to x values
        public List<double> Vs
        {
            get
            {
                _solve();
                return new List<double>(_v);
            }
        }

        // List of flight angle values corresponding to x values
        public List<double> Phis
        {
            get
            {
                _solve();
                return new List<double>(_phi);
            }
        }

        // List of flight trajectory x values
        public List<double> Xs
        {
            get
            {
                _solve();
                return new List<double>(_x);
            }
        }

        // List of flight trajectory z values
        public List<double> Zs
        {
            get
            {
                _solve();
                return new List<double>(_z);
            }
        }

        // Speed at the landing
        public double LandingSpeed
        {
            get
            {
                _solve();
                return Math.Round(_v[_v.Count - 1], 2);
            }
        }

        // Flight angle at the landing
        public double LandingAngle
        {
            get
            {
                _solve();
                return _phi[_phi.Count - 1] * 180.0 / Math.PI;
            }
        }

        // Get/set the speed at the end of the takeoff
        // then sets the dirty bit
        public double TakeoffSpeed
        {
            get
            {
                return _v0;
            }
            set
            {
                _v0 = value;
                _isSolved = false;
            }
        }

        // Get the speed needed to reach a given
        // JumpDistance and Gradient
        public double RequiredSpeed
        {
            get
            {
                _solveForSpeed();
                return _v0;
            }
        }

        // Set/get the takeoff table angle
        // Per FIS 2012/2015, the table angle selects the flight skill, so select the appropriate
        // interpolating splines. Recalculate the initial trajectory angle, then set
        // the dirty bit
        public double TakeoffAngle
        {
            get
            {
                return _alpha * 180 / Math.PI;
            }
            set
            {
                Debug.Assert((value < 13.0) && (value > 5.0));
                _alpha = value * Math.PI / 180;
                if (value <= 9.5)
                {
                    _skill = Skill.Novice;
                } else
                {
                    _skill = Skill.Advanced;
                }
                _kA = new CSpline(phi, kA[(int)_skill]);
                _kW = new CSpline(phi, kW[(int)_skill]);

                _isSolved = false;
            }
        }

        // Set/get the jumper skill
        // Per FIS, the table angle selects the flight skill, so select the appropriate
        // interpolating splines. Recalculate the initial trajectory angle, then set
        // the dirty bit
        public Skill Ability
        {
            get => _skill;
            set
            {
                if (value == Skill.Default)
                {
                    if (_alpha >= 10.0 / 180.0 * Math.PI)
                        _skill = Skill.Advanced;
                    else
                        _skill = Skill.Novice;
                }
                else
                {
                    _skill = value;
//                    _overrideSkill = true;
                }
                _kA = new CSpline(phi, kA[(int)_skill]);
                _kW = new CSpline(phi, kW[(int)_skill]);

                _isSolved = false;
            }
        }

        // Wind speed in m/s. Negative is tailwind, positive is headwind
        public double WindSpeed
        {
            get => _wind; set
            {
                _wind = value;
                _isSolved = false;
            }
        }

        // Get/set desired W (AKA jump distance or K-Point for design flight) then set the solver for h/n
        // Bounds check W
        public double JumpDistance
        {
            get
            {
                _solve();
                return _W;
            }
            set
            {
                Debug.Assert((value <= 130) && (value >= 5));
                _W = value;
                _isSolved = false;
                _solve = _solveForHN;
            }
        }

        // Get/set desired h/n and indicate to solve for W (AKA K-point)
        public double Gradient
        {
            get
            {
                _solve();
                return _HN;
            }
            set
            {
                Debug.Assert((value <= 0.60) && (value >= 0.40));
                _HN = value;
                _isSolved = false;
                _solve = _solveForW;
            }
        }

        // Get/set velocity vertical to the takeoff table due to the jumper "jumping".
        // Normally 2.2 m/s for FIS designs, setting -1 returns to default.
        // To be used for off-design evaluation
        public double PushSpeed
        {
            get
            {
                return _vp;
            }
            set
            {
                if (value == -1)
                    _vp = 2.2;
                else
                {
                    Debug.Assert((value > 0.0) && (value <= 4.4));
                    _vp = value;
                }
                _isSolved = false;
            }
        }

        public ProfilePoint MaxAGL(CSpline hill)
        {
            double maxY = 0;
            double maxX = 0;

            for(int i=0;i < _x.Count - 1;i++)
            {
                double y = _z[i] - hill.ValueAt(_x[i]);
                if (y > maxY)
                {
                    maxY = y;
                    maxX = _x[i];
                }
            }

            return new ProfilePoint() { X = maxX, Y = maxY };
        }

        // ODE for rate of change in horizontal position based on current speed and angle
        private double _dxdt(double v, double phi)
        {
            return v * Math.Cos(phi);
        }

        // ODE for rate of change in vertical position based on current speed and angle
        private double _dzdt(double v, double phi)
        {
            return -v * Math.Sin(phi);
        }

        // ODE for rate of change in speed based on current speed and angle
        private double _dvdt(double v, double phi)
        {
            // Get the drag coefficient for this trajectory angle
            double mykW = _kW.ValueAt(phi);
            double vW = _wind * Math.Cos(phi);
            return g * Math.Sin(phi) - mykW * Math.Pow(v + vW, 2.0);
        }

        // ODE for rate of change in angle based on current speed and angle
        // This equation takes advantage of the property: sin(angle) ~= angle in radians
        private double _dphidt(double v, double phi)
        {
            // Get the lift coefficient for this trajectory angle
            double mykA = _kA.ValueAt(phi);
            double vW = _wind * Math.Cos(phi);
            return (g * Math.Cos(phi) - mykA * Math.Pow(v + vW, 2.0)) / v;
        }

        // Find the h/n that gives the desired W given the initial trajectory conditions
        private void _solveForHN()
        {
            if (_isSolved)
                return;
//            Calculating = true;

            // delete the previous trajectory data except for the initial conditions at [0]
            _v = new List<double>() { Math.Sqrt(Math.Pow(_v0, 2.0) + Math.Pow(_vp, 2.0)) };
            _phi = new List<double>() { _alpha - Math.Atan(_vp / _v0) };
            _x = new List<double>() { 0.0 };
            _z = new List<double>() { 0.0 };

            double w2 = 0.0;
            double w1 = 0.0;

            while (w2 < _W)
            {
                Debug.Assert(_phi[_phi.Count - 1] < 0.8);
                _advanceTimeStep();
                w1 = w2;
                w2 = 1.005 * _x[_x.Count - 1] / Math.Cos(Math.Atan(-_z[_z.Count - 1] / _x[_x.Count - 1]));
            }

            double zl = _lInt(w1, w2, _z[_z.Count - 2], _z[_z.Count - 1], _W);
            double xl = _lInt(w1, w2, _x[_x.Count - 2], _x[_x.Count - 1], _W);
            double vl = _lInt(w1, w2, _v[_v.Count - 2], _v[_v.Count - 1], _W);
            double phil = _lInt(w1, w2, _phi[_phi.Count - 2], _phi[_phi.Count - 1], _W);

            // Set last trajectory data at K-point
            _z[_z.Count - 1] = zl;
            _x[_x.Count - 1] = xl;
            _v[_v.Count - 1] = vl;
            _phi[_phi.Count - 1] = phil;
            _HN = -_z[_z.Count - 1] / _x[_x.Count - 1];
            // Check that the calculated W matches what we targeted
            // If this fails often, either use another interpolation method
            // or reduce the time step
            Debug.Assert(Math.Abs(_W - (1.005 * xl / Math.Cos(Math.Atan(-zl / xl)))) < 0.01);
            _isSolved = true;
//            Calculating = false;
        }

        // Find the W that gives the desired h/n given the initial trajectory conditions
        private void _solveForW()
        {
            if (_isSolved)
                return;
//            Calculating = true;

            // delete the previous trajectory data except for the initial conditions at [0]
            _v = new List<double>() { Math.Sqrt(Math.Pow(_v0, 2.0) + Math.Pow(_vp, 2.0)) };
            _phi = new List<double>() { _alpha - Math.Atan(_vp / _v0) };
            _x = new List<double>() { 0.0 };
            _z = new List<double>() { 0.0 };

            // Keep advancing time steps until the computed trajectory is beyond desired h/n
            double hn2 = 0;
            double hn1 = 0;

            while (hn2 < _HN)
            {
                // If the trajectory angle gets too steep, stop the run and indicate invalid solution
                Debug.Assert(_phi[_phi.Count - 1] < 0.8726646);
                _advanceTimeStep();
                hn1 = hn2;
                hn2 = -_z[_z.Count - 1] / _x[_x.Count - 1];
            }

            int lastStep = _x.Count - 1;
            _x.RemoveAt(lastStep);
            _z.RemoveAt(lastStep);
            _v.RemoveAt(lastStep);
            _phi.RemoveAt(lastStep);

            hn2 = -_z[_z.Count - 1] / _x[_x.Count - 1];

            _dt /= 10;
            while (hn2 < _HN)
            {
                _advanceTimeStep();
                hn1 = hn2;
                hn2 = -_z[_z.Count - 1] / _x[_x.Count - 1];
            }
            _dt *= 10;

            // Interpolate trajectory data at desired h/n
            double zl = _lInt(hn1, hn2, _z[_z.Count - 2], _z[_z.Count - 1], _HN);
            double xl = _lInt(hn1, hn2, _x[_x.Count - 2], _x[_x.Count - 2], _HN);
            double vl = _lInt(hn1, hn2, _v[_v.Count - 2], _v[_v.Count - 1], _HN);
            double phil = _lInt(hn1, hn2, _phi[_phi.Count - 2], _phi[_phi.Count - 1], _HN);

            int range = _x.Count - 1 - lastStep;
            _x.RemoveRange(lastStep, range);
            _z.RemoveRange(lastStep, range);
            _v.RemoveRange(lastStep, range);
            _phi.RemoveRange(lastStep, range);

            // Set last trajectory data at K-point
            _z[_z.Count - 1] = zl;
            _x[_x.Count - 1] = xl;
            _v[_v.Count - 1] = vl;
            _phi[_phi.Count - 1] = phil;

            _W = 1.005 * _x[_x.Count - 1] / Math.Cos(Math.Atan(-_z[_z.Count - 1] / _x[_x.Count - 1]));
            // Check that the interpolated h/n matches what we targeted
            // If this fails often, either use another interpolation method
            // or reduce the time step
            Debug.Assert(Math.Abs(_HN + _z[_z.Count - 1] / _x[_x.Count - 1]) < 0.001);
            _isSolved = true;
//            Calculating = false;
        }

        // Find the trajectory up to the landing point on an arbitrary surface
        // given as a cspline.
        private void _solveForLanding()
        {
//            Calculating = true;

            // delete the previous trajectory data except for the initial conditions at [0]
            _v = new List<double>() { Math.Sqrt(Math.Pow(_v0, 2.0) + Math.Pow(_vp, 2.0)) };
            _phi = new List<double>() { _alpha - Math.Atan(_vp / _v0) };
            _x = new List<double>() { 0.0 };
            _z = new List<double>() { 0.0 };

            // Invalidate computed parameters, as these are meaningless in this context
            _W = -1.0;
            _HN = -1.0;

            // Keep advancing time steps until the computed trajectory is below the hill surface
            //double x = 0;
            //double z = 0;

            while (_z[_z.Count - 1] > _hill.ValueAt(_x[_x.Count - 1]))
            {
                // If the trajectory angle gets too steep, stop the run
                Debug.Assert(_phi[_phi.Count - 1] < 0.8726646);
                _advanceTimeStep();
            }

            // Now that we're at the first time step where the trajectory goes below
            // the hill, back up one time step, and come forward with a much smaller
            // step value until we re-intercept. For an initial dt value of 0.01 s,
            // the reduced time step will find the intersection within 1 cm even for
            // landing speeds in excess of 30 m/s
            //_dt /= 100;

            // Delete the trajectory data from the last time step
            _x.RemoveAt(_x.Count - 1);
            _z.RemoveAt(_z.Count - 1);
            _v.RemoveAt(_v.Count - 1);
            _phi.RemoveAt(_phi.Count - 1);

            while (_z[_z.Count - 1] > _hill.ValueAt(_x[_x.Count - 1]))
                _advanceTimeStep();

            // Reinstate the old time step
            _dt *= 100;
//            Calculating = false;
        }

        // Find the takeoff speed required to fly to a point defined by HN and W.
        // The current TakeoffSpeed is used as initial value.
        private void _solveForSpeed()
        {
//            Calculating = true;

            double S = _v0;
            double WP = 0;

            while (Math.Abs(_W - WP) > 0.1)
            {
                // delete the previous trajectory data except for the initial conditions at [0]
                _v = new List<double>() { Math.Sqrt(Math.Pow(S, 2.0) + Math.Pow(_vp, 2.0)) };
                _phi = new List<double>() { _alpha - Math.Atan(_vp / S) };
                _x = new List<double>() { 0.0 };
                _z = new List<double>() { 0.0 };

                // Keep advancing time steps until the computed trajectory is beyond desired h/n
                double hn2 = 0;
                double hn1 = 0;
                _dt /= 10;

                while (hn2 < _HN)
                {
                    // If the trajectory angle gets too steep, stop the run and indicate invalid solution
                    Debug.Assert(_phi[_phi.Count - 1] < 0.8726646);
                    _advanceTimeStep();
                    hn1 = hn2;
                    hn2 = -_z[_z.Count - 1] / _x[_x.Count - 1];
                }

                _dt *= 10;

                WP = 1.005 * _x[_x.Count - 1] / Math.Cos(Math.Atan(-_z[_z.Count - 1] / _x[_x.Count - 1]));

                S = (1 + 0.2 * (_W - WP) / _W) * S;
            }

            _v0 = S;
//            Calculating = true;
        }

        // Linear interpolation 
        private double _lInt(double x1, double x2, double y1, double y2, double x)
        {
            return (x - x1) / (x2 - x1) * (y2 - y1) + y1;
        }

        // Calculate trajectory data at the next time step by solving the system
        // of ODEs using the 4th order Runge Kutta method
        // Since the function for position change is independent of the current position
        // RK4 is simply Simpson's Rule for x and z
        private void _advanceTimeStep()
        {
            // Get position, speed, and direction from last time step
            double x1 = _x[_x.Count - 1];
            double z1 = _z[_z.Count - 1];
            double v1 = _v[_v.Count - 1];
            double phi1 = _phi[_phi.Count - 1];

            // Find k1 values
            double dx1 = _dt * _dxdt(v1, phi1);
            double dz1 = _dt * _dzdt(v1, phi1);
            double dv1 = _dt * _dvdt(v1, phi1);
            double dphi1 = _dt * _dphidt(v1, phi1);

            // Find speed and direction using k1
            double v2 = v1 + dv1 / 2;
            double phi2 = phi1 + dphi1 / 2;

            // Find k2 values
            double dx2 = _dt * _dxdt(v2, phi2);
            double dz2 = _dt * _dzdt(v2, phi2);
            double dv2 = _dt * _dvdt(v2, phi2);
            double dphi2 = _dt * _dphidt(v2, phi2);

            // Find speed and direction using k2
            double v3 = v1 + dv2 / 2;
            double phi3 = phi1 + dphi2 / 2;

            // Find k3 values
            double dx3 = _dt * _dxdt(v3, phi3);
            double dz3 = _dt * _dzdt(v3, phi3);
            double dv3 = _dt * _dvdt(v3, phi3);
            double dphi3 = _dt * _dphidt(v3, phi3);

            // Find speed and direction using k3
            double v4 = v1 + dv3;
            double phi4 = phi1 + dphi3;

            // Find k4 values
            double dx4 = _dt * _dxdt(v4, phi4);
            double dz4 = _dt * _dzdt(v4, phi4);
            double dv4 = _dt * _dvdt(v4, phi4);
            double dphi4 = _dt * _dphidt(v4, phi4);

            // Add the weighted averages for position, speed and direction to their
            // respective initial values and append those values to the end of their lists
            _x.Add(x1 + (dx1 + 2 * dx2 + 2 * dx3 + dx4) / 6);
            _z.Add(z1 + (dz1 + 2 * dz2 + 2 * dz3 + dz4) / 6);
            _v.Add(v1 + (dv1 + 2 * dv2 + 2 * dv3 + dv4) / 6);
            _phi.Add(phi1 + (dphi1 + 2 * dphi2 + 2 * dphi3 + dphi4) / 6);
        }
    }
}