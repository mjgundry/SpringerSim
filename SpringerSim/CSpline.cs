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
using MathNet.Numerics.LinearAlgebra;

public class CSpline
{
    // X and Y cooredinates of knots
    private List<double> _xs;
    private List<double> _zs;

    // cubic coefficients for the spline segments
    private List<double> _b;
    private List<double> _c;
    private List<double> _d;

    // Slope and the beginning and end, if clamped
    private double _alpha;
    private double _beta;

    private bool _startClamped = false;
    private bool _endClamped = false;
    private bool _isSolved = false;

	public CSpline()
	{
	}

    public CSpline(List<double> xs, List<double> zs)
    {
        Debug.Assert((xs.Count == zs.Count) && (xs.Count >= 2));
        _xs = new List<double>(xs);
        _zs = new List<double>(zs);
    }

    // Get/set the entrance slope of the spline
    public double Alpha
    {
        get
        {
            _generateCoefficients();
            if (_startClamped)
            {
                Debug.Assert(Math.Abs(_alpha - SlopeAt(_xs[0])) < 0.001);
                return _alpha;
            }
            else
            {
                return SlopeAt(_xs[0]);
            }
        }

        set
        {
            _alpha = value;
            _isSolved = false;
            _startClamped = true;
        }
    }

    // Get/set the exit slope of the spline
    public double Beta
    {
        get
        {
            _generateCoefficients();
            if (_endClamped)
            {
                Debug.Assert(Math.Abs(_beta - SlopeAt(_xs[_xs.Count - 1])) < 0.001);
                return _beta;
            }
            else
            {
                return SlopeAt(_xs[_xs.Count - 1]);
            }
        }

        set
        {
            _beta = value;
            _isSolved = false;
            _endClamped = true;
        }
    }

    // Horizontal coordinate values of knots
    public List<double> Xs => new List<double>(_xs);

    // Vertical coordinate values of knots
    public List<double> Zs => new List<double>(_zs);

    public void SetCoords(List<double> xs, List<double> zs)
    {
        Debug.Assert((xs.Count == zs.Count) && (xs.Count >= 2));
        _xs = new List<double>(xs);
        _zs = new List<double>(zs);
        _isSolved = false;
    }

    // Get the "z" value at a point
    public double ValueAt(double x)
    {
        Debug.Assert((x >= _xs[0]) && (x <= _xs[_xs.Count - 1]));
        _generateCoefficients();

        int i = 0;
        while (_xs[i + 1] < x)
            i++;

        double lx = x - _xs[i];

        return _zs[i] + _b[i] * lx + _c[i] * Math.Pow(lx, 2) + _d[i] * Math.Pow(lx, 3);
    }

    // Get the slope at a point
    public double SlopeAt(double x)
    {
        _generateCoefficients();

        // This is a natural spline, so if the data point requested is off the end of the
        // curve, return the slope for the end.
        if (x < _xs[0])
            x = _xs[0];

        if (x > _xs[_xs.Count - 1])
            x = _xs[_xs.Count - 1];

        int i = 0;
        while (_xs[i + 1] < x)
            i++;

        double lx = x - _xs[i];

        return _b[i] + 2 * _c[i] * lx + 3 * _d[i] * Math.Pow(lx, 2);
    }

    // Get the curvature at a point
    public double CurvatureAt(double x)
    {
        _generateCoefficients();

        // This is a natural spline, so if the data point requested is off the end of the
        // curve, return 0.
        if ((x < _xs[0]) || (x > _xs[_xs.Count - 1]))
            return 0.0;

        int i = 0;
        while (_xs[i + 1] < x)
            i++;

        double lx = x - _xs[i];
        double fpp = 2 * _c[i] + 6 * _d[i] * lx;
        double fp = _b[i] + 2 * _c[i] * lx + 3 * _d[i] * Math.Pow(lx, 2.0);

        return fpp / Math.Pow((1 + Math.Pow(fp, 2.0)), 1.5);
    }

    public double PathLength(double x1, double x2)
    {

        double ds = 0;
        double x = x1;
        double z1 = ValueAt(x);
        double z2;

        x += 0.1;

        while (x < x2)
        {
            z2 = z1;
            z1 = ValueAt(x);
            ds += Math.Sqrt(Math.Pow(0.1, 2.0) + Math.Pow(z2 - z1, 2.0));
            x += 0.1;
        }

        z2 = z1;
        z1 = ValueAt(x2);
        ds += Math.Sqrt(Math.Pow(0.1, 2.0) + Math.Pow(z2 - z1, 2.0));

        return ds;
    }

    private void _generateCoefficients()
    {
        if (_isSolved)
            return;

        int n = _xs.Count - 1;

        Matrix<double> A = Matrix<double>.Build.Dense(n + 1, n + 1, 0.0);
        Vector<double> B = Vector<double>.Build.Dense(n + 1, 0.0);

        for (int i = 1; i < n; i++)
        {
            A[i, i - 1] = _xs[i] - _xs[i - 1];
            A[i, i] = 2 * ((_xs[i] - _xs[i - 1]) + (_xs[i + 1] - _xs[i]));
            A[i, i + 1] = (_xs[i + 1] - _xs[i]);
            B[i] = 3 * (((_zs[i + 1] - _zs[i]) / (_xs[i + 1] - _xs[i])) -
                   ((_zs[i] - _zs[i - 1]) / (_xs[i] - _xs[i - 1])));
        }

        if (_startClamped)
        {
            A[0, 0] = 2 * (_xs[1] - _xs[0]);
            A[0, 1] = (_xs[1] - _xs[0]);
            B[0] = 3 * (((_zs[1] - _zs[0]) / (_xs[1] - _xs[0])) - _alpha);
        } else {
            A[0, 0] = 1.0;
            A[0, 1] = 0.0;
            B[0] = 0.0;
        }

        if (_endClamped)
        {
            A[n, n - 1] = (_xs[n] - _xs[n - 1]);
            A[n, n] = 2 * (_xs[n] - _xs[n - 1]);
            B[n] = 3 * (_beta - ((_zs[n] - _zs[n - 1]) / (_xs[n] - _xs[n - 1])));
        } else {
            A[n, n - 1] = 0.0;
            A[n, n] = 1.0;
            B[n] = 0.0;
        }

        _c = new List<double>(A.Solve(B).ToArray());

        // Precalculate the remaining cubic coefficients
        _b = new List<double>();
        _d = new List<double>();
        
        for (int i = 0; i < n; i++)
        {
            _b.Add((_zs[i + 1] - _zs[i]) / (_xs[i + 1] - _xs[i]) - (_xs[i + 1] - _xs[i]) / 3 * (2 * _c[i] + _c[i + 1]));
            _d.Add(1 / (3 * (_xs[i + 1] - _xs[i])) * (_c[i + 1] - _c[i]));
        }

        _isSolved = true;
    }
}
