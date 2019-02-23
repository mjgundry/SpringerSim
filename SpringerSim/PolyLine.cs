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

public class Polyline
{
    // X and Y cooredinates of knots
    private List<double> _xs;
    private List<double> _zs;

	public Polyline()
	{
	}

    public Polyline(List<double> xs, List<double> zs)
    {
        Debug.Assert((xs.Count == zs.Count) && (xs.Count >= 2));
        _xs = new List<double>(xs);
        _zs = new List<double>(zs);
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
    }

    // Get the "z" value at a point
    public double ValueAt(double x)
    {
        Debug.Assert((x >= _xs[0]) && (x <= _xs[_xs.Count - 1]));

        int i = 0;
        while (_xs[i + 1] < x)
            i++;

        double lx = x - _xs[i];

        return _zs[i] + lx * (_zs[i + 1] - _zs[i]) / (_xs[i + 1] - _xs[i]);
    }

    // Get the slope at a point
    public double SlopeAt(double x)
    {
        int i = 0;
        while (_xs[i + 1] < x)
            i++;

        return (_zs[i + 1] - _zs[i]) / (_xs[i + 1] - _xs[i]);
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
}
