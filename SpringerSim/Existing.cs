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
using System.ComponentModel;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace SpringerSim
{
    public enum ProfilePointType {none=0, A=1, B=2, E1=3, E2=4, T=5, TL=6, P=7, K=8, L=9, U=10, a=11 }

    [Serializable()]
    public class ProfilePoint
    {
        public ProfilePoint()
        {

        }

        public double X { get; set; }
        public double Y { get; set; }
        public ProfilePointType Type { get; set; }
        
    }

    [Serializable()]
    public class Existing
    {
        public BindingList<ProfilePoint> Points { get; set; }

        public Existing()
        {
            Points = new BindingList<ProfilePoint>();
        }
    }
}
