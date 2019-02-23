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

namespace SpringerSim
{
    [Serializable()]
    public class Project
    {
        public Project()
        {
            FileVersion = "1";
            Existing = new Existing();
            Proposed = new Proposed();

            Proposed.Inrun.RampAngle = 32.5;
            Proposed.Inrun.TakeoffAngle = 10.2;
            Proposed.Landing.TakeoffAngle = 10.2;
            Proposed.Inrun.TakeoffSpeed = 20.26;
            Proposed.Landing.TakeoffSpeed = 20.26;
            Proposed.Landing.W = 50.0;
            Proposed.Landing.DeltaBeta = 1.0;
            Proposed.Landing.EquivalentLandingHeight = 0.35;
        }

        [XmlAttribute]
        public string FileVersion { get; }
        public string ProjectName { get; set; }
        public string SiteName { get; set; }
        public string HillName { get; set; }
        public string Description { get; set; }
        public string Longitude { get; set; }
        public string Latitude { get; set; }
        public string Elevation { get; set; }
        public string Designer { get; set; }
        public string Checker { get; set; }
        public string Comments { get; set; }

        public Existing Existing { get; set; }

        public Proposed Proposed { get; set; }

        [XmlIgnore]
        public string Filename { get; set; }
    }
}
