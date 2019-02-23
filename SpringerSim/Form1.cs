using System;
using System.Collections.Generic;
using System.Drawing;
using System.Drawing.Drawing2D;
using System.Windows.Forms;
using System.Xml.Serialization;

namespace SpringerSim
{
    public partial class Form1 : Form
    {
        private Project project;
        private System.Windows.Forms.Timer _timer = new System.Windows.Forms.Timer();
        private ProjectInformationDialog projectInformationDialog = new ProjectInformationDialog();
        private float _zoomDgn = 1.0f;
        private float _zoomExist = 1.0f;
        private float _zoomEval = 1.0f;
        private float _xDgn, _xExist, _xEval, _yProf, _yPerf, _yExist, _yEval;
        private float radiusMarker = 0.5f;
        Point mouseDown;
        Point LastPanelMousePosition, LastPerformanceMousePosition;

        // Point collections for the drawing panels
        private PointF[] inrunRAPoints;
        private PointF[] inrunPhiPoints;
        private PointF[] inrunKappaPoints;
        private PointF[] landingRAPoints;
        private PointF[] landingPhiPoints;
        private PointF[] landingKappaPoints;
        private PointF[] speedPoints;
        private PointF[] flightPhiPoints;
        private PointF[] flightAGLPoints;

        public Form1()
        {
            InitializeComponent();
            project = new Project();

            // 
            _timer.Enabled = true;
            _timer.Start();
            _timer.Tick += new System.EventHandler(ProcessParameters);

            dataGridView1.DataSource = project.Existing.Points;

            UpdateProposed();
            zoomDesignExtents(panelDesign);
        }

        private void ProcessParameters(Object source, System.EventArgs e)
        {
            UpdateProposed();
            _timer.Stop();
        }

        private void UpdateProposed()
        {
//            while (project.Proposed.Landing.Calculating)
//            {
//                System.Threading.Thread.Sleep(100);
//            }

            numericUpDownGamma.Value = (decimal)project.Proposed.Inrun.RampAngle;
            if (project.Proposed.Landing.W >= 90)
            {
                if (project.Proposed.Inrun.RampAngle > 37)
                {
                    numericUpDownGamma.BackColor = Color.Red;
                    gammaTip.Active = true;
                    gammaTip.SetToolTip(numericUpDownGamma, "Ramp angle must be less than or equal to 37°");
                }
                else if (project.Proposed.Inrun.RampAngle > 35)
                {
                    numericUpDownGamma.BackColor = Color.Yellow;
                    gammaTip.Active = true;
                    gammaTip.SetToolTip(numericUpDownGamma, "Recommended maximum ramp angle is 35°");
                }
                else if(project.Proposed.Inrun.RampAngle < 30)
                {
                    numericUpDownGamma.BackColor = Color.Yellow;
                    gammaTip.Active = true;
                    gammaTip.SetToolTip(numericUpDownGamma, "Recommended minimum ramp angle is 30°");
                }
                else
                {
                    numericUpDownGamma.BackColor = Color.White;
                    gammaTip.Active = false;
                }
            }
            else if (project.Proposed.Landing.W >= 30 && project.Proposed.Landing.W < 90)
            {
                if (project.Proposed.Inrun.RampAngle > 37)
                {
                    numericUpDownGamma.BackColor = Color.Red;
                    gammaTip.Active = true;
                    gammaTip.SetToolTip(numericUpDownGamma, "Ramp angle must be less than or equal to 37°");
                }
                else if (project.Proposed.Inrun.RampAngle > 35)
                {
                    numericUpDownGamma.BackColor = Color.Yellow;
                    gammaTip.Active = true;
                    gammaTip.SetToolTip(numericUpDownGamma, "Recommended maximum ramp angle is 35°");
                }
                else if (project.Proposed.Inrun.RampAngle < 25)
                {
                    numericUpDownGamma.BackColor = Color.Yellow;
                    gammaTip.Active = true;
                    gammaTip.SetToolTip(numericUpDownGamma, "Recommended minimum ramp angle is 25°");
                }
                else
                {
                    numericUpDownGamma.BackColor = Color.White;
                    gammaTip.Active = false;
                }
            }
            else if (project.Proposed.Landing.W < 30)
            {
                if (project.Proposed.Inrun.RampAngle > 37)
                {
                    numericUpDownGamma.BackColor = Color.Red;
                    gammaTip.Active = true;
                    gammaTip.SetToolTip(numericUpDownGamma, "Ramp angle must be less than or equal to 37°");
                }
                else if (project.Proposed.Inrun.RampAngle > 35)
                {
                    numericUpDownGamma.BackColor = Color.Yellow;
                    gammaTip.Active = true;
                    gammaTip.SetToolTip(numericUpDownGamma, "Recommended maximum ramp angle is 32°");
                }
                else if (project.Proposed.Inrun.RampAngle < 25)
                {
                    numericUpDownGamma.BackColor = Color.Yellow;
                    gammaTip.Active = true;
                    gammaTip.SetToolTip(numericUpDownGamma, "Recommended minimum ramp angle is 25°");
                }
                else
                {
                    numericUpDownGamma.BackColor = Color.White;
                    gammaTip.Active = false;
                }
            }

            numericUpDownTakeoffSpeed.Value = (decimal)project.Proposed.Inrun.TakeoffSpeed;

            numericUpDownAlpha.Value = (decimal)project.Proposed.Inrun.TakeoffAngle;
            if (project.Proposed.Inrun.TakeoffAngle < project.Proposed.Landing.W / 30 + 6.9)
            {
                numericUpDownAlpha.BackColor = Color.Yellow;
                alphaTip.Active = true;
                alphaTip.SetToolTip(numericUpDownAlpha, "Takeoff angle is below recommended range");
                
            }
            else if (project.Proposed.Inrun.TakeoffAngle > project.Proposed.Landing.W / 30 + 7.9)
            {
                numericUpDownAlpha.BackColor = Color.Yellow;
                alphaTip.Active = true;
                alphaTip.SetToolTip(numericUpDownAlpha, "Takeoff angle is above recommended range");
            }
            else
            {
                numericUpDownAlpha.BackColor = Color.White;
                alphaTip.Active = false;
            }

            numericUpDownR1.Value = (decimal)project.Proposed.Inrun.TransitionRadius;
            numericUpDownTableLength.Value = (decimal)project.Proposed.Inrun.TableLength;
            numericUpDownInrunFrictionAngle.Value = (decimal)project.Proposed.Inrun.Rho;

            numericUpDownW.Value = (decimal)project.Proposed.Landing.W;
            numericUpDownGradient.Value = (decimal)project.Proposed.Landing.Gradient;
            if (project.Proposed.Landing.Gradient < project.Proposed.Landing.W / 800 + 0.400)
            {
                numericUpDownGradient.BackColor = Color.Yellow;
                gradientTip.Active = true;
                gradientTip.SetToolTip(numericUpDownGradient, "Hill gradient (h/n) is below the recommended range");
            }
            else if (project.Proposed.Landing.Gradient > project.Proposed.Landing.W / 1000 + 0.480)
            {
                numericUpDownGradient.BackColor = Color.Yellow;
                gradientTip.Active = true;
                gradientTip.SetToolTip(numericUpDownGradient, "Hill gradient (h/n) is below the recommended range");
            }
            else
            {
                numericUpDownGradient.BackColor = Color.White;
                gradientTip.Active = false;
            }

            numericUpDownHillSize.Value = (decimal)project.Proposed.Landing.HillSize;
            numericUpDownTOHeight.Value = (decimal)project.Proposed.Landing.TakeoffHeight;
            numericUpDownEqLandingHeight.Value = (decimal)project.Proposed.Landing.EquivalentLandingHeight;
            numericUpDownAngleAtK.Value = (decimal)project.Proposed.Landing.AngleAtK;
            numericUpDownBeta0.Value = (decimal)project.Proposed.Landing.AngleAtTakeoff;
            numericUpDownBetaP.Value = (decimal)project.Proposed.Landing.AngleAtP;
            numericUpDownBetaL.Value = (decimal)project.Proposed.Landing.AngleAtL;
            numericUpDownDeltaBeta.Value = (decimal)project.Proposed.Landing.DeltaBeta;
            numericUpDownLandingRadius.Value = (decimal)project.Proposed.Landing.LandingRadius;
            numericUpDownTransRadiusL.Value = (decimal)project.Proposed.Landing.TransitionRadiusAtL;
            numericUpDownTransRadiusU.Value = (decimal)project.Proposed.Landing.TransitionRadiusAtU;

            project.Proposed.Inrun.TopStartSpeed = project.Proposed.Landing.MaximumSpeed;
            project.Proposed.Inrun.BottomStartSpeed = project.Proposed.Landing.MinimumSpeed;

            numericUpDownInrunLength.Value = (decimal)project.Proposed.Inrun.InrunLength;
            numericUpDownStartLength.Value = (decimal)project.Proposed.Inrun.StartLength;

            panelDesign.Invalidate();
            panelPerformance.Invalidate();
        }

        // Wait 1/4 second before refreshing the interface after changing a parameter.
        // This prevents a stackup of parameter changes.
        private void SetTimer()
        {
            _timer.Stop();
            _timer.Interval = 250;
            _timer.Start();
        }

        private void numericUpDownGamma_ValueChanged(object sender, EventArgs e)
        {
            project.Proposed.Inrun.RampAngle = (double)numericUpDownGamma.Value;
            SetTimer();
        }

        private void numericUpDownTakeoffSpeed_ValueChanged(object sender, EventArgs e)
        {
            project.Proposed.Inrun.TakeoffSpeed = (double)numericUpDownTakeoffSpeed.Value;
            project.Proposed.Landing.TakeoffSpeed = (double)numericUpDownTakeoffSpeed.Value;
            SetTimer();
        }

        private void numericUpDownAlpha_ValueChanged(object sender, EventArgs e)
        {
            project.Proposed.Inrun.TakeoffAngle = (double)numericUpDownAlpha.Value;
            project.Proposed.Landing.TakeoffAngle = (double)numericUpDownAlpha.Value;
            SetTimer();
        }

        private void numericUpDownInrunFrictionAngle_ValueChanged(object sender, EventArgs e)
        {
            project.Proposed.Inrun.Rho = (double)numericUpDownInrunFrictionAngle.Value;
            SetTimer();
        }

        private void radioButtonW_CheckedChanged(object sender, EventArgs e)
        {
            radioButtonHN.Checked = !radioButtonW.Checked;
            numericUpDownW.Enabled = true;
            numericUpDownW.ReadOnly = false;
            numericUpDownGradient.Enabled = false;
            numericUpDownGradient.ReadOnly = true;
            project.Proposed.Landing.W = (double)numericUpDownW.Value;
            SetTimer();
        }

        private void radioButtonHN_CheckedChanged(object sender, EventArgs e)
        {
            radioButtonW.Checked = !radioButtonHN.Checked;
            numericUpDownW.Enabled = false;
            numericUpDownW.ReadOnly = true;
            numericUpDownGradient.Enabled = true;
            numericUpDownGradient.ReadOnly = false;
            project.Proposed.Landing.Gradient = (double)numericUpDownGradient.Value;
            SetTimer();
        }

        private void radioButtonEqLandingHeight_CheckedChanged(object sender, EventArgs e)
        {
            radioButtonAngleAtK.Checked = !radioButtonEqLandingHeight.Checked;
            numericUpDownEqLandingHeight.Enabled = true;
            numericUpDownEqLandingHeight.ReadOnly = false;
            numericUpDownAngleAtK.Enabled = false;
            numericUpDownAngleAtK.ReadOnly = true;
            project.Proposed.Landing.EquivalentLandingHeight = (double)numericUpDownEqLandingHeight.Value;
            SetTimer();
        }

        private void radioButtonAngleAtK_CheckedChanged(object sender, EventArgs e)
        {
            radioButtonEqLandingHeight.Checked = !radioButtonAngleAtK.Checked;
            numericUpDownAngleAtK.Enabled = true;
            numericUpDownAngleAtK.ReadOnly = false;
            numericUpDownEqLandingHeight.Enabled = false;
            numericUpDownEqLandingHeight.ReadOnly = true;
            project.Proposed.Landing.AngleAtK = (double)numericUpDownAngleAtK.Value;
            SetTimer();
        }

        private void numericUpDownW_ValueChanged(object sender, EventArgs e)
        {
            project.Proposed.Landing.W = (double)numericUpDownW.Value;
            SetTimer();
        }

        private void numericUpDownGradient_ValueChanged(object sender, EventArgs e)
        {
            project.Proposed.Landing.Gradient = (double)numericUpDownGradient.Value;
            SetTimer();
        }

        private void numericUpDownEqLandingHeight_ValueChanged(object sender, EventArgs e)
        {
            project.Proposed.Landing.EquivalentLandingHeight = (double)numericUpDownEqLandingHeight.Value;
            SetTimer();    
        }

        private void numericUpDownDeltaBeta_ValueChanged(object sender, EventArgs e)
        {
            project.Proposed.Landing.DeltaBeta = (double)numericUpDownDeltaBeta.Value;
            SetTimer();
        }

        private void numericUpDownAngleAtK_ValueChanged(object sender, EventArgs e)
        {
            project.Proposed.Landing.AngleAtK = (double)numericUpDownAngleAtK.Value;
            SetTimer();
        }

        private void numericUpDownTransRadiusL_ValueChanged(object sender, EventArgs e)
        {
            project.Proposed.Landing.TransitionRadiusAtL = (double)numericUpDownTransRadiusL.Value;

            if(project.Proposed.Landing.TransitionRadiusAtL > project.Proposed.Landing.LandingRadius)
            {
                numericUpDownTransRadiusL.BackColor = Color.Red;
                toolTip.SetToolTip(numericUpDownTransRadiusL, "Transition radius at L is too large");
            }

            SetTimer();
        }

        private void numericUpDownTransRadiusU_ValueChanged(object sender, EventArgs e)
        {
            project.Proposed.Landing.TransitionRadiusAtU = (double)numericUpDownTransRadiusU.Value;
            SetTimer();
        }

        private void panelDesign_Paint(object sender, PaintEventArgs e)
        {
            Panel p = sender as Panel;
            Graphics g = e.Graphics;
            GraphicsPath inrun = new GraphicsPath();
            GraphicsPath landing = new GraphicsPath();
            GraphicsPath kFlight = new GraphicsPath();
            Pen greenPen = new Pen(Color.LightGreen, 0.1f);
            greenPen.DashStyle = DashStyle.Dash;
            Pen bluePen = new Pen(Color.DarkBlue, 0.2f);
            Pen redPen = new Pen(Color.Red, 0.1f);

            g.ScaleTransform(_zoomDgn, _zoomDgn);
            g.TranslateTransform(_xDgn, _yProf);

            List<double> xs = project.Proposed.Inrun.Xs;
            List<double> zs = project.Proposed.Inrun.Zs;

            PointF[] inrunPoints = new PointF[xs.Count];

            inrunPoints[0] = new PointF((float)project.Proposed.Inrun.Ax, -(float)project.Proposed.Inrun.Az);

            for (int i = 1; i < xs.Count; i++)
            {
                inrunPoints[i] = new PointF((float)xs[i], -(float)zs[i]);
            }

            xs = project.Proposed.Landing.Xs;
            zs = project.Proposed.Landing.Zs;
            
            PointF[] landingPoints = new PointF[xs.Count];

            for (int i = 0; i < xs.Count; i++)
            {
                landingPoints[i] = new PointF((float)xs[i], -(float)zs[i]);
            }

            xs = project.Proposed.Landing.KFlightXs;
            zs = project.Proposed.Landing.KFlightZs;

            PointF[] kFlightPoints = new PointF[xs.Count];

            for (int i = 0; i < xs.Count; i++)
            {
                kFlightPoints[i] = new PointF((float)xs[i], -(float)zs[i]);
            }

            inrun.AddCurve(inrunPoints);
            landing.AddCurve(landingPoints);
            kFlight.AddCurve(kFlightPoints);

            g.SmoothingMode = SmoothingMode.AntiAlias;

            g.DrawEllipse(redPen,
                (float)project.Proposed.Inrun.Ax - radiusMarker,
                -(float)project.Proposed.Inrun.Az - radiusMarker,
                radiusMarker * 2.0f, radiusMarker * 2.0f);
            g.DrawEllipse(redPen,
                (float)project.Proposed.Inrun.Bx - radiusMarker,
                -(float)project.Proposed.Inrun.Bz - radiusMarker,
                radiusMarker * 2.0f, radiusMarker * 2.0f);
            g.DrawEllipse(redPen,
                (float)project.Proposed.Inrun.E1x - radiusMarker,
                -(float)project.Proposed.Inrun.E1z - radiusMarker,
                radiusMarker * 2.0f, radiusMarker * 2.0f);
            g.DrawEllipse(redPen,
                (float)project.Proposed.Inrun.E2x - radiusMarker,
                -(float)project.Proposed.Inrun.E2z - radiusMarker,
                radiusMarker * 2.0f, radiusMarker * 2.0f);
            g.DrawEllipse(redPen, -radiusMarker, -radiusMarker,
                radiusMarker * 2.0f, radiusMarker * 2.0f);
            g.DrawEllipse(redPen,
                -radiusMarker,
                (float)project.Proposed.Landing.TakeoffHeight - radiusMarker,
                radiusMarker * 2.0f, radiusMarker * 2.0f);
            g.DrawEllipse(new Pen(Color.Red, 0.1f),
                (float)project.Proposed.Landing.Px - radiusMarker,
                -(float)project.Proposed.Landing.Pz - radiusMarker,
                radiusMarker * 2.0f, radiusMarker * 2.0f);
            g.DrawEllipse(redPen,
                (float)project.Proposed.Landing.Kx - radiusMarker,
                -(float)project.Proposed.Landing.Kz - radiusMarker,
                radiusMarker * 2.0f, radiusMarker * 2.0f);
            g.DrawEllipse(redPen,
                (float)project.Proposed.Landing.Lx - radiusMarker,
                -(float)project.Proposed.Landing.Lz - radiusMarker,
                radiusMarker * 2.0f, radiusMarker * 2.0f);
            g.DrawEllipse(redPen,
                (float)project.Proposed.Landing.Ux - radiusMarker,
                -(float)project.Proposed.Landing.Uz - radiusMarker,
                radiusMarker * 2.0f, radiusMarker * 2.0f);
            g.DrawPath(bluePen, inrun);
            g.DrawPath(bluePen, landing);
            g.DrawPath(greenPen, kFlight);
        }

        private void panelPerformance_Paint(object sender, PaintEventArgs e)
        {
            Panel p = sender as Panel;
            Graphics g = e.Graphics;
            GraphicsPath inrunRA = new GraphicsPath();
            GraphicsPath landingRA = new GraphicsPath();
            GraphicsPath speed = new GraphicsPath();
            GraphicsPath flightAGL = new GraphicsPath();
            GraphicsPath flightPhi = new GraphicsPath();
            GraphicsPath inrunAngle = new GraphicsPath();
            GraphicsPath landingAngle = new GraphicsPath();
            GraphicsPath inrunRadius = new GraphicsPath();
            GraphicsPath landingRadius = new GraphicsPath();
            Pen blackPen = new Pen(Color.Black, 0.1f);
            Pen orangePen = new Pen(Color.DarkOrange, 0.1f);
            Pen darkGrayPen = new Pen(Color.DarkGray, 0.1f);
            Pen lightGrayPen = new Pen(Color.LightGray, 0.1f);
            Pen greenPen = new Pen(Color.Green, 0.1f);
            Pen violetPen = new Pen(Color.Violet, 0.1f);
            Pen bluePen = new Pen(Color.Blue, 0.1f);
            Pen redDashedPen = new Pen(Color.Red, 0.1f);
            redDashedPen.DashPattern = new float[] {4.0f, 4.0f };

//            while (project.Proposed.Landing.Calculating)
//            {
//                System.Threading.Thread.Sleep(100);
//            }

            float _yzoom = p.Height / 50.0f;
            _yPerf = p.Height * 0.90f / _yzoom;

            g.ScaleTransform(_zoomDgn, _yzoom);
            g.TranslateTransform(_xDgn, _yPerf);

            List<double> xs = project.Proposed.Inrun.SlideXs;
            List<double> zs = project.Proposed.Inrun.SlideRAs;

            inrunRAPoints = new PointF[xs.Count];

            for (int i = 0; i < xs.Count; i++)
            {
                inrunRAPoints[i] = new PointF((float)xs[i], -(float)zs[i]);
            }

            zs = project.Proposed.Inrun.SlideKappas;

            inrunKappaPoints = new PointF[xs.Count];

            for (int i = 0; i < xs.Count; i++)
            {
                inrunKappaPoints[i] = new PointF((float)xs[i], -(float)(zs[i] * 1200.0));
            }

            zs = project.Proposed.Inrun.SlidePhis;

            inrunPhiPoints = new PointF[xs.Count];

            for (int i = 0; i < xs.Count; i++)
            {
                inrunPhiPoints[i] = new PointF((float)xs[i], -(float)(zs[i]* 180.0 / Math.PI));
            }

            zs = project.Proposed.Inrun.SlideVs;
            xs.AddRange(project.Proposed.Landing.KFlightXs);
            zs.AddRange(project.Proposed.Landing.KFlightVs);
            xs.AddRange(project.Proposed.Landing.SlideXs);
            zs.AddRange(project.Proposed.Landing.SlideVs);

            speedPoints = new PointF[xs.Count];

            for (int i = 0; i < xs.Count; i++)
            {
                speedPoints[i] = new PointF((float)xs[i], -(float)zs[i]);
            }

            xs = project.Proposed.Landing.KFlightXs;
            zs = project.Proposed.Landing.KFlightZs;

            flightAGLPoints = new PointF[xs.Count];

            for (int i = 0; i < xs.Count; i++)
            {
                flightAGLPoints[i] = new PointF((float)xs[i], -(float)(10.0*(zs[i] - project.Proposed.Landing.HeightAt(xs[i]))));
            }

            xs = project.Proposed.Landing.SlideXs;
            zs = project.Proposed.Landing.SlideRAs;

            landingRAPoints = new PointF[xs.Count];

            for (int i = 0; i < xs.Count; i++)
            {
                landingRAPoints[i] = new PointF((float)xs[i], -(float)zs[i]);
            }

            zs = project.Proposed.Landing.SlideKappas;

            landingKappaPoints = new PointF[xs.Count];

            for (int i = 0; i < xs.Count; i++)
            {
                landingKappaPoints[i] = new PointF((float)xs[i], -(float)(zs[i] * 1200));
            }

            zs = project.Proposed.Landing.SlidePhis;

            landingPhiPoints = new PointF[xs.Count];

            for (int i = 0; i < xs.Count; i++)
            {
                landingPhiPoints[i] = new PointF((float)xs[i], -(float)(zs[i] * 180.0 / Math.PI));
            }

            inrunRA.AddLines(inrunRAPoints);
            landingRA.AddLines(landingRAPoints);
            speed.AddLines(speedPoints);
            inrunRadius.AddLines(inrunKappaPoints);
            landingRadius.AddLines(landingKappaPoints);
            inrunAngle.AddLines(inrunPhiPoints);
            landingAngle.AddLines(landingPhiPoints);
            flightAGL.AddLines(flightAGLPoints);

            g.DrawPath(blackPen, inrunRA);
            g.DrawPath(blackPen, landingRA);
            g.DrawPath(bluePen, flightAGL);
            g.DrawPath(orangePen, speed);
            g.DrawPath(greenPen, inrunRadius);
            g.DrawPath(greenPen, landingRadius);
            g.DrawPath(violetPen, inrunAngle);
            g.DrawPath(violetPen, landingAngle);
            g.DrawLine(darkGrayPen, 
                (float)project.Proposed.Inrun.Ax, 0.0f,
                (float)project.Proposed.Landing.Ux, 0.0f);
            g.DrawLine(redDashedPen,
                (float)project.Proposed.Inrun.E1x, -17f,
                (float)project.Proposed.Inrun.E2x, -17f);
            g.DrawLine(redDashedPen,
                (float)project.Proposed.Landing.Lx, -18f,
                (float)project.Proposed.Landing.Ux, -18f);
            g.DrawLine(darkGrayPen,
                (float)project.Proposed.Inrun.Ax, 0.0f,
                (float)project.Proposed.Inrun.Ax, -40.0f);
            g.DrawLine(darkGrayPen,
                (float)project.Proposed.Inrun.Ax - 1, 0.0f,
                (float)project.Proposed.Inrun.Ax, 0.0f);
            g.DrawLine(darkGrayPen,
                (float)project.Proposed.Inrun.Ax - 1, -10.0f,
                (float)project.Proposed.Inrun.Ax, -10.0f);
            g.DrawLine(darkGrayPen,
                (float)project.Proposed.Inrun.Ax - 1, -20.0f,
                (float)project.Proposed.Inrun.Ax, -20.0f);
            g.DrawLine(darkGrayPen,
                (float)project.Proposed.Inrun.Ax - 1, -30.0f,
                (float)project.Proposed.Inrun.Ax, -30.0f);
            g.DrawLine(darkGrayPen,
                (float)project.Proposed.Inrun.Ax - 1, -40.0f,
                (float)project.Proposed.Inrun.Ax, -40.0f);
        }

        private void panelExisting_Paint(object sender, PaintEventArgs e)
        {
            Panel p = sender as Panel;
            Graphics g = e.Graphics;
            g.ScaleTransform(_zoomExist, _zoomExist);
            g.TranslateTransform(_xExist, _yExist);

            GraphicsPath existingGround = new GraphicsPath();
            Pen greenDashedPen = new Pen(Color.Green, 0.2f);
//            greenDashedPen.DashPattern = new float[] { 15.0f, 15.0f };
            PointF[] existingPoints = new PointF[project.Existing.Points.Count]; 

            for (int i=0; i < project.Existing.Points.Count; i++)
            {
                existingPoints[i] = new PointF((float)project.Existing.Points[i].X, -(float)project.Existing.Points[i].Y);
            }

            if (project.Existing.Points.Count > 1)
            {
                existingGround.AddCurve(existingPoints);
                g.DrawPath(greenDashedPen, existingGround);
            }
        }

        private void panelExisting_MouseDoubleClick(object sender, MouseEventArgs e)
        {
            Panel p = sender as Panel;

            if (e.Button == MouseButtons.Middle)
            {
                // Zoom to extents
                zoomExistingExtents(p);
                panelExisting.Invalidate();
            }
        }

        private void panelDesign_MouseDoubleClick(object sender, MouseEventArgs e)
        {
            Panel p = sender as Panel;

            if (e.Button == MouseButtons.Middle)
            {
                // Zoom to extents
                zoomDesignExtents(p);
                panelDesign.Invalidate();
                panelPerformance.Invalidate();
            }
        }

        private void panelDesign_MouseEnter(object sender, EventArgs e)
        {
            // Grab the mouse focus when the mouse enters the panel area
            panelDesign.Focus();
        }

        private void panelDesign_MouseMove(object sender, MouseEventArgs e)
        {
            if (e.Button == MouseButtons.Middle)
            {
                _xDgn += (e.X - mouseDown.X) / _zoomDgn;
                _yProf += (e.Y - mouseDown.Y) / _zoomDgn;
                mouseDown = e.Location;
                panelDesign.Invalidate();
                panelPerformance.Invalidate();
            }
            else if (e.Button == MouseButtons.None && e.Location != LastPanelMousePosition)
            {
                float Wx = e.X / _zoomDgn - _xDgn;
                float Wy = -(e.Y / _zoomDgn - _yProf);
                designTip.Active = true;

                if (Math.Pow(Wx - project.Proposed.Inrun.Ax, 2.0) +
                    Math.Pow(Wy - project.Proposed.Inrun.Az, 2.0) < 0.5)
                    designTip.SetToolTip(panelDesign,
                        "Point A\nX = " + project.Proposed.Inrun.Ax.ToString("0.000") +
                        "\nZ = " + project.Proposed.Inrun.Az.ToString("0.000"));
                else if (Math.Pow(Wx - project.Proposed.Inrun.Bx, 2.0) +
                    Math.Pow(Wy - project.Proposed.Inrun.Bz, 2.0) < 0.5)
                    designTip.SetToolTip(panelDesign,
                        "Point B\nX = " + project.Proposed.Inrun.Bx.ToString("0.000") +
                        "\nZ = " + project.Proposed.Inrun.Bz.ToString("0.000"));
                else if (Math.Pow(Wx - project.Proposed.Inrun.E2x, 2.0) +
                    Math.Pow(Wy - project.Proposed.Inrun.E2z, 2.0) < 0.5)
                    designTip.SetToolTip(panelDesign,
                        "Point E2\nX = " + project.Proposed.Inrun.E2x.ToString("0.000") +
                        "\nZ = " + project.Proposed.Inrun.E2z.ToString("0.000"));
                else if (Math.Pow(Wx - project.Proposed.Inrun.E1x, 2.0) +
                    Math.Pow(Wy - project.Proposed.Inrun.E1z, 2.0) < 0.5)
                    designTip.SetToolTip(panelDesign,
                        "Point E1\nX = " + project.Proposed.Inrun.E1x.ToString("0.000") +
                        "\nZ = " + project.Proposed.Inrun.E1z.ToString("0.000"));
                else if (Math.Pow(Wx - 0.0, 2.0) +
                    Math.Pow(Wy - 0.0, 2.0) < 0.5)
                    designTip.SetToolTip(panelDesign, "Point T\nX = 0.000\nZ = 0.000");
                else if (Math.Pow(Wx - 0.0, 2.0) +
                    Math.Pow(Wy - -project.Proposed.Landing.TakeoffHeight, 2.0) < 0.5)
                    designTip.SetToolTip(panelDesign,
                        "Point T (landing)\nX = 0.000\nZ = " +
                        (-project.Proposed.Landing.TakeoffHeight).ToString("0.000"));
                else if (Math.Pow(Wx - project.Proposed.Landing.Px, 2.0) +
                    Math.Pow(Wy - project.Proposed.Landing.Pz, 2.0) < 0.5)
                    designTip.SetToolTip(panelDesign,
                        "Point P\nX = " + project.Proposed.Landing.Px.ToString("0.000") +
                        "\nZ = " + project.Proposed.Landing.Pz.ToString("0.000"));
                else if (Math.Pow(Wx - project.Proposed.Landing.Kx, 2.0) +
                    Math.Pow(Wy - project.Proposed.Landing.Kz, 2.0) < 0.5)
                    designTip.SetToolTip(panelDesign,
                        "Point K\nX = " + project.Proposed.Landing.Kx.ToString("0.000") +
                        "\nZ = " + project.Proposed.Landing.Kz.ToString("0.000"));
                else if (Math.Pow(Wx - project.Proposed.Landing.Lx, 2.0) +
                    Math.Pow(Wy - project.Proposed.Landing.Lz, 2.0) < 0.5)
                    designTip.SetToolTip(panelDesign,
                        "Point L\nX = " + project.Proposed.Landing.Lx.ToString("0.000") +
                        "\nZ = " + project.Proposed.Landing.Lz.ToString("0.000"));
                else if (Math.Pow(Wx - project.Proposed.Landing.Ux, 2.0) +
                    Math.Pow(Wy - project.Proposed.Landing.Uz, 2.0) < 0.5)
                    designTip.SetToolTip(panelDesign,
                        "Point U\nX = " + project.Proposed.Landing.Ux.ToString("0.000") +
                        "\nZ = " + project.Proposed.Landing.Uz.ToString("0.000"));
                else
                    designTip.SetToolTip(panelDesign, "X = " + Wx.ToString("0.000") + "\nZ = " + Wy.ToString("0.000"));
            }
            LastPanelMousePosition = e.Location;
        }

        private void panelPerformance_MouseEnter(object sender, EventArgs e)
        {
            panelPerformance.Focus();
        }

        private void panelPerformance_MouseMove(object sender, MouseEventArgs e)
        {
            float Wx = e.X / _zoomDgn - _xDgn;
            int i;
            string speed;
            string RA;
            string kappa;
            string angle;
            string agl;

            if (e.Button == MouseButtons.None && e.Location != LastPerformanceMousePosition)
            {
                if (Wx > speedPoints[0].X && Wx < speedPoints[speedPoints.Length - 1].X)
                {
                    i = 0;
                    while (speedPoints[i].X < Wx)
                        i++;
                    speed = "Speed = " + (-speedPoints[i].Y).ToString("0.00") + " m/s\n";
                }
                else speed = "";


                if (Wx > inrunRAPoints[0].X && Wx < inrunRAPoints[inrunRAPoints.Length - 1].X)
                {
                    i = 0;
                    while (inrunRAPoints[i].X < Wx)
                        i++;
                    RA = "Radial Acceleration = " + (-inrunRAPoints[i].Y).ToString("0.0") + " m/s²\n";
                }
                else if (Wx > landingRAPoints[0].X && Wx < landingRAPoints[landingRAPoints.Length - 1].X)
                {
                    i = 0;
                    while (landingRAPoints[i].X < Wx)
                        i++;
                    RA = "Radial Acceleration = " + (-landingRAPoints[i].Y).ToString("0.0") + " m/s²\n";
                }
                else RA = "";

                if (Wx > flightAGLPoints[0].X && Wx < flightAGLPoints[flightAGLPoints.Length - 1].X)
                {
                    i = 0;
                    while (flightAGLPoints[i].X < Wx)
                        i++;
                    agl = "Flight Height = " + (-flightAGLPoints[i].Y / 10.0).ToString("0.000") + " m\n";
                }
                else agl = "";

                if (Wx > inrunKappaPoints[0].X && Wx < inrunKappaPoints[inrunKappaPoints.Length - 1].X)
                {
                    i = 0;
                    while (inrunKappaPoints[i].X < Wx)
                        i++;
                    kappa = "Curvature = " + (-inrunKappaPoints[i].Y/1200.0).ToString("0.000") + " m¯¹\n";
                }
                else if (Wx > landingKappaPoints[0].X && Wx < landingKappaPoints[landingKappaPoints.Length - 1].X)
                {
                    i = 0;
                    while (landingKappaPoints[i].X < Wx)
                        i++;
                    kappa = "Curvature = " + (-landingKappaPoints[i].Y/1200.0).ToString("0.000") + " m¯¹\n";
                }
                else kappa = "";

                if (Wx > inrunPhiPoints[0].X && Wx < inrunPhiPoints[inrunPhiPoints.Length - 1].X)
                {
                    i = 0;
                    while (inrunPhiPoints[i].X < Wx)
                        i++;
                    angle = "Slope Angle = " + (-inrunPhiPoints[i].Y).ToString("0.0") + "°\n";
                }
                else if (Wx > landingPhiPoints[0].X && Wx < landingPhiPoints[landingPhiPoints.Length - 1].X)
                {
                    i = 0;
                    while (landingPhiPoints[i].X < Wx)
                        i++;
                    angle = "Slope Angle = " + (-landingPhiPoints[i].Y).ToString("0.0") + "°\n";
                }
                else angle = "";

                performanceTip.SetToolTip(panelPerformance, speed + kappa + RA + angle + agl);
            }

            LastPerformanceMousePosition = e.Location;
        }

        private void panelDesign_MouseDown(object sender, MouseEventArgs e)
        {
            if (e.Button == MouseButtons.Middle)
            {
                // Get start point of pan
                mouseDown = e.Location;
            }
        }

        private void panelDesign_MouseWheel(object sender, MouseEventArgs e)
        {
            Panel p = sender as Panel;
            Point ScreenPt = new Point(e.X, e.Y);

            float Wx = e.X / _zoomDgn - _xDgn;
            float Wy = e.Y / _zoomDgn - _yProf;

            _zoomDgn = _zoomDgn * ((float)(2000 + e.Delta)) / 2000.0f;
            _xDgn = e.X / _zoomDgn - Wx;
            _yProf = e.Y / _zoomDgn - Wy;

            panelDesign.Invalidate();
            panelPerformance.Invalidate();
        }

        private void panelDesign_MouseLeave(object sender, EventArgs e)
        {
            designTip.Active = false;
        }

        private void propertiesToolStripMenuItem_Click(object sender, EventArgs e)
        {
            projectInformationDialog.Prefill(project);

            if (projectInformationDialog.ShowDialog(this) == DialogResult.OK)
                projectInformationDialog.Update(project);
        }

        private void openToolStripMenuItem_Click(object sender, EventArgs e)
        {
            if (openFileDialog.ShowDialog() == System.Windows.Forms.DialogResult.OK)
            {
                System.IO.StreamReader sr = new System.IO.StreamReader(openFileDialog.FileName);
                XmlSerializer serializer = new XmlSerializer(typeof(Project));
                project = (Project)serializer.Deserialize(sr);
                sr.Close();
                project.Filename = openFileDialog.FileName;
                dataGridView1.DataSource = project.Existing.Points;
            }
            SetTimer();
        }

        private void exportSCRToolStripMenuItem_Click(object sender, EventArgs e)
        {
            if (exportSCRDialog.ShowDialog() == System.Windows.Forms.DialogResult.OK)
            {
                System.IO.StreamWriter sw = new System.IO.StreamWriter(exportSCRDialog.FileName);

                double Ax = project.Proposed.Inrun.Ax;
                double Ay = project.Proposed.Inrun.Az;
                double Bx = project.Proposed.Inrun.Bx;
                double By = project.Proposed.Inrun.Bz;
                double E1x = project.Proposed.Inrun.E1x;
                double E1y = project.Proposed.Inrun.E1z;
                double E2x = project.Proposed.Inrun.E2x;
                double E2y = project.Proposed.Inrun.E2z;
                double Tx = 0.000;
                double Ty = 0.000;
                double Ty2 = -project.Proposed.Landing.TakeoffHeight;
                double Px = project.Proposed.Landing.Px;
                double Py = project.Proposed.Landing.Pz;
                double Kx = project.Proposed.Landing.Kx;
                double Ky = project.Proposed.Landing.Kz;
                double Lx = project.Proposed.Landing.Lx;
                double Ly = project.Proposed.Landing.Lz;
                double Ux = project.Proposed.Landing.Ux;
                double Uy = project.Proposed.Landing.Uz;

                List<double> xs = project.Proposed.Inrun.Xs;
                List<double> zs = project.Proposed.Inrun.Zs;

                sw.WriteLine("_LINE");
                sw.WriteLine(String.Format("{0:0.000}", Ax) + "," + String.Format("{0:0.000}", Ay));
                sw.WriteLine(String.Format("{0:0.000}", E1x) + "," + String.Format("{0:0.000}", E1y));
                sw.WriteLine("");

                sw.WriteLine("_SPLINE");
                sw.WriteLine(String.Format("{0:0.000}", E1x) + "," + String.Format("{0:0.000}", E1y));
                for (int i = 0; i < xs.Count; i++)
                    if (xs[i] > E1x && xs[i] < E2x)
                        sw.WriteLine(String.Format("{0:0.000}", xs[i]) + "," + String.Format("{0:0.000}", zs[i]));
                sw.WriteLine(String.Format("{0:0.000}", E2x) + "," + String.Format("{0:0.000}", E2y));
                sw.WriteLine("");
                sw.WriteLine("");
                sw.WriteLine("");

                sw.WriteLine("_LINE");
                sw.WriteLine(String.Format("{0:0.000}", E2x) + "," + String.Format("{0:0.000}", E2y));
                sw.WriteLine(String.Format("{0:0.000}", Tx) + "," + String.Format("{0:0.000}", Ty));
                sw.WriteLine("");

                sw.WriteLine("_LINE");
                sw.WriteLine(String.Format("{0:0.000}", Tx) + "," + String.Format("{0:0.000}", Ty));
                sw.WriteLine(String.Format("{0:0.000}", Tx) + "," + String.Format("{0:0.000}", Ty2));
                sw.WriteLine("");

                xs = project.Proposed.Landing.Xs;
                zs = project.Proposed.Landing.Zs;

                sw.WriteLine("_SPLINE");
                for (int i = 0; i < xs.Count; i++)
                    if (xs[i] < Px)
                        sw.WriteLine(String.Format("{0:0.000}", xs[i]) + "," + String.Format("{0:0.000}", zs[i]));
                sw.WriteLine(String.Format("{0:0.000}", Px) + "," + String.Format("{0:0.000}", Py));
                sw.WriteLine("");
                sw.WriteLine("");
                sw.WriteLine("");

                sw.WriteLine("_ARC");
                sw.WriteLine(String.Format("{0:0.000}", Px) + "," + String.Format("{0:0.000}", Py));
                sw.WriteLine(String.Format("{0:0.000}", Kx) + "," + String.Format("{0:0.000}", Ky));
                sw.WriteLine(String.Format("{0:0.000}", Lx) + "," + String.Format("{0:0.000}", Ly));

                sw.WriteLine("_SPLINE");
                sw.WriteLine(String.Format("{0:0.000}", Lx) + "," + String.Format("{0:0.000}", Ly));
                for (int i = 0; i < xs.Count; i++)
                    if (xs[i] > Lx && xs[i] < Ux)
                        sw.WriteLine(String.Format("{0:0.000}", xs[i]) + "," + String.Format("{0:0.000}", zs[i]));
                sw.WriteLine(String.Format("{0:0.000}", Ux) + "," + String.Format("{0:0.000}", Uy));
                sw.WriteLine("");
                sw.WriteLine("");
                sw.WriteLine("");

                sw.WriteLine("_CIRCLE");
                sw.WriteLine(String.Format("{0:0.000}", Ax) + "," + String.Format("{0:0.000}", Ay));
                sw.WriteLine("0.25");
                sw.WriteLine("_CIRCLE");
                sw.WriteLine(String.Format("{0:0.000}", Bx) + "," + String.Format("{0:0.000}", By));
                sw.WriteLine("0.25");
                sw.WriteLine("_CIRCLE");
                sw.WriteLine(String.Format("{0:0.000}", E1x) + "," + String.Format("{0:0.000}", E1y));
                sw.WriteLine("0.25");
                sw.WriteLine("_CIRCLE");
                sw.WriteLine(String.Format("{0:0.000}", E2x) + "," + String.Format("{0:0.000}", E2y));
                sw.WriteLine("0.25");
                sw.WriteLine("_CIRCLE");
                sw.WriteLine(String.Format("{0:0.000}", Tx) + "," + String.Format("{0:0.000}", Ty));
                sw.WriteLine("0.25");
                sw.WriteLine("_CIRCLE");
                sw.WriteLine(String.Format("{0:0.000}", Px) + "," + String.Format("{0:0.000}", Py));
                sw.WriteLine("0.25");
                sw.WriteLine("_CIRCLE");
                sw.WriteLine(String.Format("{0:0.000}", Kx) + "," + String.Format("{0:0.000}", Ky));
                sw.WriteLine("0.25");
                sw.WriteLine("_CIRCLE");
                sw.WriteLine(String.Format("{0:0.000}", Lx) + "," + String.Format("{0:0.000}", Ly));
                sw.WriteLine("0.25");
                sw.WriteLine("_CIRCLE");
                sw.WriteLine(String.Format("{0:0.000}", Ux) + "," + String.Format("{0:0.000}", Uy));
                sw.WriteLine("0.25");

                xs = project.Proposed.Landing.KFlightXs;
                zs = project.Proposed.Landing.KFlightZs;

                sw.WriteLine("_SPLINE");
                for (int i = 0; i < xs.Count; i++)
                    sw.WriteLine(String.Format("{0:0.000}", xs[i]) + "," + String.Format("{0:0.000}", zs[i]));
                sw.WriteLine("");
                sw.WriteLine("");
                sw.WriteLine("");

                sw.Close();
            }
        }

        private void newToolStripMenuItem_Click(object sender, EventArgs e)
        {
            if (MessageBox.Show("Discard the current design and create a new design file?", "Confirm Discard Changes", MessageBoxButtons.YesNo, MessageBoxIcon.Warning) == DialogResult.Yes)
                project = new Project();
            SetTimer();
        }

        private void saveToolStripMenuItem_Click(object sender, EventArgs e)
        {
            if (project.Filename == null)
            {
                if (saveFileDialog.ShowDialog() == System.Windows.Forms.DialogResult.OK)
                {
                    System.IO.StreamWriter sw = new System.IO.StreamWriter(saveFileDialog.FileName);
                    XmlSerializer serializer = new XmlSerializer(typeof(Project));
                    serializer.Serialize(sw, project);
                    sw.Close();
                    project.Filename = saveFileDialog.FileName;
                }
            }
            else
            {
                System.IO.StreamWriter sw = new System.IO.StreamWriter(project.Filename);
                XmlSerializer serializer = new XmlSerializer(typeof(Project));
                serializer.Serialize(sw, project);
                sw.Close();
            }
        }

        private void saveAsToolStripMenuItem_Click(object sender, EventArgs e)
        {
            if (saveFileDialog.ShowDialog() == System.Windows.Forms.DialogResult.OK)
            {
                System.IO.StreamWriter sw = new System.IO.StreamWriter(saveFileDialog.FileName);
                XmlSerializer serializer = new XmlSerializer(typeof(Project));
                serializer.Serialize(sw, project);
                sw.Close();
                project.Filename = saveFileDialog.FileName;
            }
        }

        private void numericUpDownLandingRadius_MouseDoubleClick(object sender, MouseEventArgs e)
        {
//            numericUpDownLandingRadius.Enabled = !numericUpDownLandingRadius.Enabled;
            numericUpDownLandingRadius.ReadOnly = !numericUpDownLandingRadius.ReadOnly;

            if (!numericUpDownLandingRadius.ReadOnly)
            {
                project.Proposed.Landing.LandingRadius = (double)numericUpDownLandingRadius.Value;
                toolTip.SetToolTip(numericUpDownLandingRadius, "Double-click to use default");
            }
            else
            {
                project.Proposed.Landing.LandingRadius = 0.00;
                toolTip.SetToolTip(numericUpDownLandingRadius, "Double-click to override default");
            }

            SetTimer();
        }

        private void dataGridView1_CellEndEdit(object sender, DataGridViewCellEventArgs e)
        {
            panelExisting.Invalidate();
        }

        private void zoomDesignExtents(Panel p)
        {
            float width = (float)(project.Proposed.Landing.Ux - project.Proposed.Inrun.Ax);
            float height = (float)(project.Proposed.Landing.Uz - project.Proposed.Inrun.Az);
            float Cx = (float)(project.Proposed.Landing.Ux + project.Proposed.Inrun.Ax) / 2.0f;
            float Cy = (float)(project.Proposed.Landing.Uz + project.Proposed.Inrun.Az) / 2.0f;
            _zoomDgn = 0.9f * Math.Min(p.Width / width, p.Height / -height);
            _xDgn = (p.Width / 2.0f / _zoomDgn) - Cx;
            _yProf = (p.Height / 2.0f / _zoomDgn) + Cy;
        }

        private void zoomExistingExtents(Panel p)
        {
            float minX, maxX, minY, maxY;

            minX = maxX = (float)project.Existing.Points[0].X;
            minY = maxY = (float)project.Existing.Points[0].Y;

            for (int i = 0; i < project.Existing.Points.Count; i++)
            {
                minX = Math.Min((float)project.Existing.Points[i].X, minX);
                maxX = Math.Max((float)project.Existing.Points[i].X, maxX);
                minY = Math.Min((float)project.Existing.Points[i].Y, minY);
                maxY = Math.Max((float)project.Existing.Points[i].Y, maxY);
            }

            float width = maxX - minX;
            float height = maxY - minY;
            float Cx = (maxX + minX) / 2.0f;
            float Cy = (float)(maxY + minY) / 2.0f;
            _zoomExist = 0.9f * Math.Min(p.Width / width, p.Height / height);
            _xExist = (p.Width / 2.0f / _zoomExist) - Cx;
            _yExist = (p.Height / 2.0f / _zoomExist) + Cy;
        }
    }
}
