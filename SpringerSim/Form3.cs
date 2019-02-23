using System;
using System.Windows.Forms;

namespace SpringerSim
{
    public partial class ProjectInformationDialog : Form
    {
        public ProjectInformationDialog()
        {
            InitializeComponent();
        }

        internal void Prefill(Project project)
        {
            textBoxProjectName.Text = project.ProjectName;
            textBoxSiteName.Text = project.SiteName;
            textBoxHillName.Text = project.HillName;
            textBoxDescription.Text = project.Description;
            textBoxLatitude.Text = project.Latitude;
            textBoxLogitude.Text = project.Longitude;
            textBoxElevation.Text = project.Elevation;
            textBoxDesigner.Text = project.Designer;
            textBoxChecker.Text = project.Checker;
            textBoxComments.Text = project.Comments;
        }

        internal void Update(Project project)
        {
            project.ProjectName = textBoxProjectName.Text;
            project.SiteName = textBoxSiteName.Text;
            project.HillName = textBoxHillName.Text;
            project.Description = textBoxDescription.Text;
            project.Latitude = textBoxLatitude.Text;
            project.Longitude = textBoxLogitude.Text;
            project.Elevation = textBoxElevation.Text;
            project.Designer = textBoxDesigner.Text;
            project.Checker = textBoxChecker.Text;
            project.Comments = textBoxComments.Text;
        }
    }
}
