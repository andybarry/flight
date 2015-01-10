/*
 * Trajectory class representing a trajectory in state space
 *
 * Author: Andrew Barry, <abarry@csail.mit.edu> 2013
 *
 */

#include "Trajectory.hpp"



// Constructor that loads a trajectory from a file
Trajectory::Trajectory() {
    trajectory_number_ = -1;
    dimension_ = 0;
    udimension_ = 0;
    filename_ = "";
}

Trajectory::Trajectory(string filename_prefix, bool quiet) : Trajectory() {
    LoadTrajectory(filename_prefix, quiet);
}

void Trajectory::LoadTrajectory(string filename_prefix, bool quiet)
{
    // open the file
    vector<vector<string>> strs;


    if (!quiet)
    {
        cout << "Loading trajectory: " << endl << "\t" << filename_prefix << endl;
    }

    //int trajlibLoc = filename.rfind("trajlib");
    //string trajNumberStr = filename.substr(trajlibLoc+7, filename.length()-trajlibLoc-4-7);


    //trajNumber = stoi(trajNumberStr);

    LoadMatrixFromCSV(filename_prefix + "-x.csv", xpoints_);
    LoadMatrixFromCSV(filename_prefix + "-u.csv", upoints_);
    LoadMatrixFromCSV(filename_prefix + "-controller.csv", kpoints_);
    LoadMatrixFromCSV(filename_prefix + "-affine.csv", affine_points_);

    filename_ = filename_prefix;

    cout << "Loaded." << endl;

    //Print();
}


void Trajectory::LoadMatrixFromCSV( const std::string& filename, Eigen::MatrixXd &matrix) {
cout << "Loading" << filename << endl;
    int number_of_lines = GetNumberOfLines(filename);
    int row_num = 0;

    int i =  0;
    //                                   file, delimiter, first_line_is_header?
    CsvParser *csvparser = CsvParser_new(filename.c_str(), ",", true);
    CsvRow *header;
    CsvRow *row;

    header = CsvParser_getHeader(csvparser);
    if (header == NULL) {
        printf("%s\n", CsvParser_getErrorMessage(csvparser));
        return;
    }
    char **headerFields = CsvParser_getFields(header);
    for (i = 0 ; i < CsvParser_getNumFields(header) ; i++) {
        //printf("TITLE: %s\n", headerFields[i]);
    }

    matrix.resize(number_of_lines - 1, i); // minus 1 for header, i = number of columns

    while ((row = CsvParser_getRow(csvparser)) ) {
        //printf("NEW LINE:\n");
        char **rowFields = CsvParser_getFields(row);
        for (i = 0 ; i < CsvParser_getNumFields(row) ; i++) {

            matrix(row_num, i) = atof(rowFields[i]);

            //printf("FIELD: %20.20f\n", atof(rowFields[i]));
        }
        CsvParser_destroy_row(row);

        row_num ++;
    }
    CsvParser_destroy(csvparser);


}

int Trajectory::GetNumberOfLines(string filename) {
    int number_of_lines = 0;
    string line;
    ifstream myfile(filename);

    while (getline(myfile, line)) {
        ++number_of_lines;
    }

    return number_of_lines;
}

void Trajectory::Print() {
    cout << "------------ Trajectory print -------------" << endl;
    cout << "Filename: " << filename_ << endl;
    cout << "Dimension: " << dimension_ << endl;
    cout << "u-dimension: " << udimension_ << endl;

    cout << " t\t x\t y\t z\t roll\t pitch\t yaw \t xdot\t ydot\t zdot\t rolld\t pitchd\t yawd" << endl;

    cout << xpoints_ << endl;

    cout << "------------- u points ----------------" << endl;

    cout << " t\t u1\t u2\t u3" << endl;

    cout << upoints_ << endl;
}

void Trajectory::GetTransformedPoint(int index, BotTrans *transform, double *xyz)
{
    // apply the transformation from the global frame: orgin = (0,0,0)
    // to the local frame point
/*
    double originalPoint[3];
    originalPoint[0] = xpoints_[index][1];
    originalPoint[1] = xpoints_[index][2];
    originalPoint[2] = xpoints_[index][3];


    bot_trans_apply_vec(transform, originalPoint, xyz);
    */
}

void Trajectory::PlotTransformedTrajectory(bot_lcmgl_t *lcmgl, BotTrans *transform)
{
    bot_lcmgl_line_width(lcmgl, 2.0f);
    bot_lcmgl_begin(lcmgl, GL_LINE_STRIP);
    for (int i=0; i<int(xpoints_.size()); i++)
    {
        double xyz[3];
        GetTransformedPoint(i, transform, xyz);

        bot_lcmgl_vertex3f(lcmgl, xyz[0], xyz[1], xyz[2]);
    }
    bot_lcmgl_end(lcmgl);
}

#if 0
double Trajectory::DistanceToPoint(double x, double y, double z)
{
    double minDist = -1;

    for (int i=0; i<int(xpoints.size()); i++)
    {
        // find the distance to this point
        double thisDist = sqrt( pow(x-xpoints[i][0],2) + pow(y-xpoints[i][1],2) + pow(z-xpoints[i][2],2) );

        if (minDist < 0 || thisDist < minDist)
        {
            minDist = thisDist;
        }
    }

    return minDist;
}
#endif
