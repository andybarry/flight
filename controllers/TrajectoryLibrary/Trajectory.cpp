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
    filename_prefix_ = "";
    dt_ = 0;
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

    filename_prefix_ = filename_prefix;

    dimension_ = xpoints_.cols() - 1; // minus 1 because of time index
    udimension_ = upoints_.cols() - 1;

    if (kpoints_.cols() - 1 != dimension_ * udimension_) {
        cerr << "Error: expected to have " << dimension_ << "*" << udimension_ << "+1 = " << dimension_ * udimension_ + 1 << " columns in " << filename_prefix << "-controller.csv but found " << kpoints_.cols() << endl;
        exit(1);
    }

    if (affine_points_.cols() - 1 != udimension_) {
        cerr << "Error: expected to have " << udimension_ << "+1 = " << udimension_ + 1 << " columns in " << filename_prefix << "-affine.csv but found " << affine_points_.cols() << endl;
        exit(1);
    }

    if (xpoints_.rows() != upoints_.rows() || xpoints_.rows() != kpoints_.rows() || xpoints_.rows() != affine_points_.rows()) {
        cerr << "Error: inconsistent number of rows in CSV files: " << endl
            << "\t" << filename_prefix << "-x: " << xpoints_.rows() << endl
            << "\t" << filename_prefix << "-u: " << upoints_.rows() << endl
            << "\t" << filename_prefix << "-controller: " << kpoints_.rows() << endl
            << "\t" << filename_prefix << "-affine: " << affine_points_.rows() << endl;

            exit(1);
    }

    //cout << "x at t = 0.45:" << endl << GetState(0.323) << endl << " u = " << endl << GetUCommand(4.01) << endl;

}


void Trajectory::LoadMatrixFromCSV( const std::string& filename, Eigen::MatrixXd &matrix) {

    cout << "Loading " << filename << endl;

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

        if (row_num == 1) {
            dt_ = matrix(1, 0) - matrix(0, 0);
        } else if (row_num > 1) {
            if (matrix(row_num, 0) - matrix(row_num - 1, 0) - dt_ > std::numeric_limits<double>::epsilon()) {
                cerr << "Error: non-constant dt. Expected dt = " << dt_ << " but got matrix[" << row_num << "][0] - matrix[" << row_num - 1 << "][0] = " << matrix(row_num, 0) - matrix(row_num - 1, 0) << endl;

                cout << matrix << endl;
                exit(1);
            }
        }

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

Eigen::VectorXd Trajectory::GetState(double t) {
    int index = GetIndexFromTime(t);

    Eigen::VectorXd row_vec = xpoints_.row(index);

    return row_vec.tail(xpoints_.cols() - 1); // remove time
}

Eigen::VectorXd Trajectory::GetUCommand(double t) {
    int index = GetIndexFromTime(t);

    Eigen::VectorXd row_vec = upoints_.row(index);

    return row_vec.tail(upoints_.cols() - 1); // remove time
}


/**
 * Assuming a constant dt, we can compute the index of a point
 * based on its time.
 *
 * @param t time to find index of
 * @retval index of nearest point
 */
int Trajectory::GetIndexFromTime(double t) {

    // round t to the nearest dt_

   double t0 = xpoints_(0,0);

   double tf = xpoints_(xpoints_.rows() - 1, 0);

   if (t < t0) {
       return 0;
   } else if (t > tf) {
       return xpoints_.rows() - 1;
   }

   // otherwise, we are somewhere in the bounds of the trajectory
    int num_dts = t/dt_;
    float remainder = fmod(t, dt_);

    if (remainder > 0.5f*dt_) {
        num_dts++;
    }

    int starting_dts = t0 / dt_;

    return num_dts + starting_dts;

}




void Trajectory::Print() {
    cout << "------------ Trajectory print -------------" << endl;
    cout << "Filename: " << filename_prefix_ << endl;
    cout << "Dimension: " << dimension_ << endl;
    cout << "u-dimension: " << udimension_ << endl;

    cout << " t\t x\t y\t z\t roll\t pitch\t yaw \t xdot\t ydot\t zdot\t rolld\t pitchd\t yawd" << endl;

    cout << xpoints_ << endl;

    cout << "------------- u points ----------------" << endl;

    cout << " t\t u1\t u2\t u3" << endl;

    cout << upoints_ << endl;

    cout << "------------- k points ----------------" << endl;

    cout << kpoints_ << endl;

    cout << "------------- affine points ----------------" << endl;

    cout << affine_points_ << endl;
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
