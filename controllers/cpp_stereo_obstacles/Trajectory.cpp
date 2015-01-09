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
    filename_ = "test";
}

Trajectory::Trajectory(string filename, bool quiet) : Trajectory() {
    LoadTrajectory(filename, quiet);
}

void Trajectory::LoadTrajectory(string filename, bool quiet)
{
    // open the file
    vector<vector<string>> strs;

    string utrajFile = filename.substr(0, filename.length()-4) + "-u.csv";

    if (!quiet)
    {
        cout << "Loading trajectory: " << endl << "\t" << filename << endl <<"\t" << utrajFile << endl;
    }

    //int trajlibLoc = filename.rfind("trajlib");
    //string trajNumberStr = filename.substr(trajlibLoc+7, filename.length()-trajlibLoc-4-7);


    //trajNumber = stoi(trajNumberStr);

    LoadXFromCSV(filename);
    LoadUFromCSV(utrajFile);

    filename_ = filename;

    //Print();
}

void Trajectory::LoadXFromCSV( const std::string& filename) {
    io::CSVReader<13> in(filename);

    in.read_header(io::ignore_extra_column, "t", "x", "y", "z", "roll", "pitch", "yaw", "xdot", "ydot", "zdot", "rolldot", "pitchdot", "yawdot");

    double t, x, y, z, roll, pitch, yaw, xdot, ydot, zdot, rolldot, pitchdot, yawdot;

    int row = 0;

    while (in.read_row(t, x, y, z, roll, pitch, yaw, xdot, ydot, zdot, rolldot, pitchdot, yawdot)) {
        // put the data into a matrix

        xpoints_(row, 0) = t;
        xpoints_(row, 1) = x;
        xpoints_(row, 2) = y;
        xpoints_(row, 3) = z;
        xpoints_(row, 4) = roll;
        xpoints_(row, 5) = pitch;
        xpoints_(row, 6) = yaw;
        xpoints_(row, 7) = xdot;
        xpoints_(row, 8) = ydot;
        xpoints_(row, 9) = zdot;
        xpoints_(row, 10) = rolldot;
        xpoints_(row, 11) = pitchdot;
        xpoints_(row, 12) = yawdot;


        row ++;

    }

    cout << xpoints_;

}


// from:  answered Feb 19 at 0:37
// Jim M.
// http://stackoverflow.com/a/14947876/730138
void Trajectory::LoadUFromCSV( const std::string& filename)
{
    /*
    std::ifstream       file( filename.c_str() );
    std::vector< std::vector<std::string> >   matrix;
    std::vector<std::string>   row;
    std::string                line;
    std::string                cell;

    while( file )
    {
        std::getline(file,line);
        std::stringstream lineStream(line);
        row.clear();

        while( std::getline( lineStream, cell, ',' ) )
            row.push_back( cell );

        if( !row.empty() )
            matrix.push_back( row );
    }

    if (matrix.size() > 0)
    {
        udimension_ = int(matrix[0].size()) - 1; // minus one because the first column is the time index
    } else {
        cout << "WARNING: loaded trajectory of size 0 (did you include the timestamp in the first column): " << filename << endl;
    }

    for( int i=0; i<int(matrix.size()); i++ )
    {
        vector<double> thisRow;
        for( int j=0; j<int(matrix[i].size()); j++ )
        {
            thisRow.push_back(atof(matrix[i][j].c_str()));
        }
        upoints_.push_back(thisRow);

    }
    */

}

void Trajectory::Print() {
    cout << "------------ Trajectory print -------------" << endl;
    cout << "Filename: " << filename_ << endl;
    cout << "Dimension: " << dimension_ << endl;
    cout << "u-dimension: " << udimension_ << endl;

    cout << " t\t x\t y\t z\t roll\t pitch\t yaw \t xdot\t ydot\t zdot\t rolld\t pitchd\t yawd" << endl;
/*
    for (int i=0; i<int(xpoints_.size()); i++)
    {
        for (int j=0; j<int(xpoints_[i].size()); j++)
        {
            //cout << xpoints_[i][j] << "\t";
            printf("% 4.3f\t", xpoints_[i][j]);
        }
        cout << endl;
    }

    cout << "------------- u points ----------------" << endl;

    cout << " t\t u1\t u2\t u3" << endl;

    for (int i=0; i<int(upoints_.size()); i++)
    {
        for (int j=0; j<int(upoints_[i].size()); j++)
        {
            //cout << upoints_[i][j] << " ";
            printf("% 4.3f\t", upoints_[i][j]);
        }
        cout << endl;
    }
    */
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
