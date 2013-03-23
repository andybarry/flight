/*
 * Trajectory class representing a trajectory in state space
 *
 * Author: Andrew Barry, <abarry@csail.mit.edu> 2013
 *
 */
 
#include "Trajectory.hpp"



// Constructor that loads a trajectory from a file
Trajectory::Trajectory()
{

}

Trajectory::Trajectory(string filename)
{
    LoadTrajectory(filename);
}

void Trajectory::LoadTrajectory(string filename)
{
    // open the file
    vector<vector<string>> strs;
    
    string utrajFile = filename.substr(0, filename.length()-4) + "-u.csv";
    
    cout << "Loading trajectory: " << filename << " + " << utrajFile << endl;
    LoadXFromCSV(filename);
    LoadUFromCSV(utrajFile);
    
    //print();
}

// from:  answered Feb 19 at 0:37
// Jim M.
// http://stackoverflow.com/questions/1120140/csv-parser-in-c
void Trajectory::LoadXFromCSV( const std::string& filename)
{
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
        dimension = int(matrix[0].size());
    } else {
        cout << "WARNING: loaded trajectory of size 0." << endl;
    }
    
    for( int i=0; i<int(matrix.size()); i++ )
    {
        vector<float> thisRow;
        for( int j=0; j<int(matrix[i].size()); j++ )
        {
            thisRow.push_back(atof(matrix[i][j].c_str()));
        }
        xpoints.push_back(thisRow);
        
    }
    
}


// from:  answered Feb 19 at 0:37
// Jim M.
// http://stackoverflow.com/questions/1120140/csv-parser-in-c
void Trajectory::LoadUFromCSV( const std::string& filename)
{
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
        udimension = int(matrix[0].size());
    } else {
        cout << "WARNING: loaded trajectory of size 0: " << filename << endl;
    }
    
    for( int i=0; i<int(matrix.size()); i++ )
    {
        vector<float> thisRow;
        for( int j=0; j<int(matrix[i].size()); j++ )
        {
            thisRow.push_back(atof(matrix[i][j].c_str()));
        }
        upoints.push_back(thisRow);
        
    }
    
}

void Trajectory::print()
{
    cout << "------------ Trajectory print -------------" << endl;
    cout << "Dimension: " << dimension << endl;
    cout << "u-dimension: " << udimension << endl;
    
    for (int i=0; i<int(xpoints.size()); i++)
    {
        for (int j=0; j<int(xpoints[i].size()); j++)
        {
            cout << xpoints[i][j] << " ";
        }
        cout << endl;
    }
    
    cout << "------------- u points ----------------" << endl;
    for (int i=0; i<int(upoints.size()); i++)
    {
        for (int j=0; j<int(upoints[i].size()); j++)
        {
            cout << upoints[i][j] << " ";
        }
        cout << endl;
    }
}

void Trajectory::GetTransformedPoint(int index, BotTrans *transform, double *xyz)
{
    // apply the transformation from the global frame: orgin = (0,0,0)
    // to the local frame point
    
    double originalPoint[3];
    originalPoint[0] = xpoints[index][0];
    originalPoint[1] = xpoints[index][1];
    originalPoint[2] = xpoints[index][2];
    
    
    bot_trans_apply_vec(transform, originalPoint, xyz);
}

void Trajectory::PlotTransformedTrajectory(bot_lcmgl_t *lcmgl, BotTrans *transform)
{
    bot_lcmgl_line_width(lcmgl, 2.0f);
    bot_lcmgl_begin(lcmgl, GL_LINE_STRIP);
    for (int i=0; i<int(xpoints.size()); i++)
    {
        double xyz[3];
        GetTransformedPoint(i, transform, xyz);
        
        bot_lcmgl_vertex3f(lcmgl, xyz[0], xyz[1], xyz[2]);
    }
    bot_lcmgl_end(lcmgl);
}

#if 0
float Trajectory::DistanceToPoint(float x, float y, float z)
{
    float minDist = -1;
    
    for (int i=0; i<int(xpoints.size()); i++)
    {
        // find the distance to this point
        float thisDist = sqrt( pow(x-xpoints[i][0],2) + pow(y-xpoints[i][1],2) + pow(z-xpoints[i][2],2) );
        
        if (minDist < 0 || thisDist < minDist)
        {
            minDist = thisDist;
        }
    }
    
    return minDist;
}
#endif
