/*
 * Trajectory class representing a trajectory in state space
 *
 * Author: Andrew Barry, <abarry@csail.mit.edu> 2013
 *
 */
 
#include "Trajectory.hpp"



// Constructor that loads a trajectory from a file
Trajectory::Trajectory(string filename)
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
        vector<double> thisRow;
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
        vector<double> thisRow;
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
