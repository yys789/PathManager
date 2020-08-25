/*
 * testcubicspline.cc
 *
 *  Created on: May 19, 2020
 *      Author: zhian
 */

#include <fitHead/smoothspline3d.h>
#include <iostream>
#include <fstream>
#include <random>
#include "fitHead/bench.h"
using namespace std;
using namespace walgo;

bool readData(const char* fname, vector<point3dd>& ps, vector<point3dd>& pa)
{
	float x,y,z,a,b,c;
  	ifstream ifs(fname);
  	if ( !ifs )
  	{
  		cout << "can't read file " << fname << endl;
  		return false;
  	}
  	ps.clear();
  	pa.clear();
  	int i = 0;
  	int j = 0;
  	while ( ifs >> x && ifs >> y && ifs >> z && ifs >> a && ifs >> b && ifs >> c ) //&& ifs >> i && ifs >> j)
    {
  		ps.push_back(point3dd(x,y,z));
  		//cout << x << "  " << y << "  " << z << endl;
  		pa.push_back(point3dd(a,b,c));
    }
  	return true;
}

int main()
{
    vector<point3dd> points;
    vector<point3dd> angles;
    readData("data.txt", points, angles);
    cout << "read " <<  points.size() << " entries " << endl;
    smoothspline3d cs3d(points, 0);
    cs3d.fit();
    ofstream ofile("smoothpts.txt");
    for ( int i = 0; i < points.size()-1; i++)
    {
        point3dd p = cs3d.model(i);
        ofile << p._x << "  " << p._y << "  " << p._z << "  " << endl;
        p = cs3d.model(i+0.5);
        ofile << p._x << "  " << p._y << "  " << p._z << "  " << endl;
    }
    cout << "output written to smoothpts.txt" << endl;
}
