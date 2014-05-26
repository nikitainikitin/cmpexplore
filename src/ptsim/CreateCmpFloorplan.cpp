#include <iostream>
#include <iomanip>
#include <fstream>
#include <string.h>
#include <stdlib.h>

extern "C" {
#include "flp.h"
}

#define MAXSTRING 128

using namespace std;

flp_t *read_flp(char *, int);

// tile
//string tile_floorplan_filename("single_tile.flp");
string tile_floorplan_filename("niagara2_tile.flp");
// cmp16x16
//string cmp_floorplan_filename("cmp16x16.flp");
char cmp_floorplan_filename[MAXSTRING];
//
//string cmp_ptrace_filename("cmp16x16.ptrace");
char cmp_ptrace_filename[MAXSTRING];

int main(int argc, char **argv)
{
  int i,j, k;

  ofstream fout,pout;
  int tile_blocks;
  int cmp_blocks;

  char flp[MAXSTRING];
  char cfg[MAXSTRING];
  char initt[MAXSTRING];
  char stdyt[MAXSTRING];

  flp_t *tile_flp;

  double width, height, leftx, bottomy;
  double tile_width, tile_height;

  double cmp_width, cmp_height;
  int cmp_x_tiles = 16;
  int cmp_y_tiles = 16;
 
  int mc_num = 2;
  double mc_area = 0.00001561115;
  double mc_width = 0.001;
  double mc_height = 0.001;

  double x_offset = 0.0;
  double y_offset = 0.0;


  // reading x and y tiles number
  if (argc != 3){
    cout << "Usage is as follows:" << endl;
    cout << "CreateCmpFloorplan Nx Ny" << endl;
    return -1;
  }
  else {
    cmp_x_tiles = atoi(argv[1]);
    cmp_y_tiles = atoi(argv[2]);
    cout << endl << "Creating a " << cmp_x_tiles << " x " << cmp_y_tiles << " floorplan" << endl; 
  }
  
  // adapt C++ strings to C strings
  strcpy(flp,tile_floorplan_filename.c_str());

  // read the tile floorplan
  tile_flp = read_flp(flp,0);

  tile_blocks = tile_flp->n_units;
  cout << "DEBUG: tile blocks = " << tile_blocks << endl;

  tile_width = 0.0;
  tile_height = 0.0;
  for(i=0;i<tile_blocks;i++){
    width = (tile_flp->units+i)->width;
    height = (tile_flp->units+i)->height;
    leftx = (tile_flp->units+i)->leftx;
    bottomy = (tile_flp->units+i)->bottomy;
    if (width+leftx > tile_width)
      tile_width = width+leftx;
    if (height+bottomy > tile_height)
      tile_height = height+bottomy;
    cout << "\t" << width << "\t" << height << "\t" << leftx << "\t" << bottomy << endl;
  }
  cout << endl << "tile width = " << tile_width << ", tile_height = " << tile_height << endl;

  cmp_width = cmp_x_tiles * tile_width;
  cmp_height = cmp_y_tiles * tile_height;

  // output file opening
  sprintf(cmp_floorplan_filename,"cmp%dx%d.flp",cmp_x_tiles,cmp_y_tiles);
  sprintf(cmp_ptrace_filename,"cmp%dx%d.ptrace",cmp_x_tiles,cmp_y_tiles);
  fout.open(cmp_floorplan_filename);
  pout.open(cmp_ptrace_filename);

  fout << endl;
  fout << "# Line Format: <unit-name>\\t<width>\\t<height>\\t<left-x>\\t<bottom-y>" << endl;
  fout << "# all dimensions are in meters" << endl;
  fout << "# comment lines begin with a '#'" << endl;
  fout << "# comments and empty lines are ignored" << endl;
  fout << endl;

  mc_width = mc_area / cmp_height;
  if (mc_num >0) {
    x_offset = mc_width;
    pout << "MCW\t";
    fout << "MCW\t" << setiosflags(ios::fixed) << setprecision(6) << mc_width;
    fout << "\t" << setiosflags(ios::fixed) << setprecision(6) << cmp_height;
    fout << "\t" << setiosflags(ios::fixed) << setprecision(6) << 0.0;
    if (mc_num >= 3)
      fout << "\t" << setiosflags(ios::fixed) << setprecision(6) << mc_height << endl;
    else
      fout << "\t" << setiosflags(ios::fixed) << setprecision(6) << 0.0 << endl;
    if (mc_num >1) {
      pout << "MCE\t";
      fout << "MCE\t" << setiosflags(ios::fixed) << setprecision(6) << mc_width;
      fout << "\t" << setiosflags(ios::fixed) << setprecision(6) << cmp_height;
      fout << "\t" << setiosflags(ios::fixed) << setprecision(6) << mc_width+cmp_width;
      if (mc_num >=3)
	fout << "\t" << setiosflags(ios::fixed) << setprecision(6) << mc_height << endl;
      else
	fout << "\t" << setiosflags(ios::fixed) << setprecision(6) << 0.0 << endl;
      if (mc_num > 2) {
	y_offset = mc_height;
	pout << "MCS\t";
	fout << "MCS\t" << setiosflags(ios::fixed) << setprecision(6) << cmp_width;
	fout << "\t" << setiosflags(ios::fixed) << setprecision(6) << mc_height;
	fout << "\t" << setiosflags(ios::fixed) << setprecision(6) << mc_width;
	fout << "\t" << setiosflags(ios::fixed) << setprecision(6) << 0.0 << endl;
	if (mc_num == 4) {
	  pout << "MCN\t";
	  fout << "MCN\t" << setiosflags(ios::fixed) << setprecision(6) << cmp_width;
	  fout << "\t" << setiosflags(ios::fixed) << setprecision(6) << mc_height;
	  fout << "\t" << setiosflags(ios::fixed) << setprecision(6) << mc_width;
	  fout << "\t" << setiosflags(ios::fixed) << setprecision(6) << mc_height+cmp_height << endl;
	}
      }
    }
  }

  for(j=0; j<cmp_y_tiles; j++) {
    for(i=0; i<cmp_x_tiles; i++) {
      for(k=0; k<tile_blocks; k++) {
	fout << (tile_flp->units+k)->name << j*cmp_x_tiles+i;
	pout << (tile_flp->units+k)->name << j*cmp_x_tiles+i << "\t";
	fout << "\t" << setiosflags(ios::fixed) << setprecision(6) << (tile_flp->units+k)->width;
	fout << "\t" << setiosflags(ios::fixed) << setprecision(6) << (tile_flp->units+k)->height;
	fout << "\t" << setiosflags(ios::fixed) << setprecision(6) << (tile_flp->units+k)->leftx+i*tile_width+x_offset;
	fout << "\t" << setiosflags(ios::fixed) << setprecision(6) << (tile_flp->units+k)->bottomy+j*tile_height+y_offset << endl;
      }
    }
  }

  pout << endl;
  for (i=0;i<cmp_x_tiles*cmp_y_tiles*tile_blocks+mc_num;i++)
    pout << "0.0\t";
  pout << endl;

  // closing
  fout.close();
  pout.close();

  //delete [] power_sim;
  //delete [] temp_sim;
  return 0;
}
