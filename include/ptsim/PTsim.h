#ifndef __PTSIM_H_
#define __PTSIM_H_

#include "util.h"

/* global configuration parameters for HotSpot	*/
typedef struct global_config_t_st
{
	/* floorplan input file */
	char flp_file[STR_SIZE];
	/* input power trace file */
	char p_infile[STR_SIZE];
	/* output file for the temperature trace */
	char t_outfile[STR_SIZE];
	/* input configuration parameters from file	*/
	char config[STR_SIZE];
	/* output configuration parameters to file	*/
	char dump_config[STR_SIZE];
}global_config_t;

int sim_init(char *flp_file, char *config_file, char *init_file, char *steady_file, char *gridsteady_file);
void sim_main(double*, double*, double);
void sim_exit();

#endif
