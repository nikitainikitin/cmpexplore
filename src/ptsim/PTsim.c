/* 
 * A dummy simulator template file to illustrate the use of
 * HotSpot in a cycle-accurate simulator like Simplescalar. 
 * This file contains the following sample routines:
 * 	a) Model initialization	(sim_init)
 *	b) Model use in a cycle-by-cycle power model (sim_main)
 *	c) Model uninitialization (sim_exit)
 *  Please note that all of these routines are just instructional
 *  templates and not full-fledged code. Hence they are not used
 *  anywhere else in the distribution.
 */
#include <stdlib.h>
#include <string.h>
#include <assert.h>

#include "package.h"
#include "temperature.h"
#include "temperature_grid.h"	/* for dump_steady_temp_grid	*/
#include "flp.h"
#include "util.h"
#include "PTsim.h"

/* configuration file */
//static char *config_file="cmp.config";

/* input and output files	*/
//static char *flp_file="tile.flp";		/* has the floorplan configuration	*/
//static char *init_file="tile.init";		/* initial temperatures	from file	*/
//static char *steady_file="tile.steady";	/* steady state temperatures to file	*/
//static char *ttrace_file="tile.ttrace"; /* transient temperatures to file */

/* floorplan	*/
static flp_t *flp;
/* hotspot temperature model	*/
static RC_model_t *model;
/* instantaneous temperature and power values	*/
static double *temp, *power;
/* steady state temperature and power values	*/
static double *overall_power, *steady_temp;

/* variables for natural convection iterations */
static int natural = 0; 
static double avg_sink_temp = 0;
static int natural_convergence = 0;
static double r_convec_old;

/* table to hold options and configuration */
static str_pair table[MAX_ENTRIES];

/* total epochs */
static int epochs;

/* current time and previous time */
static double cur_time, prev_time;

/* blocks */
static int n_blocks;
static char **block_names;
static double block_power[9]={0.5,0.01,0.1,0.5,0.01,1.0,2.0,0.5,1.0};
//power[get_blk_index(flp, "MC")] =  0.5;	/* set the power numbers instead of '0'	*/
//power[get_blk_index(flp, "LinkW")] =  0.01;
//power[get_blk_index(flp, "RTR")] =  0.1;
//power[get_blk_index(flp, "L3")] =  0.5;	
//power[get_blk_index(flp, "LinkN")] =  0.01;
//power[get_blk_index(flp, "L2")] =  1.0;
//power[get_blk_index(flp, "Core")] =  2.0;
//power[get_blk_index(flp, "L1D")] =  0.5;
//power[get_blk_index(flp, "L1I")] =  1.0;	

/* misc */
static int first_call = TRUE;


char **alloc_names(int nr, int nc)
{
	int i;
	char **m;

	m = (char **) calloc (nr, sizeof(char *));
	assert(m != NULL);
	m[0] = (char *) calloc (nr * nc, sizeof(char));
	assert(m[0] != NULL);

	for (i = 1; i < nr; i++)
    	m[i] =  m[0] + nc * i;

	return m;
}

/* write a single line of temperature trace(in degree C)	*/
void write_vals(FILE *fp, double *vals, int size)
{
	int i;
	for(i=0; i < size-1; i++)
		fprintf(fp, "%.2f\t", vals[i]-273.15);
	fprintf(fp, "%.2f\n", vals[i]-273.15);
}



/* sample model initialization	*/
int sim_init(char *flp_file, char *config_file, char *init_file, char *steady_file, char *gridsteady_file)
{
        int size;
	int i, idx;

	/* initialize flp, get adjacency matrix */
	flp = read_flp(flp_file, FALSE);


	/* initialize epoch counter */
	epochs = 0;


	/* 
	 * configure thermal model parameters. default_thermal_config 
	 * returns a set of default parameters. only those configuration
	 * parameters (config.*) that need to be changed are set explicitly. 
	 */
	global_config_t global_config;
	strcpy(global_config.config, config_file);
	size += read_str_pairs(&table[size], MAX_ENTRIES, global_config.config);
	size = str_pairs_remove_duplicates(table, size);

	thermal_config_t config = default_thermal_config();
	/* modify according to command line / config file	*/
	thermal_config_add_from_strs(&config, table, size);

	/* if package model is used, run package model */
	if (((idx = get_str_index(table, size, "package_model_used")) >= 0) && !(table[idx].value==0)) {
		if (config.package_model_used) {
			avg_sink_temp = config.ambient + SMALL_FOR_CONVEC;
			natural = package_model(&config, table, size, avg_sink_temp);
			if (config.r_convec<R_CONVEC_LOW || config.r_convec>R_CONVEC_HIGH)
				printf("Warning: Heatsink convection resistance is not realistic, double-check your package settings...\n"); 
		}
	}

	strcpy(config.init_file, init_file);
	strcpy(config.steady_file, steady_file);
	strcpy(config.grid_steady_file, gridsteady_file);
	
	/* default_thermal_config selects block model as the default.
	 * in case grid model is needed, select it explicitly and
	 * set the grid model parameters (grid_rows, grid_cols, 
	 * grid_steady_file etc.) appropriately. for e.g., in the
	 * following commented line, we just choose the grid model 
	 * and let everything else to be the default. 
	 * NOTE: for modeling 3-D chips, it is essential to set
	 * the layer configuration file (grid_layer_file) parameter.
	 */
	/* strcpy(config->model_type, GRID_MODEL_STR); */
	strcpy(config.model_type, GRID_MODEL_STR);

	/* allocate and initialize the RC model	*/
	model = alloc_RC_model(&config, flp);
	populate_R_model(model, flp);
	populate_C_model(model, flp);

	/* allocate the temp and power arrays	*/
	/* using hotspot_vector to internally allocate any extra nodes needed	*/
	temp = hotspot_vector(model);
	power = hotspot_vector(model);
	steady_temp = hotspot_vector(model);
	overall_power = hotspot_vector(model);
	
	/* set up initial instantaneous temperatures */
	if (strcmp(model->config->init_file, NULLFILE)) {
	  if (!model->config->dtm_used)	{/* initial T = steady T for no DTM	*/
	    read_temp(model, temp, model->config->init_file, FALSE);
	  }
		else	/* initial T = clipped steady T with DTM	*/
			read_temp(model, temp, model->config->init_file, TRUE);
	}
	else	/* no input file - use init_temp as the common temperature	*/
		set_temp(model, temp, model->config->init_temp);


	/* n_blocks is the number of functional blocks in the block model
	 * while it is the sum total of the number of functional blocks
	 * of all the floorplans in the power dissipating layers of the 
	 * grid model. 
	 */
	//if (model->type == BLOCK_MODEL)
	//n = model->block->flp->n_units;
	//else 
	n_blocks = 0;
	if (model->type == GRID_MODEL) {
		for(i=0; i < model->grid->n_layers; i++)
			if (model->grid->layers[i].has_power)
				n_blocks += model->grid->layers[i].flp->n_units;
	} else 
		fatal("unknown model type\n");
	// DEBUG
	//printf("\n\nDEBUG: n=%d\n\n",n_blocks);
	/* names of functional units	*/
	block_names = alloc_names(MAX_UNITS, STR_SIZE);
	for( i = 0; i < n_blocks; i++) {
	  strcpy(block_names[i],(flp->units+i)->name);
	}

	return n_blocks;

}

/* 
 * sample routine to illustrate the possible use of hotspot in a 
 * cycle-by-cycle power model. note that this is just a stub 
 * function and is not called anywhere in this file	
 */
void sim_main(double *power_sim, double *temp_sim, double time_sim, int silent_mode)
{
	int i, j, base, idx, count;
	//double *vals;

	//vals = dvector(MAX_UNITS);


	//initial temperature
	/* permute back to the trace file order	*/
	if (time_sim == 0.0) {
	  prev_time = 0.0;
	  first_call = TRUE;
	  if (model->type == BLOCK_MODEL)
	    for(i=0; i < n_blocks; i++)
	      temp_sim[i] = temp[get_blk_index(flp, block_names[i])];
	  else
	    for(i=0, base=0, count=0; i < model->grid->n_layers; i++) {
	      if(model->grid->layers[i].has_power) {
		for(j=0; j < model->grid->layers[i].flp->n_units; j++) {
		  idx = get_blk_index(model->grid->layers[i].flp, block_names[count+j]);
		  temp_sim[count+j] = temp[base+idx];
		}
		count += model->grid->layers[i].flp->n_units;	
	      }	
	      base += model->grid->layers[i].flp->n_units;	
	    }
	  return;
	}

	cur_time = time_sim;


	/* set the per cycle power values as returned by Wattch/power simulator	*/
	if (model->type == BLOCK_MODEL) {
	  for (j=0; j < n_blocks; j++) {
	    power[get_blk_index(flp, block_names[j])] = power_sim[j];	/* set the power numbers instead of '0'	*/
	  }
	  
	  /* for the grid model, set the power numbers for all power dissipating layers	*/
	} 
	else
	  for(i=0, base=0; i < model->grid->n_layers; i++) {
	    if(model->grid->layers[i].has_power) {
	      for(j=0; j < n_blocks;  j++) {
		idx = get_blk_index(model->grid->layers[i].flp, block_names[j]);
		power[base+idx] = power_sim[j];
	      }
	      
	    }	
	    base += model->grid->layers[i].flp->n_units;	
	  }
	
	/* call compute_temp at regular intervals */
	if ((cur_time - prev_time) >= model->config->sampling_intvl) {
	  double elapsed_time = (cur_time - prev_time);
	  prev_time = cur_time;
    if (!silent_mode) printf("\nELAPSED TIME: %f\n",cur_time);
	  
	  /* find the average power dissipated in the elapsed time */
	  if (model->type == BLOCK_MODEL) {
	    for (i = 0; i < flp->n_units; i++) {
	      /* for steady state temperature calculation	*/
	      overall_power[i] += power[i];
	      /* 
	       * 'power' array is an aggregate of per cycle numbers over 
	       * the sampling_intvl. so, compute the average power 
	       */
	      //power[i] /= (elapsed_time * model->config->base_proc_freq);
	    }
	    /* for the grid model, account for all the power dissipating layers	*/	
	  } else
	    for(i=0, base=0; i < model->grid->n_layers; i++) {
	      if(model->grid->layers[i].has_power)
		for(j=0; j < model->grid->layers[i].flp->n_units; j++) {
		  /* for steady state temperature calculation	*/
		  overall_power[base+j] += power[base+j];
		  /* compute average power	*/
		  //power[base+j] /= (elapsed_time * model->config->base_proc_freq);
		}
	      base += model->grid->layers[i].flp->n_units;
	    }

	  /* calculate the current temp given the previous temp, time
	   * elapsed since then, and the average power dissipated during 
	   * that interval. for the grid model, only the first call to 
	   * compute_temp passes a non-null 'temp' array. if 'temp' is  NULL, 
	   * compute_temp remembers it from the last non-null call. 
	   * this is used to maintain the internal grid temperatures 
	   * across multiple calls of compute_temp
	   */
	  if (model->type == BLOCK_MODEL || first_call)
	    compute_temp(model, power, temp, elapsed_time);
	  else
	    compute_temp(model, power, NULL, elapsed_time);
	  
	  /* make sure to record the first call	*/
	  first_call = FALSE;

	
	  /* permute back to the trace file order	*/
	  if (model->type == BLOCK_MODEL)
	    for(i=0; i < n_blocks; i++)
	      temp_sim[i] = temp[get_blk_index(flp, block_names[i])];
	  else
	    for(i=0, base=0, count=0; i < model->grid->n_layers; i++) {
	      if(model->grid->layers[i].has_power) {
		for(j=0; j < model->grid->layers[i].flp->n_units; j++) {
		  idx = get_blk_index(model->grid->layers[i].flp, block_names[count+j]);
		  temp_sim[count+j] = temp[base+idx];
		}
		count += model->grid->layers[i].flp->n_units;	
	      }	
	      base += model->grid->layers[i].flp->n_units;	
	    }
	  
	  /* output instantaneous temperature trace	*/
	  //write_vals(tout, temp_sim, n_blocks);
	  
	  
	  /* reset the power array */
	  if (model->type == BLOCK_MODEL)
	    for (i = 0; i < flp->n_units; i++)
	      power[i] = 0;
	  else
	    for(i=0, base=0; i < model->grid->n_layers; i++) {
	      if(model->grid->layers[i].has_power)
		for(j=0; j < model->grid->layers[i].flp->n_units; j++)
		  power[base+j] = 0;
	      base += model->grid->layers[i].flp->n_units;
	    }
	  //increasing simulation time (1-ms step)
	  epochs ++;
	}
}

/* 
 * sample uninitialization routine to illustrate the possible use of hotspot in a 
 * cycle-by-cycle power model. note that this is just a stub 
 * function and is not called anywhere in this file	
 */
void sim_exit()
{
        double total_elapsed_cycles = (double)epochs; 	/* set this to be the correct time elapsed  (in cycles) */
	int i, j, base;

	/* find the average power dissipated in the elapsed time */
	if (model->type == BLOCK_MODEL)
		for (i = 0; i < flp->n_units; i++)
			overall_power[i] /= total_elapsed_cycles;
	else		
		for(i=0, base=0; i < model->grid->n_layers; i++) {
			if(model->grid->layers[i].has_power)
				for(j=0; j < model->grid->layers[i].flp->n_units; j++)
					overall_power[base+j] /= total_elapsed_cycles;
			base += model->grid->layers[i].flp->n_units;
		}

	/* get steady state temperatures */
	steady_state_temp(model, overall_power, steady_temp);

	/* dump temperatures if needed	*/
	if (strcmp(model->config->steady_file, NULLFILE))
		dump_temp(model, steady_temp, model->config->steady_file);

	/* for the grid model, optionally dump the internal 
	 * temperatures of the grid cells	
	 */
	if (model->type == GRID_MODEL &&
		strcmp(model->config->grid_steady_file, NULLFILE))
		dump_steady_temp_grid(model->grid, model->config->grid_steady_file);

	/* cleanup */
	delete_RC_model(model);
	free_flp(flp, FALSE);
	free_dvector(temp);
	free_dvector(power);
	free_dvector(steady_temp);
	free_dvector(overall_power);
}


/*int main(int argc, char **argv)
{
  int i;
  double *power_sim,*temp_sim, time_sim;
  FILE *tout;
  
  tout = fopen(ttrace_file, "w");

  sim_init();

  power_sim=dvector(n_blocks);
  for(i=0;i<n_blocks;i++){
    *(power_sim+i)=block_power[i];
  }
  temp_sim=dvector(n_blocks);

  time_sim = 0.0;

  for(i=0;i<=9;i++) {
    sim_main(power_sim,temp_sim,time_sim,false);

    //output instantaneous temperature trace
    write_vals(tout, temp_sim, n_blocks);
    time_sim += 0.001;
  }

  sim_exit();

  free(power_sim);
  free(temp_sim);
  return 0;
}*/
