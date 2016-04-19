#ifndef _SMP_EXTEND_LP_QP_HPP_
#define _SMP_EXTEND_LP_QP_HPP_

#include <smp/components/extenders/lq_qp.h>
#include <smp/components/extenders/state_array_double.hpp>
#include <smp/components/extenders/input_array_double.hpp>
#include <smp/planner_utils/trajectory.hpp>
#include <smp/planner_utils/vertex_edge.hpp>
#include <smp/components/extenders/base.h>

using namespace std;

template <class typeparams, int NUM_DIMENSIONS> 
double smp::extender_lq_qp<typeparams, NUM_DIMENSIONS>::
lq_qp_extend( state_t* state_from_in, 
		state_t* state_towards_in,
		int* exact_connection_out, 
		trajectory_t* trajectory_out,
		list<state_t*>* intermediate_vertices_out )
{
  return 1;
}

template< class typeparams, int NUM_DIMENSIONS >
int smp::extender_lq_qp< typeparams, NUM_DIMENSIONS >
::set_max_length (double max_length_in) {

  if (max_length_in <= 0.0)
    return 0;

  max_length = max_length_in;

  return 1;
}


template <class typeparams, int NUM_DIMENSIONS> 
smp::extender_lq_qp<typeparams, NUM_DIMENSIONS>::extender_lq_qp( )
{
}

template <class typeparams, int NUM_DIMENSIONS> 
smp::extender_lq_qp<typeparams, NUM_DIMENSIONS>::~extender_lq_qp( )
{
}

template <class typeparams, int NUM_DIMENSIONS> 
int smp::extender_lq_qp<typeparams, NUM_DIMENSIONS>::
ex_update_insert_vertex( vertex_t* vertex_in )
{
  return 1;
}

template <class typeparams, int NUM_DIMENSIONS> 
int smp::extender_lq_qp<typeparams, NUM_DIMENSIONS>::
ex_update_insert_edge( edge_t* edge_in )
{
  return 1;
}

template <class typeparams, int NUM_DIMENSIONS> 
int smp::extender_lq_qp<typeparams, NUM_DIMENSIONS>::
ex_update_delete_vertex( vertex_t* vertex_in )
{
  return 1;
}

template <class typeparams, int NUM_DIMENSIONS> 
int smp::extender_lq_qp<typeparams, NUM_DIMENSIONS>::
ex_update_delete_edge( edge_t* edge_in )
{
  return 1;
}

template <class typeparams, int NUM_DIMENSIONS> 
int smp::extender_lq_qp<typeparams, NUM_DIMENSIONS>::
extend( state_t* state_from_in, 
	  state_t* state_towards_in,
	  int* exact_connection_out, 
	  trajectory_t* trajectory_out,
	  list<state_t*>* intermediate_vertices_out )
{
  return 1;
}

#endif

