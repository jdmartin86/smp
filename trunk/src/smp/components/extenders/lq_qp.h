/*! \file components/extenders/base.h
  \brief The two-point boundary value extender. This solves a quadratic program
  to minimize the linear quadratic cost.
  
  The extender (aka, the extension function) generates that exactly or approximately 
  connects two given states. 
*/
#ifndef _SMP_EXTENDER_LQ_QP_H_
#define _SMP_EXTENDER_LQ_QP_H_


#include <smp/planner_utils/trajectory.h>
#include <smp/planner_utils/vertex_edge.h>
#include <smp/components/extenders/state_array_double.h>
#include <smp/components/extenders/input_array_double.h>

#include <smp/components/extenders/base.h>

#include <list>
#include <cmath>

namespace smp 
{

  
  //! State data structure for the single integrator dynamics
  /*!
    This class implements the state data structure for the single integrator dynamics. 
    The number of state variables is twice number of dimensions, since for each dimension
    both position and velocity has to be stored. The positions are stored in the usual order
    first, and then the all velocities are stored in their usual order, in the array.
      
    \ingroup states
  */
  template <int NUM_DIMENSIONS>
    class state_lq_qp : public state_array_double<NUM_DIMENSIONS> {

  };


  //! Input data structure for the single integrator dynamics
  /*!
    This class implements the input data structure for the single integrator dynamics. 
    The number of input variables is one plus the dimensions. The extra input variable,
    placed in the beginning of the array, is used to store the time it takes to 
    execute the trajectory segment. 
      
    \ingroup inputs
  */
  class input_lq_qp : public input_array_double<1> {
    
  };
  
    //! The abstract class that specifies the structure of the extender component.
    /*!
      An extender provides the function to generate a trajectory that connects two given 
      states. The extender can also provide a list of designated states, which become 
      vertices of their own when added to the graph maintained by the planning algorithm.
      
      \ingroup extenders_base
    */
  template<class typeparams, int NUM_DIMENSIONS>
      class extender_lq_qp : public extender_base<typeparams> {



        typedef typename typeparams::state state_t;
        typedef typename typeparams::input input_t;
        typedef typename typeparams::vertex_data vertex_data_t;
        typedef typename typeparams::edge_data edge_data_t;

        typedef vertex<typeparams> vertex_t;
        typedef edge<typeparams> edge_t;

        typedef trajectory< typeparams > trajectory_t;

	double max_length;
    
	double lq_qp_extend( state_t* state_from_in, 
			     state_t* state_towards_in,
			     int* exact_connection_out, 
			     trajectory_t* trajectory_out,
			     list<state_t*>* intermediate_vertices_out );

    public:
	extender_lq_qp( );
	~extender_lq_qp( );

        /**
         * \brief Update function for vertex insertion
         *
         * This function is called by the planner whenever a new vertex is
         * added to the graph. A pointer to the new vertex is given as an argument.
         *
         * @param vertex_in A pointer to the new vertex.
         *
         * @returns Return 1 if success, a non-positive value to indiacate error.
         */
	int ex_update_insert_vertex (vertex_t *vertex_in);
    

        /**
         * \brief Update function for edge insertion
         *
         * This function is called by the planner whenever a new edge is
         * added to the graph. A pointer to the new edge is given as an argument.
         *
         * @param edge_in A pointer to the new edge.
         *
         * @returns Return 1 for success, a non-positive value to indiacate error.
         */    
	int ex_update_insert_edge (edge_t *edge_in);    

        /**
         * \brief Update function for vertex deletion
         *
         * This function is called by the planner whenever a vertex is deleted 
         * from the graph. A pointer to the vertex is given as an argument.
         *
         * @param vertex_in A pointer to deleted vertex.
         *
         * @returns Return 1 if success, a non-positive value to indiacate error.
         */
	int ex_update_delete_vertex (vertex_t *vertex_in);
    

        /**
         * \brief Update function for edge insertion
         *
         * This function is called by the planner whenever an edge is delete
         * from the graph. A pointer to the edge is given as an argument.
         *
         * @param edge_in A pointer to deleted edge.
         *
         * @returns Return 1 for success, a non-positive value to indiacate error.
         */    
	int ex_update_delete_edge (edge_t *edge_in);    

    
        /**
         * \brief Abstract function that generates a trajectory connecting two given states. 
         *
         * Generates a trajectory, returned in the trajectory_out argument, that connects two
         * given states, provided with the state_from_in and state_towards_in arguments. If 
         * the connection is exact, i.e., the trajectory reaches state_towards_in exactly, 
         * then the output variable exact_connection_out is set to one. If, on the other hand,
         * the connection is approximate, then the same variable is set to zero. 
         *
         * @param state_from_in The state that the new trajectory starts from.
         * @param state_towards_in The state that the new trajectory is shooted towards. 
         * @param exact_connection_out Set to one if the connection is exact, otherwise 
         *                             this variable is set to zero by this function.
         * @param trajectory_out The output variable that contians the resulting trajectory.
         * @param intermediate_vertices_out The list of states that will be individual vertices. 
         * 
         * @returns Returns 1 for success, a non-positive number to indicate error.
         */
	int extend( state_t* state_from_in, 
		    state_t* state_towards_in,
		    int* exact_connection_out, 
		    trajectory_t* trajectory_out,
		    list<state_t*>* intermediate_vertices_out );

        /**
         * \brief Sets the maximum length of the trajectory returned by the algorithm.
         *
         * This method sets the length of the longest trajectory returned by the algorithm. 
         * If the trajectory connecting two given states is longer than the value specified
         * by the argument of this function then the only the maximum-length prefix of 
         * this trajectory is returned. By default, the max_length paramter is set to 1.0.
         *
         * @param max_length_in Maximum length of a trajectory.
         *
         * @returns Returns 1 for success, and a non-positive number to indicate error.
         */
        int set_max_length( double max_length_in );

    };

}

#endif
