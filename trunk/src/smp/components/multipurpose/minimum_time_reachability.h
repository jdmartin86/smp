/*! \file components/multipurpose/minimum_time_reachability.h
  \brief An implementation of the vertex and edge components in the graph.
  
  Provides an implementation of the vertex and edge components in the graph. Both classes
  are defined as templates that take the types of the state, input, and the data stored in
  the vertices as well as the type of the data that is stored in the edges as an argument.
*/


#ifndef _SMP_MINIMUM_TIME_REACHABILITY_H_
#define _SMP_MINIMUM_TIME_REACHABILITY_H_

#include <smp/planners/rrtstar.h>
#include <smp/common/region.h>
#include <smp/components/model_checkers/base.h>
#include <smp/components/cost_evaluators/base.h>



namespace smp {


    //! Vertex data for minimum-time reachability.
    /*!
      This data structure is attached to each vertex in the graph maintained by the planner
      algorithm. The data structure includes two variables. One variable indicates whether the associated
      vertex lies inside the goal region. Another variables keeps track of the cost to reach 
      this particular vertex starting from the root vertex. The latter variable is particularly 
      created to work with teh RRT* algorithm.
    */
    class minimum_time_reachability_vertex_data : public rrtstar_vertex_data {
        
    public:    
        
        //! Reachability of the goal region.
        /*!
          This variable that indicates whether the associated vertex 
          state is inside the goal region. 
        */
        bool reaches_goal;
    };



    //! Edge data for minimum-time reachability.
    /*!
      This empty class is implemented for the sake of completeness.
    */ 
    class minimum_time_reachability_edge_data : public rrtstar_edge_data {
    
    };
    
    
    
    //! A combination of the minimum-time cost evaluator and the reachability model checker
    /*!
      Combining the minimum-time cost evaluator and the reachability model checker, this class
      is able to keep track of the minimum-time that reaches the goal region. The class constitutes
      a good example of multiple-purpose algorithm component made possible with mutliple inheritance.
      
      \ingroup model_checkers
      \ingroup cost_evaluators
    */
    template< class typeparams, int NUM_DIMENSIONS >
    class minimum_time_reachability : public model_checker_base<typeparams> , public cost_evaluator_base<typeparams> {
        
        
        typedef typename typeparams::state state_t;
        typedef typename typeparams::input input_t;
        typedef typename typeparams::vertex_data vertex_data_t;
        typedef typename typeparams::edge_data edge_data_t;
        
        typedef vertex<typeparams> vertex_t;
        typedef edge<typeparams> edge_t;
        typedef trajectory<typeparams> trajectory_t;

        typedef region<NUM_DIMENSIONS> region_t;
        
        typedef int (*update_func_t)(trajectory_t *);
        
        
        list<update_func_t> list_update_functions;  // A list of functions that will be called in the 
                                                    //    event of updating the minimum cost trajectory.
    
        
        vertex_t *min_cost_vertex;         // A pointer to the minimum cost vertex in the tree
        trajectory_t min_cost_trajectory;  // A copy of the mininum cost trajectory 
    

        region_t region_goal;


    public:

        minimum_time_reachability ();
        ~minimum_time_reachability ();

        /**
         * \brief Constructor that initializes the goal region.
         *
         * This constructor initializes the goal region. Note that the
         * there is a constructor with no arguments. If initiated that
         * constructor will initialize the goal region to its default 
         * values derived from the region class, which amounts 
         * to a point in the origin. 
         *
         * @param region_goal New goal region. 
         */
        minimum_time_reachability (const region_t &region_goal);

        /**
         * \brief Modifies the goal region.
         *
         * This function sets the goal region to its new value given 
         * as an argument. 
         *
         * @param region_goal New goal region. 
         *
         * @returns Returns 1 if succcess, and a non-positive value to indicate error.
         */
        int set_goal_region (const region_t &region_goal);
    
        int ce_update_vertex_cost (vertex_t *vertex_in);
    
        int ce_update_edge_cost (edge_t *edge_in);

        int mc_update_insert_vertex (vertex_t *vertex_in);

        int mc_update_insert_edge (edge_t *edge_in);
    
        int mc_update_delete_vertex (vertex_t *vertex_in);

        int mc_update_delete_edge (edge_t *edge_in); 
    
        int get_solution (trajectory_t &trajectory_out);
    
        double evaluate_cost_trajectory (state_t *state_initial_in,
                                         trajectory_t *trajectory_in,
                                         state_t *state_final_in = 0);
    

        /**
         * \brief Returns the cost of the best trajectory.
         *
         *  This function returns the cost of the minimum cost trajectory that reaches the goal, 
         *  if such a trajectory exists. Otherwise, it returns -1.0.
         *
         * @returns Returns the cost of the minimum cost trajectory, or -1.0 to indicate no such trajectory exists.
         */
        double get_best_cost ();

        /**
         * \brief Clears the update function list for minimum cost update.
         *
         * Whenever an optimizing motion planning algorithm using this component finds
         * a better trajectory, this component calls a list of functions that are registered 
         * for this call back. This method clears this list of functions. 
         *
         * @returns Returns 1 if succcess, and a non-positive value to indicate error.
         */
        int clear_update_function_list ();

        /**
         * \brief Clears the update function list for minimum cost update.
         *
         * Whenever an optimizing motion planning algorithm using this component finds
         * a better trajectory, this component calls a list of functions that are registered 
         * for this call back. This method registers a new update function, i.e., adds 
         * the function given in the argument to the appropriate list.
         *
         * @returns Returns 1 if succcess, and a non-positive value to indicate error.
         */
        int register_new_update_function (update_func_t update_function);
    
    };


}

#endif
