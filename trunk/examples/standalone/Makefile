SUBDIRS:= rrt/single_integrator rrt/double_integrator rrt/dubins_car rrt/dubins_double_integrator_airplane \
	rrg/single_integrator rrg/single_integrator_mu_calculus rrg/dubins_car rrg/double_integrator rrg/dubins_double_integrator_airplane rrg/lq_qp \
	rrtstar/single_integrator rrtstar/single_integrator_halton rrtstar/single_integrator_trajectory_biasing rrtstar/single_integrator_trajectory_biasing_and_bnb \
	rrtstar/dubins_car rrtstar/double_integrator rrtstar/dubins_double_integrator_airplane 

.PHONY: all clean 
.SILENT:

all: 
	echo [Make all]
	@for dir in $(SUBDIRS) ; do \
	echo $$dir; $(MAKE) -C $$dir || exit 2; done

clean:
	echo [Clean]
	@for dir in $(SUBDIRS) ; do \
	echo $$dir; $(MAKE) -C $$dir clean || exit 2; done
	rm -f *~ lib/*
