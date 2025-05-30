# Makefile for Pinocchio
CC = g++
CCFLAGS = -c -O3 -Wall -w -fPIC -fpermissive -std=c++17
EXE_LIBS = -lm
SHARED_LIBS = -lm -shared -fPIC

OBJECTS = attachment.o discretization.o indexer.o lsqSolver.o mesh.o \
	graphutils.o intersector.o matrix.o skeleton.o embedding.o \
	pinocchioApi.o refinement.o quatinterface.o processor.o

all: Processor libpinocchio.so

Processor: $(OBJECTS)
	$(CC) $(OBJECTS) $(EXE_LIBS) -o Processor

libpinocchio.so: $(OBJECTS)
	$(CC) $(OBJECTS) $(SHARED_LIBS) -o libpinocchio.so

.cpp.o:
	$(CC) $(CCFLAGS) $<


#do a makedepend and remove all the external dependencies from the makefile
#works on my system, no one else should need to do it.
depend:
	makedepend *.cpp > /dev/null >& 1
	perl -pi -e 's/\.c\.o/\.o/' Makefile
	perl -pi -e 's/\/u[Ss]r[a-zA-Z0-9\+\-\/\.\_]*//' Makefile
	perl -pi -e 's/\/u[Ss]r[a-zA-Z0-9\+\-\/\.\_]*//' Makefile
	perl -pi -e 's/\/u[Ss]r[a-zA-Z0-9\+\-\/\.\_]*//' Makefile
	perl -pi -e 's/\/u[Ss]r[a-zA-Z0-9\+\-\/\.\_]*//' Makefile
	perl -pi -e 's/\/u[Ss]r[a-zA-Z0-9\+\-\/\.\_]*//' Makefile
	perl -pi -e 's/\/u[Ss]r[a-zA-Z0-9\+\-\/\.\_]*//' Makefile
	perl -pi -e 's/\/u[Ss]r[a-zA-Z0-9\+\-\/\.\_]*//' Makefile
	perl -pi -e 's/\/u[Ss]r[a-zA-Z0-9\+\-\/\.\_]*//' Makefile
	perl -pi -e 's/^[a-zA-Z].*\.o\:\s*$$//' Makefile

clean:
	rm -f $(OBJECTS) Processor libpinocchio.so *~ core gmon.out *.bak


# DO NOT DELETE

Pinocchio.o: Pinocchio.h
attachment.o: attachment.h mesh.h vector.h hashutils.h mathutils.h
attachment.o:  Pinocchio.h rect.h skeleton.h quatinterface.h
attachment.o: graphutils.h transform.h vecutils.h lsqSolver.h
discretization.o: pinocchioApi.h mesh.h vector.h hashutils.h mathutils.h
discretization.o:  Pinocchio.h rect.h
discretization.o: quaddisttree.h dtree.h indexer.h multilinear.h
discretization.o: intersector.h vecutils.h pointprojector.h debugging.h
discretization.o: attachment.h skeleton.h graphutils.h transform.h deriv.h
embedding.o: pinocchioApi.h mesh.h vector.h hashutils.h mathutils.h
embedding.o:  Pinocchio.h rect.h quaddisttree.h
embedding.o: dtree.h indexer.h multilinear.h intersector.h vecutils.h
embedding.o: pointprojector.h debugging.h attachment.h skeleton.h
embedding.o: graphutils.h transform.h
graphutils.o: graphutils.h vector.h hashutils.h mathutils.h
graphutils.o:  Pinocchio.h debugging.h
indexer.o: indexer.h hashutils.h mathutils.h 
indexer.o: Pinocchio.h vector.h
intersector.o: intersector.h mesh.h vector.h hashutils.h mathutils.h
intersector.o:  Pinocchio.h rect.h vecutils.h
lsqSolver.o: lsqSolver.h  
lsqSolver.o:   mathutils.h
lsqSolver.o: Pinocchio.h hashutils.h debugging.h
matrix.o: matrix.h mathutils.h  
matrix.o:  Pinocchio.h debugging.h
mesh.o: mesh.h vector.h hashutils.h mathutils.h vec3.hpp
mesh.o:   Pinocchio.h dual_quat_cu.hpp
mesh.o: rect.h utils.h debugging.h
pinocchioApi.o: pinocchioApi.h mesh.h vector.h hashutils.h mathutils.h
pinocchioApi.o:  Pinocchio.h rect.h
pinocchioApi.o: quaddisttree.h dtree.h indexer.h multilinear.h intersector.h
pinocchioApi.o: vecutils.h pointprojector.h debugging.h attachment.h
pinocchioApi.o: skeleton.h graphutils.h transform.h
refinement.o: pinocchioApi.h mesh.h vector.h hashutils.h mathutils.h
refinement.o:  Pinocchio.h rect.h quaddisttree.h
refinement.o: dtree.h indexer.h multilinear.h intersector.h vecutils.h
refinement.o: pointprojector.h debugging.h attachment.h skeleton.h
refinement.o: graphutils.h transform.h deriv.h
skeleton.o: skeleton.h graphutils.h vector.h hashutils.h mathutils.h
skeleton.o:  Pinocchio.h utils.h debugging.h
quatinterface.o: quatinterface.h dual_quat_cu.hpp quat_cu.hpp
quatinterface.o: transfo.hpp transform.h vector.h
processor.o:  processor.h
processor.o: mesh.h vector.h
processor.o: hashutils.h mathutils.h
processor.o:  Pinocchio.h
processor.o: rect.h transform.h
processor.o: skeleton.h graphutils.h
processor.o: utils.h debugging.h
processor.o: attachment.h mesh.h
processor.o: skeleton.h transform.h
processor.o: pinocchioApi.h quaddisttree.h
processor.o: dtree.h indexer.h
processor.o: multilinear.h intersector.h
processor.o: vecutils.h pointprojector.h
processor.o: debugging.h attachment.h
processor.o: matrix.h vector.h