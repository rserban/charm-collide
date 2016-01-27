//// Debug print
//#define RADU_DBG


/*
Simple Charm++ collision detection test program--
Orion Sky Lawlor, olawlor@acm.org, 2003/3/18
 */
#include <stdio.h>
#include <string.h>
#include "collidecharm.h"
#include "hello.decl.h"

CProxy_main mid;
CProxy_Hello arr;
int nElements;

/// Serial routine called with final list of collisions.
/// Prints the detected collision, then calls the main finalization function.
///
/// - A Collision object is simply a pair of two objects (identified
///   through the members A and B).
/// - A collision object (class CollideObjID) encapsulates the source
///   chunk (the element that created it) and an index within that chunk.
///   Additional data for each object includes "priority" (objects with
///   same priority do not collide) and the processor chunk on which the
///   (migratable) object last lived on.
///
/// Collision and CollideObjID defined in collide_util.h
void printCollisionHandler(void *param,int nColl,Collision *colls)
{
  CkPrintf("**********************************************\n");
  CkPrintf("*** Final collision handler called-- %d records:\n",nColl);
  int nPrint=nColl;
  const int maxPrint=30;
  if (nPrint>maxPrint) nPrint=maxPrint;
  for (int c=0;c<nPrint;c++) {
    CkPrintf("%d(%d):%d hits %d(%d):%d\n",
             colls[c].A.chunk, colls[c].A.pe, colls[c].A.number,
             colls[c].B.chunk, colls[c].B.pe, colls[c].B.number);
  }
  CkPrintf("**********************************************\n");
  mid.maindone();
}

// ------------------------------------------------------------------------

/// Main chare
class main : public Chare
{
public:
  main(CkMigrateMessage *m) {}
  
  /// Main entry method.
  /// 1. Create the collision grid.
  ///    The grid will span the 1st quadrant (lower-left-bottom corner at the
  ///    origin) and have cells each of dimension 2x100x2.
  ///
  ///    CollideGrid3D defined in collide_util.{h,C}
  ///
  /// 2. Create a collide client group.
  ///    Create a collide manager group (passing it the grid and the client group)
  ///    Note: a group is a collection of chares that have a single chare (called
  ///    'branch') on each PE.
  ///
  ///    Here, we use a "serial" collide client which collects (reduces) all
  ///    collisions to PE 0 and sends the resulting list to the specified function.
  ///    The function 'printCollisionHandler' prints the detected collisions and then
  ///    invokes the maindone() entry point to finalize the simulation.
  ///
  ///    CollideCreate performs the following operations:
  ///    - creates 'voxels', a 3D array of chares (initially empty)
  ///    - creates and returns a handle (CkGroupID) to a collide manager group
  ///
  ///    Note: CollideHandle == CkGroupID (typedef)
  ///
  /// 3. Create an array of "contributor" chares.
  ///    Each of them will generate a number of objects to be collided.
  ///
  /// 4. Invoke the DoIt method on all chares in the array of contributors.
  ///
  /// \see CollideCreate defined in collidecharm.{h,C}
  /// \see CollideSerialClient defined in collidecharm.{h,C}
  ///
  main(CkArgMsg* m)
  {
    nElements=5;
    if(m->argc > 1) nElements = atoi(m->argv[1]);
    delete m;
    CkPrintf("Running Hello on %d processors for %d elements\n",
	     CkNumPes(), nElements);
    mid = thishandle;

    CollideGrid3d gridMap(CkVector3d(0,0,0),CkVector3d(2,100,2));
 
    CollideHandle collide = CollideCreate(gridMap,
                                          CollideSerialClient(printCollisionHandler,0));
    

    arr = CProxy_Hello::ckNew(collide,nElements);
    
    arr.DoIt();
  };

  /// Final function (entry method).
  /// Invoked at end of execution.
  void maindone(void)
  {
    CkPrintf("All done\n");
    CkExit();
  };
};

// ------------------------------------------------------------------------

/// Hello chares (1D array)
class Hello : public ArrayElement1D
{
  CollideHandle collide;  ///< handle to the collide manager group
  int nTimes; ///< number of separate collision operations
  
public:
  
  /// Constructor (entry method).
  /// When constructed, an element registers itself with the local
  /// branch of the collide manager group as a contributor.
  Hello(const CollideHandle &collide_) :collide(collide_)
  {
    CkPrintf("Creating element %d on PE %d\n", thisIndex, CkMyPe());
    nTimes = 0;
    CollideRegister(collide, thisIndex);
  }

  Hello(CkMigrateMessage *m) :ArrayElement1D(m) {}

  /// Pack-Unpack method.
  /// We pack/unpack the handle to the collide manager group.
  /// When unpacking, we register this element with the local branch
  /// of the collide manager group on the new PE.
  void pup(PUP::er &p)
  {
    ArrayElement1D::pup(p);
    p|collide;
    if (p.isUnpacking())
      CollideRegister(collide, thisIndex);
  }

  /// Destructor.
  /// When this element is being destroyed (end of program or because it
  /// is being migrated), notify the local branch of the collide manager
  /// group about this (i.e. unregister this element as a contributor)
  ~Hello()
  {
    CollideUnregister(collide, thisIndex);
  }

  /// Main worker function (entry method).
  /// In this simple example, an element performs the following:
  /// 1. Create a bunch of boxes from this element.  These are all
  ///    in the x-y plane and all parallel to the y-axis.
  /// 2. Stretch the first box over into next object.
  ///    With this, the first box on elment 'e' will overlap with the first
  ///    and second box on element 'e+1'
  /// 3. Send these objects to be collided. Results go to the collide manager
  ///    group with which this chunk (element) was registered at construction. 
  ///    This is done by invoking the method 'contribute' on the local branch
  ///    of the collide manager group, passing it the chunk (element) number,
  ///    the number of boxes created by this element, the actual boxes, and
  ///    the array of priority flags.
  ///
  /// Setting priority=NULL here will use the default setting which is
  /// priority=thisIndex.  In other words, objects created in the same
  /// chunk (i.e. by the same element) will not collide with each other.
  ///
  /// \see CollideBoxesPrio defined in collidecharm.{h,C}
  ///
  void DoIt(void)
  {
    CkPrintf("Contributing to reduction %d, element %04d\n", nTimes, thisIndex);

    // Create a bunch of boxes from this element.

    ////CkVector3d o(-6.8, 7.9, 8.0);
    CkVector3d o( 0,   0,   0);
    CkVector3d x( 4,   0,   0);
    CkVector3d y( 0,   0.3, 0);
    CkVector3d boxSize(0.2, 0.2, 0.2);
    ////int nBoxes = 1000;
    int nBoxes = 10;
    bbox3d *box = new bbox3d[nBoxes];
    for (int i=0; i<nBoxes; i++) {
      CkVector3d c(o + x*thisIndex + y*i);
      CkVector3d c2(c + boxSize);
      // Create a box with corners 'c' and 'c2'
      box[i].empty();
      box[i].add(c); box[i].add(c2);
      
#ifdef RADU_DBG
      // Write box information
      rSeg1d x_dim = box[i].axis(0);
      rSeg1d y_dim = box[i].axis(1);
      rSeg1d z_dim = box[i].axis(2);
      CkPrintf("%d:%d   (%g, %g, %g) - (%g, %g, %g)\n",
               thisIndex, i,
               x_dim.getMin(), y_dim.getMin(), z_dim.getMin(),
               x_dim.getMax(), y_dim.getMax(), z_dim.getMax());
#endif
    } 
    
    // Stretch the first box over into next object.
    box[0].add(o + x*(thisIndex+1.5) + y*2);

#ifdef RADU_DBG	
    // Stretched box[0]
    rSeg1d x_dim = box[0].axis(0);
    rSeg1d y_dim = box[0].axis(1);
    rSeg1d z_dim = box[0].axis(2);
    CkPrintf(">>> %d:0   (%g, %g, %g) - (%g, %g, %g)\n",
             thisIndex,
             x_dim.getMin(), y_dim.getMin(), z_dim.getMin(),
             x_dim.getMax(), y_dim.getMax(), z_dim.getMax());
#endif
    
    // Send these objects to be collided.
    CollideBoxesPrio(collide, thisIndex, nBoxes, box, NULL);
    
    delete[] box;
    nTimes++;
  }
};

#include "hello.def.h"

