/*
 * Charm++ interface to Collision detection system
 * Orion Sky Lawlor, olawlor@acm.org, 2003/3/19
 */
#ifndef __UIUC_CHARM_COLLIDE_H
#define __UIUC_CHARM_COLLIDE_H

#include "charm++.h"
#include "collide_util.h"

//********************** collideClient *****************
/// A place for the assembled collision lists to go.
/// Each voxel calls this collisions() method with their 
/// lists of collisions.  Because there can be many voxels per 
/// processor, the canonical implementation does some kind 
/// of reduction over the voxels array--this way you know
/// when all voxels have reported their Collisions.
///
/// If you just want the Collisions collected onto one processor, 
/// use the serial collision client interface.
///
/// \see CollideSerialClient
/// \see serialCollideClient
class collideClient : public Group {
 public:
  virtual ~collideClient();
  virtual void collisions(ArrayElement *src,
                          int step,CollisionList &colls) =0;
};

//********************* serialCollideClient *****************
// Reduces the Collision list down to processor 0.

/// Type definition of a function to be called on PE 0 with a collision list.
/// This is the type of the serial routine to be passed to CollideSerialClient.
/// Used only with a serial collide client.
///
/// \see CollideSerialClient
typedef void (*CollisionClientFn)(void *param, int nColl, Collision *colls);

/// Call this on processor 0 to build a collision client that just calls a
/// serial routine (of type CollisionClientFn) on processor 0 with the final,
/// complete collision list.
///
/// This function creates a new serialCollideClient collide client group
/// and returns a handle to it (CkGroupID).
///
/// \see serialCollideClient
/// \see CollisionClientFn
CkGroupID CollideSerialClient(CollisionClientFn clientFn, ///< serial routine
                              void *clientParam ///< parameters to the serial routine
                              );

/****************** Collision Interface ******************/
/// Handle to a group
typedef CkGroupID CollideHandle;

/// Create a collider manager group to contribute objects to.  
/// Should be called on processor 0.
CollideHandle CollideCreate(const CollideGrid3d &gridMap, ///< collision grid
                            CkGroupID clientGroupID ///< handle to the colide client group
                            );

/// Register with this collider group. (Call on creation/arrival)
void CollideRegister(CollideHandle h, ///< handle to the collide manager group
                     int chunkNo ///< id of the contributor element
                     );

/// Unregister with this collider group. (Call on deletion)
void CollideUnregister(CollideHandle h, ///< handle to the collide manager group
                       int chunkNo ///< id of the contributor element
                       );

/// Send these objects off to be collided.
///
/// The results go the collisionClient group registered at creation time.
/// This function simply invokes the contribute() method on the local branch
/// of the collide manager group, passing to it the contributor element, the
/// number of boxes contributed by the element, the actual boxes, and the priority
/// flags.
///
/// Note: Passing NULL for the priorities defaults to assigning priorities
/// equal to the chunkNo (which means boxes created by the same element do
/// not collide)
void CollideBoxesPrio(CollideHandle h, ///< handle to the collide manager group
                      int chunkNo, ///< id of the contributor element
                      int nBox, ///< number of boxes contributed
                      const bbox3d *boxes, ///< list of actual boxes
                      const int *prio=NULL ///< priority flags
                      );

#endif
