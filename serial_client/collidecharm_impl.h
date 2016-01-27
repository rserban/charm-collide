/*
 * Parallel layer for Collision detection system
 * Orion Sky Lawlor, olawlor@acm.org, 4/8/2001
 */
#ifndef __UIUC_CHARM_COLLIDECHARM_IMPL_H
#define __UIUC_CHARM_COLLIDECHARM_IMPL_H

#include "collide_serial.h"
#include "collide_buffers.h"
#include "collidecharm.decl.h"
#include "collidecharm.h"

/******************** objListMsg *********************
A pile of objects sent to a Collision voxel.
Declared as a "packed" message
type, which converts the message to a flat byte array
only when needed (for sending across processors).
*/
class objListMsg : public CMessage_objListMsg
{
public:
	class returnReceipt {
		CkGroupID gid;
	public:
		int onPE;
		returnReceipt() {}
		returnReceipt(CkGroupID gid_,int onPE_) :gid(gid_),onPE(onPE_) {}
		void send(void);
	};
private:
	bool isHeapAllocated;
	returnReceipt receipt;
	
	int n;
	CollideObjRec *obj; //Bounding boxes & IDs
	
	void freeHeapAllocated();
public:
	objListMsg() :isHeapAllocated(false) {}
	
	//Hand control of these arrays over to this message,
	// which will delete them when appropriate.
	objListMsg(int n_,CollideObjRec *obj_,
		const returnReceipt &receipt_);
	~objListMsg() {freeHeapAllocated();}
	
	int getSource(void) {return receipt.onPE;}
	void sendReceipt(void) {receipt.send();}
	
	int getObjects(void) const {return n;}
	const CollideObjRec &getObj(int i) const {return obj[i];}
	const bbox3d &bbox(int i) const {return obj[i].box;}
	
	static void *pack(objListMsg *m);
	static objListMsg *unpack(void *m);
};


// ===============================================================================
//                      CollisionAggregator
// ===============================================================================

#include "ckhashtable.h"

// Forward declarations
class collideMgr;         // defined below
class voxelAggregator;    // defined in collidecharm.C

/// This class splits each chunk's objects into messages headed out to each voxel.  
///
/// It is implemented as a group <- R.S. this is original comment but seems wrong!!!
///
/// - Receives lists of points and triangles from the sources on a particular machine.
/// - Determines which voxels each triangle spans, and adds the triangle to each
///   voxelAggregator.
/// - Maintains a sparse hashtable voxels (hashed by the location of that voxel,
///   i.e. 3 integer coordinates)
///
/// Each member in the collide manager group has a CollisionAggregator
/// 
/// \see voxelAggregator
class CollisionAggregator {

  CollideGrid3d gridMap;  ///< collision grid
  CkHashtableT<CollideLoc3d, voxelAggregator*> voxels;  ///< hash table of voxels
  collideMgr *mgr;  ///< back-pointer to the collide manager
  
  /// Add a new accumulator to the hashtable.
  /// While aggregating the objects from a contributor element, if the destination
  /// voxel does not exist, a new voxel aggregator is created on the spot and
  /// added to the sparse hash table.
  voxelAggregator *addAccum(const CollideLoc3d &dest);

public:

  /// Construct a collision aggregator for the specified collision grid
  /// and associated with the specified collide manager.
  CollisionAggregator(const CollideGrid3d &gridMap_, ///< collision grid
                      collideMgr *mgr_               ///< associated collide manager
                      ); 

  /// Destructor: deletes all cached aggregators.
  ~CollisionAggregator();
  
  /// Distribute the objects from a chunk (element) to their appropriate
  /// voxels. In a loop over all contributed objects (boxes), do:
  /// - create a collide object record (an object ID + the actual box)
  /// - map the three axes (segments) of the box from world to grid coordinates.
  ///   The resulting 3 integer segments define a contiguous group of voxels that
  ///   are touched by this object (box). 
  /// - loop over all grid voxels touched by the box
  ///   - look-up the voxel in the cache (a voxel is identified by its 3 integer
  ///     grid coordinates)
  ///   - if the corresponding voxel aggregator does not exist already in the hash
  ///     table, create it
  ///   - add the object (box) to the voxel aggregator
  void aggregate(int pe,              ///< PE of the associated collide manager
                 int chunk,           ///< ID of the contributor element 
                 int n,               ///< number of boxes
                 const bbox3d *boxes, ///< list of boxes
                 const int *prio      ///< priority flags for these boxes
                 );
  
  /// Send off all accumulated voxel messages.
  /// Iterate over all entries in the hash table and ask the voxel aggregators to
  /// send off all their accumulated objects. Each voxel aggregator will simply
  /// pass their data to the associated collide manager for dispatch (by calling
  /// the manager's sendVoxelMessage() function)
  void send(void);
  
  /// Delete all cached aggregators.
  /// Iterate over all entries in the hash table and destory the voxel aggregators.
  void compact(void);
};

/********************* syncReductionMgr *****************
A group to synchronize on some event across the machine.
Maintains a reduction tree and waits for the "advance"
method to be called from each processor.  To handle 
non-autonomous cases, calls "pleaseAdvance" when an advance
is first expected from that PE.
*/
class syncReductionMgr : public CBase_syncReductionMgr
{
	CProxy_syncReductionMgr thisproxy;
	void status(const char *msg) {
		CkPrintf("SRMgr pe %d> %s\n",CkMyPe(),msg);
	}
	//Describes the reduction tree
	int onPE;
	enum {TREE_WID=4};
	int treeParent;//Parent in reduction tree
	int treeChildStart,treeChildEnd;//First and last+1 child
	int nChildren;//Number of children in the reduction tree
	void startStep(int stepNo,bool withProd);
	
	//State data
	int stepCount;//Increments by one every reduction, from zero
	bool stepFinished;//prior step complete
	bool localFinished;//Local advance called
	int childrenCount;//Number of tree children in delivered state
	void tryFinish(void);//Try to finish reduction

protected:
	//This is called by subclasses
	void advance(void);
	//This is offered for subclasses's optional use
	virtual void pleaseAdvance(void);
	//This is called on PE 0 once the reduction is finished
	virtual void reductionFinished(void);
public:
	int getStepCount(void) const {return stepCount;}
	syncReductionMgr();
	
	//Called by parent-- will you contribute?
	void childProd(int stepCount);
	//Called by tree children-- me and my children are finished
	void childDone(int stepCount);
};

// ===============================================================================
//                   collideMgr
// ===============================================================================

/// A chare group that synchronizes the collision detection process.
/// A single collision operation consists of:
/// - collect contributions from clients (contribute)
/// - aggregate the contributions
/// - send off triangle lists to voxels
/// - wait for replies from the voxels indicating sucessful delivery
/// - collideMgr reduction to make sure everybody's messages are delivered
/// - root broadcasts startCollision to collideVoxel array
/// - client group accepts the CollisionLists
///
/// A collide manager group is created on PE 0 (by calling CollideCreate) and a handle
/// to it is then passed to the contributor elements which will register themselves
/// with the collide manager group at creation.  As contributor elements are
/// destroyed/migrated, they always stay registered with the local branch of the 
/// collide manager group.
///
/// \see CollideCreate
/// \see CollisionAggregator
///
class collideMgr : public CBase_collideMgr
{
  CProxy_collideMgr thisproxy;  ///< handle to the collide manager group
  
private:
  /// Print a status message
  void status(const char *msg) {
    CkPrintf("CMgr pe %d> %s\n", CkMyPe(), msg);
  }

  int steps; ///< number of separate collision operations
  CProxy_collideVoxel voxelProxy; ///< handle to voxel chare array
  CollideGrid3d gridMap; ///< shape of 3D voxel grid
  CProxy_collideClient client; ///< handle to collision client group
  
  int nContrib; ///< Number of registered contributors
  int contribCount; ///< Number of contribute calls given this step
  
  CollisionAggregator aggregator; ///< aggregator of collision objects
  int msgsSent; ///< messages sent out to voxels
  int msgsRecvd; ///< return-receipt messages received from voxels

  void tryAdvance(void);
  
protected:
  /// Check if we're barren-- if so, advance now
  virtual void pleaseAdvance(void);

  /// This is called on PE 0 once the voxel send reduction is finished
  virtual void reductionFinished(void);

public:
  /// Construct a collide manager (entry method).
  collideMgr(const CollideGrid3d &gridMap, ///< collision grid
             const CProxy_collideClient &client, ///< handle to the collide client group (who receives the final collision list)
             const CProxy_collideVoxel &voxels ///< handle to (sparse) array of voxels
             );
  
  /// Register a new contributor.
  /// Maintain contributor registration count: increment count of contributors.
  void registerContributor(int chunkNo);

  /// Unregister an existing contributor.
  /// Maintain contributor registration count: decrement count of contributors.
  void unregisterContributor(int chunkNo);
  
  /// Clients call this to contribute their objects:
  ///
  /// - Aggregate the contributions
  /// - Send the contributions (deliver all outgoing messages) which are packed and provided
  ///   by the voxel aggregators maintained by the aggregator object.
  /// - When all contributors have contributed their parts:
  ///   - compact the aggregator (blow away all the old voxels to save memory)
  ///   - invoke tryAdvance()
  void contribute(int chunkNo,
                  int n,const bbox3d *boxes,const int *prio);
  
  /// voxelAggregators deliver messages to voxels via this bottleneck
  void sendVoxelMessage(const CollideLoc3d &dest,
                        int n,CollideObjRec *obj);
  
  /// collideVoxels send a return receipt here
  void voxelMessageRecvd(void);
};

/********************** collideVoxel ********************
A sparse 3D array that represents a region of space where
Collisions may occur.  Each step it accumulates triangle
lists and then computes their intersection
*/

class collideVoxel : public CBase_collideVoxel
{
	growableBufferT<objListMsg *> msgs;
	void status(const char *msg);
	void emptyMessages();
	void collide(const bbox3d &territory,CollisionList &dest);
public:
	collideVoxel(void);
	collideVoxel(CkMigrateMessage *m);
	~collideVoxel();
	void pup(PUP::er &p);
	
	void add(objListMsg *msg);
	void startCollision(int step,
		const CollideGrid3d &gridMap,
		const CProxy_collideClient &client);
};


/********************** serialCollideClient *****************
Reduces the Collision list down to processor 0.
*/
class serialCollideClient : public collideClient {
	CollisionClientFn clientFn;
	void *clientParam;
public:
	serialCollideClient(void);
	
	/// Call this client function on processor 0:
	void setClient(CollisionClientFn clientFn,void *clientParam);
	
	/// Called by voxel array on each processor:
	virtual void collisions(ArrayElement *src,
		int step,CollisionList &colls);
	
	/// Called after the reduction is complete:
	virtual void reductionDone(CkReductionMsg *m);
};


#endif //def(thisHeader)
