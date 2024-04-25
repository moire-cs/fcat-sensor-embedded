#if defined(SELF_ADDRESS) && defined(TARGET_ADDRESS)
const uint8_t selfAddress_ = SELF_ADDRESS;
const uint8_t targetAddress_ = TARGET_ADDRESS;
#else
// Topology
// define the node address
#define NODE_ADDRESS_1 0 // 
#define NODE_ADDRESS_2 1 //
#define NODE_ADDRESS_3 2 //
#define ENDNODE_ADDRESS 254 // purposefully using the last number (0-254, 255 is broadcast)
// TODO: according to this, we might have a max of 256 nodes in one mesh
// selfAddress is node
// targetAddress will be our gateway
const uint8_t selfAddress_ = ENDNODE_ADDRESS;      // CHANGE THIS!!!
const uint8_t targetAddress_ = ENDNODE_ADDRESS; // integer value
#endif