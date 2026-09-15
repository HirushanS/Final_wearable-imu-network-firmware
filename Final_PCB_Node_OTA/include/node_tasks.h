#ifndef NODE_TASKS_H
#define NODE_TASKS_H

// Creates the BNO085 acquisition, ESP-NOW transmit and housekeeping tasks.
// Returns true when all tasks and the latest-frame queue were created.
bool startNodeFreeRTOSTasks();

#endif
