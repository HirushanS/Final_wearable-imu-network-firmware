#ifndef FIREBASE_MANAGER_H
#define FIREBASE_MANAGER_H

// Configures the Firebase client. Call once during setup().
void initFirebaseManager();

// Services batch creation and upload timing.
// Normally called only by the dedicated FirebaseUpload task.
void handleFirebaseUploads();

// Creates the low-priority Firebase upload task.
// Returns true when the task is running.
bool startFirebaseUploadTask();

#endif
