#include "firebase_manager.h"

#include <Arduino.h>
#include <Firebase_ESP_Client.h>
#include <addons/RTDBHelper.h>

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include "center_data_manager.h"
#include "project_config.h"
#include "secrets.h"


namespace {


#if CONFIG_FREERTOS_UNICORE
constexpr BaseType_t FIREBASE_TASK_CORE = 0;
#else
constexpr BaseType_t FIREBASE_TASK_CORE = 1;
#endif


constexpr uint32_t FIREBASE_TASK_STACK_BYTES = 12288UL;
constexpr UBaseType_t FIREBASE_TASK_PRIORITY = 1;
constexpr uint32_t FIREBASE_TASK_SERVICE_PERIOD_MS = 20UL;


// Scheduler-friendly pause between Center/Node source uploads.
constexpr uint32_t FIREBASE_SOURCE_GAP_MS = 5UL;


// Firebase connection status LED




FirebaseData firebaseData;
FirebaseAuth firebaseAuth;
FirebaseConfig firebaseConfig;


TaskHandle_t firebaseTaskHandle = nullptr;


unsigned long lastFirebaseUpdateMs = 0;
uint8_t firebaseRetryCount = 0;


// Batch numbers
uint32_t nextBatchNumber = 1;
uint32_t pendingBatchNumber = 0;



String getSourcePath(uint8_t sourceID) {

  if (sourceID == ProjectConfig::CENTER_SOURCE_ID) {

    return "/FYP_IMU_Session/Center_Batch";

  }


  return
      "/FYP_IMU_Session/Node" +
      String(sourceID) +
      "_Batch";

}



void printSourceName(uint8_t sourceID) {

  if (sourceID == ProjectConfig::CENTER_SOURCE_ID) {

    Serial.print("Center");
    return;

  }


  Serial.print("Node ");
  Serial.print(sourceID);

}





bool uploadSingleSourceBatch(
  uint8_t sourceID,
  const IMU_Node_Frame *buffer,
  uint8_t count,
  uint32_t batchNumber
) {


  if (count == 0) {

    return true;

  }



  FirebaseJson compactBatch;
  FirebaseJsonArray samples;



  for (uint8_t i = 0; i < count; i++) {


    FirebaseJsonArray sample;


    sample.add(buffer[i].sequenceNo);
    sample.add(buffer[i].timestamp_ms);


    sample.add(buffer[i].accelX);
    sample.add(buffer[i].accelY);
    sample.add(buffer[i].accelZ);


    sample.add(buffer[i].gyroX);
    sample.add(buffer[i].gyroY);
    sample.add(buffer[i].gyroZ);


    sample.add(buffer[i].quatW);
    sample.add(buffer[i].quatX);
    sample.add(buffer[i].quatY);
    sample.add(buffer[i].quatZ);


    samples.add(sample);

  }



  compactBatch.set("b", batchNumber);
  compactBatch.set("t0", buffer[0].timestamp_ms);
  compactBatch.set("d", samples);



  const String path = getSourcePath(sourceID);



  Serial.print("Uploading compact ");
  printSourceName(sourceID);
  Serial.print(" | Batch: ");
  Serial.print(batchNumber);
  Serial.print(" | Samples: ");
  Serial.print(count);
  Serial.print(" | Free heap: ");
  Serial.println(ESP.getFreeHeap());




  const bool success = Firebase.RTDB.setJSON(
    &firebaseData,
    path.c_str(),
    &compactBatch
  );



  if(success) {

    printSourceName(sourceID);
    Serial.println(" compact batch uploaded.");

  }

  else {

    printSourceName(sourceID);

    Serial.print(
      " upload failed. HTTP code: "
    );

    Serial.println(
      firebaseData.httpCode()
    );


    Serial.print(
      "Firebase error: "
    );

    Serial.println(
      firebaseData.errorReason()
    );

  }



  samples.clear();
  compactBatch.clear();



  return success;

}






bool uploadPendingBatch() {


  if(!hasPendingBatch()) {

    return false;

  }



  if(!Firebase.ready()) {


    Serial.println(
      "Firebase is not ready. Pending data retained."
    );


    return false;

  }




  bool anyFailure = false;




  for(
    uint8_t sourceID = 0;
    sourceID < ProjectConfig::SOURCE_COUNT;
    sourceID++
  ) {



    const PendingSourceView source =
        getPendingSource(sourceID);




    if(source.count == 0) {

      continue;

    }




    if(
      uploadSingleSourceBatch(
        sourceID,
        source.frames,
        source.count,
        pendingBatchNumber
      )
    ) {

      markPendingSourceUploaded(sourceID);

    }

    else {

      anyFailure = true;

    }



    vTaskDelay(
      pdMS_TO_TICKS(FIREBASE_SOURCE_GAP_MS)
    );


  }




  if(!hasPendingBatch()) {


    firebaseRetryCount = 0;


    Serial.println(
      "All pending source batches uploaded."
    );


    return true;

  }




  if(!anyFailure) {

    return false;

  }



  firebaseRetryCount++;


  Serial.print(
    "Firebase retry "
  );


  Serial.print(firebaseRetryCount);


  Serial.print("/");


  Serial.println(
    ProjectConfig::MAX_FIREBASE_RETRIES
  );



  if(
    firebaseRetryCount >=
    ProjectConfig::MAX_FIREBASE_RETRIES
  ) {


    Serial.println(
      "Maximum retries reached. Discarding only the remaining failed batches."
    );


    clearPendingBatch();

    firebaseRetryCount = 0;


  }

  else {


    Serial.println(
      "Failed source batches retained for the next upload cycle."
    );


  }



  return false;

}







void firebaseUploadTask(void *parameter) {


  (void)parameter;



  Serial.print(
    "Firebase upload task running on core "
  );


  Serial.println(
    xPortGetCoreID()
  );



  while(true) {


    handleFirebaseUploads();



    vTaskDelay(
      pdMS_TO_TICKS(
        FIREBASE_TASK_SERVICE_PERIOD_MS
      )
    );


  }


}



} // namespace






void initFirebaseManager() {


  // Firebase status LED
  pinMode(
    ProjectConfig::FIREBASE_STATUS_LED,
    OUTPUT
  );


  digitalWrite(
    ProjectConfig::FIREBASE_STATUS_LED,
    HIGH
  );



  firebaseConfig.database_url = DATABASE_URL;



  firebaseConfig.signer.tokens.legacy_token =
      DATABASE_SECRET;




  firebaseData.setBSSLBufferSize(
    ProjectConfig::FIREBASE_SSL_RX_BUFFER_SIZE,
    ProjectConfig::FIREBASE_SSL_TX_BUFFER_SIZE
  );



  firebaseData.setResponseSize(
    ProjectConfig::FIREBASE_RESPONSE_SIZE
  );




  Firebase.begin(
    &firebaseConfig,
    &firebaseAuth
  );



  Firebase.reconnectWiFi(true);



  Serial.println(
    "Firebase manager initialized."
  );

}







void handleFirebaseUploads() {


  // Firebase connection LED status
  if (Firebase.ready()) {
    digitalWrite(
      ProjectConfig::FIREBASE_STATUS_LED,
      HIGH
    );
  } else {
    digitalWrite(
      ProjectConfig::FIREBASE_STATUS_LED,
      LOW
    );
  }





  const unsigned long now = millis();



  if(
    now - lastFirebaseUpdateMs <
    ProjectConfig::FIREBASE_UPDATE_INTERVAL_MS
  ) {

    return;

  }




  lastFirebaseUpdateMs = now;





  if(!hasPendingBatch()) {


    if(!createPendingBatch()) {

      return;

    }



    pendingBatchNumber = nextBatchNumber++;




    if(nextBatchNumber == 0) {

      nextBatchNumber = 1;

    }



    firebaseRetryCount = 0;


  }




  uploadPendingBatch();


}







bool startFirebaseUploadTask() {


  if(firebaseTaskHandle != nullptr) {

    return true;

  }




  const BaseType_t result =
    xTaskCreatePinnedToCore(
      firebaseUploadTask,
      "FirebaseUpload",
      FIREBASE_TASK_STACK_BYTES,
      nullptr,
      FIREBASE_TASK_PRIORITY,
      &firebaseTaskHandle,
      FIREBASE_TASK_CORE
    );




  if(result != pdPASS) {


    firebaseTaskHandle = nullptr;



    Serial.println(
      "ERROR: Failed to create FirebaseUpload task."
    );



    return false;


  }



  return true;

}