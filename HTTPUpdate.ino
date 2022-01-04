



void UpdateProgressBar(int cur, int total) {
    u8g2.drawFrame(10, 45, 102, 12);
    u8g2.drawBox(11, 46, cur/total*100, 10);

}


  
void FirmwareUpdate()
{  
  WiFiClientSecure client;
  //client.setTrustAnchors(&cert);
  client.setInsecure();
  if (!client.connect(host, httpsPort)) {
    Serial.println("Connection failed");
    return;
  }
  Serial.print(String("GET ") + URL_fw_Version + " HTTP/1.1\r\n" +
               "Host: " + host + "\r\n" +
               "User-Agent: BuildFailureDetectorESP8266\r\n" +
               "Connection: close\r\n\r\n");
  client.print(String("GET ") + URL_fw_Version + " HTTP/1.1\r\n" +
               "Host: " + host + "\r\n" +
               "User-Agent: BuildFailureDetectorESP8266\r\n" +
               "Connection: close\r\n\r\n");
  while (client.connected()) {
    String line = client.readStringUntil('\n');
    Serial.println(line);
    if (line == "\r") {
      //Serial.println("Headers received");
      break;
    }
  }
  String payload = client.readStringUntil('\n');
Serial.println(payload);
  
  Serial.println(payload);
  if(payload.equals(FirmwareVer) )
  {   
     Serial.println("Device already on latest firmware version"); 
  }
  else
  {
    Serial.println("New firmware detected");
    ESPhttpUpdate.setLedPin(LED_BUILTIN, LOW); 
    t_httpUpdate_return ret = ESPhttpUpdate.update(client, URL_fw_Bin);
        
    switch (ret) {
      case HTTP_UPDATE_FAILED:
        Serial.printf("HTTP_UPDATE_FAILD Error (%d): %s\n", ESPhttpUpdate.getLastError(), ESPhttpUpdate.getLastErrorString().c_str());
        break;

      case HTTP_UPDATE_NO_UPDATES:
        Serial.println("HTTP_UPDATE_NO_UPDATES");
        break;

      case HTTP_UPDATE_OK:
        Serial.println("HTTP_UPDATE_OK");
        break;
    } 
  }
 }  //  client.setInsecure();
