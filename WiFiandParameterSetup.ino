void WiFiManegerSetup(){


  
  WiFiManagerParameter custom_html(); // only custom html
  WiFiManagerParameter custom_mqtt_server("server", "mqtt server", "", 40);
  WiFiManagerParameter custom_token("api_token", "api token", "", 16);
  

  const char _customHtml_checkbox[] = "type=\"checkbox\""; 
  WiFiManagerParameter custom_checkbox("checkbox", "my checkbox", "T", 2, _customHtml_checkbox, WFM_LABEL_AFTER);



}

// Save parameters from the costom parameters page
void SaveCustomParams (){
  


    EEPROM.put( 0 + eepS, LATT);
    EEPROM.put( 4 + eepS, LONG);
    EEPROM.put( 8 + eepS, Time_ZONE);
    EEPROM.put( 12 + eepS, After_Sunrise);
    EEPROM.put( 16 + eepS, After_Sunset);
    EEPROM.put( 20 + eepS, up_limit);
    EEPROM.put( 24 + eepS, down_limit);
    EEPROM.put( 28 + eepS, Feed_time);
    EEPROM.put( 32 + eepS, Feed_length);
    EEPROM.put( 36 + eepS, APIkey);
    EEPROM.put( 53 + eepS, StateField);
    EEPROM.put( 54 + eepS, TempField);
    EEPROM.put( 55 + eepS, BatField);
    EEPROM.put( 56 + eepS, InField);
    EEPROM.put( 57 + eepS, DSE);
}
