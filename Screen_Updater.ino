 void Update_Screen(){
  // put your main code here, to run repeatedly:
  int Ctype_POS=90;
  u8g2.firstPage();
  u8g2.setFont(u8g2_font_open_iconic_all_1x_t);
      u8g2.setDrawColor(0);
    u8g2.drawBox(Ctype_POS, 1, 12, 8);
        u8g2.drawBox(0, 0, 128, 64);
    u8g2.setDrawColor(1);
    if (Time_Method==0){
      u8g2.drawGlyph(Ctype_POS, 8, 165); // Brightness
    }
    else if (Time_Method==1){
      u8g2.drawGlyph(Ctype_POS, 8, 121); // No Schedule
    }
    else if (Time_Method==2){
      u8g2.drawGlyph(Ctype_POS, 8, 269); // Clock Schedule
    }
    else if (Time_Method==3){
      u8g2.drawGlyph(Ctype_POS, 8, 175); // SunSet
    }
  // Open Logic


  // Door Status
  int Status_POS = 77;
  u8g2.setFont(u8g2_font_open_iconic_all_1x_t);
      u8g2.setDrawColor(0);
    u8g2.drawBox(Status_POS, 1, 12, 8);
    u8g2.setDrawColor(1);
  if (is_top && is_bot) {//Unplugged
    u8g2.drawGlyph(Status_POS, 8, 282);
  }
  else if (!is_bot && !is_top) {//In between
    u8g2.drawGlyph(Status_POS, 8, 240);
  }
  else if (is_top) {//Top Unlocked
    u8g2.drawGlyph(Status_POS, 8, 203);
  }
  else if (is_bot) {//Bottom Locked
    u8g2.drawGlyph(Status_POS, 8, 202);
  }


    // put your setup code here, to run once:
    int BAT_P = 105;
    int BAT_LEV_C;
    u8g2.drawFrame(BAT_P, 0, 12, 7);
    u8g2.drawBox(BAT_P + 12, 2, 2, 3);
    BAT_LEV_C=7.142857*BAT_LEV-20;

    u8g2.drawBox(BAT_P + 1, 1, max(min(BAT_LEV_C+1,10),0), 5);



    u8g2.setFont(u8g2_font_trixel_square_tn);
u8g2.setFont(u8g2_font_timR08_tn);
  
  tt = WiFi.localIP().toString();

    u8g2.drawStr(12, 8,tt.c_str ());
    u8g2.setFont(u8g2_font_open_iconic_all_1x_t);
    if (In_Level>BAT_LEV) {
    u8g2.drawGlyph(121, 8, 96);}
    u8g2.drawGlyph(1, 8, 248);
    u8g2.nextPage();

  




 }
