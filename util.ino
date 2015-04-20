void printi(int val, int width) {
  int absval = abs(val);
  int netwidth = (val < 0) ? 1 : 0;
  if (absval > 999) netwidth += 4;
      else if (absval > 99) netwidth += 3;
      else if (absval > 9) netwidth += 2;
      else netwidth += 1;
  for (int i = width - netwidth; i >= 0; i--) {
      Serial.print(' ');
  }
  Serial.print(val);
}


