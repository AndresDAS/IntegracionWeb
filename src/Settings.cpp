#include "Settings.h"

Settings::Settings()
{
}

void Settings::load()
{
  ssidLen = EEPROM.read(ssidLenAdr);
  passwordLen = EEPROM.read(passwordLenAdr);

  ssid = "";
  password = "";
  for (int i = 0; i < ssidLen; i++)
    ssid += (char)EEPROM.read(ssidAdr + i);
  for (int i = 0; i < passwordLen; i++)
    password += (char)EEPROM.read(passwordAdr + i);
}

void Settings::save()
{
  ssidLen = ssid.length();
  passwordLen = password.length();

  EEPROM.write(ssidLenAdr, ssidLen);
  EEPROM.write(passwordLenAdr, passwordLen);
  for (int i = 0; i < ssidLen; i++)
    EEPROM.write(ssidAdr + i, ssid[i]);
  for (int i = 0; i < passwordLen; i++)
    EEPROM.write(passwordAdr + i, password[i]);

  EEPROM.commit();

  info();
  Serial.println("settings saved");

}

void Settings::info()
{
  Serial.println("settings:");
  Serial.println("SSID: " + ssid);
  Serial.println("SSID length: " + (String)ssidLen);
  Serial.println("password: " + password);
  Serial.println("password length: " + (String)passwordLen);
}

void Settings::reset()
{
  Serial.print("reset settings...");

  ssid = "dummy";
  password = "dummy";
  ssidHidden = false;
  apChannel = 1;

  ssidLen = ssid.length();
  passwordLen = password.length();

  Serial.println("done");

  save();
}